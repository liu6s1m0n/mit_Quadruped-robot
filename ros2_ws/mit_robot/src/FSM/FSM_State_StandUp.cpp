#include "FSM/FSM_State_StandUp.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

template<typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T> * control_fsm_data)
: FSM_State<T>(control_fsm_data, FSM_StateName::STAND_UP, "STAND_UP")
{
  if (control_fsm_data == nullptr || !control_fsm_data->valid()) {
    throw std::invalid_argument("stand-up state requires valid FSM data");
  }
  this->turnOffAllSafetyChecks();
  // GO1 输出足端轨迹；DM1 输出关节轨迹，因此只对 GO1 检查 pDesFoot。
  this->checkPDesFoot =
    !control_fsm_data->control_parameters->start_in_prone_home;
  const T stand_up_duration =
    control_fsm_data->quadruped->robotType() == RobotType::UNITREE_GO1 ?
    T(0.5) : control_fsm_data->control_parameters->stand_up_duration;
  ramp_iterations_ = std::max<std::size_t>(
    1, static_cast<std::size_t>(
      std::ceil(stand_up_duration / control_fsm_data->control_time_step)));
}

template<typename T>
void FSM_State_StandUp<T>::onEnter()
{
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  iteration_ = 0;
  stand_up_complete_ = false;

  auto & controller = *this->_data->leg_controller;
  controller.zeroCommand();
  controller.setEnabled(true);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & feedback = controller.datas[leg];
    if (feedback.valid && feedback.p.allFinite()) {
      initial_foot_positions_[leg] = feedback.p;
    } else {
      const LegId leg_id = static_cast<LegId>(leg);
      computeLegJacobianAndPosition(
        *this->_data->quadruped,
        this->_data->control_parameters->motor_zero_position,
        static_cast<Mat3<T> *>(nullptr), &initial_foot_positions_[leg], leg_id);
    }
    initial_joint_positions_[leg] = feedback.valid && feedback.q.allFinite() ?
      feedback.q : this->_data->control_parameters->motor_zero_position;
  }
  this->_data->gait_scheduler->requestGait(GaitType::STAND);
}

template<typename T>
void FSM_State_StandUp<T>::run()
{
  const T linear_progress = std::clamp(
    static_cast<T>(iteration_) / static_cast<T>(ramp_iterations_), T(0), T(1));
  const T progress = linear_progress * linear_progress *
    (T(3) - T(2) * linear_progress);
  const T target_height = -this->_data->quadruped->nominalBodyHeight();

  // DM1 的发卡形趴卧零位不能直接承担机身重量。先在机身仍贴地时把腿
  // 展开到质心附近的稳定支撑区；轨迹结束后再交给 BalanceStand/WBC
  // 抬升机身并约束 roll/pitch。
  if (this->_data->control_parameters->start_in_prone_home) {
    const T duration = static_cast<T>(ramp_iterations_) *
      this->_data->control_time_step;
    const T progress_rate = duration > T(0) ?
      T(6) * linear_progress * (T(1) - linear_progress) / duration : T(0);
    auto & controller = *this->_data->leg_controller;
    controller.zeroCommand();
    controller.setEnabled(true);
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      auto & command = controller.commands[leg];
      const LegId leg_id = static_cast<LegId>(leg);
      const auto & joints = this->_data->quadruped->leg(leg_id).joints;
      const Vec3<T> target =
        (this->_data->control_parameters->stand_up_prepare_position -
        joints.zero_offset)
        .cwiseMax(joints.lower_limit).cwiseMin(joints.upper_limit);
      const Vec3<T> travel = target - initial_joint_positions_[leg];
      command.position_desired = initial_joint_positions_[leg] + progress * travel;
      command.velocity_desired = progress_rate * travel;
      command.kp_joint = this->_data->control_parameters->stand_up_joint_kp;
      command.kd_joint = this->_data->control_parameters->stand_up_joint_kd;
    }
    if (iteration_ >= ramp_iterations_) {
      // 收腿时足端与地面的摩擦会让趴卧机身产生少量水平位移。WBC 应从
      // 收腿结束后的实际位置原地抬升，不能追赶点击按钮前保存的旧坐标。
      auto & desired = *this->_data->desired_state;
      const auto & estimate = *this->_data->state_estimate;
      desired.body_position_world.x() = estimate.position_world.x();
      desired.body_position_world.y() = estimate.position_world.y();
      desired.body_rpy << T(0), T(0), estimate.rpy.z();
      desired.body_velocity_world.setZero();
      desired.body_acceleration_world.setZero();
      desired.body_angular_velocity.setZero();
      stand_up_complete_ = true;
      desired.mode = ControlMode::BalanceStand;
    }
    ++iteration_;
    return;
  }

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    auto & command = this->_data->leg_controller->commands[leg];
    command.zero();
    command.foot_position_desired = initial_foot_positions_[leg];
    command.foot_position_desired.z() =
      (T(1) - progress) * initial_foot_positions_[leg].z() + progress * target_height;
    if (this->_data->quadruped->robotType() == RobotType::UNITREE_GO1) {
      command.kp_cartesian.diagonal().setConstant(T(500));
      command.kd_cartesian.diagonal().setConstant(T(8));
    } else {
      command.kp_cartesian.diagonal() =
        this->_data->control_parameters->stand_up_cartesian_kp;
      command.kd_cartesian.diagonal() =
        this->_data->control_parameters->stand_up_cartesian_kd;
    }
  }
  this->_data->leg_controller->setEnabled(true);
  if (iteration_ >= ramp_iterations_) {
    stand_up_complete_ = true;
    this->_data->desired_state->mode = ControlMode::BalanceStand;
  }
  ++iteration_;
}

template<typename T>
FSM_StateName FSM_State_StandUp<T>::checkTransition()
{
  const ControlMode mode = this->_data->desired_state->mode;
  if (mode == ControlMode::Passive) {
    this->nextStateName = FSM_StateName::PASSIVE;
  } else if (!stand_up_complete_ || mode == ControlMode::StandUp) {
    this->nextStateName = this->stateName;
  } else {
    switch (mode) {
      case ControlMode::Passive:
      case ControlMode::StandUp:
        break;
      case ControlMode::JointPd:
        this->nextStateName = FSM_StateName::JOINT_PD;
        break;
      case ControlMode::BalanceStand:
        this->nextStateName = FSM_StateName::BALANCE_STAND;
        break;
      case ControlMode::Locomotion:
        this->nextStateName = FSM_StateName::LOCOMOTION;
        break;
      case ControlMode::RecoveryStand:
        this->nextStateName = FSM_StateName::RECOVERY_STAND;
        break;
      case ControlMode::FrontJump:
        // 起立动作未完成时不允许直接起跳。
        this->_data->desired_state->mode = ControlMode::BalanceStand;
        this->nextStateName = FSM_StateName::BALANCE_STAND;
        break;
    }
  }
  this->transitionDuration = T(0);
  return this->nextStateName;
}

template<typename T>
TransitionData<T> FSM_State_StandUp<T>::transition()
{
  if (this->nextStateName == FSM_StateName::PASSIVE) {
    this->_data->leg_controller->zeroCommand();
    this->_data->leg_controller->setEnabled(false);
  } else {
    run();
  }
  this->transitionData.done = this->nextStateName != FSM_StateName::STAND_UP;
  return this->transitionData;
}

template<typename T>
void FSM_State_StandUp<T>::onExit()
{
  iteration_ = 0;
  stand_up_complete_ = false;
}

template class FSM_State_StandUp<float>;
