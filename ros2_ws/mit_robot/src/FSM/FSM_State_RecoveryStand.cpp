// 恢复站立状态只生成受模型限位保护的关节 PD 指令，不依赖 MPC 或 WBC。
#include "FSM/FSM_State_RecoveryStand.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
template<typename T>
Vec3<T> clampToJointLimits(
  const Vec3<T> & position, const JointModelParameters<T> & joints)
{
  return position.cwiseMax(joints.lower_limit).cwiseMin(joints.upper_limit);
}
}  // namespace

template<typename T>
FSM_State_RecoveryStand<T>::FSM_State_RecoveryStand(
  ControlFSMData<T> * control_fsm_data)
: FSM_State<T>(
    control_fsm_data, FSM_StateName::RECOVERY_STAND, "RECOVERY_STAND")
{
  if (control_fsm_data == nullptr || !control_fsm_data->valid()) {
    throw std::invalid_argument("recovery-stand state requires valid FSM data");
  }

  // 倒地本身就意味着姿态超出正常站立范围，因此恢复期间关闭姿态前置检查。
  this->turnOffAllSafetyChecks();

  rollover_ramp_iterations_ = iterationsFor(T(0.30));
  rollover_settle_iterations_ = iterationsFor(T(0.30));
  fold_ramp_iterations_ = iterationsFor(T(0.80));
  fold_settle_iterations_ = iterationsFor(T(1.40));
  standup_ramp_iterations_ = iterationsFor(T(0.50));

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const LegId leg_id = static_cast<LegId>(leg);
    const auto & joints = control_fsm_data->quadruped->leg(leg_id).joints;

    // 当前工程的 GO1 关节方向为 [Hip, +thigh, -calf]，与导入代码相反。
    stand_joint_positions_[leg] = joints.home_position;
    fold_joint_positions_[leg] = clampToJointLimits(
      Vec3<T>(T(0), T(1.4), T(-2.7)), joints);

    // 利用同向 Hip 外展和左右腿不对称收腿产生侧滚力矩，所有目标仍受模型限位保护。
    const bool right_side = leg_id == LegId::FR || leg_id == LegId::RR;
    rolling_joint_positions_[leg] = clampToJointLimits(
      Vec3<T>(T(0.8), right_side ? T(1.6) : T(3.1), T(-2.77)), joints);
  }
}

template<typename T>
void FSM_State_RecoveryStand<T>::onEnter()
{
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  state_iteration_ = 0;
  motion_start_iteration_ = 0;
  recovery_complete_ = false;

  auto & controller = *this->_data->leg_controller;
  controller.zeroCommand();
  controller.setEnabled(true);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & feedback = controller.datas[leg];
    initial_joint_positions_[leg] = feedback.valid && feedback.q.allFinite() ?
      feedback.q : stand_joint_positions_[leg];
  }

  this->_data->gait_scheduler->requestGait(GaitType::STAND);
  const T height = this->_data->state_estimate->position_world.z();
  const bool standing_height = std::isfinite(static_cast<double>(height)) &&
    height > T(0.20) && height < T(0.45);
  motion_ = !upsideDown() && standing_height ? Motion::StandUp : Motion::FoldLegs;
}

template<typename T>
void FSM_State_RecoveryStand<T>::run()
{
  const std::size_t motion_iteration = state_iteration_ - motion_start_iteration_;
  switch (motion_) {
    case Motion::StandUp:
      standUp(motion_iteration);
      break;
    case Motion::FoldLegs:
      foldLegs(motion_iteration);
      break;
    case Motion::RollOver:
      rollOver(motion_iteration);
      break;
  }
  ++state_iteration_;
}

template<typename T>
void FSM_State_RecoveryStand<T>::setJointPositionInterpolated(
  std::size_t current_iteration, std::size_t maximum_iteration,
  std::size_t leg, const Vec3<T> & initial, const Vec3<T> & final)
{
  const T linear_progress = std::clamp(
    static_cast<T>(current_iteration) / static_cast<T>(maximum_iteration),
    T(0), T(1));
  // 三次平滑步进使轨迹两端速度为零，降低恢复动作开始和结束时的冲击。
  const T progress = linear_progress * linear_progress *
    (T(3) - T(2) * linear_progress);

  auto & command = this->_data->leg_controller->commands[leg];
  command.position_desired = (T(1) - progress) * initial + progress * final;
  command.velocity_desired.setZero();
  command.torque_feedforward.setZero();
  command.force_feedforward.setZero();
  command.kp_cartesian.setZero();
  command.kd_cartesian.setZero();
  command.kp_joint.setConstant(T(20));
  command.kd_joint.setConstant(T(2));
}

template<typename T>
void FSM_State_RecoveryStand<T>::rollOver(std::size_t motion_iteration)
{
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    setJointPositionInterpolated(
      motion_iteration, rollover_ramp_iterations_, leg,
      initial_joint_positions_[leg], rolling_joint_positions_[leg]);
  }

  if (motion_iteration >= rollover_ramp_iterations_ + rollover_settle_iterations_) {
    startMotion(Motion::FoldLegs, rolling_joint_positions_);
  }
}

template<typename T>
void FSM_State_RecoveryStand<T>::standUp(std::size_t motion_iteration)
{
  const T height = this->_data->state_estimate->position_world.z();
  const bool invalid_height = !std::isfinite(static_cast<double>(height)) ||
    height < T(0.10);
  const bool failed = upsideDown() || invalid_height;
  const std::size_t failure_check_iteration =
    (standup_ramp_iterations_ * std::size_t(7)) / std::size_t(10);

  if (motion_iteration > failure_check_iteration && failed) {
    JointPositions current_positions;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const auto & feedback = this->_data->leg_controller->datas[leg];
      current_positions[leg] = feedback.valid && feedback.q.allFinite() ?
        feedback.q : this->_data->leg_controller->commands[leg].position_desired;
    }
    startMotion(Motion::FoldLegs, current_positions);
    std::cerr << "[CONTROL FSM] Recovery stand failed; folding legs before retry\n";
    return;
  }

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    setJointPositionInterpolated(
      motion_iteration, standup_ramp_iterations_, leg,
      initial_joint_positions_[leg], stand_joint_positions_[leg]);
  }
  if (motion_iteration >= standup_ramp_iterations_) {recovery_complete_ = true;}
}

template<typename T>
void FSM_State_RecoveryStand<T>::foldLegs(std::size_t motion_iteration)
{
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    setJointPositionInterpolated(
      motion_iteration, fold_ramp_iterations_, leg,
      initial_joint_positions_[leg], fold_joint_positions_[leg]);
  }

  if (motion_iteration >= fold_ramp_iterations_ + fold_settle_iterations_) {
    startMotion(
      upsideDown() ? Motion::RollOver : Motion::StandUp,
      fold_joint_positions_);
  }
}

template<typename T>
void FSM_State_RecoveryStand<T>::startMotion(
  Motion motion, const JointPositions & initial_positions)
{
  motion_ = motion;
  initial_joint_positions_ = initial_positions;
  // run() 会在本周期末递增 state_iteration_，下个动作从插值进度 0 开始。
  motion_start_iteration_ = state_iteration_ + 1;
}

template<typename T>
bool FSM_State_RecoveryStand<T>::upsideDown() const noexcept
{
  const auto & estimate = *this->_data->state_estimate;
  return !estimate.valid || !estimate.rotation_world_from_body.allFinite() ||
         estimate.rotation_world_from_body(2, 2) < T(0);
}

template<typename T>
std::size_t FSM_State_RecoveryStand<T>::iterationsFor(T duration_seconds) const noexcept
{
  return std::max<std::size_t>(
    1, static_cast<std::size_t>(
      std::ceil(duration_seconds / this->_data->control_time_step)));
}

template<typename T>
FSM_StateName FSM_State_RecoveryStand<T>::checkTransition()
{
  // 恢复完成前保持本状态；Passive 始终可作为无条件紧急退出通道。
  if (this->_data->desired_state->mode == ControlMode::Passive) {
    this->nextStateName = FSM_StateName::PASSIVE;
    this->transitionDuration = T(0);
    return this->nextStateName;
  }
  if (!recovery_complete_) {
    this->nextStateName = this->stateName;
    return this->nextStateName;
  }

  switch (this->_data->desired_state->mode) {
    case ControlMode::Passive:
      // 上面的快速路径已经处理，此分支仅用于穷举枚举值。
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
    case ControlMode::StandUp:
      this->nextStateName = FSM_StateName::STAND_UP;
      break;
    case ControlMode::RecoveryStand:
      this->nextStateName = this->stateName;
      break;
    case ControlMode::FrontJump:
      // 恢复动作优先，拒绝在姿态未恢复时起跳。
      this->_data->desired_state->mode = ControlMode::RecoveryStand;
      this->nextStateName = this->stateName;
      break;
  }
  this->transitionDuration = T(0);
  return this->nextStateName;
}

template<typename T>
TransitionData<T> FSM_State_RecoveryStand<T>::transition()
{
  if (this->nextStateName == FSM_StateName::PASSIVE) {
    this->_data->leg_controller->zeroCommand();
    this->_data->leg_controller->setEnabled(false);
  } else {
    run();
  }
  this->transitionData.done = this->nextStateName != FSM_StateName::INVALID &&
    this->nextStateName != FSM_StateName::RECOVERY_STAND;
  return this->transitionData;
}

template<typename T>
void FSM_State_RecoveryStand<T>::onExit()
{
  state_iteration_ = 0;
  motion_start_iteration_ = 0;
  recovery_complete_ = false;
}

template class FSM_State_RecoveryStand<float>;
