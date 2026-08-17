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
  this->checkPDesFoot = true;
  ramp_iterations_ = std::max<std::size_t>(
    1, static_cast<std::size_t>(
      std::ceil(T(0.5) / control_fsm_data->control_time_step)));
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
      continue;
    }
    const LegId leg_id = static_cast<LegId>(leg);
    computeLegJacobianAndPosition(
      *this->_data->quadruped,
      this->_data->quadruped->leg(leg_id).joints.home_position,
      static_cast<Mat3<T> *>(nullptr), &initial_foot_positions_[leg], leg_id);
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

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    auto & command = this->_data->leg_controller->commands[leg];
    command.zero();
    command.foot_position_desired = initial_foot_positions_[leg];
    command.foot_position_desired.z() =
      (T(1) - progress) * initial_foot_positions_[leg].z() + progress * target_height;
    command.kp_cartesian.diagonal().setConstant(T(500));
    command.kd_cartesian.diagonal().setConstant(T(8));
  }
  this->_data->leg_controller->setEnabled(true);
  if (iteration_ >= ramp_iterations_) {stand_up_complete_ = true;}
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
