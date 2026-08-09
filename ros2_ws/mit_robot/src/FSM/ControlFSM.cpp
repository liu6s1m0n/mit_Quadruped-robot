#include "FSM/ControlFSM.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{

FSM_StateName stateForMode(ControlMode mode) noexcept
{
  switch (mode) {
    case ControlMode::Passive: return FSM_StateName::PASSIVE;
    case ControlMode::JointPd: return FSM_StateName::JOINT_PD;
    case ControlMode::BalanceStand: return FSM_StateName::BALANCE_STAND;
    case ControlMode::Locomotion: return FSM_StateName::LOCOMOTION;
  }
  return FSM_StateName::INVALID;
}

template<typename T>
class BasicState final : public FSM_State<T>
{
public:
  BasicState(ControlFSMData<T> * data, FSM_StateName name, const char * label)
  : FSM_State<T>(data, name, label) {}

  void onEnter() override
  {
    this->nextStateName = this->stateName;
    this->transitionData.zero();
  }

  void run() override
  {
    auto & controller = *this->_data->leg_controller;
    controller.zeroCommand();
    if (this->stateName == FSM_StateName::PASSIVE) {
      controller.setEnabled(false);
      return;
    }

    controller.setEnabled(true);
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const LegId leg_id = static_cast<LegId>(leg);
      controller.commands[leg].position_desired =
        this->_data->quadruped->leg(leg_id).joints.home_position;
      controller.commands[leg].kp_joint.setConstant(T(20));
      controller.commands[leg].kd_joint.setConstant(T(2));
    }
  }

  FSM_StateName checkTransition() override
  {
    this->nextStateName = stateForMode(this->_data->desired_state->mode);
    this->transitionDuration = T(0);
    return this->nextStateName;
  }

  TransitionData<T> transition() override
  {
    run();
    this->transitionData.done = true;
    return this->transitionData;
  }

  void onExit() override {}
};

}  // namespace

template<typename T>
ControlFSM<T>::ControlFSM(
  const Quadruped<T> & quadruped, StateEstimate<T> & state_estimate,
  const std::array<JointState<T>, kNumLegs> & joint_states,
  LegController<T> & leg_controller, GaitScheduler<T> & gait_scheduler,
  DesiredState<T> & desired_state, T control_time_step)
{
  data.quadruped = &quadruped;
  data.state_estimate = &state_estimate;
  data.joint_states = &joint_states;
  data.leg_controller = &leg_controller;
  data.gait_scheduler = &gait_scheduler;
  data.desired_state = &desired_state;
  data.control_time_step = control_time_step;
  if (!data.valid()) {throw std::invalid_argument("invalid ControlFSM dependencies");}

  statesList.passive = std::make_unique<BasicState<T>>(
    &data, FSM_StateName::PASSIVE, "PASSIVE");
  statesList.joint_pd = std::make_unique<BasicState<T>>(
    &data, FSM_StateName::JOINT_PD, "JOINT_PD");
  statesList.balance_stand = std::make_unique<FSM_State_BalanceStand<T>>(&data);
  statesList.locomotion = std::make_unique<FSM_State_Locomotion<T>>(&data);
  initialize();
}

template<typename T>
void ControlFSM<T>::initialize()
{
  currentState = statesList.passive.get();
  currentState->onEnter();
  nextState = currentState;
  nextStateName = currentState->stateName;
  operating_mode_ = FSM_OperatingMode::NORMAL;
}

template<typename T>
void ControlFSM<T>::runFSM()
{
  operating_mode_ = safetyPreCheck();
  if (operating_mode_ == FSM_OperatingMode::ESTOP) {
    if (currentState != statesList.passive.get()) {currentState->onExit();}
    currentState = statesList.passive.get();
    currentState->onEnter();
    nextState = currentState;
    nextStateName = currentState->stateName;
    currentState->run();
    printInfo(0);
    ++iteration_;
    return;
  }

  if (operating_mode_ == FSM_OperatingMode::NORMAL) {
    nextStateName = currentState->checkTransition();
    if (nextStateName != currentState->stateName) {
      nextState = getNextState(nextStateName);
      if (nextState == nullptr) {
        operating_mode_ = FSM_OperatingMode::ESTOP;
      } else {
        operating_mode_ = FSM_OperatingMode::TRANSITIONING;
      }
    } else {
      currentState->run();
    }
  }

  if (operating_mode_ == FSM_OperatingMode::TRANSITIONING) {
    transitionData = currentState->transition();
    safetyPostCheck();
    if (transitionData.done) {
      currentState->onExit();
      currentState = nextState;
      currentState->onEnter();
      operating_mode_ = FSM_OperatingMode::NORMAL;
    }
  } else {
    safetyPostCheck();
  }
  printInfo(0);
  ++iteration_;
}

template<typename T>
FSM_OperatingMode ControlFSM<T>::safetyPreCheck()
{
  if (!data.state_estimate->valid || !data.state_estimate->rpy.allFinite()) {
    return FSM_OperatingMode::ESTOP;
  }
  if (currentState->checkSafeOrientation &&
    (std::abs(data.state_estimate->rpy.x()) > T(1.4) ||
    std::abs(data.state_estimate->rpy.y()) > T(1.4)))
  {
    return FSM_OperatingMode::ESTOP;
  }
  return operating_mode_;
}

template<typename T>
FSM_OperatingMode ControlFSM<T>::safetyPostCheck()
{
  for (const auto & command : data.leg_controller->commands) {
    if (!command.position_desired.allFinite() ||
      !command.velocity_desired.allFinite() ||
      !command.torque_feedforward.allFinite() ||
      !command.force_feedforward.allFinite())
    {
      data.leg_controller->zeroCommand();
      data.leg_controller->setEnabled(false);
      operating_mode_ = FSM_OperatingMode::ESTOP;
      break;
    }
  }
  return operating_mode_;
}

template<typename T>
FSM_State<T> * ControlFSM<T>::getNextState(FSM_StateName state_name) noexcept
{
  switch (state_name) {
    case FSM_StateName::PASSIVE: return statesList.passive.get();
    case FSM_StateName::JOINT_PD: return statesList.joint_pd.get();
    case FSM_StateName::BALANCE_STAND: return statesList.balance_stand.get();
    case FSM_StateName::LOCOMOTION: return statesList.locomotion.get();
    case FSM_StateName::INVALID: return nullptr;
  }
  return nullptr;
}

template<typename T>
FSM_StateName ControlFSM<T>::currentStateName() const noexcept
{
  return currentState == nullptr ? FSM_StateName::INVALID : currentState->stateName;
}

template<typename T>
void ControlFSM<T>::printInfo(int option)
{
  if (option == 0 && ++print_iteration_ < print_num_) {return;}
  print_iteration_ = 0;
  std::cout << "[CONTROL FSM] iteration " << iteration_ << ", state "
            << (currentState == nullptr ? "INVALID" : currentState->stateString)
            << ", gait " << data.gait_scheduler->gait_data.gait_name << '\n';
}

template class ControlFSM<float>;
