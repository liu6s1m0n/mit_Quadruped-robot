#include "FSM/FSM_State_Passive.h"

#include <stdexcept>

template<typename T>
FSM_State_Passive<T>::FSM_State_Passive(ControlFSMData<T> * control_fsm_data)
: FSM_State<T>(control_fsm_data, FSM_StateName::PASSIVE, "PASSIVE")
{
  if (control_fsm_data == nullptr || !control_fsm_data->valid()) {
    throw std::invalid_argument("passive state requires valid FSM data");
  }
  this->turnOffAllSafetyChecks();
}

template<typename T>
void FSM_State_Passive<T>::onEnter()
{
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  run();
}

template<typename T>
void FSM_State_Passive<T>::run()
{
  this->_data->leg_controller->zeroCommand();
  this->_data->leg_controller->setEnabled(false);
}

template<typename T>
FSM_StateName FSM_State_Passive<T>::checkTransition()
{
  switch (this->_data->desired_state->mode) {
    case ControlMode::Passive:
      this->nextStateName = FSM_StateName::PASSIVE;
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
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      break;
  }
  this->transitionDuration = T(0);
  return this->nextStateName;
}

template<typename T>
TransitionData<T> FSM_State_Passive<T>::transition()
{
  run();
  this->transitionData.done = true;
  return this->transitionData;
}

template<typename T>
void FSM_State_Passive<T>::onExit()
{
}

template class FSM_State_Passive<float>;
