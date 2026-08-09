#include "FSM/FSM_State_BalanceStand.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "model/floating_base_model_factory.hpp"

template<typename T>
FSM_State_BalanceStand<T>::FSM_State_BalanceStand(
  ControlFSMData<T> * control_fsm_data)
: FSM_State<T>(
    control_fsm_data, FSM_StateName::BALANCE_STAND, "BALANCE_STAND")
{
  if (control_fsm_data == nullptr || !control_fsm_data->valid()) {
    throw std::invalid_argument("balance-stand state requires valid FSM data");
  }
  this->turnOnAllSafetyChecks();
  this->checkPDesFoot = false;
  wbc_ctrl_ = std::make_unique<LocomotionCtrl<T>>(
    model::makeFloatingBaseModel(*control_fsm_data->quadruped));
  wbc_ctrl_->setFloatingBaseWeight(T(1000));
}

template<typename T>
void FSM_State_BalanceStand<T>::onEnter()
{
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  this->_data->gait_scheduler->requestGait(GaitType::STAND);

  initial_body_position_ = this->_data->state_estimate->position_world;
  if (initial_body_position_.z() < T(0.2)) {initial_body_position_.z() = T(0.3);}
  last_height_command_ = initial_body_position_.z();
  initial_body_rpy_ = this->_data->state_estimate->rpy;
  body_weight_ = this->_data->quadruped->bodyInertia().mass * T(9.81);
}

template<typename T>
void FSM_State_BalanceStand<T>::run()
{
  BalanceStandStep();
}

template<typename T>
FSM_StateName FSM_State_BalanceStand<T>::checkTransition()
{
  ++iteration_;
  switch (this->_data->desired_state->mode) {
    case ControlMode::BalanceStand:
      break;
    case ControlMode::Locomotion:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      this->transitionDuration = T(0);
      this->_data->gait_scheduler->requestGait(GaitType::TROT);
      break;
    case ControlMode::Passive:
      this->nextStateName = FSM_StateName::PASSIVE;
      this->transitionDuration = T(0);
      break;
    case ControlMode::JointPd:
      this->nextStateName = FSM_StateName::JOINT_PD;
      this->transitionDuration = T(0);
      break;
  }
  return this->nextStateName;
}

template<typename T>
TransitionData<T> FSM_State_BalanceStand<T>::transition()
{
  if (this->nextStateName == FSM_StateName::LOCOMOTION) {BalanceStandStep();}
  if (this->nextStateName == FSM_StateName::PASSIVE) {
    this->turnOffAllSafetyChecks();
  }
  this->transitionData.done = true;
  return this->transitionData;
}

template<typename T>
void FSM_State_BalanceStand<T>::onExit()
{
  iteration_ = 0;
}

template<typename T>
void FSM_State_BalanceStand<T>::BalanceStandStep()
{
  wbc_data_.pBody_des = initial_body_position_;
  wbc_data_.vBody_des.setZero();
  wbc_data_.aBody_des.setZero();
  wbc_data_.pBody_RPY_des = initial_body_rpy_;
  wbc_data_.vBody_Ori_des.setZero();

  const DesiredState<T> & desired = *this->_data->desired_state;
  if (desired.valid) {
    wbc_data_.pBody_des = desired.body_position_world;
    wbc_data_.pBody_RPY_des = desired.body_rpy;
    wbc_data_.vBody_des = desired.body_velocity_world;
    wbc_data_.aBody_des = desired.body_acceleration_world;
    wbc_data_.vBody_Ori_des = desired.body_angular_velocity;
  }
  if (last_height_command_ - wbc_data_.pBody_des.z() > T(0.001)) {
    wbc_data_.pBody_des.z() = last_height_command_ - T(0.001);
  }
  last_height_command_ = wbc_data_.pBody_des.z();

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    wbc_data_.pFoot_des[leg].setZero();
    wbc_data_.vFoot_des[leg].setZero();
    wbc_data_.aFoot_des[leg].setZero();
    wbc_data_.Fr_des[leg] = Vec3<T>(T(0), T(0), body_weight_ / T(4));
    wbc_data_.contact_state[leg] = T(1);
  }

  const bool wbc_valid = wbc_ctrl_->runAndApply(
    &wbc_data_, *this->_data->state_estimate, *this->_data->joint_states,
    *this->_data->leg_controller);
  if (!wbc_valid) {return;}

  // The body/contact tasks do not uniquely determine the twelve joint angles:
  // with all four feet constrained, KinWBC still has a posture null space.  A
  // joint impedance target prevents that null space from drifting to the other
  // inverse-kinematics branch while WBIC continues to supply the whole-body
  // feed-forward torque and reaction-force solution.
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const LegId leg_id = static_cast<LegId>(leg);
    auto & command = this->_data->leg_controller->commands[leg];
    const auto & leg_model = this->_data->quadruped->leg(leg_id);
    const Vec3<T> home = leg_model.joints.home_position;
    const T nominal_height = this->_data->quadruped->nominalBodyHeight();
    const T height_ratio = wbc_data_.pBody_des.z() / nominal_height;
    const T cosine = std::clamp(
      std::cos(home.y()) * height_ratio, T(0), T(1));
    const T thigh_angle = std::acos(cosine);
    command.position_desired << home.x(), thigh_angle, -T(2) * thigh_angle;
    command.position_desired = command.position_desired.cwiseMax(
      leg_model.joints.lower_limit).cwiseMin(leg_model.joints.upper_limit);
    command.velocity_desired.setZero();
    command.kp_joint.setConstant(T(20));
    command.kd_joint.setConstant(T(2));
  }
}

template class FSM_State_BalanceStand<float>;
