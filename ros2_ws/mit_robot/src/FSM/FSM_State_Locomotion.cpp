// 行走状态的数据流：状态估计 -> MPC 接触力规划 -> WBC 关节力矩。
#include "FSM/FSM_State_Locomotion.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "model/floating_base_model_factory.hpp"
#include "orientation_tools.h"

template<typename T>
FSM_State_Locomotion<T>::FSM_State_Locomotion(
  ControlFSMData<T> * control_fsm_data)
: FSM_State<T>(control_fsm_data, FSM_StateName::LOCOMOTION, "LOCOMOTION")
{
  if (control_fsm_data == nullptr || !control_fsm_data->valid()) {
    throw std::invalid_argument("locomotion state requires valid FSM data");
  }
  // MPC 不必在 500 Hz 物理频率下每帧求解，这里约每 27 ms 更新一次。
  const std::size_t mpc_interval = static_cast<std::size_t>(std::max(
      T(1), T(0.027) / control_fsm_data->control_time_step));
  mpc_ = std::make_unique<mpc::ConvexMPCLocomotion<T>>(
    *control_fsm_data->quadruped, control_fsm_data->control_time_step,
    mpc_interval);
  wbc_ctrl_ = std::make_unique<LocomotionCtrl<T>>(
    model::makeFloatingBaseModel(*control_fsm_data->quadruped));
  this->turnOnAllSafetyChecks();
  this->checkPDesFoot = false;
}

template<typename T>
void FSM_State_Locomotion<T>::onEnter()
{
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  mpc_->initialize();
  mpc_->setGait(GaitType::TROT);
  this->_data->gait_scheduler->requestGait(GaitType::TROT);
}

template<typename T>
void FSM_State_Locomotion<T>::run()
{
  LocomotionControlStep();
}

template<typename T>
FSM_StateName FSM_State_Locomotion<T>::checkTransition()
{
  ++iteration_;
  if (!locomotionSafe()) {
    this->nextStateName = FSM_StateName::BALANCE_STAND;
    this->transitionDuration = T(0);
    return this->nextStateName;
  }

  switch (this->_data->desired_state->mode) {
    case ControlMode::Locomotion:
      break;
    case ControlMode::BalanceStand:
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      this->transitionDuration = T(0);
      break;
    case ControlMode::Passive:
      this->nextStateName = FSM_StateName::PASSIVE;
      this->transitionDuration = T(0);
      break;
    case ControlMode::JointPd:
      this->nextStateName = FSM_StateName::JOINT_PD;
      this->transitionDuration = T(0);
      break;
    case ControlMode::StandUp:
      this->nextStateName = FSM_StateName::STAND_UP;
      this->transitionDuration = T(0);
      break;
    case ControlMode::RecoveryStand:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      this->transitionDuration = T(0);
      break;
  }
  return this->nextStateName;
}

template<typename T>
TransitionData<T> FSM_State_Locomotion<T>::transition()
{
  if (this->nextStateName == FSM_StateName::BALANCE_STAND) {
    LocomotionControlStep();
  }
  if (this->nextStateName == FSM_StateName::PASSIVE) {
    this->turnOffAllSafetyChecks();
  }
  this->transitionData.done = true;
  return this->transitionData;
}

template<typename T>
bool FSM_State_Locomotion<T>::locomotionSafe() const
{
  // 行走安全条件比普通 FSM 姿态检查更严格，并同时限制足端位置与速度。
  const StateEstimate<T> & estimate = *this->_data->state_estimate;
  constexpr T max_roll_degrees = T(40);
  constexpr T max_pitch_degrees = T(40);
  if (!estimate.valid ||
    std::abs(estimate.rpy.x()) > ori::deg2rad(max_roll_degrees) ||
    std::abs(estimate.rpy.y()) > ori::deg2rad(max_pitch_degrees))
  {
    return false;
  }

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & data = this->_data->leg_controller->datas[leg];
    if (!data.valid || data.p.z() > T(0) || std::abs(data.p.y()) > T(0.18) ||
      data.v.norm() > T(9))
    {
      return false;
    }
  }
  return true;
}

template<typename T>
std::array<Vec3<T>, kNumLegs>
FSM_State_Locomotion<T>::footPositionsWorld() const
{
  std::array<Vec3<T>, kNumLegs> positions{};
  const auto & estimate = *this->_data->state_estimate;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const LegId leg_id = static_cast<LegId>(leg);
    // 腿传感器给出足端相对髋的位置：先平移到机身系，再旋转和平移到世界系。
    const Vec3<T> foot_body = this->_data->quadruped->hipLocation(leg_id) +
      this->_data->leg_controller->datas[leg].p;
    positions[leg] = estimate.position_world +
      estimate.rotation_world_from_body * foot_body;
  }
  return positions;
}

template<typename T>
void FSM_State_Locomotion<T>::LocomotionControlStep()
{
  // MPC 输入必须使用统一的世界坐标系，否则反作用力方向会与 WBC 不一致。
  const auto feet_world = footPositionsWorld();
  const auto result = mpc_->run(
    *this->_data->state_estimate, *this->_data->desired_state, feet_world);
  if (!result.valid) {
    this->_data->leg_controller->zeroCommand();
    this->_data->leg_controller->setEnabled(false);
    return;
  }

  const auto & desired = *this->_data->desired_state;
  wbc_data_.pBody_des = desired.body_position_world;
  wbc_data_.vBody_des = desired.body_velocity_world;
  wbc_data_.aBody_des = desired.body_acceleration_world;
  wbc_data_.pBody_RPY_des = desired.body_rpy;
  wbc_data_.vBody_Ori_des = desired.body_angular_velocity;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    // 当前 MPC 负责接触时序和反力优化，但公开结果尚未提供独立落脚轨迹；
    // 因此本周期继续使用当前世界系足端位置作为摆动足目标。
    wbc_data_.pFoot_des[leg] = feet_world[leg];
    wbc_data_.vFoot_des[leg].setZero();
    wbc_data_.aFoot_des[leg].setZero();
    wbc_data_.Fr_des[leg] = result.reaction_forces_world[leg];
    wbc_data_.contact_state[leg] = result.contact_phase[leg] > T(0) ? T(1) : T(0);
  }

  if (this->_data->use_wbc) {
    // 正常路径由 WBC 同时输出位置、速度和前馈力矩。
    wbc_ctrl_->runAndApply(
      &wbc_data_, *this->_data->state_estimate, *this->_data->joint_states,
      *this->_data->leg_controller);
  } else {
    // 关闭 WBC 时的降级路径只发送足端前馈力，主要用于调试算法分层。
    auto & controller = *this->_data->leg_controller;
    controller.zeroCommand();
    controller.setEnabled(true);
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      controller.commands[leg].force_feedforward =
        this->_data->state_estimate->rotation_world_from_body.transpose() *
        result.reaction_forces_world[leg];
    }
  }
}

template<typename T>
void FSM_State_Locomotion<T>::onExit()
{
  iteration_ = 0;
}

template class FSM_State_Locomotion<float>;
