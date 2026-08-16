#include "FSM/FSM_State_FrontJump.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

template<typename T>
FSM_State_FrontJump<T>::FSM_State_FrontJump(
  ControlFSMData<T> * control_fsm_data)
: FSM_State<T>(control_fsm_data, FSM_StateName::FRONT_JUMP, "FRONT_JUMP")
{
  if (control_fsm_data == nullptr || !control_fsm_data->valid()) {
    throw std::invalid_argument("front-jump state requires valid FSM data");
  }

  // 跳跃时允许短时较大俯仰；ControlFSM 仍会统一拒绝任何 NaN/Inf 命令。
  this->turnOffAllSafetyChecks();
  const T dt = control_fsm_data->control_time_step;
  const auto cycles = [dt](T duration) {
      return std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(duration / dt)));
    };
  crouch_iterations_ = cycles(T(0.18));
  thrust_iterations_ = cycles(T(0.18));
  tuck_iterations_ = cycles(T(0.17));
  landing_iterations_ = cycles(T(0.25));

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    // 预蹲储能；蹬伸时足端相对髋部向后，使地面反力带来向前冲量。
    crouch_positions_[leg] << T(0), T(1.22), T(-2.42);
    const bool front_leg = leg == static_cast<std::size_t>(LegId::FR) ||
      leg == static_cast<std::size_t>(LegId::FL);
    if (front_leg) {
      // 前腿足端更多向后扫，增加水平推进并减少造成过大俯仰的竖直冲量。
      thrust_positions_[leg] << T(0), T(1.20), T(-1.25);
    } else {
      // 后腿保留更多竖直支撑，产生与实测负向俯仰相反的校正力矩。
      thrust_positions_[leg] << T(0), T(1.10), T(-1.20);
    }
    tuck_positions_[leg] << T(0), T(1.12), T(-2.24);
    landing_positions_[leg] =
      control_fsm_data->quadruped->leg(static_cast<LegId>(leg)).joints.home_position;
  }
}

template<typename T>
void FSM_State_FrontJump<T>::onEnter()
{
  this->nextStateName = this->stateName;
  this->transitionData.zero();
  iteration_ = 0;
  jump_complete_ = false;
  auto & controller = *this->_data->leg_controller;
  controller.zeroCommand();
  controller.setEnabled(true);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    initial_positions_[leg] = controller.datas[leg].valid ?
      controller.datas[leg].q : landing_positions_[leg];
  }
  this->_data->gait_scheduler->requestGait(GaitType::STAND);
}

template<typename T>
T FSM_State_FrontJump<T>::smoothstep(T progress) noexcept
{
  progress = std::clamp(progress, T(0), T(1));
  return progress * progress * (T(3) - T(2) * progress);
}

template<typename T>
T FSM_State_FrontJump<T>::smoothstepRate(T progress, T duration) noexcept
{
  progress = std::clamp(progress, T(0), T(1));
  return duration > T(0) ? T(6) * progress * (T(1) - progress) / duration : T(0);
}

template<typename T>
void FSM_State_FrontJump<T>::applyTrajectory(
  const std::array<Vec3<T>, kNumLegs> & start,
  const std::array<Vec3<T>, kNumLegs> & target,
  std::size_t phase_iteration, std::size_t phase_iterations, T kp, T kd)
{
  const T duration = static_cast<T>(phase_iterations) *
    this->_data->control_time_step;
  const T linear_progress = std::clamp(
    static_cast<T>(phase_iteration) / static_cast<T>(phase_iterations), T(0), T(1));
  const T blend = smoothstep(linear_progress);
  const T blend_rate = smoothstepRate(linear_progress, duration);
  auto & controller = *this->_data->leg_controller;
  controller.zeroCommand();
  controller.setEnabled(true);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    auto & command = controller.commands[leg];
    const Vec3<T> travel = target[leg] - start[leg];
    command.position_desired = start[leg] + blend * travel;
    command.velocity_desired = blend_rate * travel;
    const auto & limits = this->_data->quadruped->leg(
      static_cast<LegId>(leg)).joints;
    command.position_desired = command.position_desired.cwiseMax(
      limits.lower_limit).cwiseMin(limits.upper_limit);
    command.velocity_desired = command.velocity_desired.cwiseMax(
      -limits.velocity_limit).cwiseMin(limits.velocity_limit);
    command.kp_joint.setConstant(kp);
    command.kd_joint.setConstant(kd);
  }
}

template<typename T>
void FSM_State_FrontJump<T>::run()
{
  const std::size_t crouch_end = crouch_iterations_;
  const std::size_t thrust_end = crouch_end + thrust_iterations_;
  const std::size_t tuck_end = thrust_end + tuck_iterations_;
  const std::size_t landing_end = tuck_end + landing_iterations_;

  if (iteration_ < crouch_end) {
    applyTrajectory(
      initial_positions_, crouch_positions_, iteration_, crouch_iterations_,
      T(48), T(5));
  } else if (iteration_ < thrust_end) {
    applyTrajectory(
      crouch_positions_, thrust_positions_, iteration_ - crouch_end,
      thrust_iterations_, T(70), T(4));
  } else if (iteration_ < tuck_end) {
    applyTrajectory(
      thrust_positions_, tuck_positions_, iteration_ - thrust_end,
      tuck_iterations_, T(32), T(3));
  } else {
    applyTrajectory(
      tuck_positions_, landing_positions_, iteration_ - tuck_end,
      landing_iterations_, T(38), T(5));
    // 落地阶段用 IMU 俯仰和俯仰角速度做有限幅差动腿长修正。当前坐标约定下
    // 负 pitch 为抬头：缩短前腿、伸长后腿可抑制继续后翻；正 pitch 反向处理。
    const T pitch = this->_data->state_estimate->rpy.y();
    const T pitch_rate = this->_data->state_estimate->angular_velocity_body.y();
    const T correction = std::clamp(
      -T(0.35) * pitch - T(0.06) * pitch_rate, T(-0.12), T(0.12));
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const bool front_leg = leg == static_cast<std::size_t>(LegId::FR) ||
        leg == static_cast<std::size_t>(LegId::FL);
      const T signed_correction = front_leg ? correction : -correction;
      auto & command = this->_data->leg_controller->commands[leg];
      command.position_desired.y() += signed_correction;
      command.position_desired.z() -= T(2) * signed_correction;
      const auto & limits = this->_data->quadruped->leg(
        static_cast<LegId>(leg)).joints;
      command.position_desired = command.position_desired.cwiseMax(
        limits.lower_limit).cwiseMin(limits.upper_limit);
      command.kp_joint.setConstant(T(44));
      command.kd_joint.setConstant(T(6));
    }
  }

  ++iteration_;
  if (iteration_ >= landing_end) {
    jump_complete_ = true;
    // 单次动作完成后主动回到平衡站立，无需用户再次点击。
    this->_data->desired_state->mode = ControlMode::BalanceStand;
  }
}

template<typename T>
FSM_StateName FSM_State_FrontJump<T>::checkTransition()
{
  if (this->_data->desired_state->mode == ControlMode::Passive) {
    this->nextStateName = FSM_StateName::PASSIVE;
  } else if (jump_complete_) {
    this->nextStateName = FSM_StateName::BALANCE_STAND;
  } else {
    this->nextStateName = this->stateName;
  }
  this->transitionDuration = T(0);
  return this->nextStateName;
}

template<typename T>
TransitionData<T> FSM_State_FrontJump<T>::transition()
{
  if (this->nextStateName == FSM_StateName::PASSIVE) {
    this->_data->leg_controller->zeroCommand();
    this->_data->leg_controller->setEnabled(false);
  } else {
    run();
  }
  this->transitionData.done = true;
  return this->transitionData;
}

template<typename T>
void FSM_State_FrontJump<T>::onExit()
{
  iteration_ = 0;
  jump_complete_ = false;
}

template class FSM_State_FrontJump<float>;
