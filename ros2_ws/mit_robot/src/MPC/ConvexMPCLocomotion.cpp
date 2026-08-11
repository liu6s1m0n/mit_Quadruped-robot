#include "MPC/ConvexMPCLocomotion.h"

#include <cmath>
#include <stdexcept>
#include <vector>

// 运动层 MPC 封装：根据步态生成接触表和目标轨迹，再调用 SolverMPC 计算支撑力。
namespace mpc
{

template<typename T>
ConvexMPCLocomotion<T>::ConvexMPCLocomotion(
  const Quadruped<T> & quadruped, T control_time_step,
  std::size_t iterations_between_mpc, const SolverSettings<T> & settings)
: control_time_step_(control_time_step),
  iterations_between_mpc_(iterations_between_mpc), solver_(quadruped, settings),
  //构造站立步态
  stand_(settings.horizon, {0, 0, 0, 0},
    {settings.horizon, settings.horizon, settings.horizon, settings.horizon}, "stand"),
  //构造 TROT 步态
  trot_(settings.horizon, {0, settings.horizon / 2, settings.horizon / 2, 0},
    {settings.horizon / 2, settings.horizon / 2,
      settings.horizon / 2, settings.horizon / 2}, "trot")
{
  // 对角小跑中 LF+RH 与 RF+LH 分成两组，相位相差半个预测时域。
  if (!std::isfinite(static_cast<double>(control_time_step_)) ||
    control_time_step_ <= T(0) || iterations_between_mpc_ == 0 || settings.horizon < 2 ||
    settings.horizon % 2 != 0)
  {
    throw std::invalid_argument("invalid convex MPC locomotion timing");
  }
}

template<typename T>
void ConvexMPCLocomotion<T>::initialize() noexcept
{
  iteration_ = 0;
  command_initialized_ = false;
}

template<typename T>
void ConvexMPCLocomotion<T>::setForwardVelocity(T velocity)
{
  constexpr T maximum_velocity = T(1);
  if (!std::isfinite(static_cast<double>(velocity)) ||
    std::abs(velocity) > maximum_velocity)
  {
    throw std::invalid_argument(
            "locomotion forward velocity must be within [-1, 1] m/s");
  }
  forward_velocity_ = velocity;
}

template<typename T>
DesiredState<T> ConvexMPCLocomotion<T>::setupCommand(
  const StateEstimate<T> & estimate, const DesiredState<T> & desired)
{
  DesiredState<T> command = desired;
  if (!command_initialized_) {
    command_position_world_ = estimate.position_world;
    command_position_world_.z() = desired.body_position_world.z();
    command_initialized_ = true;
  }

  // “向前”由期望偏航角定义，再转换到 MPC 使用的世界坐标系。
  const T yaw = desired.body_rpy.z();
  command.body_velocity_world <<
    forward_velocity_ * std::cos(yaw),
    forward_velocity_ * std::sin(yaw), T(0);
  command.body_acceleration_world.setZero();
  command.body_angular_velocity.setZero();
  command_position_world_.template head<2>() +=
    control_time_step_ * command.body_velocity_world.template head<2>();
  command_position_world_.z() = desired.body_position_world.z();

  // 执行器饱和时限制位置参考与实测机身之间的距离，避免目标无限跑远。
  Vec2<T> position_error = command_position_world_.template head<2>() -
    estimate.position_world.template head<2>();
  if (position_error.norm() > maximum_position_error_) {
    position_error.normalize();
    command_position_world_.template head<2>() =
      estimate.position_world.template head<2>() +
      maximum_position_error_ * position_error;
  }
  command.body_position_world = command_position_world_;
  command.valid = desired.valid && command.body_position_world.allFinite() &&
    command.body_velocity_world.allFinite();
  return command;
}

template<typename T>
void ConvexMPCLocomotion<T>::setGait(GaitType gait)
{
  if (gait != GaitType::STAND && gait != GaitType::TROT) {
    throw std::invalid_argument("MPC locomotion currently supports STAND and TROT");
  }
  gait_type_ = gait;
}

template<typename T>
OffsetDurationGait & ConvexMPCLocomotion<T>::activeGait() noexcept
{
  return gait_type_ == GaitType::STAND ? stand_ : trot_;
}

template<typename T>
LocomotionResult<T> ConvexMPCLocomotion<T>::run(
  const StateEstimate<T> & estimate, const DesiredState<T> & desired,
  const std::array<Vec3<T>, kNumLegs> & foot_positions_world)
{
  LocomotionResult<T> result;
  if (!estimate.valid || !desired.valid) {return result;}
  result.command = setupCommand(estimate, desired);
  if (!result.command.valid) {return result;}

  auto & gait = activeGait();
  // iteration_ 是高速控制周期计数，步态内部会换算成较慢的 MPC 分段相位。
  gait.advance(iteration_, iterations_between_mpc_);
  const auto contact_phase = gait.contactPhase();
  const auto swing_phase = gait.swingPhase();
  const auto & contact_table = gait.contactTable();
  const T gait_segment_time =
    control_time_step_ * static_cast<T>(iterations_between_mpc_);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    result.contact_phase[leg] = static_cast<T>(contact_phase[leg]);
    result.swing_phase[leg] = static_cast<T>(swing_phase[leg]);
    result.stance_time[leg] = static_cast<T>(gait.stanceTime(
        static_cast<float>(gait_segment_time), leg));
    result.swing_time[leg] = static_cast<T>(gait.swingTime(
        static_cast<float>(gait_segment_time), leg));
    // contactPhase 在支撑阶段的第一个采样恰好为零，不能用 phase > 0
    // 判断接触；预测表第一行才是无歧义的当前接触状态。
    result.contact_state[leg] = contact_table[leg] != 0;
  }

  std::vector<DesiredState<T>> trajectory(
    solver_.settings().horizon, result.command);
  // 用期望速度外推未来位置和偏航角，形成 MPC 需要的整段参考轨迹。
  for (std::size_t step = 0; step < trajectory.size(); ++step) {
    const T lookahead = solver_.settings().time_step * static_cast<T>(step + 1);
    trajectory[step].body_position_world =
      result.command.body_position_world +
      lookahead * result.command.body_velocity_world;
    trajectory[step].body_rpy.z() =
      result.command.body_rpy.z() +
      lookahead * result.command.body_angular_velocity.z();
    trajectory[step].body_angular_velocity =
      estimate.rotation_world_from_body * result.command.body_angular_velocity;
  }

  const RobotState<T> state = RobotState<T>::fromEstimate(estimate, foot_positions_world);
  // 每次 run 都重新求解，但只把第一步地面力交给下游 WBC 使用。
  const SolverResult<T> solution = solver_.solve(state, trajectory, contact_table);
  if (!solution.valid) {++iteration_; return result;}

  result.reaction_forces_world = solution.reaction_forces_world;
  result.mpc_converged = solution.converged;
  result.valid = true;
  ++iteration_;
  return result;
}

template struct LocomotionResult<float>;
template struct LocomotionResult<double>;
template class ConvexMPCLocomotion<float>;
template class ConvexMPCLocomotion<double>;

}  // namespace mpc
