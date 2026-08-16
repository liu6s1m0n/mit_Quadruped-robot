#include "MPC/ConvexMPCLocomotion.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

// 运动层 MPC 封装：根据步态生成接触表和目标轨迹，再调用 SolverMPC 计算支撑力。
namespace mpc
{

template<typename T>
ConvexMPCLocomotion<T>::ConvexMPCLocomotion(
  const Quadruped<T> & quadruped, T control_time_step,
  std::size_t iterations_between_mpc, const SolverSettings<T> & settings,
  std::size_t iterations_per_gait_segment)
: control_time_step_(control_time_step),
  iterations_between_mpc_(iterations_between_mpc),
  iterations_per_gait_segment_(
    iterations_per_gait_segment == 0 ? iterations_between_mpc :
    iterations_per_gait_segment),
  solver_(quadruped, settings),
  //构造站立步态
  stand_(settings.horizon, {0, 0, 0, 0},
    {settings.horizon, settings.horizon, settings.horizon, settings.horizon}, "stand"),
  // 构造 60% 支撑率的 TROT：默认 10 段中支撑 6 段、摆动 4 段。
  // 当前 FSM 每段接触时序为 50 ms，因此完整摆动时间为 0.20 s。
  trot_(settings.horizon, {0, settings.horizon / 2, settings.horizon / 2, 0},
    {settings.horizon * 3 / 5, settings.horizon * 3 / 5,
      settings.horizon * 3 / 5, settings.horizon * 3 / 5}, "trot_walk")
{
  // 对角小跑中 LF+RH 与 RF+LH 分成两组，相位相差半个预测时域。
  if (!std::isfinite(static_cast<double>(control_time_step_)) ||
    control_time_step_ <= T(0) || iterations_between_mpc_ == 0 ||
    iterations_per_gait_segment_ == 0 || settings.horizon < 2 ||
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
  commanded_velocity_body_.setZero();
  commanded_yaw_rate_ = T(0);
  command_yaw_ = T(0);
  cached_reaction_forces_world_ = {};
  cached_contact_state_ = {};
  cached_solution_valid_ = false;
  cached_solution_converged_ = false;
}

template<typename T>
void ConvexMPCLocomotion<T>::setForwardVelocity(T velocity)
{
  setVelocityCommand(velocity, T(0), T(0));
}

template<typename T>
void ConvexMPCLocomotion<T>::setVelocityCommand(
  T forward_velocity, T lateral_velocity, T yaw_rate)
{
  constexpr T maximum_linear_velocity = T(1);
  constexpr T maximum_yaw_rate = T(2);
  const Vec2<T> linear_velocity(forward_velocity, lateral_velocity);
  if (!linear_velocity.allFinite() ||
    linear_velocity.norm() > maximum_linear_velocity)
  {
    throw std::invalid_argument(
            "locomotion planar velocity magnitude must not exceed 1 m/s");
  }
  if (!std::isfinite(static_cast<double>(yaw_rate)) ||
    std::abs(yaw_rate) > maximum_yaw_rate)
  {
    throw std::invalid_argument(
            "locomotion yaw rate must be within [-2, 2] rad/s");
  }
  velocity_body_ = linear_velocity;
  yaw_rate_ = yaw_rate;
}

template<typename T>
DesiredState<T> ConvexMPCLocomotion<T>::setupCommand(
  const StateEstimate<T> & estimate, const DesiredState<T> & desired)
{
  DesiredState<T> command = desired;
  if (!command_initialized_) {
    command_yaw_ = desired.body_rpy.z();
    command_initialized_ = true;
  }

  // 对二维线速度做向量限幅，方向切换时总加速度也不会超过设定值。
  Vec2<T> velocity_change = velocity_body_ - commanded_velocity_body_;
  const T maximum_velocity_step = maximum_linear_acceleration_ * control_time_step_;
  if (velocity_change.norm() > maximum_velocity_step) {
    velocity_change *= maximum_velocity_step / velocity_change.norm();
  }
  commanded_velocity_body_ += velocity_change;

  const T maximum_yaw_rate_step = maximum_yaw_acceleration_ * control_time_step_;
  commanded_yaw_rate_ += std::clamp(
    yaw_rate_ - commanded_yaw_rate_, -maximum_yaw_rate_step,
    maximum_yaw_rate_step);

  // 将连续偏航目标重表达到实测偏航附近，跨越 +/-pi 时避免产生 2*pi 跳变。
  const T relative_yaw = command_yaw_ - estimate.rpy.z();
  command_yaw_ = estimate.rpy.z() +
    std::atan2(std::sin(relative_yaw), std::cos(relative_yaw));
  command_yaw_ += commanded_yaw_rate_ * control_time_step_;
  command.body_rpy.x() = T(0);
  command.body_rpy.y() = T(0);
  command.body_rpy.z() = command_yaw_;

  const T cosine_yaw = std::cos(command_yaw_);
  const T sine_yaw = std::sin(command_yaw_);
  command.body_velocity_world <<
    cosine_yaw * commanded_velocity_body_.x() -
    sine_yaw * commanded_velocity_body_.y(),
    sine_yaw * commanded_velocity_body_.x() +
    cosine_yaw * commanded_velocity_body_.y(), T(0);
  command.body_acceleration_world.setZero();
  command.body_angular_velocity << T(0), T(0), commanded_yaw_rate_;
  // 速度遥控模式不长期积分世界 x/y 目标。每周期从当前实测位置生成短时
  // 预测轨迹，可避免打滑或跟踪误差累积成不断增大的“追赶位置”，进而让
  // MPC 把实际速度推到命令速度两倍以上。
  command_position_world_.template head<2>() =
    estimate.position_world.template head<2>();
  command_position_world_.z() = desired.body_position_world.z();
  command.body_position_world = command_position_world_;
  command.valid = desired.valid && command.body_position_world.allFinite() &&
    command.body_velocity_world.allFinite() && command.body_rpy.allFinite() &&
    command.body_angular_velocity.allFinite();
  return command;
}

template<typename T>
void ConvexMPCLocomotion<T>::setGait(GaitType gait)
{
  if (gait != GaitType::STAND && gait != GaitType::TROT) {
    throw std::invalid_argument("MPC locomotion currently supports STAND and TROT");
  }
  if (gait_type_ != gait) {
    gait_type_ = gait;
    // 接触约束随步态改变，旧反力不能继续沿用。
    cached_solution_valid_ = false;
    cached_solution_converged_ = false;
  }
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
  gait.advance(iteration_, iterations_per_gait_segment_);
  const auto contact_phase = gait.contactPhase();
  const auto swing_phase = gait.swingPhase();
  const auto & contact_table = gait.contactTable();
  const T gait_segment_time =
    control_time_step_ * static_cast<T>(iterations_per_gait_segment_);
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

  // 500 Hz 控制循环通常每 iterations_between_mpc_ 帧执行一次昂贵的优化。
  // 其余帧仍更新命令、接触相位和摆腿轨迹，并把最近的有效反力交给 WBC；
  // 接触组合发生变化时例外，必须立即刷新反力。
  // 求解周期和步态分段解耦后，接触切换不一定落在固定的 25 Hz 刷新点。
  // 接触组合变化时立即补算一次，避免新支撑腿短时间沿用摆动期的零反力。
  const bool contact_state_changed = cached_solution_valid_ &&
    result.contact_state != cached_contact_state_;
  const bool solve_due = !cached_solution_valid_ || contact_state_changed ||
    iteration_ % iterations_between_mpc_ == 0;
  if (solve_due) {
    std::vector<DesiredState<T>> trajectory(
      solver_.settings().horizon, result.command);
    // 用期望速度外推未来位置和偏航角，形成 MPC 需要的整段参考轨迹。
    for (std::size_t step = 0; step < trajectory.size(); ++step) {
      const T lookahead = solver_.settings().time_step * static_cast<T>(step + 1);
      trajectory[step].body_position_world =
        result.command.body_position_world +
        lookahead * result.command.body_velocity_world;
      const Vec3<T> desired_rpy_rate =
        RobotState<T>::rpyRateFromBodyAngularVelocity(
        result.command.body_rpy, result.command.body_angular_velocity);
      trajectory[step].body_rpy =
        result.command.body_rpy + lookahead * desired_rpy_rate;
      // DesiredState 明确定义为机身系角速度；SolverMPC 在构造目标向量时负责
      // 按目标姿态转换为 RPY 导数，此处不得提前改写成世界系。
      trajectory[step].body_angular_velocity = result.command.body_angular_velocity;
    }

    const RobotState<T> state =
      RobotState<T>::fromEstimate(estimate, foot_positions_world);
    const SolverResult<T> solution = solver_.solve(state, trajectory, contact_table);
    if (solution.valid) {
      cached_reaction_forces_world_ = solution.reaction_forces_world;
      cached_contact_state_ = result.contact_state;
      cached_solution_valid_ = true;
      cached_solution_converged_ = solution.converged;
      result.mpc_updated = true;
    } else {
      // 刷新点通常也是接触预测进入下一段的边界。新解失败时不能继续把
      // 上一段接触组合的反力施加到当前支撑腿；标记无效并在下一帧重试。
      cached_solution_valid_ = false;
      cached_solution_converged_ = false;
    }
  }

  if (cached_solution_valid_) {
    result.reaction_forces_world = cached_reaction_forces_world_;
    result.mpc_converged = cached_solution_converged_;
    result.valid = true;
  }
  ++iteration_;
  return result;
}

template struct LocomotionResult<float>;
template struct LocomotionResult<double>;
template class ConvexMPCLocomotion<float>;
template class ConvexMPCLocomotion<double>;

}  // namespace mpc
