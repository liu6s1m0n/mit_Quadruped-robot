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

  auto & gait = activeGait();
  // iteration_ 是高速控制周期计数，步态内部会换算成较慢的 MPC 分段相位。
  gait.advance(iteration_, iterations_between_mpc_);
  const auto contact_phase = gait.contactPhase();
  const auto swing_phase = gait.swingPhase();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    result.contact_phase[leg] = static_cast<T>(contact_phase[leg]);
    result.swing_phase[leg] = static_cast<T>(swing_phase[leg]);
  }

  std::vector<DesiredState<T>> trajectory(solver_.settings().horizon, desired);
  // 用期望速度外推未来位置和偏航角，形成 MPC 需要的整段参考轨迹。
  for (std::size_t step = 0; step < trajectory.size(); ++step) {
    const T lookahead = solver_.settings().time_step * static_cast<T>(step + 1);
    trajectory[step].body_position_world =
      desired.body_position_world + lookahead * desired.body_velocity_world;
    trajectory[step].body_rpy.z() =
      desired.body_rpy.z() + lookahead * desired.body_angular_velocity.z();
    trajectory[step].body_angular_velocity =
      estimate.rotation_world_from_body * desired.body_angular_velocity;
  }

  const RobotState<T> state = RobotState<T>::fromEstimate(estimate, foot_positions_world);
  // 每次 run 都重新求解，但只把第一步地面力交给下游 WBC 使用。
  const SolverResult<T> solution = solver_.solve(state, trajectory, gait.contactTable());
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
