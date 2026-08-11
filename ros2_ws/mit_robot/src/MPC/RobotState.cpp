#include "MPC/RobotState.h"

#include <cmath>

// 把状态估计器的输出整理成 MPC 使用的世界坐标系状态，并集中完成有效性检查。
namespace mpc
{

template<typename T>
RobotState<T> RobotState<T>::fromEstimate(
  const StateEstimate<T> & estimate,
  const std::array<Vec3<T>, kNumLegs> & feet_world)
{
  RobotState result;
  result.position_world = estimate.position_world;
  result.velocity_world = estimate.velocity_world;
  result.rpy = estimate.rpy;
  result.rotation_world_from_body = estimate.rotation_world_from_body;
  // 估计器给出机身系角速度；MPC 的动力学方程统一在世界坐标系中计算。
  result.angular_velocity_world =
    estimate.rotation_world_from_body * estimate.angular_velocity_body;
  result.foot_position_world = feet_world;
  result.timestamp = estimate.timestamp;
  result.valid = estimate.valid;
  return result;
}

template<typename T>
bool RobotState<T>::isValid() const noexcept
{
  if (!valid || !position_world.allFinite() || !velocity_world.allFinite() ||
    !rpy.allFinite() || !angular_velocity_world.allFinite() ||
    !rotation_world_from_body.allFinite() ||
    !std::isfinite(static_cast<double>(timestamp)))
  {
    return false;
  }
  for (const auto & foot : foot_position_world) {
    if (!foot.allFinite()) {return false;}
  }
  return true;
}

template struct RobotState<float>;
template struct RobotState<double>;

}  // namespace mpc
