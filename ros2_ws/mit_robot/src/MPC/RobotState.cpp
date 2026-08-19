/**
 * @file RobotState.cpp
 * @brief MPC 紧凑状态的构造、坐标转换和有效性检查实现。
 */

#include "MPC/RobotState.h"

#include <cmath>
#include <limits>

// 把状态估计器的输出整理成 MPC 使用的世界坐标系状态，并集中完成有效性检查。
namespace mpc
{

/**
 * @brief 将机身角速度转换为 ZYX 欧拉角导数。
 * @tparam T 标量类型。
 * @param rpy 当前滚转、俯仰、偏航角。
 * @param angular_velocity_body 机身坐标系角速度。
 * @return 欧拉角速度；俯仰角接近奇异时返回 NaN。
 */
template<typename T>
Vec3<T> RobotState<T>::rpyRateFromBodyAngularVelocity(
  const Vec3<T> & rpy, const Vec3<T> & angular_velocity_body)
{ 
  const T sin_roll = std::sin(rpy.x());
  const T cos_roll = std::cos(rpy.x());
  const T cos_pitch = std::cos(rpy.y());
  if (std::abs(cos_pitch) <= T(1e-4)) {
    return Vec3<T>::Constant(std::numeric_limits<T>::quiet_NaN());
  }
  const T tan_pitch = std::tan(rpy.y());
  Mat3<T> body_angular_velocity_to_rpy_rate;
  body_angular_velocity_to_rpy_rate <<
    T(1), sin_roll * tan_pitch, cos_roll * tan_pitch,
    T(0), cos_roll, -sin_roll,
    T(0), sin_roll / cos_pitch, cos_roll / cos_pitch;
  return body_angular_velocity_to_rpy_rate * angular_velocity_body;
}

/**
 * @brief 从状态估计构造 MPC 所需的机器人状态。
 * @tparam T 标量类型。
 * @param estimate 状态估计数据。
 * @param feet_world 世界坐标系足端位置。
 * @return 填充后的 MPC 状态。
 */
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
  // MPC 状态直接积分 RPY，因此这里必须使用欧拉角导数，不能用世界角速度替代。
  result.rpy_rate = rpyRateFromBodyAngularVelocity(
    estimate.rpy, estimate.angular_velocity_body);
  result.foot_position_world = feet_world;
  result.timestamp = estimate.timestamp;
  result.valid = estimate.valid;
  return result;
}

/**
 * @brief 检查状态是否可以安全地进入 MPC 求解器。
 * @tparam T 标量类型。
 * @return 所有状态量有限且 valid 标志为 true 时返回 true。
 */
template<typename T>
bool RobotState<T>::isValid() const noexcept
{
  if (!valid || !position_world.allFinite() || !velocity_world.allFinite() ||
    !rpy.allFinite() || !rpy_rate.allFinite() ||
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
