#pragma once

// 提供 MPC 测试使用的站立足端、状态估计和期望状态基准数据。

#include <array>

#include "model/robot_types.hpp"

namespace test_support
{

inline std::array<Vec3<double>, kNumLegs> standingFeet()
{
  // 足端顺序固定为 FR、FL、RR、RL，与控制器和步态表保持一致。
  return {
    Vec3<double>(0.2, -0.13, 0.0), Vec3<double>(0.2, 0.13, 0.0),
    Vec3<double>(-0.2, -0.13, 0.0), Vec3<double>(-0.2, 0.13, 0.0)};
}

inline StateEstimate<double> standingEstimate()
{
  // 构造有效的水平站立机身状态，供状态映射和求解器测试复用。
  StateEstimate<double> estimate;
  estimate.position_world << 0.0, 0.0, 0.27;
  estimate.valid = true;
  return estimate;
}

inline DesiredState<double> standingDesired()
{
  // 站立目标只填写 MPC 需要的机身高度和控制模式。
  DesiredState<double> desired;
  desired.mode = ControlMode::BalanceStand;
  desired.body_position_world << 0.0, 0.0, 0.27;
  desired.valid = true;
  return desired;
}

}  // namespace test_support
