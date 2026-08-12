#pragma once

#include <array>

#include "model/robot_types.hpp"

namespace test_support
{

inline std::array<Vec3<double>, kNumLegs> standingFeet()
{
  return {
    Vec3<double>(0.2, -0.13, 0.0), Vec3<double>(0.2, 0.13, 0.0),
    Vec3<double>(-0.2, -0.13, 0.0), Vec3<double>(-0.2, 0.13, 0.0)};
}

inline StateEstimate<double> standingEstimate()
{
  StateEstimate<double> estimate;
  estimate.position_world << 0.0, 0.0, 0.27;
  estimate.valid = true;
  return estimate;
}

inline DesiredState<double> standingDesired()
{
  DesiredState<double> desired;
  desired.mode = ControlMode::BalanceStand;
  desired.body_position_world << 0.0, 0.0, 0.27;
  desired.valid = true;
  return desired;
}

}  // namespace test_support
