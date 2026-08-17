#include <gtest/gtest.h>

// 验证 MPC 状态映射将机身角速度正确转换为 RPY 角速度，而不是简单旋转向量。

#include "MPC/RobotState.h"
#include "mpc_test_support.hpp"

namespace
{

TEST(MpcRobotState, MapsBodyAngularVelocityToRpyRate)
{
  // 采用非零 roll、pitch、yaw 检查欧拉角速率映射中的姿态耦合项。
  StateEstimate<double> estimate;
  estimate.rpy << 0.31, -0.22, 0.47;
  estimate.orientation_world_from_body =
    Eigen::AngleAxisd(estimate.rpy.z(), Vec3<double>::UnitZ()) *
    Eigen::AngleAxisd(estimate.rpy.y(), Vec3<double>::UnitY()) *
    Eigen::AngleAxisd(estimate.rpy.x(), Vec3<double>::UnitX());
  estimate.rotation_world_from_body =
    estimate.orientation_world_from_body.toRotationMatrix();
  estimate.angular_velocity_body << 0.42, -0.17, 0.28;
  estimate.valid = true;

  const auto state = mpc::RobotState<double>::fromEstimate(
    estimate, test_support::standingFeet());
  constexpr double dt = 1.0e-7;
  const Eigen::Quaterniond advanced = estimate.orientation_world_from_body *
    Eigen::Quaterniond(
    Eigen::AngleAxisd(
      estimate.angular_velocity_body.norm() * dt,
      estimate.angular_velocity_body.normalized()));
  const Mat3<double> rotation = advanced.toRotationMatrix();
  Vec3<double> advanced_rpy;
  advanced_rpy <<
    std::atan2(rotation(2, 1), rotation(2, 2)),
    std::asin(-rotation(2, 0)),
    std::atan2(rotation(1, 0), rotation(0, 0));
  const Vec3<double> numerical_rate = (advanced_rpy - estimate.rpy) / dt;
  // 用小时间步数值微分作为参考，并排除错误的 R*omega 直接映射。
  EXPECT_TRUE(state.rpy_rate.isApprox(numerical_rate, 1.0e-7));
  EXPECT_FALSE(
    state.rpy_rate.isApprox(
      estimate.rotation_world_from_body * estimate.angular_velocity_body,
      1.0e-3));
}

}  // namespace
