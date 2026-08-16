#include <gtest/gtest.h>

// 验证 MPC 求解器的默认迭代配置、站立力约束以及非法输入保护。

#include <vector>

#include "MPC/SolverMPC.h"
#include "model/robots/unitree_go1.hpp"
#include "mpc_test_support.hpp"

namespace
{

TEST(MpcSolver, UsesRaisedDefaultIterationLimit)
{
  // 默认设置应保留项目为提高求解收敛裕量而采用的迭代上限。
  const mpc::SolverSettings<double> settings;
  EXPECT_EQ(settings.maximum_iterations, 75U);
}

TEST(MpcSolver, SupportsStandingWeightAndFrictionConstraints)
{
  // 四足全支撑时，输出力应满足法向力边界和摩擦锥约束。
  const auto quadruped = robots::unitree_go1::makeModel<double>();
  mpc::SolverSettings<double> settings;
  settings.horizon = 4;
  settings.maximum_iterations = 1000;
  settings.convergence_tolerance = 1e-7;
  mpc::SolverMPC<double> solver(quadruped, settings);
  const auto state = mpc::RobotState<double>::fromEstimate(
    test_support::standingEstimate(), test_support::standingFeet());
  std::vector<DesiredState<double>> trajectory(
    settings.horizon, test_support::standingDesired());
  std::vector<int> contacts(settings.horizon * kNumLegs, 1);

  const auto result = solver.solve(state, trajectory, contacts);
  ASSERT_TRUE(result.valid);
  double total_vertical_force = 0.0;
  for (const auto & force : result.reaction_forces_world) {
    EXPECT_GE(force.z(), settings.minimum_normal_force);
    EXPECT_LE(force.z(), settings.maximum_normal_force);
    EXPECT_LE(force.head<2>().norm(), settings.friction_coefficient * force.z() + 1e-9);
    total_vertical_force += force.z();
  }
  EXPECT_NEAR(total_vertical_force, quadruped.totalMass() * 9.81, 8.0);
}

TEST(MpcSolver, RejectsInvalidInputWithoutPublishingForces)
{
  // 无效状态不能让求解器发布可被控制器误用的反作用力。
  const auto quadruped = robots::unitree_go1::makeModel<double>();
  mpc::SolverSettings<double> settings;
  settings.horizon = 4;
  mpc::SolverMPC<double> solver(quadruped, settings);
  mpc::RobotState<double> invalid_state;
  std::vector<DesiredState<double>> trajectory(
    settings.horizon, test_support::standingDesired());
  std::vector<int> contacts(settings.horizon * kNumLegs, 1);
  EXPECT_FALSE(solver.solve(invalid_state, trajectory, contacts).valid);
}

}  // namespace
