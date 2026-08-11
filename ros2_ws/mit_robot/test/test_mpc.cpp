#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "MPC/ConvexMPCLocomotion.h"
#include "MPC/Gait.h"
#include "MPC/SolverMPC.h"
#include "model/quadruped.hpp"
#include "model/robots/unitree_go1.hpp"

// 验证小跑接触表、站立受力与摩擦约束，以及 MPC 输入异常时的安全失败行为。
namespace
{

std::array<Vec3<double>, kNumLegs> standingFeet()
{
  return {
    Vec3<double>(0.2, -0.13, 0.0), Vec3<double>(0.2, 0.13, 0.0),
    Vec3<double>(-0.2, -0.13, 0.0), Vec3<double>(-0.2, 0.13, 0.0)};
}

StateEstimate<double> standingEstimate()
{
  StateEstimate<double> estimate;
  estimate.position_world << 0.0, 0.0, 0.27;
  estimate.valid = true;
  return estimate;
}

DesiredState<double> standingDesired()
{
  DesiredState<double> desired;
  desired.mode = ControlMode::BalanceStand;
  desired.body_position_world << 0.0, 0.0, 0.27;
  desired.valid = true;
  return desired;
}

TEST(MpcGait, BuildsTrotContactPredictionInProjectLegOrder)
{
  mpc::OffsetDurationGait gait(10, {0, 5, 5, 0}, {5, 5, 5, 5}, "trot");
  gait.advance(0, 10);
  const auto & table = gait.contactTable();
  ASSERT_EQ(table.size(), 40U);
  EXPECT_EQ(table[0], 1);
  EXPECT_EQ(table[1], 0);
  EXPECT_EQ(table[2], 0);
  EXPECT_EQ(table[3], 1);
  EXPECT_EQ(table[5 * kNumLegs], 0);
  EXPECT_EQ(table[5 * kNumLegs + 1], 1);
}

TEST(MpcSolver, SupportsStandingWeightAndFrictionConstraints)
{
  const auto quadruped = robots::unitree_go1::makeModel<double>();
  mpc::SolverSettings<double> settings;
  settings.horizon = 4;
  settings.maximum_iterations = 1000;
  settings.convergence_tolerance = 1e-7;
  mpc::SolverMPC<double> solver(quadruped, settings);
  const auto estimate = standingEstimate();
  const auto state = mpc::RobotState<double>::fromEstimate(estimate, standingFeet());
  std::vector<DesiredState<double>> trajectory(settings.horizon, standingDesired());
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

TEST(MpcLocomotion, RunsWithCurrentStateAndDesiredStateTypes)
{
  const auto quadruped = robots::unitree_go1::makeModel<double>();
  mpc::SolverSettings<double> settings;
  settings.horizon = 4;
  mpc::ConvexMPCLocomotion<double> controller(quadruped, 0.002, 15, settings);
  controller.setGait(GaitType::STAND);

  const auto result = controller.run(
    standingEstimate(), standingDesired(), standingFeet());
  EXPECT_TRUE(result.valid);
  for (double phase : result.contact_phase) {EXPECT_GE(phase, 0.0);}
}

TEST(MpcSolver, RejectsInvalidInputWithoutPublishingForces)
{
  const auto quadruped = robots::unitree_go1::makeModel<double>();
  mpc::SolverSettings<double> settings;
  settings.horizon = 4;
  mpc::SolverMPC<double> solver(quadruped, settings);
  mpc::RobotState<double> invalid_state;
  std::vector<DesiredState<double>> trajectory(settings.horizon, standingDesired());
  std::vector<int> contacts(settings.horizon * kNumLegs, 1);
  EXPECT_FALSE(solver.solve(invalid_state, trajectory, contacts).valid);
}

}  // namespace
