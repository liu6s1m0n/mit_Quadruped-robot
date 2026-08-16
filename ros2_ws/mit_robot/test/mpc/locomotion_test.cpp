#include <gtest/gtest.h>

#include "MPC/ConvexMPCLocomotion.h"
#include "model/robots/unitree_go1.hpp"
#include "mpc_test_support.hpp"

namespace
{

TEST(MpcLocomotion, RunsWithCurrentStateAndDesiredStateTypes)
{
  const auto quadruped = robots::unitree_go1::makeModel<double>();
  mpc::SolverSettings<double> settings;
  settings.horizon = 4;
  mpc::ConvexMPCLocomotion<double> controller(quadruped, 0.002, 15, settings);
  controller.setGait(GaitType::STAND);
  controller.setForwardVelocity(0.3);

  const auto result = controller.run(
    test_support::standingEstimate(), test_support::standingDesired(),
    test_support::standingFeet());
  EXPECT_TRUE(result.valid);
  EXPECT_NEAR(result.command.body_velocity_world.x(), 0.0015, 1.0e-12);
  EXPECT_NEAR(result.command.body_velocity_world.y(), 0.0, 1.0e-12);
  EXPECT_NEAR(result.command.body_position_world.x(), 0.0, 1.0e-12);
  for (double phase : result.contact_phase) {
    EXPECT_GE(phase, 0.0);
  }
}

TEST(MpcLocomotion, PublishesUnambiguousContactStateAndGaitTiming)
{
  const auto quadruped = robots::unitree_go1::makeModel<double>();
  mpc::ConvexMPCLocomotion<double> controller(quadruped, 0.002, 15);
  controller.setGait(GaitType::TROT);

  const auto result = controller.run(
    test_support::standingEstimate(), test_support::standingDesired(),
    test_support::standingFeet());
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.contact_phase[static_cast<std::size_t>(LegId::FR)], 0.0);
  EXPECT_TRUE(result.contact_state[static_cast<std::size_t>(LegId::FR)]);
  EXPECT_TRUE(result.contact_state[static_cast<std::size_t>(LegId::FL)]);
  EXPECT_TRUE(result.contact_state[static_cast<std::size_t>(LegId::RR)]);
  EXPECT_TRUE(result.contact_state[static_cast<std::size_t>(LegId::RL)]);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_NEAR(result.stance_time[leg], 0.18, 1.0e-6);
    EXPECT_NEAR(result.swing_time[leg], 0.12, 1.0e-6);
  }
}

TEST(MpcLocomotion, SupportsBodyFrameLateralMotionAndYawRotation)
{
  const auto quadruped = robots::unitree_go1::makeModel<double>();
  mpc::SolverSettings<double> settings;
  settings.horizon = 4;
  mpc::ConvexMPCLocomotion<double> controller(quadruped, 0.002, 15, settings);
  controller.setGait(GaitType::STAND);
  controller.setVelocityCommand(0.0, 0.3, 0.8);

  const auto result = controller.run(
    test_support::standingEstimate(), test_support::standingDesired(),
    test_support::standingFeet());
  ASSERT_TRUE(result.valid);
  // 线加速度 0.75 m/s^2、角加速度 0.8 rad/s^2 在首个 2 ms 周期的斜坡结果。
  EXPECT_NEAR(result.command.body_velocity_world.x(), 0.0, 1.0e-7);
  EXPECT_NEAR(result.command.body_velocity_world.y(), 0.0015, 1.0e-7);
  EXPECT_NEAR(result.command.body_angular_velocity.z(), 0.0016, 1.0e-12);
  EXPECT_GT(result.command.body_rpy.z(), 0.0);
}

TEST(MpcLocomotion, SolvesAtConfiguredIntervalAndReusesForces)
{
  const auto quadruped = robots::unitree_go1::makeModel<double>();
  mpc::SolverSettings<double> settings;
  settings.horizon = 4;
  mpc::ConvexMPCLocomotion<double> controller(quadruped, 0.002, 3, settings);
  controller.setGait(GaitType::STAND);

  const auto first = controller.run(
    test_support::standingEstimate(), test_support::standingDesired(),
    test_support::standingFeet());
  ASSERT_TRUE(first.valid);
  EXPECT_TRUE(first.mpc_updated);

  for (int cycle = 1; cycle < 3; ++cycle) {
    const auto cached = controller.run(
      test_support::standingEstimate(), test_support::standingDesired(),
      test_support::standingFeet());
    ASSERT_TRUE(cached.valid);
    EXPECT_FALSE(cached.mpc_updated);
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      EXPECT_TRUE(
        cached.reaction_forces_world[leg].isApprox(
          first.reaction_forces_world[leg], 1.0e-12));
    }
  }

  const auto refreshed = controller.run(
    test_support::standingEstimate(), test_support::standingDesired(),
    test_support::standingFeet());
  EXPECT_TRUE(refreshed.valid);
  EXPECT_TRUE(refreshed.mpc_updated);
}

TEST(MpcLocomotion, KeepsGaitTimingIndependentAndRefreshesAtContactChanges)
{
  const auto quadruped = robots::unitree_go1::makeModel<double>();
  mpc::SolverSettings<double> settings;
  settings.horizon = 10;
  mpc::ConvexMPCLocomotion<double> controller(
    quadruped, 0.002, 20, settings, 25);
  controller.setGait(GaitType::TROT);

  auto result = controller.run(
    test_support::standingEstimate(), test_support::standingDesired(),
    test_support::standingFeet());
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.mpc_updated);
  EXPECT_NEAR(result.stance_time[0], 0.30, 1.0e-6);
  EXPECT_NEAR(result.swing_time[0], 0.20, 1.0e-6);

  for (int cycle = 1; cycle < 25; ++cycle) {
    result = controller.run(
      test_support::standingEstimate(), test_support::standingDesired(),
      test_support::standingFeet());
  }
  result = controller.run(
    test_support::standingEstimate(), test_support::standingDesired(),
    test_support::standingFeet());
  EXPECT_TRUE(result.mpc_updated);
}

TEST(MpcLocomotion, RejectsUnsafeForwardVelocity)
{
  const auto quadruped = robots::unitree_go1::makeModel<double>();
  mpc::ConvexMPCLocomotion<double> controller(quadruped, 0.002);
  EXPECT_THROW(controller.setForwardVelocity(1.01), std::invalid_argument);
  EXPECT_THROW(controller.setVelocityCommand(0.8, 0.8, 0.0), std::invalid_argument);
  EXPECT_THROW(controller.setVelocityCommand(0.0, 0.0, 2.01), std::invalid_argument);
}

}  // namespace
