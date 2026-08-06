#include <stdexcept>

#include <gtest/gtest.h>

#include "controller/FootSwingTrajectory.hpp"

namespace
{

constexpr float kTolerance = 1e-5F;

FootSwingTrajectory<float> makeTrajectory()
{
  FootSwingTrajectory<float> trajectory;
  trajectory.setInitialPosition(Vec3<float>(0.0F, 0.1F, -0.3F));
  trajectory.setFinalPosition(Vec3<float>(0.2F, -0.1F, -0.25F));
  trajectory.setHeight(0.1F);
  return trajectory;
}

}  // namespace

TEST(FootSwingTrajectoryTest, StartsAndEndsAtConfiguredPositions)
{
  auto trajectory = makeTrajectory();

  trajectory.computeSwingTrajectoryBezier(0.0F, 0.4F);
  EXPECT_TRUE(
    trajectory.getPosition().isApprox(
      Vec3<float>(0.0F, 0.1F, -0.3F), kTolerance));
  EXPECT_TRUE(trajectory.getVelocity().isZero(kTolerance));

  trajectory.computeSwingTrajectoryBezier(1.0F, 0.4F);
  EXPECT_TRUE(
    trajectory.getPosition().isApprox(
      Vec3<float>(0.2F, -0.1F, -0.25F), kTolerance));
  EXPECT_TRUE(trajectory.getVelocity().isZero(kTolerance));
}

TEST(FootSwingTrajectoryTest, ReachesConfiguredHeightAtMidSwing)
{
  auto trajectory = makeTrajectory();

  trajectory.computeSwingTrajectoryBezier(0.5F, 0.4F);

  EXPECT_NEAR(trajectory.getPosition().x(), 0.1F, kTolerance);
  EXPECT_NEAR(trajectory.getPosition().y(), 0.0F, kTolerance);
  EXPECT_NEAR(trajectory.getPosition().z(), -0.2F, kTolerance);
  EXPECT_NEAR(trajectory.getVelocity().z(), 0.0F, kTolerance);
}

TEST(FootSwingTrajectoryTest, ClampsPhaseToValidRange)
{
  auto trajectory = makeTrajectory();

  trajectory.computeSwingTrajectoryBezier(-0.1F, 0.4F);
  EXPECT_TRUE(
    trajectory.getPosition().isApprox(
      Vec3<float>(0.0F, 0.1F, -0.3F), kTolerance));

  trajectory.computeSwingTrajectoryBezier(1.1F, 0.4F);
  EXPECT_TRUE(
    trajectory.getPosition().isApprox(
      Vec3<float>(0.2F, -0.1F, -0.25F), kTolerance));
}

TEST(FootSwingTrajectoryTest, RejectsInvalidTimeAndHeight)
{
  auto trajectory = makeTrajectory();

  EXPECT_THROW(
    trajectory.computeSwingTrajectoryBezier(0.5F, 0.0F),
    std::invalid_argument);

  trajectory.setHeight(-0.1F);
  EXPECT_THROW(
    trajectory.computeSwingTrajectoryBezier(0.5F, 0.4F),
    std::invalid_argument);
}
