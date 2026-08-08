#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "controller/GaitScheduler.hpp"

namespace
{

constexpr float kTolerance = 1e-5F;

}  // namespace

TEST(GaitSchedulerTest, InitializesAllLegsInStandingContact)
{
  GaitScheduler<float> scheduler(0.002F);

  EXPECT_EQ(scheduler.gait_data.current_gait, GaitType::STAND);
  EXPECT_EQ(scheduler.gait_data.gait_name, "STAND");
  scheduler.step();

  for (std::size_t index = 0; index < kNumLegs; ++index) {
    const LegId leg = static_cast<LegId>(index);
    EXPECT_TRUE(scheduler.gait_data.contactScheduled(leg));
    EXPECT_EQ(
      scheduler.gait_data.touchdown_scheduled(
        static_cast<Eigen::Index>(index)), 1);
  }

  scheduler.step();
  EXPECT_TRUE(scheduler.gait_data.touchdown_scheduled.isZero());
  EXPECT_TRUE(scheduler.gait_data.liftoff_scheduled.isZero());
}

TEST(GaitSchedulerTest, PreservesTrotDiagonalPhasePattern)
{
  GaitScheduler<float> scheduler(0.001F);
  scheduler.requestGait(GaitType::TROT);
  scheduler.step();

  EXPECT_EQ(scheduler.gait_data.current_gait, GaitType::TROT);
  EXPECT_NEAR(scheduler.gait_data.period_time_nominal, 0.5F, kTolerance);
  EXPECT_NEAR(scheduler.gait_data.switching_phase_nominal, 0.5F, kTolerance);

  // 本工程腿序为 FR、FL、RR、RL；对角腿同相。
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(0), 1);
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(1), 0);
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(2), 0);
  EXPECT_EQ(scheduler.gait_data.contact_state_scheduled(3), 1);
  EXPECT_NEAR(
    scheduler.gait_data.phase_variable(0),
    scheduler.gait_data.phase_variable(3), kTolerance);
  EXPECT_NEAR(
    scheduler.gait_data.phase_variable(1),
    scheduler.gait_data.phase_variable(2), kTolerance);

  const auto probabilities =
    scheduler.gait_data.scheduledContactProbabilities();
  EXPECT_FLOAT_EQ(probabilities[static_cast<std::size_t>(LegId::FR)], 1.0F);
  EXPECT_FLOAT_EQ(probabilities[static_cast<std::size_t>(LegId::FL)], 0.0F);
  EXPECT_FLOAT_EQ(probabilities[static_cast<std::size_t>(LegId::RR)], 0.0F);
  EXPECT_FLOAT_EQ(probabilities[static_cast<std::size_t>(LegId::RL)], 1.0F);
}

TEST(GaitSchedulerTest, AppliesRuntimeTimingOverrideToOverrideableGait)
{
  GaitSchedulerParameters<float> parameters;
  parameters.override_mode = GaitOverrideMode::OVERRIDE_TIMING;
  parameters.gait_type = GaitType::TROT;
  parameters.gait_period_time = 0.8F;
  parameters.gait_switching_phase = 0.6F;
  GaitScheduler<float> scheduler(parameters, 0.002F);

  scheduler.step();

  EXPECT_EQ(scheduler.gait_data.current_gait, GaitType::TROT);
  EXPECT_NEAR(scheduler.gait_data.period_time_nominal, 0.8F, kTolerance);
  EXPECT_NEAR(scheduler.gait_data.switching_phase_nominal, 0.6F, kTolerance);
  EXPECT_TRUE(
    scheduler.gait_data.time_stance.isApprox(
      Vec4<float>::Constant(0.48F), kTolerance));
  EXPECT_TRUE(
    scheduler.gait_data.time_swing.isApprox(
      Vec4<float>::Constant(0.32F), kTolerance));
}

TEST(GaitSchedulerTest, ThreeFootGaitKeepsDisabledLegFinite)
{
  GaitScheduler<float> scheduler(0.002F);
  scheduler.requestGait(GaitType::THREE_FOOT);
  scheduler.step();

  EXPECT_EQ(scheduler.gait_data.gait_enabled(0), 0);
  EXPECT_FLOAT_EQ(scheduler.gait_data.period_time(0), 0.0F);
  EXPECT_FLOAT_EQ(scheduler.gait_data.time_stance(0), 0.0F);
  EXPECT_FLOAT_EQ(scheduler.gait_data.time_swing(0), 0.0F);
  EXPECT_TRUE(scheduler.gait_data.period_time.allFinite());
  EXPECT_TRUE(scheduler.gait_data.time_stance.allFinite());
  EXPECT_TRUE(scheduler.gait_data.time_swing.allFinite());
}

TEST(GaitSchedulerTest, RejectsInvalidParametersAndTimeStep)
{
  EXPECT_THROW(GaitScheduler<float>(0.0F), std::invalid_argument);

  GaitSchedulerParameters<float> parameters;
  parameters.gait_period_time =
    std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(GaitScheduler<float>(parameters, 0.002F), std::invalid_argument);
}
