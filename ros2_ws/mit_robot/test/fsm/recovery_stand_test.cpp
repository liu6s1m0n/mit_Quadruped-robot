#include <gtest/gtest.h>

#include "FSM/FSM_State_RecoveryStand.h"
#include "fsm_test_support.hpp"

namespace
{

TEST(RecoveryStandTest, ProducesRateIndependentCommandsWithinModelLimits)
{
  test_support::FsmContext context(0.01F);
  ASSERT_TRUE(context.initialized);
  context.estimate.position_world.z() = 0.05F;
  context.desired.mode = ControlMode::Passive;
  auto data = context.data();
  ASSERT_TRUE(data.valid());

  FSM_State_RecoveryStand<float> recovery(&data);
  recovery.onEnter();
  context.desired.mode = ControlMode::BalanceStand;
  EXPECT_EQ(recovery.checkTransition(), FSM_StateName::RECOVERY_STAND);
  for (std::size_t iteration = 0; iteration <= 80; ++iteration) {
    recovery.run();
  }

  EXPECT_TRUE(context.leg_controller.legsEnabled());
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & command = context.leg_controller.commands[leg];
    const auto & limits = context.quadruped.leg(static_cast<LegId>(leg)).joints;
    EXPECT_TRUE(command.position_desired.allFinite());
    EXPECT_TRUE((command.position_desired.array() >= limits.lower_limit.array()).all());
    EXPECT_TRUE((command.position_desired.array() <= limits.upper_limit.array()).all());
    EXPECT_NEAR(command.position_desired.y(), 1.4F, 1e-5F);
    EXPECT_NEAR(command.position_desired.z(), -2.7F, 1e-5F);
  }

  context.estimate.position_world.z() = 0.27F;
  for (std::size_t iteration = 0; iteration < 140 + 51; ++iteration) {
    recovery.run();
  }
  EXPECT_EQ(recovery.checkTransition(), FSM_StateName::BALANCE_STAND);
}

}  // namespace
