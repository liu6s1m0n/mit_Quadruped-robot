#include <gtest/gtest.h>

#include "FSM/SafetyChecker.h"
#include "fsm_test_support.hpp"

namespace
{

TEST(SafetyCheckerTest, UsesCurrentModelAndCommandFields)
{
  test_support::FsmContext context(0.001F);
  ASSERT_TRUE(context.initialized);
  auto data = context.data();
  ASSERT_TRUE(data.valid());

  SafetyChecker<float> checker(&data);
  context.estimate.rpy.x() = 1.5F;
  EXPECT_FALSE(checker.checkSafeOrientation());
  context.estimate.rpy.setZero();
  EXPECT_TRUE(checker.checkSafeOrientation());

  for (auto & command : context.leg_controller.commands) {
    command.foot_position_desired << 1.0F, -1.0F, 0.0F;
    command.force_feedforward.setConstant(1000.0F);
  }
  EXPECT_FALSE(checker.checkPDesFoot());
  EXPECT_FALSE(checker.checkForceFeedForward());
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & model = context.quadruped.leg(static_cast<LegId>(leg));
    const auto & command = context.leg_controller.commands[leg];
    EXPECT_LE(
      command.foot_position_desired.head<2>().cwiseAbs().maxCoeff(),
      model.maximumLegLength());
    EXPECT_LE(command.foot_position_desired.z(), -model.maximumLegLength() / 4.0F);
    EXPECT_TRUE(command.force_feedforward.allFinite());
    EXPECT_LT(command.force_feedforward.cwiseAbs().maxCoeff(), 1000.0F);
  }
}

}  // namespace
