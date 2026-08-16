#include <gtest/gtest.h>

// 验证前跳状态的单次动作轨迹、关节限位保护以及动作结束后的回站逻辑。

#include "FSM/FSM_State_FrontJump.h"
#include "fsm_test_support.hpp"

namespace
{

TEST(FrontJumpStateTest, ProducesBoundedOneShotTrajectoryAndReturnsToStand)
{
  // 前跳应依次经过收腿/蹬伸阶段，并在有限控制周期内完成。
  test_support::FsmContext context(0.01F);
  ASSERT_TRUE(context.initialized);
  context.desired.mode = ControlMode::FrontJump;
  auto data = context.data();
  FSM_State_FrontJump<float> jump(&data);
  jump.onEnter();

  bool observed_crouch = false;
  bool observed_thrust = false;
  for (int cycle = 0; cycle < 90 && !jump.complete(); ++cycle) {
    jump.run();
    // 每个阶段生成的关节目标都必须保持有限且位于机器人模型限位内。
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const auto & command = context.leg_controller.commands[leg];
      const auto & limits = context.quadruped.leg(
        static_cast<LegId>(leg)).joints;
      EXPECT_TRUE(command.position_desired.allFinite());
      EXPECT_TRUE(command.velocity_desired.allFinite());
      EXPECT_TRUE((command.position_desired.array() >= limits.lower_limit.array()).all());
      EXPECT_TRUE((command.position_desired.array() <= limits.upper_limit.array()).all());
      observed_crouch = observed_crouch || command.position_desired.y() > 1.15F;
      observed_thrust = observed_thrust ||
        (command.position_desired.y() < 1.25F && command.position_desired.z() > -1.3F);
    }
  }

  EXPECT_TRUE(observed_crouch);
  EXPECT_TRUE(observed_thrust);
  EXPECT_TRUE(jump.complete());
  EXPECT_EQ(context.desired.mode, ControlMode::BalanceStand);
  EXPECT_EQ(jump.checkTransition(), FSM_StateName::BALANCE_STAND);
}

}  // namespace
