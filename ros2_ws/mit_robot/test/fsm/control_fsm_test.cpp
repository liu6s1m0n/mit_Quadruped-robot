#include <gtest/gtest.h>

#include <cstdint>

#include "FSM/ControlFSM.h"
#include "fsm_test_support.hpp"

namespace
{

TEST(ControlFSMTest, TransitionsBetweenProjectControlModes)
{
  test_support::FsmContext context(0.01F);
  ASSERT_TRUE(context.initialized);
  ControlFSM<float> fsm(
    context.quadruped, context.estimate, context.joints, context.leg_controller,
    context.gait_scheduler, context.desired, context.control_time_step);
  EXPECT_NO_THROW(fsm.setLocomotionForwardVelocity(0.3F));
  EXPECT_NO_THROW(fsm.setLocomotionVelocityCommand(0.0F, 0.2F, 0.5F));

  EXPECT_EQ(static_cast<std::uint8_t>(ControlMode::BalanceStand), 2);
  EXPECT_EQ(static_cast<std::uint8_t>(ControlMode::Locomotion), 3);
  EXPECT_EQ(static_cast<std::uint8_t>(ControlMode::StandUp), 4);
  EXPECT_EQ(static_cast<std::uint8_t>(ControlMode::RecoveryStand), 5);

  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::PASSIVE);
  context.desired.mode = ControlMode::JointPd;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::JOINT_PD);

  context.desired.mode = ControlMode::BalanceStand;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::BALANCE_STAND);

  context.desired.mode = ControlMode::Locomotion;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::LOCOMOTION);

  context.desired.mode = ControlMode::Passive;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::PASSIVE);

  context.desired.mode = ControlMode::StandUp;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::STAND_UP);
  for (std::size_t iteration = 0; iteration <= 50; ++iteration) {
    fsm.runFSM();
  }
  context.desired.mode = ControlMode::BalanceStand;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::BALANCE_STAND);

  context.desired.mode = ControlMode::Passive;
  fsm.runFSM();
  context.desired.mode = ControlMode::RecoveryStand;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::RECOVERY_STAND);
}

TEST(ControlFSMTest, RegisteredSafetyCheckerStopsUnsafeBalanceOrientation)
{
  test_support::FsmContext context(0.001F);
  ASSERT_TRUE(context.initialized);
  context.desired.mode = ControlMode::BalanceStand;
  ControlFSM<float> fsm(
    context.quadruped, context.estimate, context.joints, context.leg_controller,
    context.gait_scheduler, context.desired);

  fsm.runFSM();
  ASSERT_EQ(fsm.currentStateName(), FSM_StateName::BALANCE_STAND);
  context.estimate.rpy.x() = 1.5F;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::PASSIVE);
  EXPECT_EQ(fsm.operatingMode(), FSM_OperatingMode::ESTOP);
  EXPECT_FALSE(context.leg_controller.legsEnabled());
}

}  // namespace
