#include <gtest/gtest.h>

#include <array>

#include "FSM/ControlFSM.h"
#include "model/robots/unitree_go1.hpp"

namespace
{

TEST(ControlFSMTest, TransitionsBetweenProjectControlModes)
{
  const auto quadruped = robots::unitree_go1::makeModel<float>();
  LegController<float> leg_controller(quadruped);
  std::array<JointState<float>, kNumLegs> joints;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    joints[leg].leg = static_cast<LegId>(leg);
    joints[leg].position = quadruped.leg(joints[leg].leg).joints.home_position;
    joints[leg].valid = true;
    ASSERT_TRUE(leg_controller.updateData(joints[leg]));
  }

  StateEstimate<float> estimate;
  estimate.position_world.z() = 0.27F;
  estimate.valid = true;
  DesiredState<float> desired;
  desired.body_position_world = estimate.position_world;
  desired.valid = true;
  GaitScheduler<float> gait_scheduler(0.001F);
  ControlFSM<float> fsm(
    quadruped, estimate, joints, leg_controller, gait_scheduler, desired);

  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::PASSIVE);
  desired.mode = ControlMode::JointPd;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::JOINT_PD);

  desired.mode = ControlMode::BalanceStand;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::BALANCE_STAND);

  desired.mode = ControlMode::Locomotion;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::LOCOMOTION);

  desired.mode = ControlMode::Passive;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::PASSIVE);
}

}  // namespace
