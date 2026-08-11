#include <gtest/gtest.h>

#include <array>
#include <cstdint>

#include "FSM/ControlFSM.h"
#include "FSM/FSM_State_Locomotion.h"
#include "FSM/FSM_State_RecoveryStand.h"
#include "FSM/SafetyChecker.h"
#include "controller/FootSwingTrajectory.hpp"
#include "model/robots/unitree_go1.hpp"

// 验证用户控制模式能映射到正确的 FSM 状态，且状态切换不改变既定状态机逻辑。
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
  GaitScheduler<float> gait_scheduler(0.01F);
  ControlFSM<float> fsm(
    quadruped, estimate, joints, leg_controller, gait_scheduler, desired, 0.01F);
  EXPECT_NO_THROW(fsm.setLocomotionForwardVelocity(0.3F));

  EXPECT_EQ(static_cast<std::uint8_t>(ControlMode::BalanceStand), 2);
  EXPECT_EQ(static_cast<std::uint8_t>(ControlMode::Locomotion), 3);
  EXPECT_EQ(static_cast<std::uint8_t>(ControlMode::StandUp), 4);
  EXPECT_EQ(static_cast<std::uint8_t>(ControlMode::RecoveryStand), 5);

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

  desired.mode = ControlMode::StandUp;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::STAND_UP);
  for (std::size_t iteration = 0; iteration <= 50; ++iteration) {
    fsm.runFSM();
  }
  desired.mode = ControlMode::BalanceStand;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::BALANCE_STAND);

  desired.mode = ControlMode::Passive;
  fsm.runFSM();
  desired.mode = ControlMode::RecoveryStand;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::RECOVERY_STAND);
}

TEST(LocomotionStateTest, LatchesSwingEndpointsUntilTouchdown)
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
  desired.mode = ControlMode::Locomotion;
  desired.body_position_world = estimate.position_world;
  desired.valid = true;
  GaitScheduler<float> gait_scheduler(0.05F);
  ControlFSMData<float> data;
  data.quadruped = &quadruped;
  data.state_estimate = &estimate;
  data.joint_states = &joints;
  data.leg_controller = &leg_controller;
  data.gait_scheduler = &gait_scheduler;
  data.desired_state = &desired;
  data.control_time_step = 0.05F;
  ASSERT_TRUE(data.valid());

  FSM_State_Locomotion<float> locomotion(&data);
  locomotion.setForwardVelocity(0.3F);
  locomotion.onEnter();
  locomotion.run();

  constexpr std::size_t kRearRight = static_cast<std::size_t>(LegId::RR);
  ASSERT_TRUE(locomotion.swingLegActive(LegId::RR));
  const Vec3<float> initial = estimate.position_world +
    quadruped.hipLocation(LegId::RR) + leg_controller.datas[kRearRight].p;
  EXPECT_TRUE(locomotion.footPositionTargetWorld(LegId::RR).isApprox(initial, 1.0e-5F));

  // 模拟摆动中实测后脚被额外抬高；锁存轨迹不应把这个扰动变成新目标。
  leg_controller.datas[kRearRight].p.z() += 0.04F;
  locomotion.run();

  FootSwingTrajectory<float> expected;
  expected.setInitialPosition(initial);
  Vec3<float> landing = initial;
  landing.x() += 0.3F * (0.25F + 0.25F);
  expected.setFinalPosition(landing);
  expected.setHeight(0.06F);
  expected.computeSwingTrajectoryBezier(0.2F, 0.25F);
  EXPECT_TRUE(
    locomotion.footPositionTargetWorld(LegId::RR).isApprox(
      expected.getPosition(), 1.0e-5F));
  EXPECT_GT(locomotion.footPositionTargetWorld(LegId::RR).z(), initial.z());

  // 推进到下一支撑沿，摆动态必须清除，后续离地才会重新锁存。
  for (std::size_t iteration = 0; iteration < 4; ++iteration) {
    locomotion.run();
  }
  EXPECT_FALSE(locomotion.swingLegActive(LegId::RR));
}

TEST(ControlFSMTest, RegisteredSafetyCheckerStopsUnsafeBalanceOrientation)
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
  desired.mode = ControlMode::BalanceStand;
  desired.body_position_world = estimate.position_world;
  desired.valid = true;
  GaitScheduler<float> gait_scheduler(0.001F);
  ControlFSM<float> fsm(
    quadruped, estimate, joints, leg_controller, gait_scheduler, desired);

  fsm.runFSM();
  ASSERT_EQ(fsm.currentStateName(), FSM_StateName::BALANCE_STAND);
  estimate.rpy.x() = 1.5F;
  fsm.runFSM();
  EXPECT_EQ(fsm.currentStateName(), FSM_StateName::PASSIVE);
  EXPECT_EQ(fsm.operatingMode(), FSM_OperatingMode::ESTOP);
  EXPECT_FALSE(leg_controller.legsEnabled());
}

TEST(RecoveryStandTest, ProducesRateIndependentCommandsWithinModelLimits)
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
  estimate.position_world.z() = 0.05F;
  estimate.valid = true;
  DesiredState<float> desired;
  desired.mode = ControlMode::Passive;
  desired.valid = true;
  GaitScheduler<float> gait_scheduler(0.01F);
  ControlFSMData<float> data;
  data.quadruped = &quadruped;
  data.state_estimate = &estimate;
  data.joint_states = &joints;
  data.leg_controller = &leg_controller;
  data.gait_scheduler = &gait_scheduler;
  data.desired_state = &desired;
  data.control_time_step = 0.01F;
  ASSERT_TRUE(data.valid());

  FSM_State_RecoveryStand<float> recovery(&data);
  recovery.onEnter();
  desired.mode = ControlMode::BalanceStand;
  EXPECT_EQ(recovery.checkTransition(), FSM_StateName::RECOVERY_STAND);
  // 0.8 s 的收腿阶段应随控制周期换算为 80 次，而不是沿用固定频率计数。
  for (std::size_t iteration = 0; iteration <= 80; ++iteration) {
    recovery.run();
  }

  EXPECT_TRUE(leg_controller.legsEnabled());
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & command = leg_controller.commands[leg];
    const auto & limits = quadruped.leg(static_cast<LegId>(leg)).joints;
    EXPECT_TRUE(command.position_desired.allFinite());
    EXPECT_TRUE((command.position_desired.array() >= limits.lower_limit.array()).all());
    EXPECT_TRUE((command.position_desired.array() <= limits.upper_limit.array()).all());
    EXPECT_NEAR(command.position_desired.y(), 1.4F, 1e-5F);
    EXPECT_NEAR(command.position_desired.z(), -2.7F, 1e-5F);
  }

  // 收腿稳定后，姿态正常且高度恢复时完成站起，再服从原有 BalanceStand 请求。
  estimate.position_world.z() = 0.27F;
  for (std::size_t iteration = 0; iteration < 140 + 51; ++iteration) {
    recovery.run();
  }
  EXPECT_EQ(recovery.checkTransition(), FSM_StateName::BALANCE_STAND);
}

TEST(SafetyCheckerTest, UsesCurrentModelAndCommandFields)
{
  const auto quadruped = robots::unitree_go1::makeModel<float>();
  LegController<float> leg_controller(quadruped);
  std::array<JointState<float>, kNumLegs> joints;
  StateEstimate<float> estimate;
  estimate.valid = true;
  DesiredState<float> desired;
  desired.valid = true;
  GaitScheduler<float> gait_scheduler(0.001F);
  ControlFSMData<float> data;
  data.quadruped = &quadruped;
  data.state_estimate = &estimate;
  data.joint_states = &joints;
  data.leg_controller = &leg_controller;
  data.gait_scheduler = &gait_scheduler;
  data.desired_state = &desired;
  ASSERT_TRUE(data.valid());

  SafetyChecker<float> checker(&data);
  estimate.rpy.x() = 1.5F;
  EXPECT_FALSE(checker.checkSafeOrientation());
  estimate.rpy.setZero();
  EXPECT_TRUE(checker.checkSafeOrientation());

  for (auto & command : leg_controller.commands) {
    command.foot_position_desired << 1.0F, -1.0F, 0.0F;
    command.force_feedforward.setConstant(1000.0F);
  }
  EXPECT_FALSE(checker.checkPDesFoot());
  EXPECT_FALSE(checker.checkForceFeedForward());
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & model = quadruped.leg(static_cast<LegId>(leg));
    const auto & command = leg_controller.commands[leg];
    EXPECT_LE(
      command.foot_position_desired.head<2>().cwiseAbs().maxCoeff(),
      model.maximumLegLength());
    EXPECT_LE(command.foot_position_desired.z(), -model.maximumLegLength() / 4.0F);
    EXPECT_TRUE(command.force_feedforward.allFinite());
    EXPECT_LT(command.force_feedforward.cwiseAbs().maxCoeff(), 1000.0F);
  }
}

}  // namespace
