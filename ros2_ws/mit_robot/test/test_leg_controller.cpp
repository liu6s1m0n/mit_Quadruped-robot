#include <limits>

#include <gtest/gtest.h>

#include "controller/leg_controller.hpp"

namespace
{

constexpr float kTolerance = 1e-5F;

JointState<float> validState(LegId leg_id)
{
  JointState<float> result;
  result.leg = leg_id;
  result.position.setZero();
  result.velocity << 0.1F, 0.2F, 0.3F;
  result.torque_estimate << 1.0F, 2.0F, 3.0F;
  result.timestamp = 0.5F;
  result.valid = true;
  return result;
}

}  // namespace

TEST(LegControllerTest, ComputesGo1PositionAndJacobian)
{
  const auto quadruped = makeQuadruped<float>(RobotType::UNITREE_GO1);
  Mat3<float> jacobian;
  Vec3<float> position;
  const Vec3<float> q = Vec3<float>::Zero();

  computeLegJacobianAndPosition(
    quadruped, q, &jacobian, &position, LegId::FR);

  EXPECT_NEAR(position.x(), 0.0F, kTolerance);
  EXPECT_NEAR(position.y(), -0.08F, kTolerance);
  EXPECT_NEAR(position.z(), -0.426F, kTolerance);
  EXPECT_NEAR(jacobian(0, 1), 0.426F, kTolerance);
  EXPECT_NEAR(jacobian(0, 2), 0.213F, kTolerance);
  EXPECT_NEAR(jacobian(1, 0), 0.426F, kTolerance);
  EXPECT_NEAR(jacobian(2, 0), -0.08F, kTolerance);
}

TEST(LegControllerTest, UpdatesFeedbackFromJointState)
{
  const auto quadruped = makeQuadruped<float>(RobotType::UNITREE_GO1);
  LegController<float> controller(quadruped);
  const JointState<float> state = validState(LegId::FL);

  ASSERT_TRUE(controller.updateData(state));
  const auto & data = controller.datas[static_cast<std::size_t>(LegId::FL)];
  EXPECT_EQ(data.leg, LegId::FL);
  EXPECT_TRUE(data.q.isApprox(state.position));
  EXPECT_TRUE(data.qd.isApprox(state.velocity));
  EXPECT_TRUE(data.torque_estimate.isApprox(state.torque_estimate));
  EXPECT_TRUE(data.v.isApprox(data.J * state.velocity));
  EXPECT_FLOAT_EQ(data.timestamp, state.timestamp);
  EXPECT_TRUE(data.valid);
}

TEST(LegControllerTest, RejectsInvalidFeedback)
{
  const auto quadruped = makeQuadruped<float>(RobotType::UNITREE_GO1);
  LegController<float> controller(quadruped);
  JointState<float> state = validState(LegId::RR);
  state.position.x() = std::numeric_limits<float>::quiet_NaN();

  EXPECT_FALSE(controller.updateData(state));
  EXPECT_FALSE(controller.datas[static_cast<std::size_t>(LegId::RR)].valid);
}

TEST(LegControllerTest, GeneratesJointAndCartesianCommand)
{
  const auto quadruped = makeQuadruped<float>(RobotType::UNITREE_GO1);
  LegController<float> controller(quadruped);
  ASSERT_TRUE(controller.updateData(validState(LegId::FR)));

  auto & desired = controller.commands[static_cast<std::size_t>(LegId::FR)];
  desired.position_desired << 0.1F, 0.2F, 0.3F;
  desired.velocity_desired << 0.4F, 0.5F, 0.6F;
  desired.kp_joint << 10.0F, 20.0F, 30.0F;
  desired.kd_joint << 1.0F, 2.0F, 3.0F;
  desired.torque_feedforward.setConstant(1.0F);
  desired.force_feedforward << 0.0F, 0.0F, 2.0F;
  controller.setEnabled(true);

  const JointCommand<float> command = controller.command(LegId::FR, 0.6F);

  EXPECT_TRUE(command.enabled);
  EXPECT_TRUE(command.position_desired.isApprox(desired.position_desired));
  EXPECT_TRUE(command.velocity_desired.isApprox(desired.velocity_desired));
  EXPECT_TRUE(command.kp.isApprox(desired.kp_joint));
  EXPECT_TRUE(command.kd.isApprox(desired.kd_joint));
  const auto & data = controller.datas[static_cast<std::size_t>(LegId::FR)];
  const Vec3<float> expected_torque =
    desired.torque_feedforward + data.J.transpose() * desired.force_feedforward;
  EXPECT_TRUE(command.torque_feedforward.isApprox(expected_torque, kTolerance));
  EXPECT_FLOAT_EQ(command.timestamp, 0.6F);
  EXPECT_EQ(command.sequence, 1U);
}

TEST(LegControllerTest, EmergencyDampingUsesJointCommandInterface)
{
  const auto quadruped = makeQuadruped<float>(RobotType::UNITREE_GO1);
  LegController<float> controller(quadruped);
  ASSERT_TRUE(controller.updateData(validState(LegId::RL)));

  controller.edampCommand(RobotType::UNITREE_GO1, 4.0F);
  const JointCommand<float> command = controller.command(LegId::RL);

  EXPECT_TRUE(controller.legsEnabled());
  EXPECT_TRUE(command.enabled);
  EXPECT_TRUE(command.position_desired.isZero());
  EXPECT_TRUE(command.velocity_desired.isZero());
  EXPECT_TRUE(command.kp.isZero());
  EXPECT_TRUE(command.kd.isConstant(4.0F));
  EXPECT_TRUE(command.torque_feedforward.isZero());
}
