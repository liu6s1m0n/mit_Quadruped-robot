#include <array>
#include <cstddef>

#include <eigen3/Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include "model/quadruped.hpp"
#include "model/robot_control_parameters.hpp"

// float 精度下机器精度约为 1e-7，容差需放宽到 1e-4 才能稳定通过。
constexpr float kTolerance = 1e-4f;

void expectMirroredAcrossBodyXzPlane(
  const Vec3<float> & right, const Vec3<float> & left)
{
  EXPECT_NEAR(right.x(), left.x(), kTolerance);
  EXPECT_NEAR(right.y(), -left.y(), kTolerance);
  EXPECT_NEAR(right.z(), left.z(), kTolerance);
}

void expectSymmetricPositiveDefinite(const RigidBodyInertia<float> & inertia)
{
  EXPECT_TRUE(
    inertia.inertia_com.isApprox(
      inertia.inertia_com.transpose(), kTolerance));

  Eigen::SelfAdjointEigenSolver<Mat3<float>> solver(inertia.inertia_com);
  ASSERT_EQ(solver.info(), Eigen::Success);
  EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

TEST(Go1QuadrupedTest, HasExpectedTotalMass)
{
  const auto model = makeQuadruped<float>(RobotType::UNITREE_GO1);

  EXPECT_NEAR(model.totalMass(), 12.743448, kTolerance);
}

TEST(Go1QuadrupedTest, StoresLegsInControllerOrder)
{
  const auto model = makeQuadruped<float>(RobotType::UNITREE_GO1);
  constexpr std::array<LegId, kNumLegs> expected_order{
    LegId::FR, LegId::FL, LegId::RR, LegId::RL};

  for (std::size_t index = 0; index < expected_order.size(); ++index) {
    SCOPED_TRACE(index);
    EXPECT_EQ(model.legs()[index].leg, expected_order[index]);
    EXPECT_EQ(&model.legs()[index], &model.leg(expected_order[index]));
  }
}

TEST(Go1QuadrupedTest, HasLeftRightMirrorSymmetry)
{
  const auto model = makeQuadruped<float>(RobotType::UNITREE_GO1);
  const auto & front_right = model.leg(LegId::FR);
  const auto & front_left = model.leg(LegId::FL);
  const auto & rear_right = model.leg(LegId::RR);
  const auto & rear_left = model.leg(LegId::RL);

  expectMirroredAcrossBodyXzPlane(
    front_right.hip_location_body, front_left.hip_location_body);
  expectMirroredAcrossBodyXzPlane(
    rear_right.hip_location_body, rear_left.hip_location_body);
  expectMirroredAcrossBodyXzPlane(
    front_right.hip_inertia.center_of_mass,
    front_left.hip_inertia.center_of_mass);
  expectMirroredAcrossBodyXzPlane(
    rear_right.hip_inertia.center_of_mass,
    rear_left.hip_inertia.center_of_mass);
  expectMirroredAcrossBodyXzPlane(
    front_right.thigh_inertia.center_of_mass,
    front_left.thigh_inertia.center_of_mass);
  expectMirroredAcrossBodyXzPlane(
    rear_right.thigh_inertia.center_of_mass,
    rear_left.thigh_inertia.center_of_mass);

  EXPECT_FLOAT_EQ(model.sideSign(LegId::FR), -1.0);
  EXPECT_FLOAT_EQ(model.sideSign(LegId::RR), -1.0);
  EXPECT_FLOAT_EQ(model.sideSign(LegId::FL), 1.0);
  EXPECT_FLOAT_EQ(model.sideSign(LegId::RL), 1.0);
}

TEST(Go1QuadrupedTest, UsesExpectedJointAndActuatorParameters)
{
  const auto model = makeQuadruped<float>(RobotType::UNITREE_GO1);
  Vec3<float> expected_lower;
  Vec3<float> expected_upper;
  Vec3<float> expected_velocity;
  Vec3<float> expected_torque;
  expected_lower << -0.863, -0.686, -2.818;
  expected_upper << 0.863, 4.501, -0.888;
  expected_velocity << 30.1, 30.1, 20.06;
  expected_torque << 23.7, 23.7, 35.55;

  for (const auto & leg : model.legs()) {
    SCOPED_TRACE(static_cast<unsigned int>(leg.leg));
    EXPECT_TRUE(leg.joints.lower_limit.isApprox(expected_lower, kTolerance));
    EXPECT_TRUE(leg.joints.upper_limit.isApprox(expected_upper, kTolerance));
    EXPECT_TRUE(leg.joints.velocity_limit.isApprox(expected_velocity, kTolerance));
    EXPECT_TRUE(leg.joints.torque_limit.isApprox(expected_torque, kTolerance));
    EXPECT_TRUE(leg.joints.damping.isApprox(Vec3<float>(1.0, 2.0, 2.0), kTolerance));
    EXPECT_TRUE(
      leg.joints.friction_loss.isApprox(Vec3<float>::Constant(0.2), kTolerance));
    EXPECT_TRUE(leg.joints.armature.isApprox(Vec3<float>::Constant(0.01), kTolerance));
    EXPECT_TRUE(
      (leg.joints.home_position.array() >=
      leg.joints.lower_limit.array()).all());
    EXPECT_TRUE(
      (leg.joints.home_position.array() <=
      leg.joints.upper_limit.array()).all());
  }
}

TEST(Go1QuadrupedTest, HasSymmetricPositiveDefiniteInertiaMatrices)
{
  const auto model = makeQuadruped<float>(RobotType::UNITREE_GO1);

  SCOPED_TRACE("body");
  expectSymmetricPositiveDefinite(model.bodyInertia());

  for (const auto & leg : model.legs()) {
    SCOPED_TRACE(static_cast<unsigned int>(leg.leg));
    expectSymmetricPositiveDefinite(leg.hip_inertia);
    expectSymmetricPositiveDefinite(leg.thigh_inertia);
    expectSymmetricPositiveDefinite(leg.calf_inertia);
  }
}

TEST(Dm1QuadrupedTest, FactoryProvidesACompleteIndependentModel)
{
  const auto model = makeQuadruped<float>(RobotType::DM_BOT1);

  EXPECT_EQ(model.robotType(), RobotType::DM_BOT1);
  EXPECT_NEAR(model.nominalBodyHeight(), 0.39F, kTolerance);
  EXPECT_NEAR(model.totalMass(), 14.705035F, kTolerance);
  EXPECT_TRUE(model.isValid());
  for (const auto & leg : model.legs()) {
    EXPECT_TRUE((leg.joints.home_position.array() <=
      leg.joints.upper_limit.array()).all());
    EXPECT_TRUE((leg.joints.home_position.array() >=
      leg.joints.lower_limit.array()).all());
  }
  EXPECT_NEAR(model.leg(LegId::FR).joints.home_position.y(), -0.597F, kTolerance);
  EXPECT_NEAR(model.leg(LegId::RR).joints.home_position.y(), -0.597F, kTolerance);
  EXPECT_NEAR(model.leg(LegId::FR).joints.home_position.z(), 1.432F, kTolerance);
  EXPECT_NEAR(model.leg(LegId::FL).joints.home_position.z(), 1.432F, kTolerance);
  EXPECT_NEAR(model.leg(LegId::RR).joints.home_position.z(), 1.468F, kTolerance);
  EXPECT_NEAR(model.leg(LegId::RL).joints.home_position.z(), 1.468F, kTolerance);
  EXPECT_TRUE(model.leg(LegId::FR).joints.zero_offset.isApprox(
    Vec3<float>(0.0F, -0.203F, -2.25F)));
  EXPECT_TRUE(model.leg(LegId::RR).joints.zero_offset.isApprox(
    Vec3<float>(0.0F, -0.203F, -2.25F)));
}

TEST(RobotControlParametersTest, KeepsGo1AndDm1ProfilesIndependent)
{
  const auto go1 = makeRobotControlParameters<float>(RobotType::UNITREE_GO1);
  const auto dm1 = makeRobotControlParameters<float>(RobotType::DM_BOT1);

  EXPECT_FLOAT_EQ(go1.maximum_standing_height, 0.34F);
  EXPECT_FLOAT_EQ(dm1.maximum_standing_height, 0.42F);
  EXPECT_FLOAT_EQ(go1.joint_initialization_duration, 0.4F);
  EXPECT_FLOAT_EQ(dm1.joint_initialization_duration, 1.2F);
  EXPECT_FLOAT_EQ(go1.maximum_normal_force, 1500.0F);
  EXPECT_FLOAT_EQ(dm1.maximum_normal_force, 150.0F);
  EXPECT_FLOAT_EQ(go1.locomotion_swing_height, 0.09F);
  EXPECT_FLOAT_EQ(dm1.locomotion_swing_height, 0.075F);
  EXPECT_FLOAT_EQ(go1.locomotion_max_lateral_foot_offset, 0.18F);
  EXPECT_FLOAT_EQ(dm1.locomotion_max_lateral_foot_offset, 0.24F);
  EXPECT_FALSE(go1.start_in_prone_home);
  EXPECT_TRUE(dm1.start_in_prone_home);
  EXPECT_TRUE(dm1.motor_zero_position.isZero());
  EXPECT_TRUE(go1.use_go1_height_joint_mapping);
  EXPECT_FALSE(dm1.use_go1_height_joint_mapping);
  EXPECT_FALSE(go1.initialization_kp.isApprox(dm1.initialization_kp));
  EXPECT_FALSE(
    go1.balance_body_position_kd.isApprox(dm1.balance_body_position_kd));
}
