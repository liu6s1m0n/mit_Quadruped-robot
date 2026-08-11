#include <gtest/gtest.h>

#include <cmath>

#include "model/robots/unitree_go1.hpp"

// 验证 Go1 参数能正确构建 6+12 自由度模型，并检查运动学、动力学矩阵和异常输入。
namespace
{

TEST(FloatingBaseModel, Go1FactoryUsesFactualGenericModel)
{
  const auto quadruped = robots::unitree_go1::makeModel<double>();
  auto model = robots::unitree_go1::makeFloatingBaseModel<double>();

  EXPECT_EQ(model.getNumDof(), 18U);
  EXPECT_EQ(model.getNumActuatedDof(), 12U);
  EXPECT_EQ(model.getNumGroundContacts(), 4U);
  EXPECT_EQ(model.getFootIndices().size(), 4U);
  EXPECT_NEAR(model.totalNonRotorMass(), quadruped.totalMass(), 1e-10);
  EXPECT_DOUBLE_EQ(model.totalRotorMass(), 0.0);
  ASSERT_EQ(model._jointArmatures.size(), 18U);
  for (std::size_t index = 6; index < model._jointArmatures.size(); ++index) {
    EXPECT_DOUBLE_EQ(model._jointArmatures[index], 0.01);
  }
}

TEST(FloatingBaseModel, Go1KinematicsAndDynamicsAreFinite)
{
  auto model = robots::unitree_go1::makeFloatingBaseModel<double>();
  FBModelState<double> state;
  state.bodyOrientation << 1.0, 0.0, 0.0, 0.0;
  state.bodyPosition << 0.0, 0.0, 0.27;
  state.bodyVelocity.setZero();
  state.q = DVec<double>::Zero(12);
  state.qd = DVec<double>::Zero(12);
  model.setState(state);

  model.forwardKinematics();
  ASSERT_EQ(model._pGC.size(), 4U);
  EXPECT_TRUE(model._pGC[0].isApprox(Vec3<double>(0.1881, -0.12675, -0.156), 1e-10));
  EXPECT_TRUE(model._pGC[1].isApprox(Vec3<double>(0.1881, 0.12675, -0.156), 1e-10));
  EXPECT_TRUE(model._pGC[2].isApprox(Vec3<double>(-0.1881, -0.12675, -0.156), 1e-10));
  EXPECT_TRUE(model._pGC[3].isApprox(Vec3<double>(-0.1881, 0.12675, -0.156), 1e-10));

  const DMat<double> mass_matrix = model.massMatrix();
  EXPECT_EQ(mass_matrix.rows(), 18);
  EXPECT_EQ(mass_matrix.cols(), 18);
  EXPECT_TRUE(mass_matrix.allFinite());
  EXPECT_TRUE(mass_matrix.isApprox(mass_matrix.transpose(), 1e-10));
  Eigen::SelfAdjointEigenSolver<DMat<double>> eigen_solver(mass_matrix);
  ASSERT_EQ(eigen_solver.info(), Eigen::Success);
  EXPECT_GT(eigen_solver.eigenvalues().minCoeff(), 0.0);

  EXPECT_TRUE(model.generalizedGravityForce().allFinite());
  EXPECT_TRUE(model.generalizedCoriolisForce().allFinite());
  model.contactJacobians();
  for (const auto & jacobian : model.getContactJacobians()) {
    EXPECT_EQ(jacobian.rows(), 3);
    EXPECT_EQ(jacobian.cols(), 18);
    EXPECT_TRUE(jacobian.allFinite());
  }

  FBModelStateDerivative<double> derivative;
  model.runABA(DVec<double>::Zero(12), derivative);
  EXPECT_TRUE(derivative.dBodyPosition.allFinite());
  EXPECT_TRUE(derivative.dBodyVelocity.allFinite());
  EXPECT_TRUE(derivative.qdd.allFinite());
}

TEST(FloatingBaseModel, RejectsWrongStateDimensions)
{
  auto model = robots::unitree_go1::makeFloatingBaseModel<double>();
  FBModelState<double> state;
  state.bodyOrientation << 1.0, 0.0, 0.0, 0.0;
  state.bodyPosition.setZero();
  state.bodyVelocity.setZero();
  state.q = DVec<double>::Zero(11);
  state.qd = DVec<double>::Zero(12);
  EXPECT_THROW(model.setState(state), std::invalid_argument);
}

TEST(FloatingBaseModel, ConvertsCurrentEstimatorAndLegFeedbackTypes)
{
  StateEstimate<double> estimate;
  estimate.valid = true;
  estimate.orientation_world_from_body =
    Eigen::Quaternion<double>(Eigen::AngleAxis<double>(0.2, Vec3<double>::UnitZ()));
  estimate.position_world << 1.0, 2.0, 0.3;
  estimate.velocity_body << 0.1, 0.2, 0.3;
  estimate.angular_velocity_body << -0.1, -0.2, -0.3;

  std::array<JointState<double>, kNumLegs> joints;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    joints[leg].valid = true;
    joints[leg].leg = static_cast<LegId>(leg);
    joints[leg].position.setConstant(static_cast<double>(leg));
    joints[leg].velocity.setConstant(-static_cast<double>(leg));
  }

  const auto state = model::makeFloatingBaseState(estimate, joints);
  EXPECT_TRUE(state.bodyPosition.isApprox(estimate.position_world));
  EXPECT_TRUE(state.bodyVelocity.head<3>().isApprox(estimate.angular_velocity_body));
  EXPECT_TRUE(state.bodyVelocity.tail<3>().isApprox(estimate.velocity_body));
  EXPECT_EQ(state.q.size(), 12);
  EXPECT_EQ(state.qd.size(), 12);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_TRUE(state.q.segment<3>(leg * kJointsPerLeg).isApprox(joints[leg].position));
    EXPECT_TRUE(state.qd.segment<3>(leg * kJointsPerLeg).isApprox(joints[leg].velocity));
  }
}

}  // namespace
