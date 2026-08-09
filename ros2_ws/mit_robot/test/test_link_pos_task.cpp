#include <gtest/gtest.h>

#include "WBC/WBC_Ctrl/LinkPosTask.hpp"
#include "model/robots/unitree_go1.hpp"

namespace
{

FBModelState<double> nominalState()
{
  FBModelState<double> state;
  state.bodyOrientation << 1.0, 0.0, 0.0, 0.0;
  state.bodyPosition << 0.0, 0.0, 0.27;
  state.bodyVelocity.setZero();
  state.q = DVec<double>::Zero(kNumJoints);
  state.qd = DVec<double>::Zero(kNumJoints);
  return state;
}

TEST(LinkPosTask, TracksRegisteredPointInWorldCoordinates)
{
  auto model = robots::unitree_go1::makeFloatingBaseModel<double>();
  model.setState(nominalState());
  model.contactJacobians();
  ASSERT_GT(model.getNumGroundContacts(), 0U);
  LinkPosTask<double> task(model, 0);
  task.setProportionalGain(Vec3<double>::Constant(10.0));

  const Vec3<double> current = model.getGroundContactPositions().at(0);
  const Vec3<double> desired = current + Vec3<double>(0.01, -0.02, 0.03);
  ASSERT_TRUE(task.update(desired));

  DVec<double> command;
  DVec<double> error;
  DMat<double> jacobian;
  task.getCommand(command);
  error = task.getPosError();
  task.getTaskJacobian(jacobian);
  EXPECT_TRUE(command.isApprox(Vec3<double>(0.1, -0.2, 0.3), 1e-10));
  EXPECT_TRUE(error.isApprox(desired - current, 1e-12));
  EXPECT_EQ(jacobian.rows(), 3);
  EXPECT_EQ(jacobian.cols(), 18);
  EXPECT_TRUE(jacobian.allFinite());
}

TEST(LinkPosTask, CanRemoveFloatingBaseDependence)
{
  auto model = robots::unitree_go1::makeFloatingBaseModel<double>();
  model.setState(nominalState());
  model.contactJacobians();
  LinkPosTask<double> task(model, 0, false);
  ASSERT_TRUE(task.update(model.getGroundContactPositions().at(0)));

  DMat<double> jacobian;
  task.getTaskJacobian(jacobian);
  EXPECT_TRUE(jacobian.leftCols(6).isZero(1e-12));
}

TEST(LinkPosTask, ValidatesIndexAndGain)
{
  auto model = robots::unitree_go1::makeFloatingBaseModel<double>();
  EXPECT_THROW(
    LinkPosTask<double>(model, model.getNumGroundContacts()),
    std::out_of_range);
  LinkPosTask<double> task(model, 0);
  EXPECT_THROW(
    task.setDerivativeGain(Vec3<double>(1.0, -1.0, 1.0)),
    std::invalid_argument);
}

}  // namespace
