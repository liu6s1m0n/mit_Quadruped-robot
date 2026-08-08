#include <gtest/gtest.h>

#include <vector>

#include "WBC/KinWBC.hpp"
#include "WBC/WBIC.hpp"

namespace
{

template<typename T>
class TestTask final : public Task<T>
{
public:
  TestTask(std::size_t dimension, std::size_t velocity_dimension)
  : Task<T>(dimension)
  {
    this->Jt_ = DMat<T>::Zero(dimension, velocity_dimension);
    this->JtDotQdot_ = DVec<T>::Zero(dimension);
    this->op_cmd_ = DVec<T>::Zero(dimension);
    this->pos_err_ = DVec<T>::Zero(dimension);
    this->vel_des_ = DVec<T>::Zero(dimension);
    this->acc_des_ = DVec<T>::Zero(dimension);
  }

  void select(std::size_t task_row, std::size_t generalized_column, T value = T(1))
  {
    this->Jt_(task_row, generalized_column) = value;
  }

  void setKinematicCommand(const DVec<T> & position_error, const DVec<T> & velocity)
  {
    this->pos_err_ = position_error;
    this->vel_des_ = velocity;
  }

private:
  bool _UpdateCommand(const void *, const DVec<T> &, const DVec<T> &) override {return true;}
  bool _UpdateTaskJacobian() override {return true;}
  bool _UpdateTaskJDotQdot() override {return true;}
  bool _AdditionalUpdate() override {return true;}
};

TEST(KinWBC, AcceptsProjectJointVectorAndResizesOutputs)
{
  KinWBC<double> controller(18);
  TestTask<double> task(1, 18);
  task.select(0, 6);
  DVec<double> error(1);
  DVec<double> velocity(1);
  error << 0.2;
  velocity << -0.3;
  task.setKinematicCommand(error, velocity);
  std::vector<Task<double> *> tasks{&task};
  std::vector<ContactSpec<double> *> contacts;

  const DVec<double> current = DVec<double>::Zero(12);
  DVec<double> position_command;
  DVec<double> velocity_command;
  ASSERT_TRUE(
    controller.findConfiguration(
      current, tasks, contacts, position_command, velocity_command));
  ASSERT_EQ(position_command.size(), 12);
  ASSERT_EQ(velocity_command.size(), 12);
  EXPECT_NEAR(position_command[0], 0.2, 1e-12);
  EXPECT_NEAR(velocity_command[0], -0.3, 1e-12);
}

TEST(KinWBC, EmptyTaskListKeepsCurrentJointPosition)
{
  KinWBC<double> controller(18);
  const DVec<double> current = DVec<double>::LinSpaced(12, -0.5, 0.5);
  DVec<double> position_command;
  DVec<double> velocity_command;
  std::vector<Task<double> *> tasks;
  std::vector<ContactSpec<double> *> contacts;
  ASSERT_TRUE(
    controller.FindConfiguration(
      current, tasks, contacts, position_command, velocity_command));
  EXPECT_TRUE(position_command.isApprox(current));
  EXPECT_TRUE(velocity_command.isZero());
}

TEST(WBIC, SolvesNoContactDynamicsAndRejectsMissingSettings)
{
  std::vector<ContactSpec<double> *> contacts;
  std::vector<Task<double> *> tasks;
  WBIC<double> controller(18, &contacts, &tasks);
  WBICExtraData<double> data(6, 0);
  DVec<double> command;

  EXPECT_FALSE(controller.makeTorque(command, data));
  EXPECT_EQ(command.size(), 12);
  EXPECT_TRUE(command.isZero());

  DMat<double> mass = DMat<double>::Identity(18, 18);
  DVec<double> coriolis = DVec<double>::Zero(18);
  DVec<double> gravity = DVec<double>::LinSpaced(18, -0.5, 1.2);
  controller.UpdateSetting(mass, mass, coriolis, gravity);
  ASSERT_TRUE(controller.makeTorque(command, data));
  EXPECT_TRUE(command.isApprox(gravity.tail(12), 1e-10));
  EXPECT_EQ(data._qddot.size(), 18);
  EXPECT_EQ(data._opt_result.size(), 6);
  EXPECT_EQ(data._Fr.size(), 0);
}

}  // namespace
