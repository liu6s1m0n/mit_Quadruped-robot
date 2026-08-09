#include <gtest/gtest.h>

#include <array>
#include <utility>

#include "WBC/WBC_Ctrl/WBC_Ctrl.hpp"
#include "model/robots/unitree_go1.hpp"

namespace
{

template<typename T>
class EmptyWbcController final : public WBC_Ctrl<T>
{
public:
  explicit EmptyWbcController(FloatingBaseModel<T> model)
  : WBC_Ctrl<T>(std::move(model)) {}

  bool prepare_called = false;

protected:
  bool prepareTasksAndContacts(const void * input) override
  {
    prepare_called = true;
    return input == nullptr;
  }
};

StateEstimate<float> makeEstimate()
{
  StateEstimate<float> estimate;
  estimate.position_world << 0.0, 0.0, 0.27;
  estimate.valid = true;
  return estimate;
}

std::array<JointState<float>, kNumLegs> makeJointStates(
  const Quadruped<float> & quadruped)
{
  std::array<JointState<float>, kNumLegs> result;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    result[leg].leg = static_cast<LegId>(leg);
    result[leg].position = quadruped.leg(static_cast<LegId>(leg)).joints.home_position;
    result[leg].valid = true;
  }
  return result;
}

TEST(WbcController, RunsCommonPipelineAndAppliesProjectLegCommands)
{
  const auto quadruped = robots::unitree_go1::makeModel<float>();
  EmptyWbcController<float> controller(
    robots::unitree_go1::makeFloatingBaseModel<float>());
  LegController<float> leg_controller(quadruped);
  const auto joints = makeJointStates(quadruped);
  for (const auto & joint : joints) {ASSERT_TRUE(leg_controller.updateData(joint));}

  ASSERT_TRUE(controller.runAndApply(nullptr, makeEstimate(), joints, leg_controller));
  EXPECT_TRUE(controller.prepare_called);
  EXPECT_TRUE(controller.result().valid);
  EXPECT_EQ(controller.result().joint_torque.size(), 12);
  EXPECT_TRUE(leg_controller.legsEnabled());
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_TRUE(leg_controller.commands[leg].position_desired.isApprox(joints[leg].position));
    EXPECT_TRUE(leg_controller.commands[leg].torque_feedforward.allFinite());
  }
}

TEST(WbcController, RejectsInvalidStateAndDisablesOutput)
{
  const auto quadruped = robots::unitree_go1::makeModel<float>();
  EmptyWbcController<float> controller(
    robots::unitree_go1::makeFloatingBaseModel<float>());
  LegController<float> leg_controller(quadruped);
  auto estimate = makeEstimate();
  estimate.valid = false;

  EXPECT_FALSE(
    controller.runAndApply(nullptr, estimate, makeJointStates(quadruped), leg_controller));
  EXPECT_FALSE(controller.result().valid);
  EXPECT_FALSE(leg_controller.legsEnabled());
}

}  // namespace
