#include <array>
#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "sensor/leg.hpp"

namespace
{

using ModelPointer = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPointer = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

ModelPointer loadGo1Model()
{
  char error[1024]{};
  ModelPointer model(
    mj_loadXML(MYMIT_ROBOT_TEST_SCENE_PATH, nullptr, error, sizeof(error)),
    &mj_deleteModel);
  if (!model) {
    throw std::runtime_error(error);
  }
  return model;
}

int jointId(const mjModel * model, const char * name)
{
  const int result = mj_name2id(model, mjOBJ_JOINT, name);
  if (result < 0) {
    throw std::runtime_error(std::string("joint not found: ") + name);
  }
  return result;
}

}  // namespace

TEST(LegTest, SimLegReadsMujocoJointState)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);

  const std::array<const char *, kJointsPerLeg> names{
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"};
  for (std::size_t index = 0; index < names.size(); ++index) {
    const int id = jointId(model.get(), names[index]);
    const int position = model->jnt_qposadr[id];
    const int velocity = model->jnt_dofadr[id];
    data->qpos[position] = 0.1 * static_cast<double>(index + 1);
    data->qvel[velocity] = 1.0 * static_cast<double>(index + 1);
    data->qfrc_actuator[velocity] = 10.0 * static_cast<double>(index + 1);
  }
  data->time = 1.25;

  SimLeg leg(model.get(), data.get(), LegId::FR);
  const auto state = leg.read();

  EXPECT_TRUE(state.valid);
  EXPECT_EQ(state.leg, LegId::FR);
  EXPECT_TRUE(state.position.isApprox(Vec3<float>(0.1F, 0.2F, 0.3F)));
  EXPECT_TRUE(state.velocity.isApprox(Vec3<float>(1.0F, 2.0F, 3.0F)));
  EXPECT_TRUE(state.torque_estimate.isApprox(Vec3<float>(10.0F, 20.0F, 30.0F)));
  EXPECT_FLOAT_EQ(state.timestamp, 1.25F);
  EXPECT_TRUE(leg.leg.valid);
}

TEST(LegTest, SimLegReadsAllLegsAfterMujocoForward)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);
  mj_forward(model.get(), data.get());

  constexpr std::array<LegId, kNumLegs> legs{
    LegId::FR, LegId::FL, LegId::RR, LegId::RL};
  for (const LegId id : legs) {
    const auto state = SimLeg(model.get(), data.get(), id).read();
    EXPECT_TRUE(state.valid);
    EXPECT_EQ(state.leg, id);
    EXPECT_TRUE(state.position.allFinite());
    EXPECT_TRUE(state.velocity.allFinite());
    EXPECT_TRUE(state.torque_estimate.allFinite());
  }
}

TEST(LegTest, HardwareLegKeepsHardwareInterfaceSafeUntilImplemented)
{
  HardwareLeg leg(LegId::RL);
  const auto state = leg.read();

  EXPECT_EQ(state.leg, LegId::RL);
  EXPECT_FALSE(state.valid);
  EXPECT_FALSE(leg.leg.valid);
}

TEST(LegTest, FactorySelectsBetweenSimulatorAndHardware)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);
  mj_forward(model.get(), data.get());

  auto simulator = makeLeg(
    LegSource::SIMULATOR, LegId::FL, model.get(), data.get());
  ASSERT_NE(simulator, nullptr);
  EXPECT_TRUE(simulator->read().valid);

  auto hardware = makeLeg(LegSource::HARDWARE, LegId::FL);
  ASSERT_NE(hardware, nullptr);
  EXPECT_FALSE(hardware->read().valid);
}

TEST(LegTest, RejectsMissingJoint)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  SimLeg::JointNames names{"missing_joint", "FR_thigh_joint", "FR_calf_joint"};

  EXPECT_THROW(
    SimLeg(model.get(), data.get(), LegId::FR, names),
    std::invalid_argument);
}
