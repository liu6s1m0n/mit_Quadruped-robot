#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "controller/ContactEstimator.hpp"
#include "model/quadruped.hpp"
#include "sensor/imu.hpp"
#include "sensor/leg.hpp"

namespace
{

using ModelPointer = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPointer = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
using LegPointer = std::unique_ptr<LegSensor>;
using LegArray = std::array<LegPointer, kNumLegs>;
using LegPtrArray = std::array<LegSensor *, kNumLegs>;

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

// 构造四条仿真腿数据源（FR、FL、RR、RL）。
LegArray makeLegs(const mjModel * model, const mjData * data)
{
  LegArray legs;
  for (std::size_t index = 0; index < kNumLegs; ++index) {
    legs[index] = makeLeg(
      LegSource::SIMULATOR, static_cast<LegId>(index), model, data);
  }
  return legs;
}

LegPtrArray rawPtrs(LegArray & legs)
{
  LegPtrArray result;
  for (std::size_t index = 0; index < kNumLegs; ++index) {
    result[index] = legs[index].get();
  }
  return result;
}

// 设置 home keyframe，并把 12 个位置执行器的 ctrl 写成站立目标角。
void setupStandingPose(mjModel * model, mjData * data)
{
  const int home = mj_name2id(model, mjOBJ_KEY, "home");
  if (home < 0) {
    throw std::runtime_error("home keyframe not found");
  }
  mj_resetDataKeyframe(model, data, home);

  const std::array<std::array<const char *, kJointsPerLeg>, kNumLegs>
  actuator_names{{
    {{"FR_hip", "FR_thigh", "FR_calf"}},
    {{"FL_hip", "FL_thigh", "FL_calf"}},
    {{"RR_hip", "RR_thigh", "RR_calf"}},
    {{"RL_hip", "RL_thigh", "RL_calf"}}
  }};

  const auto quad = makeQuadruped<float>(RobotType::UNITREE_GO1);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const Vec3<float> home_position =
      quad.leg(static_cast<LegId>(leg)).joints.home_position;
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const int actuator_id =
        mj_name2id(model, mjOBJ_ACTUATOR, actuator_names[leg][joint]);
      if (actuator_id < 0) {
        throw std::runtime_error("actuator not found");
      }
      data->ctrl[actuator_id] =
        home_position[static_cast<Eigen::Index>(joint)];
    }
  }
}

// 让机器人在站立目标下稳定一段时间（2 秒，dt=0.002）。
void settleStanding(mjModel * model, mjData * data)
{
  for (int step = 0; step < 1000; ++step) {
    mj_step(model, data);
  }
}

}  // namespace

TEST(ContactEstimatorTest, DetectsAllFeetInContactWhenStanding)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);

  setupStandingPose(model.get(), data.get());
  settleStanding(model.get(), data.get());

  auto legs = makeLegs(model.get(), data.get());
  SimImu imu(model.get(), data.get());
  const auto quad = makeQuadruped<float>(RobotType::UNITREE_GO1);
  ContactEstimator<float> estimator(quad, imu, rawPtrs(legs));

  ASSERT_TRUE(estimator.run());

  const auto & contacts = estimator.footContacts();
  float total_force = 0.0f;
  for (const auto & contact : contacts) {
    EXPECT_TRUE(contact.valid);
    EXPECT_TRUE(contact.contact) << "站立时四脚都应接触";
    EXPECT_GE(contact.contact_probability, 0.5f);
    EXPECT_GT(contact.normal_force, 0.0f);
    EXPECT_FLOAT_EQ(contact.timestamp, estimator.timestamp());
    total_force += contact.normal_force;
  }

  // 四脚法向力之和应接近机身总重量。
  const float weight = quad.totalMass() * 9.81f;
  EXPECT_NEAR(total_force, weight, weight * 0.3f);
}

TEST(ContactEstimatorTest, ZeroTorqueLegIsDetectedAsNoContact)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);

  setupStandingPose(model.get(), data.get());
  settleStanding(model.get(), data.get());

  // 人为清零 FL 腿三个关节的执行器力，模拟该腿不承担任何支撑。
  const char * fl_joints[] = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"};
  for (const char * name : fl_joints) {
    const int joint_id = mj_name2id(model.get(), mjOBJ_JOINT, name);
    ASSERT_GE(joint_id, 0);
    const int dof = model.get()->jnt_dofadr[joint_id];
    data.get()->qfrc_actuator[dof] = 0.0;
  }

  auto legs = makeLegs(model.get(), data.get());
  SimImu imu(model.get(), data.get());
  const auto quad = makeQuadruped<float>(RobotType::UNITREE_GO1);
  ContactEstimator<float> estimator(quad, imu, rawPtrs(legs));

  ASSERT_TRUE(estimator.run());
  const auto & contacts = estimator.footContacts();

  // FL 腿不再支撑，不应判为接触。
  EXPECT_FALSE(contacts[static_cast<std::size_t>(LegId::FL)].contact);
  EXPECT_LT(
    contacts[static_cast<std::size_t>(LegId::FL)].contact_probability, 0.5f);

  // 其余三条腿仍保持支撑。
  for (const LegId leg_id : {LegId::FR, LegId::RR, LegId::RL}) {
    const auto & contact = contacts[static_cast<std::size_t>(leg_id)];
    EXPECT_TRUE(contact.contact) << "其余腿应仍处于接触状态";
  }
}

TEST(ContactEstimatorTest, HandlesInvalidImuData)
{
  auto model = loadGo1Model();
  // 绑定空数据指针的 SimImu：read() 返回 valid=false。
  SimImu imu(model.get(), nullptr);
  auto legs = makeLegs(model.get(), nullptr);
  const auto quad = makeQuadruped<float>(RobotType::UNITREE_GO1);
  ContactEstimator<float> estimator(quad, imu, rawPtrs(legs));

  EXPECT_FALSE(estimator.run());
  EXPECT_FALSE(estimator.legsValid());
}

TEST(ContactEstimatorTest, RejectsZeroNormImuQuaternion)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);

  setupStandingPose(model.get(), data.get());
  settleStanding(model.get(), data.get());

  const int sensor_id =
    mj_name2id(model.get(), mjOBJ_SENSOR, "imu_orientation");
  ASSERT_GE(sensor_id, 0);
  const int sensor_address = model.get()->sensor_adr[sensor_id];
  for (int index = 0; index < 4; ++index) {
    data.get()->sensordata[sensor_address + index] = 0.0;
  }

  auto legs = makeLegs(model.get(), data.get());
  SimImu imu(model.get(), data.get());
  const auto quad = makeQuadruped<float>(RobotType::UNITREE_GO1);
  ContactEstimator<float> estimator(quad, imu, rawPtrs(legs));

  EXPECT_FALSE(estimator.run());
  for (const auto & contact : estimator.footContacts()) {
    EXPECT_FALSE(contact.valid);
    EXPECT_FALSE(contact.contact);
    EXPECT_FLOAT_EQ(contact.contact_probability, 0.0f);
  }
}

TEST(ContactEstimatorTest, ClearsOldContactsWhenLegFeedbackBecomesInvalid)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);

  setupStandingPose(model.get(), data.get());
  settleStanding(model.get(), data.get());

  auto legs = makeLegs(model.get(), data.get());
  SimImu imu(model.get(), data.get());
  const auto quad = makeQuadruped<float>(RobotType::UNITREE_GO1);
  ContactEstimator<float> estimator(quad, imu, rawPtrs(legs));
  ASSERT_TRUE(estimator.run());

  const int joint_id = mj_name2id(model.get(), mjOBJ_JOINT, "FL_hip_joint");
  ASSERT_GE(joint_id, 0);
  const int position_address = model.get()->jnt_qposadr[joint_id];
  data.get()->qpos[position_address] =
    std::numeric_limits<mjtNum>::quiet_NaN();

  EXPECT_FALSE(estimator.run());
  for (const auto & contact : estimator.footContacts()) {
    EXPECT_FALSE(contact.valid);
    EXPECT_FALSE(contact.contact);
    EXPECT_FLOAT_EQ(contact.contact_probability, 0.0f);
    EXPECT_FLOAT_EQ(contact.normal_force, 0.0f);
  }
}

TEST(ContactEstimatorTest, DampedSolveHandlesStraightLegConfiguration)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);

  setupStandingPose(model.get(), data.get());
  const char * fr_joints[] = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"};
  for (const char * name : fr_joints) {
    const int joint_id = mj_name2id(model.get(), mjOBJ_JOINT, name);
    ASSERT_GE(joint_id, 0);
    const int position_address = model.get()->jnt_qposadr[joint_id];
    data.get()->qpos[position_address] = 0.0;
  }
  mj_forward(model.get(), data.get());

  auto legs = makeLegs(model.get(), data.get());
  SimImu imu(model.get(), data.get());
  const auto quad = makeQuadruped<float>(RobotType::UNITREE_GO1);
  ContactEstimator<float> estimator(quad, imu, rawPtrs(legs));

  EXPECT_TRUE(estimator.run());
  const auto & contact =
    estimator.footContacts()[static_cast<std::size_t>(LegId::FR)];
  EXPECT_TRUE(contact.valid);
  EXPECT_TRUE(std::isfinite(contact.normal_force));
}

TEST(ContactEstimatorTest, RejectsNonFiniteParameters)
{
  ContactEstimatorParameters<float> parameters;
  parameters.contact_probability_threshold =
    std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(parameters.isValid());

  parameters = ContactEstimatorParameters<float>{};
  parameters.minimum_support_ratio =
    std::numeric_limits<float>::infinity();
  EXPECT_FALSE(parameters.isValid());
}
