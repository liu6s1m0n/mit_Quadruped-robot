#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "RobotRunner.hpp"
#include "model/floating_base_model_factory.hpp"

namespace
{

using ModelPointer = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPointer = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

constexpr std::array<const char *, kNumLegs> kLegNames{"FR", "FL", "RR", "RL"};
constexpr std::array<const char *, kJointsPerLeg> kJointSuffixes{
  "hip_joint", "thigh_joint", "calf_joint"};
constexpr std::array<const char *, kJointsPerLeg> kActuatorSuffixes{
  "hip", "thigh", "calf"};

struct JointAddress
{
  int qpos = -1;
  int dof = -1;
  int actuator = -1;
};

using JointAddresses =
  std::array<std::array<JointAddress, kJointsPerLeg>, kNumLegs>;

ModelPointer loadModel()
{
  char error[1024]{};
  ModelPointer model(
    mj_loadXML(MYMIT_ROBOT_TEST_SCENE_PATH, nullptr, error, sizeof(error)),
    &mj_deleteModel);
  if (!model) {throw std::runtime_error(error);}
  return model;
}

int namedId(const mjModel * model, int type, const std::string & name)
{
  const int id = mj_name2id(model, type, name.c_str());
  if (id < 0) {throw std::runtime_error("MuJoCo object not found: " + name);}
  return id;
}

JointAddresses jointAddresses(const mjModel * model)
{
  JointAddresses result{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const std::string prefix = kLegNames[leg];
      const int joint_id = namedId(
        model, mjOBJ_JOINT, prefix + "_" + kJointSuffixes[joint]);
      result[leg][joint] = JointAddress{
        model->jnt_qposadr[joint_id], model->jnt_dofadr[joint_id],
        namedId(model, mjOBJ_ACTUATOR, prefix + "_" + kActuatorSuffixes[joint])};
    }
  }
  return result;
}

double clampForce(const mjModel * model, int actuator, double force)
{
  if (model->actuator_forcelimited[actuator] == 0) {return force;}
  return std::clamp(
    force, model->actuator_forcerange[2 * actuator],
    model->actuator_forcerange[2 * actuator + 1]);
}

void writeCommands(
  const RobotRunner & runner, const JointAddresses & addresses,
  const mjModel * model, mjData * data)
{
  mju_zero(data->qfrc_applied, model->nv);
  const auto & commands = runner.jointCommands();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const auto & address = addresses[leg][joint];
      const auto & command = commands[leg];
      data->ctrl[address.actuator] = data->qpos[address.qpos];
      if (!command.enabled) {continue;}
      const Eigen::Index index = static_cast<Eigen::Index>(joint);
      const double torque = command.torque_feedforward[index] +
        command.kp[index] * (command.position_desired[index] - data->qpos[address.qpos]) +
        command.kd[index] * (command.velocity_desired[index] - data->qvel[address.dof]);
      data->qfrc_applied[address.dof] = clampForce(model, address.actuator, torque);
    }
  }
}

Vec3<float> bodyPositionTruth(const mjModel * model, const mjData * data)
{
  const int trunk = namedId(model, mjOBJ_BODY, "trunk");
  return Eigen::Map<const Vec3<double>>(data->xpos + 3 * trunk).cast<float>();
}

Vec3<float> bodyLinearVelocityTruth(const mjModel * model, const mjData * data)
{
  const int trunk = namedId(model, mjOBJ_BODY, "trunk");
  mjtNum velocity[6]{};
  mj_objectVelocity(model, data, mjOBJ_BODY, trunk, velocity, 0);
  return Eigen::Map<const Vec3<double>>(velocity + 3).cast<float>();
}

float orientationError(
  const mjModel * model, const mjData * data,
  const Eigen::Quaternionf & estimate)
{
  const int trunk = namedId(model, mjOBJ_BODY, "trunk");
  const mjtNum * value = data->xquat + 4 * trunk;
  Eigen::Quaternionf truth(
    static_cast<float>(value[0]), static_cast<float>(value[1]),
    static_cast<float>(value[2]), static_cast<float>(value[3]));
  return Eigen::AngleAxisf(truth.conjugate() * estimate).angle();
}

bool hasCalfCollision(const mjModel * model, const mjData * data)
{
  const int floor = namedId(model, mjOBJ_GEOM, "floor");
  std::array<int, kNumLegs> foot_geoms{};
  std::array<int, kNumLegs> calf_bodies{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    foot_geoms[leg] = namedId(model, mjOBJ_GEOM, kLegNames[leg]);
    calf_bodies[leg] = namedId(
      model, mjOBJ_BODY, std::string(kLegNames[leg]) + "_calf");
  }
  for (int index = 0; index < data->ncon; ++index) {
    const mjContact & contact = data->contact[index];
    const int other = contact.geom1 == floor ? contact.geom2 :
      (contact.geom2 == floor ? contact.geom1 : -1);
    if (other < 0 || std::find(foot_geoms.begin(), foot_geoms.end(), other) !=
      foot_geoms.end())
    {
      continue;
    }
    if (std::find(
        calf_bodies.begin(), calf_bodies.end(), model->geom_bodyid[other]) !=
      calf_bodies.end())
    {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(SimulationEstimationConsistency, WalkingEstimateTracksMujocoTruth)
{
  auto model = loadModel();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);
  const int home = namedId(model.get(), mjOBJ_KEY, "home");
  mj_resetDataKeyframe(model.get(), data.get(), home);
  mj_forward(model.get(), data.get());

  RobotRunner runner(model.get(), data.get());
  const JointAddresses addresses = jointAddresses(model.get());
  Vec3<float> position_offset = Vec3<float>::Zero();
  bool offset_initialized = false;
  float maximum_position_error = 0.0F;
  float maximum_velocity_error = 0.0F;
  float maximum_orientation_error = 0.0F;
  float maximum_standing_pitch = 0.0F;
  float maximum_walking_pitch = 0.0F;
  float final_walking_pitch = 0.0F;
  float minimum_walking_height = std::numeric_limits<float>::infinity();
  std::size_t calf_collision_frames = 0;
  double squared_position_error_sum = 0.0;
  double squared_velocity_error_sum = 0.0;
  std::size_t compared_frames = 0;
  int rejected_frames = 0;

  // 覆盖多个完整步态周期；旧实现约在 1300～1700 帧后才开始后腿拉直、
  // 机身下沉，因此短测试无法捕获持续行走失稳。
  for (int step = 0; step < 2400; ++step) {
    if (step == 400) {runner.setControlMode(ControlMode::Locomotion);}
    if (!runner.run()) {++rejected_frames;}
    const auto & estimate = runner.stateEstimate();
    ASSERT_TRUE(estimate.valid) << "invalid estimate at step " << step;
    const Vec3<float> truth_position = bodyPositionTruth(model.get(), data.get());
    if (!offset_initialized) {
      // IMU+腿运动学不能观测绝对水平原点，只比较相对启动点的位置。
      position_offset = truth_position - estimate.position_world;
      position_offset.z() = 0.0F;
      offset_initialized = true;
    }
    const float position_error =
      (estimate.position_world + position_offset - truth_position).norm();
    const float velocity_error =
      (estimate.velocity_world - bodyLinearVelocityTruth(model.get(), data.get())).norm();
    maximum_position_error = std::max(maximum_position_error, position_error);
    maximum_velocity_error = std::max(maximum_velocity_error, velocity_error);
    squared_position_error_sum += position_error * position_error;
    squared_velocity_error_sum += velocity_error * velocity_error;
    ++compared_frames;
    maximum_orientation_error = std::max(
      maximum_orientation_error,
      orientationError(model.get(), data.get(), estimate.orientation_world_from_body));
    if (step >= 250 && step < 400) {
      maximum_standing_pitch = std::max(
        maximum_standing_pitch, std::abs(estimate.rpy.y()));
    } else if (step >= 550) {
      maximum_walking_pitch = std::max(
        maximum_walking_pitch, std::abs(estimate.rpy.y()));
      final_walking_pitch = estimate.rpy.y();
      minimum_walking_height = std::min(
        minimum_walking_height, estimate.position_world.z());
      if (hasCalfCollision(model.get(), data.get())) {++calf_collision_frames;}
    }
    writeCommands(runner, addresses, model.get(), data.get());
    mj_step(model.get(), data.get());
  }

  RecordProperty("maximum_position_error_m", std::to_string(maximum_position_error));
  RecordProperty("maximum_velocity_error_mps", std::to_string(maximum_velocity_error));
  RecordProperty(
    "rms_position_error_m",
    std::to_string(std::sqrt(squared_position_error_sum / compared_frames)));
  RecordProperty(
    "rms_velocity_error_mps",
    std::to_string(std::sqrt(squared_velocity_error_sum / compared_frames)));
  RecordProperty(
    "maximum_orientation_error_rad", std::to_string(maximum_orientation_error));
  RecordProperty("maximum_standing_pitch_rad", std::to_string(maximum_standing_pitch));
  RecordProperty("maximum_walking_pitch_rad", std::to_string(maximum_walking_pitch));
  RecordProperty("final_walking_pitch_rad", std::to_string(final_walking_pitch));
  RecordProperty("minimum_walking_height_m", std::to_string(minimum_walking_height));
  RecordProperty("calf_collision_frames", static_cast<int>(calf_collision_frames));
  RecordProperty("rejected_control_frames", rejected_frames);
  EXPECT_LE(rejected_frames, 2);
  // 测试窗口延长到 4.8 s 后，纯 IMU/足端里程计允许少量水平累计漂移；
  // 同时保留速度、RMS 和姿态约束，避免用放宽峰值掩盖估计发散。
  EXPECT_LT(maximum_position_error, 0.06F);
  EXPECT_LT(
    std::sqrt(squared_position_error_sum / compared_frames), 0.035);
  EXPECT_LT(maximum_velocity_error, 0.25F);
  EXPECT_LT(maximum_orientation_error, 1.0e-4F);
  EXPECT_LT(maximum_standing_pitch, 0.08F);
  EXPECT_LT(maximum_walking_pitch, 0.10F);
  EXPECT_GT(minimum_walking_height, 0.22F);
  EXPECT_EQ(calf_collision_frames, 0U);
}
