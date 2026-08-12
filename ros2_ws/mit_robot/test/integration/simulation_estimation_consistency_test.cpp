#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "RobotRunner.hpp"
#include "SimulationDiagnostics.hpp"
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
  SimulationDiagnostics diagnostics(model.get());
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
  double squared_position_error_sum = 0.0;
  double squared_velocity_error_sum = 0.0;
  std::size_t compared_frames = 0;
  int rejected_frames = 0;

  // 覆盖多个完整步态周期；旧实现约在 1300～1700 帧后才开始后腿拉直、
  // 机身下沉，因此短测试无法捕获持续行走失稳。
  for (int step = 0; step < 2400; ++step) {
    if (step == 400) {runner.setControlMode(ControlMode::Locomotion);}
    const bool control_valid = runner.run();
    if (!control_valid) {++rejected_frames;}
    const auto & estimate = runner.stateEstimate();
    ASSERT_TRUE(estimate.valid) << "invalid estimate at step " << step;
    const Vec3<float> truth_position = diagnostics.bodyPosition(data.get());
    if (!offset_initialized) {
      // IMU+腿运动学不能观测绝对水平原点，只比较相对启动点的位置。
      position_offset = truth_position - estimate.position_world;
      position_offset.z() = 0.0F;
      offset_initialized = true;
    }
    const float position_error =
      (estimate.position_world + position_offset - truth_position).norm();
    const float velocity_error =
      (estimate.velocity_world - diagnostics.bodyLinearVelocity(data.get())).norm();
    maximum_position_error = std::max(maximum_position_error, position_error);
    maximum_velocity_error = std::max(maximum_velocity_error, velocity_error);
    squared_position_error_sum += position_error * position_error;
    squared_velocity_error_sum += velocity_error * velocity_error;
    ++compared_frames;
    maximum_orientation_error = std::max(
      maximum_orientation_error,
      diagnostics.orientationError(data.get(), estimate.orientation_world_from_body));
    if (step >= 250 && step < 400) {
      maximum_standing_pitch = std::max(
        maximum_standing_pitch, std::abs(estimate.rpy.y()));
    } else if (step >= 550) {
      maximum_walking_pitch = std::max(
        maximum_walking_pitch, std::abs(estimate.rpy.y()));
      final_walking_pitch = estimate.rpy.y();
      minimum_walking_height = std::min(
        minimum_walking_height, estimate.position_world.z());
      // 正式SimulationBridge使用同一observe接口持续统计；测试在稳定行走窗口
      // 直接复用它，保证运行时诊断不会与回归判据形成两套实现。
      diagnostics.observe(data.get(), estimate, control_valid);
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
  const auto & diagnostic_report = diagnostics.report();
  RecordProperty(
    "calf_collision_frames",
    static_cast<int>(diagnostic_report.calf_collision_frames));
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
  EXPECT_EQ(diagnostic_report.observed_frames, 1850U);
  EXPECT_EQ(diagnostic_report.calf_collision_frames, 0U);
}

TEST(SimulationEstimationConsistency, ExecutesAllFiveMujocoDirectionCommands)
{
  auto model = loadModel();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);
  const int home = namedId(model.get(), mjOBJ_KEY, "home");
  mj_resetDataKeyframe(model.get(), data.get(), home);
  mj_forward(model.get(), data.get());

  RobotRunner runner(model.get(), data.get());
  SimulationDiagnostics diagnostics(model.get());
  const JointAddresses addresses = jointAddresses(model.get());
  float minimum_height = std::numeric_limits<float>::infinity();
  int rejected_frames = 0;
  const auto advance = [&](int frame_count) {
      for (int frame = 0; frame < frame_count; ++frame) {
        if (!runner.run()) {++rejected_frames;}
        minimum_height = std::min(
          minimum_height, runner.stateEstimate().position_world.z());
        writeCommands(runner, addresses, model.get(), data.get());
        mj_step(model.get(), data.get());
      }
    };

  // 完成上电关节初始化，然后每次从 BalanceStand 重新进入 Locomotion，
  // 使每个方向都从零速度斜坡开始，避免前一方向的惯性污染符号检查。
  advance(450);
  const auto translate = [&](float forward, float lateral) {
      const Vec3<float> start = diagnostics.bodyPosition(data.get());
      runner.setLocomotionVelocityCommand(forward, lateral, 0.0F);
      runner.setControlMode(ControlMode::Locomotion);
      advance(500);
      const Vec3<float> displacement =
        diagnostics.bodyPosition(data.get()) - start;
      runner.setControlMode(ControlMode::BalanceStand);
      advance(100);
      return displacement;
    };

  const Vec3<float> forward = translate(0.25F, 0.0F);
  const Vec3<float> backward = translate(-0.25F, 0.0F);
  const Vec3<float> left = translate(0.0F, 0.20F);
  const Vec3<float> right = translate(0.0F, -0.20F);

  const float yaw_start = diagnostics.bodyYaw(data.get());
  runner.setLocomotionVelocityCommand(0.0F, 0.0F, 0.6F);
  runner.setControlMode(ControlMode::Locomotion);
  advance(700);
  const float yaw_end = diagnostics.bodyYaw(data.get());
  const float yaw_change = std::atan2(
    std::sin(yaw_end - yaw_start), std::cos(yaw_end - yaw_start));

  RecordProperty("forward_displacement_m", std::to_string(forward.x()));
  RecordProperty("backward_displacement_m", std::to_string(backward.x()));
  RecordProperty("left_displacement_m", std::to_string(left.y()));
  RecordProperty("right_displacement_m", std::to_string(right.y()));
  RecordProperty("ccw_yaw_change_rad", std::to_string(yaw_change));
  RecordProperty("minimum_height_m", std::to_string(minimum_height));
  EXPECT_GT(forward.x(), 0.02F);
  EXPECT_LT(backward.x(), -0.02F);
  EXPECT_GT(left.y(), 0.015F);
  EXPECT_LT(right.y(), -0.015F);
  EXPECT_GT(yaw_change, 0.10F);
  EXPECT_GT(minimum_height, 0.20F);
  EXPECT_LE(rejected_frames, 2);
}
