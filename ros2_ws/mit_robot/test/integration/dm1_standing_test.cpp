#include <algorithm>
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

namespace
{
using ModelPointer = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPointer = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

struct JointAddress {int qpos; int dof; int actuator;};
using JointAddresses =
  std::array<std::array<JointAddress, kJointsPerLeg>, kNumLegs>;

constexpr std::array<const char *, kNumLegs> kLegNames{"FR", "FL", "RR", "RL"};

ModelPointer loadModel()
{
  char error[1024]{};
  ModelPointer model(
    mj_loadXML(MYMIT_ROBOT_DM1_TEST_SCENE_PATH, nullptr, error, sizeof(error)),
    &mj_deleteModel);
  if (!model) {throw std::runtime_error(error);}
  return model;
}

JointAddresses jointAddresses(const mjModel * model)
{
  constexpr std::array<const char *, kNumLegs> legs{"FR", "FL", "RR", "RL"};
  constexpr std::array<const char *, kJointsPerLeg> joint_suffixes{
    "hip_joint", "thigh_joint", "calf_joint"};
  constexpr std::array<const char *, kJointsPerLeg> actuator_suffixes{
    "hip", "thigh", "calf"};
  JointAddresses result{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const int joint_id = mj_name2id(
        model, mjOBJ_JOINT,
        (std::string(legs[leg]) + "_" + joint_suffixes[joint]).c_str());
      const int actuator_id = mj_name2id(
        model, mjOBJ_ACTUATOR,
        (std::string(legs[leg]) + "_" + actuator_suffixes[joint]).c_str());
      if (joint_id < 0 || actuator_id < 0) {throw std::runtime_error("DM1 mapping incomplete");}
      result[leg][joint] = {
        model->jnt_qposadr[joint_id], model->jnt_dofadr[joint_id], actuator_id};
    }
  }
  return result;
}

void writeCommands(
  const RobotRunner & runner, const JointAddresses & addresses,
  const mjModel * model, mjData * data)
{
  mju_zero(data->qfrc_applied, model->nv);
  const auto & commands = runner.jointCommands();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const auto & command = commands[leg];
      const auto & address = addresses[leg][joint];
      data->ctrl[address.actuator] = data->qpos[address.qpos];
      if (!command.enabled) {continue;}
      const Eigen::Index index = static_cast<Eigen::Index>(joint);
      const double torque = command.torque_feedforward[index] +
        command.kp[index] * (command.position_desired[index] - data->qpos[address.qpos]) +
        command.kd[index] * (command.velocity_desired[index] - data->qvel[address.dof]);
      const double limit = runner.quadruped().leg(static_cast<LegId>(leg))
        .joints.torque_limit[index];
      data->qfrc_applied[address.dof] = std::clamp(torque, -limit, limit);
    }
  }
}

std::array<bool, kNumLegs> footGroundContacts(
  const mjModel * model, const mjData * data)
{
  const int floor = mj_name2id(model, mjOBJ_GEOM, "floor");
  std::array<int, kNumLegs> feet{};
  std::array<bool, kNumLegs> contacts{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    feet[leg] = mj_name2id(model, mjOBJ_GEOM, kLegNames[leg]);
  }
  for (int index = 0; index < data->ncon; ++index) {
    const auto & contact = data->contact[index];
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      contacts[leg] = contacts[leg] ||
        ((contact.geom1 == floor && contact.geom2 == feet[leg]) ||
        (contact.geom2 == floor && contact.geom1 == feet[leg]));
    }
  }
  return contacts;
}

std::array<bool, kNumLegs> calfGroundContacts(
  const mjModel * model, const mjData * data)
{
  const int floor = mj_name2id(model, mjOBJ_GEOM, "floor");
  std::array<int, kNumLegs> calf_bodies{};
  std::array<bool, kNumLegs> contacts{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    calf_bodies[leg] = mj_name2id(
      model, mjOBJ_BODY, (std::string(kLegNames[leg]) + "_calf").c_str());
  }
  for (int index = 0; index < data->ncon; ++index) {
    const auto & contact = data->contact[index];
    const int other_geom = contact.geom1 == floor ? contact.geom2 :
      (contact.geom2 == floor ? contact.geom1 : -1);
    if (other_geom < 0) {continue;}
    const int other_body = model->geom_bodyid[other_geom];
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      contacts[leg] = contacts[leg] || other_body == calf_bodies[leg];
    }
  }
  return contacts;
}
}  // namespace

TEST(Dm1StandingIntegration, HomeIsLowestStandingPosture)
{
  auto model = loadModel();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);
  const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
  ASSERT_GE(home, 0);
  mj_resetDataKeyframe(model.get(), data.get(), home);
  mj_forward(model.get(), data.get());

  const JointAddresses addresses = jointAddresses(model.get());
  for (const auto & leg : addresses) {
    for (const auto & joint : leg) {
      EXPECT_NEAR(data->qpos[joint.qpos], 0.0, 1.0e-9);
    }
  }

  // Home 保持站立时的四足落点，只把机身竖直降到底；HAA 不向两侧摊开。
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const std::string prefix = kLegNames[leg];
    const int thigh = mj_name2id(
      model.get(), mjOBJ_BODY, (prefix + "_thigh").c_str());
    const int knee = mj_name2id(
      model.get(), mjOBJ_BODY, (prefix + "_calf").c_str());
    const int foot = mj_name2id(
      model.get(), mjOBJ_BODY, (prefix + "_foot").c_str());
    ASSERT_GE(thigh, 0);
    ASSERT_GE(knee, 0);
    ASSERT_GE(foot, 0);
    const double expected_x = leg < 2 ? 0.241 : -0.183;
    EXPECT_NEAR(data->xpos[3 * foot], expected_x, 0.01) << kLegNames[leg];
    EXPECT_LT(data->xpos[3 * foot + 2], 0.03) << kLegNames[leg];
    EXPECT_LT(std::abs(data->xpos[3 * foot + 1]), 0.22) << kLegNames[leg];
    // 小腿两端等高且碰撞胶囊半径为 0.018 m，因而整段小腿而非单独足球
    // 都处于地面接触高度。
    EXPECT_NEAR(data->xpos[3 * knee + 2], data->xpos[3 * foot + 2], 0.003)
      << kLegNames[leg];
    EXPECT_LT(data->xpos[3 * knee + 2], 0.025) << kLegNames[leg];
  }

  const auto quadruped = makeQuadruped<float>(RobotType::DM_BOT1);
  EXPECT_TRUE(quadruped.leg(LegId::FR).joints.zero_offset.isApprox(
    Vec3<float>(0.0F, -0.203F, -2.25F)));
  EXPECT_NEAR(
    quadruped.leg(LegId::RR).joints.zero_offset.y(), -0.203F, 1.0e-6F);

  // 让接触求解器稳定后，四段小腿都必须直接接触地面，而不是仅有足端球。
  for (int frame = 0; frame < 100; ++frame) {mj_step(model.get(), data.get());}
  const auto calf_contacts = calfGroundContacts(model.get(), data.get());
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_TRUE(calf_contacts[leg]) << kLegNames[leg];
  }
}

TEST(Dm1StandingIntegration, InitializesAndMaintainsBalanceStand)
{
  auto model = loadModel();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);
  const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
  ASSERT_GE(home, 0);
  mj_resetDataKeyframe(model.get(), data.get(), home);
  mj_forward(model.get(), data.get());

  RobotRunner runner(model.get(), data.get(), RobotType::DM_BOT1);
  SimulationDiagnostics diagnostics(model.get());
  const JointAddresses addresses = jointAddresses(model.get());
  int rejected_frames = 0;
  float minimum_height = 1.0F;
  float maximum_tilt = 0.0F;
  double squared_roll_pitch_sum = 0.0;
  double squared_angular_velocity_sum = 0.0;
  double squared_joint_velocity_sum = 0.0;
  double height_sum = 0.0;
  double roll_sum = 0.0;
  double pitch_sum = 0.0;
  float settled_minimum_height = 1.0F;
  float settled_maximum_height = 0.0F;
  std::size_t settled_calf_collision_frames = 0;
  std::size_t settled_frames = 0;
  for (int frame = 0; frame < 3800; ++frame) {
    if (frame == 650) {
      EXPECT_EQ(runner.currentStateName(), FSM_StateName::JOINT_PD);
      EXPECT_LT(diagnostics.bodyPosition(data.get()).z(), 0.18F);
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
          EXPECT_NEAR(data->qpos[addresses[leg][joint].qpos], 0.0, 0.01)
            << kLegNames[leg] << " joint " << joint;
        }
      }
      EXPECT_TRUE(runner.requestStandUp());
    }
    if (!runner.run()) {++rejected_frames;}
    writeCommands(runner, addresses, model.get(), data.get());
    mj_step(model.get(), data.get());
    const float height = diagnostics.bodyPosition(data.get()).z();
    minimum_height = std::min(minimum_height, height);
    maximum_tilt = std::max(
      maximum_tilt,
      std::max(std::abs(runner.stateEstimate().rpy.x()),
      std::abs(runner.stateEstimate().rpy.y())));
    if (frame >= 3300) {
      const auto & estimate = runner.stateEstimate();
      squared_roll_pitch_sum += estimate.rpy.x() * estimate.rpy.x() +
        estimate.rpy.y() * estimate.rpy.y();
      roll_sum += estimate.rpy.x();
      pitch_sum += estimate.rpy.y();
      squared_angular_velocity_sum += estimate.angular_velocity_body.squaredNorm();
      for (const auto & leg : addresses) {
        for (const auto & joint : leg) {
          const double velocity = data->qvel[joint.dof];
          squared_joint_velocity_sum += velocity * velocity;
        }
      }
      height_sum += height;
      settled_minimum_height = std::min(settled_minimum_height, height);
      settled_maximum_height = std::max(settled_maximum_height, height);
      settled_calf_collision_frames += diagnostics.hasCalfCollision(data.get()) ? 1U : 0U;
      ++settled_frames;
    }
  }
  ASSERT_GT(settled_frames, 0U);
  const double roll_pitch_rms =
    std::sqrt(squared_roll_pitch_sum / (2.0 * settled_frames));
  const double angular_velocity_rms =
    std::sqrt(squared_angular_velocity_sum / (3.0 * settled_frames));
  const double joint_velocity_rms = std::sqrt(
    squared_joint_velocity_sum /
    (static_cast<double>(kNumJoints) * settled_frames));
  const double mean_height = height_sum / settled_frames;
  RecordProperty("mean_height_m", std::to_string(mean_height));
  RecordProperty("mean_roll_rad", std::to_string(roll_sum / settled_frames));
  RecordProperty("mean_pitch_rad", std::to_string(pitch_sum / settled_frames));
  RecordProperty(
    "height_peak_to_peak_m",
    std::to_string(settled_maximum_height - settled_minimum_height));
  RecordProperty("roll_pitch_rms_rad", std::to_string(roll_pitch_rms));
  RecordProperty("angular_velocity_rms_radps", std::to_string(angular_velocity_rms));
  RecordProperty("joint_velocity_rms_radps", std::to_string(joint_velocity_rms));
  RecordProperty("maximum_startup_tilt_rad", std::to_string(maximum_tilt));
  RecordProperty(
    "settled_calf_collision_frames",
    static_cast<int>(settled_calf_collision_frames));
  EXPECT_LE(rejected_frames, 2);
  EXPECT_GT(minimum_height, 0.115F);
  EXPECT_LT(maximum_tilt, 0.15F);
  EXPECT_NEAR(mean_height, 0.39, 0.01);
  EXPECT_LT(settled_maximum_height - settled_minimum_height, 0.001F);
  EXPECT_LT(roll_pitch_rms, 0.005);
  EXPECT_LT(angular_velocity_rms, 0.01);
  EXPECT_LT(joint_velocity_rms, 0.02);
  EXPECT_EQ(settled_calf_collision_frames, 0U);
  EXPECT_EQ(runner.currentStateName(), FSM_StateName::BALANCE_STAND);
}

TEST(Dm1StandingIntegration, RecoversFromModerateLateralPush)
{
  auto model = loadModel();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);
  const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
  const int trunk = mj_name2id(model.get(), mjOBJ_BODY, "trunk");
  ASSERT_GE(home, 0);
  ASSERT_GE(trunk, 0);
  mj_resetDataKeyframe(model.get(), data.get(), home);
  mj_forward(model.get(), data.get());

  RobotRunner runner(model.get(), data.get(), RobotType::DM_BOT1);
  SimulationDiagnostics diagnostics(model.get());
  const JointAddresses addresses = jointAddresses(model.get());
  const auto advance = [&]() {
      const bool valid = runner.run();
      writeCommands(runner, addresses, model.get(), data.get());
      mj_step(model.get(), data.get());
      return valid;
    };

  int rejected_frames = 0;
  for (int frame = 0; frame < 2800; ++frame) {
    if (frame == 650) {EXPECT_TRUE(runner.requestStandUp());}
    if (!advance()) {++rejected_frames;}
  }
  const Vec3<float> initial_position = diagnostics.bodyPosition(data.get());

  // 在质心附近施加 0.1 s、40 N 的横向推力，模拟手推或轻微碰撞。
  float maximum_tilt = 0.0F;
  float maximum_lateral_displacement = 0.0F;
  for (int frame = 0; frame < 50; ++frame) {
    data->xfrc_applied[6 * trunk + 1] = 40.0;
    if (!advance()) {++rejected_frames;}
  }
  mju_zero(data->xfrc_applied, 6 * model->nbody);

  int recovery_frame = -1;
  double final_squared_tilt_sum = 0.0;
  double final_squared_angular_velocity_sum = 0.0;
  for (int frame = 0; frame < 2000; ++frame) {
    if (!advance()) {++rejected_frames;}
    const auto & estimate = runner.stateEstimate();
    const float tilt = std::max(
      std::abs(estimate.rpy.x()), std::abs(estimate.rpy.y()));
    const float lateral_displacement = std::abs(
      diagnostics.bodyPosition(data.get()).y() - initial_position.y());
    maximum_tilt = std::max(maximum_tilt, tilt);
    maximum_lateral_displacement = std::max(
      maximum_lateral_displacement, lateral_displacement);
    if (recovery_frame < 0 && frame >= 50 && tilt < 0.02F &&
      lateral_displacement < 0.02F &&
      estimate.angular_velocity_body.norm() < 0.05F)
    {
      recovery_frame = frame;
    }
    if (frame >= 1500) {
      final_squared_tilt_sum += estimate.rpy.x() * estimate.rpy.x() +
        estimate.rpy.y() * estimate.rpy.y();
      final_squared_angular_velocity_sum +=
        estimate.angular_velocity_body.squaredNorm();
    }
  }
  const double final_tilt_rms = std::sqrt(final_squared_tilt_sum / 1000.0);
  const double final_angular_velocity_rms =
    std::sqrt(final_squared_angular_velocity_sum / 1500.0);
  RecordProperty("maximum_tilt_rad", std::to_string(maximum_tilt));
  RecordProperty(
    "maximum_lateral_displacement_m",
    std::to_string(maximum_lateral_displacement));
  RecordProperty(
    "recovery_time_s",
    recovery_frame < 0 ? "not_recovered" :
    std::to_string(recovery_frame * model->opt.timestep));
  RecordProperty("final_tilt_rms_rad", std::to_string(final_tilt_rms));
  RecordProperty(
    "final_angular_velocity_rms_radps",
    std::to_string(final_angular_velocity_rms));

  EXPECT_LE(rejected_frames, 4);
  EXPECT_LT(maximum_tilt, 0.05F);
  EXPECT_LT(maximum_lateral_displacement, 0.04F);
  EXPECT_GE(recovery_frame, 0);
  EXPECT_LT(recovery_frame * model->opt.timestep, 0.5);
  EXPECT_LT(final_tilt_rms, 0.005);
  EXPECT_LT(final_angular_velocity_rms, 0.01);
}

TEST(Dm1StandingIntegration, LocomotionDirectionsUseFeetWithoutCalfContact)
{
  struct MotionResult
  {
    Vec3<float> displacement = Vec3<float>::Zero();
    float yaw_change = 0.0F;
    float minimum_height = std::numeric_limits<float>::infinity();
    float maximum_tilt = 0.0F;
    int rejected_frames = 0;
    std::size_t non_locomotion_frames = 0;
    std::size_t calf_collision_frames = 0;
    std::array<std::size_t, kNumLegs> airborne_frames{};
    std::array<std::size_t, kNumLegs> contact_transitions{};
  };

  const auto run_motion = [](float forward, float lateral, float yaw_rate) {
      auto model = loadModel();
      DataPointer data(mj_makeData(model.get()), &mj_deleteData);
      if (!data) {throw std::runtime_error("failed to allocate DM1 simulation data");}
      const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
      if (home < 0) {throw std::runtime_error("DM1 home keyframe missing");}
      mj_resetDataKeyframe(model.get(), data.get(), home);
      mj_forward(model.get(), data.get());

      RobotRunner runner(model.get(), data.get(), RobotType::DM_BOT1);
      SimulationDiagnostics diagnostics(model.get());
      const JointAddresses addresses = jointAddresses(model.get());
      MotionResult result;
      const auto advance = [&]() {
          if (!runner.run()) {++result.rejected_frames;}
          writeCommands(runner, addresses, model.get(), data.get());
          mj_step(model.get(), data.get());
        };

      // DM1 先保持趴卧零位，收到显式站立请求后才建立站姿并切换步态。
      for (int frame = 0; frame < 2800; ++frame) {
        if (frame == 650 && !runner.requestStandUp()) {
          throw std::runtime_error("DM1 stand-up request was rejected");
        }
        advance();
      }
      if (runner.currentStateName() != FSM_StateName::BALANCE_STAND) {
        throw std::runtime_error("DM1 did not reach BalanceStand");
      }
      const Vec3<float> start_position = diagnostics.bodyPosition(data.get());
      const float start_yaw = diagnostics.bodyYaw(data.get());
      runner.setLocomotionVelocityCommand(forward, lateral, yaw_rate);
      runner.setControlMode(ControlMode::Locomotion);

      std::array<bool, kNumLegs> previous_contacts{};
      bool previous_initialized = false;
      for (int frame = 0; frame < 1200; ++frame) {
        advance();
        if (frame >= 5 && runner.currentStateName() != FSM_StateName::LOCOMOTION) {
          ++result.non_locomotion_frames;
        }
        const auto contacts = footGroundContacts(model.get(), data.get());
        if (previous_initialized) {
          for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            result.contact_transitions[leg] +=
              contacts[leg] != previous_contacts[leg] ? 1U : 0U;
          }
        }
        previous_contacts = contacts;
        previous_initialized = true;
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
          result.airborne_frames[leg] += contacts[leg] ? 0U : 1U;
        }
        result.calf_collision_frames +=
          diagnostics.hasCalfCollision(data.get()) ? 1U : 0U;
        result.minimum_height = std::min(
          result.minimum_height, diagnostics.bodyPosition(data.get()).z());
        result.maximum_tilt = std::max(
          result.maximum_tilt,
          std::max(std::abs(runner.stateEstimate().rpy.x()),
          std::abs(runner.stateEstimate().rpy.y())));
      }
      result.displacement = diagnostics.bodyPosition(data.get()) - start_position;
      const float end_yaw = diagnostics.bodyYaw(data.get());
      result.yaw_change = std::atan2(
        std::sin(end_yaw - start_yaw), std::cos(end_yaw - start_yaw));
      return result;
    };

  // 使用正式 GUI 的默认档位，而不是为了测试降低命令幅值。
  const MotionResult forward = run_motion(0.36F, 0.0F, 0.0F);
  const MotionResult backward = run_motion(-0.30F, 0.0F, 0.0F);
  const MotionResult left = run_motion(0.0F, 0.25F, 0.0F);
  const MotionResult right = run_motion(0.0F, -0.25F, 0.0F);
  const MotionResult counter_clockwise = run_motion(0.0F, 0.0F, 0.35F);
  const MotionResult clockwise = run_motion(0.0F, 0.0F, -0.35F);

  RecordProperty("forward_x_m", std::to_string(forward.displacement.x()));
  RecordProperty("backward_x_m", std::to_string(backward.displacement.x()));
  RecordProperty("left_y_m", std::to_string(left.displacement.y()));
  RecordProperty("right_y_m", std::to_string(right.displacement.y()));
  RecordProperty("ccw_yaw_rad", std::to_string(counter_clockwise.yaw_change));
  RecordProperty("cw_yaw_rad", std::to_string(clockwise.yaw_change));
  const std::array<MotionResult, 6> results{
    forward, backward, left, right, counter_clockwise, clockwise};
  for (std::size_t motion = 0; motion < results.size(); ++motion) {
    RecordProperty(
      "motion_" + std::to_string(motion) + "_min_height_m",
      std::to_string(results[motion].minimum_height));
    RecordProperty(
      "motion_" + std::to_string(motion) + "_max_tilt_rad",
      std::to_string(results[motion].maximum_tilt));
    RecordProperty(
      "motion_" + std::to_string(motion) + "_calf_collision_frames",
      static_cast<int>(results[motion].calf_collision_frames));
    RecordProperty(
      "motion_" + std::to_string(motion) + "_non_locomotion_frames",
      static_cast<int>(results[motion].non_locomotion_frames));
    EXPECT_GT(results[motion].minimum_height, 0.24F) << "motion " << motion;
    EXPECT_LT(results[motion].maximum_tilt, 0.20F) << "motion " << motion;
    EXPECT_EQ(results[motion].calf_collision_frames, 0U) << "motion " << motion;
    EXPECT_LE(results[motion].rejected_frames, 3) << "motion " << motion;
    EXPECT_EQ(results[motion].non_locomotion_frames, 0U) << "motion " << motion;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      RecordProperty(
        "motion_" + std::to_string(motion) + "_" + kLegNames[leg] +
        "_airborne_frames", static_cast<int>(results[motion].airborne_frames[leg]));
      RecordProperty(
        "motion_" + std::to_string(motion) + "_" + kLegNames[leg] +
        "_contact_transitions",
        static_cast<int>(results[motion].contact_transitions[leg]));
      EXPECT_GT(results[motion].airborne_frames[leg], 100U)
        << "motion " << motion << " leg " << kLegNames[leg];
      EXPECT_GE(results[motion].contact_transitions[leg], 4U)
        << "motion " << motion << " leg " << kLegNames[leg];
    }
  }
  EXPECT_GT(forward.displacement.x(), 0.04F);
  EXPECT_LT(backward.displacement.x(), -0.04F);
  EXPECT_GT(left.displacement.y(), 0.03F);
  EXPECT_LT(right.displacement.y(), -0.03F);
  EXPECT_GT(counter_clockwise.yaw_change, 0.12F);
  EXPECT_LT(clockwise.yaw_change, -0.12F);
}

TEST(Dm1StandingIntegration, ExecutesProfiledForwardJumpAndRecovers)
{
  auto model = loadModel();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);
  const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
  ASSERT_GE(home, 0);
  mj_resetDataKeyframe(model.get(), data.get(), home);
  mj_forward(model.get(), data.get());

  RobotRunner runner(model.get(), data.get(), RobotType::DM_BOT1);
  SimulationDiagnostics diagnostics(model.get());
  const JointAddresses addresses = jointAddresses(model.get());
  int rejected_frames = 0;
  const auto advance = [&]() {
      if (!runner.run()) {++rejected_frames;}
      writeCommands(runner, addresses, model.get(), data.get());
      mj_step(model.get(), data.get());
    };

  // 验证 home 的十二个电机读数确实为零，且没有按钮请求时保持趴卧。
  for (const auto & leg : addresses) {
    for (const auto & joint : leg) {EXPECT_NEAR(data->qpos[joint.qpos], 0.0, 1.0e-9);}
  }
  for (int frame = 0; frame < 650; ++frame) {advance();}
  EXPECT_EQ(runner.currentStateName(), FSM_StateName::JOINT_PD);
  EXPECT_LT(diagnostics.bodyPosition(data.get()).z(), 0.25F);
  ASSERT_TRUE(runner.requestStandUp());
  for (int frame = 0; frame < 2200; ++frame) {advance();}
  ASSERT_EQ(runner.currentStateName(), FSM_StateName::BALANCE_STAND);

  const Vec3<float> start = diagnostics.bodyPosition(data.get());
  ASSERT_TRUE(runner.requestFrontJump());
  float maximum_height = start.z();
  float maximum_tilt = 0.0F;
  std::size_t calf_collision_frames = 0;
  int first_calf_collision_frame = -1;
  for (int frame = 0; frame < 1000; ++frame) {
    advance();
    maximum_height = std::max(maximum_height, diagnostics.bodyPosition(data.get()).z());
    maximum_tilt = std::max(
      maximum_tilt,
      std::max(std::abs(runner.stateEstimate().rpy.x()),
      std::abs(runner.stateEstimate().rpy.y())));
    const bool calf_collision = diagnostics.hasCalfCollision(data.get());
    if (calf_collision && first_calf_collision_frame < 0) {
      first_calf_collision_frame = frame;
    }
    calf_collision_frames += calf_collision ? 1U : 0U;
  }
  const Vec3<float> finish = diagnostics.bodyPosition(data.get());
  RecordProperty("jump_height_gain_m", std::to_string(maximum_height - start.z()));
  RecordProperty("jump_forward_displacement_m", std::to_string(finish.x() - start.x()));
  RecordProperty("jump_maximum_tilt_rad", std::to_string(maximum_tilt));
  RecordProperty("jump_final_height_m", std::to_string(finish.z()));
  RecordProperty("jump_calf_collision_frames", static_cast<int>(calf_collision_frames));

  EXPECT_EQ(runner.currentStateName(), FSM_StateName::BALANCE_STAND);
  EXPECT_GT(maximum_height - start.z(), 0.025F);
  EXPECT_GT(finish.x() - start.x(), 0.015F);
  EXPECT_LT(maximum_tilt, 0.35F);
  EXPECT_NEAR(finish.z(), 0.39F, 0.02F);
  EXPECT_EQ(calf_collision_frames, 0U)
    << "first collision frame: " << first_calf_collision_frame;
  EXPECT_LE(rejected_frames, 5);
}
