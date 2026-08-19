#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "controller/leg_controller.hpp"
#include "model/quadruped.hpp"
#include "model/robots/dm1.hpp"

namespace
{
using ModelPointer = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPointer = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

constexpr std::array<const char *, kNumLegs> kLegNames{"FR", "FL", "RR", "RL"};
constexpr std::array<const char *, kJointsPerLeg> kJointSuffixes{
  "hip_joint", "thigh_joint", "calf_joint"};

ModelPointer loadDm1()
{
  char error[1024]{};
  ModelPointer model(
    mj_loadXML(MYMIT_ROBOT_DM1_TEST_SCENE_PATH, nullptr, error, sizeof(error)),
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

Vec3<double> vectorAt(const mjtNum * values, int index)
{
  return Eigen::Map<const Vec3<double>>(values + 3 * index);
}
}  // namespace

TEST(Dm1ModelLoad, ProvidesTheCommonControllerInterface)
{
  const auto model = loadDm1();
  EXPECT_EQ(model->nq, 19);
  EXPECT_EQ(model->nv, 18);
  EXPECT_EQ(model->nu, 12);
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_KEY, "home"), 0);

  constexpr std::array<const char *, kNumLegs> legs{"FR", "FL", "RR", "RL"};
  constexpr std::array<const char *, kJointsPerLeg> joint_suffixes{
    "hip_joint", "thigh_joint", "calf_joint"};
  constexpr std::array<const char *, kJointsPerLeg> actuator_suffixes{
    "hip", "thigh", "calf"};
  for (const char * leg : legs) {
    EXPECT_GE(mj_name2id(model.get(), mjOBJ_GEOM, leg), 0);
    EXPECT_GE(mj_name2id(model.get(), mjOBJ_BODY, std::string(leg).append("_calf").c_str()), 0);
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      EXPECT_GE(mj_name2id(
        model.get(), mjOBJ_JOINT,
        (std::string(leg) + "_" + joint_suffixes[joint]).c_str()), 0);
      EXPECT_GE(mj_name2id(
        model.get(), mjOBJ_ACTUATOR,
        (std::string(leg) + "_" + actuator_suffixes[joint]).c_str()), 0);
    }
  }
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_SENSOR, "imu_orientation"), 0);
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_SENSOR, "imu_angular_velocity"), 0);
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_SENSOR, "imu_linear_acceleration"), 0);
}

TEST(Dm1ModelLoad, ControllerFactoryUsesDm1Parameters)
{
  const auto model = makeQuadruped<float>(RobotType::DM_BOT1);
  EXPECT_EQ(model.robotType(), RobotType::DM_BOT1);
  EXPECT_FLOAT_EQ(model.nominalBodyHeight(), 0.39F);
  EXPECT_TRUE(model.isValid());
}

TEST(Dm1ModelLoad, FootKinematicsAndJacobianMatchMujoco)
{
  auto mujoco_model = loadDm1();
  DataPointer data(mj_makeData(mujoco_model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);
  const auto quadruped = robots::dm1::makeModel<double>();

  // 基座固定为单位姿态，使 MuJoCo 世界坐标减去 Hip 安装点后，直接等于
  // 控制器使用的单腿局部坐标。每条腿采用不同关节角，覆盖前/后、左/右链。
  data->qpos[0] = 0.0;
  data->qpos[1] = 0.0;
  data->qpos[2] = 0.0;
  data->qpos[3] = 1.0;
  data->qpos[4] = 0.0;
  data->qpos[5] = 0.0;
  data->qpos[6] = 0.0;

  std::array<Vec3<double>, kNumLegs> joint_positions{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    joint_positions[leg] <<
      -0.16 + 0.10 * static_cast<double>(leg),
      -0.95 + 0.09 * static_cast<double>(leg),
      -0.62 - 0.11 * static_cast<double>(leg);
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const int joint_id = namedId(
        mujoco_model.get(), mjOBJ_JOINT,
        std::string(kLegNames[leg]) + "_" + kJointSuffixes[joint]);
      data->qpos[mujoco_model->jnt_qposadr[joint_id]] =
        joint_positions[leg][static_cast<Eigen::Index>(joint)];
    }
  }
  mj_forward(mujoco_model.get(), data.get());

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const LegId leg_id = static_cast<LegId>(leg);
    const int foot_id = namedId(mujoco_model.get(), mjOBJ_GEOM, kLegNames[leg]);
    const Vec3<double> mujoco_foot_from_hip =
      vectorAt(data->geom_xpos, foot_id) - quadruped.hipLocation(leg_id);
    Vec3<double> controller_foot;
    Mat3<double> controller_jacobian;
    computeLegJacobianAndPosition(
      quadruped, joint_positions[leg], &controller_jacobian,
      &controller_foot, leg_id);
    EXPECT_TRUE(controller_foot.isApprox(mujoco_foot_from_hip, 1.0e-10))
      << "leg " << kLegNames[leg] << " controller="
      << controller_foot.transpose() << " mujoco="
      << mujoco_foot_from_hip.transpose();

    // MuJoCo 的点雅可比使用整机自由度列；取本腿三个关节列后与控制器解析值比较。
    std::vector<mjtNum> jacobian(3 * mujoco_model->nv, 0.0);
    mj_jacGeom(mujoco_model.get(), data.get(), jacobian.data(), nullptr, foot_id);
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const int joint_id = namedId(
        mujoco_model.get(), mjOBJ_JOINT,
        std::string(kLegNames[leg]) + "_" + kJointSuffixes[joint]);
      const int dof = mujoco_model->jnt_dofadr[joint_id];
      for (int axis = 0; axis < 3; ++axis) {
        EXPECT_NEAR(
          controller_jacobian(axis, static_cast<Eigen::Index>(joint)),
          jacobian[axis * mujoco_model->nv + dof], 1.0e-10)
          << "leg " << kLegNames[leg] << " joint " << joint
          << " axis " << axis;
      }
    }
  }
}
