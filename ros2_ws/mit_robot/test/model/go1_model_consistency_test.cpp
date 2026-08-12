#include <array>
#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "controller/leg_controller.hpp"
#include "model/robots/unitree_go1.hpp"

namespace
{

using ModelPointer = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPointer = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

constexpr std::array<const char *, kNumLegs> kLegNames{"FR", "FL", "RR", "RL"};
constexpr std::array<const char *, kJointsPerLeg> kJointSuffixes{
  "hip_joint", "thigh_joint", "calf_joint"};
constexpr std::array<const char *, kJointsPerLeg> kActuatorSuffixes{
  "hip", "thigh", "calf"};

ModelPointer loadGo1Model()
{
  char error[1024]{};
  ModelPointer model(
    mj_loadXML(MYMIT_ROBOT_TEST_SCENE_PATH, nullptr, error, sizeof(error)),
    &mj_deleteModel);
  if (!model) {throw std::runtime_error(error);}
  return model;
}

int namedId(const mjModel * model, int object_type, const std::string & name)
{
  const int id = mj_name2id(model, object_type, name.c_str());
  if (id < 0) {throw std::runtime_error("MuJoCo object not found: " + name);}
  return id;
}

Vec3<double> vectorAt(const mjtNum * values, int index)
{
  return Eigen::Map<const Vec3<double>>(values + 3 * index);
}

RigidBodyInertia<double> mujocoBodyInertia(const mjModel * model, int body)
{
  const mjtNum * quaternion = model->body_iquat + 4 * body;
  return makeRigidBodyInertia(
    model->body_mass[body], vectorAt(model->body_ipos, body),
    Eigen::Quaternion<double>(
      quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
    vectorAt(model->body_inertia, body));
}

void expectInertiaNear(
  const RigidBodyInertia<double> & actual,
  const RigidBodyInertia<double> & expected)
{
  EXPECT_NEAR(actual.mass, expected.mass, 1.0e-12);
  EXPECT_TRUE(actual.center_of_mass.isApprox(expected.center_of_mass, 1.0e-12));
  EXPECT_TRUE(actual.inertia_com.isApprox(expected.inertia_com, 1.0e-10));
}

const RigidBodyInertia<double> & legBodyInertia(
  const LegModelParameters<double> & leg, std::size_t joint)
{
  if (joint == 0) {return leg.hip_inertia;}
  if (joint == 1) {return leg.thigh_inertia;}
  return leg.calf_inertia;
}

}  // namespace

TEST(Go1ModelConsistency, ControllerParametersMatchCompiledMujocoModel)
{
  auto mujoco_model = loadGo1Model();
  const auto controller_model = robots::unitree_go1::makeModel<double>();

  const int trunk = namedId(mujoco_model.get(), mjOBJ_BODY, "trunk");
  expectInertiaNear(
    controller_model.bodyInertia(),
    mujocoBodyInertia(mujoco_model.get(), trunk));

  double total_mass = mujoco_model->body_mass[trunk];
  for (std::size_t leg_index = 0; leg_index < kNumLegs; ++leg_index) {
    const auto & leg = controller_model.leg(static_cast<LegId>(leg_index));
    const std::string prefix = kLegNames[leg_index];
    const int hip_body = namedId(mujoco_model.get(), mjOBJ_BODY, prefix + "_hip");
    EXPECT_TRUE(
      leg.hip_location_body.isApprox(vectorAt(mujoco_model->body_pos, hip_body), 1.0e-12));

    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const std::string body_suffix = joint == 0 ? "hip" : joint == 1 ? "thigh" : "calf";
      const int body = namedId(
        mujoco_model.get(), mjOBJ_BODY, prefix + "_" + body_suffix);
      const int joint_id = namedId(
        mujoco_model.get(), mjOBJ_JOINT,
        prefix + "_" + kJointSuffixes[joint]);
      const int dof = mujoco_model->jnt_dofadr[joint_id];
      const int actuator = namedId(
        mujoco_model.get(), mjOBJ_ACTUATOR,
        prefix + "_" + kActuatorSuffixes[joint]);

      expectInertiaNear(
        legBodyInertia(leg, joint),
        mujocoBodyInertia(mujoco_model.get(), body));
      total_mass += mujoco_model->body_mass[body];
      EXPECT_TRUE(
        leg.joints.joint_axes.col(static_cast<Eigen::Index>(joint)).isApprox(
          vectorAt(mujoco_model->jnt_axis, joint_id), 1.0e-12));
      EXPECT_NEAR(
        leg.joints.lower_limit[static_cast<Eigen::Index>(joint)],
        mujoco_model->jnt_range[2 * joint_id], 1.0e-12);
      EXPECT_NEAR(
        leg.joints.upper_limit[static_cast<Eigen::Index>(joint)],
        mujoco_model->jnt_range[2 * joint_id + 1], 1.0e-12);
      EXPECT_NEAR(
        leg.joints.damping[static_cast<Eigen::Index>(joint)],
        mujoco_model->dof_damping[dof], 1.0e-12);
      EXPECT_NEAR(
        leg.joints.friction_loss[static_cast<Eigen::Index>(joint)],
        mujoco_model->dof_frictionloss[dof], 1.0e-12);
      EXPECT_NEAR(
        leg.joints.armature[static_cast<Eigen::Index>(joint)],
        mujoco_model->dof_armature[dof], 1.0e-12);
      EXPECT_NEAR(
        leg.joints.torque_limit[static_cast<Eigen::Index>(joint)],
        mujoco_model->actuator_forcerange[2 * actuator + 1], 1.0e-12);
    }

    const int foot = namedId(mujoco_model.get(), mjOBJ_GEOM, prefix);
    EXPECT_NEAR(leg.foot_radius, mujoco_model->geom_size[3 * foot], 1.0e-12);
    EXPECT_TRUE(
      leg.foot_friction.isApprox(vectorAt(mujoco_model->geom_friction, foot), 1.0e-12));
  }
  EXPECT_NEAR(controller_model.totalMass(), total_mass, 1.0e-12);
}

TEST(Go1ModelConsistency, KinematicsAndMassMatrixMatchMujoco)
{
  auto mujoco_model = loadGo1Model();
  DataPointer data(mj_makeData(mujoco_model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);
  const auto quadruped = robots::unitree_go1::makeModel<double>();
  auto dynamics = robots::unitree_go1::makeFloatingBaseModel<double>();

  FBModelState<double> state;
  state.bodyOrientation << 1.0, 0.0, 0.0, 0.0;
  state.bodyPosition.setZero();
  state.bodyVelocity.setZero();
  state.q = DVec<double>::Zero(kNumJoints);
  state.qd = DVec<double>::Zero(kNumJoints);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const Eigen::Index offset = static_cast<Eigen::Index>(leg * kJointsPerLeg);
    state.q.segment<3>(offset) <<
      -0.12 + 0.08 * static_cast<double>(leg),
      0.65 + 0.07 * static_cast<double>(leg),
      -1.45 - 0.09 * static_cast<double>(leg);
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const int joint_id = namedId(
        mujoco_model.get(), mjOBJ_JOINT,
        std::string(kLegNames[leg]) + "_" + kJointSuffixes[joint]);
      data->qpos[mujoco_model->jnt_qposadr[joint_id]] =
        state.q[offset + static_cast<Eigen::Index>(joint)];
    }
  }
  data->qpos[0] = 0.0;
  data->qpos[1] = 0.0;
  data->qpos[2] = 0.0;
  data->qpos[3] = 1.0;
  data->qpos[4] = 0.0;
  data->qpos[5] = 0.0;
  data->qpos[6] = 0.0;
  mj_forward(mujoco_model.get(), data.get());

  dynamics.setState(state);
  dynamics.forwardKinematics();
  const auto & dynamics_feet = dynamics.getGroundContactPositions();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const int foot = namedId(mujoco_model.get(), mjOBJ_GEOM, kLegNames[leg]);
    const Vec3<double> mujoco_foot = vectorAt(data->geom_xpos, foot);
    EXPECT_TRUE(dynamics_feet[leg].isApprox(mujoco_foot, 1.0e-10));

    const Vec3<double> joint_position = state.q.segment<3>(
      static_cast<Eigen::Index>(leg * kJointsPerLeg));
    Vec3<double> analytic_foot;
    computeLegJacobianAndPosition(
      quadruped, joint_position,
      static_cast<Mat3<double> *>(nullptr), &analytic_foot,
      static_cast<LegId>(leg));
    EXPECT_TRUE(
      (quadruped.hipLocation(static_cast<LegId>(leg)) + analytic_foot)
      .isApprox(mujoco_foot, 1.0e-10));
  }

  DMat<double> mujoco_mass = DMat<double>::Zero(mujoco_model->nv, mujoco_model->nv);
  mj_fullM(mujoco_model.get(), data.get(), mujoco_mass.data());
  const DMat<double> controller_mass = dynamics.massMatrix();
  ASSERT_EQ(controller_mass.rows(), mujoco_model->nv);
  ASSERT_EQ(controller_mass.cols(), mujoco_model->nv);
  // FloatingBaseModel 使用 [角速度, 线速度, 关节]，MuJoCo freejoint 使用
  // [线速度, 角速度, 关节]；在单位基座姿态下只需交换前两个三维块。
  std::array<int, 6 + kNumJoints> mujoco_index{};
  mujoco_index[0] = 3;
  mujoco_index[1] = 4;
  mujoco_index[2] = 5;
  mujoco_index[3] = 0;
  mujoco_index[4] = 1;
  mujoco_index[5] = 2;
  for (std::size_t joint = 0; joint < kNumJoints; ++joint) {
    mujoco_index[6 + joint] = 6 + static_cast<int>(joint);
  }
  DMat<double> reordered_mass = DMat<double>::Zero(mujoco_model->nv, mujoco_model->nv);
  for (int row = 0; row < mujoco_model->nv; ++row) {
    for (int column = 0; column < mujoco_model->nv; ++column) {
      reordered_mass(row, column) =
        mujoco_mass(
        mujoco_index[static_cast<std::size_t>(row)],
        mujoco_index[static_cast<std::size_t>(column)]);
    }
  }
  EXPECT_TRUE(controller_mass.isApprox(reordered_mass, 1.0e-9));
}

TEST(Go1ModelConsistency, RotatedBaseKinematicsMatchMujocoTruth)
{
  auto mujoco_model = loadGo1Model();
  DataPointer data(mj_makeData(mujoco_model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);
  auto dynamics = robots::unitree_go1::makeFloatingBaseModel<double>();

  FBModelState<double> state;
  const Eigen::Quaterniond orientation =
    Eigen::AngleAxisd(0.55, Vec3<double>::UnitZ()) *
    Eigen::AngleAxisd(-0.18, Vec3<double>::UnitY()) *
    Eigen::AngleAxisd(0.12, Vec3<double>::UnitX());
  state.bodyOrientation << orientation.w(), orientation.x(), orientation.y(), orientation.z();
  state.bodyPosition << 0.37, -0.21, 0.42;
  state.bodyVelocity.setZero();
  state.q = DVec<double>::Zero(kNumJoints);
  state.qd = DVec<double>::Zero(kNumJoints);

  data->qpos[0] = state.bodyPosition.x();
  data->qpos[1] = state.bodyPosition.y();
  data->qpos[2] = state.bodyPosition.z();
  data->qpos[3] = orientation.w();
  data->qpos[4] = orientation.x();
  data->qpos[5] = orientation.y();
  data->qpos[6] = orientation.z();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const Eigen::Index offset = static_cast<Eigen::Index>(leg * kJointsPerLeg);
    state.q.segment<3>(offset) <<
      -0.08 + 0.05 * static_cast<double>(leg),
      0.72 + 0.04 * static_cast<double>(leg),
      -1.52 - 0.06 * static_cast<double>(leg);
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const int joint_id = namedId(
        mujoco_model.get(), mjOBJ_JOINT,
        std::string(kLegNames[leg]) + "_" + kJointSuffixes[joint]);
      data->qpos[mujoco_model->jnt_qposadr[joint_id]] =
        state.q[offset + static_cast<Eigen::Index>(joint)];
    }
  }
  mj_forward(mujoco_model.get(), data.get());

  dynamics.setState(state);
  dynamics.forwardKinematics();
  const auto & controller_feet = dynamics.getGroundContactPositions();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const int foot = namedId(mujoco_model.get(), mjOBJ_GEOM, kLegNames[leg]);
    EXPECT_TRUE(
      controller_feet[leg].isApprox(vectorAt(data->geom_xpos, foot), 1.0e-10));
  }
}
