/**
 * @file SimulationDiagnostics.hpp
 * @brief 正式仿真运行时诊断：比较控制器估计和MuJoCo真值并监测小腿碰地。
 *
 * 这些能力最初只用于端到端测试，现在同时供正式SimulationBridge和测试复用。
 * 诊断只读取仿真状态，不修改控制命令或物理状态。
 */
#ifndef MYMIT_ROBOT_USER_SIMULATION_DIAGNOSTICS_HPP_
#define MYMIT_ROBOT_USER_SIMULATION_DIAGNOSTICS_HPP_

#include <array>
#include <cstddef>
#include <limits>

#include <mujoco/mujoco.h>

#include "controller/OrientationEstimator.hpp"

struct SimulationDiagnosticReport
{
  float maximum_position_error = 0.0F;
  float maximum_velocity_error = 0.0F;
  float maximum_orientation_error = 0.0F;
  float maximum_absolute_pitch = 0.0F;
  float minimum_height = std::numeric_limits<float>::infinity();
  std::size_t calf_collision_frames = 0;
  std::size_t rejected_control_frames = 0;
  std::size_t observed_frames = 0;
  double squared_position_error_sum = 0.0;
  double squared_velocity_error_sum = 0.0;

  float rmsPositionError() const noexcept;
  float rmsVelocityError() const noexcept;
};

class SimulationDiagnostics
{
public:
  explicit SimulationDiagnostics(const mjModel * model);

  void reset() noexcept;
  void observe(
    const mjData * data, const StateEstimate<float> & estimate,
    bool control_valid);

  Vec3<float> bodyPosition(const mjData * data) const;
  Vec3<float> bodyLinearVelocity(const mjData * data) const;
  float bodyYaw(const mjData * data) const;
  float orientationError(
    const mjData * data, const Eigen::Quaternionf & estimate) const;
  bool hasCalfCollision(const mjData * data) const;
  const SimulationDiagnosticReport & report() const noexcept {return report_;}

private:
  static int requireObject(const mjModel * model, int type, const char * name);

  const mjModel * model_ = nullptr;
  int trunk_body_ = -1;
  int floor_geom_ = -1;
  std::array<int, kNumLegs> foot_geoms_{};
  std::array<int, kNumLegs> calf_bodies_{};
  Vec3<float> position_offset_ = Vec3<float>::Zero();
  bool position_offset_initialized_ = false;
  SimulationDiagnosticReport report_{};
};

#endif  // MYMIT_ROBOT_USER_SIMULATION_DIAGNOSTICS_HPP_
