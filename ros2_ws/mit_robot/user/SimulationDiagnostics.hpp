/**
 * @file SimulationDiagnostics.hpp
 * @brief 正式仿真运行时诊断：比较控制器估计和MuJoCo真值并监测小腿碰地。
 *
 * 供正式SimulationBridge持续发布控制器估计误差和接触安全状态。
 * 诊断只读取仿真状态，不修改控制命令或物理状态。
 */
#ifndef MYMIT_ROBOT_USER_SIMULATION_DIAGNOSTICS_HPP_
#define MYMIT_ROBOT_USER_SIMULATION_DIAGNOSTICS_HPP_

#include <array>
#include <cstddef>

#include <mujoco/mujoco.h>

#include "SimulationTypes.hpp"
#include "controller/OrientationEstimator.hpp"

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
