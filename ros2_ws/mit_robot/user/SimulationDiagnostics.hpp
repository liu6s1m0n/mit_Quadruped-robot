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
  float maximum_position_error = 0.0F;      ///< 估计位置与 MuJoCo 真值的最大距离，m。
  float maximum_velocity_error = 0.0F;      ///< 估计线速度与真值的最大差值，m/s。
  float maximum_orientation_error = 0.0F;   ///< 估计姿态与真值的最大旋转角误差，rad。
  float maximum_absolute_pitch = 0.0F;      ///< 机身俯仰角绝对值的历史最大值，rad。
  float minimum_height = std::numeric_limits<float>::infinity();  ///< 机身最低离地高度，m。
  std::size_t calf_collision_frames = 0;    ///< 检测到任一小腿碰地的仿真帧数。
  std::size_t rejected_control_frames = 0;  ///< 控制器输出无效而被安全层拒绝的帧数。
  std::size_t observed_frames = 0;          ///< 已纳入统计的有效仿真帧总数。
  double squared_position_error_sum = 0.0;  ///< 位置误差平方累计值，用于计算 RMS。
  double squared_velocity_error_sum = 0.0;  ///< 速度误差平方累计值，用于计算 RMS。

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

  const mjModel * model_ = nullptr;  ///< 非拥有型 MuJoCo 模型指针，用于查询对象和状态布局。
  int trunk_body_ = -1;             ///< trunk 刚体在 model->body_* 数组中的编号。
  int floor_geom_ = -1;             ///< 地面几何体在 model->geom_* 数组中的编号。
  std::array<int, kNumLegs> foot_geoms_{};   ///< FR/FL/RR/RL 足端几何体编号。
  std::array<int, kNumLegs> calf_bodies_{};  ///< FR/FL/RR/RL 小腿刚体编号。
  Vec3<float> position_offset_ = Vec3<float>::Zero();  ///< 首帧对齐估计坐标系和世界坐标系的平移量，m。
  bool position_offset_initialized_ = false;  ///< position_offset_ 是否已由首帧建立。
  SimulationDiagnosticReport report_{};       ///< 从 reset() 后累计到当前帧的诊断结果。
};

#endif  // MYMIT_ROBOT_USER_SIMULATION_DIAGNOSTICS_HPP_
