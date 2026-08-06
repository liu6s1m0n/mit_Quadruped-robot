/*! @file FootSwingTrajectory.hpp
 *  @brief 单足摆动阶段的三次 Bézier 轨迹生成器。
 *
 *  本类根据摆动起点、落脚点、抬脚高度、摆动相位和摆动总时长，计算足端在
 *  腿部/机身坐标系中的位置、速度和加速度。坐标系由调用方决定，但起点和
 *  终点必须使用同一个坐标系，长度单位统一为 m。
 */

#ifndef MYMIT_ROBOT_CONTROLLER_FOOT_SWING_TRAJECTORY_HPP_
#define MYMIT_ROBOT_CONTROLLER_FOOT_SWING_TRAJECTORY_HPP_

#include "cppTypes.h"

/**
 * @brief 一只脚在摆动阶段的位置、速度和加速度轨迹。
 *
 * 水平方向使用一条完整的三次 Bézier 曲线从起点移动到终点；竖直 z 方向分为
 * 两段：前半程从起点抬到最高点，后半程从最高点落到终点。最高点定义为
 * initial_position.z() + height。
 *
 * 模板参数 T 通常为 float；当前模型库同时显式提供 float 和 double 版本。
 */
template<typename T>
class FootSwingTrajectory
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief 创建一条全部参数和输出均为零的摆动轨迹。 */
  FootSwingTrajectory();

  /**
   * @brief 清空起点、终点、抬脚高度以及最近一次计算结果。
   *
   * 重新规划一次完全不同的摆动轨迹前可以调用此函数。
   */
  void reset();

  /**
   * @brief 设置摆动开始时的足端位置。
   * @param initial_position 足端起点 [x, y, z]，单位 m。
   */
  void setInitialPosition(const Vec3<T> & initial_position);

  /**
   * @brief 设置摆动结束时的目标落脚位置。
   * @param final_position 足端终点 [x, y, z]，单位 m。
   */
  void setFinalPosition(const Vec3<T> & final_position);

  /**
   * @brief 设置足端相对起点的抬脚高度。
   * @param height 非负高度，单位 m；摆动相位为 0.5 时到达最高点。
   */
  void setHeight(T height);

  /**
   * @brief 计算指定摆动相位处的足端轨迹。
   *
   * phase=0 对应起点，phase=0.5 对应最高点，phase=1 对应终点。为避免
   * 控制周期累计误差导致越界，有限的 phase 会自动限制到 [0, 1]。
   *
   * @param phase 当前摆动相位。
   * @param swing_time 完整摆动过程持续时间，单位 s，必须为有限正数。
   * @throws std::invalid_argument 输入包含 NaN/Inf、负抬脚高度或非法时长。
   */
  void computeSwingTrajectoryBezier(T phase, T swing_time);

  /** @brief 返回最近一次计算的足端位置，单位 m。 */
  Vec3<T> getPosition() const noexcept {return position_;}

  /** @brief 返回最近一次计算的足端速度，单位 m/s。 */
  Vec3<T> getVelocity() const noexcept {return velocity_;}

  /** @brief 返回最近一次计算的足端加速度，单位 m/s^2。 */
  Vec3<T> getAcceleration() const noexcept {return acceleration_;}

private:
  Vec3<T> initial_position_ = Vec3<T>::Zero();  ///< 摆动起点，m。
  Vec3<T> final_position_ = Vec3<T>::Zero();    ///< 目标落脚点，m。
  Vec3<T> position_ = Vec3<T>::Zero();          ///< 当前相位位置，m。
  Vec3<T> velocity_ = Vec3<T>::Zero();          ///< 当前相位速度，m/s。
  Vec3<T> acceleration_ = Vec3<T>::Zero();      ///< 当前相位加速度，m/s^2。
  T height_ = T(0);                             ///< 相对起点的抬脚高度，m。
};

#endif  // MYMIT_ROBOT_CONTROLLER_FOOT_SWING_TRAJECTORY_HPP_
