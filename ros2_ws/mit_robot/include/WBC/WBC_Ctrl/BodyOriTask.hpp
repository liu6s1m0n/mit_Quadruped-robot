/**
 * @file BodyOriTask.hpp
 * @brief 机身姿态任务，将四元数姿态误差转换为三维角加速度命令。
 */
#ifndef MYMIT_ROBOT_WBC_CTRL_BODY_ORI_TASK_HPP_
#define MYMIT_ROBOT_WBC_CTRL_BODY_ORI_TASK_HPP_

#include <eigen3/Eigen/Geometry>

#include "WBC/FloatingBaseModel.h"
#include "WBC/Task.hpp"
#include "model/robot_types.hpp"

/** Three-axis floating-base orientation task expressed in the body frame. */
template<typename T>
class BodyOriTask final : public Task<T>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief 创建三维机身姿态任务并绑定浮动基模型。 */
  explicit BodyOriTask(const FloatingBaseModel<T> & model);
  ~BodyOriTask() override = default;

  /**
   * @brief 设置期望姿态、角速度和角加速度并更新任务数据。
   * @param orientation_world_from_body_desired 世界系到机身系的期望四元数。
   * @param angular_velocity_body_desired 机身坐标系期望角速度，rad/s。
   * @param angular_acceleration_body_desired 机身坐标系期望角加速度，rad/s^2。
   * @return 输入有效且任务命令、雅可比均有限时返回 true。
   */
  bool update(
    const Eigen::Quaternion<T> & orientation_world_from_body_desired,
    const Vec3<T> & angular_velocity_body_desired = Vec3<T>::Zero(),
    const Vec3<T> & angular_acceleration_body_desired = Vec3<T>::Zero());

  /** @brief 设置姿态误差到运动学参考的增益。 */
  void setKinematicGain(const Vec3<T> & gain);
  /** @brief 设置姿态比例反馈增益。 */
  void setProportionalGain(const Vec3<T> & gain);
  /** @brief 设置角速度微分反馈增益。 */
  void setDerivativeGain(const Vec3<T> & gain);
  /** @brief 获取运动学姿态误差增益。 */
  const Vec3<T> & kinematicGain() const noexcept {return kp_kinematic_;}
  /** @brief 获取姿态比例反馈增益。 */
  const Vec3<T> & proportionalGain() const noexcept {return kp_;}
  /** @brief 获取角速度微分反馈增益。 */
  const Vec3<T> & derivativeGain() const noexcept {return kd_;}

private:
  bool _UpdateCommand(
    const void * position_desired, const DVec<T> & velocity_desired,
    const DVec<T> & acceleration_desired) override;
  bool _UpdateTaskJacobian() override;
  bool _UpdateTaskJDotQdot() override;
  bool _AdditionalUpdate() override {return true;}

  static void validateGain(const Vec3<T> & gain);

  const FloatingBaseModel<T> * model_;  ///< 非拥有型浮动基模型指针。
  Vec3<T> kp_kinematic_ = Vec3<T>::Ones();       ///< 运动学姿态误差增益。
  Vec3<T> kp_ = Vec3<T>::Constant(T(50));        ///< 姿态比例增益。
  Vec3<T> kd_ = Vec3<T>::Ones();                 ///< 角速度微分增益。
};

extern template class BodyOriTask<float>;
extern template class BodyOriTask<double>;

#endif  // MYMIT_ROBOT_WBC_CTRL_BODY_ORI_TASK_HPP_
