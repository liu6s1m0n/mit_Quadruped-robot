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

  explicit BodyOriTask(const FloatingBaseModel<T> & model);
  ~BodyOriTask() override = default;

  bool update(
    const Eigen::Quaternion<T> & orientation_world_from_body_desired,
    const Vec3<T> & angular_velocity_body_desired = Vec3<T>::Zero(),
    const Vec3<T> & angular_acceleration_body_desired = Vec3<T>::Zero());

  void setKinematicGain(const Vec3<T> & gain);
  void setProportionalGain(const Vec3<T> & gain);
  void setDerivativeGain(const Vec3<T> & gain);

  const Vec3<T> & kinematicGain() const noexcept {return kp_kinematic_;}
  const Vec3<T> & proportionalGain() const noexcept {return kp_;}
  const Vec3<T> & derivativeGain() const noexcept {return kd_;}

private:
  bool _UpdateCommand(
    const void * position_desired, const DVec<T> & velocity_desired,
    const DVec<T> & acceleration_desired) override;
  bool _UpdateTaskJacobian() override;
  bool _UpdateTaskJDotQdot() override;
  bool _AdditionalUpdate() override {return true;}

  static void validateGain(const Vec3<T> & gain);

  const FloatingBaseModel<T> * model_;
  Vec3<T> kp_kinematic_ = Vec3<T>::Ones();
  Vec3<T> kp_ = Vec3<T>::Constant(T(50));
  Vec3<T> kd_ = Vec3<T>::Ones();
};

extern template class BodyOriTask<float>;
extern template class BodyOriTask<double>;

#endif  // MYMIT_ROBOT_WBC_CTRL_BODY_ORI_TASK_HPP_
