#include "WBC/WBC_Ctrl/BodyOriTask.hpp"

#include <cmath>
#include <stdexcept>

// 机身姿态任务：由当前姿态到目标姿态的最短旋转构造 3 维角加速度命令。
template<typename T>
BodyOriTask<T>::BodyOriTask(const FloatingBaseModel<T> & model)
: Task<T>(3), model_(&model)
{
  this->Jt_ = DMat<T>::Zero(3, 6 + kNumJoints);
  this->Jt_.template block<3, 3>(0, 0).setIdentity();
  this->JtDotQdot_ = DVec<T>::Zero(3);
}

template<typename T>
void BodyOriTask<T>::validateGain(const Vec3<T> & gain)
{
  if (!gain.allFinite() || (gain.array() < T(0)).any()) {
    throw std::invalid_argument("body orientation task gain must be finite and non-negative");
  }
}

template<typename T>
void BodyOriTask<T>::setKinematicGain(const Vec3<T> & gain)
{
  validateGain(gain);
  kp_kinematic_ = gain;
}

template<typename T>
void BodyOriTask<T>::setProportionalGain(const Vec3<T> & gain)
{
  validateGain(gain);
  kp_ = gain;
}

template<typename T>
void BodyOriTask<T>::setDerivativeGain(const Vec3<T> & gain)
{
  validateGain(gain);
  kd_ = gain;
}

template<typename T>
bool BodyOriTask<T>::update(
  const Eigen::Quaternion<T> & orientation_world_from_body_desired,
  const Vec3<T> & angular_velocity_body_desired,
  const Vec3<T> & angular_acceleration_body_desired)
{
  DVec<T> velocity = angular_velocity_body_desired;
  DVec<T> acceleration = angular_acceleration_body_desired;
  /*在task文件中更新*/
  return this->UpdateTask(
    &orientation_world_from_body_desired, velocity, acceleration);
}

template<typename T>
bool BodyOriTask<T>::_UpdateCommand(
  const void * position_desired, const DVec<T> & velocity_desired,
  const DVec<T> & acceleration_desired)
{
  if (position_desired == nullptr || model_ == nullptr) {return false;}
  const auto & desired_input =
    *static_cast<const Eigen::Quaternion<T> *>(position_desired);
  if (!desired_input.coeffs().allFinite()) {return false;}

  const auto & model_state = model_->getState();
  if (!model_state.bodyOrientation.allFinite() ||
    !model_state.bodyVelocity.allFinite()) {return false;}

  Eigen::Quaternion<T> desired = desired_input;
  Eigen::Quaternion<T> current(
    model_state.bodyOrientation[0], model_state.bodyOrientation[1],
    model_state.bodyOrientation[2], model_state.bodyOrientation[3]);
  if (desired.norm() <= T(1e-8) || current.norm() <= T(1e-8)) {return false;}
  desired.normalize();
  current.normalize();

  // current^-1 * desired 表示机身坐标系下从当前姿态转到目标姿态的旋转。
  Eigen::Quaternion<T> error = current.conjugate() * desired;
  // q 与 -q 表示同一姿态，统一选 w >= 0 的一支可得到较短的旋转路径。
  if (error.w() < T(0)) {error.coeffs() *= T(-1);}
  error.normalize();
  const Eigen::AngleAxis<T> angle_axis(error);
  Vec3<T> orientation_error = Vec3<T>::Zero();
  if (std::isfinite(static_cast<double>(angle_axis.angle())) &&
    angle_axis.axis().allFinite())
  {
    orientation_error = angle_axis.angle() * angle_axis.axis();
  } else {
    return false;
  }
  /*此处为计算重点*/
  const Vec3<T> current_angular_velocity =
    model_state.bodyVelocity.template head<3>();
  const Vec3<T> desired_velocity = velocity_desired.template head<3>();
  const Vec3<T> desired_acceleration = acceleration_desired.template head<3>();
  this->pos_err_ = kp_kinematic_.cwiseProduct(orientation_error);
  this->vel_des_ = desired_velocity;
  this->acc_des_ = desired_acceleration;
  this->op_cmd_ = kp_.cwiseProduct(orientation_error) +
    // 任务命令采用“姿态 PD + 目标前馈角加速度”。
    kd_.cwiseProduct(desired_velocity - current_angular_velocity) +
    desired_acceleration;
  return this->pos_err_.allFinite() && this->op_cmd_.allFinite();
}

/** 姿态任务直接作用于浮动基座角速度 */
template<typename T>
bool BodyOriTask<T>::_UpdateTaskJacobian()
{
  this->Jt_.setZero(3, 6 + kNumJoints);
  this->Jt_.template block<3, 3>(0, 0).setIdentity();
  return true;
}

/*当前实现假设姿态任务雅可比结构恒定，因此不计算非零的 \(\dot J_R\dot q\)。
                               在完整刚体动力学建模中，任务加速度关系为：*/
template<typename T>
bool BodyOriTask<T>::_UpdateTaskJDotQdot()
{
  this->JtDotQdot_.setZero(3);
  return true;
}

template class BodyOriTask<float>;
template class BodyOriTask<double>;
