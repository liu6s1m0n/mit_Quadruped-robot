#include "WBC/WBC_Ctrl/BodyPosTask.hpp"

#include <stdexcept>

#include "Utilities/orientation_tools.h"

// 机身位置任务：在世界坐标系中跟踪机身位置、线速度和线加速度参考。
template<typename T>
BodyPosTask<T>::BodyPosTask(const FloatingBaseModel<T> * robot)
: Task<T>(3), _robot_sys(robot)
{
  if (_robot_sys == nullptr) {
    throw std::invalid_argument("body position task requires a robot model");
  }
  this->Jt_ = DMat<T>::Zero(this->dim_task_, _robot_sys->getNumDof());
  this->Jt_.template block<3, 3>(0, 3).setIdentity();
  this->JtDotQdot_ = DVec<T>::Zero(this->dim_task_);
  _Kp_kin = DVec<T>::Constant(this->dim_task_, T(1));
  _Kp = DVec<T>::Constant(this->dim_task_, T(50));
  _Kd = DVec<T>::Constant(this->dim_task_, T(1));
}

template<typename T>
bool BodyPosTask<T>::_UpdateCommand(
  const void * position_desired, const DVec<T> & velocity_desired,
  const DVec<T> & acceleration_desired)
{
  if (position_desired == nullptr) {return false;}
  const auto & desired = *static_cast<const Vec3<T> *>(position_desired);
  const auto & state = _robot_sys->getState();
  if (!desired.allFinite() || !state.bodyPosition.allFinite() ||
    !state.bodyVelocity.allFinite())
  {
    return false;
  }

  const Mat3<T> rotation =
    ori::quaternionToRotationMatrix(state.bodyOrientation);
  SVec<T> current_velocity = state.bodyVelocity;
  // 模型的空间速度平移部分采用机身系表示，这里转成世界系后再与目标比较。
  current_velocity.template tail<3>() =
    rotation.transpose() * current_velocity.template tail<3>();

  const Vec3<T> position_error = desired - state.bodyPosition;
  this->pos_err_ = _Kp_kin.cwiseProduct(position_error);
  this->vel_des_ = velocity_desired;
  this->acc_des_ = acceleration_desired;
  // op_cmd 是任务空间的期望加速度：位置 PD 修正叠加前馈加速度。
  this->op_cmd_ = _Kp.cwiseProduct(position_error) +
    _Kd.cwiseProduct(
      velocity_desired - current_velocity.template tail<3>()) +
    acceleration_desired;
  return this->op_cmd_.allFinite();
}

template<typename T>
bool BodyPosTask<T>::_UpdateTaskJacobian()
{
  const Mat3<T> rotation =
    ori::quaternionToRotationMatrix(_robot_sys->getState().bodyOrientation);
  this->Jt_.setZero(3, _robot_sys->getNumDof());
  // 雅可比把广义速度映射成世界系机身线速度。
  this->Jt_.template block<3, 3>(0, 3) = rotation.transpose();
  return this->Jt_.allFinite();
}

template<typename T>
bool BodyPosTask<T>::_UpdateTaskJDotQdot()
{
  this->JtDotQdot_.setZero();
  return true;
}

template class BodyPosTask<float>;
template class BodyPosTask<double>;
