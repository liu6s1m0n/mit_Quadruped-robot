#include "WBC/WBC_Ctrl/LinkPosTask.hpp"

#include <stdexcept>

template<typename T>
LinkPosTask<T>::LinkPosTask(
  const FloatingBaseModel<T> & model, std::size_t contact_point_index,
  bool include_floating_base)
: Task<T>(3), model_(&model), contact_point_index_(contact_point_index),
  include_floating_base_(include_floating_base)
{
  if (contact_point_index_ >= model_->getNumGroundContacts()) {
    throw std::out_of_range("link position task contact-point index is out of range");
  }
  this->Jt_ = DMat<T>::Zero(3, model_->getNumDof());
  this->JtDotQdot_ = DVec<T>::Zero(3);
}

template<typename T>
void LinkPosTask<T>::validateGain(const Vec3<T> & gain)
{
  if (!gain.allFinite() || (gain.array() < T(0)).any()) {
    throw std::invalid_argument("link position task gain must be finite and non-negative");
  }
}

template<typename T>
void LinkPosTask<T>::setKinematicGain(const Vec3<T> & gain)
{
  validateGain(gain);
  kp_kinematic_ = gain;
}

template<typename T>
void LinkPosTask<T>::setProportionalGain(const Vec3<T> & gain)
{
  validateGain(gain);
  kp_ = gain;
}

template<typename T>
void LinkPosTask<T>::setDerivativeGain(const Vec3<T> & gain)
{
  validateGain(gain);
  kd_ = gain;
}

template<typename T>
bool LinkPosTask<T>::update(
  const Vec3<T> & position_world_desired,
  const Vec3<T> & velocity_world_desired,
  const Vec3<T> & acceleration_world_desired)
{
  DVec<T> velocity = velocity_world_desired;
  DVec<T> acceleration = acceleration_world_desired;
  return this->UpdateTask(&position_world_desired, velocity, acceleration);
}

template<typename T>
bool LinkPosTask<T>::_UpdateCommand(
  const void * position_desired, const DVec<T> & velocity_desired,
  const DVec<T> & acceleration_desired)
{
  if (position_desired == nullptr || model_ == nullptr) {return false;}
  const auto & positions = model_->getGroundContactPositions();
  const auto & velocities = model_->getGroundContactVelocities();
  if (contact_point_index_ >= positions.size() || contact_point_index_ >= velocities.size()) {
    return false;
  }
  const auto & desired = *static_cast<const Vec3<T> *>(position_desired);
  const Vec3<T> & current_position = positions[contact_point_index_];
  const Vec3<T> & current_velocity = velocities[contact_point_index_];
  if (!desired.allFinite() || !current_position.allFinite() ||
    !current_velocity.allFinite()) {return false;}

  const Vec3<T> position_error = desired - current_position;
  const Vec3<T> desired_velocity = velocity_desired.template head<3>();
  const Vec3<T> desired_acceleration = acceleration_desired.template head<3>();
  this->pos_err_ = kp_kinematic_.cwiseProduct(position_error);
  this->vel_des_ = desired_velocity;
  this->acc_des_ = desired_acceleration;
  this->op_cmd_ = kp_.cwiseProduct(position_error) +
    kd_.cwiseProduct(desired_velocity - current_velocity) + desired_acceleration;
  return this->op_cmd_.allFinite();
}

template<typename T>
bool LinkPosTask<T>::_UpdateTaskJacobian()
{
  const auto & jacobians = model_->getContactJacobians();
  if (contact_point_index_ >= jacobians.size()) {return false;}
  this->Jt_ = jacobians[contact_point_index_];
  if (!include_floating_base_) {this->Jt_.leftCols(6).setZero();}
  return this->Jt_.rows() == 3 &&
    this->Jt_.cols() == static_cast<Eigen::Index>(model_->getNumDof()) &&
    this->Jt_.allFinite();
}

template<typename T>
bool LinkPosTask<T>::_UpdateTaskJDotQdot()
{
  const auto & values = model_->getContactJacobianDotQdot();
  if (contact_point_index_ >= values.size()) {return false;}
  this->JtDotQdot_ = values[contact_point_index_];
  return this->JtDotQdot_.allFinite();
}

template class LinkPosTask<float>;
template class LinkPosTask<double>;
