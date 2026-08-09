#include "WBC/LocomotionCtrl/LocomotionCtrl.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "orientation_tools.h"

template<typename T>
LocomotionCtrlData<T>::LocomotionCtrlData() noexcept
{
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    pFoot_des[leg].setZero();
    vFoot_des[leg].setZero();
    aFoot_des[leg].setZero();
    Fr_des[leg].setZero();
  }
}

template<typename T>
bool LocomotionCtrlData<T>::allFinite() const noexcept
{
  if (!pBody_des.allFinite() || !vBody_des.allFinite() ||
    !aBody_des.allFinite() || !pBody_RPY_des.allFinite() ||
    !vBody_Ori_des.allFinite() || !contact_state.allFinite())
  {
    return false;
  }
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    if (!pFoot_des[leg].allFinite() || !vFoot_des[leg].allFinite() ||
      !aFoot_des[leg].allFinite() || !Fr_des[leg].allFinite())
    {
      return false;
    }
  }
  return true;
}

template<typename T>
LocomotionCtrl<T>::LocomotionCtrl(FloatingBaseModel<T> model)
: WBC_Ctrl<T>(std::move(model))
{
  const auto & foot_indices = this->model().getFootIndices();
  if (foot_indices.size() != kNumLegs) {
    throw std::invalid_argument("locomotion controller requires exactly four foot contacts");
  }

  body_position_task_ = std::make_unique<BodyPosTask<T>>(&this->model());
  body_orientation_task_ = std::make_unique<BodyOriTask<T>>(this->model());
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    foot_contact_indices_[leg] = static_cast<std::size_t>(foot_indices[leg]);
    foot_contacts_[leg] =
      std::make_unique<SingleContact<T>>(this->model(), foot_contact_indices_[leg]);
    foot_tasks_[leg] =
      std::make_unique<LinkPosTask<T>>(this->model(), foot_contact_indices_[leg]);
  }
}

template<typename T>
void LocomotionCtrl<T>::setBodyPositionGains(const Vec3<T> & kp, const Vec3<T> & kd)
{
  if (!kp.allFinite() || !kd.allFinite() || (kp.array() < T(0)).any() ||
    (kd.array() < T(0)).any())
  {
    throw std::invalid_argument("body position gains must be finite and non-negative");
  }
  body_position_task_->_Kp = kp;
  body_position_task_->_Kd = kd;
}

template<typename T>
void LocomotionCtrl<T>::setBodyOrientationGains(
  const Vec3<T> & kp, const Vec3<T> & kd)
{
  body_orientation_task_->setProportionalGain(kp);
  body_orientation_task_->setDerivativeGain(kd);
}

template<typename T>
void LocomotionCtrl<T>::setFootPositionGains(const Vec3<T> & kp, const Vec3<T> & kd)
{
  for (auto & task : foot_tasks_) {
    task->setProportionalGain(kp);
    task->setDerivativeGain(kd);
  }
}

template<typename T>
void LocomotionCtrl<T>::setMaxNormalForce(T max_fz)
{
  if (!std::isfinite(static_cast<double>(max_fz)) || max_fz <= T(0)) {
    throw std::invalid_argument("maximum normal force must be finite and positive");
  }
  for (auto & contact : foot_contacts_) {contact->setMaxFz(max_fz);}
}

template<typename T>
bool LocomotionCtrl<T>::prepareTasksAndContacts(const void * input)
{
  if (input == nullptr) {return false;}
  const auto & input_data = *static_cast<const LocomotionCtrlData<T> *>(input);
  if (!input_data.allFinite()) {return false;}
  active_contact_state_ = input_data.contact_state;

  const Quat<T> quaternion = ori::rpyToQuat(input_data.pBody_RPY_des);
  const Eigen::Quaternion<T> desired_orientation(
    quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
  if (!body_orientation_task_->update(
      desired_orientation, input_data.vBody_Ori_des, Vec3<T>::Zero()))
  {
    return false;
  }

  DVec<T> body_velocity = input_data.vBody_des;
  DVec<T> body_acceleration = input_data.aBody_des;
  if (!body_position_task_->UpdateTask(
      &input_data.pBody_des, body_velocity, body_acceleration))
  {
    return false;
  }
  this->addTask(*body_orientation_task_);
  this->addTask(*body_position_task_);

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    if (input_data.contact_state[leg] > T(0)) {
      DVec<T> desired_force = input_data.Fr_des[leg];
      foot_contacts_[leg]->setRFDesired(desired_force);
      if (!foot_contacts_[leg]->UpdateContactSpec()) {return false;}
      this->addContact(*foot_contacts_[leg]);
    } else {
      if (!foot_tasks_[leg]->update(
          input_data.pFoot_des[leg], input_data.vFoot_des[leg],
          input_data.aFoot_des[leg]))
      {
        return false;
      }
      this->addTask(*foot_tasks_[leg]);
    }
  }
  return true;
}

template<typename T>
std::array<Vec3<T>, kNumLegs> LocomotionCtrl<T>::reactionForces() const
{
  std::array<Vec3<T>, kNumLegs> forces{};
  for (auto & force : forces) {force.setZero();}
  if (!this->result().valid) {return forces;}

  Eigen::Index offset = 0;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    if (active_contact_state_[leg] > T(0)) {
      if (offset + 3 > this->result().reaction_force.size()) {
        for (auto & force : forces) {force.setZero();}
        return forces;
      }
      forces[leg] = this->result().reaction_force.template segment<3>(offset);
      offset += 3;
    }
  }
  return forces;
}

template struct LocomotionCtrlData<float>;
template struct LocomotionCtrlData<double>;
template class LocomotionCtrl<float>;
