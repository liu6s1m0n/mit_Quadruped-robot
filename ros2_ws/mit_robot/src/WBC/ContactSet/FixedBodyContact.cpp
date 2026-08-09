#include "WBC/ContactSet/FixedBodyContact.hpp"

#include <stdexcept>

namespace
{
constexpr std::size_t kProjectNumQdot = 18;
}

template<typename T>
FixedBodyContact<T>::FixedBodyContact() : FixedBodyContact(kProjectNumQdot) {}

template<typename T>
FixedBodyContact<T>::FixedBodyContact(std::size_t num_qdot) : ContactSpec<T>(6)
{
  if (num_qdot < this->dim_contact_) {
    throw std::invalid_argument("fixed-body contact requires at least six velocities");
  }
  this->Jc_ = DMat<T>::Zero(this->dim_contact_, num_qdot);
  this->Jc_.template leftCols<6>().setIdentity();
  this->JcDotQdot_ = DVec<T>::Zero(this->dim_contact_);
  this->Uf_ = DMat<T>::Zero(1, this->dim_contact_);
  this->ieq_vec_ = DVec<T>::Zero(1);
}

template<typename T>
FixedBodyContact<T>::FixedBodyContact(const FloatingBaseModel<T> * robot)
: FixedBodyContact(robot == nullptr ? 0 : robot->getNumDof())
{
  if (robot == nullptr) {
    throw std::invalid_argument("fixed-body contact requires a robot model");
  }
}

template<typename T>
FixedBodyContact<T>::FixedBodyContact(const FloatingBaseModel<T> & robot)
: FixedBodyContact(robot.getNumDof()) {}

template<typename T>
bool FixedBodyContact<T>::_UpdateJc() {return true;}

template<typename T>
bool FixedBodyContact<T>::_UpdateJcDotQdot()
{
  this->JcDotQdot_.setZero();
  return true;
}

template<typename T>
bool FixedBodyContact<T>::_UpdateUf() {return true;}

template<typename T>
bool FixedBodyContact<T>::_UpdateInequalityVector() {return true;}

template class FixedBodyContact<float>;
template class FixedBodyContact<double>;
