#include "WBC/ContactSet/SingleContact.hpp"

#include <limits>
#include <stdexcept>

// 单点足端接触：提供 3 维接触雅可比，并用线性不等式近似库仑摩擦锥。
template<typename T>
SingleContact<T>::SingleContact(const FloatingBaseModel<T> * robot, int point)
: ContactSpec<T>(3), _max_Fz(T(1500)),
  _contact_pt(point < 0 ? 0 : static_cast<std::size_t>(point)), _dim_U(6),
  robot_sys_(robot)
{
  if (robot == nullptr) {throw std::invalid_argument("single contact requires a model");}
  if (point < 0 || _contact_pt >= robot->getNumGroundContacts()) {
    throw std::out_of_range("contact point index is outside the robot model");
  }
  this->idx_Fz_ = 2;
  this->Jc_ = DMat<T>::Zero(this->dim_contact_, robot->getNumDof());
  this->JcDotQdot_ = DVec<T>::Zero(this->dim_contact_);
  this->Uf_ = DMat<T>::Zero(_dim_U, this->dim_contact_);
  const T friction = T(0.4);
  // 约束依次表示 Fz >= 0、|Fx| <= mu*Fz、|Fy| <= mu*Fz、Fz <= max_Fz。

  /*  0  0  1                    0
      1  0  μ                    0
     -1  0  μ       Fx           0
      0  1  μ   x   Fy     =     0
      0 -1  μ       Fz           0
      0  0 -1                 -Fz,max
  */
  this->Uf_(0, 2) = T(1);
  this->Uf_(1, 0) = T(1);  this->Uf_(1, 2) = friction;
  this->Uf_(2, 0) = T(-1); this->Uf_(2, 2) = friction;
  this->Uf_(3, 1) = T(1);  this->Uf_(3, 2) = friction;
  this->Uf_(4, 1) = T(-1); this->Uf_(4, 2) = friction;
  this->Uf_(5, 2) = T(-1);
}

template<typename T>
SingleContact<T>::SingleContact(
  const FloatingBaseModel<T> & robot, std::size_t contact_point)
: SingleContact(
    &robot, contact_point > static_cast<std::size_t>(std::numeric_limits<int>::max()) ?
    -1 : static_cast<int>(contact_point)) {}

template<typename T>
bool SingleContact<T>::_UpdateJc()
{
  // 接触雅可比满足足端速度 v_foot = Jc * qdot。
  const auto & values = robot_sys_->getContactJacobians();
  if (_contact_pt >= values.size()) {return false;}
  this->Jc_ = values[_contact_pt];
  return this->Jc_.allFinite();
}

template<typename T>
bool SingleContact<T>::_UpdateJcDotQdot()
{
  // 足端加速度为 Jc*qddot + JcDot*qdot，固定接触时该结果应为零。
  const auto & values = robot_sys_->getContactJacobianDotQdot();
  if (_contact_pt >= values.size()) {return false;}
  this->JcDotQdot_ = values[_contact_pt];
  return this->JcDotQdot_.allFinite();
}

template<typename T>
bool SingleContact<T>::_UpdateUf() {return true;}

template<typename T>
bool SingleContact<T>::_UpdateInequalityVector()
{
  // Uf*Fr >= ieq_vec；最后一行 -Fz >= -max_Fz 给出法向力上限。
  this->ieq_vec_ = DVec<T>::Zero(_dim_U);
  this->ieq_vec_[5] = -_max_Fz;
  return true;
}

template class SingleContact<float>;
template class SingleContact<double>;
