/**
 * @file SingleContact.cpp
 * @brief 单个足端三维点接触约束的实现。
 *
 * 本文件负责初始化足端接触的力不等式约束，并从
 * FloatingBaseModel 读取接触雅可比及其偏置项。
 */

#include "WBC/ContactSet/SingleContact.hpp"

#include <limits>
#include <stdexcept>

/**
 * @brief 构造单个足端的三维点接触约束。
 *
 * 接触力向量为 @f$F=[F_x,F_y,F_z]^T@f$。六个不等式约束为
 * @f$F_z\geq0@f$、@f$F_x+\mu F_z\geq0@f$、
 * @f$-F_x+\mu F_z\geq0@f$、@f$F_y+\mu F_z\geq0@f$、
 * @f$-F_y+\mu F_z\geq0@f$ 和 @f$-F_z\geq-F_{z,max}@f$。
 *
 * @tparam T 标量类型。
 * @param robot 浮动基座机器人模型指针。
 * @param point 接触点索引。
 * @throws std::invalid_argument 当 robot 为空时抛出。
 * @throws std::out_of_range 当 point 不是模型中的有效接触点索引时抛出。
 */
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

  /*  0  0  1              >=    0
      1  0  μ              >=    0
     -1  0  μ       Fx     >=    0
      0  1  μ   x   Fy     >=    0
      0 -1  μ       Fz     >=    0
      0  0 -1              >= -Fz,max
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

/**
 * @brief 从机器人模型更新足端接触雅可比。
 *
 * 对三维足端线速度而言，雅可比满足 @f$v_{foot}=J_c\dot{q}@f$。
 *
 * @tparam T 标量类型。
 * @return 接触点存在且雅可比所有元素有限时返回 true，否则返回 false。
 */
template<typename T>
bool SingleContact<T>::_UpdateJc()
{
  // 接触雅可比满足足端速度 v_foot = Jc * qdot。
  const auto & values = robot_sys_->getContactJacobians();
  if (_contact_pt >= values.size()) {return false;}
  this->Jc_ = values[_contact_pt];
  return this->Jc_.allFinite();
}

/**
 * @brief 从机器人模型更新接触雅可比的速度偏置项。
 *
 * 足端加速度满足 @f$a_{foot}=J_c\ddot{q}+\dot{J}_c\dot{q}@f$；
 * 该函数读取模型计算得到的 @f$\dot{J}_c\dot{q}@f$，供约束求解器使用。
 *
 * @tparam T 标量类型。
 * @return 接触点存在且偏置向量所有元素有限时返回 true，否则返回 false。
 */
template<typename T>
bool SingleContact<T>::_UpdateJcDotQdot()
{
  // 足端加速度为 Jc*qddot + JcDot*qdot，固定接触时该结果应为零。
  const auto & values = robot_sys_->getContactJacobianDotQdot();
  if (_contact_pt >= values.size()) {return false;}
  this->JcDotQdot_ = values[_contact_pt];
  return this->JcDotQdot_.allFinite();
}

/**
 * @brief 更新接触力不等式矩阵。
 *
 * 矩阵已在构造函数中根据固定摩擦系数初始化，运行过程中无需重复计算。
 * @return true。
 */
template<typename T>
bool SingleContact<T>::_UpdateUf() {return true;}

/**
 * @brief 更新接触力不等式右端向量。
 *
 * 最大法向力约束为 @f$-F_z\geq-F_{z,max}@f$，因此设置
 * @f$i_{eq,5}=-F_{z,max}@f$。
 * @return true。
 */
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
