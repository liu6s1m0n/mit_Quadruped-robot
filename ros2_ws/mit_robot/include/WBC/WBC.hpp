/**
 * @file WBC.hpp
 * @brief 全身控制求解器基类，保存质量矩阵、科氏力、重力和浮动基选择矩阵。
 */
#ifndef WHOLE_BODY_CONTROLLER
#define WHOLE_BODY_CONTROLLER

#include "Utilities/Utilities_print.h"
#include "Utilities/pseudoInverse.h"
#include "Utilities/cppTypes.h"
#include <vector>
#include <stdexcept>
#include "ContactSpec.hpp"
#include "Task.hpp"

// Assume first 6 (or 3 in 2D case) joints are for the representation of
// a floating base.

#define WB WBC<T>

template<typename T>
class WBC
{
public:
  explicit WBC(size_t num_qdot)
  : num_act_joint_(num_qdot >= 6 ? num_qdot - 6 : 0),
    num_qdot_(num_qdot),
    b_updatesetting_(false),
    b_internal_constraint_(false)
  {
    if (num_qdot < 6) {
      throw std::invalid_argument("WBC requires at least six floating-base velocities");
    }
    Sa_ = DMat<T>::Zero(num_act_joint_, num_qdot_);
    Sv_ = DMat<T>::Zero(6, num_qdot_);

    Sa_.block(0, 6, num_act_joint_, num_act_joint_).setIdentity();
    Sv_.block(0, 0, 6, 6).setIdentity();
  }
  virtual ~WBC() {}

  virtual void UpdateSetting(
    const DMat<T> & A, const DMat<T> & Ainv,
    const DVec<T> & cori, const DVec<T> & grav,
    void * extra_setting = nullptr) = 0;

  virtual void MakeTorque(DVec<T> & cmd, void * extra_input = nullptr) = 0;

protected:
  // full rank fat matrix only
  void _WeightedInverse(
    const DMat<T> & J, const DMat<T> & Winv, DMat<T> & Jinv,
    double threshold = 0.0001)
  {
    // lambda 是任务空间等效惯量的逆：J * A^{-1} * J^T。
    DMat<T> lambda(J * Winv * J.transpose());
    // lambda_inv 是上式的截断伪逆，奇异方向不会产生无限大的控制量。
    DMat<T> lambda_inv;
    pseudoInverse(lambda, threshold, lambda_inv);
    Jinv = Winv * J.transpose() * lambda_inv;
  }

  size_t num_act_joint_;  ///< 可被电机驱动的关节速度/力矩维数，GO1 为 12。
  size_t num_qdot_;       ///< 完整广义速度维数：6 维浮动基座加可驱动关节。

  DMat<T> Sa_;  ///< 驱动关节选择矩阵，从广义向量中取后 12 维。
  DMat<T> Sv_;  ///< 浮动基选择矩阵，从广义向量中取前 6 维。

  DMat<T> A_;     ///< 当前姿态下的广义质量矩阵。
  DMat<T> Ainv_;  ///< 广义质量矩阵的逆，用于动力学加权伪逆。
  DVec<T> cori_;  ///< 科里奥利力和离心力组成的广义偏置向量。
  DVec<T> grav_;  ///< 重力造成的广义力向量。

  bool b_updatesetting_;      ///< 本控制周期是否已经收到有效动力学矩阵。
  bool b_internal_constraint_;  ///< 是否启用闭链等内部运动学约束。
};

#endif
