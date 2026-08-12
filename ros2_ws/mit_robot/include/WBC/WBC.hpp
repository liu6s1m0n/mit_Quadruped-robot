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
    DMat<T> lambda(J * Winv * J.transpose());
    DMat<T> lambda_inv;
    pseudoInverse(lambda, threshold, lambda_inv);
    Jinv = Winv * J.transpose() * lambda_inv;
  }

  size_t num_act_joint_;
  size_t num_qdot_;

  DMat<T> Sa_;  // Actuated joint
  DMat<T> Sv_;  // Virtual joint

  DMat<T> A_;
  DMat<T> Ainv_;
  DVec<T> cori_;
  DVec<T> grav_;

  bool b_updatesetting_;
  bool b_internal_constraint_;
};

#endif
