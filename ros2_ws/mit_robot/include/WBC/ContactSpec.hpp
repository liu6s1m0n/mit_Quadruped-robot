/**
 * @file ContactSpec.hpp
 * @brief WBC 接触约束的抽象接口。
 *
 * Jc 描述接触点速度与广义速度的关系，Uf 和不等式向量描述摩擦锥及法向力边界。
 */
#ifndef CONTACT_SPEC
#define CONTACT_SPEC

#include "Utilities/cppTypes.h"
#include <stdexcept>

#define Contact ContactSpec<T>

template<typename T>
class ContactSpec
{
public:
  explicit ContactSpec(size_t dim)
  : dim_contact_(dim), b_set_contact_(false)
  {
    if (dim == 0) {throw std::invalid_argument("contact dimension must be positive");}
    idx_Fz_ = dim - 1;  // because normally (tau_x,y,z , linear_x,y,z)
    Fr_des_ = DVec<T>::Zero(dim);
  }
  virtual ~ContactSpec() = default;

  size_t getDim() const {return dim_contact_;}
  size_t getDimRFConstraint() const {return Uf_.rows();}
  size_t getFzIndex() const {return idx_Fz_;}

  void getContactJacobian(DMat<T> & Jc) const {Jc = Jc_;}
  void getJcDotQdot(DVec<T> & JcDotQdot) const {JcDotQdot = JcDotQdot_;}
  void UnsetContact() {b_set_contact_ = false;}

  void getRFConstraintMtx(DMat<T> & Uf) const {Uf = Uf_;}
  void getRFConstraintVec(DVec<T> & ieq_vec) const {ieq_vec = ieq_vec_;}
  const DVec<T> & getRFDesired() const {return Fr_des_;}
  void setRFDesired(const DVec<T> & Fr_des)
  {
    if (Fr_des.size() != static_cast<Eigen::Index>(dim_contact_) ||
      !Fr_des.allFinite())
    {
      throw std::invalid_argument("desired reaction force has invalid dimension");
    }
    Fr_des_ = Fr_des;
  }

  bool UpdateContactSpec()
  {
    b_set_contact_ = _UpdateJc() && _UpdateJcDotQdot() && _UpdateUf() &&
      _UpdateInequalityVector();
    return b_set_contact_;
  }

protected:
  virtual bool _UpdateJc() = 0;
  virtual bool _UpdateJcDotQdot() = 0;
  virtual bool _UpdateUf() = 0;
  virtual bool _UpdateInequalityVector() = 0;

  int idx_Fz_;           ///< 接触力向量中法向力 Fz 的索引。
  DMat<T> Uf_;           ///< 线性化摩擦锥和法向力边界的系数矩阵。
  DVec<T> ieq_vec_;      ///< 接触不等式边界，使 Uf*Fr >= ieq_vec。
  DVec<T> Fr_des_;       ///< 当前接触点的期望反力，单位 N。

  DMat<T> Jc_;           ///< 接触雅可比，将广义速度映射到接触点速度。
  DVec<T> JcDotQdot_;    ///< 接触点偏置加速度 Jc_dot*qdot。
  size_t dim_contact_;   ///< 本接触约束的力/运动维数。
  bool b_set_contact_;   ///< 本周期接触雅可比和约束是否已成功更新。
};
#endif
