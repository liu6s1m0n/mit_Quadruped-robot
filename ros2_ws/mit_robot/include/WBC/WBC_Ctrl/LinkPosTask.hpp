#ifndef MYMIT_ROBOT_WBC_CTRL_LINK_POS_TASK_HPP_
#define MYMIT_ROBOT_WBC_CTRL_LINK_POS_TASK_HPP_

#include <cstddef>

#include "WBC/FloatingBaseModel.h"
#include "WBC/Task.hpp"

/** Cartesian position task for one registered model contact point. */
template<typename T>
class LinkPosTask final : public Task<T>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LinkPosTask(
    const FloatingBaseModel<T> & model, std::size_t contact_point_index,
    bool include_floating_base = true);
  ~LinkPosTask() override = default;

  bool update(
    const Vec3<T> & position_world_desired,
    const Vec3<T> & velocity_world_desired = Vec3<T>::Zero(),
    const Vec3<T> & acceleration_world_desired = Vec3<T>::Zero());

  void setKinematicGain(const Vec3<T> & gain);
  void setProportionalGain(const Vec3<T> & gain);
  void setDerivativeGain(const Vec3<T> & gain);

  std::size_t contactPointIndex() const noexcept {return contact_point_index_;}
  bool includesFloatingBase() const noexcept {return include_floating_base_;}

private:
  bool _UpdateCommand(
    const void * position_desired, const DVec<T> & velocity_desired,
    const DVec<T> & acceleration_desired) override;
  bool _UpdateTaskJacobian() override;
  bool _UpdateTaskJDotQdot() override;
  bool _AdditionalUpdate() override {return true;}
  static void validateGain(const Vec3<T> & gain);

  const FloatingBaseModel<T> * model_;
  std::size_t contact_point_index_;
  bool include_floating_base_;
  Vec3<T> kp_ = Vec3<T>::Constant(T(100));
  Vec3<T> kd_ = Vec3<T>::Constant(T(5));
  Vec3<T> kp_kinematic_ = Vec3<T>::Ones();
};

extern template class LinkPosTask<float>;
extern template class LinkPosTask<double>;

#endif  // MYMIT_ROBOT_WBC_CTRL_LINK_POS_TASK_HPP_
