#ifndef MYMIT_ROBOT_WBC_LOCOMOTION_CTRL_HPP_
#define MYMIT_ROBOT_WBC_LOCOMOTION_CTRL_HPP_

#include <array>
#include <cstddef>
#include <memory>

#include "WBC/ContactSet/SingleContact.hpp"
#include "WBC/WBC_Ctrl/BodyOriTask.hpp"
#include "WBC/WBC_Ctrl/BodyPosTask.hpp"
#include "WBC/WBC_Ctrl/LinkPosTask.hpp"
#include "WBC/WBC_Ctrl/WBC_Ctrl.hpp"
#include "model/robot_types.hpp"

/** Desired whole-body locomotion command. All Cartesian values are world-frame. */
template<typename T>
struct LocomotionCtrlData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LocomotionCtrlData() noexcept;

  Vec3<T> pBody_des = Vec3<T>::Zero();
  Vec3<T> vBody_des = Vec3<T>::Zero();
  Vec3<T> aBody_des = Vec3<T>::Zero();
  Vec3<T> pBody_RPY_des = Vec3<T>::Zero();
  Vec3<T> vBody_Ori_des = Vec3<T>::Zero();

  // C arrays are intentionally retained for source compatibility.
  Vec3<T> pFoot_des[kNumLegs]{};
  Vec3<T> vFoot_des[kNumLegs]{};
  Vec3<T> aFoot_des[kNumLegs]{};
  Vec3<T> Fr_des[kNumLegs]{};
  Vec4<T> contact_state = Vec4<T>::Zero();

  bool allFinite() const noexcept;
};

/** GO1 locomotion WBC using body tasks and one contact/swing task per foot. */
template<typename T>
class LocomotionCtrl final : public WBC_Ctrl<T>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit LocomotionCtrl(FloatingBaseModel<T> model);
  ~LocomotionCtrl() override = default;

  void setBodyPositionGains(const Vec3<T> & kp, const Vec3<T> & kd);
  void setBodyOrientationGains(const Vec3<T> & kp, const Vec3<T> & kd);
  void setFootPositionGains(const Vec3<T> & kp, const Vec3<T> & kd);
  void setMaxNormalForce(T max_fz);

  std::array<Vec3<T>, kNumLegs> reactionForces() const;

protected:
  bool prepareTasksAndContacts(const void * input) override;

private:
  std::unique_ptr<BodyPosTask<T>> body_position_task_;
  std::unique_ptr<BodyOriTask<T>> body_orientation_task_;
  std::array<std::unique_ptr<LinkPosTask<T>>, kNumLegs> foot_tasks_;
  std::array<std::unique_ptr<SingleContact<T>>, kNumLegs> foot_contacts_;
  std::array<std::size_t, kNumLegs> foot_contact_indices_{};
  Vec4<T> active_contact_state_ = Vec4<T>::Zero();
};

extern template struct LocomotionCtrlData<float>;
extern template struct LocomotionCtrlData<double>;
extern template class LocomotionCtrl<float>;

#endif  // MYMIT_ROBOT_WBC_LOCOMOTION_CTRL_HPP_
