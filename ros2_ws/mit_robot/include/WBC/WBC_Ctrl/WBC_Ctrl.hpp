/**
 * @file WBC_Ctrl.hpp
 * @brief WBC 高层编排器：更新模型、收集任务/接触、调用 KinWBC 和 WBIC、输出关节命令。
 */
#ifndef MYMIT_ROBOT_WBC_CTRL_WBC_CTRL_HPP_
#define MYMIT_ROBOT_WBC_CTRL_WBC_CTRL_HPP_

#include <array>
#include <cstddef>
#include <memory>
#include <vector>

#include "WBC/FloatingBaseModel.h"
#include "WBC/KinWBC.hpp"
#include "WBC/WBIC.hpp"
#include "controller/leg_controller.hpp"
#include "model/floating_base_model_factory.hpp"
#include "model/robot_types.hpp"

/**
 * Common execution shell for robot-specific whole-body controllers.
 *
 * Derived controllers own their Task and ContactSpec objects. During
 * prepareTasksAndContacts() they update those objects and register pointers in
 * priority order. WBC_Ctrl never deletes registered objects.
 */
template<typename T>
class WBC_Ctrl
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct Result
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DVec<T> joint_position = DVec<T>::Zero(kNumJoints);
    DVec<T> joint_velocity = DVec<T>::Zero(kNumJoints);
    DVec<T> joint_torque = DVec<T>::Zero(kNumJoints);
    DVec<T> generalized_acceleration;
    DVec<T> reaction_force;
    bool valid = false;
  };

  explicit WBC_Ctrl(FloatingBaseModel<T> model);
  virtual ~WBC_Ctrl() = default;

  WBC_Ctrl(const WBC_Ctrl &) = delete;
  WBC_Ctrl & operator=(const WBC_Ctrl &) = delete;
  WBC_Ctrl(WBC_Ctrl &&) = delete;
  WBC_Ctrl & operator=(WBC_Ctrl &&) = delete;

  bool run(
    const void * input, const StateEstimate<T> & estimate,
    const std::array<JointState<T>, kNumLegs> & joint_states);

  bool runAndApply(
    const void * input, const StateEstimate<T> & estimate,
    const std::array<JointState<T>, kNumLegs> & joint_states,
    LegController<T> & leg_controller);

  void setFloatingBaseWeight(T weight);
  void setReactionForceWeight(T weight);
  void setJointGains(const Vec3<T> & kp, const Vec3<T> & kd);

  const Result & result() const noexcept {return result_;}
  std::size_t iteration() const noexcept {return iteration_;}

protected:
  /** Update concrete objects, then call addContact()/addTask() in priority order. */
  virtual bool prepareTasksAndContacts(const void * input) = 0;

  void addTask(Task<T> & task);
  void addContact(ContactSpec<T> & contact);

  FloatingBaseModel<T> & model() noexcept {return model_;}
  const FBModelState<T> & modelState() const noexcept {return state_;}
  const StateEstimate<T> & stateEstimate() const noexcept {return estimate_;}
  const std::array<JointState<T>, kNumLegs> & jointStates() const noexcept
  {
    return joint_states_;
  }

private:
  bool updateModel(
    const StateEstimate<T> & estimate,
    const std::array<JointState<T>, kNumLegs> & joint_states);
  bool compute();
  void applyResult(LegController<T> & leg_controller) const;
  void invalidate() noexcept;

  FloatingBaseModel<T> model_;
  FBModelState<T> state_;
  StateEstimate<T> estimate_;
  std::array<JointState<T>, kNumLegs> joint_states_{};

  std::vector<ContactSpec<T> *> contacts_;
  std::vector<Task<T> *> tasks_;
  std::unique_ptr<KinWBC<T>> kin_wbc_;
  std::unique_ptr<WBIC<T>> wbic_;
  WBIC_ExtraData<T> wbic_data_;

  DMat<T> mass_matrix_;
  DMat<T> mass_matrix_inverse_;
  DVec<T> gravity_;
  DVec<T> coriolis_;
  Result result_;
  Vec3<T> kp_joint_ = Vec3<T>::Constant(T(5));
  Vec3<T> kd_joint_ = Vec3<T>::Constant(T(1.5));
  T floating_base_weight_ = T(0.1);
  T reaction_force_weight_ = T(1);
  std::size_t iteration_ = 0;
};

extern template class WBC_Ctrl<float>;

#endif  // MYMIT_ROBOT_WBC_CTRL_WBC_CTRL_HPP_
