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

  /** @brief 一次 WBC 求解的运动参考、力矩和接触力结果。 */
  struct Result
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    DVec<T> joint_position = DVec<T>::Zero(kNumJoints);  ///< 关节位置参考，rad。
    DVec<T> joint_velocity = DVec<T>::Zero(kNumJoints);  ///< 关节速度参考，rad/s。
    DVec<T> joint_torque = DVec<T>::Zero(kNumJoints);    ///< 关节前馈力矩，N*m。
    DVec<T> generalized_acceleration;  ///< 浮动基加关节的广义加速度，rad/s^2。
    DVec<T> reaction_force;            ///< 按支撑接触顺序拼接的反力，N。
    bool valid = false;                ///< 以上结果是否可安全发送给执行层。
  };

  /** @brief 创建通用 WBC 执行框架并初始化 KinWBC/WBIC 求解器。 */
  explicit WBC_Ctrl(FloatingBaseModel<T> model);
  virtual ~WBC_Ctrl() = default;

  WBC_Ctrl(const WBC_Ctrl &) = delete;
  WBC_Ctrl & operator=(const WBC_Ctrl &) = delete;
  WBC_Ctrl(WBC_Ctrl &&) = delete;
  WBC_Ctrl & operator=(WBC_Ctrl &&) = delete;

  /**
   * @brief 执行一个完整 WBC 周期，但不写入腿部控制器。
   * @param input 派生类输入结构，由 prepareTasksAndContacts() 解释。
   * @param estimate 当前 IMU/状态估计结果。
   * @param joint_states 四条腿的关节测量状态。
   * @return 模型更新、任务准备和 KinWBC/WBIC 求解均成功时返回 true。
   */
  bool run(
    const void * input, const StateEstimate<T> & estimate,
    const std::array<JointState<T>, kNumLegs> & joint_states);

  /**
   * @brief 执行 WBC 并将有效结果转换为四条腿的 JointCommand。
   * @return 求解成功且命令有效时返回 true；失败时会关闭腿部输出。
   */
  bool runAndApply(
    const void * input, const StateEstimate<T> & estimate,
    const std::array<JointState<T>, kNumLegs> & joint_states,
    LegController<T> & leg_controller);

  /** @brief 设置优化中浮动基加速度修正的正权重。 */
  void setFloatingBaseWeight(T weight);
  /** @brief 设置优化中接触反力修正的正权重。 */
  void setReactionForceWeight(T weight);
  /** @brief 设置最终关节 PD 修正的比例和微分增益。 */
  void setJointGains(const Vec3<T> & kp, const Vec3<T> & kd);

  const Result & result() const noexcept {return result_;}
  std::size_t iteration() const noexcept {return iteration_;}

protected:
  /**
   * @brief 更新派生类任务/接触对象，并按优先级注册它们。
   * @param input 派生控制器定义的输入结构。
   * @return 任务和接触数据有效时返回 true。
   */
  virtual bool prepareTasksAndContacts(const void * input) = 0;

  /** @brief 注册一个任务；注册顺序决定任务优先级。 */
  void addTask(Task<T> & task);
  /** @brief 注册一个支撑接触约束。 */
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

  FloatingBaseModel<T> model_;  ///< 当前控制周期使用的浮动基动力学模型。
  FBModelState<T> state_;       ///< 由估计和关节测量转换出的模型状态。
  StateEstimate<T> estimate_;   ///< 最近一次成功更新的状态估计。
  std::array<JointState<T>, kNumLegs> joint_states_{};  ///< 最近一次四腿关节测量。

  std::vector<ContactSpec<T> *> contacts_;  ///< 本周期的支撑接触，非拥有指针。
  std::vector<Task<T> *> tasks_;            ///< 本周期按优先级排列的任务，非拥有指针。
  std::unique_ptr<KinWBC<T>> kin_wbc_;      ///< 运动学层求解器。
  std::unique_ptr<WBIC<T>> wbic_;           ///< 动力学层二次规划求解器。
  WBIC_ExtraData<T> wbic_data_;             ///< WBIC 输入权重和输出缓存。

  DMat<T> mass_matrix_;          ///< 当前姿态的广义质量矩阵。
  DMat<T> mass_matrix_inverse_;  ///< 广义质量矩阵逆，用于加权伪逆。
  DVec<T> gravity_;              ///< 广义重力项。
  DVec<T> coriolis_;             ///< 广义科氏/离心项。
  Result result_;                ///< 最近一次 run() 的结果。
  Vec3<T> kp_joint_ = Vec3<T>::Constant(T(5));   ///< 输出关节位置 PD 比例增益。
  Vec3<T> kd_joint_ = Vec3<T>::Constant(T(1.5)); ///< 输出关节速度 PD 微分增益。
  T floating_base_weight_ = T(0.1);   ///< 浮动基修正项在 WBIC 代价中的权重。
  T reaction_force_weight_ = T(1);     ///< 接触反力修正项在 WBIC 代价中的权重。
  std::size_t iteration_ = 0;          ///< 已执行的 WBC 周期数。
};

extern template class WBC_Ctrl<float>;

#endif  // MYMIT_ROBOT_WBC_CTRL_WBC_CTRL_HPP_
