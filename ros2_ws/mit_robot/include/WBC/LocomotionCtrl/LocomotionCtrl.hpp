/**
 * @file LocomotionCtrl.hpp
 * @brief 把机身任务、摆动足任务和支撑足接触组合成一次 WBC 求解。
 *
 * contact_state 大于零表示支撑腿，否则该腿被当作摆动足跟踪位置轨迹。
 */
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

/**
 * @brief LocomotionCtrl 一次控制周期的期望输入。
 *
 * 所有笛卡尔位置、速度、加速度和反力均使用世界坐标系；数组顺序固定为
 * FR、FL、RR、RL。支撑腿使用 Fr_des 和接触约束，摆动腿使用足端轨迹任务。
 */
template<typename T>
struct LocomotionCtrlData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief 将机身、足端轨迹和期望反力全部初始化为零。 */
  LocomotionCtrlData() noexcept;

  Vec3<T> pBody_des = Vec3<T>::Zero();       ///< 期望机身位置，世界坐标系，m。
  Vec3<T> vBody_des = Vec3<T>::Zero();       ///< 期望机身线速度，世界坐标系，m/s。
  Vec3<T> aBody_des = Vec3<T>::Zero();       ///< 期望机身线加速度，世界坐标系，m/s^2。
  Vec3<T> pBody_RPY_des = Vec3<T>::Zero();   ///< 期望机身滚转/俯仰/偏航角，rad。
  Vec3<T> vBody_Ori_des = Vec3<T>::Zero();  ///< 期望机身角速度，世界坐标系，rad/s。

  // C arrays are intentionally retained for source compatibility.
  Vec3<T> pFoot_des[kNumLegs]{};  ///< 每条摆动腿的期望足端位置，世界坐标系，m。
  Vec3<T> vFoot_des[kNumLegs]{};  ///< 每条摆动腿的期望足端速度，世界坐标系，m/s。
  Vec3<T> aFoot_des[kNumLegs]{};  ///< 每条摆动腿的期望足端加速度，世界坐标系，m/s^2。
  Vec3<T> Fr_des[kNumLegs]{};     ///< 每条支撑腿的期望地面反力，世界坐标系，N。
  Vec4<T> contact_state = Vec4<T>::Zero();  ///< 接触标志；大于 0 为支撑腿。

  /**
   * @brief 检查所有输入向量是否为有限数。
   * @return 所有机身/足端/反力/接触状态均有效时返回 true。
   */
  bool allFinite() const noexcept;
};

/**
 * @brief 运动状态使用的全身控制器。
 *
 * 每个周期固定加入机身位置和姿态任务；四条腿根据 contact_state 分别加入
 * 支撑接触约束或摆动足位置任务，最后交给 WBC_Ctrl/KinWBC/WBIC 求解关节力矩。
 */
template<typename T>
class LocomotionCtrl final : public WBC_Ctrl<T>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief 创建运动 WBC，并为四个足端建立任务和接触对象。
   * @param model 浮动基机器人模型；对象会被移动到控制器内部。
   * @throws std::invalid_argument 模型不是四足或足端索引数量不为 4 时抛出。
   */
  explicit LocomotionCtrl(FloatingBaseModel<T> model);
  ~LocomotionCtrl() override = default;

  /** @brief 设置机身位置任务的比例和微分增益；增益必须有限且非负。 */
  void setBodyPositionGains(const Vec3<T> & kp, const Vec3<T> & kd);
  /** @brief 设置机身姿态任务的比例和微分增益。 */
  void setBodyOrientationGains(const Vec3<T> & kp, const Vec3<T> & kd);
  /** @brief 将同一组比例/微分增益应用到四条摆动足位置任务。 */
  void setFootPositionGains(const Vec3<T> & kp, const Vec3<T> & kd);
  /**
   * @brief 设置每个支撑足允许的最大法向力。
   * @param max_fz 最大法向力，单位 N，必须为有限正数。
   */
  void setMaxNormalForce(T max_fz);

  /**
   * @brief 读取最近一次 WBIC 求解得到的四腿地面反力。
   * @return 按 FR、FL、RR、RL 排列的世界坐标系反力；无有效结果时全为零。
   */
  std::array<Vec3<T>, kNumLegs> reactionForces() const;

protected:
  /**
   * @brief 将本周期输入转换为机身/足端任务和支撑接触约束。
   * @param input 实际类型必须是 const LocomotionCtrlData<T>*，不能为空。
   * @return 输入有效且所有任务/接触更新成功时返回 true。
   */
  bool prepareTasksAndContacts(const void * input) override;

private:
  std::unique_ptr<BodyPosTask<T>> body_position_task_;  ///< 机身位置任务。
  std::unique_ptr<BodyOriTask<T>> body_orientation_task_;  ///< 机身姿态任务。
  std::array<std::unique_ptr<LinkPosTask<T>>, kNumLegs> foot_tasks_;  ///< 四条摆动足位置任务。
  std::array<std::unique_ptr<SingleContact<T>>, kNumLegs> foot_contacts_;  ///< 四条支撑足接触约束。
  std::array<std::size_t, kNumLegs> foot_contact_indices_{};  ///< 四个足端在浮动基模型中的刚体索引。
  Vec4<T> active_contact_state_ = Vec4<T>::Zero();  ///< 本次求解实际启用的接触状态，用于恢复反力数组。
};

extern template struct LocomotionCtrlData<float>;
extern template struct LocomotionCtrlData<double>;
extern template class LocomotionCtrl<float>;

#endif  // MYMIT_ROBOT_WBC_LOCOMOTION_CTRL_HPP_
