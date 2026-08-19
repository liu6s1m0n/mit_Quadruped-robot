/**
 * @file LinkPosTask.hpp
 * @brief 指定连杆（本工程主要是足端）的三维位置跟踪任务。
 */
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

  /**
   * @brief 创建一个连杆/足端笛卡尔位置任务。
   * @param model 浮动基模型。
   * @param contact_point_index 模型接触点数组中的足端索引。
   * @param include_floating_base 是否保留雅可比前 6 列的浮动基贡献。
   * @throws std::out_of_range 接触点索引超出模型范围时抛出。
   */
  LinkPosTask(
    const FloatingBaseModel<T> & model, std::size_t contact_point_index,
    bool include_floating_base = true);
  ~LinkPosTask() override = default;

  /**
   * @brief 设置足端世界坐标系位置、速度和加速度期望。
   * @return 输入有效且任务更新成功时返回 true。
   */
  bool update(
    const Vec3<T> & position_world_desired,
    const Vec3<T> & velocity_world_desired = Vec3<T>::Zero(),
    const Vec3<T> & acceleration_world_desired = Vec3<T>::Zero());

  /** @brief 设置位置误差到运动学参考的增益。 */
  void setKinematicGain(const Vec3<T> & gain);
  /** @brief 设置足端位置比例反馈增益。 */
  void setProportionalGain(const Vec3<T> & gain);
  /** @brief 设置足端速度微分反馈增益。 */
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

  const FloatingBaseModel<T> * model_;  ///< 非拥有型浮动基模型指针。
  std::size_t contact_point_index_;     ///< 本任务跟踪的接触点索引。
  bool include_floating_base_;          ///< 是否让浮动基自由度参与足端运动。
  Vec3<T> kp_ = Vec3<T>::Constant(T(100));  ///< 足端位置比例增益。
  Vec3<T> kd_ = Vec3<T>::Constant(T(5));    ///< 足端速度微分增益。
  Vec3<T> kp_kinematic_ = Vec3<T>::Ones();   ///< 运动学误差缩放增益。
};

extern template class LinkPosTask<float>;
extern template class LinkPosTask<double>;

#endif  // MYMIT_ROBOT_WBC_CTRL_LINK_POS_TASK_HPP_
