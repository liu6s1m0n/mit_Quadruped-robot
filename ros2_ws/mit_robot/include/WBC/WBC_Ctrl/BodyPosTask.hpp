/**
 * @file BodyPosTask.hpp
 * @brief 机身质心位置任务，使用位置/速度 PD 加前馈加速度生成任务命令。
 */
#ifndef MYMIT_ROBOT_WBC_CTRL_BODY_POS_TASK_HPP_
#define MYMIT_ROBOT_WBC_CTRL_BODY_POS_TASK_HPP_

#include "WBC/FloatingBaseModel.h"
#include "WBC/Task.hpp"

template<typename T>
class BodyPosTask : public Task<T>
{
public:
  /**
   * @brief 创建机身位置任务。
   * @param model 浮动基模型指针，不能为空。
   * @throws std::invalid_argument model 为空时抛出。
   */
  explicit BodyPosTask(const FloatingBaseModel<T> * model);
  explicit BodyPosTask(const FloatingBaseModel<T> & model) : BodyPosTask(&model) {}
  ~BodyPosTask() override = default;

  DVec<T> _Kp_kin;  ///< 运动学层位置误差缩放增益。
  DVec<T> _Kp;      ///< 任务加速度的位置比例增益。
  DVec<T> _Kd;      ///< 任务加速度的速度微分增益。

protected:
  /** @brief 根据位置、速度和加速度参考生成机身任务命令。 */
  bool _UpdateCommand(
    const void * position_desired, const DVec<T> & velocity_desired,
    const DVec<T> & acceleration_desired) override;
  /** @brief 更新世界坐标系机身位置任务雅可比。 */
  bool _UpdateTaskJacobian() override;
  /** @brief 更新机身位置任务的 Jdot*qdot 偏置项。 */
  bool _UpdateTaskJDotQdot() override;
  bool _AdditionalUpdate() override {return true;}

  const FloatingBaseModel<T> * _robot_sys;  ///< 非拥有型浮动基模型指针。
};

extern template class BodyPosTask<float>;
extern template class BodyPosTask<double>;

#endif  // MYMIT_ROBOT_WBC_CTRL_BODY_POS_TASK_HPP_
