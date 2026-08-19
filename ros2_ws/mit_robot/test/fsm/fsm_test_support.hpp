#pragma once

// 提供 FSM 单元测试共用的机器人模型、传感器反馈、期望状态和调度器。

#include <array>
#include <cstddef>

#include "FSM/FSM_State.h"
#include "model/robots/unitree_go1.hpp"

namespace test_support
{

struct FsmContext
{
  explicit FsmContext(float time_step)
  : quadruped(robots::unitree_go1::makeModel<float>()),
    control_parameters(makeRobotControlParameters<float>(RobotType::UNITREE_GO1)),
    leg_controller(quadruped),
    gait_scheduler(time_step),
    control_time_step(time_step)
  {
    // 用 GO1 默认姿态初始化四条腿，并让腿控制器先接收一帧有效反馈。
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      joints[leg].leg = static_cast<LegId>(leg);
      joints[leg].position = quadruped.leg(joints[leg].leg).joints.home_position;
      joints[leg].valid = true;
      initialized = leg_controller.updateData(joints[leg]) && initialized;
    }
    estimate.position_world.z() = 0.27F;
    estimate.valid = true;
    desired.body_position_world = estimate.position_world;
    desired.valid = true;
  }

  ControlFSMData<float> data()
  {
    // FSM 只借用这些对象，因此返回的控制数据保存的是非拥有指针。
    ControlFSMData<float> result;
    result.quadruped = &quadruped;
    result.control_parameters = &control_parameters;
    result.state_estimate = &estimate;
    result.joint_states = &joints;
    result.leg_controller = &leg_controller;
    result.gait_scheduler = &gait_scheduler;
    result.desired_state = &desired;
    result.control_time_step = control_time_step;
    return result;
  }

  Quadruped<float> quadruped;
  RobotControlParameters<float> control_parameters;
  LegController<float> leg_controller;
  std::array<JointState<float>, kNumLegs> joints{};
  StateEstimate<float> estimate{};
  DesiredState<float> desired{};
  GaitScheduler<float> gait_scheduler;
  float control_time_step;
  bool initialized{true};
};

}  // namespace test_support
