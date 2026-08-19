/**
 * @file RobotRunner.hpp
 * @brief 单个控制周期的总调度器，是学习本工程控制数据流的最佳入口。
 *
 * 每次 run() 依次完成：传感器读取与状态估计 -> 初始化保护 -> FSM ->
 * MPC/WBC -> LegController 命令。RobotRunner 不推进物理仿真，只计算控制量。
 */
#ifndef MYMIT_ROBOT_USER_ROBOT_RUNNER_HPP_
#define MYMIT_ROBOT_USER_ROBOT_RUNNER_HPP_

#include <array>
#include <memory>

#include <mujoco/mujoco.h>

#include "FSM/ControlFSM.h"
#include "controller/PositionVelocityEstimator.hpp"
#include "controller/leg_controller.hpp"
#include "model/quadruped.hpp"
#include "model/robot_control_parameters.hpp"
#include "sensor/imu.hpp"
#include "sensor/leg.hpp"

/** 拥有并执行完整的估计器、FSM、MPC、WBC 控制管线。 */
/*  RobotRunner 的职责不是推进 MuJoCo 仿真，而是：
    读取传感器；更新状态估计；更新步态；运行 FSM；收集关节命令。
    仿真的时间推进通常由外层 MuJoCo 循环负责。*/
class RobotRunner
{
public:
  /** @brief 创建指定机型的完整控制管线。 */
  RobotRunner(
    mjModel * model, const mjData * data,
    RobotType robot_type = RobotType::UNITREE_GO1);
  ~RobotRunner() = default;

  RobotRunner(const RobotRunner &) = delete;
  RobotRunner & operator=(const RobotRunner &) = delete;

  /** 执行一个控制周期；返回 false 表示本周期命令不安全、已被关闭。 */
  bool run();
  /** 仿真 Reset 后清空控制器内部历史，但不会改写 MuJoCo 的物理状态。 */
  void reset();
  void setControlMode(ControlMode mode) noexcept;
  /** 从 DM1 趴卧零位请求执行一次站起；条件不满足时返回 false。 */
  bool requestStandUp() noexcept;
  /** 稳定站立时请求执行一次向前跳；条件不满足时返回 false。 */
  bool requestFrontJump() noexcept;
  /** 将前进速度透传给 ControlFSM 内的 Locomotion/MPC，单位 m/s。 */
  void setWalkingForwardSpeed(float speed);
  /** 设置机身系前后、左右速度和偏航角速度，单位 m/s、rad/s。 */
  void setLocomotionVelocityCommand(
    float forward_velocity, float lateral_velocity, float yaw_rate);
  /** 设置站立目标高度，实际命令会按 standing_height_rate_limit_ 平滑跟随。 */
  void setStandingHeight(float height);
  void setDesiredState(const DesiredState<float> & desired);

  float minimumStandingHeight() const noexcept
  {return control_parameters_.minimum_standing_height;}
  float maximumStandingHeight() const noexcept
  {return control_parameters_.maximum_standing_height;}
  static constexpr float defaultWalkingForwardSpeed() noexcept {return 0.32F;}
  float standingHeightTarget() const noexcept {return standing_height_target_;}
  /*返回四条腿当前生成的最终关节命令。外部执行层可以读取它并写入：
    MuJoCo 的控制数组；真实电机总线。*/
  const std::array<JointCommand<float>, kNumLegs> & jointCommands() const noexcept
  {
    return joint_commands_;
  }
  /*返回当前状态估计结果。*/
  const StateEstimate<float> & stateEstimate() const noexcept {return state_estimate_;}
  /*返回机器人模型*/
  const Quadruped<float> & quadruped() const noexcept {return quadruped_;}
  /** 返回 FSM 当前实际状态，用于确认安全回退没有反复重置步态。 */
  FSM_StateName currentStateName() const noexcept
  {
    return control_fsm_->currentStateName();
  }
  /** 当前是否已经处在允许行走的站立控制状态。 */
  bool standingReady() const noexcept;

private:
  /*保存四条腿传感器对象的所有权。*/
  using LegSensorOwners = std::array<std::unique_ptr<LegSensor>, kNumLegs>;
  /*保存四条腿传感器的裸指针。为什么同时保存两种形式？
    unique_ptr 负责生命周期；裸指针数组方便传给状态估计器。*/
  using LegSensorPointers = std::array<LegSensor *, kNumLegs>;
  /*创建四条腿的传感器对象。*/
  static LegSensorOwners makeLegSensors(const mjModel * model, const mjData * data);
  /*把 unique_ptr 转换成裸指针数组。*/
  static LegSensorPointers sensorPointers(const LegSensorOwners & sensors);
  /*刚启动或 Reset 后，将关节从当前实际位置平滑移动到默认姿态。*/
  void prepareJointInitialization();
  /*按照高度变化速率限制，更新当前实际发送的站立高度。*/
  void updateStandingHeightCommand();
  /*判断关节初始化是否完成。*/
  bool jointInitializationComplete() const noexcept;
  /*从 LegController 收集四条腿最终命令。*/
  bool collectJointCommands();
  /*清零并关闭所有腿部输出。*/
  void disableCommands() noexcept;
  /** 仅在 DM1 Home 启用整段小腿贴地代理，运动前关闭以免擦地。 */
  void setHomeCalfContactsEnabled(bool enabled) noexcept;
 
  /*保存 MuJoCo 模型和当前数据。
    它们都是非拥有型指针，RobotRunner 不负责释放。*/
  mjModel * model_ = nullptr;
  const mjData * data_ = nullptr;
  RobotControlParameters<float> control_parameters_;  ///< GO1 或 DM1 独立控制参数。
  /*四足机器人模型。*/
  Quadruped<float> quadruped_;
  /*四腿控制器。保存腿部反馈；计算足端运动学；生成 JointCommand。*/
  LegController<float> leg_controller_;
  /*IMU 传感器对象。*/
  std::unique_ptr<ImuSensor> imu_;
  /*分别保存：传感器对象所有权；
            提供给状态估计器的裸指针。*/
  LegSensorOwners leg_sensors_;
  LegSensorPointers leg_sensor_pointers_{};
  /*位置速度状态估计器。*/
  std::unique_ptr<PositionVelocityEstimator<float>> state_estimator_;
  /*步态调度器。*/
  GaitScheduler<float> gait_scheduler_;
  /*当前状态估计结果*/
  StateEstimate<float> state_estimate_;
  /*四条腿当前的关节状态*/
  std::array<JointState<float>, kNumLegs> joint_states_{};
  /*用户要求的目标状态*/
  DesiredState<float> desired_state_;
  /*总状态机。内部拥有：
    Passive；
    BalanceStand；
    Locomotion；
    WBC；
    MPC 等状态对象。*/
  std::unique_ptr<ControlFSM<float>> control_fsm_;
  /*最终输出给仿真器或硬件的四条腿命令。*/
  std::array<JointCommand<float>, kNumLegs> joint_commands_{};
  /*记录初始化开始瞬间四条腿的实际关节角*/
  std::array<Vec3<float>, kNumLegs> initial_joint_positions_{};
  /*关节初始化持续 0.4 秒；启动阶段优先保证平稳，不参与行走电机提速。*/
  float joint_initialization_start_time_ = 0.0F;
  float joint_initialization_duration_ = 0.4F;
  /*当前实际发送给控制器的高度。
    它会逐渐逼近目标值。*/
  float standing_height_target_ = 0.27F;
  float standing_height_command_ = 0.27F;
  /*高度变化速率限制为：*/
  float standing_height_rate_limit_ = 0.16F;
  /*标记是否已经开始关节初始化*/
  bool joint_initialization_started_ = false;
  /*是否已经根据当前估计高度初始化过高度命令*/
  bool standing_height_command_initialized_ = false;
  /*是否已经建立首次闭环期望状态*/
  bool desired_state_initialized_ = false;
};

#endif  // MYMIT_ROBOT_USER_ROBOT_RUNNER_HPP_
