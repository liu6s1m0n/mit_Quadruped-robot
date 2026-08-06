/*! @file leg_controller.hpp
 *  @brief 四足机器人单腿关节/笛卡尔控制器的公共接口。
 *
 *  数据流如下：
 *  LegSensor -> JointState -> LegControllerData
 *            -> LegControllerCommand -> JointCommand -> 仿真器/真实电机
 *
 *  LegController 不直接访问 MuJoCo 数组或真实电机总线。仿真和硬件首先通过
 *  LegSensor 生成相同格式的 JointState，因此控制算法不需要区分数据来源。
 */

#ifndef MYMIT_ROBOT_CONTROLLER_LEG_CONTROLLER_HPP_
#define MYMIT_ROBOT_CONTROLLER_LEG_CONTROLLER_HPP_

#include <array>
#include <cstdint>

#include "model/quadruped.hpp"
#include "model/robot_types.hpp"
#include "sensor/leg.hpp"

/**
 * @brief 上层控制算法发送给一条腿的期望命令。
 *
 * 关节空间命令最终直接映射到 JointCommand；笛卡尔空间命令先按照
 *
 * F = F_ff + Kp_cartesian * (p_des - p) + Kd_cartesian * (v_des - v)
 *
 * 计算期望足端力，再通过 tau_cartesian = J^T * F 转换为关节前馈力矩。
 * 最终的前馈力矩为 tau_ff + tau_cartesian。
 *
 * 所有三维关节向量的固定顺序均为 [Hip, thigh, calf]。
 */
template<typename T>
struct LegControllerCommand
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LegControllerCommand() {zero();}

  /** @brief 将一条腿的所有期望量和增益清零。 */
  void zero();

  Vec3<T> torque_feedforward = Vec3<T>::Zero();  ///< 关节前馈力矩，N*m。
  Vec3<T> force_feedforward = Vec3<T>::Zero();   ///< 足端前馈力，腿部坐标系，N。
  Vec3<T> position_desired = Vec3<T>::Zero();    ///< 期望关节角，rad。
  Vec3<T> velocity_desired = Vec3<T>::Zero();    ///< 期望关节角速度，rad/s。
  Vec3<T> foot_position_desired = Vec3<T>::Zero();  ///< 期望足端位置，腿部坐标系，m。
  Vec3<T> foot_velocity_desired = Vec3<T>::Zero();  ///< 期望足端速度，腿部坐标系，m/s。
  Mat3<T> kp_cartesian = Mat3<T>::Zero();        ///< 足端位置增益。
  Mat3<T> kd_cartesian = Mat3<T>::Zero();        ///< 足端速度增益。
  Vec3<T> kp_joint = Vec3<T>::Zero();            ///< 三个关节的位置增益。
  Vec3<T> kd_joint = Vec3<T>::Zero();            ///< 三个关节的速度增益。
};

/**
 * @brief 一条腿提供给上层控制算法的反馈和运动学结果。
 *
 * q、qd 和 torque_estimate 直接来自 JointState；p、J 和 v 由控制器根据
 * Quadruped 中保存的该腿几何参数计算，其中 v = J * qd。
 */
template<typename T>
struct LegControllerData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LegControllerData() {zero();}

  /**
   * @brief 绑定计算足端运动学时使用的机器人模型。
   * @param quad 生命周期必须覆盖本 LegControllerData 的机器人模型。
   */
  void setQuadruped(const Quadruped<T> & quad) {quadruped = &quad;}

  /** @brief 清空反馈和运动学结果，并标记数据无效。 */
  void zero();

  LegId leg = LegId::FR;                         ///< 当前反馈所属腿。
  Vec3<T> q = Vec3<T>::Zero();                   ///< 实际关节角，rad。
  Vec3<T> qd = Vec3<T>::Zero();                  ///< 实际关节角速度，rad/s。
  Vec3<T> p = Vec3<T>::Zero();                   ///< 实际足端位置，腿部坐标系，m。
  Vec3<T> v = Vec3<T>::Zero();                   ///< 实际足端速度，腿部坐标系，m/s。
  Mat3<T> J = Mat3<T>::Zero();                   ///< 足端雅可比，v = J * qd。
  Vec3<T> torque_estimate = Vec3<T>::Zero();     ///< 估计或测量关节力矩，N*m。
  T timestamp = T(0);                            ///< 本次反馈的单调时间戳，s。
  bool valid = false;                            ///< false 时不得参与控制计算。
  const Quadruped<T> * quadruped = nullptr;      ///< 非拥有指针，不负责释放模型。
};

/**
 * @brief 管理四条腿反馈、运动学计算和统一关节命令生成。
 *
 * 数据源和命令发送层都位于控制器外部：仿真与硬件数据先转换成 JointState，
 * 控制器生成 JointCommand 后，再由相应执行层写入 MuJoCo 或真实电机。
 */
template<typename T>
class LegController
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief 创建四腿控制器，并让四份反馈数据共享同一个机器人模型。
   * @param quadruped 包含连杆长度、左右腿符号和关节力矩限制的模型。
   */
  explicit LegController(const Quadruped<T> & quadruped);

  /** @brief 清零四条腿命令并关闭总使能。 */
  void zeroCommand();

  /**
   * @brief 清零原命令，设置纯速度阻尼命令并打开总使能。
   *
   * 此时 kp=0、期望速度为 0，执行层计算出的反馈项为 -gain * qd。
   * @param robot 用于检查调用方选择的机型是否与当前模型一致。
   * @param gain 三个关节统一使用的非负速度阻尼增益。
   */
  void edampCommand(RobotType robot, T gain);

  /**
   * @brief 使用一个统一 JointState 更新对应腿的反馈和运动学数据。
   * @param state 包含腿编号、三个关节反馈和时间戳的一次采样。
   * @return 输入及计算结果均有效时返回 true，否则清空该腿数据并返回 false。
   */
  bool updateData(const JointState<T> & state);

  /**
   * @brief 从仿真或硬件 LegSensor 读取一次数据并更新对应腿。
   *
   * source 可以是 SimLeg 或 HardwareLeg；本函数读取后统一转交给上面的
   * JointState 重载处理。
   * @return 读取结果及运动学计算有效时返回 true。
   */
  bool updateData(LegSensor & source);

  /**
   * @brief 根据关节命令和笛卡尔命令生成一条腿的统一 JointCommand。
   *
   * 总使能关闭、该腿反馈无效或期望值含 NaN/Inf 时，返回的 enabled=false。
   * 本函数只限制已经计算出的前馈力矩；执行层还必须限制包含 PD 项的最终力矩。
   * @param leg_id 要生成命令的腿。
   * @param timestamp 命令生成时间，单位 s。
   * @return 可交给 MuJoCo 或真实硬件发送层处理的关节命令。
   */
  JointCommand<T> command(LegId leg_id, T timestamp = T(0));

  /** @brief 设置四条腿的总输出使能；false 时 command() 始终禁止输出。 */
  void setEnabled(bool enabled) noexcept {_legsEnabled = enabled;}
  /** @brief 查询当前四腿总输出使能状态。 */
  bool legsEnabled() const noexcept {return _legsEnabled;}

  // 两个数组的顺序由 LegId 规定，固定为 FR、FL、RR、RL。
  std::array<LegControllerCommand<T>, kNumLegs> commands;  ///< 四条腿的期望命令。
  std::array<LegControllerData<T>, kNumLegs> datas;       ///< 四条腿的实时反馈。
  const Quadruped<T> & _quadruped;                        ///< 只读机器人参数。
  bool _legsEnabled = false;                              ///< 四腿总输出使能。
  std::uint32_t _calibrateEncoders = 0;                   ///< 预留编码器标定标志。

private:
  // 每条腿分别维护递增序号，执行层可以用它发现重复命令或丢帧。
  std::array<std::uint64_t, kNumLegs> command_sequences_{};
};

/**
 * @brief 根据三个关节角计算腿部坐标系中的足端位置和雅可比。
 *
 * 坐标系原点位于该腿 Hip 安装位置。q 的顺序为 [Hip, thigh, calf]；
 * J 的行对应足端 x/y/z 速度，列对应三个关节速度，并满足 v = J * qd。
 * J 或 p 可以为 nullptr，此时跳过对应输出。
 *
 * @param quad 提供当前腿三段连杆长度和左右腿镜像符号的机器人模型。
 * @param q 三个关节的当前角度，rad。
 * @param J 雅可比输出地址；不需要雅可比时传 nullptr。
 * @param p 足端位置输出地址；不需要位置时传 nullptr。
 * @param leg_id 用于选择腿参数并判断左腿/右腿。
 */
template<typename T>
void computeLegJacobianAndPosition(
  const Quadruped<T> & quad, const Vec3<T> & q,
  Mat3<T> * J, Vec3<T> * p, LegId leg_id);

#endif  // MYMIT_ROBOT_CONTROLLER_LEG_CONTROLLER_HPP_
