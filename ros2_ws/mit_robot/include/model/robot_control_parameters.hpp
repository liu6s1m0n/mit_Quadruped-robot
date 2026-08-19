/**
 * @file robot_control_parameters.hpp
 * @brief GO1 与 DM1 独立的控制器调参集合。
 */
#ifndef MYMIT_ROBOT_MODEL_ROBOT_CONTROL_PARAMETERS_HPP_
#define MYMIT_ROBOT_MODEL_ROBOT_CONTROL_PARAMETERS_HPP_

#include <stdexcept>

#include "model/robot_types.hpp"

template<typename T>
struct RobotControlParameters
{
  T minimum_standing_height;       ///< UI/服务允许设置的最低机身高度，m。
  T maximum_standing_height;       ///< UI/服务允许设置的最高机身高度，m。
  T standing_height_rate;          ///< 高度目标最大变化速度，m/s。
  T balance_height_step;           ///< BalanceStand 单周期最大高度变化，m。
  T joint_initialization_duration; ///< 启动关节插值持续时间，s。
  Vec3<T> initialization_kp;       ///< 启动阶段三个关节的位置增益。
  Vec3<T> initialization_kd;       ///< 启动阶段三个关节的速度阻尼。
  bool start_in_prone_home;        ///< true 时上电保持趴卧电机零位，等待站立请求。
  Vec3<T> motor_zero_position;     ///< 趴卧校零姿态下的三个电机读数，rad。
  Vec3<T> prone_home_joint_kp;     ///< 趴卧零位锁定的关节位置增益。
  Vec3<T> prone_home_joint_kd;     ///< 趴卧零位锁定的关节速度阻尼。

  Vec3<T> balance_body_position_kp;    ///< 站立时机身 xyz 位置比例增益。
  Vec3<T> balance_body_position_kd;    ///< 站立时机身 xyz 速度阻尼。
  Vec3<T> balance_body_orientation_kp; ///< 站立时 roll/pitch/yaw 姿态增益。
  Vec3<T> balance_body_orientation_kd; ///< 站立时角速度阻尼。
  Vec3<T> balance_joint_kp;            ///< 站立零空间关节位置增益。
  Vec3<T> balance_joint_kd;            ///< 站立零空间关节速度阻尼。
  T balance_floating_base_weight;      ///< WBC 浮动基座加速度代价权重。
  T balance_reaction_force_weight;     ///< WBC 地面反力修正代价权重。
  T maximum_normal_force;              ///< 单足允许的最大竖直地面反力，N。
  T standing_supported_mass;           ///< 站立反力前馈所支撑的等效质量，kg。
  bool use_go1_height_joint_mapping;   ///< 是否使用 GO1 专用高度到腿角映射。

  T stand_up_duration;             ///< StandUp 预备轨迹持续时间，s。
  Vec3<T> stand_up_prepare_position; ///< 趴卧展开到支撑区的机械关节目标，rad。
  Vec3<T> stand_up_joint_kp;       ///< 趴卧收腿阶段三个关节的位置增益。
  Vec3<T> stand_up_joint_kd;       ///< 趴卧收腿阶段三个关节的速度阻尼。
  Vec3<T> stand_up_cartesian_kp;   ///< StandUp 足端 xyz 位置增益。
  Vec3<T> stand_up_cartesian_kd;   ///< StandUp 足端 xyz 速度阻尼。

  Vec3<T> locomotion_body_orientation_kp; ///< 行走时机身姿态比例增益。
  Vec3<T> locomotion_body_orientation_kd; ///< 行走时机身角速度阻尼。
  Vec3<T> locomotion_joint_kp;            ///< 行走时三类关节位置增益。
  Vec3<T> locomotion_joint_kd;            ///< 行走时三类关节速度阻尼。
  T locomotion_swing_height;              ///< 行走摆动足抬升高度，m。
  T locomotion_max_lateral_foot_offset;   ///< 足端相对髋的横向安全边界，m。

  T jump_crouch_duration;           ///< 前跳预蹲阶段持续时间，s。
  T jump_thrust_duration;           ///< 前跳蹬伸阶段持续时间，s。
  T jump_tuck_duration;             ///< 前跳腾空收腿阶段持续时间，s。
  T jump_landing_duration;          ///< 前跳落地回站阶段持续时间，s。
  Vec3<T> jump_crouch_position;     ///< 四腿预蹲机械关节目标 [Hip, thigh, calf]。
  Vec3<T> jump_front_thrust_position; ///< 前腿蹬伸机械关节目标。
  Vec3<T> jump_rear_thrust_position;  ///< 后腿蹬伸机械关节目标。
  Vec3<T> jump_tuck_position;       ///< 腾空收腿机械关节目标。
  T jump_crouch_kp;                 ///< 预蹲关节位置增益。
  T jump_crouch_kd;                 ///< 预蹲关节速度阻尼。
  T jump_thrust_kp;                 ///< 蹬伸关节位置增益。
  T jump_thrust_kd;                 ///< 蹬伸关节速度阻尼。
  T jump_tuck_kp;                   ///< 收腿关节位置增益。
  T jump_tuck_kd;                   ///< 收腿关节速度阻尼。
  T jump_landing_kp;                ///< 落地关节位置增益。
  T jump_landing_kd;                ///< 落地关节速度阻尼。
  T jump_pitch_position_gain;       ///< 落地俯仰角差动腿长修正增益。
  T jump_pitch_velocity_gain;       ///< 落地俯仰角速度修正增益。
  T jump_pitch_correction_limit;    ///< 单次俯仰关节修正最大幅值，rad。
};

/** @brief 返回指定机型独立的一套控制参数。 */
template<typename T>
RobotControlParameters<T> makeRobotControlParameters(RobotType robot_type)
{
  if (robot_type == RobotType::UNITREE_GO1) {
    return {
      T(0.18), T(0.34), T(0.16), T(0.001), T(0.4),
      Vec3<T>::Constant(T(60)), Vec3<T>::Constant(T(3)),
      false, Vec3<T>::Zero(),
      Vec3<T>::Constant(T(60)), Vec3<T>::Constant(T(3)),
      Vec3<T>::Constant(T(50)), Vec3<T>::Constant(T(1)),
      Vec3<T>::Constant(T(50)), Vec3<T>::Constant(T(1)),
      Vec3<T>::Constant(T(20)), Vec3<T>::Constant(T(2)),
      T(1000), T(1), T(1500), T(5.204), true,
      T(0.5), Vec3<T>::Zero(),
      Vec3<T>::Constant(T(60)), Vec3<T>::Constant(T(3)),
      Vec3<T>::Constant(T(500)), Vec3<T>::Constant(T(8)),
      Vec3<T>(T(100), T(100), T(50)), Vec3<T>(T(10), T(10), T(3)),
      Vec3<T>(T(30), T(42), T(42)), Vec3<T>(T(4), T(4.5), T(4.5)),
      T(0.09), T(0.18),
      T(0.18), T(0.18), T(0.17), T(0.25),
      Vec3<T>(T(0), T(1.22), T(-2.42)),
      Vec3<T>(T(0), T(1.20), T(-1.25)),
      Vec3<T>(T(0), T(1.10), T(-1.20)),
      Vec3<T>(T(0), T(1.12), T(-2.24)),
      T(48), T(5), T(70), T(4), T(32), T(3), T(44), T(6),
      T(0.35), T(0.06), T(0.12)};
  }
  if (robot_type == RobotType::DM_BOT1) {
    // DM1 更重且采用趴卧校零：上电保持电机零位，站立/行走使用独立站姿。
    return {
      T(0.30), T(0.42), T(0.08), T(0.00016), T(1.2),
      Vec3<T>(T(28), T(36), T(36)), Vec3<T>(T(4), T(5), T(5)),
      true, Vec3<T>::Zero(),
      Vec3<T>(T(1000), T(280), T(280)), Vec3<T>(T(25), T(12), T(12)),
      Vec3<T>(T(45), T(45), T(90)), Vec3<T>(T(8), T(8), T(14)),
      Vec3<T>(T(100), T(100), T(40)), Vec3<T>(T(18), T(18), T(8)),
      Vec3<T>(T(30), T(42), T(42)), Vec3<T>(T(4), T(5), T(5)),
      T(300), T(1), T(150), T(14.705035), false,
      T(1.2), Vec3<T>(T(0), T(0.1), T(-2.15)),
      Vec3<T>(T(35), T(70), T(80)), Vec3<T>(T(5), T(8), T(10)),
      Vec3<T>(T(220), T(220), T(300)), Vec3<T>(T(12), T(12), T(16)),
      Vec3<T>(T(70), T(70), T(35)), Vec3<T>(T(12), T(12), T(6)),
      Vec3<T>(T(50), T(42), T(42)), Vec3<T>(T(8), T(5), T(5)),
      T(0.075), T(0.24),
      T(0.30), T(0.16), T(0.16), T(0.32),
      Vec3<T>(T(0), T(-0.55), T(-1.275)),
      Vec3<T>(T(0), T(-0.8), T(-0.2)),
      Vec3<T>(T(0), T(-1.0), T(-0.1)),
      Vec3<T>(T(0), T(-0.65), T(-1.15)),
      T(42), T(6), T(70), T(5), T(30), T(4), T(48), T(8),
      T(0.55), T(0.10), T(0.15)};
  }
  throw std::invalid_argument("unsupported robot control parameter profile");
}

#endif  // MYMIT_ROBOT_MODEL_ROBOT_CONTROL_PARAMETERS_HPP_
