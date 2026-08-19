// ============================================================
// 四足腿部控制器实现
// 负责反馈数据整理、腿部运动学计算和统一 JointCommand 生成。
// ============================================================

#include "controller/leg_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace
{

// 将腿编号转换为四腿数组下标。
// LegId 的正常取值依次为 FR=0、FL=1、RR=2、RL=3；由于枚举可以被强制
// 转换成其他数值，这里统一进行边界检查，避免访问 commands/datas 数组越界。
std::size_t checkedLegIndex(LegId leg_id)
{
  // 枚举底层值正好与四腿数组下标一致。
  const auto index = static_cast<std::size_t>(leg_id);
  // kNumLegs 当前为 4，任何大于等于 4 的值都是非法腿编号。
  if (index >= kNumLegs) {
    throw std::invalid_argument("invalid leg id");
  }
  return index;
}

}  // namespace

// ---------- 控制命令和反馈结构初始化 ----------

// 清空一条腿的全部期望命令。
// 同时清空关节空间和笛卡尔空间字段，确保旧控制模式留下的增益或前馈量
// 不会在切换控制模式后继续生效。
template<typename T>
void LegControllerCommand<T>::zero()
{
  // 清空关节力矩与足端力两个前馈项。
  torque_feedforward.setZero();
  force_feedforward.setZero();
  // 清空关节空间的位置、速度期望。
  position_desired.setZero();
  velocity_desired.setZero();
  // 清空笛卡尔空间的足端位置、速度期望。
  foot_position_desired.setZero();
  foot_velocity_desired.setZero();
  // 清空笛卡尔阻抗增益。
  kp_cartesian.setZero();
  kd_cartesian.setZero();
  // 清空关节 PD 增益。
  kp_joint.setZero();
  kd_joint.setZero();
}

// 清空一条腿的反馈和派生运动学数据。
// zero() 后 valid=false；调用者必须等待下一次有效 updateData()，才能使用
// q/qd/p/v/J 等字段进行控制。
template<typename T>
void LegControllerData<T>::zero()
{
  // 使用 FR 作为安全默认枚举值；更新具体腿时调用方会立即覆盖它。
  leg = LegId::FR;
  // 清空直接来自传感器的关节反馈。
  q.setZero();
  qd.setZero();
  // 清空由关节反馈计算出的足端运动学结果。
  p.setZero();
  v.setZero();
  J.setZero();
  // 清空力矩反馈和采样时间。
  torque_estimate.setZero();
  timestamp = T(0);
  // 清零数据不能被控制器当成一次真实采样。
  valid = false;
  // Quadruped 由所属 LegController 在清零后重新绑定。
  quadruped = nullptr;
}

// ---------- LegController 生命周期和安全命令 ----------

// 构造四腿控制器。
// quadruped 以只读引用保存，因此它的生命周期必须长于本控制器。
template<typename T>
LegController<T>::LegController(const Quadruped<T> & quadruped)
: _quadruped(quadruped)
{
  // datas 数组下标与 LegId 底层值一一对应：0/1/2/3 = FR/FL/RR/RL。
  for (std::size_t index = 0; index < kNumLegs; ++index) {
    const auto leg_id = static_cast<LegId>(index);
    // 预先写入腿编号，便于外部直接按数组查看反馈所属腿。
    datas[index].leg = leg_id;
    // 四条腿共享同一份只读 Quadruped 参数，不复制模型数据。
    datas[index].setQuadruped(_quadruped);
  }
  // 构造完成时默认禁止输出，防止控制参数尚未设置便驱动电机。
  zeroCommand();
}

// 清空四条腿的所有命令，并关闭总使能。
// 该函数适合控制器初始化、模式切换和普通安全停机；由于 kd 也会被清零，
// 如果需要主动速度阻尼，应改用 edampCommand()。
template<typename T>
void LegController<T>::zeroCommand()
{
  // 逐条腿清除关节命令和笛卡尔命令。
  for (auto & leg_command : commands) {
    leg_command.zero();
  }
  // 即使命令数组均为零，也明确通知后续执行层不要使能输出。
  _legsEnabled = false;
}

// 设置四条腿的紧急阻尼命令。
// 生成的关节控制形式为 tau = kd * (0 - qd) = -gain * qd，机器人运动越快，
// 反向阻尼力矩越大，用于在异常状态下抑制关节运动。
template<typename T>
void LegController<T>::edampCommand(RobotType robot, T gain)
{
  // 防止用其他机型选择的安全参数控制当前机器人。
  if (robot != _quadruped.robotType()) {
    throw std::invalid_argument("emergency damping robot type does not match model");
  }
  // 负增益会产生正反馈，NaN/Inf 也不能下发，所以在修改命令前拒绝它们。
  if (!std::isfinite(static_cast<double>(gain)) || gain < T(0)) {
    throw std::invalid_argument("emergency damping gain must be finite and non-negative");
  }

  // 先清除之前的位置、前馈力和其他增益，避免与紧急阻尼叠加。
  zeroCommand();
  // 四条腿、每条腿三个关节使用相同的速度阻尼增益。
  for (auto & leg_command : commands) {
    leg_command.kd_joint.setConstant(gain);
  }
  // 阻尼命令必须被发送给执行层，因此在参数检查通过后重新打开总使能。
  _legsEnabled = true;
}

// ---------- 统一反馈读取和运动学更新 ----------

// 使用一次 JointState 更新它所属腿的控制器反馈。
// 成功路径：复制 q/qd/tau -> 计算 p/J -> 计算 v=J*qd -> 标记有效。
// 失败路径：清空对应 LegControllerData 并返回 false，防止保留上次旧数据。
template<typename T>
bool LegController<T>::updateData(const JointState<T> & state)
{
  // state 自带腿编号，先将其映射到 datas[FR/FL/RR/RL] 的对应位置。
  const std::size_t index = checkedLegIndex(state.leg);
  auto & data = datas[index];

  // 输入层已将读取失败标记为 valid=false；这里再次检查全部数值是否有限，
  // 防止 NaN/Inf 经过三角函数、矩阵乘法继续扩散到控制命令中。
  if (!state.valid || !state.position.allFinite() ||
    !state.velocity.allFinite() || !state.torque_estimate.allFinite() ||
    !std::isfinite(static_cast<double>(state.timestamp)))
  {
    // 丢弃该腿此前的有效反馈，避免控制器误以为旧数据仍是当前数据。
    data.zero();
    // zero() 会恢复默认 FR，因此需要写回本次输入实际对应的腿编号。
    data.leg = state.leg;
    // zero() 同时清除了模型指针；重新绑定以保持数据结构初始化完整。
    data.setQuadruped(_quadruped);
    return false;
  }

  // 复制传感器直接测得或仿真器直接读取的字段。
  data.leg = state.leg;
  data.q = state.position;
  data.qd = state.velocity;
  data.torque_estimate = state.torque_estimate;
  data.timestamp = state.timestamp;
  // 根据当前关节角和对应腿几何参数计算足端位置 p 与雅可比 J。
  computeLegJacobianAndPosition(
    _quadruped, data.q, &data.J, &data.p, state.leg);
  // 雅可比将三个关节角速度映射为腿部坐标系中的足端线速度。
  data.v = data.J * data.qd;
  // 理论计算结果仍做最终有限性检查，通过后才允许 command() 使用。
  data.valid = data.p.allFinite() && data.v.allFinite() && data.J.allFinite();
  return data.valid;
}

// 直接从统一腿传感器接口读取数据。
// SimLeg 和 HardwareLeg 都继承 LegSensor，因此上层只需替换 source 对象，
// 不需要修改 LegController 内部的数据处理流程。
template<typename T>
bool LegController<T>::updateData(LegSensor & source)
{
  // 当前 LegSensor 公共接口固定返回 float 精度 JointState。
  const JointState<float> sensor_state = source.read();
  // 将传感器结果转换为控制器模板使用的数值类型 T。
  JointState<T> state;
  state.leg = sensor_state.leg;
  state.position = sensor_state.position.template cast<T>();
  state.velocity = sensor_state.velocity.template cast<T>();
  state.torque_estimate = sensor_state.torque_estimate.template cast<T>();
  state.timestamp = static_cast<T>(sensor_state.timestamp);
  state.valid = sensor_state.valid;
  // 所有合法性检查和运动学计算都复用 JointState 重载，避免两套逻辑不一致。
  return updateData(state);
}

// ---------- 统一关节命令生成 ----------

// 生成一条腿可供仿真器或硬件执行层使用的 JointCommand。
// 本函数不直接写 mjData::ctrl，也不直接发送电机总线数据，只完成控制空间转换、
// 有效性检查和前馈力矩限幅。
template<typename T>
JointCommand<T> LegController<T>::command(LegId leg_id, T timestamp)
{
  // 取得该腿在四腿数组中的位置，并引用其期望命令与实时反馈。
  const std::size_t index = checkedLegIndex(leg_id);
  const auto & desired = commands[index];
  const auto & data = datas[index];

  // 默认构造的 JointCommand enabled=false，只有全部检查通过后才会改为 true。
  JointCommand<T> result;
  result.leg = leg_id;
  result.timestamp = timestamp;
  // 每调用一次递增一次，四条腿分别计数，便于执行层检测重复帧或丢帧。
  result.sequence = ++command_sequences_[index];

  // JointCommand 可以直接表达关节空间 PD：
  // tau_joint = kp*(q_des-q) + kd*(qd_des-qd)。
  result.position_desired = desired.position_desired;
  result.velocity_desired = desired.velocity_desired;
  result.kp = desired.kp_joint;
  result.kd = desired.kd_joint;
  // 先复制用户给定的关节前馈；后面再叠加笛卡尔控制产生的前馈力矩。
  result.torque_feedforward = desired.torque_feedforward;

  // 检查每一个参与命令计算的输入。即使某个控制增益当前为零，也检查其对应
  // 期望量，以防切换控制模式后隐藏的 NaN/Inf 突然进入输出。
  const bool desired_is_finite =
    desired.torque_feedforward.allFinite() &&
    desired.force_feedforward.allFinite() &&
    desired.position_desired.allFinite() &&
    desired.velocity_desired.allFinite() &&
    desired.foot_position_desired.allFinite() &&
    desired.foot_velocity_desired.allFinite() &&
    desired.kp_cartesian.allFinite() && desired.kd_cartesian.allFinite() &&
    desired.kp_joint.allFinite() && desired.kd_joint.allFinite() &&
    std::isfinite(static_cast<double>(timestamp));

  // 三道安全门：总使能已打开、反馈是本周期有效数据、命令全部为有限数值。
  // 任一条件不满足都保持 enabled=false，执行层必须据此进入安全模式。
  if (!_legsEnabled || !data.valid || !desired_is_finite) {
    result.enabled = false;
    return result;
  }

  // 在腿部笛卡尔坐标系中计算期望足端作用力：
  //   前馈力 + 位置误差产生的弹簧力 + 速度误差产生的阻尼力。
  const Vec3<T> foot_force =
    desired.force_feedforward +
    desired.kp_cartesian * (desired.foot_position_desired - data.p) +
    desired.kd_cartesian * (desired.foot_velocity_desired - data.v);
  // 根据虚功原理，用雅可比转置将足端力转换为三个关节的等效力矩，
  // 再叠加调用方直接设置的关节前馈力矩。
  result.torque_feedforward += data.J.transpose() * foot_force;

  // 从当前机型、当前腿的模型参数中取得 [Hip, thigh, calf] 力矩上限。
  const Vec3<T> torque_limit = _quadruped.leg(leg_id).joints.torque_limit;
  // 将三个前馈力矩分别限制在 [-torque_limit, +torque_limit]。
  // 注意：本层只知道前馈项；执行层计算出“前馈 + PD”后仍必须再次限幅。
  result.torque_feedforward =
    result.torque_feedforward.cwiseMin(torque_limit).cwiseMax(-torque_limit);
  // 转换和限幅结果仍为有限数值时，才正式允许执行层下发此命令。
  result.enabled = result.torque_feedforward.allFinite();
  return result;
}

// ---------- 单腿正运动学和雅可比 ----------

// 计算一条三自由度腿的足端位置 p 和解析雅可比 J。注意角度为0的位置，腿是竖直向下的
//
// 关节向量约定：
//   q(0) = Hip 外展/内收角；q(1) = thigh 俯仰角；q(2) = calf 俯仰角。
// 坐标约定：
//   x 为机器人前方，y 为机器人左方，z 为机器人上方。
//   计算结果使用以该腿 Hip 安装点为原点的腿部局部坐标系。
//
// 左腿和右腿的 Hip 横向连杆互为镜像，side_sign 用 +1/-1 统一两侧公式。
template<typename T>
void computeLegJacobianAndPosition(
  const Quadruped<T> & quad, const Vec3<T> & q,
  Mat3<T> * J, Vec3<T> * p, LegId leg_id)
{
  // 由 LegId 取得这一条腿的模型参数；不直接访问具体 GO1 参数文件。
  const auto & leg = quad.leg(leg_id);
  if (quad.robotType() == RobotType::UNITREE_GO1) {
    // 保留 GO1 已通过长期仿真回归的解析公式，避免改变其浮点运算路径。
    const T l1 = leg.hip_link_length;
    const T l2 = leg.thigh_link_length;
    const T l3 = leg.calf_link_length;
    const T side_sign = quad.sideSign(leg_id);
    const T s1 = std::sin(q(0));
    const T s2 = std::sin(q(1));
    const T s3 = std::sin(q(2));
    const T c1 = std::cos(q(0));
    const T c2 = std::cos(q(1));
    const T c3 = std::cos(q(2));
    const T c23 = c2 * c3 - s2 * s3;
    const T s23 = s2 * c3 + c2 * s3;
    if (J != nullptr) {
      (*J)(0, 0) = T(0);
      (*J)(0, 1) = -l3 * c23 - l2 * c2;
      (*J)(0, 2) = -l3 * c23;
      (*J)(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - l1 * side_sign * s1;
      (*J)(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
      (*J)(1, 2) = -l3 * s1 * s23;
      (*J)(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + l1 * side_sign * c1;
      (*J)(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
      (*J)(2, 2) = l3 * c1 * s23;
    }
    if (p != nullptr) {
      (*p)(0) = -l3 * s23 - l2 * s2;
      (*p)(1) = l1 * side_sign * c1 + l3 * s1 * c23 + l2 * c2 * s1;
      (*p)(2) = l1 * side_sign * s1 - l3 * c1 * c23 - l2 * c1 * c2;
    }
    return;
  }
  // 把现场校零后的电机读数还原为机械关节角，再计算通用串联链运动学。
  const Vec3<T> mechanical_q = q + leg.joints.zero_offset;
  const Mat3<T> rotation_hip =
    Eigen::AngleAxis<T>(
    mechanical_q(0), leg.joints.joint_axes.col(0)).toRotationMatrix();
  const Mat3<T> rotation_thigh =
    Eigen::AngleAxis<T>(
    mechanical_q(1), leg.joints.joint_axes.col(1)).toRotationMatrix();
  const Mat3<T> rotation_calf =
    Eigen::AngleAxis<T>(
    mechanical_q(2), leg.joints.joint_axes.col(2)).toRotationMatrix();
  const Vec3<T> hip_origin = Vec3<T>::Zero();
  const Vec3<T> thigh_origin = rotation_hip * leg.hip_to_thigh;
  const Mat3<T> hip_thigh_rotation = rotation_hip * rotation_thigh;
  const Vec3<T> calf_origin =
    thigh_origin + hip_thigh_rotation * leg.thigh_to_calf;
  const Vec3<T> foot_position = calf_origin +
    hip_thigh_rotation * rotation_calf * leg.calf_to_foot;

  if (J != nullptr) {
    const Vec3<T> hip_axis = leg.joints.joint_axes.col(0);
    const Vec3<T> thigh_axis = rotation_hip * leg.joints.joint_axes.col(1);
    const Vec3<T> calf_axis =
      hip_thigh_rotation * leg.joints.joint_axes.col(2);
    J->col(0) = hip_axis.cross(foot_position - hip_origin);
    J->col(1) = thigh_axis.cross(foot_position - thigh_origin);
    J->col(2) = calf_axis.cross(foot_position - calf_origin);
  }
  if (p != nullptr) {*p = foot_position;}
}

// 模板实现放在 .cpp 中，因此需要显式生成项目当前实际使用的 float 版本。
// 以后若整个模型库增加 double 支持，应在这里同步增加 double 显式实例化。
template struct LegControllerCommand<float>;
template struct LegControllerData<float>;
template class LegController<float>;
template void computeLegJacobianAndPosition<float>(
  const Quadruped<float> &, const Vec3<float> &,
  Mat3<float> *, Vec3<float> *, LegId);
template void computeLegJacobianAndPosition<double>(
  const Quadruped<double> &, const Vec3<double> &,
  Mat3<double> *, Vec3<double> *, LegId);
