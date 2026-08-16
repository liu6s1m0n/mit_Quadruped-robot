// ============================================================
// 单腿关节数据源实现
// 包含仿真器数据源（SimLeg）与真实硬件数据源（HardwareLeg，占位），
// 以及按来源类型创建数据源的工厂函数 makeLeg()。
// ============================================================

#include "sensor/leg.hpp"

#include <cmath>
#include <stdexcept>

// ---------- 内部辅助函数（匿名命名空间） ----------
namespace
{

// 将腿编号转换为 GO1 MJCF 关节名称使用的前缀。
// 非法枚举值会抛出异常，防止静默绑定到错误的腿。
const char * legPrefix(LegId leg_id)
{
  switch (leg_id) {
    case LegId::FR:
      return "FR";
    case LegId::FL:
      return "FL";
    case LegId::RR:
      return "RR";
    case LegId::RL:
      return "RL";
  }
  throw std::invalid_argument("invalid leg id");
}

// 校验一个标量数据地址是否落在给定数组范围内。
bool addressIsValid(int address, int size)
{
  return address >= 0 && address < size;
}

}  // namespace

// ---------- MuJoCo 仿真腿数据源（SimLeg） ----------

// 使用默认命名规则构造：例如 FR 对应 FR_hip_joint、FR_thigh_joint、
// FR_calf_joint；实际查找和校验交给另一个构造函数完成。
SimLeg::SimLeg(const mjModel * model, const mjData * data, LegId leg_id)
: SimLeg(model, data, leg_id, defaultJointNames(leg_id))
{
}

// 使用调用方指定的三个关节名称构造，并缓存 qpos/qvel 数据地址。
SimLeg::SimLeg(
  const mjModel * model, const mjData * data, LegId leg_id,
  const JointNames & joint_names)
: data_(data), leg_id_(leg_id)
{
  // 模型用于解析关节结构，构造阶段必须有效。
  if (model == nullptr) {
    throw std::invalid_argument("MuJoCo model must not be null");
  }
  // 主动校验 leg_id，即使调用方传入了自定义关节名称也不接受非法腿编号。
  static_cast<void>(legPrefix(leg_id));

  // 关节名称只在构造阶段查找一次，控制循环内直接使用缓存地址。
  for (std::size_t index = 0; index < joint_addresses_.size(); ++index) {
    joint_addresses_[index] = requireJoint(model, joint_names[index]);
  }
  // 初始化最近一次采样所属的腿，尚未 read() 时 valid 仍为 false。
  leg.leg = leg_id_;
}

// 读取一次单腿反馈：从 MuJoCo 中提取三个关节的位置、速度、执行器力矩和时间戳，
// 做有限性检查后写入统一的 JointState<float>，并同步保存到成员 leg。
JointState<float> SimLeg::read()
{
  // 默认构造的结果 valid=false，任何校验失败都可以安全返回。
  JointState<float> result;
  result.leg = leg_id_;

  // 仿真数据及三个必要数组必须存在，仿真时间也必须是有限数值。
  if (data_ == nullptr || data_->qpos == nullptr || data_->qvel == nullptr ||
    data_->qfrc_actuator == nullptr ||
    !std::isfinite(static_cast<float>(data_->time)))
  {
    leg = result;
    return result;
  }

  // 数组顺序与 JointState 约定一致：Hip、thigh、calf。
  for (std::size_t index = 0; index < joint_addresses_.size(); ++index) {
    const auto & address = joint_addresses_[index];
    // qpos 使用关节位置地址；标量 hinge 关节占一个元素。
    result.position[static_cast<Eigen::Index>(index)] =
      static_cast<float>(data_->qpos[address.position]);
    // qvel 使用该关节对应的自由度地址。
    result.velocity[static_cast<Eigen::Index>(index)] =
      static_cast<float>(data_->qvel[address.velocity]);
    // qfrc_actuator 与 qvel 共用自由度索引，表示当前执行器广义力。
    result.torque_estimate[static_cast<Eigen::Index>(index)] =
      static_cast<float>(data_->qfrc_actuator[address.velocity]);
  }
  // 使用 MuJoCo 单调递增的仿真时间作为采样时间戳。
  result.timestamp = static_cast<float>(data_->time);

  // 任一反馈包含 NaN/Inf 时丢弃整个采样，避免无效数值进入控制器。
  if (!result.position.allFinite() || !result.velocity.allFinite() ||
    !result.torque_estimate.allFinite())
  {
    result = JointState<float>{};
    result.leg = leg_id_;
    leg = result;
    return result;
  }

  // 所有字段校验通过后标记有效，并更新外部可访问的最近一次采样。
  result.valid = true;
  leg = result;
  return result;
}

// 根据腿编号生成 GO1 的三个默认关节名称，返回顺序固定为 Hip、thigh、calf。
SimLeg::JointNames SimLeg::defaultJointNames(LegId leg_id)
{
  const std::string prefix = legPrefix(leg_id);
  return {
    prefix + "_hip_joint",
    prefix + "_thigh_joint",
    prefix + "_calf_joint"};
}

// 在 MuJoCo 模型中查找一个关节，要求其为标量 hinge 关节，
// 并返回已经过 nq/nv 边界检查的位置和自由度地址。
SimLeg::JointAddress SimLeg::requireJoint(
  const mjModel * model, const std::string & name)
{
  // 通过 MJCF 关节名称取得模型内的关节 ID。
  const int joint_id = mj_name2id(model, mjOBJ_JOINT, name.c_str());
  if (joint_id < 0) {
    throw std::invalid_argument("MuJoCo leg joint not found: " + name);
  }
  // 当前 JointState 的每个关节只保存一个标量，因此只接受 hinge 关节。
  if (model->jnt_type[joint_id] != mjJNT_HINGE) {
    throw std::invalid_argument("MuJoCo leg joint is not a hinge joint: " + name);
  }

  // MuJoCo 分别提供 qpos 地址和自由度地址，二者不能假定相同。
  JointAddress result;
  result.position = model->jnt_qposadr[joint_id];
  result.velocity = model->jnt_dofadr[joint_id];
  // 构造阶段完成边界检查，read() 中便可直接读取缓存地址。
  if (!addressIsValid(result.position, model->nq) ||
    !addressIsValid(result.velocity, model->nv))
  {
    throw std::invalid_argument("MuJoCo leg joint has an invalid data address: " + name);
  }
  return result;
}

// ---------- 真实硬件腿数据源 ----------

// 保存硬件腿编号并初始化最近一次采样；非法腿编号会立即抛出异常。
HardwareLeg::HardwareLeg(LegId leg_id)
: leg_id_(leg_id)
{
  static_cast<void>(legPrefix(leg_id));
  leg.leg = leg_id_;
}

bool HardwareLeg::update(const JointState<float> & state)
{
  const bool state_valid = state.valid && state.leg == leg_id_ &&
    state.position.allFinite() && state.velocity.allFinite() &&
    state.torque_estimate.allFinite() && std::isfinite(state.timestamp);
  if (!state_valid) {
    leg = JointState<float>{};
    leg.leg = leg_id_;
    return false;
  }
  leg = state;
  return true;
}

JointState<float> HardwareLeg::read()
{
  return leg;
}

// ---------- 工厂：按来源类型创建单腿数据源 ----------

// 仿真器来源需要有效的 model/data；硬件来源只使用 leg_id，
// 调用方始终通过 LegSensor 基类获得相同的 read() 接口。
std::unique_ptr<LegSensor> makeLeg(
  LegSource source, LegId leg_id, const mjModel * model, const mjData * data)
{
  switch (source) {
    case LegSource::SIMULATOR:
      return std::make_unique<SimLeg>(model, data, leg_id);
    case LegSource::HARDWARE:
      return std::make_unique<HardwareLeg>(leg_id);
  }
  throw std::invalid_argument("unknown leg source");
}
