/*! @file leg.hpp
 *  @brief 统一的单腿关节数据源：支持 MuJoCo 仿真与真实硬件接口。
 */

#ifndef MYMIT_ROBOT_SENSOR_LEG_HPP_
#define MYMIT_ROBOT_SENSOR_LEG_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "model/robot_types.hpp"

/**
 * @brief 单腿关节反馈的数据来源。
 *
 * SIMULATOR：从 MuJoCo 仿真器读取；HARDWARE：从真实电机或总线读取。
 */
enum class LegSource : std::uint8_t
{
  SIMULATOR = 0,
  HARDWARE = 1
};

/**
 * @brief 单腿关节反馈的统一读取接口。
 *
 * read() 返回 robot_types.hpp 定义的 JointState<float>，并将最近一次结果
 * 保存到成员 leg 中。控制算法不需要区分数据来自 MuJoCo 还是真实硬件。
 */
class LegSensor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief 虚析构函数，保证通过基类指针释放派生数据源时行为正确。 */
  virtual ~LegSensor() = default;

  /** @brief 读取一次单腿反馈；结果同时保存到成员 leg 中。 */
  virtual JointState<float> read() = 0;

  /** @brief 最近一次 read() 读取并保存的单腿关节反馈。 */
  JointState<float> leg;
};

/**
 * @brief 从 MuJoCo 的 qpos、qvel 和 qfrc_actuator 读取一条腿的关节反馈。
 *
 * 构造时按名称检查三个关节并缓存地址，read() 中不再进行名称查找。
 * torque_estimate 表示 MuJoCo 当前计算出的关节执行器广义力。
 */
class SimLeg : public LegSensor
{
public:
  /** @brief 三个关节名称，顺序固定为 Hip、thigh、calf。 */
  using JointNames = std::array<std::string, kJointsPerLeg>;

  /**
   * @brief 使用 GO1 默认关节名称绑定一条 MuJoCo 仿真腿。
   *
   * @param model  MuJoCo 模型指针，不能为空。
   * @param data   MuJoCo 仿真数据指针，read() 从中读取关节状态。
   * @param leg_id 腿编号，用于生成 FR/FL/RR/RL 对应的默认关节名称。
   */
  explicit SimLeg(const mjModel * model, const mjData * data, LegId leg_id);

  /**
   * @brief 使用调用方给定的三个关节名称绑定一条 MuJoCo 仿真腿。
   *
   * @param model       MuJoCo 模型指针，不能为空。
   * @param data        MuJoCo 仿真数据指针。
   * @param leg_id      返回的 JointState 中保存的腿编号。
   * @param joint_names Hip、thigh、calf 三个关节在 MJCF 中的名称。
   */
  SimLeg(
    const mjModel * model, const mjData * data, LegId leg_id,
    const JointNames & joint_names);

  /**
   * @brief 从绑定的 mjData 读取位置、速度、执行器力矩和仿真时间。
   * @return 统一格式的 JointState<float>；数据异常时 valid=false。
   */
  JointState<float> read() override;

private:
  /** @brief 一个标量铰链关节在 qpos 和自由度数组中的地址。 */
  struct JointAddress
  {
    int position = -1;  ///< 关节位置在 mjData::qpos 中的地址。
    int velocity = -1;  ///< 关节速度和广义力在自由度数组中的地址。
  };

  /**
   * @brief 根据腿编号生成 GO1 默认关节名称。
   * @param leg_id 腿编号。
   * @return 按 Hip、thigh、calf 排列的三个 MJCF 关节名称。
   */
  static JointNames defaultJointNames(LegId leg_id);

  /**
   * @brief 按名称查找铰链关节，并校验 qpos/qvel 地址。
   * @param model MuJoCo 模型指针。
   * @param name  MJCF 中的关节名称。
   * @return 可直接用于读取 mjData 的位置与自由度地址。
   */
  static JointAddress requireJoint(const mjModel * model, const std::string & name);

  // 绑定的 MuJoCo 仿真数据；read() 只读取，不负责推进仿真。
  const mjData * data_ = nullptr;
  // 当前读取器对应的腿编号。
  LegId leg_id_ = LegId::FR;
  // Hip、thigh、calf 三个关节缓存的数据地址。
  std::array<JointAddress, kJointsPerLeg> joint_addresses_{};
};

/**
 * @brief 真实硬件单腿反馈接口的占位实现。
 *
 * 接入电机驱动或总线后，在 read() 中填充三个关节的位置、速度、估计力矩和
 * 硬件时间戳。当前返回 valid=false，避免控制器误用零值。
 */
class HardwareLeg : public LegSensor
{
public:
  /**
   * @brief 创建真实硬件腿接口并保存腿编号。
   * @param leg_id 真实硬件反馈所属的腿。
   */
  explicit HardwareLeg(LegId leg_id);

  /**
   * @brief 读取真实硬件反馈的预留入口。
   * @return 驱动尚未接入时返回 valid=false 的 JointState<float>。
   */
  JointState<float> read() override;

private:
  // 当前硬件接口对应的腿编号。
  LegId leg_id_ = LegId::FR;
};

/**
 * @brief 按来源创建一条腿的数据读取器。
 *
 * @param source 数据来源：MuJoCo 仿真器或真实硬件。
 * @param leg_id 要读取的腿编号。
 * @param model  仿真器模式需要的 MuJoCo 模型；硬件模式可传 nullptr。
 * @param data   仿真器模式需要的 MuJoCo 数据；硬件模式可传 nullptr。
 * @return 对应数据源的 unique_ptr；未知来源时抛出 std::invalid_argument。
 */
std::unique_ptr<LegSensor> makeLeg(
  LegSource source, LegId leg_id,
  const mjModel * model = nullptr, const mjData * data = nullptr);

#endif  // MYMIT_ROBOT_SENSOR_LEG_HPP_
