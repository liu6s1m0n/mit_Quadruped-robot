/*! @file imu.hpp
 *  @brief 统一的 IMU 数据源：可选择从仿真器（MuJoCo）或真实硬件读取，
 *         每次读取结果统一保存为 ImuData<float>。
 */

#ifndef MYMIT_ROBOT_SENSOR_IMU_HPP_
#define MYMIT_ROBOT_SENSOR_IMU_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "model/robot_types.hpp"

/**
 * @brief IMU 数据来源类型。
 *
 * SIMULATOR：从 MuJoCo 仿真器读取；HARDWARE：从真实硬件读取（尚未实现）。
 */
enum class ImuSource : std::uint8_t
{
  SIMULATOR = 0,
  HARDWARE = 1
};

/**
 * @brief 统一的 IMU 数据源接口。
 *
 * 仿真器与真实硬件各自实现 read()，返回相同格式的 ImuData<float>。
 * 硬件实现应把设备输出的姿态角转换为 orientation_world_from_body，
 * 而不是只上传原始陀螺仪数据交给 OrientationEstimator 积分；
 * 每次读取的结果都会保存到成员 imu 中，供控制循环直接访问。
 */
class ImuSensor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~ImuSensor() = default;

  /** @brief 读取一次 IMU 数据；结果同时保存到成员 imu 中。 */
  virtual ImuData<float> read() = 0;

  /** @brief 最近一次 read() 读取并保存的 IMU 采样数据。 */
  ImuData<float> imu;
};

/**
 * @brief 仿真器 IMU 数据源：从 MuJoCo 仿真数据读取。
 *
 * 构造时完成传感器名称、类型和维度检查，并缓存 sensordata 地址；
 * read() 在仿真控制循环中直接调用，不会重复查找传感器名称。
 */
class SimImu : public ImuSensor
{
public:
  /**
   * @brief 构造函数：绑定 MuJoCo 模型与仿真数据，并定位三个 IMU 传感器。
   *
   * @param model               MuJoCo 模型指针，不能为空（为空会抛异常）。
   * @param data                MuJoCo 仿真数据指针，read() 从其中读取 sensordata。
   * @param orientation_sensor   姿态传感器的名称，类型须为 FRAMEQUAT（四元数）。
   *                           默认为 "imu_orientation"。
   * @param angular_velocity_sensor 角速度传感器的名称，类型须为 GYRO（陀螺仪）。
   *                           默认为 "imu_angular_velocity"。
   * @param acceleration_sensor 加速度传感器的名称，类型须为 ACCELEROMETER（加速度计）。
   *                           默认为 "imu_linear_acceleration"。
   */
  explicit SimImu(
    const mjModel * model, const mjData * data,
    const std::string & orientation_sensor = "imu_orientation",
    const std::string & angular_velocity_sensor = "imu_angular_velocity",
    const std::string & acceleration_sensor = "imu_linear_acceleration");

  ImuData<float> read() override;

private:
  /**
   * @brief 按名称查找传感器并校验其类型、维度与数据地址。
   *
   * @param model             MuJoCo 模型指针。
   * @param name              要查找的传感器名称（如 "imu_orientation"）。
   * @param expected_type     期望的传感器类型（如 mjSENS_FRAMEQUAT）。
   * @param expected_dimension 期望的传感器维度（姿态为 4，角速度/加速度为 3）。
   * @return 该传感器数据在 sensordata 缓冲区中的起始偏移地址。
   */
  static int requireSensor(
    const mjModel * model, const std::string & name,
    int expected_type, int expected_dimension);

  // 绑定的 MuJoCo 仿真数据指针，read() 时从中读取 sensordata
  const mjData * data_ = nullptr;
  // 姿态传感器（四元数）数据在 sensordata 缓冲区中的起始地址
  int orientation_address_ = -1;
  // 角速度传感器（陀螺仪）数据在 sensordata 缓冲区中的起始地址
  int angular_velocity_address_ = -1;
  // 加速度传感器（加速度计）数据在 sensordata 缓冲区中的起始地址
  int acceleration_address_ = -1;
};

/**
 * @brief 真实硬件 IMU 数据源（占位实现，尚未完成）。
 *
 * 当前 read() 始终返回无效数据（valid == false），避免调用方误用占位值；
 * 接入可输出姿态角和角速度的真实 IMU 后在此实现驱动读取。
 */
class HardwareImu : public ImuSensor
{
public:
  HardwareImu() = default;

  ImuData<float> read() override;
};

/**
 * @brief 按来源类型创建 IMU 数据源。
 *
 * @param source 数据来源：仿真器或真实硬件。
 * @param model  仿真器模式需要的 MuJoCo 模型指针；硬件模式可传 nullptr。
 * @param data   仿真器模式需要的 MuJoCo 数据指针；硬件模式可传 nullptr。
 * @return 指向 IMU 数据源对象的 unique_ptr；未知来源时抛出 std::invalid_argument。
 */
std::unique_ptr<ImuSensor> makeImu(
  ImuSource source, const mjModel * model = nullptr, const mjData * data = nullptr);

#endif  // MYMIT_ROBOT_SENSOR_IMU_HPP_
