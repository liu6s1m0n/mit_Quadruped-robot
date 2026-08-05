/*! @file sim_imu.hpp
 *  @brief 从 MuJoCo 仿真数据读取统一格式的 IMU 采样。
 */

#ifndef MYMIT_ROBOT_SENSOR_sim_imu_HPP_
#define MYMIT_ROBOT_SENSOR_sim_imu_HPP_

#include <string>

#include <mujoco/mujoco.h>

#include "model/robot_types.hpp"

/**
 * @brief 将 MuJoCo 的 IMU 传感器输出转换为 ImuData<float>。
 *
 * 构造时完成传感器名称、类型和维度检查，并缓存 sensordata 地址；read()
 * 可在仿真控制循环中直接调用，不会重复查找传感器名称。
 */
class Imu
{
public:
  /**
   * @brief 构造函数：在 MuJoCo 模型中定位并校验三个 IMU 传感器。
   *
   * @param model               MuJoCo 模型指针，不能为空（为空会抛异常）。
   * @param orientation_sensor   姿态传感器的名称，类型须为 FRAMEQUAT（四元数）。
   *                           默认为 "imu_orientation"。
   * @param angular_velocity_sensor 角速度传感器的名称，类型须为 GYRO（陀螺仪）。
   *                           默认为 "imu_angular_velocity"。
   * @param acceleration_sensor 加速度传感器的名称，类型须为 ACCELEROMETER（加速度计）。
   *                           默认为 "imu_linear_acceleration"。
   */
  explicit Imu(
    const mjModel * model,
    const std::string & orientation_sensor = "imu_orientation",
    const std::string & angular_velocity_sensor = "imu_angular_velocity",
    const std::string & acceleration_sensor = "imu_linear_acceleration");

  /**
   * @brief 读取当前仿真步的 IMU 数据；输入无效或数据非有限时 valid=false。
   *
   * @param data MuJoCo 仿真数据指针，包含当前步的 sensordata 与仿真时间。
   * @return 封装好的 IMU 数据（姿态四元数、角速度、加速度、时间戳、有效性标志）。
   */
  ImuData<float> read(const mjData * data) const noexcept;

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

  // 姿态传感器（四元数）数据在 sensordata 缓冲区中的起始地址
  int orientation_address_ = -1;
  // 角速度传感器（陀螺仪）数据在 sensordata 缓冲区中的起始地址
  int angular_velocity_address_ = -1;
  // 加速度传感器（加速度计）数据在 sensordata 缓冲区中的起始地址
  int acceleration_address_ = -1;
};

#endif  // MYMIT_ROBOT_SENSOR_sim_imu_HPP_
