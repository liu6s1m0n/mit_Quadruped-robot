// ============================================================
// IMU 传感器类实现
// 从 MuJoCo 仿真数据中读取 IMU（惯性测量单元）的
// 姿态、角速度和加速度信息，并对数据进行有效性校验。
// ============================================================

#include "sensor/sim_imu.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

// ---------- 内部辅助函数（匿名命名空间） ----------
namespace
{

// 校验传感器数据地址是否合法：
// 地址非负、维度为正，且地址与维度之和不超过传感器数据缓冲区的总长度。
bool sensorRangeIsValid(const mjModel * model, int address, int dimension)
{
  return address >= 0 && dimension > 0 &&
         address <= model->nsensordata - dimension;
}

}  // namespace

// ---------- 构造函数：解析并定位三个 IMU 传感器 ----------
// 根据给定的传感器名称，在模型中查找并校验
// 姿态（四元数）、角速度（陀螺仪）和加速度（加速度计）传感器，
// 记录它们在 sensordata 缓冲区的起始地址。
Imu::Imu(
  const mjModel * model, const std::string & orientation_sensor,
  const std::string & angular_velocity_sensor,
  const std::string & acceleration_sensor)
{
  // 校验模型指针非空
  if (model == nullptr) {
    throw std::invalid_argument("MuJoCo model must not be null");
  }

  // 定位姿态传感器：FRAMEQUAT（四元数），维度为 4
  orientation_address_ = requireSensor(
    model, orientation_sensor, mjSENS_FRAMEQUAT, 4);
  // 定位角速度传感器：GYRO（陀螺仪），维度为 3
  angular_velocity_address_ = requireSensor(
    model, angular_velocity_sensor, mjSENS_GYRO, 3);
  // 定位加速度传感器：ACCELEROMETER（加速度计），维度为 3
  acceleration_address_ = requireSensor(
    model, acceleration_sensor, mjSENS_ACCELEROMETER, 3);
}

// ---------- 读取一次 IMU 数据 ----------
// 从 MuJoCo 仿真数据中提取姿态、角速度、加速度和时间戳，
// 并对读取结果做有限性（finite）与模长校验，
// 校验失败时返回无效数据（valid == false）。
ImuData<float> Imu::read(const mjData * data) const noexcept
{
  ImuData<float> result;

  // 数据有效性检查：指针为空或时间戳非有限值则直接返回无效数据
  if (data == nullptr || data->sensordata == nullptr ||
    !std::isfinite(static_cast<float>(data->time)))
  {
    return result;
  }

  // 根据记录的地址偏移，指向各传感器数据在缓冲区中的位置
  const mjtNum * orientation = data->sensordata + orientation_address_;
  const mjtNum * angular_velocity = data->sensordata + angular_velocity_address_;
  const mjtNum * acceleration = data->sensordata + acceleration_address_;

  // 提取原始数据并填充到结果结构体
  // 姿态：世界系到机体系的四元数 (w, x, y, z)
  result.orientation_world_from_body = Eigen::Quaternionf(
    orientation[0], orientation[1], orientation[2], orientation[3]);
  // 角速度：机体坐标系下的 (x, y, z) 分量
  result.angular_velocity_body <<
    angular_velocity[0], angular_velocity[1], angular_velocity[2];
  // 加速度：机体坐标系下的 (x, y, z) 分量
  result.acceleration_body <<
    acceleration[0], acceleration[1], acceleration[2];
  // 时间戳：当前仿真时间
  result.timestamp = static_cast<float>(data->time);

  // 校验读取结果：所有数值必须有限，且四元数模长不能过小（防止除零/退化）
  const float quaternion_norm = result.orientation_world_from_body.norm();
  if (!result.orientation_world_from_body.coeffs().allFinite() ||
    !result.angular_velocity_body.allFinite() ||
    !result.acceleration_body.allFinite() ||
    quaternion_norm <= std::numeric_limits<float>::epsilon())
  {
    return ImuData<float>{};
  }

  // 归一化四元数以保证旋转的有效性，并标记数据有效
  result.orientation_world_from_body.normalize();
  result.valid = true;
  return result;
}

// ---------- 传感器查找与校验 ----------
// 按名称在 MuJoCo 模型中查找传感器，校验其类型、维度和数据地址，
// 全部通过后返回该传感器数据在 sensordata 缓冲区中的起始地址。
int Imu::requireSensor(
  const mjModel * model, const std::string & name,
  int expected_type, int expected_dimension)
{
  // 通过名称获取传感器 ID
  const int sensor_id = mj_name2id(model, mjOBJ_SENSOR, name.c_str());
  // 未找到该传感器则抛出异常
  if (sensor_id < 0) {
    throw std::invalid_argument("MuJoCo IMU sensor not found: " + name);
  }

  // 校验传感器的类型与维度是否与期望一致
  if (model->sensor_type[sensor_id] != expected_type ||
    model->sensor_dim[sensor_id] != expected_dimension)
  {
    throw std::invalid_argument(
            "MuJoCo IMU sensor has unexpected type or dimension: " + name);
  }

  // 获取数据起始地址并校验其范围是否合法
  const int address = model->sensor_adr[sensor_id];
  if (!sensorRangeIsValid(model, address, expected_dimension)) {
    throw std::invalid_argument("MuJoCo IMU sensor has an invalid data address: " + name);
  }
  return address;
}
