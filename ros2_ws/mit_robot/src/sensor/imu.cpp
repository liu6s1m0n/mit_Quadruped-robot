// ============================================================
// IMU 数据源实现
// 包含仿真器数据源（SimImu）与真实硬件数据源（HardwareImu，占位），
// 以及按来源类型创建数据源的工厂函数 makeImu()。
// ============================================================

#include "sensor/imu.hpp"

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

// ---------- 仿真器 IMU 数据源（SimImu） ----------

// 构造函数：绑定模型与仿真数据，并解析定位三个 IMU 传感器。
SimImu::SimImu(
  const mjModel * model, const mjData * data,
  const std::string & orientation_sensor,
  const std::string & angular_velocity_sensor,
  const std::string & acceleration_sensor)
: data_(data)
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

// 读取一次 IMU 数据：从 MuJoCo 仿真数据中提取姿态、角速度、加速度和时间戳，
// 并对读取结果做有限性（finite）与模长校验，校验失败时返回无效数据。
// 每次读取的结果都会同步保存到成员 imu 中。
ImuData<float> SimImu::read()
{
  ImuData<float> result;

  // 数据有效性检查：数据指针为空或时间戳非有限值则直接返回无效数据
  if (data_ == nullptr || data_->sensordata == nullptr ||
    !std::isfinite(static_cast<float>(data_->time)))
  {
    imu = result;  // 保存无效数据，保持成员与返回值一致
    return result;
  }

  // 根据记录的地址偏移，指向各传感器数据在缓冲区中的位置
  const mjtNum * orientation = data_->sensordata + orientation_address_;
  const mjtNum * angular_velocity = data_->sensordata + angular_velocity_address_;
  const mjtNum * acceleration = data_->sensordata + acceleration_address_;

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
  result.timestamp = static_cast<float>(data_->time);

  // 校验读取结果：所有数值必须有限，且四元数模长不能过小（防止除零/退化）
  const float quaternion_norm = result.orientation_world_from_body.norm();
  if (!result.orientation_world_from_body.coeffs().allFinite() ||
    !result.angular_velocity_body.allFinite() ||
    !result.acceleration_body.allFinite() ||
    quaternion_norm <= std::numeric_limits<float>::epsilon())
  {
    result = ImuData<float>{};  // 校验失败，重置为无效数据
    imu = result;               // 保存到成员，保持与返回值一致
    return result;
  }

  // 归一化四元数以保证旋转的有效性，并标记数据有效
  result.orientation_world_from_body.normalize();
  result.valid = true;

  // 将本次读取的数据保存到成员 imu，供外部直接访问
  imu = result;
  return result;
}

// 传感器查找与校验：按名称在 MuJoCo 模型中查找传感器，
// 校验其类型、维度和数据地址，全部通过后返回 sensordata 缓冲区中的起始地址。
int SimImu::requireSensor(
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

// ---------- 真实硬件 IMU 数据源（HardwareImu，占位） ----------

ImuData<float> HardwareImu::read()
{
  // TODO(real-hardware)：接入真实 IMU（如 BMI088 / MPU6500）驱动后，
  // 从这里读取并返回真实数据。
  // 当前尚未实现，统一返回无效数据，避免调用方误用占位值。
  imu = ImuData<float>{};
  return imu;
}

// ---------- 工厂：按来源类型创建 IMU 数据源 ----------

std::unique_ptr<ImuSensor> makeImu(
  ImuSource source, const mjModel * model, const mjData * data)
{
  switch (source) {
    case ImuSource::SIMULATOR:
      return std::make_unique<SimImu>(model, data);
    case ImuSource::HARDWARE:
      return std::make_unique<HardwareImu>();
  }
  throw std::invalid_argument("unknown IMU source");
}
