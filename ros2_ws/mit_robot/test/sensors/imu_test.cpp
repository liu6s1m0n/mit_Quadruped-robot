#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "sensor/imu.hpp"

namespace
{

using ModelPointer = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPointer = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

ModelPointer loadGo1Model()
{
  char error[1024]{};
  ModelPointer model(
    mj_loadXML(MYMIT_ROBOT_TEST_SCENE_PATH, nullptr, error, sizeof(error)),
    &mj_deleteModel);
  if (!model) {
    throw std::runtime_error(error);
  }
  return model;
}

int sensorAddress(const mjModel * model, const char * name)
{
  return model->sensor_adr[mj_name2id(model, mjOBJ_SENSOR, name)];
}

}  // namespace

TEST(ImuTest, SimImuReadsMujocoSensorData)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);

  SimImu imu(model.get(), data.get());
  const int orientation = sensorAddress(model.get(), "imu_orientation");
  const int angular_velocity = sensorAddress(model.get(), "imu_angular_velocity");
  const int acceleration = sensorAddress(model.get(), "imu_linear_acceleration");

  data->sensordata[orientation + 0] = 2.0;
  data->sensordata[orientation + 1] = 0.0;
  data->sensordata[orientation + 2] = 0.0;
  data->sensordata[orientation + 3] = 0.0;
  data->sensordata[angular_velocity + 0] = 1.0;
  data->sensordata[angular_velocity + 1] = 2.0;
  data->sensordata[angular_velocity + 2] = 3.0;
  data->sensordata[acceleration + 0] = 4.0;
  data->sensordata[acceleration + 1] = 5.0;
  data->sensordata[acceleration + 2] = 6.0;
  data->time = 1.25;

  const auto sample = imu.read();

  EXPECT_TRUE(sample.valid);
  EXPECT_TRUE(sample.orientation_valid);
  EXPECT_DOUBLE_EQ(sample.orientation_world_from_body.w(), 1.0);
  EXPECT_TRUE(sample.angular_velocity_body.isApprox(Vec3<float>(1.0, 2.0, 3.0)));
  EXPECT_TRUE(sample.acceleration_body.isApprox(Vec3<float>(4.0, 5.0, 6.0)));
  EXPECT_DOUBLE_EQ(sample.timestamp, 1.25);

  // 读取结果应同时保存到成员 imu 中
  EXPECT_TRUE(imu.imu.valid);
  EXPECT_TRUE(imu.imu.angular_velocity_body.isApprox(Vec3<float>(1.0, 2.0, 3.0)));
}

TEST(ImuTest, SimImuReadsLiveSimulationSample)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);

  mj_forward(model.get(), data.get());
  const auto sample = SimImu(model.get(), data.get()).read();

  EXPECT_TRUE(sample.valid);
  EXPECT_TRUE(sample.orientation_world_from_body.coeffs().allFinite());
  EXPECT_TRUE(sample.angular_velocity_body.allFinite());
  EXPECT_TRUE(sample.acceleration_body.allFinite());
  EXPECT_DOUBLE_EQ(sample.timestamp, data->time);
}

TEST(ImuTest, SimImuRejectsMissingSensors)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  EXPECT_THROW(
    SimImu(model.get(), data.get(), "missing_orientation"),
    std::invalid_argument);
}

TEST(ImuTest, HardwareImuAcceptsRawGyroAndAccelerometerData)
{
  HardwareImu imu;
  EXPECT_FALSE(imu.read().valid);

  ImuData<float> input;
  input.orientation_valid = false;
  input.angular_velocity_body << 0.1F, 0.2F, 0.3F;
  input.acceleration_body << 0.0F, 0.0F, 9.81F;
  input.timestamp = 1.0F;
  input.valid = true;
  ASSERT_TRUE(imu.update(input));
  const auto sample = imu.read();

  EXPECT_TRUE(sample.valid);
  EXPECT_FALSE(sample.orientation_valid);
  EXPECT_TRUE(sample.angular_velocity_body.isApprox(input.angular_velocity_body));
  EXPECT_TRUE(sample.acceleration_body.isApprox(input.acceleration_body));
}

TEST(ImuTest, FactorySelectsBetweenSimulatorAndHardware)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);

  // 仿真器来源：能读到有效数据
  auto sim = makeImu(ImuSource::SIMULATOR, model.get(), data.get());
  ASSERT_NE(sim, nullptr);
  mj_forward(model.get(), data.get());
  EXPECT_TRUE(sim->read().valid);

  // 硬件来源：没有驱动注入数据前保持无效。
  auto hw = makeImu(ImuSource::HARDWARE, nullptr, nullptr);
  ASSERT_NE(hw, nullptr);
  EXPECT_FALSE(hw->read().valid);
}

TEST(ImuTest, HardwareImuConvertsDirectRpyWithoutIntegration)
{
  HardwareImu imu;
  HardwareImuMeasurement measurement;
  measurement.rpy_world_from_body << 0.1F, -0.2F, 0.3F;
  measurement.angular_velocity_body << 5.0F, 6.0F, 7.0F;
  measurement.angular_acceleration_body << 0.4F, 0.5F, 0.6F;
  measurement.timestamp = 2.0F;
  measurement.valid = true;

  ASSERT_TRUE(imu.update(measurement));
  const auto sample = imu.read();
  EXPECT_TRUE(sample.valid);
  EXPECT_TRUE(sample.orientation_valid);
  EXPECT_FALSE(sample.acceleration_valid);
  EXPECT_TRUE(sample.angular_acceleration_valid);
  EXPECT_TRUE(
    sample.angular_acceleration_body.isApprox(
      measurement.angular_acceleration_body));

  const Mat3<float> rotation = sample.orientation_world_from_body.toRotationMatrix();
  const float pitch = std::asin(std::clamp(-rotation(2, 0), -1.0F, 1.0F));
  EXPECT_NEAR(std::atan2(rotation(2, 1), rotation(2, 2)), 0.1F, 1e-5F);
  EXPECT_NEAR(pitch, -0.2F, 1e-5F);
  EXPECT_NEAR(std::atan2(rotation(1, 0), rotation(0, 0)), 0.3F, 1e-5F);
}
