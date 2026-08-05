#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "sensor/sim_imu.hpp"

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

TEST(ImuTest, ReadsMujocoSensorData)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);

  Imu imu(model.get());
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

  const auto sample = imu.read(data.get());

  EXPECT_TRUE(sample.valid);
  EXPECT_DOUBLE_EQ(sample.orientation_world_from_body.w(), 1.0);
  EXPECT_TRUE(sample.angular_velocity_body.isApprox(Vec3<float>(1.0, 2.0, 3.0)));
  EXPECT_TRUE(sample.acceleration_body.isApprox(Vec3<float>(4.0, 5.0, 6.0)));
  EXPECT_DOUBLE_EQ(sample.timestamp, 1.25);
}

TEST(ImuTest, ReadsLiveSimulationSample)
{
  auto model = loadGo1Model();
  DataPointer data(mj_makeData(model.get()), &mj_deleteData);
  ASSERT_NE(data, nullptr);

  mj_forward(model.get(), data.get());
  const auto sample = Imu(model.get()).read(data.get());

  EXPECT_TRUE(sample.valid);
  EXPECT_TRUE(sample.orientation_world_from_body.coeffs().allFinite());
  EXPECT_TRUE(sample.angular_velocity_body.allFinite());
  EXPECT_TRUE(sample.acceleration_body.allFinite());
  EXPECT_DOUBLE_EQ(sample.timestamp, data->time);
}

TEST(ImuTest, RejectsMissingSensors)
{
  auto model = loadGo1Model();
  EXPECT_THROW(Imu(model.get(), "missing_orientation"), std::invalid_argument);
}
