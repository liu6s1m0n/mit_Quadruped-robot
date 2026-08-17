#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "controller/OrientationEstimator.hpp"

namespace
{

constexpr float kTolerance = 1e-5F;

class FakeImu : public ImuSensor
{
public:
  ImuData<float> sample;

  ImuData<float> read() override
  {
    imu = sample;
    return imu;
  }
};

class FakeLeg : public LegSensor
{
public:
  explicit FakeLeg(LegId leg_id)
  {
    sample.leg = leg_id;
  }

  JointState<float> read() override
  {
    leg = sample;
    return leg;
  }

  JointState<float> sample;
};

struct SensorFixture
{
  SensorFixture()
  : legs{
      FakeLeg(LegId::FR), FakeLeg(LegId::FL),
      FakeLeg(LegId::RR), FakeLeg(LegId::RL)}
  {
    imu.sample.orientation_world_from_body = Eigen::Quaternionf::Identity();
    imu.sample.orientation_valid = true;
    imu.sample.angular_velocity_body << 0.1F, 0.2F, 0.3F;
    imu.sample.acceleration_body << 1.0F, 2.0F, 3.0F;
    imu.sample.timestamp = 1.0F;
    imu.sample.valid = true;

    for (auto & leg : legs) {
      leg.sample.timestamp = 1.0F;
      leg.sample.valid = true;
    }
  }

  OrientationEstimator<float>::LegSensors legPointers()
  {
    OrientationEstimator<float>::LegSensors result{};
    for (std::size_t index = 0; index < kNumLegs; ++index) {
      result[index] = &legs[index];
    }
    return result;
  }

  FakeImu imu;
  std::array<FakeLeg, kNumLegs> legs;
};

}  // namespace

TEST(OrientationEstimatorTest, ReadsImuAndAllFourLegSensors)
{
  SensorFixture fixture;
  OrientationEstimator<float> estimator(
    fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::SIMULATION_TRUTH);

  ASSERT_TRUE(estimator.run());

  const auto & result = estimator.result();
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.orientation_world_from_body.isApprox(Eigen::Quaternionf::Identity()));
  EXPECT_TRUE(result.angular_velocity_body.isApprox(fixture.imu.sample.angular_velocity_body));
  EXPECT_TRUE(result.acceleration_body.isApprox(fixture.imu.sample.acceleration_body));
  EXPECT_TRUE(result.acceleration_world.isApprox(fixture.imu.sample.acceleration_body));
  EXPECT_TRUE(estimator.legsValid());

  for (std::size_t index = 0; index < kNumLegs; ++index) {
    EXPECT_EQ(estimator.jointStates()[index].leg, static_cast<LegId>(index));
  }
}

TEST(OrientationEstimatorTest, SimulationTruthUsesImuQuaternionDirectly)
{
  SensorFixture fixture;
  const Eigen::AngleAxisf yaw(0.7F, Vec3<float>::UnitZ());
  const Eigen::AngleAxisf pitch(-0.2F, Vec3<float>::UnitY());
  const Eigen::AngleAxisf roll(0.1F, Vec3<float>::UnitX());
  fixture.imu.sample.orientation_world_from_body = yaw * pitch * roll;
  OrientationEstimator<float> estimator(
    fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::SIMULATION_TRUTH);

  ASSERT_TRUE(estimator.run());

  EXPECT_NEAR(estimator.result().rpy.x(), 0.1F, kTolerance);
  EXPECT_NEAR(estimator.result().rpy.y(), -0.2F, kTolerance);
  EXPECT_NEAR(estimator.result().rpy.z(), 0.7F, kTolerance);
}

TEST(OrientationEstimatorTest, RotatesAccelerationFromBodyToWorld)
{
  SensorFixture fixture;
  fixture.imu.sample.orientation_world_from_body = Eigen::Quaternionf(
    Eigen::AngleAxisf(1.57079632679F, Vec3<float>::UnitZ()));
  fixture.imu.sample.acceleration_body = Vec3<float>::UnitX();
  OrientationEstimator<float> estimator(
    fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::SIMULATION_TRUTH);

  ASSERT_TRUE(estimator.run());

  EXPECT_TRUE(
    estimator.result().acceleration_world.isApprox(
      Vec3<float>::UnitY(), kTolerance));
}

TEST(OrientationEstimatorTest, RejectsInvalidOrUnsynchronizedLegData)
{
  SensorFixture fixture;
  OrientationEstimator<float> estimator(
    fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::IMU_FUSION, 0.01F);

  fixture.legs[2].sample.timestamp = 1.1F;
  EXPECT_FALSE(estimator.run());
  EXPECT_FALSE(estimator.result().valid);
  EXPECT_FALSE(estimator.legsValid());

  fixture.legs[2].sample.timestamp = 1.0F;
  fixture.legs[2].sample.valid = false;
  EXPECT_FALSE(estimator.run());
  EXPECT_FALSE(estimator.result().valid);
}

TEST(OrientationEstimatorTest, ImuFusionAddsDeviceOrientationCorrection)
{
  SensorFixture fixture;
  fixture.imu.sample.acceleration_body << 20.0F, 0.0F, 0.0F;
  fixture.imu.sample.angular_velocity_body << 0.0F, 0.0F, 1.0F;
  fixture.imu.sample.orientation_world_from_body = Eigen::Quaternionf(
    Eigen::AngleAxisf(0.2F, Vec3<float>::UnitZ()));
  OrientationEstimator<float> estimator(
    fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::IMU_FUSION, 0.02F, 0.0F, 2.0F);

  // 首帧直接使用 IMU 已解算的姿态，不再以 yaw=0 启动。
  ASSERT_TRUE(estimator.run());
  EXPECT_NEAR(estimator.result().rpy.z(), 0.2F, kTolerance);

  fixture.imu.sample.timestamp = 1.05F;
  fixture.imu.sample.orientation_world_from_body = Eigen::Quaternionf(
    Eigen::AngleAxisf(0.5F, Vec3<float>::UnitZ()));
  for (auto & leg : fixture.legs) {
    leg.sample.timestamp = 1.05F;
  }
  ASSERT_TRUE(estimator.run());
  // 角速度先预测到 0.25 rad，再以 2*0.05=0.1 的权重向 IMU 角度 0.5 rad 修正。
  EXPECT_NEAR(estimator.result().rpy.z(), 0.275F, 1e-4F);
  EXPECT_TRUE(
    estimator.result().angular_velocity_body.isApprox(
      fixture.imu.sample.angular_velocity_body));
}

TEST(OrientationEstimatorTest, CanSwitchFromTruthToImuFusionMode)
{
  SensorFixture fixture;
  fixture.imu.sample.acceleration_body << 0.0F, 0.0F, 9.81F;
  fixture.imu.sample.orientation_world_from_body = Eigen::Quaternionf(
    Eigen::AngleAxisf(0.8F, Vec3<float>::UnitZ()));
  OrientationEstimator<float> estimator(
    fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::SIMULATION_TRUTH);
  ASSERT_TRUE(estimator.run());
  EXPECT_NEAR(estimator.result().rpy.z(), 0.8F, kTolerance);

  estimator.setMode(OrientationEstimatorMode::IMU_FUSION);
  ASSERT_TRUE(estimator.run());
  EXPECT_NEAR(estimator.result().rpy.z(), 0.8F, kTolerance);
}

TEST(OrientationEstimatorTest, ImuFusionRetainsAccelerometerCorrection)
{
  SensorFixture fixture;
  fixture.imu.sample.angular_velocity_body.setZero();
  fixture.imu.sample.acceleration_body << 0.0F, 0.0F, 9.81F;
  OrientationEstimator<float> estimator(
    fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::IMU_FUSION, 0.02F, 5.0F, 0.0F);
  ASSERT_TRUE(estimator.run());

  constexpr float expected_roll = 0.2F;
  fixture.imu.sample.acceleration_body <<
    0.0F, 9.81F * std::sin(expected_roll), 9.81F * std::cos(expected_roll);
  for (int step = 1; step <= 30; ++step) {
    const float timestamp = 1.0F + 0.01F * static_cast<float>(step);
    fixture.imu.sample.timestamp = timestamp;
    for (auto & leg : fixture.legs) {
      leg.sample.timestamp = timestamp;
    }
    ASSERT_TRUE(estimator.run());
  }

  EXPECT_NEAR(estimator.result().rpy.x(), expected_roll, 0.05F);
  EXPECT_NEAR(estimator.result().rpy.y(), 0.0F, 1e-4F);
}

TEST(OrientationEstimatorTest, ImuFusionRejectsDegenerateDeviceOrientation)
{
  SensorFixture fixture;
  fixture.imu.sample.orientation_world_from_body.coeffs().setZero();
  OrientationEstimator<float> estimator(
    fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::IMU_FUSION);

  EXPECT_FALSE(estimator.run());
  EXPECT_FALSE(estimator.result().valid);
}

TEST(OrientationEstimatorTest, ImuFusionWorksWithoutAbsoluteOrientation)
{
  SensorFixture fixture;
  fixture.imu.sample.orientation_valid = false;
  fixture.imu.sample.angular_velocity_body.setZero();
  fixture.imu.sample.acceleration_body << 0.0F, 0.0F, 9.81F;
  OrientationEstimator<float> estimator(
    fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::IMU_FUSION, 0.02F, 2.0F, 10.0F);

  ASSERT_TRUE(estimator.run());
  EXPECT_NEAR(estimator.result().rpy.x(), 0.0F, kTolerance);
  EXPECT_NEAR(estimator.result().rpy.y(), 0.0F, kTolerance);
  EXPECT_NEAR(estimator.result().rpy.z(), 0.0F, kTolerance);

  fixture.imu.sample.angular_velocity_body << 0.0F, 0.0F, 1.0F;
  fixture.imu.sample.timestamp = 1.01F;
  for (auto & leg : fixture.legs) {
    leg.sample.timestamp = 1.01F;
  }
  ASSERT_TRUE(estimator.run());
  EXPECT_NEAR(estimator.result().rpy.z(), 0.01F, 1e-4F);
}

TEST(OrientationEstimatorTest, SimulationTruthRequiresAbsoluteOrientation)
{
  SensorFixture fixture;
  fixture.imu.sample.orientation_valid = false;
  OrientationEstimator<float> estimator(
    fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::SIMULATION_TRUTH);

  EXPECT_FALSE(estimator.run());
  EXPECT_FALSE(estimator.result().valid);
}

TEST(OrientationEstimatorTest, HardwareDirectUsesAnglesWithoutGyroIntegration)
{
  SensorFixture fixture;
  fixture.imu.sample.orientation_world_from_body = Eigen::Quaternionf(
    Eigen::AngleAxisf(0.4F, Vec3<float>::UnitZ()));
  fixture.imu.sample.angular_velocity_body << 0.0F, 0.0F, 50.0F;
  OrientationEstimator<float> estimator(
    fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::HARDWARE_DIRECT);

  ASSERT_TRUE(estimator.run());
  EXPECT_NEAR(estimator.result().rpy.z(), 0.4F, 1e-5F);

  fixture.imu.sample.timestamp = 1.05F;
  fixture.imu.sample.orientation_world_from_body = Eigen::Quaternionf(
    Eigen::AngleAxisf(0.6F, Vec3<float>::UnitZ()));
  for (auto & leg : fixture.legs) {
    leg.sample.timestamp = 1.05F;
  }
  ASSERT_TRUE(estimator.run());
  // 即使角速度极大，输出也严格采用设备本帧角度 0.6，而不是积分到 2.9 rad。
  EXPECT_NEAR(estimator.result().rpy.z(), 0.6F, 1e-5F);
}
