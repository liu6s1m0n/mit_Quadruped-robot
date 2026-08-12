#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "controller/PositionVelocityEstimator.hpp"

// 用可控的假 IMU/腿传感器验证状态估计数据流、接触融合和传感器异常处理。
namespace
{

class FakeImu : public ImuSensor
{
public:
  ImuData<float> read() override
  {
    ++read_count;
    imu = sample;
    return imu;
  }

  ImuData<float> sample;
  int read_count = 0;
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
    ++read_count;
    leg = sample;
    return leg;
  }

  JointState<float> sample;
  int read_count = 0;
};

struct SensorFixture
{
  SensorFixture()
  : quadruped(makeQuadruped<float>(RobotType::UNITREE_GO1)),
    legs{
      FakeLeg(LegId::FR), FakeLeg(LegId::FL),
      FakeLeg(LegId::RR), FakeLeg(LegId::RL)}
  {
    imu.sample.orientation_world_from_body = Eigen::Quaternionf::Identity();
    imu.sample.angular_velocity_body.setZero();
    imu.sample.acceleration_body << 0.0F, 0.0F, 9.81F;
    imu.sample.timestamp = 1.0F;
    imu.sample.valid = true;

    for (std::size_t index = 0; index < kNumLegs; ++index) {
      const LegId leg_id = static_cast<LegId>(index);
      legs[index].sample.leg = leg_id;
      legs[index].sample.position = quadruped.leg(leg_id).joints.home_position;
      legs[index].sample.velocity.setZero();
      legs[index].sample.torque_estimate.setZero();
      legs[index].sample.timestamp = 1.0F;
      legs[index].sample.valid = true;
    }
  }

  PositionVelocityEstimator<float>::LegSensors legPointers()
  {
    PositionVelocityEstimator<float>::LegSensors result{};
    for (std::size_t index = 0; index < kNumLegs; ++index) {
      result[index] = &legs[index];
    }
    return result;
  }

  void advance(float time_step)
  {
    imu.sample.timestamp += time_step;
    for (auto & leg : legs) {
      leg.sample.timestamp += time_step;
    }
  }

  Quadruped<float> quadruped;
  FakeImu imu;
  std::array<FakeLeg, kNumLegs> legs;
};

}  // namespace

TEST(PositionVelocityEstimatorTest, InitializesStandingHeightFromLegKinematics)
{
  SensorFixture fixture;
  PositionVelocityEstimator<float> estimator(
    fixture.quadruped, fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::SIMULATION_TRUTH);

  ASSERT_TRUE(estimator.run());

  EXPECT_TRUE(estimator.result().valid);
  EXPECT_NEAR(
    estimator.result().position_world.z(),
    fixture.quadruped.nominalBodyHeight() +
    fixture.quadruped.leg(LegId::FR).foot_radius, 0.02F);
  EXPECT_TRUE(estimator.result().velocity_world.isZero(1e-5F));
  EXPECT_EQ(fixture.imu.read_count, 1);
  for (const auto & leg : fixture.legs) {
    EXPECT_EQ(leg.read_count, 1);
  }
}

TEST(PositionVelocityEstimatorTest, KeepsStaticStandingRobotStable)
{
  SensorFixture fixture;
  PositionVelocityEstimator<float> estimator(
    fixture.quadruped, fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::SIMULATION_TRUTH);
  ASSERT_TRUE(estimator.run());
  const float initial_height = estimator.result().position_world.z();

  for (int step = 0; step < 100; ++step) {
    fixture.advance(0.002F);
    ASSERT_TRUE(estimator.run());
  }

  EXPECT_NEAR(estimator.result().position_world.z(), initial_height, 0.01F);
  EXPECT_TRUE(estimator.result().velocity_world.isZero(0.02F));
}

TEST(PositionVelocityEstimatorTest, EstimatesForwardVelocityFromFixedSupportFeet)
{
  SensorFixture fixture;
  PositionVelocityEstimator<float> estimator(
    fixture.quadruped, fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::SIMULATION_TRUTH);
  ASSERT_TRUE(estimator.run());

  // 在名义姿态下，thigh 正向转动使足端沿机身 -x 运动。若足端支撑固定，
  // 机身速度必须估计为 +x；旧公式的 x 符号相反，会得到负速度和负位置。
  for (auto & leg : fixture.legs) {
    leg.sample.velocity << 0.0F, 1.0F, 0.0F;
  }
  for (int step = 0; step < 10; ++step) {
    fixture.advance(0.002F);
    ASSERT_TRUE(estimator.run());
  }

  EXPECT_GT(estimator.result().velocity_world.x(), 0.1F);
  EXPECT_GT(estimator.result().position_world.x(), 0.0F);
  EXPECT_NEAR(estimator.result().velocity_world.y(), 0.0F, 0.02F);
}

TEST(PositionVelocityEstimatorTest, RejectsInvalidLegFrame)
{
  SensorFixture fixture;
  PositionVelocityEstimator<float> estimator(
    fixture.quadruped, fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::IMU_FUSION);
  fixture.legs[1].sample.valid = false;

  EXPECT_FALSE(estimator.run());
  EXPECT_FALSE(estimator.result().valid);
}

TEST(PositionVelocityEstimatorTest, UsesDirectImuAngleThroughOrientationEstimator)
{
  SensorFixture fixture;
  fixture.imu.sample.orientation_world_from_body = Eigen::Quaternionf(
    Eigen::AngleAxisf(0.6F, Vec3<float>::UnitZ()));
  PositionVelocityEstimator<float> estimator(
    fixture.quadruped, fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::IMU_FUSION);

  ASSERT_TRUE(estimator.run());
  EXPECT_NEAR(estimator.result().rpy.z(), 0.6F, 1e-5F);
}

TEST(PositionVelocityEstimatorTest, AcceptsAndClampsContactProbabilities)
{
  SensorFixture fixture;
  PositionVelocityEstimator<float> estimator(
    fixture.quadruped, fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::SIMULATION_TRUTH);

  PositionVelocityEstimator<float>::ContactProbabilities probabilities{
    1.0F, 0.0F, 1.2F, -0.2F};
  estimator.setContactProbabilities(probabilities);
  ASSERT_TRUE(estimator.run());
  fixture.advance(0.002F);
  EXPECT_TRUE(estimator.run());
}

TEST(PositionVelocityEstimatorTest, SwingLegMotionDoesNotBecomeBodyMotion)
{
  SensorFixture fixture;
  PositionVelocityEstimator<float> estimator(
    fixture.quadruped, fixture.imu, fixture.legPointers(),
    OrientationEstimatorMode::SIMULATION_TRUTH);
  estimator.setContactProbabilities({1.0F, 0.0F, 1.0F, 0.0F});
  ASSERT_TRUE(estimator.run());

  // 只运动两条计划摆动腿；支撑腿与 IMU 保持静止，机身估计不应跟着摆腿漂移。
  for (int step = 0; step < 100; ++step) {
    const float phase = static_cast<float>(step + 1) / 100.0F;
    fixture.legs[1].sample.position.y() += 0.002F * std::sin(phase * 3.1415926F);
    fixture.legs[3].sample.position.y() -= 0.002F * std::sin(phase * 3.1415926F);
    fixture.legs[1].sample.velocity.y() = 0.8F * std::cos(phase * 3.1415926F);
    fixture.legs[3].sample.velocity.y() = -0.8F * std::cos(phase * 3.1415926F);
    fixture.advance(0.002F);
    ASSERT_TRUE(estimator.run());
  }

  EXPECT_LT(estimator.result().velocity_world.norm(), 0.03F);
  EXPECT_LT(estimator.result().position_world.head<2>().norm(), 0.01F);
}
