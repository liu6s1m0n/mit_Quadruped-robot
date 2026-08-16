#include <gtest/gtest.h>

// 验证摆动腿目标在摆动期间保持锁存，并在触地后切换回支撑状态。

#include "FSM/FSM_State_Locomotion.h"
#include "controller/FootSwingTrajectory.hpp"
#include "fsm_test_support.hpp"

namespace
{

TEST(LocomotionStateTest, LatchesSwingEndpointsUntilTouchdown)
{
  // 使用较大的测试步长，便于在少量周期内观察摆动和落脚阶段。
  test_support::FsmContext context(0.05F);
  ASSERT_TRUE(context.initialized);
  context.desired.mode = ControlMode::Locomotion;
  auto data = context.data();
  ASSERT_TRUE(data.valid());

  FSM_State_Locomotion<float> locomotion(&data);
  locomotion.setForwardVelocity(0.3F);
  locomotion.onEnter();
  locomotion.run();
  locomotion.run();

  constexpr std::size_t kRearRight = static_cast<std::size_t>(LegId::RR);
  ASSERT_TRUE(locomotion.swingLegActive(LegId::RR));
  const Vec3<float> initial = context.estimate.position_world +
    context.quadruped.hipLocation(LegId::RR) + context.leg_controller.datas[kRearRight].p;
  EXPECT_TRUE(locomotion.footPositionTargetWorld(LegId::RR).isApprox(initial, 1.0e-5F));

  context.leg_controller.datas[kRearRight].p.z() += 0.04F;
  locomotion.run();

  // 根据当前腿部反馈复现期望落点，检查控制器生成的 Bezier 轨迹。

  FootSwingTrajectory<float> expected;
  expected.setInitialPosition(initial);
  Vec3<float> landing = initial;
  landing.x() = context.quadruped.hipLocation(LegId::RR).x() - 0.006F;
  Vec3<float> nominal_foot_from_hip = Vec3<float>::Zero();
  computeLegJacobianAndPosition(
    context.quadruped, context.quadruped.leg(LegId::RR).joints.home_position,
    static_cast<Mat3<float> *>(nullptr), &nominal_foot_from_hip, LegId::RR);
  landing.y() = context.quadruped.hipLocation(LegId::RR).y() + nominal_foot_from_hip.y();
  expected.setFinalPosition(landing);
  expected.setHeight(0.10F);
  expected.computeSwingTrajectoryBezier(0.25F, 0.2F);
  EXPECT_TRUE(
    locomotion.footPositionTargetWorld(LegId::RR).isApprox(expected.getPosition(), 1.0e-5F));
  EXPECT_GT(locomotion.footPositionTargetWorld(LegId::RR).z(), initial.z());

  for (std::size_t iteration = 0; iteration < 4; ++iteration) {
    locomotion.run();
  }
  EXPECT_FALSE(locomotion.swingLegActive(LegId::RR));
}

}  // namespace
