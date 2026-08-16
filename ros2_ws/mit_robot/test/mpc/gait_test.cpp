#include <gtest/gtest.h>

// 验证 MPC 步态表按工程约定的 FR、FL、RR、RL 顺序生成小跑接触预测。

#include "MPC/Gait.h"

namespace
{

TEST(MpcGait, BuildsTrotContactPredictionInProjectLegOrder)
{
  // 10 个预测段、半周期相位差和 5 段支撑共同构成标准 TROT 接触表。
  mpc::OffsetDurationGait gait(10, {0, 5, 5, 0}, {5, 5, 5, 5}, "trot");
  gait.advance(0, 10);
  const auto & table = gait.contactTable();
  ASSERT_EQ(table.size(), 40U);
  EXPECT_EQ(table[0], 1);
  EXPECT_EQ(table[1], 0);
  EXPECT_EQ(table[2], 0);
  EXPECT_EQ(table[3], 1);
  EXPECT_EQ(table[5 * kNumLegs], 0);
  EXPECT_EQ(table[5 * kNumLegs + 1], 1);
}

}  // namespace
