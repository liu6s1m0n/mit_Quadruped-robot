#include <gtest/gtest.h>

#include "MPC/Gait.h"

namespace
{

TEST(MpcGait, BuildsTrotContactPredictionInProjectLegOrder)
{
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
