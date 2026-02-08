#include <gtest/gtest.h>
#include "fleetops_health/timeout.hpp"

TEST(TimeoutLogic, BoundaryCases)
{
  EXPECT_FALSE(is_timed_out(1.9, 2.0));
  EXPECT_FALSE(is_timed_out(2.0, 2.0));
  EXPECT_TRUE(is_timed_out(2.01, 2.0));
}
