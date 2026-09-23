#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include "lunar_incremental_navigation_core/rolling_update_policy.hpp"
using lunar::incremental_navigation::RollingUpdatePolicy;

TEST(RollingUpdatePolicy, EarlyAttemptDoesNotBusyLoop) {
  RollingUpdatePolicy p(1.0, 2.0);
  p.Started(0.0, 1); p.Finished();
  EXPECT_TRUE(p.ShouldStart(.2, 2, 1.0));
  p.Started(.2, 2);
  EXPECT_FALSE(p.ShouldStart(.25, 2, 1.0));
  p.Finished();
  EXPECT_FALSE(p.ShouldStart(.3, 2, 1.0));
  EXPECT_TRUE(p.ShouldStart(1.3, 2, 1.0));
}
TEST(RollingUpdatePolicy, PeriodFreezeDisableAndReset) {
  RollingUpdatePolicy p(1., 2.);
  p.Started(0., 1); p.Finished();
  EXPECT_FALSE(p.ShouldStart(.9, 1, 10.));
  EXPECT_TRUE(p.ShouldStart(1., 1, 10.));
  p.SetFrozen(true);
  EXPECT_FALSE(p.ShouldStart(2., 2, 1.));
  p.Reset();
  EXPECT_TRUE(p.ShouldStart(2., 2, 1.));
  RollingUpdatePolicy disabled(0., 2.);
  EXPECT_FALSE(disabled.ShouldStart(100., 2, 1.));
}
TEST(RollingUpdatePolicy, RejectsInvalidConfiguration) {
  EXPECT_THROW(RollingUpdatePolicy(-1., 2.), std::invalid_argument);
  EXPECT_THROW(RollingUpdatePolicy(1., -2.), std::invalid_argument);
  EXPECT_THROW(RollingUpdatePolicy(std::numeric_limits<double>::infinity(), 2.), std::invalid_argument);
  EXPECT_THROW(RollingUpdatePolicy(1., std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(RollingUpdatePolicy, NewlyPublishedShortPathWaitsForPeriod) {
  RollingUpdatePolicy p(.5, 1.);
  p.Started(0.,1);p.Finished();
  p.Published(2,.8);
  EXPECT_FALSE(p.ShouldStart(.1,2,.7));
  EXPECT_TRUE(p.ShouldStart(.5,2,.6));
  p.Started(.5,2);p.Finished();
  p.Published(3,4.);
  EXPECT_TRUE(p.ShouldStart(.6,3,.9));
}
