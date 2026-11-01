#include <gtest/gtest.h>
#include "lunar_navigation_rviz/boundary_selection.hpp"
using lunar_navigation_rviz::BoundarySelection;
TEST(BoundarySelection, RejectsWrongFrameAndNonFiniteCoordinates) {
  BoundarySelection region;
  EXPECT_FALSE(region.add("odom", 1, 2));
  EXPECT_FALSE(region.add("map", std::numeric_limits<double>::infinity(), 2));
  EXPECT_TRUE(region.points().empty());
}
TEST(BoundarySelection, RequiresSimpleNonDegeneratePolygon) {
  BoundarySelection region;
  EXPECT_FALSE(region.close());
  region.add("map", 0, 0); region.add("map", 1, 0); region.add("map", 2, 0);
  EXPECT_FALSE(region.close());
  region.clear();
  region.add("map", 0, 0); region.add("map", 2, 2);
  region.add("map", 0, 2); region.add("map", 2, 0);
  EXPECT_FALSE(region.close());
}
TEST(BoundarySelection, ClosesSquareAndEditingInvalidatesClosure) {
  BoundarySelection region;
  region.add("map", 0, 0); region.add("map", 2, 0);
  region.add("map", 2, 2); region.add("map", 0, 2);
  ASSERT_TRUE(region.close()); EXPECT_DOUBLE_EQ(region.area(), 4.0);
  EXPECT_TRUE(region.closed());
  EXPECT_FALSE(region.add("map", 8, 8));
  region.undo(); EXPECT_FALSE(region.closed()); EXPECT_EQ(region.points().size(), 3u);
  region.clear(); EXPECT_TRUE(region.points().empty()); EXPECT_FALSE(region.closed());
}
TEST(BoundarySelection, RejectsRepeatedVertices) {
  BoundarySelection region;
  EXPECT_TRUE(region.add("map", 0, 0));
  EXPECT_FALSE(region.add("map", 0, 0));
}
TEST(BoundarySelection, RejectsVerticesThatCollapseInPoint32WirePrecision) {
  BoundarySelection region;
  EXPECT_TRUE(region.add("map", 1e9, 1e9));
  EXPECT_FALSE(region.add("map", 1e9 + 1., 1e9));
  EXPECT_EQ(region.points().size(), 1u);
}
