#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "lunar_incremental_navigation_ros/exploration_map.hpp"
#include "lunar_incremental_navigation_ros/platform_config.hpp"

namespace lunar::incremental_navigation_ros {
namespace core = lunar::incremental_navigation;
namespace {
OwnedElevationEvidence Flat(double resolution, double x = 0., double y = 0., double size = 2.) {
  const auto n = static_cast<std::size_t>(std::round(size / resolution));
  return {.geometry={.frame_id="odom", .width=n, .height=n,
      .resolution_m=resolution, .origin_m={x,y,0}},
      .elevation_m=std::vector<float>(n*n,0.F),
      .map_from_source={.parent_frame="map",.child_frame="odom"}};
}
core::WheeledCapability Wheel() {
  return {.footprint_xy_m={{-.05,-.05},{.05,-.05},{.05,.05},{-.05,.05}},
    .body_extent_m={.1,.1,.1}, .minimum_underbody_clearance_m=.1,
    .maximum_local_obstacle_relief_m=.2, .maximum_slope_rad=.5,
    .minimum_clearance_m=.0};
}
core::TraversabilityProfile Profile() {
  return {.planar_envelope_xy_m=Wheel().footprint_xy_m,
    .slope_weight=1.,.relief_weight=1.,.goal_position_tolerance_m=.1,
    .goal_yaw_tolerance_rad=.05};
}
}
TEST(ExplorationMap, ResolutionHotChangeAndRollingHistory) {
  ExplorationMap map(Wheel(),Profile(),.2,1.);
  for (double resolution : {.01,.05,.2,.07}) {
    auto source=Flat(resolution,0.,0.,resolution==.07?2.1:2.);
    ASSERT_TRUE(map.Apply(source,10));
    const auto captured=map.Capture();
    ASSERT_NE(captured.fine,nullptr);
    EXPECT_DOUBLE_EQ(captured.fine->geometry().resolution_m(),.2);
    EXPECT_EQ(captured.fine->State({4,4}),core::FineCellState::kFree);
  }
  ASSERT_TRUE(map.Apply(Flat(.05,4.),20));
  auto captured=map.Capture();
  EXPECT_EQ(captured.fine->State({4,4}),core::FineCellState::kFree);
  EXPECT_EQ(captured.fine->State({24,4}),core::FineCellState::kFree);
  EXPECT_EQ(captured.fine->State({15,4}),core::FineCellState::kUnknown);
  const auto revision=captured.fine->fine_traversability_revision();
  ASSERT_TRUE(map.Apply(Flat(.05,4.),30));
  EXPECT_EQ(map.Capture().processed_stamp_ns,30);
  const auto delta=map.Capture(revision);
  EXPECT_FALSE(delta.full_snapshot);
  EXPECT_TRUE(delta.dirty_tiles.empty());
  EXPECT_TRUE(map.Capture(99999).full_snapshot);
}
TEST(ExplorationMap, PartialCoverageUnknownAndNanHistory) {
  ExplorationMap map(Wheel(),Profile(),.2,1.);
  auto partial=Flat(.01,0.,0.,.1);
  ASSERT_TRUE(map.Apply(partial,1));
  EXPECT_NE(map.Capture().fine->State({0,0}),core::FineCellState::kFree);
  ASSERT_TRUE(map.Apply(Flat(.05),2));
  const auto known=map.Capture().fine->State({4,4});
  EXPECT_EQ(known,core::FineCellState::kFree);
  auto holes=Flat(.05);
  std::fill(holes.elevation_m.begin(),holes.elevation_m.end(),std::numeric_limits<float>::quiet_NaN());
  ASSERT_TRUE(map.Apply(holes,3));
  EXPECT_EQ(map.Capture().fine->State({4,4}),known);
  // 有限但不完整的本征证据覆盖旧 FREE；NaN 仍保留历史。
  holes.elevation_m[18*40+18]=0.F;
  ASSERT_TRUE(map.Apply(holes,4));
  EXPECT_NE(map.Capture().fine->State({4,4}),core::FineCellState::kFree);
}
TEST(ExplorationMap, ThinSteepFeatureSurvivesConservativeProjection) {
  ExplorationMap map(Wheel(),Profile(),.2,1.);
  auto source=Flat(.01);
  source.elevation_m[100*200+100]=.04F;
  ASSERT_TRUE(map.Apply(source,1));
  const auto result=map.Capture();
  EXPECT_EQ(result.fine->State({5,5}),core::FineCellState::kBlocked);
  EXPECT_NE(map.Project().data[0],0);
}
TEST(ExplorationMap, RotatedTranslatedSourceKeepsActualWorldGeometry) {
  ExplorationMap map(Wheel(),Profile(),.2,1.);
  auto source=Flat(.05);
  const double yaw=.37;
  source.map_from_source.rotation={.w=std::cos(yaw/2),.z=std::sin(yaw/2)};
  source.map_from_source.translation_m={3.,-2.,0.};
  ASSERT_TRUE(map.Apply(source,1));
  const auto fine=map.Capture().fine;
  const auto x=static_cast<std::int64_t>(std::floor((3.+std::cos(yaw)-std::sin(yaw))/.2));
  const auto y=static_cast<std::int64_t>(std::floor((-2.+std::sin(yaw)+std::cos(yaw))/.2));
  EXPECT_EQ(fine->State({x,y}),core::FineCellState::kFree);
  EXPECT_EQ(fine->State({4,4}),core::FineCellState::kUnknown);
}
}  // namespace lunar::incremental_navigation_ros
