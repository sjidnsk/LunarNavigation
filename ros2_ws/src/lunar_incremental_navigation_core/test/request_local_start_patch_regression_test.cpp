#include <array>
#include <cmath>
#include <limits>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "lunar_incremental_navigation_core/request_local_start_patch.hpp"

namespace lunar::incremental_navigation {
namespace {

// Two missing centers from the captured stencil isolate geometry from terrain.
struct CapturedFixture {
  WheeledCapability capability;
  TraversabilityProfile profile;
  std::shared_ptr<const ElevationSnapshot> raw;
  std::shared_ptr<const FineTraversabilitySnapshot> fine;
};

CapturedFixture Captured() {
  CapturedFixture f;
  f.capability.footprint_xy_m = {{.591, .409}, {.591, -.409},
                                {-.591, -.409}, {-.591, .409}};
  f.capability.maximum_slope_rad = .3490658503988659;
  f.capability.maximum_local_obstacle_relief_m = .2;
  f.capability.minimum_underbody_clearance_m = .21;
  f.profile.planar_envelope_xy_m = f.capability.footprint_xy_m;
  f.profile.start_blind_zone_margin_m = .2;
  const GridGeometry geometry{.frame_id = "map", .width = 27, .height = 27,
                              .resolution_m = .2};
  std::vector<float> heights(27 * 27,
                            std::numeric_limits<float>::quiet_NaN());
  std::fill(heights.begin(), heights.end(), 0.0F);
  heights[11 * 27 + 16] = std::numeric_limits<float>::quiet_NaN();
  heights[10 * 27 + 17] = std::numeric_limits<float>::quiet_NaN();
  PersistentElevationMap map;
  const auto applied = map.Apply({
      .geometry = geometry, .elevation_m = heights,
      .map_from_source = {.parent_frame = "map", .child_frame = "map"}});
  if (applied.status != ElevationUpdateResult::Status::kApplied) {
    throw std::runtime_error("captured fixture was rejected");
  }
  f.raw = map.Snapshot();
  f.fine = FineTraversabilityBuilder{}.Derive(f.raw, f.capability, f.profile);
  return f;
}

TEST(StartSupportRegression, ShiftedUnknownPoseDoesNotReceiveAssumedSupport) {
  const auto f = Captured();
  const Pose2 anchor{.position_m = {2.697352191150726, 2.79620491463381},
                     .yaw_rad = 1.58025788508474};
  ASSERT_EQ(f.fine->State({13, 13}), FineCellState::kUnknown);
  const auto result = RequestLocalStartPatchBuilder{}.Build(
      f.fine, f.fine->geometry(), anchor, f.capability, f.profile);
  EXPECT_EQ(result.status, StartPatchResult::Status::kUnresolved);
  EXPECT_FALSE(result.view);
  EXPECT_EQ(result.assumed_cells, 0U);
  // 起点未知时不拟合高程，也不修改持久观测。
  EXPECT_EQ(f.fine->State({13, 13}), FineCellState::kUnknown);
  EXPECT_FALSE(f.raw->ElevationAt({17, 10}));
}

TEST(StartSupportRegression, FractionalMotionDoesNotFillUnknownSupport) {
  const auto f = Captured();
  for (const double dx : {-.09, -.03, 0., .03, .09}) {
    for (const double dy : {0., .03, .096, .099}) {
      SCOPED_TRACE(::testing::Message() << "offset " << dx << "," << dy);
      const Pose2 anchor{.position_m = {2.7 + dx, 2.7 + dy}};
      const auto result = RequestLocalStartPatchBuilder{}.Build(
          f.fine, f.fine->geometry(), anchor, f.capability, f.profile);
      EXPECT_EQ(result.status, StartPatchResult::Status::kUnresolved);
      EXPECT_FALSE(result.view);
      EXPECT_EQ(result.assumed_cells, 0U);
    }
  }
}

TEST(StartSupportRegression, MissingDerivativeHaloRemainsUnknown) {
  const auto f = Captured();
  const Pose2 anchor{.position_m = {2.697352191150726, 2.79620491463381}};
  const auto result = RequestLocalStartPatchBuilder{}.Build(
      f.fine, anchor, f.capability, f.profile);
  EXPECT_EQ(result.status, StartPatchResult::Status::kUnresolved);
  EXPECT_FALSE(result.view);
  EXPECT_EQ(result.assumed_cells, 0U);
  EXPECT_EQ(f.fine->State({17,10}), FineCellState::kUnknown);
  EXPECT_FALSE(f.raw->ElevationAt({17,10}));
}

}  // namespace
}  // namespace lunar::incremental_navigation
