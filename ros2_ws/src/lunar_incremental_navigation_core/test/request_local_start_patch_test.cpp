#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <ranges>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "lunar_incremental_navigation_core/elevation_map.hpp"
#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
#include "lunar_incremental_navigation_core/request_local_start_patch.hpp"
#include "lunar_incremental_navigation_core/wheel_local_planner.hpp"
#include "lunar_incremental_navigation_core/local_target_selector.hpp"
#include "lunar_incremental_navigation_core/types/platform_capability.hpp"

namespace lunar::incremental_navigation {
namespace {

[[nodiscard]] GridGeometry Geometry(const std::size_t width = 25U,
                                    const std::size_t height = 25U,
                                    const double resolution_m = 1.0) {
  return GridGeometry{.frame_id = "map",
                      .width = width,
                      .height = height,
                      .resolution_m = resolution_m};
}

[[nodiscard]] RigidTransform IdentityMapTransform() {
  return RigidTransform{.parent_frame = "map", .child_frame = "map"};
}

[[nodiscard]] std::size_t Offset(const GridGeometry& geometry,
                                 const GridIndex index) {
  return static_cast<std::size_t>(index.y) * geometry.width +
         static_cast<std::size_t>(index.x);
}

[[nodiscard]] Pose2 PoseAt(const GridGeometry& geometry,
                           const GridIndex index) {
  return Pose2{.position_m =
                   {.x = geometry.origin_m.x +
                             (static_cast<double>(index.x) + 0.5) *
                                 geometry.resolution_m,
                    .y = geometry.origin_m.y +
                             (static_cast<double>(index.y) + 0.5) *
                                 geometry.resolution_m}};
}

[[nodiscard]] WheeledCapability WheelCapability() {
  return WheeledCapability{
      .footprint_xy_m = {{-0.1, -0.1}, {0.1, -0.1},
                         {0.1, 0.1}, {-0.1, 0.1}},
      .body_extent_m = {.x = 0.2, .y = 0.2, .z = 0.3},
      .wheel_diameter_m = 0.3,
      .wheel_width_m = 0.08,
      .wheelbase_m = 0.4,
      .track_width_m = 0.3,
      .minimum_underbody_clearance_m = 0.5,
      .maximum_local_obstacle_relief_m = 0.2,
      .allow_unsupported_gap = false,
      .minimum_body_z_m = 0.0,
      .maximum_body_z_m = 1.0,
      .maximum_forward_speed_mps = 1.0,
      .maximum_reverse_speed_mps = 0.5,
      .maximum_spin_rate_radps = 1.0,
      .maximum_acceleration_mps2 = 1.0,
      .maximum_braking_deceleration_mps2 = 1.0,
      .maximum_yaw_acceleration_radps2 = 1.0,
      .maximum_lateral_acceleration_mps2 = 1.0,
      .maximum_curvature_per_m = 2.0,
      .maximum_slope_rad = std::numbers::pi / 2.0,
      .minimum_clearance_m = 0.2,
  };
}

[[nodiscard]] LeggedCapability LeggedCapabilityForTest() {
  return LeggedCapability{
      .body_extent_m = {.x = 0.2, .y = 0.2, .z = 0.3},
      .nominal_body_height_m = 0.5,
      .platform_mass_kg = 20.0,
      .nominal_payload_kg = 2.0,
      .maximum_payload_kg = 10.0,
      .maximum_slope_rad = std::numbers::pi / 2.0,
      .maximum_step_height_m = 0.6,
      .maximum_gap_width_m = 0.2,
      .minimum_body_clearance_m = 0.3,
      .step_vertical_rate_mps = 0.2,
      .body_height_m = {.lower = 0.4, .upper = 0.6},
      .forward_speed_mps = {.lower = -0.5, .upper = 0.8},
      .lateral_speed_mps = {.lower = -0.4, .upper = 0.4},
      .yaw_rate_radps = {.lower = -0.8, .upper = 0.8},
      .maximum_linear_acceleration_mps2 = 1.0,
      .maximum_yaw_acceleration_radps2 = 1.0,
      .unknown_is_traversable = false,
  };
}

[[nodiscard]] TraversabilityProfile Profile(
    const double margin_m = 2.0,
    const TraversalCostWeights weights = {},
    const double preferred_clearance_m = 0.0) {
  return TraversabilityProfile{
      .planar_envelope_xy_m = {{-0.1, -0.1}, {0.1, -0.1},
                               {0.1, 0.1}, {-0.1, 0.1}},
      .preferred_clearance_m = preferred_clearance_m,
      .slope_weight = weights.slope,
      .relief_weight = weights.relief,
      .clearance_weight = weights.clearance,
      .start_blind_zone_margin_m = margin_m,
  };
}

struct SnapshotFixture final {
  std::shared_ptr<const ElevationSnapshot> raw;
  std::shared_ptr<const FineTraversabilitySnapshot> fine;
};

[[nodiscard]] SnapshotFixture MakeFixture(
    const GridGeometry& geometry, const std::vector<float>& elevation,
    const PlatformCapability& capability,
    const TraversabilityProfile& profile) {
  PersistentElevationMap map;
  const ElevationUpdateResult update = map.Apply(ElevationEvidence{
      .geometry = geometry,
      .elevation_m = elevation,
      .map_from_source = IdentityMapTransform(),
  });
  if (update.status != ElevationUpdateResult::Status::kApplied) {
    throw std::runtime_error("test elevation fixture was rejected");
  }
  auto raw = map.Snapshot();
  FineTraversabilityBuilder fine_builder;
  auto fine = fine_builder.Derive(raw, capability, profile);
  return {.raw = std::move(raw), .fine = std::move(fine)};
}

[[nodiscard]] std::vector<float> FlatWithMissingSquare(
    const GridGeometry& geometry, const GridIndex center,
    const std::int64_t half_extent) {
  std::vector<float> elevation(geometry.CellCount(), 0.0F);
  for (std::int64_t dy = -half_extent; dy <= half_extent; ++dy) {
    for (std::int64_t dx = -half_extent; dx <= half_extent; ++dx) {
      elevation[Offset(geometry,
                       {.x = center.x + dx, .y = center.y + dy})] =
          std::numeric_limits<float>::quiet_NaN();
    }
  }
  return elevation;
}

TEST(RequestLocalStartPatch, UnknownStartNeverGetsFittedSupportOrConnections) {
  const auto geometry = Geometry();
  const GridIndex start{12, 12};
  for (const auto& capability : {PlatformCapability{WheelCapability()},
                                 PlatformCapability{LeggedCapabilityForTest()}}) {
    for (double margin : {0.0, 2.0, 100.0}) {
      const auto profile = Profile(margin);
      const auto fixture = MakeFixture(geometry,
          FlatWithMissingSquare(geometry, start, 1), capability, profile);
      ASSERT_EQ(fixture.fine->State(start), FineCellState::kUnknown);
      const auto revision = fixture.raw->raw_elevation_revision();
      const auto result = RequestLocalStartPatchBuilder{}.Build(
          fixture.fine, PoseAt(geometry, start), capability, profile);
      EXPECT_EQ(result.status, StartPatchResult::Status::kUnresolved);
      EXPECT_FALSE(result.view);
      EXPECT_EQ(result.assumed_cells, 0U);
      const auto connections = RequestLocalStartPatchBuilder{}.BuildStartConnections(
          fixture.fine, fixture.fine->geometry(), PoseAt(geometry, start), capability, profile);
      EXPECT_EQ(connections.status, StartPatchResult::Status::kUnresolved);
      EXPECT_TRUE(connections.connections.empty());
      const auto stationary = RequestLocalStartPatchBuilder{}.BuildStationary(
          fixture.fine, PoseAt(geometry, start), capability, profile);
      EXPECT_EQ(stationary.status, StartPatchResult::Status::kUnresolved);
      EXPECT_FALSE(stationary.view);
      EXPECT_EQ(stationary.assumed_cells, 0U);
      EXPECT_FALSE(fixture.raw->ElevationAt(start));
      EXPECT_EQ(fixture.fine->State(start), FineCellState::kUnknown);
      EXPECT_EQ(fixture.raw->raw_elevation_revision(), revision);
    }
  }
}

TEST(RequestLocalStartPatch, MeasuredCenterWithIncompleteSupportStaysUnknown) {
  const auto geometry = Geometry(25, 25, 0.2);
  const GridIndex start{12, 12};
  auto heights = FlatWithMissingSquare(geometry, start, 1);
  heights[Offset(geometry, start)] = 0.0F;
  const auto fixture = MakeFixture(geometry, heights, WheelCapability(), Profile());
  ASSERT_TRUE(fixture.raw->ElevationAt(start));
  ASSERT_EQ(fixture.fine->State(start), FineCellState::kUnknown);
  const auto result = RequestLocalStartPatchBuilder{}.BuildStationary(
      fixture.fine, PoseAt(geometry, start), WheelCapability(), Profile());
  EXPECT_EQ(result.status, StartPatchResult::Status::kUnresolved);
  EXPECT_FALSE(result.view);
  EXPECT_EQ(result.assumed_cells, 0U);
}

TEST(RequestLocalStartPatch, KnownFreeUsesMeasuredCostsAndContainingCellConnection) {
  const auto geometry = Geometry();
  const GridIndex start{12, 12};
  std::vector<float> heights(geometry.CellCount());
  for (std::size_t y=0; y<geometry.height; ++y)
    for (std::size_t x=0; x<geometry.width; ++x)
      heights[y*geometry.width+x] = 0.02F * static_cast<float>(x);
  const auto profile = Profile(2.0, {.slope=5.0, .relief=2.0});
  const auto fixture = MakeFixture(geometry, heights, WheelCapability(), profile);
  const auto pose = PoseAt(geometry, start);
  const auto result = RequestLocalStartPatchBuilder{}.Build(
      fixture.fine, pose, WheelCapability(), profile);
  ASSERT_EQ(result.status, StartPatchResult::Status::kNotNeeded);
  ASSERT_TRUE(result.view);
  EXPECT_TRUE(result.view->overrides().empty());
  EXPECT_EQ(result.assumed_cells, 0U);
  EXPECT_EQ(result.view->Source(start), LocalCellSource::kEvidenceFree);
  EXPECT_GT(result.view->TraversalCost(start), 0.0);
  EXPECT_DOUBLE_EQ(result.view->TraversalCost(start), fixture.fine->TraversalCost(start));
  const auto connections = RequestLocalStartPatchBuilder{}.BuildStartConnections(
      fixture.fine, fixture.fine->geometry(), pose, WheelCapability(), profile);
  ASSERT_EQ(connections.status, StartPatchResult::Status::kNotNeeded);
  ASSERT_EQ(connections.connections.size(), 1U);
  EXPECT_EQ(connections.connections.front().index, start);
  EXPECT_EQ(connections.connections.front().phase, StartPhase::kNormal);
  const auto plan = WheelLocalPlanner(WheelCapability()).Plan(*result.view, pose,
      LocalTarget{.center={16.5,12.5}, .position_tolerance_m=.01, .is_final_goal=true},
      SearchDeadline::max(), StopToken{});
  EXPECT_EQ(plan.status, LocalPlanResult::Status::kPlanFound);
  ASSERT_FALSE(plan.path.empty());
  for (const auto& point : plan.path)
    EXPECT_EQ(point.phase, StartPhase::kNormal);
}

TEST(RequestLocalStartPatch, KnownFreeStationarySupportRemainsAtActualAnchor) {
  const auto geometry = Geometry();
  const GridIndex start{12,12};
  const auto profile = Profile();
  const auto fixture = MakeFixture(geometry, std::vector<float>(geometry.CellCount(), 0.0F),
                                   WheelCapability(), profile);
  const auto pose = PoseAt(geometry, start);
  const auto result = RequestLocalStartPatchBuilder{}.BuildStationary(
      fixture.fine, pose, WheelCapability(), profile);
  ASSERT_EQ(result.status, StartPatchResult::Status::kReady);
  ASSERT_TRUE(result.view);
  EXPECT_TRUE(result.view->CanCertifyStationary(pose.position_m));
  EXPECT_FALSE(result.view->CanCertifyStationary({pose.position_m.x+.01,pose.position_m.y}));
  EXPECT_TRUE(result.view->overrides().empty());
  EXPECT_EQ(result.view->Source(start), LocalCellSource::kEvidenceFree);
  const FinalGoal goal{.target_x_m=pose.position_m.x, .target_y_m=pose.position_m.y,
                      .has_target_yaw=true, .target_yaw_rad=1.57};
  const auto selected = LocalTargetSelector{}.SelectRolling(*fixture.fine,
      fixture.fine->geometry(), pose.position_m, goal, std::nullopt);
  ASSERT_TRUE(selected);
  const auto plan = WheelLocalPlanner(WheelCapability()).Plan(*result.view, pose, *selected,
      SearchDeadline::max(), StopToken{});
  EXPECT_EQ(plan.status, LocalPlanResult::Status::kPlanFound);
  ASSERT_EQ(plan.path.size(), 1U);
  EXPECT_DOUBLE_EQ(plan.path.front().pose.position_m.x, pose.position_m.x);
  EXPECT_DOUBLE_EQ(plan.path.front().pose.position_m.y, pose.position_m.y);
  // 静止旋转快速路径沿用kStartPrefix标签；地图来源仍是上面验证的真实FREE。
  EXPECT_EQ(plan.path.front().phase, StartPhase::kStartPrefix);
}

TEST(RequestLocalStartPatch, KnownBlockedStartRemainsBlocked) {
  const auto geometry = Geometry();
  const GridIndex start{12,12};
  std::vector<float> heights(geometry.CellCount(), 0.0F);
  heights[Offset(geometry,start)] = 1.0F;
  const auto profile=Profile();
  const auto fixture=MakeFixture(geometry, heights, WheelCapability(), profile);
  ASSERT_EQ(fixture.fine->State(start), FineCellState::kBlocked);
  const auto pose=PoseAt(geometry,start);
  const auto patch=RequestLocalStartPatchBuilder{}.Build(fixture.fine,pose,WheelCapability(),profile);
  EXPECT_EQ(patch.status,StartPatchResult::Status::kStartBlocked);
  EXPECT_FALSE(patch.view);
  const auto stationary=RequestLocalStartPatchBuilder{}.BuildStationary(fixture.fine,pose,WheelCapability(),profile);
  EXPECT_EQ(stationary.status,StartPatchResult::Status::kStartBlocked);
  EXPECT_FALSE(stationary.view);
  EXPECT_EQ(fixture.fine->State(start), FineCellState::kBlocked);
}

TEST(RequestLocalStartPatch, ObsoleteBlindZoneMarginDoesNotAffectKnownFreeSupport) {
  const auto geometry = Geometry();
  auto profile = Profile();
  const auto fixture = MakeFixture(geometry, std::vector<float>(geometry.CellCount(), 0.0F),
                                   WheelCapability(), profile);
  const auto pose = PoseAt(geometry, {12,12});
  for (const double margin : {-1.0, 0.0, 100.0,
                              std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN()}) {
    profile.start_blind_zone_margin_m = margin;
    const auto result = RequestLocalStartPatchBuilder{}.Build(
        fixture.fine, pose, WheelCapability(), profile);
    ASSERT_EQ(result.status, StartPatchResult::Status::kNotNeeded);
    ASSERT_TRUE(result.view);
    EXPECT_TRUE(result.view->overrides().empty());
    EXPECT_DOUBLE_EQ(result.view->start_patch_radius_m(), fixture.fine->hard_inflation_radius_m());
    const auto stationary = RequestLocalStartPatchBuilder{}.BuildStationary(
        fixture.fine, pose, WheelCapability(), profile);
    EXPECT_EQ(stationary.status, StartPatchResult::Status::kReady);
    ASSERT_TRUE(stationary.view);
    EXPECT_TRUE(stationary.view->CanCertifyStationary(pose.position_m));
  }
  EXPECT_EQ(RequestLocalStartPatchBuilder{}.Build(fixture.fine,
      Pose2{.position_m={-1.,-1.}},WheelCapability(),profile).status,StartPatchResult::Status::kUnresolved);
  EXPECT_THROW((void)RequestLocalStartPatchBuilder{}.Build(nullptr,pose,WheelCapability(),profile),
               std::invalid_argument);
}

}  // namespace
}  // namespace lunar::incremental_navigation
