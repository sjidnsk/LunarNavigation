#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "lunar_incremental_navigation_ros/policy_map_exporter.hpp"

#include "lunar_incremental_navigation_core/elevation_map.hpp"
#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"

namespace lunar::incremental_navigation_ros {

TEST(PolicyMapExporter, ExportsOnlyMeasuredIntrinsicClassificationAsObserved) {
  EXPECT_EQ(EffectiveObservedState(
                lunar::incremental_navigation::IntrinsicCellState::kFree,
                0.0F),
            kPolicyMapFree);
  EXPECT_EQ(EffectiveObservedState(
                lunar::incremental_navigation::IntrinsicCellState::kBlocked,
                0.0F),
            kPolicyMapBlocked);
  EXPECT_EQ(EffectiveObservedState(
                lunar::incremental_navigation::IntrinsicCellState::kFree,
                std::numeric_limits<float>::quiet_NaN()),
            kPolicyMapUnknown);
  EXPECT_EQ(EffectiveObservedState(
                lunar::incremental_navigation::IntrinsicCellState::kUnknown,
                0.0F),
            kPolicyMapUnknown);
}

TEST(PolicyMapExporter, UsesFullSnapshotForBootstrapAndMapRevisionChanges) {
  using namespace lunar::incremental_navigation;
  const WheeledCapability wheel{.footprint_xy_m = {{-.05, -.05}, {.05, -.05}, {.05, .05}, {-.05, .05}},
      .body_extent_m = {.x=.1,.y=.1,.z=.1}, .wheel_diameter_m=.2, .wheel_width_m=.05,
      .wheelbase_m=.1, .track_width_m=.1, .minimum_underbody_clearance_m=.1,
      .maximum_local_obstacle_relief_m=.2, .maximum_forward_speed_mps=.2,
      .maximum_reverse_speed_mps=.2, .maximum_spin_rate_radps=1., .maximum_acceleration_mps2=.5,
      .maximum_braking_deceleration_mps2=.5, .maximum_yaw_acceleration_radps2=.5,
      .maximum_lateral_acceleration_mps2=.5, .maximum_curvature_per_m=1.,
      .maximum_slope_rad=.5, .minimum_clearance_m=.0};
  const TraversabilityProfile profile{.planar_envelope_xy_m = wheel.footprint_xy_m,
      .slope_weight=1., .relief_weight=1.};
  PersistentElevationMap elevation;
  const GridGeometry geometry{.frame_id="map", .width=7, .height=7, .resolution_m=.2};
  const std::vector<float> heights(49, 0.F);
  ASSERT_EQ(elevation.Apply(ElevationEvidence{.geometry=geometry, .elevation_m=heights,
      .map_from_source={.parent_frame="map", .child_frame="map"}}).status,
      ElevationUpdateResult::Status::kApplied);
  const auto fine = FineTraversabilityBuilder{}.Derive(elevation.Snapshot(), wheel, profile);
  PolicyMapExporter exporter(wheel, profile);
  const PolicyMapExportInput input{.fine=fine, .epoch="e", .processed_stamp_ns=10};
  const auto bootstrap = exporter.Export(input, 0);
  EXPECT_TRUE(bootstrap.full_snapshot);
  EXPECT_FALSE(bootstrap.tiles.empty());
  const PolicyMapExportInput refresh_input{.fine=fine, .epoch="e", .processed_stamp_ns=11,
                                            .base_revision=fine->fine_traversability_revision(),
                                            .full_snapshot=false};
  const auto refresh = exporter.Export(refresh_input, fine->fine_traversability_revision());
  EXPECT_FALSE(refresh.full_snapshot);
  EXPECT_TRUE(refresh.tiles.empty());
  // 起点未知时，两个导出入口均保留未知，不推断起点支持或连接。
  auto states = fine->FindTile({})->states();
  const GridIndex start{3,3};
  states[TileCellOffset(start)] = FineCellState::kUnknown;
  auto directory = fine->tile_directory().WithTile({},
      std::make_shared<const FineTraversabilityTile>(states, fine->FindTile({})->traversal_costs()));
  auto blind = std::make_shared<const FineTraversabilitySnapshot>(fine->geometry(),
      fine->raw_elevation_revision(), 2U, fine->platform_profile_hash(),
      fine->hard_inflation_radius_m(), fine->preferred_clearance_m(), fine->cost_weights(),
      fine->elevation(), std::move(directory), std::vector<TileIndex>{},
      std::vector<TileIndex>{}, FineSnapshotMetrics{});
  auto with_patch = exporter.Export({.fine=blind,
      .anchor=Pose2{.position_m={.7,.7}}, .local_window_size_m=32., .epoch="e", .processed_stamp_ns=12},0);
  EXPECT_TRUE(with_patch.assumed_start_x.empty());
  EXPECT_TRUE(with_patch.assumed_start_y.empty());
  EXPECT_FALSE(with_patch.stationary_support_ready);
  EXPECT_TRUE(with_patch.start_connection_x.empty());
  EXPECT_EQ(with_patch.start_connection_status,
      static_cast<std::uint8_t>(StartPatchResult::Status::kUnresolved));
  const auto observed = exporter.ExportObserved({.fine=blind,
      .anchor=Pose2{.position_m={.7,.7}}, .local_window_size_m=32., .epoch="e", .processed_stamp_ns=12}, 0);
  EXPECT_TRUE(observed.assumed_start_x.empty());
  EXPECT_FALSE(observed.stationary_support_ready);
  EXPECT_TRUE(observed.start_connection_x.empty());
  EXPECT_EQ(observed.reason_code, "INPUT_UNAVAILABLE");
  const auto direct = exporter.ExportObserved({.fine=fine,
      .anchor=Pose2{.position_m={.7,.7}}, .local_window_size_m=32., .epoch="e", .processed_stamp_ns=12}, 0);
  EXPECT_TRUE(direct.ready);
  EXPECT_TRUE(direct.stationary_support_ready);
  EXPECT_FALSE(direct.start_connection_x.empty());
  EXPECT_TRUE(direct.assumed_start_x.empty());
  const auto legacy_direct = exporter.Export({.fine=fine,
      .anchor=Pose2{.position_m={.7,.7}}, .local_window_size_m=32., .epoch="e", .processed_stamp_ns=12}, 0);
  EXPECT_TRUE(legacy_direct.stationary_support_ready);
  ASSERT_EQ(legacy_direct.start_connection_x.size(), 1U);
  ASSERT_EQ(legacy_direct.start_connection_y.size(), 1U);
  EXPECT_EQ(legacy_direct.start_connection_x.front(), start.x);
  EXPECT_EQ(legacy_direct.start_connection_y.front(), start.y);
  EXPECT_TRUE(legacy_direct.assumed_start_x.empty());
  const auto bad_delta = exporter.ExportObserved({.fine=fine, .epoch="e",
      .base_revision=999, .full_snapshot=false}, 999);
  EXPECT_TRUE(bad_delta.full_snapshot);
  EXPECT_EQ(bad_delta.base_revision, 0U);

  // 真实高程有限但细地图仍UNKNOWN时，也不能绕过已知支撑。
  auto unknown_states = fine->FindTile({})->states();
  std::fill(unknown_states.begin(), unknown_states.end(), FineCellState::kUnknown);
  const auto make_snapshot = [&](auto cell_states) {
    auto tiles = fine->tile_directory().WithTile({},
        std::make_shared<const FineTraversabilityTile>(cell_states, fine->FindTile({})->traversal_costs()));
    return std::make_shared<const FineTraversabilitySnapshot>(fine->geometry(),
        fine->raw_elevation_revision(), 3U, fine->platform_profile_hash(),
        fine->hard_inflation_radius_m(), fine->preferred_clearance_m(), fine->cost_weights(),
        fine->elevation(), std::move(tiles), std::vector<TileIndex>{},
        std::vector<TileIndex>{}, FineSnapshotMetrics{});
  };
  const auto stationary = exporter.Export({.fine=make_snapshot(unknown_states),
      .anchor=Pose2{.position_m={.7,.7}}, .local_window_size_m=32., .epoch="e", .processed_stamp_ns=13},0);
  EXPECT_FALSE(stationary.stationary_support_ready);
  EXPECT_TRUE(stationary.start_connection_x.empty());
  for (const auto& tile : stationary.tiles)
    for (const auto state : tile.states) EXPECT_EQ(state, kPolicyMapUnknown);
  unknown_states[TileCellOffset(start)] = FineCellState::kBlocked;
  const auto blocked = exporter.Export({.fine=make_snapshot(unknown_states),
      .anchor=Pose2{.position_m={.7,.7}}, .local_window_size_m=32., .epoch="e", .processed_stamp_ns=14},0);
  EXPECT_FALSE(blocked.stationary_support_ready);
  EXPECT_FALSE(bootstrap.stationary_support_ready);

  for(const auto& tile:with_patch.tiles)
    if(tile.tile_x==0 && tile.tile_y==0)
      EXPECT_EQ(tile.states[TileCellOffset(start)],kPolicyMapUnknown);

}

}  // namespace lunar::incremental_navigation_ros
