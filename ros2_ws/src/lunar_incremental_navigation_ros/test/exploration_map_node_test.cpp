#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "lunar_incremental_navigation_ros/exploration_map.hpp"
#include "lunar_incremental_navigation_ros/platform_config.hpp"

namespace lunar::incremental_navigation_ros {
namespace core = lunar::incremental_navigation;
namespace {
OwnedElevationEvidence Flat(double resolution, double x = 0., double y = 0.,
                            double size = 2.) {
  const auto n = static_cast<std::size_t>(std::round(size / resolution));
  return {.geometry = {.frame_id = "odom",
                       .width = n,
                       .height = n,
                       .resolution_m = resolution,
                       .origin_m = {x, y, 0}},
          .elevation_m = std::vector<float>(n * n, 0.F),
          .map_from_source = {.parent_frame = "map", .child_frame = "odom"}};
}
core::WheeledCapability Wheel() {
  return {
      .footprint_xy_m = {{-.05, -.05}, {.05, -.05}, {.05, .05}, {-.05, .05}},
      .body_extent_m = {.1, .1, .1},
      .minimum_underbody_clearance_m = .1,
      .maximum_local_obstacle_relief_m = .2,
      .maximum_slope_rad = .5,
      .minimum_clearance_m = .0};
}
core::TraversabilityProfile Profile() {
  return {.planar_envelope_xy_m = Wheel().footprint_xy_m,
          .slope_weight = 1.,
          .relief_weight = 1.,
          .goal_position_tolerance_m = .1,
          .goal_yaw_tolerance_rad = .05};
}
}  // namespace
TEST(ExplorationMap, ResolutionHotChangeAndRollingHistory) {
  ExplorationMap map(Wheel(), Profile(), .2, 1.);
  for (double resolution : {.01, .05, .2, .07}) {
    auto source = Flat(resolution, 0., 0., resolution == .07 ? 2.1 : 2.);
    ASSERT_TRUE(map.Apply(source, 10));
    const auto captured = map.Capture();
    ASSERT_NE(captured.map.fine, nullptr);
    EXPECT_DOUBLE_EQ(captured.map.fine->geometry().resolution_m(), .2);
    EXPECT_EQ(captured.map.fine->State({4, 4}), core::FineCellState::kFree);
  }
  ASSERT_TRUE(map.Apply(Flat(.05, 4.), 20));
  auto captured = map.Capture();
  EXPECT_EQ(captured.map.fine->State({4, 4}), core::FineCellState::kFree);
  EXPECT_EQ(captured.map.fine->State({24, 4}), core::FineCellState::kFree);
  EXPECT_EQ(captured.map.fine->State({15, 4}), core::FineCellState::kUnknown);
  const auto revision = captured.map.fine->fine_traversability_revision();
  ASSERT_TRUE(map.Apply(Flat(.05, 4.), 30));
  EXPECT_EQ(map.Capture().map.processed_stamp_ns, 30);
  const auto delta = map.Capture(revision);
  EXPECT_FALSE(delta.map.full_snapshot);
  EXPECT_TRUE(delta.map.dirty_tiles.empty());
  EXPECT_TRUE(map.Capture(99999).map.full_snapshot);
}
TEST(ExplorationMap, PartialCoverageUnknownAndNanHistory) {
  ExplorationMap map(Wheel(), Profile(), .2, 1.);
  auto partial = Flat(.01, 0., 0., .1);
  ASSERT_TRUE(map.Apply(partial, 1));
  EXPECT_NE(map.Capture().map.fine->State({0, 0}), core::FineCellState::kFree);
  ASSERT_TRUE(map.Apply(Flat(.05), 2));
  const auto known = map.Capture().map.fine->State({4, 4});
  EXPECT_EQ(known, core::FineCellState::kFree);
  auto holes = Flat(.05);
  std::fill(holes.elevation_m.begin(), holes.elevation_m.end(),
            std::numeric_limits<float>::quiet_NaN());
  ASSERT_TRUE(map.Apply(holes, 3));
  EXPECT_EQ(map.Capture().map.fine->State({4, 4}), known);
  // 有限但不完整的本征证据覆盖旧 FREE；NaN 仍保留历史。
  holes.elevation_m[18 * 40 + 18] = 0.F;
  ASSERT_TRUE(map.Apply(holes, 4));
  EXPECT_NE(map.Capture().map.fine->State({4, 4}), core::FineCellState::kFree);
}
TEST(ExplorationMap, ThinSteepFeatureSurvivesConservativeProjection) {
  ExplorationMap map(Wheel(), Profile(), .2, 1.);
  auto source = Flat(.01);
  source.elevation_m[100 * 200 + 100] = .04F;
  ASSERT_TRUE(map.Apply(source, 1));
  const auto result = map.Capture();
  EXPECT_EQ(result.map.fine->State({5, 5}), core::FineCellState::kBlocked);
  EXPECT_NE(map.Project().data[0], 0);
}
TEST(ExplorationMap, RotatedTranslatedSourceKeepsActualWorldGeometry) {
  ExplorationMap map(Wheel(), Profile(), .2, 1.);
  auto source = Flat(.05);
  const double yaw = .37;
  source.map_from_source.rotation = {.w = std::cos(yaw / 2),
                                     .z = std::sin(yaw / 2)};
  source.map_from_source.translation_m = {3., -2., 0.};
  ASSERT_TRUE(map.Apply(source, 1));
  const auto fine = map.Capture().map.fine;
  const auto x = static_cast<std::int64_t>(
      std::floor((3. + std::cos(yaw) - std::sin(yaw)) / .2));
  const auto y = static_cast<std::int64_t>(
      std::floor((-2. + std::sin(yaw) + std::cos(yaw)) / .2));
  EXPECT_EQ(fine->State({x - static_cast<std::int64_t>(std::llround(
                                 fine->geometry().origin_m().x / .2)),
                         y - static_cast<std::int64_t>(std::llround(
                                 fine->geometry().origin_m().y / .2))}),
            core::FineCellState::kFree);
  EXPECT_EQ(fine->State({4 - static_cast<std::int64_t>(std::llround(
                                 fine->geometry().origin_m().x / .2)),
                         4 - static_cast<std::int64_t>(std::llround(
                                 fine->geometry().origin_m().y / .2))}),
            core::FineCellState::kUnknown);
}

TEST(ExplorationMap, FiniteUnknownChangesFineAtUnchangedRawAndDeltaEqualsFull) {
  ExplorationMap map(Wheel(), Profile(), .2, 1.);
  ASSERT_TRUE(map.Apply(Flat(.2), 1));
  const auto before = map.Capture();
  auto source = Flat(.2);
  source.terrain_measurements.resize(source.elevation_m.size(),
                                     {true, true, 0., 0., 0.});
  source.terrain_measurements[4 * 10 + 4] = {true, false, 0., 0., 0.};
  ASSERT_TRUE(map.Apply(source, 2));
  const auto after = map.Capture();
  EXPECT_EQ(after.map.fine->raw_elevation_revision(),
            before.map.fine->raw_elevation_revision());
  EXPECT_GT(after.map.fine->fine_traversability_revision(),
            before.map.fine->fine_traversability_revision());
  EXPECT_EQ(before.map.fine->State({4, 4}), core::FineCellState::kFree);
  EXPECT_EQ(after.map.fine->State({4, 4}), core::FineCellState::kUnknown);
  PolicyMapExporter exporter(Wheel(), Profile());
  const auto full = exporter.ExportObserved(after, 0);
  const auto delta = exporter.ExportObserved(
      map.Capture(before.map.fine->fine_traversability_revision()),
      before.map.fine->fine_traversability_revision());
  ASSERT_FALSE(delta.full_snapshot);
  ASSERT_EQ(full.tiles.size(), delta.tiles.size());
  for (std::size_t i = 0; i < full.tiles.size(); ++i) {
    EXPECT_EQ(full.tiles[i].states, delta.tiles[i].states);
    EXPECT_EQ(full.tiles[i].intrinsic_states, delta.tiles[i].intrinsic_states);
    EXPECT_EQ(full.tiles[i].observed, delta.tiles[i].observed);
    EXPECT_EQ(full.tiles[i].costs, delta.tiles[i].costs);
    const auto index = core::TileCellOffset({4, 4});
    EXPECT_EQ(full.tiles[i].observed[index], kPolicyMapUnknown);
  }
}
TEST(ExplorationMap, OptionalStatisticsRemainSourceLocalUnderYawAndCoarsening) {
  ExplorationMap map(Wheel(), Profile(), .2, 1.);
  auto source = Flat(.05);
  source.terrain_measurements.resize(source.elevation_m.size(),
                                     {true, true, 0., 0., 0.});
  source.terrain_measurements[20 * 40 + 20] = {true, true, .8, .03, .03};
  source.map_from_source.rotation = {.w = std::cos(.185), .z = std::sin(.185)};
  ASSERT_TRUE(map.Apply(source, 1));
  const auto snapshot = map.Capture();
  bool blocked = false;
  for (const auto tile : snapshot.map.fine->tile_indices()) {
    const auto t = snapshot.map.fine->FindTile(tile);
    blocked |= std::find(t->states().begin(), t->states().end(),
                         core::FineCellState::kBlocked) != t->states().end();
  }
  EXPECT_TRUE(blocked);
  for (const auto tile : snapshot.map.fine->elevation()->tile_indices())
    for (std::int64_t y = 0; y < 256; ++y)
      for (std::int64_t x = 0; x < 256; ++x)
        EXPECT_FALSE(snapshot.map.fine->elevation()->TerrainMeasurementsAt(
            {tile.x * 256 + x, tile.y * 256 + y}));
}
TEST(ExplorationMap, TiltAndDegenerateFootprintsDoNotFabricateFreeSupport) {
  ExplorationMap map(Wheel(), Profile(), .2, 1.);
  auto source = Flat(.05);
  source.map_from_source.rotation = {.w = std::cos(.2), .x = std::sin(.2)};
  ASSERT_TRUE(map.Apply(source, 1));
  const auto capture = map.Capture();
  ASSERT_TRUE(capture.map.fine);
  for (const auto tile : capture.map.fine->tile_indices()) {
    const auto states = capture.map.fine->FindTile(tile)->states();
    EXPECT_EQ(
        std::count(states.begin(), states.end(), core::FineCellState::kFree),
        0);
  }
  source.map_from_source.rotation = {.w = std::sqrt(.5), .x = std::sqrt(.5)};
  ASSERT_TRUE(map.Apply(source, 2));
  EXPECT_EQ(map.Capture().map.processed_stamp_ns, 2);
}

TEST(ExplorationMap, NativeSourceCostsAndHistorySizeAreRetained) {
  ExplorationMap map(Wheel(), Profile(), .2, 1.);
  auto source = Flat(.01);
  for (std::size_t y = 0; y < 200; ++y)
    for (std::size_t x = 0; x < 200; ++x)
      source.elevation_m[y * 200 + x] = .002F * x;
  ASSERT_TRUE(map.Apply(source, 1));
  const auto fine = map.Capture().map.fine;
  ASSERT_EQ(fine->State({4, 4}), core::FineCellState::kFree);
  EXPECT_GT(fine->TraversalCost({4, 4}), 0.);
  core::PersistentElevationMap native;
  ASSERT_EQ(native.Apply(source.View()).status,
            core::ElevationUpdateResult::Status::kApplied);
  const auto expected = core::MakePlatformElevationEvaluator(Wheel())->Evaluate(
      *native.Snapshot(), {90, 90});
  EXPECT_NEAR(fine->TraversalCost({4, 4}),
              std::pow(expected.slope_rad / .5, 2) +
                  std::pow(expected.relief_m / .2, 2),
              1e-3);
  EXPECT_EQ(map.StorageStats().policy_cells, 100U);
  EXPECT_EQ(map.StorageStats().fragments, 40000U);
  ASSERT_TRUE(map.Apply(source, 2));
  EXPECT_EQ(map.StorageStats().fragments, 40000U);
}
}  // namespace lunar::incremental_navigation_ros

namespace lunar::incremental_navigation_ros {
TEST(ExplorationMap, DefaultProjectionKeepsTwentyCentimeterFreeCells) {
  ExplorationMap map(Wheel(), Profile());
  ASSERT_TRUE(map.Apply(Flat(.2), 1));
  const auto fine = map.Capture().map.fine;
  const auto projection = map.Project();
  ASSERT_NE(fine, nullptr);
  EXPECT_DOUBLE_EQ(projection.geometry.resolution_m(), .2);
  std::size_t fine_free = 0;
  const auto& g = fine->geometry();
  for (auto y = g.min_inclusive().y; y < g.max_exclusive().y; ++y)
    for (auto x = g.min_inclusive().x; x < g.max_exclusive().x; ++x)
      fine_free += fine->State({x, y}) == core::FineCellState::kFree;
  EXPECT_GT(fine_free, 0U);
  EXPECT_EQ(std::count(projection.data.begin(), projection.data.end(), 0), fine_free);
}
}  // namespace lunar::incremental_navigation_ros
