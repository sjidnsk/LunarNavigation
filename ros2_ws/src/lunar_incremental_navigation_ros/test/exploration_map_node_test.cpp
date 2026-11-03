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
TEST(ExplorationMap, ThinHeightObstacleSurvivesConservativeProjection) {
  ExplorationMap map(Wheel(), Profile(), .2, 1.);
  auto source = Flat(.01);
  source.elevation_m[100 * 200 + 100] = .21F;
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

TEST(ExplorationMap, LegacyIncompleteStatisticDoesNotInvalidateObservedGround) {
  ExplorationMap map(Wheel(), Profile(), .2, 1.);
  ASSERT_TRUE(map.Apply(Flat(.2), 1));
  const auto before = map.Capture();
  auto source = Flat(.2);
  source.terrain_measurements.resize(source.elevation_m.size(), {true,true,0.,0.,0.});
  source.terrain_measurements[4*10+4] = {true,false,0.,0.,0.};
  ASSERT_TRUE(map.Apply(source,2));
  const auto after=map.Capture();
  EXPECT_EQ(after.map.fine->raw_elevation_revision(),before.map.fine->raw_elevation_revision());
  EXPECT_EQ(after.map.fine->fine_traversability_revision(),before.map.fine->fine_traversability_revision());
  EXPECT_EQ(after.map.fine->State({4,4}),core::FineCellState::kFree);
  PolicyMapExporter exporter(Wheel(),Profile());
  const auto full=exporter.ExportObserved(after,0);
  const auto delta=exporter.ExportObserved(map.Capture(before.map.fine->fine_traversability_revision()),before.map.fine->fine_traversability_revision());
  EXPECT_TRUE(full.full_snapshot);
  EXPECT_FALSE(delta.full_snapshot);
  EXPECT_TRUE(delta.tiles.empty());
  EXPECT_EQ(full.fine_revision,delta.fine_revision);
}
TEST(ExplorationMap, LegacyDerivativeStatisticsDoNotOverridePlatformWindow) {
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
  EXPECT_FALSE(blocked);  // 旧3x3坡度标量不能否决新的平台尺度原始高程证据。
  source.elevation_m[20*40+20]=.21F;
  ASSERT_TRUE(map.Apply(source,2));
  bool height_blocked=false;
  for(const auto tile:map.Capture().map.fine->tile_indices()){
    const auto states=map.Capture().map.fine->FindTile(tile)->states();
    height_blocked|=std::find(states.begin(),states.end(),core::FineCellState::kBlocked)!=states.end();
  }
  EXPECT_TRUE(height_blocked);  // 真正超过平台高差能力的原始高程仍须保留。
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

namespace lunar::incremental_navigation_ros {
TEST(ExplorationMap, AutoResolutionMigratesPhysicalHistory) {
  ExplorationMap map(Wheel(), Profile());
  EXPECT_EQ(map.Capture().map.fine, nullptr);
  ASSERT_TRUE(map.Apply(Flat(.2), 1));
  auto old = map.Capture().map.fine;
  EXPECT_DOUBLE_EQ(old->geometry().resolution_m(), .2);
  ASSERT_TRUE(map.Apply(Flat(.05, 4.), 2));
  auto fine = map.Capture().map.fine;
  EXPECT_DOUBLE_EQ(fine->geometry().resolution_m(), .05);
  EXPECT_EQ(fine->State({20,20}), core::FineCellState::kFree);
  EXPECT_EQ(fine->State({100,20}), core::FineCellState::kFree);
  EXPECT_EQ(fine->State({60,20}), core::FineCellState::kUnknown);
  EXPECT_DOUBLE_EQ(map.Project().geometry.resolution_m(), .05);
  EXPECT_TRUE(map.Capture(old->fine_traversability_revision()).map.full_snapshot);
  PolicyMapExporter exporter(Wheel(), Profile());
  auto migrated_export = map.Capture(old->fine_traversability_revision());
  migrated_export.map.epoch = "auto-resolution";
  EXPECT_TRUE(exporter.ExportObserved(migrated_export, old->fine_traversability_revision()).full_snapshot);
  ASSERT_TRUE(map.Apply(Flat(.2, 8.), 3));
  EXPECT_DOUBLE_EQ(map.Capture().map.fine->geometry().resolution_m(), .2);
  EXPECT_EQ(map.Capture().map.fine->State({5,5}), core::FineCellState::kFree);
  EXPECT_EQ(map.Capture().map.fine->State({25,5}), core::FineCellState::kFree);
}
}  // namespace lunar::incremental_navigation_ros

namespace lunar::incremental_navigation_ros {
TEST(ExplorationMap, AutoCoarseningDoesNotFillUnobservedArea) {
  ExplorationMap map(Wheel(), Profile());
  ASSERT_TRUE(map.Apply(Flat(.05, 0., 0., .1), 1));
  auto holes = Flat(.2, 4.);
  ASSERT_TRUE(map.Apply(holes, 2));
  const auto capture = map.Capture();
  EXPECT_DOUBLE_EQ(capture.map.fine->geometry().resolution_m(), .2);
  EXPECT_NE(capture.map.fine->State({0,0}), core::FineCellState::kFree);
  EXPECT_NE(capture.intrinsic->Evaluate(*capture.map.fine->elevation(), {0,0}).state,
            core::IntrinsicCellState::kFree);
}
TEST(ExplorationMap, AutoInvalidInputDoesNotInitializeAndNegativeResolutionRejected) {
  ExplorationMap map(Wheel(), Profile());
  auto invalid = Flat(.05);
  invalid.geometry.resolution_m = 0.;
  EXPECT_FALSE(map.Apply(invalid, 1));
  EXPECT_EQ(map.Capture().map.fine, nullptr);
  EXPECT_THROW(ExplorationMap(Wheel(), Profile(), -.1), std::invalid_argument);
  EXPECT_THROW(ExplorationMap(Wheel(), Profile(), 0., -.1), std::invalid_argument);
}
}  // namespace lunar::incremental_navigation_ros

namespace lunar::incremental_navigation_ros {
TEST(ExplorationMap, AutoMigrationRetainsBlockedEvidenceAndRejectsOverflowAtomically) {
  ExplorationMap map(Wheel(), Profile());
  auto source = Flat(.2);
  source.elevation_m[5 * 10 + 5] = .3F;
  ASSERT_TRUE(map.Apply(source, 1));
  EXPECT_EQ(map.Capture().map.fine->State({5,5}), core::FineCellState::kBlocked);
  const auto original = map.Capture().map.fine;
  auto invalid = Flat(.05);
  invalid.map_from_source.translation_m.x = 1e30;
  EXPECT_FALSE(map.Apply(invalid, 2));
  EXPECT_EQ(map.Capture().map.fine, original);
  ASSERT_TRUE(map.Apply(Flat(.05,4.), 3));
  EXPECT_EQ(map.Capture().map.fine->State({21,21}), core::FineCellState::kBlocked);
  ASSERT_TRUE(map.Apply(Flat(.2,8.), 4));
  EXPECT_EQ(map.Capture().map.fine->State({5,5}), core::FineCellState::kBlocked);
  EXPECT_TRUE(map.Capture(original->fine_traversability_revision()).map.full_snapshot);
}
}  // namespace lunar::incremental_navigation_ros

namespace lunar::incremental_navigation_ros {
TEST(ExplorationMapService, SourceGridMigrationCannotBeDowngradedToDelta) {
  ExplorationMap map(Wheel(), Profile());
  PolicyMapExporter exporter(Wheel(), Profile());
  ASSERT_TRUE(map.Apply(Flat(.2), 1));
  const auto old_revision = map.Capture().map.fine->fine_traversability_revision();
  ASSERT_TRUE(map.Apply(Flat(.05,4.), 2));
  auto capture = map.Capture();
  capture.map.epoch = "service-test";
  PublishedExplorationMap published{exporter.ExportObserved(capture, 0),
      map.MinimumCompatibleRevision()};
  const auto after_migration = SelectExplorationMapResponse(published, old_revision);
  EXPECT_TRUE(after_migration.full_snapshot);
  EXPECT_EQ(after_migration.base_revision, 0U);
  EXPECT_FALSE(after_migration.tiles.empty());
  const auto current = SelectExplorationMapResponse(published, published.response.fine_revision);
  EXPECT_FALSE(current.full_snapshot);
  EXPECT_TRUE(current.tiles.empty());
  ASSERT_TRUE(map.Apply(Flat(.05,8.), 3));
  capture = map.Capture(); capture.map.epoch = "service-test";
  const auto compatible_revision = published.response.fine_revision;
  published = {exporter.ExportObserved(capture, 0), map.MinimumCompatibleRevision()};
  const auto compatible = SelectExplorationMapResponse(published, compatible_revision);
  EXPECT_FALSE(compatible.full_snapshot);
  EXPECT_EQ(compatible.base_revision, compatible_revision);
  EXPECT_FALSE(compatible.tiles.empty());
  EXPECT_TRUE(SelectExplorationMapResponse(published, old_revision).full_snapshot);
  const auto too_old = SelectExplorationMapResponse(published, 0, 4);
  EXPECT_FALSE(too_old.ready);
  EXPECT_EQ(too_old.reason_code, "MAP_STAMP_TOO_OLD");
}
}  // namespace lunar::incremental_navigation_ros

namespace lunar::incremental_navigation_ros {
TEST(ExplorationMap, SameScaleProjectionPreservesCanonicalNonzeroOrigin) {
  for (double origin : {.6, 4.}) {
    ExplorationMap map(Wheel(), Profile());
    ASSERT_TRUE(map.Apply(Flat(.2, origin, origin), 1));
    const auto fine = map.Capture().map.fine;
    const auto projected = map.Project();
    EXPECT_DOUBLE_EQ(projected.geometry.origin_m().x, fine->geometry().origin_m().x);
    EXPECT_DOUBLE_EQ(projected.geometry.origin_m().y, fine->geometry().origin_m().y);
    EXPECT_EQ(projected.geometry.min_inclusive(), fine->geometry().min_inclusive());
    EXPECT_EQ(projected.geometry.max_exclusive(), fine->geometry().max_exclusive());
    ASSERT_EQ(projected.data.size(), fine->geometry().CellCount());
    std::size_t offset = 0;
    for (auto y=fine->geometry().min_inclusive().y; y<fine->geometry().max_exclusive().y; ++y)
      for (auto x=fine->geometry().min_inclusive().x; x<fine->geometry().max_exclusive().x; ++x) {
        const auto state = fine->State({x,y});
        const int expected = state == core::FineCellState::kFree ? 0 :
            state == core::FineCellState::kBlocked ? 100 : -1;
        EXPECT_EQ(projected.data[offset++], expected);
      }
  }
}
}  // namespace lunar::incremental_navigation_ros

namespace lunar::incremental_navigation_ros {
namespace {
ExplorationMapFacts Facts(std::uint64_t revision=1, double resolution=.2) {
  return {.source_resolution_m=resolution, .raw_revision=revision,
          .frame="map", .epoch="facts-test", .complete_snapshot=true,
          .observation_fragments={{{{0,0},{2,0},{2,2},{0,2}}, .4F, .3, .07,
            core::IntrinsicCellState::kFree}}};
}
}
TEST(ExplorationMapFacts, SuppliedClassificationAndMeasurementsAreNotReevaluated) {
  ExplorationMap map(Wheel(), Profile());
  auto facts = Facts();
  ASSERT_TRUE(map.ApplyFacts(facts, 10));
  const auto captured = map.Capture();
  ASSERT_TRUE(captured.map.fine);
  EXPECT_EQ(captured.intrinsic->Evaluate(*captured.map.fine->elevation(), {5,5}).state,
            core::IntrinsicCellState::kFree);
  EXPECT_DOUBLE_EQ(captured.intrinsic->Evaluate(*captured.map.fine->elevation(), {5,5}).slope_rad, .3);
  EXPECT_DOUBLE_EQ(captured.intrinsic->Evaluate(*captured.map.fine->elevation(), {5,5}).relief_m, .07);
  EXPECT_FLOAT_EQ(*captured.map.fine->elevation()->ElevationAt({5,5}), .4F);
  EXPECT_GT(captured.map.fine->TraversalCost({5,5}), 0.);
  EXPECT_EQ(map.SourceEpoch(), "facts-test");
  EXPECT_EQ(map.SourceRawRevision(), 1U);
  const auto immutable = captured.map.fine;
  facts.raw_revision=2;
  facts.observation_fragments[0].state=core::IntrinsicCellState::kUnknown;
  ASSERT_TRUE(map.ApplyFacts(facts, 20));
  const auto unknown=map.Capture();
  EXPECT_EQ(unknown.intrinsic->Evaluate(*unknown.map.fine->elevation(), {5,5}).state,
            core::IntrinsicCellState::kUnknown);
  EXPECT_FLOAT_EQ(*unknown.map.fine->elevation()->ElevationAt({5,5}), .4F);
  EXPECT_EQ(immutable->State({5,5}), core::FineCellState::kFree);
  PolicyMapExporter exporter(Wheel(), Profile());
  const auto exported=exporter.ExportObserved(unknown,0);
  for (const auto& tile:exported.tiles)
    for (const auto observed:tile.observed) EXPECT_EQ(observed,kPolicyMapUnknown);
}
TEST(ExplorationMapFacts, CompleteReplacementEmptyAndRevisionIdentityAreExplicit) {
  ExplorationMap map(Wheel(), Profile());
  auto facts=Facts();
  ASSERT_TRUE(map.ApplyFacts(facts,1));
  const auto first=map.Capture().map.fine;
  auto incomplete=Facts(2); incomplete.complete_snapshot=false;
  EXPECT_FALSE(map.ApplyFacts(incomplete,2));
  EXPECT_EQ(map.Capture().map.fine,first);
  EXPECT_EQ(map.SourceRawRevision(),1U);
  ASSERT_TRUE(map.ApplyFacts(facts,3));
  EXPECT_EQ(map.Capture().map.fine,first);
  facts.raw_revision=2; facts.observation_fragments.clear();
  ASSERT_TRUE(map.ApplyFacts(facts,4));
  EXPECT_EQ(map.Capture().map.fine,nullptr);
  EXPECT_EQ(map.StorageStats().fragments,0U);
  EXPECT_EQ(map.SourceRawRevision(),2U);
  EXPECT_FALSE(map.ApplyFacts(Facts(1),5));
  facts=Facts(1,.05); facts.epoch="new-epoch"; facts.frame="world";
  ASSERT_TRUE(map.ApplyFacts(facts,6));
  EXPECT_EQ(map.Capture().map.fine->geometry().frame_id(),"world");
  EXPECT_DOUBLE_EQ(map.Capture().map.fine->geometry().resolution_m(),.05);
  EXPECT_GT(map.Capture().map.fine->fine_traversability_revision(),first->fine_traversability_revision());
  EXPECT_TRUE(map.Capture(first->fine_traversability_revision()).map.full_snapshot);
}
TEST(ExplorationMapFacts, CustomScalePartialFootprintAndMalformedInputStayConservative) {
  ExplorationMap map(Wheel(), Profile(),.2,0.);
  auto facts=Facts(1,.05);
  facts.observation_fragments[0].polygon={{0,0},{.1,0},{.1,.1},{0,.1}};
  ASSERT_TRUE(map.ApplyFacts(facts,1));
  auto captured=map.Capture();
  EXPECT_DOUBLE_EQ(captured.map.fine->geometry().resolution_m(),.2);
  EXPECT_NE(captured.intrinsic->Evaluate(*captured.map.fine->elevation(),{0,0}).state,
            core::IntrinsicCellState::kFree);
  auto bad=Facts(2); bad.observation_fragments[0].polygon[0].x=std::numeric_limits<double>::infinity();
  EXPECT_FALSE(map.ApplyFacts(bad,2));
  EXPECT_EQ(map.Capture().map.fine,captured.map.fine);
  bad=Facts(2); bad.observation_fragments.push_back(bad.observation_fragments[0]);
  EXPECT_FALSE(map.ApplyFacts(bad,2));
  EXPECT_EQ(map.Capture().map.fine,captured.map.fine);
}
}  // namespace lunar::incremental_navigation_ros

namespace lunar::incremental_navigation_ros {
TEST(ExplorationMapFacts, UnknownMissingTerrainKeepsMeasuredHeightWithoutFreeFill) {
  ExplorationMap map(Wheel(),Profile());
  auto facts=Facts(); auto& f=facts.observation_fragments[0];
  f.state=core::IntrinsicCellState::kUnknown;
  f.slope_rad=std::numeric_limits<double>::quiet_NaN();
  f.relief_m=std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(map.ApplyFacts(facts,1));
  auto captured=map.Capture();
  const auto intrinsic=captured.intrinsic->Evaluate(*captured.map.fine->elevation(),{5,5});
  EXPECT_EQ(intrinsic.state,core::IntrinsicCellState::kUnknown);
  EXPECT_TRUE(std::isnan(intrinsic.slope_rad)); EXPECT_TRUE(std::isnan(intrinsic.relief_m));
  EXPECT_FLOAT_EQ(*captured.map.fine->elevation()->ElevationAt({5,5}),.4F);
  EXPECT_EQ(captured.map.fine->State({5,5}),core::FineCellState::kUnknown);
}
TEST(ExplorationMapFacts, ReplacementRemovesOldWorldFootprintAndPreservesNewBlockedState) {
  ExplorationMap map(Wheel(),Profile());
  auto facts=Facts();
  ASSERT_TRUE(map.ApplyFacts(facts,1));
  const auto old=map.Capture().map.fine;
  facts.raw_revision=2; facts.source_resolution_m=.05;
  for(auto& p:facts.observation_fragments[0].polygon) {p.x+=20.;p.y-=4.;}
  facts.observation_fragments[0].state=core::IntrinsicCellState::kBlocked;
  ASSERT_TRUE(map.ApplyFacts(facts,2));
  const auto captured=map.Capture(old->fine_traversability_revision());
  EXPECT_TRUE(captured.map.full_snapshot);
  EXPECT_DOUBLE_EQ(captured.map.fine->geometry().resolution_m(),.05);
  EXPECT_FALSE(captured.map.fine->elevation()->ElevationAtWorld(1.,1.).has_value());
  EXPECT_FLOAT_EQ(*captured.map.fine->elevation()->ElevationAtWorld(21.,-3.),.4F);
  const auto& g=captured.map.fine->geometry();
  const core::GridIndex at{static_cast<std::int64_t>(std::floor((21.-g.origin_m().x)/.05)),
      static_cast<std::int64_t>(std::floor((-3.-g.origin_m().y)/.05))};
  EXPECT_EQ(captured.intrinsic->Evaluate(*captured.map.fine->elevation(),at).state,
            core::IntrinsicCellState::kBlocked);
  EXPECT_EQ(captured.map.fine->State(at),core::FineCellState::kBlocked);
}
}  // namespace lunar::incremental_navigation_ros

namespace lunar::incremental_navigation_ros {
TEST(ExplorationMapPublication, SharedEmptySnapshotRevokesPublishedTilesAndKeepsIdentity) {
  ExplorationMap map(Wheel(),Profile()); PolicyMapExporter exporter(Wheel(),Profile());
  auto facts=Facts(); ASSERT_TRUE(map.ApplyFacts(facts,10));
  auto capture=map.Capture(); capture.map.epoch=map.SourceEpoch();
  auto published=BuildExplorationMapPublication(capture,nullptr,exporter,
      map.MinimumCompatibleRevision(),facts.frame);
  ASSERT_FALSE(published.response.tiles.empty());
  facts.raw_revision=2; facts.observation_fragments.clear();
  ASSERT_TRUE(map.ApplyFacts(facts,20));
  capture=map.Capture(); capture.map.epoch=map.SourceEpoch();
  auto cleared=BuildExplorationMapPublication(capture,&published,exporter,
      map.MinimumCompatibleRevision(),facts.frame);
  EXPECT_FALSE(cleared.response.ready); EXPECT_EQ(cleared.response.reason_code,"INPUT_UNAVAILABLE");
  EXPECT_EQ(cleared.response.epoch,"facts-test"); EXPECT_EQ(cleared.response.frame_id,"map");
  EXPECT_EQ(cleared.response.processed_stamp.nanosec,20U);
  EXPECT_TRUE(cleared.response.full_snapshot); EXPECT_TRUE(cleared.response.tiles.empty());
  EXPECT_TRUE(SelectExplorationMapResponse(cleared,published.response.fine_revision).full_snapshot);
}
TEST(ExplorationMapPublication, SameRawNewStampRetainsTilesAndEpochChangeCannotReuseOldTiles) {
  ExplorationMap map(Wheel(),Profile()); PolicyMapExporter exporter(Wheel(),Profile());
  auto facts=Facts(); ASSERT_TRUE(map.ApplyFacts(facts,10));
  auto capture=map.Capture(); capture.map.epoch=map.SourceEpoch();
  auto first=BuildExplorationMapPublication(capture,nullptr,exporter,
      map.MinimumCompatibleRevision(),facts.frame);
  ASSERT_FALSE(first.response.tiles.empty());
  // 带标记的旧tile用于确认实际发布逻辑复用旧缓存，而非重新导出同样的内容。
  first.response.tiles.front().costs.front()=123.F;
  ASSERT_TRUE(map.ApplyFacts(facts,30));
  capture=map.Capture(); capture.map.epoch=map.SourceEpoch();
  const auto refreshed=BuildExplorationMapPublication(capture,&first,exporter,
      map.MinimumCompatibleRevision(),facts.frame);
  EXPECT_EQ(refreshed.response.processed_stamp.nanosec,30U);
  EXPECT_EQ(refreshed.response.tiles.front().costs.front(),123.F);
  // 即使fine revision碰巧相同，epoch不同也不能使用旧缓存。
  capture.map.epoch="new-source-epoch";
  const auto replaced=BuildExplorationMapPublication(capture,&first,exporter,
      map.MinimumCompatibleRevision(),facts.frame);
  EXPECT_NE(replaced.response.tiles.front().costs.front(),123.F);
  EXPECT_EQ(replaced.response.epoch,"new-source-epoch");
  EXPECT_TRUE(replaced.response.full_snapshot);
}
}  // namespace lunar::incremental_navigation_ros

namespace lunar::incremental_navigation_ros {
TEST(ExplorationMapFacts, TouchingFineFragmentGridHasNoExactOverlapCandidates) {
  ExplorationMap map(Wheel(),Profile(),.2);
  auto facts=Facts(); facts.observation_fragments.clear();
  constexpr int side=60; const double step=.2/side;
  for(int y=0;y<side;++y) for(int x=0;x<side;++x)
    facts.observation_fragments.push_back({{{x*step,y*step},{(x+1)*step,y*step},
        {(x+1)*step,(y+1)*step},{x*step,(y+1)*step}},0.F,0.,0.,core::IntrinsicCellState::kFree});
  ASSERT_TRUE(map.ApplyFacts(facts,1));
  const auto stats=map.StorageStats();
  EXPECT_EQ(stats.fragments,side*side);
  EXPECT_EQ(stats.facts_overlap_candidates,0U);
  EXPECT_GT(stats.facts_aabb_checks,0U);
  EXPECT_LT(stats.facts_aabb_checks,static_cast<std::size_t>(side*side*side*side/10));
  facts.raw_revision=2;facts.observation_fragments.push_back(facts.observation_fragments.front());
  EXPECT_FALSE(map.ApplyFacts(facts,2));
  EXPECT_EQ(map.SourceRawRevision(),1U);
}
TEST(ExplorationMapFacts, NativeP3RollingOriginSliversDoNotRejectCompleteSnapshot) {
  // 来自真实NativeP3(2x2@.05→1x1@.1, origin.x=1e-13)的完整观测足迹。
  auto facts=Facts(2,.1);facts.observation_fragments={
    {{{0.,0.},{1.0000333894311098e-13,0.},{1.000000082740371e-13,.05},{0.,.05}},
      0.F,0.,0.,core::IntrinsicCellState::kFree},
    {{{0.,.05},{1.0000333894311098e-13,.05},{1.000000082740371e-13,.1},{0.,.1}},
      0.F,0.,0.,core::IntrinsicCellState::kFree},
    {{{1.0000333894311098e-13,0.},{.10000000000010001,0.},
       {.10000000000010001,.1},{1.0000333894311098e-13,.1}},
      0.F,0.,0.,core::IntrinsicCellState::kUnknown}};
  ExplorationMap map(Wheel(),Profile());
  ASSERT_TRUE(map.ApplyFacts(facts,1));
  const auto captured=map.Capture(); ASSERT_TRUE(captured.map.fine);
  EXPECT_EQ(captured.intrinsic->Evaluate(*captured.map.fine->elevation(),{0,0}).state,
      core::IntrinsicCellState::kUnknown);
}
}  // namespace lunar::incremental_navigation_ros
