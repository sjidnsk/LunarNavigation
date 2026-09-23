#include "local/request_cell_cache.hpp"
#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <random>

namespace lunar::incremental_navigation {
namespace {
std::shared_ptr<const FineTraversabilitySnapshot> Snapshot(double value) {
  PersistentElevationMap map;
  GridGeometry geometry{.frame_id = "map",
                        .width = 41,
                        .height = 29,
                        .resolution_m = .05,
                        .origin_m = {-1., -.5, 0.}};
  std::vector<float> z(geometry.CellCount(), static_cast<float>(value));
  for (std::size_t i = 0; i < z.size(); i += 7)
    z[i] = std::numeric_limits<float>::quiet_NaN();
  const std::vector<float> anchor_height{static_cast<float>(value)};
  (void)map.Apply(
      {.geometry =
           {.frame_id = "map", .width = 1, .height = 1, .resolution_m = .05},
       .elevation_m = anchor_height,
       .map_from_source = {.parent_frame = "map", .child_frame = "map"}});
  const auto applied = map.Apply(
      {.geometry = geometry,
       .elevation_m = z,
       .map_from_source = {.parent_frame = "map", .child_frame = "map"}});
  EXPECT_EQ(applied.status, ElevationUpdateResult::Status::kApplied);
  auto raw = map.Snapshot();
  struct Data {
    FineTraversabilityTile::StateArray states;
    FineTraversabilityTile::CostArray costs;
    Data() {
      states.fill(FineCellState::kUnknown);
      costs.fill(0.);
    }
  };
  std::map<TileIndex, Data> tiles;
  const auto lo = raw->geometry().min_inclusive(),
             hi = raw->geometry().max_exclusive();
  for (auto y = lo.y; y < hi.y; ++y)
    for (auto x = lo.x; x < hi.x; ++x) {
      GridIndex p{x, y};
      auto &t = tiles[TileForCell(p)];
      auto i = TileCellOffset(p);
      auto k = (x - lo.x + 3 * (y - lo.y)) % 3;
      t.states[i] = k == 0   ? FineCellState::kFree
                    : k == 1 ? FineCellState::kBlocked
                             : FineCellState::kUnknown;
      t.costs[i] =
          value + static_cast<double>((x - lo.x + y - lo.y) % 11) * .125;
    }
  FineTraversabilityTileDirectory directory(raw->geometry());
  for (auto &[k, t] : tiles)
    directory =
        directory.WithTile(k, std::make_shared<const FineTraversabilityTile>(
                                  std::move(t.states), std::move(t.costs)));
  return std::make_shared<const FineTraversabilitySnapshot>(
      raw->geometry(), raw->raw_elevation_revision(), 1U, "cache-test", .2, 0.,
      TraversalCostWeights{}, raw, std::move(directory),
      std::vector<TileIndex>{}, std::vector<TileIndex>{},
      FineSnapshotMetrics{});
}
TEST(RequestCellCache,
     MatchesSnapshotAcrossPagesNegativeIndicesAndMissingElevation) {
  auto fine = Snapshot(.25);
  RequestLocalPlanningView view(fine, {}, 0., {});
  local::RequestCellCache cache(view);
  const auto lo = view.geometry().min_inclusive(),
             hi = view.geometry().max_exclusive();
  std::vector<GridIndex> cells;
  for (auto y = lo.y - 2; y < hi.y + 2; ++y)
    for (auto x = lo.x - 2; x < hi.x + 2; ++x)
      cells.push_back({x, y});
  std::mt19937 rng(721);
  std::shuffle(cells.begin(), cells.end(), rng);
  for (int pass = 0; pass < 3; ++pass)
    for (auto p : cells) {
      EXPECT_EQ(cache.Source(p), view.Source(p));
      EXPECT_DOUBLE_EQ(cache.TraversalCost(p), view.TraversalCost(p));
      auto expected = fine->elevation()->ElevationRangeAt(p),
           actual = cache.ElevationRangeAt(p);
      ASSERT_EQ(actual.has_value(), expected.has_value());
      if (actual) {
        EXPECT_FLOAT_EQ(actual->min_m, expected->min_m);
        EXPECT_FLOAT_EQ(actual->max_m, expected->max_m);
      }
      for (auto phase : {StartPhase::kStartPrefix, StartPhase::kNormal})
        EXPECT_EQ(cache.AdvancePhase(phase, p), view.AdvancePhase(phase, p));
    }
}
TEST(RequestCellCache,
     KeepsOverridePhaseAndIndependentSnapshotEvidenceSeparate) {
  auto a = Snapshot(.25), b = Snapshot(.75);
  const auto lo = a->geometry().min_inclusive();
  GridIndex unknown{lo.x + 2, lo.y};
  Pose2 anchor{.position_m = {-1., -.5}};
  RequestLocalPlanningView patched(
      a, anchor, 3., {{unknown, LocalCellSource::kStartAssumedFree, 4.5}}),
      plain(a, anchor, 3., {}), changed(b, anchor, 3., {});
  local::RequestCellCache pc(patched), nc(plain), cc(changed);
  EXPECT_EQ(pc.Source(unknown), LocalCellSource::kStartAssumedFree);
  EXPECT_DOUBLE_EQ(pc.TraversalCost(unknown), 4.5);
  EXPECT_EQ(pc.AdvancePhase(StartPhase::kStartPrefix, unknown),
            StartPhase::kStartPrefix);
  EXPECT_FALSE(pc.AdvancePhase(StartPhase::kNormal, unknown));
  EXPECT_EQ(nc.Source(unknown), LocalCellSource::kUnknown);
  EXPECT_NE(nc.TraversalCost(unknown), cc.TraversalCost(unknown));
  const GridIndex free{lo.x, lo.y};
  EXPECT_EQ(pc.AdvancePhase(StartPhase::kStartPrefix, free),
            StartPhase::kNormal);
}
TEST(RequestCellCache, WindowDoesNotHideBaseElevationOutsideItsBoundary) {
  auto fine = Snapshot(.25);
  auto g = fine->geometry();
  auto lo = g.min_inclusive(), hi = g.max_exclusive();
  SparseGridGeometry window(g.frame_id(), g.resolution_m(), g.origin_m(),
                            {lo.x + 4, lo.y + 4}, {hi.x - 4, hi.y - 4});
  RequestLocalPlanningView view(fine, window, {}, 0., {});
  local::RequestCellCache cache(view);
  GridIndex p{lo.x + 1, lo.y};
  EXPECT_EQ(cache.Source(p), LocalCellSource::kUnknown);
  EXPECT_DOUBLE_EQ(cache.TraversalCost(p), 0.);
  ASSERT_TRUE(cache.ElevationRangeAt(p));
  EXPECT_FLOAT_EQ(cache.ElevationRangeAt(p)->min_m, .25F);
}
} // namespace
} // namespace lunar::incremental_navigation
