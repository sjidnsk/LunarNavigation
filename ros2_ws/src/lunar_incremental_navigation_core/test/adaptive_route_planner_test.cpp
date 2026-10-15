#include "lunar_incremental_navigation_core/adaptive_route_planner.hpp"
#include "support/complex_terrain_scenarios.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <stop_token>

namespace lunar::incremental_navigation {
namespace {
using test_support::ComplexTerrainScenario;
ComplexTerrainScenario Grid(int n = 32) {
  auto s = test_support::MakeComplexTerrainScenario(
      {.width = static_cast<std::size_t>(n),
       .height = static_cast<std::size_t>(n),
       .resolution_m = 1.});
  s.fine_states.assign(n * n, FineCellState::kBlocked);
  s.traversal_costs.assign(n * n, 0.);
  return s;
}
void Cell(ComplexTerrainScenario &s, int x, int y,
          FineCellState state = FineCellState::kFree, double cost = 0.) {
  auto k = static_cast<std::size_t>(y) * s.geometry.width + x;
  s.fine_states[k] = state;
  s.traversal_costs[k] = cost;
}
auto Fine(const ComplexTerrainScenario &s) {
  return test_support::MakeFineSnapshot(s, "adaptive-test");
}
struct PreparedPlanner {
  AdaptiveNavigationMapBuilder builder;
  AdaptiveRoutePlanner planner;
  AdaptiveRouteResult Plan(const FineTraversabilitySnapshot &fine, Point2 from,
                           Point2 to, SearchDeadline deadline,
                           const StopToken &stop) {
    auto ptr = std::shared_ptr<const FineTraversabilitySnapshot>(
        &fine, [](const auto *) {});
    auto map = builder.Update(ptr, deadline, stop);
    if (!map.snapshot) {
      AdaptiveRouteResult r;
      r.route.status = GuidanceStatus::kTimeout;
      r.reason = stop.stop_requested() ? "CANCELED" : "TIMEOUT";
      return r;
    }
    auto r = planner.Plan(*map.snapshot, from, to, deadline, stop);
    r.metrics.built_tiles = map.metrics.built_tiles;
    r.metrics.reused_tiles = map.metrics.reused_tiles;
    return r;
  }
};
SearchDeadline Deadline() {
  return SteadyClock::now() + std::chrono::seconds(5);
}
void Safe(const GlobalRoute &route, const FineTraversabilitySnapshot &fine) {
  const auto &g = fine.geometry();
  for (std::size_t i = 1; i < route.poses_map.size(); ++i) {
    const auto a = route.poses_map[i - 1].position_m,
               b = route.poses_map[i].position_m;
    const int count = std::max(
        1, static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) /
                                      g.resolution_m() * 10.)));
    for (int k = 0; k <= count; ++k) {
      double t = static_cast<double>(k) / count;
      GridIndex c{
          static_cast<std::int64_t>(std::floor(
              (a.x + (b.x - a.x) * t - g.origin_m().x) / g.resolution_m())),
          static_cast<std::int64_t>(std::floor(
              (a.y + (b.y - a.y) * t - g.origin_m().y) / g.resolution_m()))};
      ASSERT_EQ(fine.State(c), FineCellState::kFree)
          << "segment=" << i << " sample=" << k;
    }
  }
  for (auto &p : route.poses_map) {
    GridIndex c{static_cast<std::int64_t>(std::floor(
                    (p.position_m.x - g.origin_m().x) / g.resolution_m())),
                static_cast<std::int64_t>(std::floor(
                    (p.position_m.y - g.origin_m().y) / g.resolution_m()))};
    EXPECT_EQ(fine.State(c), FineCellState::kFree);
  }
}
TEST(AdaptiveRoutePlanner, RetainsUDetourOutsideStartGoalRectangle) {
  auto s = Grid();
  for (int y = 2; y < 29; ++y) {
    Cell(s, 4, y);
    Cell(s, 24, y);
  }
  for (int x = 4; x <= 24; ++x)
    Cell(s, x, 2);
  PreparedPlanner p;
  auto f = Fine(s);
  auto r = p.Plan(*f, {4.5, 27.5}, {24.5, 27.5}, Deadline(), {});
  ASSERT_EQ(r.route.status, GuidanceStatus::kAvailable);
  ASSERT_TRUE(r.route.route);
  EXPECT_GT(r.metrics.length_m, 65.);
  Safe(*r.route.route, *f);
  EXPECT_LT(std::min_element(r.route.route->poses_map.begin(),
                             r.route.route->poses_map.end(),
                             [](auto &a, auto &b) {
                               return a.position_m.y < b.position_m.y;
                             })
                ->position_m.y,
            3.);
}
TEST(AdaptiveRoutePlanner, ClosedEndpointTouchingBlockedCellIsRejected) {
  auto s = Grid();
  s.fine_states.assign(32 * 32, FineCellState::kFree);
  for (int x = 1; x < 14; ++x) Cell(s, x, 3, FineCellState::kBlocked);
  PreparedPlanner p;
  auto f = Fine(s);
  auto r = p.Plan(*f, {1.5, 4.0}, {12.5, 4.0}, Deadline(), {});
  EXPECT_EQ(r.route.status, GuidanceStatus::kNoRoute);
  EXPECT_FALSE(r.route.route);
}
TEST(AdaptiveRoutePlanner,
     TenKilometreSparseExtentDoesNotAllocateItsRectangle) {
  auto s = Grid();
  for (int y = 2; y < 29; ++y) {
    Cell(s, 4, y);
    Cell(s, 24, y);
  }
  for (int x = 4; x <= 24; ++x)
    Cell(s, x, 2);
  auto local = Fine(s);
  PersistentElevationMap raw;
  ASSERT_EQ(raw.Apply({.geometry = s.geometry,
                       .elevation_m = s.elevation_m,
                       .map_from_source = {.parent_frame = "map",
                                           .child_frame = "map"}})
                .status,
            ElevationUpdateResult::Status::kApplied);
  const std::vector<float> marker{0.F};
  auto far = s.geometry;
  far.width = far.height = 1;
  far.origin_m = {9999., 9999., 0.};
  ASSERT_EQ(raw.Apply({.geometry = far,
                       .elevation_m = marker,
                       .map_from_source = {.parent_frame = "map",
                                           .child_frame = "map"}})
                .status,
            ElevationUpdateResult::Status::kApplied);
  auto elevation = raw.Snapshot();
  auto geometry = elevation->geometry();
  ASSERT_EQ(geometry.width(), 10000U);
  auto sparse = std::make_shared<const FineTraversabilitySnapshot>(
      geometry, elevation->raw_elevation_revision(), 2, "adaptive-test", 0., 0.,
      TraversalCostWeights{}, elevation,
      local->tile_directory().WithGeometry(geometry), std::vector<TileIndex>{},
      std::vector<TileIndex>{}, FineSnapshotMetrics{});
  EXPECT_EQ(sparse->tile_indices().size(), 1U);
  EXPECT_EQ(sparse->State({5000, 5000}), FineCellState::kUnknown);
  AdaptiveNavigationMapBuilder builder;
  auto map = builder.Update(sparse, Deadline(), {});
  ASSERT_TRUE(map.snapshot);
  EXPECT_EQ(map.metrics.built_tiles, 1U);
  EXPECT_LT(map.metrics.cached_bytes, 1024U * 1024U);
  AdaptiveRoutePlanner planner;
  auto route =
      planner.Plan(*map.snapshot, {4.5, 27.5}, {24.5, 27.5}, Deadline(), {});
  ASSERT_EQ(route.route.status, GuidanceStatus::kAvailable);
  Safe(*route.route.route, *sparse);
}
TEST(AdaptiveRoutePlanner, ClosedWallAndDisconnectedComponentsAreNotJoined) {
  auto s = Grid();
  Cell(s, 3, 3);
  Cell(s, 4, 4);
  auto f = Fine(s);
  PreparedPlanner p;
  auto r = p.Plan(*f, {3.5, 3.5}, {4.5, 4.5}, Deadline(), {});
  EXPECT_EQ(r.route.status, GuidanceStatus::kNoRoute);
  EXPECT_FALSE(r.route.route);
}
TEST(AdaptiveRoutePlanner, UnknownShortcutDoesNotReplaceKnownUDetour) {
  auto s = Grid();
  for (int y = 2; y < 28; ++y) {
    Cell(s, 3, y);
    Cell(s, 20, y);
  }
  for (int x = 3; x <= 20; ++x)
    Cell(s, x, 2);
  for (int x = 4; x < 20; ++x)
    Cell(s, x, 27, FineCellState::kUnknown);
  auto f = Fine(s);
  PreparedPlanner p;
  auto r = p.Plan(*f, {3.5, 27.5}, {20.5, 27.5}, Deadline(), {});
  ASSERT_EQ(r.route.status, GuidanceStatus::kAvailable);
  ASSERT_TRUE(r.route.route);
  Safe(*r.route.route, *f);
  EXPECT_GT(r.metrics.length_m, 60.);
}
TEST(AdaptiveRoutePlanner, MissingMapIsNotClosedNoPath) {
  auto s = Grid();
  Cell(s, 3, 3);
  Cell(s, 4, 3, FineCellState::kUnknown);
  Cell(s, 15, 15);
  auto f = Fine(s);
  PreparedPlanner p;
  auto r = p.Plan(*f, {3.5, 3.5}, {15.5, 15.5}, Deadline(), {});
  EXPECT_EQ(r.reason, "MAP_INSUFFICIENT");
  EXPECT_FALSE(r.route.route);
}
TEST(AdaptiveRoutePlanner, OpenSpaceCompressesAndPreservesExactEndpoints) {
  auto s = Grid(256);
  s.fine_states.assign(256 * 256, FineCellState::kFree);
  auto f = Fine(s);
  PreparedPlanner p;
  auto r = p.Plan(*f, {.12, .27}, {255.8, 253.9}, Deadline(), {});
  ASSERT_EQ(r.route.status, GuidanceStatus::kAvailable);
  ASSERT_TRUE(r.route.route);
  EXPECT_LE(r.metrics.regions, 4U);
  EXPECT_LT(r.route.statistics.expanded_states, 8U);
  EXPECT_DOUBLE_EQ(r.route.route->poses_map.front().position_m.x, .12);
  EXPECT_DOUBLE_EQ(r.route.route->poses_map.back().position_m.y, 253.9);
  EXPECT_NEAR(r.metrics.length_m, std::hypot(255.8 - .12, 253.9 - .27), 1e-8);
}
TEST(AdaptiveRoutePlanner, FreeDirectCorridorAvoidsFragmentedRegionExpansion) {
  auto s = Grid(256);
  for (int y = 1; y < 255; ++y)
    for (int x = 1; x < 255; ++x)
      if ((x * 37 + y * 19) % 11 > 1 || std::abs(y - 128) <= 2)
        Cell(s, x, y);
  auto fine = Fine(s);
  AdaptiveNavigationMapBuilder builder;
  auto map = builder.Update(fine, Deadline(), {});
  ASSERT_TRUE(map.snapshot);
  auto now = SteadyClock::now();
  const auto deadline = now + std::chrono::microseconds(100);
  AdaptiveRoutePlanner planner({.now = [&now] {
    now += std::chrono::microseconds(1);
    return now;
  }});
  const Point2 from{1.17, 128.22}, to{253.83, 128.32};
  auto r = planner.Plan(*map.snapshot, from, to, deadline, {});
  ASSERT_EQ(r.route.status, GuidanceStatus::kAvailable);
  EXPECT_EQ(r.route.statistics.expanded_states, 0U);
  EXPECT_NEAR(r.metrics.length_m, std::hypot(to.x - from.x, to.y - from.y),
              1e-9);
  Safe(*r.route.route, *fine);
}
TEST(AdaptiveRoutePlanner, TraversalCostsRetainAlternativeLowRiskRoute) {
  auto s = Grid();
  for (int y = 2; y <= 10; ++y)
    for (int x = 2; x <= 29; ++x)
      Cell(s, x, y);
  for (int x = 5; x < 26; ++x)
    Cell(s, x, 6, FineCellState::kFree, 100.);
  auto f = Fine(s);
  PreparedPlanner p;
  auto r = p.Plan(*f, {2.5, 6.5}, {29.5, 6.5}, Deadline(), {});
  ASSERT_EQ(r.route.status, GuidanceStatus::kAvailable);
  EXPECT_LT(r.metrics.cost, 40.);
}
TEST(AdaptiveRoutePlanner, BudgetIncludesColdBuildAndCancellation) {
  auto s = Grid();
  auto f = Fine(s);
  PreparedPlanner p;
  auto r = p.Plan(*f, {2.5, 2.5}, {5.5, 5.5},
                  SteadyClock::now() - std::chrono::seconds(1), {});
  EXPECT_EQ(r.route.status, GuidanceStatus::kTimeout);
  EXPECT_FALSE(r.route.route);
  std::stop_source stop;
  stop.request_stop();
  r = p.Plan(*f, {2.5, 2.5}, {5.5, 5.5}, Deadline(), stop.get_token());
  EXPECT_EQ(r.reason, "CANCELED");
  EXPECT_FALSE(r.route.route);
}
TEST(AdaptiveRoutePlanner,
     ContentIdentityInvalidatesSameRevisionAndNewShortcut) {
  auto s = Grid();
  for (int y = 2; y < 29; ++y) {
    Cell(s, 4, y);
    Cell(s, 24, y);
  }
  for (int x = 4; x <= 24; ++x)
    Cell(s, x, 2);
  auto f = Fine(s);
  PreparedPlanner p;
  auto a = p.Plan(*f, {4.5, 27.5}, {24.5, 27.5}, Deadline(), {});
  auto warm = p.Plan(*f, {4.5, 27.5}, {24.5, 27.5}, Deadline(), {});
  ASSERT_EQ(a.route.status, GuidanceStatus::kAvailable);
  EXPECT_EQ(warm.metrics.built_tiles, 0U);
  EXPECT_GE(warm.metrics.reused_tiles, 1U);
  for (int x = 4; x <= 24; ++x)
    Cell(s, x, 27);
  auto f2 = Fine(s);
  auto b = p.Plan(*f2, {4.5, 27.5}, {24.5, 27.5}, Deadline(), {});
  ASSERT_EQ(b.route.status, GuidanceStatus::kAvailable);
  EXPECT_LT(b.metrics.length_m, a.metrics.length_m - 40.);
  for (int y = 0; y < 32; ++y)
    Cell(s, 14, y, FineCellState::kBlocked);
  auto f3 = Fine(s);
  auto c = p.Plan(*f3, {4.5, 27.5}, {24.5, 27.5}, Deadline(), {});
  EXPECT_EQ(c.route.status, GuidanceStatus::kNoRoute);
  EXPECT_FALSE(c.route.route);
}
TEST(AdaptiveRoutePlanner, RandomRockPathsAreSafeBetweenVertices) {
  for (unsigned seed = 1; seed <= 30; ++seed) {
    auto s = test_support::MakeComplexTerrainScenario(
        {.pattern = test_support::ComplexTerrainPattern::kDenseRockField,
         .width = 96,
         .height = 96,
         .resolution_m = .2,
         .seed = seed});
    auto fine = Fine(s);
    PreparedPlanner planner;
    auto start = test_support::StartPose(s).position_m,
         goal = test_support::LocalTargetForGoal(s).center;
    auto r = planner.Plan(*fine, start, goal, Deadline(), {});
    ASSERT_EQ(r.route.status, GuidanceStatus::kAvailable) << seed;
    ASSERT_TRUE(r.route.route);
    Safe(*r.route.route, *fine);
  }
}
TEST(AdaptiveRoutePlanner, UnknownPartOfMergedNonFreeFaceIsNotProvenNoRoute) {
  auto s = Grid(64);
  for (int y = 16; y < 32; ++y)
    for (int x = 16; x < 32; ++x)
      Cell(s, x, y);
  Cell(s, 32, 31, FineCellState::kUnknown);
  Cell(s, 45, 45);
  auto f = Fine(s);
  PreparedPlanner p;
  auto r = p.Plan(*f, {23.5, 23.5}, {45.5, 45.5}, Deadline(), {});
  EXPECT_EQ(r.reason, "MAP_INSUFFICIENT");
  EXPECT_EQ(r.route.status, GuidanceStatus::kUnavailable);
}
TEST(AdaptiveRoutePlanner, SameRegionCostUsesOriginalFineCosts) {
  auto s = Grid(32);
  s.fine_states.assign(32 * 32, FineCellState::kFree);
  for (int x = 0; x < 8; ++x)
    Cell(s, x, 0, FineCellState::kFree, .005 * x);
  auto f = Fine(s);
  PreparedPlanner p;
  auto r = p.Plan(*f, {.5, .5}, {7.5, .5}, Deadline(), {});
  ASSERT_EQ(r.route.status, GuidanceStatus::kAvailable);
  EXPECT_NEAR(r.metrics.cost, 7.1225, 1e-9);
}
TEST(AdaptiveNavigationMap, SmoothCostsCompressWithoutChangingInputMap) {
  auto s = Grid(256);
  s.fine_states.assign(256 * 256, FineCellState::kFree);
  for (int y = 0; y < 256; ++y)
    for (int x = 0; x < 256; ++x)
      Cell(s, x, y, FineCellState::kFree,
           .5 + .4 * std::sin(x * .031) * std::cos(y * .027));
  auto fine = Fine(s);
  double before = fine->TraversalCost({121, 87});
  AdaptiveNavigationMapBuilder builder;
  auto map = builder.Update(fine, Deadline(), {});
  ASSERT_TRUE(map.snapshot);
  EXPECT_LT(map.metrics.regions, 16384U);
  EXPECT_EQ(map.snapshot->fine_ptr(), fine);
  EXPECT_DOUBLE_EQ(fine->TraversalCost({121, 87}), before);
}
TEST(AdaptiveNavigationMap, PublishedSnapshotRemainsImmutableAcrossUpdates) {
  auto s = Grid();
  for (int x = 1; x < 31; ++x)
    Cell(s, x, 3);
  auto fine = Fine(s);
  AdaptiveNavigationMapBuilder builder;
  auto first = builder.Update(fine, Deadline(), {});
  ASSERT_TRUE(first.snapshot);
  Cell(s, 15, 3, FineCellState::kBlocked);
  auto next = builder.Update(Fine(s), Deadline(), {});
  ASSERT_TRUE(next.snapshot);
  ASSERT_TRUE(first.snapshot->RegionAt({15, 3}));
  EXPECT_TRUE(first.snapshot->RegionAt({15, 3})->free);
  EXPECT_FALSE(next.snapshot->RegionAt({15, 3})->free);
  AdaptiveRoutePlanner p;
  EXPECT_EQ(p.Plan(*first.snapshot, {1.5, 3.5}, {29.5, 3.5}, Deadline(), {})
                .route.status,
            GuidanceStatus::kAvailable);
  EXPECT_EQ(p.Plan(*next.snapshot, {1.5, 3.5}, {29.5, 3.5}, Deadline(), {})
                .route.status,
            GuidanceStatus::kNoRoute);
}
TEST(AdaptiveRoutePlanner, NegativeWorldCoordinatesRemainValid) {
  auto s = Grid();
  s.geometry.origin_m = {-17., -49., 0.};
  for (int x = 1; x < 30; ++x)
    Cell(s, x, 3);
  auto f = Fine(s);
  PreparedPlanner p;
  auto r = p.Plan(*f, {-15.5, -45.5}, {11.5, -45.5}, Deadline(), {});
  ASSERT_EQ(r.route.status, GuidanceStatus::kAvailable);
  ASSERT_TRUE(r.route.route);
  Safe(*r.route.route, *f);
}
} // namespace
} // namespace lunar::incremental_navigation

#include "lunar_incremental_navigation_core/planning_session_coordinator.hpp"
#include "local/grid_supercover.hpp"
namespace lunar::incremental_navigation {
TEST(AdaptiveOnlineNavigation, SafePrefixWaitResumeAndCancelKeepFinalGoal) {
  auto s = Grid();
  s.fine_states.assign(32 * 32, FineCellState::kUnknown);
  for (int y = 2; y <= 10; ++y) for (int x = 2; x <= 12; ++x) Cell(s, x, y);
  auto fine = test_support::MakeFineSnapshot(s, "online", 1);
  AdaptiveNavigationMapBuilder builder;
  auto nav = builder.Update(fine, Deadline(), {}).snapshot;
  const auto wheel = test_support::BenchmarkWheelCapability();
  const auto profile = test_support::BenchmarkTraversabilityProfile();
  WheelLocalPlanner local(wheel); LocalTargetSelector selector;
  RequestLocalStartPatchBuilder patch; AdaptiveRoutePlanner global;
  PlanningSessionPorts ports;
  ports.plan_adaptive_global = [&](const auto& m, auto from, auto goal, auto d, const auto& stop) {
    EXPECT_DOUBLE_EQ(goal.x, 26.5);
    return global.Plan(m, from, goal, d, stop);
  };
  ports.select_target_controlled = [&](const auto& f, const auto& w, auto st,
      const auto& goal, const auto& route, const auto& control) {
    return selector.SelectRolling(f, w, st, goal, route, control);
  };
  ports.build_start_patch = [&](auto f, const auto& w, const auto& st, const auto& c, const auto& p) {
    return patch.Build(f, w, st, c, p);
  };
  ports.plan_wheel = [&](const auto& v, const auto& st, const auto& t, auto d, const auto& stop) {
    return local.Plan(v, st, t, d, stop);
  };
  PlanningSessionCoordinator c(wheel, profile,
      {.goal_position_tolerance_m=.3, .goal_yaw_tolerance_rad=.26}, ports);
  ASSERT_TRUE(c.Start(SessionId{{1}}, {.target_x_m=26.5, .target_y_m=6.5}).accepted);
  auto out = c.PlanCycle({.base_link_pose={.position_m={3.5,6.5}}},
      {.fine=fine, .navigation=nav}, CycleTrigger::kContinue, Deadline());
  ASSERT_TRUE(out.path_reference) << out.feedback.reason_code;
  EXPECT_FALSE(out.terminal); EXPECT_FALSE(out.path_reference->reaches_final_goal);
  EXPECT_EQ(out.global_reason, "KNOWN_PREFIX_FOUND");
  const auto& poses = out.path_reference->path.poses;
  for (size_t i=1; i<poses.size(); ++i)
    EXPECT_TRUE(local::VisitSupercoverCells(fine->geometry(), poses[i-1].position_m,
        poses[i].position_m, [&](GridIndex g) { return fine->State(g)==FineCellState::kFree; }));
  const auto end = poses.back().position_m;
  EXPECT_GT(end.x, 11.); EXPECT_LT(end.x, 13.);
  StateInput at_end{.base_link_pose={.position_m={end.x,end.y}}};
  out = c.PlanCycle(at_end, {.fine=fine, .navigation=nav}, CycleTrigger::kSegmentEnd, Deadline());
  EXPECT_FALSE(out.terminal); EXPECT_EQ(out.feedback.reason_code,"WAITING_FOR_MAP");
  out = c.PlanCycle(at_end, {.fine=fine, .navigation=nav}, CycleTrigger::kContinue, Deadline());
  EXPECT_FALSE(out.path_reference); EXPECT_EQ(out.feedback.reason_code,"WAITING_FOR_MAP");
  for (int y=2; y<=10; ++y) for(int x=13; x<=29; ++x) Cell(s,x,y);
  fine=test_support::MakeFineSnapshot(s,"online",2);
  nav=builder.Update(fine,Deadline(),{}).snapshot;
  out=c.PlanCycle(at_end,{.fine=fine,.navigation=nav},CycleTrigger::kContinue,Deadline());
  ASSERT_TRUE(out.path_reference) << out.feedback.reason_code;
  EXPECT_EQ(out.global_reason,"KNOWN_ROUTE_FOUND"); EXPECT_TRUE(out.path_reference->reaches_final_goal);
  EXPECT_DOUBLE_EQ(out.path_reference->path.poses.back().position_m.x,26.5);
  const auto canceled=c.Cancel(CancelReason::kCanceled);
  EXPECT_EQ(canceled.result.outcome,SessionOutcome::kCanceled);
}
}
namespace lunar::incremental_navigation {
TEST(AdaptiveRoutePlanner, UnknownClosedEndpointContactRemainsMapInsufficient) {
  auto s=Grid();s.fine_states.assign(32*32,FineCellState::kFree);
  for(int x=1;x<14;++x) Cell(s,x,3,FineCellState::kUnknown);
  PreparedPlanner p;auto fine=Fine(s);
  auto out=p.Plan(*fine,{1.5,4.5},{12.5,4.},Deadline(),{});
  EXPECT_EQ(out.reason,"MAP_INSUFFICIENT"); EXPECT_FALSE(out.route.route);
  EXPECT_EQ(out.route.status,GuidanceStatus::kUnavailable);
  out=p.Plan(*fine,{1.5,4.5},{0.,4.5},Deadline(),{});
  EXPECT_EQ(out.reason,"MAP_INSUFFICIENT"); EXPECT_FALSE(out.route.route);
}
}
