// Differential harness: run the identical binary against baseline and candidate
// libraries via LD_LIBRARY_PATH. Timings exclude output serialization.
#include "lunar_incremental_navigation_core/legged_local_planner.hpp"
#include "lunar_incremental_navigation_core/wheel_local_planner.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <numbers>
#include <random>
#include <sstream>
#include <stop_token>
using namespace lunar::incremental_navigation;

LeggedCapability Legged() {
  LeggedCapability c;
  c.maximum_step_height_m = .5;
  c.maximum_gap_width_m = .3;
  c.motion_primitives = {{.primitive_id = "forward",
                          .kind = LeggedPrimitiveKind::kForward,
                          .body_frame_displacement_m = {.2, 0, 0}},
                         {.primitive_id = "back",
                          .kind = LeggedPrimitiveKind::kBackward,
                          .body_frame_displacement_m = {-.2, 0, 0}},
                         {.primitive_id = "left",
                          .kind = LeggedPrimitiveKind::kLateralLeft,
                          .body_frame_displacement_m = {0, .2, 0}},
                         {.primitive_id = "right",
                          .kind = LeggedPrimitiveKind::kLateralRight,
                          .body_frame_displacement_m = {0, -.2, 0}},
                         {.primitive_id = "spin",
                          .kind = LeggedPrimitiveKind::kSpin,
                          .yaw_change_rad = std::numbers::pi / 16}};
  return c;
}
WheeledCapability Wheel() {
  WheeledCapability c;
  c.motion_primitives = {{.primitive_id = "spin",
                          .kind = WheelPrimitiveKind::kSpinCounterclockwise}};
  return c;
}
std::shared_ptr<const FineTraversabilitySnapshot>
Fixture(int seed, int scene, int epoch, double resolution) {
  constexpr int n = 64;
  // Shifted world origin exercises negative global cell and tile indices.
  GridGeometry g{.frame_id = "map",
                 .width = n,
                 .height = n,
                 .resolution_m = resolution,
                 .origin_m = {-10 * resolution, -5 * resolution, 0}};
  std::mt19937 rng(seed * 71 + scene * 17 + epoch * 3);
  std::vector<float> z(n * n, 0.F);
  struct Data {
    FineTraversabilityTile::StateArray states;
    FineTraversabilityTile::CostArray costs;
    Data() {
      states.fill(FineCellState::kUnknown);
      costs.fill(0.);
    }
  };
  std::map<TileIndex, Data> data;
  for (int y = 0; y < n; ++y)
    for (int x = 0; x < n; ++x) {
      GridIndex cell{x - 10, y - 5};
      auto &tile = data[TileForCell(cell)];
      const auto i = TileCellOffset(cell);
      auto dice = rng() % 100;
      auto state = FineCellState::kFree;
      if (scene == 2 && dice < 20)
        state = FineCellState::kBlocked;
      if (scene == 3 && dice < 20) {
        state = FineCellState::kUnknown;
        z[y * n + x] = std::numeric_limits<float>::quiet_NaN();
      }
      if (scene == 4)
        z[y * n + x] = (x / 8 % 2 ? 1.F : -1.F) * (epoch == 0 ? .2F : .6F);
      if (scene == 6 && x == 32)
        state = FineCellState::kBlocked;
      if (scene == 7 && x == 32 && (epoch == 1 || y < 30 || y > 34))
        state = FineCellState::kBlocked;
      if ((x <= 4 && y <= 4) || (x >= 59 && y >= 59)) {
        state = FineCellState::kFree;
        z[y * n + x] = 0.;
      }
      if (scene == 5 && x <= 3 && y <= 3) {
        state = FineCellState::kUnknown;
        z[y * n + x] = std::numeric_limits<float>::quiet_NaN();
      }
      tile.states[i] = state;
      tile.costs[i] = (scene == 1 || scene == 2 || scene == 7)
                          ? (rng() % 100) * .03125
                          : 0.;
    }
  PersistentElevationMap map;
  // Establish origin zero before growing into negative indices.
  const std::vector<float> anchor_height{0.F};
  (void)map.Apply(
      {.geometry = {.frame_id = "map",
                    .width = 1,
                    .height = 1,
                    .resolution_m = resolution},
       .elevation_m = anchor_height,
       .map_from_source = {.parent_frame = "map", .child_frame = "map"}});
  if (map.Apply(
             {.geometry = g,
              .elevation_m = z,
              .map_from_source = {.parent_frame = "map", .child_frame = "map"}})
          .status != ElevationUpdateResult::Status::kApplied)
    throw std::runtime_error("invalid fixture");
  auto raw = map.Snapshot();
  FineTraversabilityTileDirectory tiles(raw->geometry());
  for (auto &[index, tile] : data)
    tiles = tiles.WithTile(index,
                           std::make_shared<const FineTraversabilityTile>(
                               std::move(tile.states), std::move(tile.costs)));
  return std::make_shared<const FineTraversabilitySnapshot>(
      raw->geometry(), raw->raw_elevation_revision(), epoch + 1,
      "synthetic-search", .2, 0., TraversalCostWeights{}, raw, std::move(tiles),
      std::vector<TileIndex>{}, std::vector<TileIndex>{},
      FineSnapshotMetrics{});
}
void Emit(int seed, int scene, int epoch, double res, int platform, int query,
          const LocalPlanResult &r, double ms) {
  std::cout << seed << ',' << scene << ',' << epoch << ',' << res << ','
            << platform << ',' << query << ',' << int(r.status) << ',' << ms
            << ',' << r.statistics.expanded_states << ','
            << r.reaches_final_goal << ',' << r.reason_code << ',';
  std::ostringstream signature;
  signature << std::hexfloat;
  for (auto *path : {&r.raw_path, &r.path}) {
    signature << '|';
    for (const auto &p : *path) {
      auto v = p.pose.position_m;
      auto q = p.pose.orientation;
      signature << v.x << ':' << v.y << ':' << v.z << ':' << q.w << ':' << q.x
                << ':' << q.y << ':' << q.z << ':' << int(p.phase) << ';';
    }
  }
  std::cout << signature.str() << '\n';
}
int main(int argc, char **argv) {
  const int seeds = argc > 1 ? std::stoi(argv[1]) : 10;
  std::cout << "seed,scene,epoch,resolution,platform,query,status,plan_ms,"
               "expanded,reaches_final_goal,reason,path_signature\n";
  for (int seed = 0; seed < seeds; ++seed)
    for (int scene = 0; scene < 8; ++scene)
      for (double res : {.2, .1, .05}) {
        WheelLocalPlanner wheel(Wheel());
        LeggedLocalPlanner legged(Legged(),
                                  {.terminal_yaw_tolerance_rad = .025});
        for (int epoch = 0; epoch < 2; ++epoch) {
          auto fine = Fixture(seed, scene, epoch, res);
          auto point = [&](double x, double y) {
            return Point2{(x - 10) * res, (y - 5) * res};
          };
          for (int query = 0; query < 5; ++query) {
            Pose2 start{.position_m = point(query == 1 ? 61.5 : 2.5,
                                            query == 1 ? 61.5 : 2.5),
                        .yaw_rad = query == 1 ? std::numbers::pi : 0.};
            LocalTarget goal{.center = query == 1   ? point(2.5, 2.5)
                                       : query == 2 ? point(4.25, 2.5)
                                                    : point(61.25, 61.75),
                             .position_tolerance_m = .01,
                             .is_final_goal = true,
                             .terminal_yaw_rad =
                                 (seed % 4) * std::numbers::pi / 2};
            std::vector<LocalCellOverride> overrides;
            if (scene == 5 && query != 1)
              for (int y = 0; y <= 3; ++y)
                for (int x = 0; x <= 3; ++x)
                  overrides.push_back({{x - 10, y - 5},
                                       LocalCellSource::kStartAssumedFree,
                                       .25});
            RequestLocalPlanningView view(fine, start, 3.,
                                          std::move(overrides));
            std::stop_source source;
            if (query == 3)
              source.request_stop();
            auto deadline = query == 4 ? SteadyClock::time_point::min()
                                       : SteadyClock::time_point::max();
            for (int platform = 0; platform < 2; ++platform) {
              auto begin = SteadyClock::now();
              auto result = platform ? legged.Plan(view, start, goal, deadline,
                                                   source.get_token())
                                     : wheel.Plan(view, start, goal, deadline,
                                                  source.get_token());
              double ms = std::chrono::duration<double, std::milli>(
                              SteadyClock::now() - begin)
                              .count();
              Emit(seed, scene, epoch, res, platform, query, result, ms);
            }
          }
        }
      }
}
