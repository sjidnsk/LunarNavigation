// Same maps, endpoints and deadline for the production legacy planner, an
// independent full-fine A* reference, and read-only adaptive-map navigation.
#include "lunar_incremental_navigation_core/adaptive_route_planner.hpp"
#include "lunar_incremental_navigation_core/global_guidance_builder.hpp"
#include "support/complex_terrain_scenarios.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>
using namespace lunar::incremental_navigation;
namespace ts = lunar::incremental_navigation::test_support;
namespace {
using Clock = SteadyClock;
double Ms(Clock::time_point a) {
  return std::chrono::duration<double, std::milli>(Clock::now() - a).count();
}
struct Hash {
  std::size_t operator()(GridIndex p) const {
    return static_cast<std::uint64_t>(p.x) * 0x9e3779b97f4a7c15ULL ^
           static_cast<std::uint64_t>(p.y);
  }
};
std::uint64_t Mix(std::uint64_t x) {
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}
struct Fixture {
  std::shared_ptr<const FineTraversabilitySnapshot> fine;
  GridIndex start, goal;
  std::uint64_t hash{};
  GridIndex change;
};
Fixture Make(const std::string &pattern, int n, double resolution,
             unsigned seed) {
  ts::ComplexTerrainScenario s;
  s.geometry = {"map",
                static_cast<std::size_t>(n),
                static_cast<std::size_t>(n),
                resolution,
                {}};
  const std::size_t count = static_cast<std::size_t>(n) * n;
  s.elevation_m.assign(count, 0.f);
  s.fine_states.assign(count, FineCellState::kFree);
  s.traversal_costs.assign(count, 0.);
  s.start_cell = {3, n / 2};
  s.goal_cell = {n - 4, n / 2};
  auto set = [&](int x, int y, FineCellState state, double cost = 0.) {
    auto i = static_cast<std::size_t>(y) * n + x;
    s.fine_states[i] = state;
    s.traversal_costs[i] = cost;
  };
  if (pattern == "u_cave" || pattern == "unknown_shortcut") {
    s.fine_states.assign(count, FineCellState::kBlocked);
    int a = n / 4 + seed % 3, b = 3 * n / 4 - seed % 5,
        top = 3 * n / 4 - seed % 7;
    for (int y = 3; y <= top; ++y)
      for (int d = -2; d <= 2; ++d) {
        set(a + d, y, FineCellState::kFree);
        set(b + d, y, FineCellState::kFree);
      }
    for (int x = a - 2; x <= b + 2; ++x)
      for (int y = 3; y <= 7; ++y)
        set(x, y, FineCellState::kFree);
    s.start_cell = {a, top};
    s.goal_cell = {b, top};
    if (pattern == "unknown_shortcut")
      for (int x = a + 3; x < b - 2; ++x)
        for (int d = -2; d <= 2; ++d)
          set(x, top + d, FineCellState::kUnknown);
  } else if (pattern == "long_wall") {
    for (int y = 6; y < n - 1; ++y)
      for (int d = -1; d <= 1; ++d)
        set(n / 2 + d, y, FineCellState::kBlocked);
  } else if (pattern == "maze") {
    int gap = std::max(5, n / 32), spacing = std::max(12, n / 12);
    bool low = true;
    for (int x = spacing; x < n - spacing; x += spacing) {
      int cy = low ? gap : n - gap;
      for (int y = 1; y < n - 1; ++y)
        if (std::abs(y - cy) > gap / 2)
          set(x, y, FineCellState::kBlocked);
      low = !low;
    }
  } else if (pattern == "rocks") {
    for (int y = 1; y < n - 1; ++y)
      for (int x = 1; x < n - 1; ++x)
        if (Mix(static_cast<std::uint64_t>(y) * n + x + seed) % 100 < 18)
          set(x, y, FineCellState::kBlocked);
    for (int x = 2; x < n - 2; ++x)
      for (int d = -2; d <= 2; ++d)
        set(x, n / 2 + d, FineCellState::kFree);
  } else if (pattern == "risk_band") {
    for (int y = 8; y < n - 8; ++y)
      for (int x = n / 3; x < 2 * n / 3; ++x)
        set(x, y, FineCellState::kFree, 12.);
  } else if (pattern == "rough_cost") {
    for (int y = 1; y < n - 1; ++y)
      for (int x = 1; x < n - 1; ++x)
        set(x, y, FineCellState::kFree,
            .5 + .4 * std::sin(x * .031 + seed) * std::cos(y * .027));
  } else if (pattern == "dead_ends") {
    s.fine_states.assign(count, FineCellState::kBlocked);
    for (int x = 2; x < n - 2; ++x)
      for (int d = -2; d <= 2; ++d)
        set(x, n / 2 + d, FineCellState::kFree);
    for (int x = 10; x < n - 10; x += std::max(8, n / 24))
      for (int y = n / 4; y < 3 * n / 4; ++y)
        for (int d = -1; d <= 1; ++d)
          set(x + d, y, FineCellState::kFree);
  } else if (pattern == "enclosed") {
    const int x = n - 8, y = n / 2;
    for (int d = -3; d <= 3; ++d) {
      set(x + d, y - 3, FineCellState::kBlocked);
      set(x + d, y + 3, FineCellState::kBlocked);
      set(x - 3, y + d, FineCellState::kBlocked);
      set(x + 3, y + d, FineCellState::kBlocked);
    }
    s.goal_cell = {x, y};
  } else if (pattern == "diagonal_trap") {
    s.fine_states.assign(count, FineCellState::kBlocked);
    s.start_cell = {3, 3};
    s.goal_cell = {4, 4};
    set(3, 3, FineCellState::kFree);
    set(4, 4, FineCellState::kFree);
  } else if (pattern == "winding_cave") {
    s.fine_states.assign(count, FineCellState::kBlocked);
    int last = n / 2;
    for (int x = 2; x < n - 2; ++x) {
      int cy = n / 2 +
               static_cast<int>(.3 * n * std::sin(6.283185307 * x / (n - 1.)));
      for (int y = std::min(last, cy) - 3; y <= std::max(last, cy) + 3; ++y)
        set(x, y, FineCellState::kFree);
      last = cy;
    }
    s.start_cell = {
        3, n / 2 +
               static_cast<int>(.3 * n * std::sin(6.283185307 * 3 / (n - 1.)))};
    s.goal_cell = {
        n - 4,
        n / 2 + static_cast<int>(.3 * n *
                                 std::sin(6.283185307 * (n - 4) / (n - 1.)))};
  } else if (pattern == "open") {
    s.start_cell = {3, 3};
    s.goal_cell = {n - 4, n - 4};
  } else
    throw std::runtime_error("unknown scenario");
  for (int x = 0; x < n; ++x) {
    set(x, 0, FineCellState::kBlocked);
    set(x, n - 1, FineCellState::kBlocked);
  }
  for (int y = 0; y < n; ++y) {
    set(0, y, FineCellState::kBlocked);
    set(n - 1, y, FineCellState::kBlocked);
  }
  std::uint64_t hash = 0;
  GridIndex change = s.start_cell;
  double nearest = INFINITY;
  for (std::size_t i = 0; i < count; ++i) {
    hash ^= Mix(i ^ Mix(static_cast<unsigned>(s.fine_states[i]) ^
                        (static_cast<std::uint64_t>(
                             std::llround(s.traversal_costs[i] * 1e6))
                         << 3)));
    GridIndex cell{static_cast<std::int64_t>(i % n),
                   static_cast<std::int64_t>(i / n)};
    double d = std::hypot(cell.x - (s.start_cell.x + s.goal_cell.x) / 2.,
                          cell.y - (s.start_cell.y + s.goal_cell.y) / 2.);
    if (s.fine_states[i] == FineCellState::kFree && cell != s.start_cell &&
        cell != s.goal_cell && d < nearest) {
      nearest = d;
      change = cell;
    }
  }
  return {ts::MakeFineSnapshot(s, "benchmark-identical-platform"), s.start_cell,
          s.goal_cell, hash, change};
}
Point2 Point(const FineTraversabilitySnapshot &f, GridIndex c) {
  auto &g = f.geometry();
  return {g.origin_m().x + (c.x + .5) * g.resolution_m(),
          g.origin_m().y + (c.y + .5) * g.resolution_m()};
}
GridIndex Cell(const FineTraversabilitySnapshot &f, Point2 p) {
  auto &g = f.geometry();
  return {static_cast<std::int64_t>(
              std::floor((p.x - g.origin_m().x) / g.resolution_m())),
          static_cast<std::int64_t>(
              std::floor((p.y - g.origin_m().y) / g.resolution_m()))};
}
GlobalRouteResult Reference(const FineTraversabilitySnapshot &f,
                            GridIndex start, GridIndex goal,
                            SearchDeadline deadline) {
  struct R {
    double g = INFINITY;
    GridIndex parent{};
    bool closed = false;
  };
  struct Q {
    GridIndex c;
    double g, h;
  };
  struct Later {
    bool operator()(const Q &a, const Q &b) const {
      return a.g + a.h == b.g + b.h ? a.g < b.g : a.g + a.h > b.g + b.h;
    }
  };
  std::unordered_map<GridIndex, R, Hash> records;
  std::priority_queue<Q, std::vector<Q>, Later> open;
  GlobalRouteResult result;
  const auto h = [&](GridIndex c) {
    return std::hypot(static_cast<double>(c.x - goal.x),
                      static_cast<double>(c.y - goal.y)) *
           f.geometry().resolution_m();
  };
  if (f.State(start) != FineCellState::kFree ||
      f.State(goal) != FineCellState::kFree)
    return result;
  records[start].g = 0.;
  open.push({start, 0., h(start)});
  bool found = false;
  while (!open.empty()) {
    if (Clock::now() >= deadline) {
      result.status = GuidanceStatus::kTimeout;
      return result;
    }
    auto q = open.top();
    open.pop();
    auto &record = records[q.c];
    if (record.closed || record.g != q.g)
      continue;
    record.closed = true;
    ++result.statistics.expanded_states;
    if (q.c == goal) {
      found = true;
      break;
    }
    for (int dy = -1; dy <= 1; ++dy)
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0)
          continue;
        GridIndex c{q.c.x + dx, q.c.y + dy};
        if (f.State(c) != FineCellState::kFree)
          continue;
        if (dx && dy &&
            (f.State({c.x, q.c.y}) != FineCellState::kFree ||
             f.State({q.c.x, c.y}) != FineCellState::kFree))
          continue;
        auto cost =
            q.g + std::hypot(dx, dy) * f.geometry().resolution_m() *
                      (1. + .5 * (f.TraversalCost(q.c) + f.TraversalCost(c)));
        auto &next = records[c];
        if (cost >= next.g)
          continue;
        next = {cost, q.c, false};
        open.push({c, cost, h(c)});
        ++result.statistics.generated_states;
        result.statistics.open_peak =
            std::max(result.statistics.open_peak, open.size());
      }
  }
  if (!found) {
    result.status = GuidanceStatus::kNoRoute;
    return result;
  }
  GlobalRoute path;
  for (auto c = goal;; c = records[c].parent) {
    if (Clock::now() >= deadline) {
      result.status = GuidanceStatus::kTimeout;
      return result;
    }
    auto p = Point(f, c);
    Pose3 pose{};
    pose.position_m = {p.x, p.y, 0.};
    path.poses_map.push_back(pose);
    if (c == start)
      break;
  }
  std::reverse(path.poses_map.begin(), path.poses_map.end());
  result.status = GuidanceStatus::kAvailable;
  result.route = std::move(path);
  return result;
}
struct Validation {
  bool safe = true;
  std::size_t unknown{}, blocked{};
  double length{}, cost{};
};
Validation Validate(const FineTraversabilitySnapshot &f,
                    const GlobalRoute &route) {
  Validation v;
  const double resolution = f.geometry().resolution_m();
  if (route.poses_map.empty()) {
    v.safe = false;
    return v;
  }
  for (std::size_t i = 1; i < route.poses_map.size(); ++i) {
    auto a = route.poses_map[i - 1].position_m,
         b = route.poses_map[i].position_m;
    double length = std::hypot(b.x - a.x, b.y - a.y);
    v.length += length;
    int steps =
        std::max(1, static_cast<int>(std::ceil(length / (resolution * .2))));
    auto previous = Cell(f, {a.x, a.y});
    for (int j = 0; j <= steps; ++j) {
      double t = static_cast<double>(j) / steps;
      auto c = Cell(f, {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
      auto state = f.State(c);
      if (state != FineCellState::kFree) {
        v.safe = false;
        v.unknown += state == FineCellState::kUnknown;
        v.blocked += state == FineCellState::kBlocked;
      }
      if (c.x != previous.x && c.y != previous.y &&
          (f.State({c.x, previous.y}) != FineCellState::kFree ||
           f.State({previous.x, c.y}) != FineCellState::kFree))
        v.safe = false;
      if (j)
        v.cost += length / steps * (1. + f.TraversalCost(c));
      previous = c;
    }
  }
  return v;
}
std::shared_ptr<const FineTraversabilitySnapshot>
Change(std::shared_ptr<const FineTraversabilitySnapshot> f, GridIndex c,
       FineCellState state, double cost) {
  auto tile_index = TileForCell(c);
  auto tile = f->FindTile(tile_index);
  auto states = tile->states();
  auto costs = tile->traversal_costs();
  states[TileCellOffset(c)] = state;
  costs[TileCellOffset(c)] = cost;
  auto directory = f->tile_directory().WithTile(
      tile_index,
      std::make_shared<const FineTraversabilityTile>(states, costs));
  return std::make_shared<const FineTraversabilitySnapshot>(
      f->geometry(), f->raw_elevation_revision(),
      f->fine_traversability_revision() + 1, f->platform_profile_hash(),
      f->hard_inflation_radius_m(), f->preferred_clearance_m(),
      f->cost_weights(), f->elevation(), std::move(directory),
      std::vector<TileIndex>{tile_index}, std::vector<TileIndex>{tile_index},
      FineSnapshotMetrics{});
}
std::string Name(GuidanceStatus s) {
  return s == GuidanceStatus::kAvailable ? "AVAILABLE"
         : s == GuidanceStatus::kNoRoute ? "NO_ROUTE"
         : s == GuidanceStatus::kTimeout ? "TIMEOUT"
                                         : "UNAVAILABLE";
}
void Run(const std::string &algorithm, const std::string &pattern, int n,
         double resolution, unsigned seed, int iterations, int budget,
         const Fixture &fixture,
         std::shared_ptr<const GlobalGuidanceSnapshot> initial_guidance,
         double guidance_ms, const std::string &scope) {
  auto fine = fixture.fine;
  auto guidance = initial_guidance;
  GlobalRoutePlanner legacy;
  AdaptiveNavigationMapBuilder builder;
  AdaptiveRoutePlanner adaptive;
  GlobalGuidanceBuilder guide(1.);
  auto map_hash = fixture.hash;
  for (int i = 0; i < iterations + 4; ++i) {
    std::string phase = i == 0                ? "cold"
                        : i < iterations      ? "warm_same"
                        : i == iterations     ? "moving_start"
                        : i == iterations + 1 ? "changed_goal"
                        : i == iterations + 2 ? "new_obstacle"
                                              : "obstacle_removed";
    auto start = fixture.start, goal = fixture.goal;
    if (phase == "moving_start") {
      GridIndex candidate{start.x + 1, start.y};
      if (fine->State(candidate) == FineCellState::kFree)
        start = candidate;
    }
    if (phase == "changed_goal") {
      GridIndex candidate{goal.x, goal.y - 1};
      if (fine->State(candidate) == FineCellState::kFree)
        goal = candidate;
    }
    if (phase == "new_obstacle" || phase == "obstacle_removed") {
      GridIndex at = fixture.change;
      const auto original_state = fixture.fine->State(at);
      const auto original_cost = fixture.fine->TraversalCost(at);
      const auto next_state =
          phase == "new_obstacle" ? FineCellState::kBlocked : original_state;
      const double next_cost = phase == "new_obstacle" ? 0. : original_cost;
      fine = Change(fine, at, next_state, next_cost);
      const auto index = static_cast<std::uint64_t>(at.y) * n + at.x;
      auto value = [&](FineCellState state, double cost) {
        return Mix(
            index ^
            Mix(static_cast<unsigned>(state) ^
                (static_cast<std::uint64_t>(std::llround(cost * 1e6)) << 3)));
      };
      map_hash = fixture.hash ^ value(original_state, original_cost) ^
                 value(next_state, next_cost);
      if (algorithm == "legacy_coarse") {
        auto t = Clock::now();
        guidance = guide.Derive(fine, {}, guidance);
        guidance_ms = Ms(t);
      }
    }
    auto begin = Clock::now();
    auto deadline = begin + std::chrono::milliseconds(budget);
    double map_ms = 0., search_ms = 0.;
    GlobalRouteResult route;
    AdaptiveMapMetrics map_metrics;
    if (algorithm == "legacy_coarse") {
      map_ms =
          (i == 0 || phase == "new_obstacle" || phase == "obstacle_removed")
              ? guidance_ms
              : 0.;
      auto t = Clock::now();
      if (scope == "pipeline")
        deadline -= std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double, std::milli>(map_ms));
      route = legacy.Plan(*guidance, Point(*fine, start), Point(*fine, goal),
                          deadline, {});
      search_ms = Ms(t);
    } else if (algorithm == "fine_reference") {
      auto t = Clock::now();
      route = Reference(*fine, start, goal, deadline);
      search_ms = Ms(t);
    } else {
      auto map = builder.Update(
          fine, scope == "search" ? begin + std::chrono::seconds(5) : deadline,
          {});
      map_metrics = map.metrics;
      map_ms = Ms(begin);
      if (map.snapshot) {
        auto t = Clock::now();
        if (scope == "search")
          deadline = t + std::chrono::milliseconds(budget);
        auto r = adaptive.Plan(*map.snapshot, Point(*fine, start),
                               Point(*fine, goal), deadline, {});
        search_ms = Ms(t);
        route = std::move(r.route);
      } else
        route.status = GuidanceStatus::kTimeout;
    }
    const double query_total = Ms(begin);
    Validation validation;
    if (route.route)
      validation = Validate(*fine, *route.route);
    else
      validation.safe = false;
    rusage resources{};
    getrusage(RUSAGE_SELF, &resources);
    std::cout << std::setprecision(12) << "{\"scenario\":\"" << pattern
              << "\",\"algorithm\":\"" << algorithm << "\",\"size\":" << n
              << ",\"resolution_m\":" << resolution
              << ",\"extent_m\":" << n * resolution << ",\"seed\":" << seed
              << ",\"map_hash\":\"" << map_hash << "\",\"phase\":\"" << phase
              << "\",\"iteration\":" << i << ",\"budget_ms\":" << budget
              << ",\"budget_scope\":\"" << scope
              << "\",\"fine_revision\":" << fine->fine_traversability_revision()
              << ",\"start_x\":" << start.x << ",\"start_y\":" << start.y
              << ",\"goal_x\":" << goal.x << ",\"goal_y\":" << goal.y
              << ",\"status\":\"" << Name(route.status)
              << "\",\"safe\":" << (validation.safe ? "true" : "false")
              << ",\"map_ms\":" << map_ms << ",\"search_ms\":" << search_ms
              << ",\"query_total_ms\":" << query_total
              << ",\"pipeline_total_ms\":"
              << (algorithm == "legacy_coarse" ? query_total + map_ms
                                               : query_total)
              << ",\"length_m\":" << validation.length
              << ",\"cost\":" << validation.cost
              << ",\"unknown_samples\":" << validation.unknown
              << ",\"blocked_samples\":" << validation.blocked
              << ",\"expanded\":" << route.statistics.expanded_states
              << ",\"open_peak\":" << route.statistics.open_peak
              << ",\"regions\":" << map_metrics.regions
              << ",\"built_tiles\":" << map_metrics.built_tiles
              << ",\"reused_tiles\":" << map_metrics.reused_tiles
              << ",\"cache_bytes\":" << map_metrics.cached_bytes
              << ",\"peak_rss_kib\":" << resources.ru_maxrss
              << ",\"pid\":" << getpid() << "}\n";
    std::cout.flush();
  }
}
} // namespace
int main(int argc, char **argv) {
  if (argc < 6) {
    std::cerr << "scenario size resolution seed iterations [budget_ms]\n";
    return 2;
  }
  try {
    std::string pattern = argv[1];
    int n = std::stoi(argv[2]);
    double r = std::stod(argv[3]);
    unsigned seed = std::stoul(argv[4]);
    int iterations = std::stoi(argv[5]);
    int budget = argc > 6 ? std::stoi(argv[6]) : 500;
    if (n < 32 || n > 4096 || r <= 0 || iterations < 1 || budget < 1)
      throw std::runtime_error("invalid benchmark arguments");
    auto fixture = Make(pattern, n, r, seed);
    auto t = Clock::now();
    GlobalGuidanceBuilder guide(1.);
    auto guidance = guide.Derive(fixture.fine, {});
    double guidance_ms = Ms(t);
    // Fork after common input preparation: isolated caches/RSS, identical
    // immutable maps, no concurrent timed algorithms, no duplicated fixture
    // work.
    std::array<std::string, 3> algorithms{"legacy_coarse", "fine_reference",
                                          "adaptive"};
    std::rotate(algorithms.begin(), algorithms.begin() + seed % 3,
                algorithms.end());
    for (auto scope : {"pipeline", "search"})
      for (const auto &algorithm : algorithms) {
        std::cout.flush();
        auto pid = fork();
        if (pid < 0)
          throw std::runtime_error("fork failed");
        if (pid == 0) {
          try {
            Run(algorithm, pattern, n, r, seed, iterations, budget, fixture,
                guidance, guidance_ms, scope);
            _exit(0);
          } catch (const std::exception &e) {
            std::cerr << e.what() << '\n';
            _exit(3);
          }
        }
        int status{};
        waitpid(pid, &status, 0);
        if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
          return 3;
      }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
  return 0;
}
