#include "lunar_incremental_navigation_core/adaptive_route_planner.hpp"
#include "local/grid_supercover.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lunar::incremental_navigation {
namespace {
constexpr double kDiagonal = 1.4142135623730950488;
struct Interrupted final {};
struct Control final {
  const NowFn &now;
  SearchDeadline deadline;
  const StopToken &stop;
  void Check() const {
    if (stop.stop_requested() || now() >= deadline)
      throw Interrupted{};
  }
};
using Region = AdaptiveNavigationRegion;
Point2 Center(const SparseGridGeometry &g, GridIndex c) {
  return {
      std::fma(static_cast<double>(c.x) + .5, g.resolution_m(), g.origin_m().x),
      std::fma(static_cast<double>(c.y) + .5, g.resolution_m(),
               g.origin_m().y)};
}
std::optional<GridIndex> CellAt(const SparseGridGeometry &g, Point2 p) {
  if (!std::isfinite(p.x) || !std::isfinite(p.y))
    return {};
  const double x = std::floor((p.x - g.origin_m().x) / g.resolution_m());
  const double y = std::floor((p.y - g.origin_m().y) / g.resolution_m());
  if (x < static_cast<double>(g.min_inclusive().x) ||
      x >= static_cast<double>(g.max_exclusive().x) ||
      y < static_cast<double>(g.min_inclusive().y) ||
      y >= static_cast<double>(g.max_exclusive().y))
    return {};
  return GridIndex{static_cast<std::int64_t>(x), static_cast<std::int64_t>(y)};
}
double GridLength(GridIndex a, GridIndex b, double r) {
  const double x =
      std::abs(static_cast<double>(a.x) - static_cast<double>(b.x));
  const double y =
      std::abs(static_cast<double>(a.y) - static_cast<double>(b.y));
  return r * (std::max(x, y) + (kDiagonal - 1.) * std::min(x, y));
}
void AppendGrid(std::vector<GridIndex> &path, GridIndex to,
                const Control &control) {
  if (path.empty()) {
    path.push_back(to);
    return;
  }
  auto from = path.back();
  while (from != to) {
    if ((path.size() & 255U) == 0U)
      control.Check();
    from.x += (to.x > from.x) - (to.x < from.x);
    from.y += (to.y > from.y) - (to.y < from.y);
    path.push_back(from);
  }
}
double StepCost(const FineTraversabilitySnapshot &f, GridIndex a, GridIndex b) {
  return GridLength(a, b, f.geometry().resolution_m()) *
         (1. + .5 * (f.TraversalCost(a) + f.TraversalCost(b)));
}
// Bresenham is restricted to FREE cells, with both side cells checked on
// diagonals. Smoothing cannot create an unobserved shortcut or cut a corner.
bool Sight(const FineTraversabilitySnapshot &f, GridIndex a, GridIndex b,
           const Control &control, double &cost) {
  cost = 0.;
  auto p = a;
  auto dx = std::abs(b.x - a.x), dy = std::abs(b.y - a.y);
  const int sx = (b.x > a.x) - (b.x < a.x), sy = (b.y > a.y) - (b.y < a.y);
  auto error = dx - dy;
  std::size_t count = 0;
  if (f.State(p) != FineCellState::kFree)
    return false;
  while (p != b) {
    if ((++count & 63U) == 0U)
      control.Check();
    auto next = p;
    const auto e2 = error * 2;
    if (e2 > -dy) {
      error -= dy;
      next.x += sx;
    }
    if (e2 < dx) {
      error += dx;
      next.y += sy;
    }
    if (f.State(next) != FineCellState::kFree)
      return false;
    if (next.x != p.x && next.y != p.y &&
        (f.State({next.x, p.y}) != FineCellState::kFree ||
         f.State({p.x, next.y}) != FineCellState::kFree))
      return false;
    cost += StepCost(f, p, next);
    p = next;
  }
  return true;
}
// Traverse every crossed fine cell, including both sides of exact corners.
// A FREE, zero-terrain-cost straight line attains the Euclidean cost lower
// bound, so accepting it cannot suppress a cheaper risk-avoiding alternative.
bool ContinuousSegment(const FineTraversabilitySnapshot &f, Point2 a, Point2 b,
                       const Control &control, bool require_zero_cost,
                       double &cost) {
  std::size_t visited = 0;
  if (!local::VisitSupercoverCells(f.geometry(), {a.x, a.y, 0.}, {b.x, b.y, 0.},
      [&](GridIndex cell) {
        if ((++visited & 63U) == 0U) control.Check();
        return f.State(cell) == FineCellState::kFree &&
            (!require_zero_cost || f.TraversalCost(cell) == 0.);
      })) return false;
  const auto &g = f.geometry();
  auto cell = *CellAt(g, a);
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double length = std::hypot(dx, dy);
  double t = 0.;
  cost = 0.;
  std::size_t count = 0;
  while (t < 1.) {
    if ((++count & 63U) == 0U)
      control.Check();
    if (f.State(cell) != FineCellState::kFree ||
        (require_zero_cost && f.TraversalCost(cell) != 0.))
      return false;
    const double tx =
        dx == 0. ? INFINITY
                 : (g.origin_m().x +
                    (static_cast<double>(cell.x) + (dx > 0. ? 1. : 0.)) *
                        g.resolution_m() -
                    a.x) /
                       dx;
    const double ty =
        dy == 0. ? INFINITY
                 : (g.origin_m().y +
                    (static_cast<double>(cell.y) + (dy > 0. ? 1. : 0.)) *
                        g.resolution_m() -
                    a.y) /
                       dy;
    const double next = std::max(t, std::min({1., tx, ty}));
    cost += (next - t) * length * (1. + f.TraversalCost(cell));
    if (next >= 1.)
      break;
    if (tx == ty && (f.State({cell.x + ((dx > 0.) - (dx < 0.)), cell.y}) !=
                         FineCellState::kFree ||
                     f.State({cell.x, cell.y + ((dy > 0.) - (dy < 0.))}) !=
                         FineCellState::kFree))
      return false;
    if (tx <= ty)
      cell.x += (dx > 0.) - (dx < 0.);
    if (ty <= tx)
      cell.y += (dy > 0.) - (dy < 0.);
    t = next;
  }
  return true;
}
struct Edge final {
  const Region *to{};
  GridIndex exit{}, entry{};
  double cost{};
};
struct Record final {
  double cost{std::numeric_limits<double>::infinity()};
  const Region *parent{};
  GridIndex exit{}, entry{};
  bool closed{};
};
struct Open final {
  const Region *node;
  double g{}, f{};
  std::uint64_t order{};
};
struct Later final {
  bool operator()(const Open &a, const Open &b) const {
    if (a.f != b.f)
      return a.f > b.f;
    if (a.g != b.g)
      return a.g < b.g;
    return a.order > b.order;
  }
};
} // namespace

struct AdaptiveRoutePlanner::Impl final {
  AdaptiveRoutePlannerConfig config;
  explicit Impl(AdaptiveRoutePlannerConfig c) : config(std::move(c)) {
    if (!config.now)
      throw std::invalid_argument("adaptive planner requires a clock");
  }
  std::vector<Edge> Neighbors(const AdaptiveNavigationSnapshot &map,
                              const Region *node, GridIndex anchor,
                              const Region *final, GridIndex goal,
                              const Control &c, bool &unknown) const {
    const auto &f = map.fine();
    std::vector<Edge> edges;
    const auto add = [&](GridIndex exit, GridIndex entry) {
      const auto *next = map.RegionAt(entry);
      if (!next || !next->free) {
        if (!f.geometry().Contains(entry) ||
            f.State(entry) == FineCellState::kUnknown)
          unknown = true;
        return;
      }
      if (next == node)
        return;
      const auto target = next == final ? goal : next->Anchor();
      const double cost =
          GridLength(anchor, exit, f.geometry().resolution_m()) *
              (1. + node->cost) +
          StepCost(f, exit, entry) +
          GridLength(entry, target, f.geometry().resolution_m()) *
              (1. + next->cost);
      auto existing = std::find_if(edges.begin(), edges.end(),
                                   [&](const Edge &e) { return e.to == next; });
      Edge edge{next, exit, entry, cost};
      if (existing == edges.end())
        edges.push_back(edge);
      else if (cost < existing->cost)
        *existing = edge;
    };
    const auto &g = f.geometry();
    const auto xmin = node->x, xmax = node->x + node->size - 1;
    const auto ymin = node->y, ymax = node->y + node->size - 1;
    for (int side = 0; side < 4; ++side) {
      const bool vertical = side < 2;
      const bool lower = (side % 2) == 0;
      const auto fixed =
          vertical ? (lower ? xmin : xmax) : (lower ? ymin : ymax);
      // A FREE leaf cannot extend beyond the published geometry.
      if ((lower &&
           fixed == (vertical ? g.min_inclusive().x : g.min_inclusive().y)) ||
          (!lower && fixed == (vertical ? g.max_exclusive().x - 1
                                        : g.max_exclusive().y - 1))) {
        unknown = true;
        continue;
      }
      auto pos = vertical ? ymin : xmin;
      const auto end = vertical ? ymax : xmax;
      while (pos <= end) {
        c.Check();
        GridIndex entry = vertical ? GridIndex{fixed + (lower ? -1 : 1), pos}
                                   : GridIndex{pos, fixed + (lower ? -1 : 1)};
        const auto *next = map.RegionAt(entry);
        auto last = pos;
        if (next)
          last = std::min(end, (vertical ? next->y : next->x) + next->size - 1);
        // Prefer a portal close to the current anchor while retaining every
        // adjacent component. No discretized entrance can erase a passage.
        const auto portal =
            std::clamp(vertical ? anchor.y : anchor.x, pos, last);
        GridIndex exit =
            vertical ? GridIndex{fixed, portal} : GridIndex{portal, fixed};
        entry = vertical ? GridIndex{fixed + (lower ? -1 : 1), portal}
                         : GridIndex{portal, fixed + (lower ? -1 : 1)};
        add(exit, entry);
        if (last == end)
          break;
        pos = last + 1;
      }
    }
    for (int dy : {-1, 1})
      for (int dx : {-1, 1}) {
        GridIndex exit{dx < 0 ? xmin : xmax, dy < 0 ? ymin : ymax};
        if (exit.x == g.min_inclusive().x ||
            exit.x == g.max_exclusive().x - 1 ||
            exit.y == g.min_inclusive().y || exit.y == g.max_exclusive().y - 1)
          continue;
        GridIndex entry{exit.x + dx, exit.y + dy};
        if (f.State({entry.x, exit.y}) == FineCellState::kFree &&
            f.State({exit.x, entry.y}) == FineCellState::kFree)
          add(exit, entry);
      }
    return edges;
  }
};
AdaptiveRoutePlanner::AdaptiveRoutePlanner(AdaptiveRoutePlannerConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}
AdaptiveRoutePlanner::~AdaptiveRoutePlanner() = default;
AdaptiveRoutePlanner::AdaptiveRoutePlanner(AdaptiveRoutePlanner &&) noexcept =
    default;
AdaptiveRoutePlanner &
AdaptiveRoutePlanner::operator=(AdaptiveRoutePlanner &&) noexcept = default;

AdaptiveRouteResult
AdaptiveRoutePlanner::Plan(const AdaptiveNavigationSnapshot &map, Point2 start,
                           Point2 goal, SearchDeadline deadline,
                           const StopToken &stop) {
  const auto &fine = map.fine();
  AdaptiveRouteResult result;
  result.metrics.regions = map.metrics().regions;
  result.metrics.cached_bytes = map.metrics().cached_bytes;
  const Control control{impl_->config.now, deadline, stop};
  const auto begin = impl_->config.now();
  auto search_begin = begin;
  try {
    control.Check();
    const auto &g = fine.geometry();
    const auto from = CellAt(g, start), to = CellAt(g, goal);
    if (!from || !to) {
      result.reason = "MAP_INSUFFICIENT";
      return result;
    }
    if (fine.State(*from) != FineCellState::kFree ||
        fine.State(*to) != FineCellState::kFree) {
      result.reason = (fine.State(*from) == FineCellState::kUnknown ||
                       fine.State(*to) == FineCellState::kUnknown)
                          ? "MAP_INSUFFICIENT"
                          : "ENDPOINT_BLOCKED";
      result.route.status = result.reason == "ENDPOINT_BLOCKED"
                                ? GuidanceStatus::kNoRoute
                                : GuidanceStatus::kUnavailable;
      return result;
    }
    for (const auto p : {start, goal}) {
      bool blocked = false, unknown = false;
      const bool inside = local::VisitSupercoverCells(g, {p.x, p.y, 0.}, {p.x, p.y, 0.},
          [&](GridIndex cell) {
            control.Check();
            blocked |= fine.State(cell) == FineCellState::kBlocked;
            unknown |= fine.State(cell) == FineCellState::kUnknown;
            return true;
          });
      if (blocked || unknown || !inside) {
        result.reason = blocked ? "ENDPOINT_BLOCKED" : "MAP_INSUFFICIENT";
        result.route.status = blocked ? GuidanceStatus::kNoRoute : GuidanceStatus::kUnavailable;
        return result;
      }
    }

    search_begin = impl_->config.now();
    double direct_cost = 0.;
    if (ContinuousSegment(fine, start, goal, control, true, direct_cost)) {
      control.Check();
      GlobalRoute route;
      for (const auto p : {start, goal}) {
        Pose3 pose{};
        pose.position_m = {p.x, p.y, 0.};
        route.poses_map.push_back(pose);
      }
      result.metrics.cost = direct_cost;
      result.metrics.length_m = std::hypot(goal.x - start.x, goal.y - start.y);
      result.route.status = GuidanceStatus::kAvailable;
      result.route.route = std::move(route);
      result.reason = "KNOWN_ROUTE_FOUND";
      result.metrics.search_ms = std::chrono::duration<double, std::milli>(
                                     impl_->config.now() - search_begin)
                                     .count();
      return result;
    }
    const auto *first = map.RegionAt(*from);
    const auto *final = map.RegionAt(*to);
    std::unordered_map<const Region *, Record> records;
    std::priority_queue<Open, std::vector<Open>, Later> open;
    std::uint64_t order = 0;
    const auto point = [&](const Region *n) {
      return Center(g, n == first ? *from : n == final ? *to : n->Anchor());
    };
    const auto heuristic = [&](const Region *n) {
      auto p = point(n), q = Center(g, *to);
      return std::hypot(p.x - q.x, p.y - q.y);
    };
    records[first].cost = 0.;
    open.push({first, 0., heuristic(first), order++});
    auto &stats = result.route.statistics;
    stats.open_peak = 1;
    bool found = false, unknown = false;
    while (!open.empty()) {
      control.Check();
      auto item = open.top();
      open.pop();
      auto &current = records[item.node];
      if (current.closed || current.cost != item.g)
        continue;
      current.closed = true;
      ++stats.expanded_states;
      if (item.node == final) {
        found = true;
        break;
      }
      auto anchor = item.node == first ? *from : item.node->Anchor();
      auto neighbors = impl_->Neighbors(map, item.node, anchor, final, *to,
                                        control, unknown);
      for (const auto &edge : neighbors) {
        ++stats.evaluated_transitions;
        const double cost = item.g + edge.cost;
        auto &next = records[edge.to];
        if (cost >= next.cost)
          continue;
        // Reopening preserves correctness even if a future heuristic changes.
        next = {cost, item.node, edge.exit, edge.entry, false};
        open.push({edge.to, cost, cost + heuristic(edge.to), order++});
        ++stats.generated_states;
        stats.open_peak = std::max(stats.open_peak, open.size());
      }
    }
    if (!found) {
      result.reason = unknown ? "MAP_INSUFFICIENT" : "NO_KNOWN_ROUTE";
      result.route.status =
          unknown ? GuidanceStatus::kUnavailable : GuidanceStatus::kNoRoute;
      result.metrics.search_ms = std::chrono::duration<double, std::milli>(
                                     impl_->config.now() - search_begin)
                                     .count();
      return result;
    }
    std::vector<const Region *> chain;
    for (auto n = final; n; n = records[n].parent) {
      control.Check();
      chain.push_back(n);
    }
    std::reverse(chain.begin(), chain.end());
    std::vector<GridIndex> cells{*from};
    for (std::size_t i = 1; i < chain.size(); ++i) {
      const auto &r = records[chain[i]];
      AppendGrid(cells, r.exit, control);
      AppendGrid(cells, r.entry, control);
      AppendGrid(cells, chain[i] == final ? *to : chain[i]->Anchor(), control);
    }
    if (cells.back() != *to)
      AppendGrid(cells, *to, control);
    std::vector<double> cumulative(cells.size(), 0.);
    for (std::size_t i = 1; i < cells.size(); ++i) {
      if ((i & 63U) == 0U)
        control.Check();
      if (fine.State(cells[i]) != FineCellState::kFree)
        throw std::logic_error("adaptive witness left FREE space");
      if (cells[i].x != cells[i - 1].x && cells[i].y != cells[i - 1].y &&
          (fine.State({cells[i].x, cells[i - 1].y}) != FineCellState::kFree ||
           fine.State({cells[i - 1].x, cells[i].y}) != FineCellState::kFree))
        throw std::logic_error("adaptive witness cut a corner");
      cumulative[i] =
          cumulative[i - 1] + StepCost(fine, cells[i - 1], cells[i]);
    }
    std::vector<GridIndex> simplified{cells.front()};
    for (std::size_t i = 0; i + 1 < cells.size();) {
      control.Check();
      double cost = 0.;
      std::size_t chosen = i + 1;
      const auto safe = [&](std::size_t j) {
        return Sight(fine, cells[i], cells[j], control, cost) &&
               cost <= cumulative[j] - cumulative[i] + 1.e-9;
      };
      if (safe(cells.size() - 1))
        chosen = cells.size() - 1;
      else {
        std::size_t lo = i + 1, hi = cells.size() - 1;
        while (lo + 1 < hi) {
          auto mid = lo + (hi - lo) / 2;
          if (safe(mid))
            lo = mid;
          else
            hi = mid;
        }
        chosen = lo;
      }
      simplified.push_back(cells[chosen]);
      i = chosen;
    }
    GlobalRoute route;
    route.expanded_states = stats.expanded_states;
    const auto append = [&](Point2 p) {
      if (!route.poses_map.empty() &&
          route.poses_map.back().position_m.x == p.x &&
          route.poses_map.back().position_m.y == p.y)
        return;
      Pose3 pose{};
      pose.position_m = {p.x, p.y, 0.};
      route.poses_map.push_back(pose);
    };
    if (first == final) {
      // The entire continuous segment lies in one convex, uniform FREE leaf.
      append(start);
      append(goal);
      if (!ContinuousSegment(fine, start, goal, control, false,
                             result.metrics.cost))
        throw std::logic_error("same-region segment left FREE space");
    } else {
      append(start);
      for (auto cell : simplified)
        append(Center(g, cell));
      append(goal);
      for (std::size_t i = 1; i < simplified.size(); ++i) {
        double cost = 0.;
        if (!Sight(fine, simplified[i - 1], simplified[i], control, cost))
          throw std::logic_error("invalid smoothed witness");
        result.metrics.cost += cost;
      }
      auto a = Center(g, *from), b = Center(g, *to);
      result.metrics.cost += std::hypot(start.x - a.x, start.y - a.y) *
                                 (1. + fine.TraversalCost(*from)) +
                             std::hypot(goal.x - b.x, goal.y - b.y) *
                                 (1. + fine.TraversalCost(*to));
    }
    for (std::size_t i = 1; i < route.poses_map.size(); ++i) {
      const auto &a = route.poses_map[i - 1].position_m;
      const auto &b = route.poses_map[i].position_m;
      result.metrics.length_m += std::hypot(b.x - a.x, b.y - a.y);
    }
    control.Check();
    result.route.status = GuidanceStatus::kAvailable;
    result.route.route = std::move(route);
    result.reason = "KNOWN_ROUTE_FOUND";
    result.metrics.search_ms = std::chrono::duration<double, std::milli>(
                                   impl_->config.now() - search_begin)
                                   .count();
  } catch (const Interrupted &) {
    result.route.status = GuidanceStatus::kTimeout;
    result.route.route.reset();
    result.reason = stop.stop_requested() ? "CANCELED" : "TIMEOUT";
    result.metrics.search_ms = std::chrono::duration<double, std::milli>(
                                   impl_->config.now() - search_begin)
                                   .count();
  }
  return result;
}
} // namespace lunar::incremental_navigation
