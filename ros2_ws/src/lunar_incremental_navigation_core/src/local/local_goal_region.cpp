#include "lunar_incremental_navigation_core/local_goal_region.hpp"

#include "lunar_incremental_navigation_core/arrival_tolerance.hpp"
#include "lunar_incremental_navigation_core/local_planning_window.hpp"
#include "local/grid_supercover.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace lunar::incremental_navigation {
namespace {

[[nodiscard]] double Distance(const Point2 a, const Point2 b) noexcept {
  return std::hypot(a.x - b.x, a.y - b.y);
}

[[nodiscard]] Point2 Center(const SparseGridGeometry& geometry,
                            const GridIndex cell) noexcept {
  return {std::fma(static_cast<double>(cell.x) + 0.5,
                   geometry.resolution_m(), geometry.origin_m().x),
          std::fma(static_cast<double>(cell.y) + 0.5,
                   geometry.resolution_m(), geometry.origin_m().y)};
}

struct RouteSegment final {
  Point2 a;
  Point2 b;
  double length{};
  double remaining{};
};

[[nodiscard]] bool Interrupted(const SearchControl& control) {
  return control.canceled() || control.expired();
}

// Only the part of a guidance segment inside this request's local window is
// eligible as a connection. Tail arc length still belongs to the full route.
[[nodiscard]] std::optional<RouteSegment> Clip(
    const RouteSegment& segment, const SparseGridGeometry& window) {
  long double enter = 0., leave = 1.;
  const auto lo = window.min_inclusive(), hi = window.max_exclusive();
  const auto origin = window.origin_m();
  const long double resolution = window.resolution_m();
  const auto axis = [&](long double a, long double b, long double minimum,
                        long double maximum) {
    if (a == b) return a >= minimum && a <= maximum;
    long double first = (minimum - a) / (b - a);
    long double last = (maximum - a) / (b - a);
    if (first > last) std::swap(first, last);
    enter = std::max(enter, first);
    leave = std::min(leave, last);
    return enter <= leave;
  };
  if (!axis(segment.a.x, segment.b.x, origin.x + lo.x * resolution,
            origin.x + hi.x * resolution) ||
      !axis(segment.a.y, segment.b.y, origin.y + lo.y * resolution,
            origin.y + hi.y * resolution)) return std::nullopt;
  const auto point = [&](long double t) -> Point2 {
    return {static_cast<double>(std::lerp(static_cast<long double>(segment.a.x),
                                         static_cast<long double>(segment.b.x), t)),
            static_cast<double>(std::lerp(static_cast<long double>(segment.a.y),
                                         static_cast<long double>(segment.b.y), t))};
  };
  const auto a = point(enter), b = point(leave);
  return RouteSegment{a, b, Distance(a, b),
      static_cast<double>((1. - leave) * segment.length + segment.remaining)};
}

[[nodiscard]] bool FreeConnection(
    const Point2 a, const Point2 b, const FineTraversabilitySnapshot& fine,
    const SearchControl& control) {
  // Both endpoints lie in the convex local window. The fine geometry is used
  // for closed-segment contacts at its boundary; no map is modified or copied.
  return local::VisitSupercoverCells(fine.geometry(), {a.x, a.y, 0.},
      {b.x, b.y, 0.}, [&](GridIndex cell) {
        return !Interrupted(control) && fine.State(cell) == FineCellState::kFree;
      });
}

[[nodiscard]] double ConnectedRemaining(
    const Point2 point, const std::vector<RouteSegment>& local_route,
    const FineTraversabilitySnapshot& fine, const SearchControl& control) {
  double best = std::numeric_limits<double>::infinity();
  for (const auto& segment : local_route) {
    if (Interrupted(control)) return best;
    // Along one straight segment, connection distance + tail arc cannot
    // increase when the connection advances. A visible forward endpoint is
    // therefore optimal on this segment, including genuine open-space cuts.
    const double endpoint_cost = Distance(point, segment.b) + segment.remaining;
    if (endpoint_cost >= best) continue;
    if (FreeConnection(point, segment.b, fine, control)) {
      best = endpoint_cost;
      continue;
    }
    const long double dx = static_cast<long double>(segment.b.x) - segment.a.x;
    const long double dy = static_cast<long double>(segment.b.y) - segment.a.y;
    const long double denominator = dx * dx + dy * dy;
    const long double t = denominator > 0. ? std::clamp(
        ((static_cast<long double>(point.x) - segment.a.x) * dx +
         (static_cast<long double>(point.y) - segment.a.y) * dy) / denominator,
        0.L, 1.L) : 0.;
    const Point2 projected{
        static_cast<double>(std::lerp(static_cast<long double>(segment.a.x),
                                     static_cast<long double>(segment.b.x), t)),
        static_cast<double>(std::lerp(static_cast<long double>(segment.a.y),
                                     static_cast<long double>(segment.b.y), t))};
    const double remaining = Distance(point, projected) +
        static_cast<double>((1. - t) * segment.length) + segment.remaining;
    if (remaining < best && FreeConnection(point, projected, fine, control)) {
      best = remaining;
    }
    // An occluded endpoint or perpendicular foot does not invalidate every
    // connection to the segment. Its beginning is another conservative anchor.
    const double beginning_cost = Distance(point, segment.a) +
        segment.length + segment.remaining;
    if (beginning_cost < best && FreeConnection(point, segment.a, fine, control)) {
      best = beginning_cost;
    }
  }
  return best;
}

// Nearest geometric projection onto the ordered route, then its remaining
// arc length. Lateral distance penalizes departing from that guidance. This
// permits U turns whose first executable leg increases final-goal distance.
[[nodiscard]] double Remaining(const Point2 point,
                               const std::vector<RouteSegment>& route,
                               const Point2 goal,
                               const bool include_lateral = true) noexcept {
  double nearest = std::numeric_limits<double>::infinity();
  double remaining = Distance(point, goal);
  for (const auto& segment : route) {
    const double dx = segment.b.x - segment.a.x;
    const double dy = segment.b.y - segment.a.y;
    const double t = std::clamp(
        ((point.x - segment.a.x) * dx + (point.y - segment.a.y) * dy) /
            (segment.length * segment.length),
        0.0, 1.0);
    const double lateral =
        Distance(point, {segment.a.x + t * dx, segment.a.y + t * dy});
    if (lateral < nearest - kArrivalComparisonTolerance) {
      nearest = lateral;
      remaining = (include_lateral ? lateral : 0.0) +
                  (1.0 - t) * segment.length + segment.remaining;
    }
  }
  return remaining;
}

}  // namespace

const LocalGoalCandidate* LocalGoalRegion::At(const GridIndex cell) const noexcept {
  if (!geometry.Contains(cell)) {
    return nullptr;
  }
  const auto minimum = geometry.min_inclusive();
  const auto offset = static_cast<std::size_t>(cell.y - minimum.y) *
                          geometry.width() +
                      static_cast<std::size_t>(cell.x - minimum.x);
  if (offset >= candidate_by_cell.size() || candidate_by_cell[offset] < 0) {
    return nullptr;
  }
  return &candidates[static_cast<std::size_t>(candidate_by_cell[offset])];
}

bool LocalGoalRegion::HasForwardUnknownBoundaryAt(const GridIndex cell) const noexcept {
  if (!geometry.Contains(cell)) {
    return false;
  }
  const auto minimum = geometry.min_inclusive();
  const auto offset = static_cast<std::size_t>(cell.y - minimum.y) *
                          geometry.width() +
                      static_cast<std::size_t>(cell.x - minimum.x);
  return offset < forward_unknown_by_cell.size() &&
         forward_unknown_by_cell[offset] != 0U;
}

double LocalGoalRegion::LowerBound(const Point2 point) const noexcept {
  // Every terminal cost is at least its distance to the final point. Euclidean
  // distance is therefore a consistent lower bound for all regional goals.
  return Distance(point, final_point);
}

std::optional<LocalTarget> LocalTargetSelector::SelectRolling(
    const FineTraversabilitySnapshot& fine, const SparseGridGeometry& window,
    const Point2 start, const FinalGoal& goal,
    const std::optional<GlobalRoute>& guidance) const {
  return SelectRolling(fine, window, start, goal, guidance, SearchControl{});
}

std::optional<LocalTarget> LocalTargetSelector::SelectRolling(
    const FineTraversabilitySnapshot& fine, const SparseGridGeometry& window,
    const Point2 start, const FinalGoal& goal,
    const std::optional<GlobalRoute>& guidance, const SearchControl& control) const {
  if (Interrupted(control)) return std::nullopt;
  if (!IsLocalPlanningWindowFor(window, fine.geometry()) ||
      !IsValidFinalGoal(goal) || !std::isfinite(start.x) ||
      !std::isfinite(start.y) ||
      window.width() > kMaximumLocalPlanningWindowAxisCells ||
      window.height() > kMaximumLocalPlanningWindowAxisCells) {
    return std::nullopt;
  }
  auto region = std::make_shared<LocalGoalRegion>();
  region->geometry = window;
  region->final_point = {goal.target_x_m, goal.target_y_m};
  const auto& fine_geometry = fine.geometry();
  const auto fine_min = fine_geometry.min_inclusive();
  const auto fine_max = fine_geometry.max_exclusive();
  const double final_x = (goal.target_x_m - fine_geometry.origin_m().x) /
                         fine_geometry.resolution_m();
  const double final_y = (goal.target_y_m - fine_geometry.origin_m().y) /
                         fine_geometry.resolution_m();
  region->has_unknown_boundary =
      final_x < static_cast<double>(fine_min.x) ||
      final_x >= static_cast<double>(fine_max.x) ||
      final_y < static_cast<double>(fine_min.y) ||
      final_y >= static_cast<double>(fine_max.y);
  const bool final_outside_fine = region->has_unknown_boundary;
  region->candidate_by_cell.assign(window.CellCount(), -1);
  region->forward_unknown_by_cell.assign(window.CellCount(), 0U);

  std::vector<Point2> points;
  if (guidance) {
    for (const auto& pose : guidance->poses_map) {
      if (Interrupted(control)) return std::nullopt;
      const Point2 point{pose.position_m.x, pose.position_m.y};
      if (std::isfinite(point.x) && std::isfinite(point.y)) {
        points.push_back(point);
      }
    }
  }
  if (points.empty()) {
    points.push_back(start);
  }
  if (points.back() != region->final_point) {
    points.push_back(region->final_point);
  }
  std::vector<RouteSegment> route;
  double remaining = 0.0;
  for (std::size_t i = points.size(); i > 1U; --i) {
    const double length = Distance(points[i - 2U], points[i - 1U]);
    if (!std::isfinite(length) || !std::isfinite(remaining + length)) return std::nullopt;
    if (length > kArrivalComparisonTolerance) {
      route.push_back({points[i - 2U], points[i - 1U], length, remaining});
    }
    remaining += length;
  }
  std::reverse(route.begin(), route.end());
  const double start_remaining = Remaining(start, route, region->final_point);
  const double start_arc_remaining =
      Remaining(start, route, region->final_point, false);
  std::vector<RouteSegment> local_route;
  bool known_guidance = guidance.has_value() && !route.empty();
  if (known_guidance) {
    for (const auto& segment : route) {
      if (Interrupted(control)) return std::nullopt;
      if (const auto clipped = Clip(segment, window)) {
        if (!FreeConnection(clipped->a, clipped->b, fine, control)) {
          known_guidance = false;
          break;
        }
        local_route.push_back(*clipped);
      }
    }
    known_guidance &= !local_route.empty();
  }
  if (Interrupted(control)) return std::nullopt;
  const double connected_start = known_guidance
      ? ConnectedRemaining(start, local_route, fine, control) : start_remaining;
  known_guidance &= std::isfinite(connected_start);
  const double target_start_remaining = known_guidance ? connected_start : start_remaining;
  const double resolution = window.resolution_m();
  const double progress = std::max(0.5 * resolution, 1.e-6);
  const double diagonal = std::hypot(static_cast<double>(window.width()),
                                     static_cast<double>(window.height())) *
                          resolution;
  const auto minimum = window.min_inclusive();
  const auto maximum = window.max_exclusive();
  const auto append = [&](const GridIndex cell, LocalTarget target,
                           const double cost) {
    const auto offset = static_cast<std::size_t>(cell.y - minimum.y) *
                            window.width() +
                        static_cast<std::size_t>(cell.x - minimum.x);
    region->candidate_by_cell[offset] =
        static_cast<std::int32_t>(region->candidates.size());
    region->candidates.push_back({std::move(target), cost});
  };
  for (auto y = minimum.y; y < maximum.y; ++y) {
    if (Interrupted(control)) return std::nullopt;
    for (auto x = minimum.x; x < maximum.x; ++x) {
      const GridIndex cell{x, y};
      if (fine.State(cell) != FineCellState::kFree) {
        continue;
      }
      const Point2 center = Center(window, cell);
      bool unknown = false;
      std::array<Point2, 8U> unknown_neighbors;
      std::size_t unknown_neighbor_count = 0U;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          // Check the boundary before adding integer displacements. A valid
          // sparse geometry may begin at INT64_MIN on either axis.
          const bool outside_fine =
              (dx < 0 && x == fine_min.x) ||
              (dx > 0 && x == fine_max.x - 1) ||
              (dy < 0 && y == fine_min.y) ||
              (dy > 0 && y == fine_max.y - 1);
          const bool neighbor_unknown =
              !outside_fine &&
              fine.State({x + dx, y + dy}) == FineCellState::kUnknown;
          unknown |= neighbor_unknown;
          // A diagonal UNKNOWN touching through a blocked corner cannot
          // explain a missing executable continuation. Use shared cell edges
          // for waiting evidence; normal candidate eligibility stays unchanged.
          if ((dx == 0 || dy == 0) &&
              (neighbor_unknown || (outside_fine && final_outside_fine))) {
            // Out-of-extent evidence is meaningful only at a reachable map
            // edge, never merely because the requested goal lies outside.
            unknown_neighbors[unknown_neighbor_count++] = {
                center.x + static_cast<double>(dx) * resolution,
                center.y + static_cast<double>(dy) * resolution};
          }
        }
      }
      region->has_unknown_boundary |= unknown;
      if (unknown_neighbor_count != 0U) {
        const double cell_remaining = Remaining(center, route, region->final_point);
        const double cell_arc_remaining =
            Remaining(center, route, region->final_point, false);
        for (std::size_t i = 0U; i < unknown_neighbor_count; ++i) {
          const double neighbor_remaining =
              Remaining(unknown_neighbors[i], route, region->final_point);
          const double neighbor_arc_remaining =
              Remaining(unknown_neighbors[i], route, region->final_point, false);
          // Either advance along the route or approach it laterally. Counting
          // arc progress separately avoids cancellation of forward progress
          // by lateral distance for cardinal neighbors on a diagonal route.
          const bool advances_from_cell =
              !WithinArrivalTolerance(cell_remaining - neighbor_remaining, 0.0) ||
              !WithinArrivalTolerance(cell_arc_remaining - neighbor_arc_remaining, 0.0);
          const bool advances_from_start =
              !WithinArrivalTolerance(start_remaining - neighbor_remaining, 0.0) ||
              !WithinArrivalTolerance(start_arc_remaining - neighbor_arc_remaining, 0.0);
          if (advances_from_cell && advances_from_start) {
            const auto offset = static_cast<std::size_t>(y - minimum.y) *
                                    window.width() +
                                static_cast<std::size_t>(x - minimum.x);
            region->forward_unknown_by_cell[offset] = 1U;
            break;
          }
        }
      }
      const bool final_cell = static_cast<double>(x) == std::floor(final_x) &&
                              static_cast<double>(y) == std::floor(final_y);
      if (final_cell) {
        // Exact final target retains its continuous endpoint and yaw contract.
        append(cell,
               {.center = region->final_point,
                .position_tolerance_m = 0.0,
                .is_final_goal = true,
                .terminal_yaw_rad = goal.has_target_yaw
                                        ? std::optional<double>(goal.target_yaw_rad)
                                        : std::nullopt},
               0.0);
        continue;
      }
      const bool boundary = x == minimum.x || y == minimum.y ||
                            x == maximum.x - 1 || y == maximum.y - 1;
      if ((!boundary && !unknown) ||
          WithinArrivalTolerance(Distance(start, center), progress)) {
        continue;
      }
      const double rest = known_guidance
          ? ConnectedRemaining(center, local_route, fine, control)
          : Remaining(center, route, region->final_point);
      if (!std::isfinite(rest) ||
          WithinArrivalTolerance(target_start_remaining - rest, progress)) {
        continue;
      }
      // A doubled route remainder prefers useful forward motion over a nearby
      // frontier. Window exits outrank premature unknown frontiers at equal
      // route progress; final targets carry no intermediate stopping penalty.
      const double cost =
          std::max(Distance(center, region->final_point), 2.0 * rest) +
          diagonal + (boundary ? 0.0 : diagonal);
      append(cell,
             {.center = center, .position_tolerance_m = 0.0,
              .is_final_goal = false},
             cost);
    }
  }
  if (Interrupted(control)) return std::nullopt;
  return LocalTarget{.center = region->final_point,
                     .position_tolerance_m = 0.5 * resolution,
                     .is_final_goal = false,
                     .region = std::move(region)};
}

}  // namespace lunar::incremental_navigation
