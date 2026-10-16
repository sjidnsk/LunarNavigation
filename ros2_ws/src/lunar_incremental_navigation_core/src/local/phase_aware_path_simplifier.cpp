#include "lunar_incremental_navigation_core/wheel_local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "local/grid_supercover.hpp"
#include "local/terrain_segment_cost.hpp"
#include "local/boundary_distance_index.hpp"

namespace lunar::incremental_navigation {
namespace {

[[nodiscard]] bool Interrupted(const SearchControl& control) {
  return control.canceled() || control.expired();
}

[[nodiscard]] bool Allowed(const RequestLocalPlanningView& view,
                           const GridIndex index, const StartPhase phase) {
  return view.Source(index) ==
         (phase == StartPhase::kStartPrefix
              ? LocalCellSource::kStartAssumedFree
              : LocalCellSource::kEvidenceFree);
}

// Local cell coordinates keep the distance comparison independent of world
// offsets and resolution. No physical clearance threshold is introduced.
using Point = local::BoundaryPoint;
using Segment = local::BoundarySegment;

// Per-call geometry of the current phase's allowed-cell union, including the
// local-window boundary. This does not modify maps or their source semantics.
class PhaseGeometry final {
 public:
  PhaseGeometry(const RequestLocalPlanningView& view, StartPhase phase,
                const SearchControl& control)
      : view_(view), phase_(phase), control_(control) {}

  [[nodiscard]] bool Build() {
    const auto& geometry = view_.geometry();
    const auto lo = geometry.min_inclusive();
    const std::size_t width = geometry.width(), height = geometry.height();
    allowed_.assign(geometry.CellCount(), 0);
    auto& allowed = allowed_;
    for (std::size_t y = 0; y < height; ++y) {
      if (Interrupted(control_)) return false;
      for (std::size_t x = 0; x < width; ++x) {
        allowed[y * width + x] = Allowed(
            view_, {lo.x + static_cast<std::int64_t>(x),
                    lo.y + static_cast<std::int64_t>(y)}, phase_);
      }
    }
    // Merge consecutive exposed cell faces into exact boundary segments.
    for (std::size_t y = 0; y <= height; ++y) {
      if (Interrupted(control_)) return false;
      std::size_t start = 0;
      bool active = false;
      for (std::size_t x = 0; x <= width; ++x) {
        const bool face = x < width &&
            ((y > 0 && allowed[(y - 1) * width + x]) !=
             (y < height && allowed[y * width + x]));
        if (face && !active) {
          start = x;
          active = true;
        }
        if (!face && active) {
          boundary_.push_back(
              {{static_cast<long double>(start), static_cast<long double>(y)},
               {static_cast<long double>(x), static_cast<long double>(y)}});
          active = false;
        }
      }
    }
    for (std::size_t x = 0; x <= width; ++x) {
      if (Interrupted(control_)) return false;
      std::size_t start = 0;
      bool active = false;
      for (std::size_t y = 0; y <= height; ++y) {
        const bool face = y < height &&
            ((x > 0 && allowed[y * width + x - 1]) !=
             (x < width && allowed[y * width + x]));
        if (face && !active) {
          start = y;
          active = true;
        }
        if (!face && active) {
          boundary_.push_back(
              {{static_cast<long double>(x), static_cast<long double>(start)},
               {static_cast<long double>(x), static_cast<long double>(y)}});
          active = false;
        }
      }
    }
    return boundary_index_.Build(std::move(boundary_), control_);
  }

  [[nodiscard]] bool Legal(const PathPoint& a, const PathPoint& b) const {
    return !Interrupted(control_) && view_.CertifySegment(
        a.pose.position_m, b.pose.position_m, a.phase, b.phase, &control_) &&
        !Interrupted(control_);
  }

  // Negative means illegal/interrupted. Original and replacement segments use
  // exactly the same evaluation; there are no shortcut boundary exceptions.
  [[nodiscard]] long double Evaluate(const PathPoint& a,
                                     const PathPoint& b) const {
    if (!Legal(a, b)) return -1;
    const Segment line{Convert(a.pose.position_m), Convert(b.pose.position_m)};
    const auto nearest = boundary_index_.Query(line, control_);
    return nearest.value_or(-1);
  }

 private:
  [[nodiscard]] Point Convert(Vec3 p) const {
    const auto& geometry = view_.geometry();
    const auto origin = geometry.origin_m();
    const auto lo = geometry.min_inclusive();
    return {(static_cast<long double>(p.x) - origin.x) /
                geometry.resolution_m() - lo.x,
            (static_cast<long double>(p.y) - origin.y) /
                geometry.resolution_m() - lo.y};
  }

  const RequestLocalPlanningView& view_;
  StartPhase phase_;
  const SearchControl& control_;
  std::vector<Segment> boundary_;
  std::vector<unsigned char> allowed_;
  local::BoundaryDistanceIndex boundary_index_;
};

[[nodiscard]] bool SameDirection(const PathPoint& a, const PathPoint& b,
                                 const PathPoint& c) {
  const long double dx = static_cast<long double>(b.pose.position_m.x) - a.pose.position_m.x;
  const long double dy = static_cast<long double>(b.pose.position_m.y) - a.pose.position_m.y;
  const long double ex = static_cast<long double>(c.pose.position_m.x) - b.pose.position_m.x;
  const long double ey = static_cast<long double>(c.pose.position_m.y) - b.pose.position_m.y;
  // Exact same-direction collinearity changes sampling, not geometry.
  return dx * ey == dy * ex && dx * ex + dy * ey > 0;
}

[[nodiscard]] bool SimplifyPhase(
    const RequestLocalPlanningView& view, std::span<const PathPoint> raw,
    std::vector<PathPoint>& output, const SearchControl& control) {
  std::vector<PathPoint> path;
  path.reserve(raw.size());
  for (const auto& point : raw) {
    if (Interrupted(control)) return false;
    if (path.size() >= 2 &&
        SameDirection(path[path.size() - 2], path.back(), point)) {
      path.back() = point;
    } else {
      path.push_back(point);
    }
  }
  PhaseGeometry geometry(view, raw.front().phase, control);
  if (path.size() < 3) {
    if (path.size() == 1 && !geometry.Legal(path[0], path[0])) return false;
    if (path.size() == 2 && !geometry.Legal(path[0], path[1])) return false;
    output.insert(output.end(), path.begin(), path.end());
    return true;
  }
  if (!geometry.Build()) return false;
  std::vector<long double> clearance(path.size() - 1), minimum(path.size());
  for (std::size_t i = 0; i < clearance.size(); ++i) {
    clearance[i] = geometry.Evaluate(path[i], path[i + 1]);
    if (clearance[i] < 0) return false;
  }
  std::vector<long double> cost_prefix(path.size(), 0.L);
  for (std::size_t i=1;i<path.size();++i)
    cost_prefix[i]=cost_prefix[i-1]+local::TerrainSegmentCost(
        view,path[i-1].pose.position_m,path[i].pose.position_m,control);
  output.push_back(path.front());
  std::size_t anchor = 0;
  while (anchor + 1 < path.size()) {
    if (Interrupted(control)) return false;
    minimum[anchor] = std::numeric_limits<long double>::infinity();
    for (std::size_t j = anchor + 1; j < path.size(); ++j) {
      minimum[j] = std::min(minimum[j - 1], clearance[j - 1]);
    }
    std::size_t selected = anchor + 1;
    for (std::size_t j = path.size() - 1; j > anchor + 1; --j) {
      const long double candidate = geometry.Evaluate(path[anchor], path[j]);
      if (Interrupted(control)) return false;
      // Numerical roundoff in squared cell units, not a physical margin.
      // Reference values always come from the original geometry, so repeated
      // shortcuts cannot accumulate a sequence of tolerance relaxations.
      const long double tolerance = 64 * std::numeric_limits<double>::epsilon() *
                                    std::max(1.0L, minimum[j]);
      const long double shortcut_cost=local::TerrainSegmentCost(
          view,path[anchor].pose.position_m,path[j].pose.position_m,control);
      const long double original_cost=cost_prefix[j]-cost_prefix[anchor];
      if (candidate >= 0 && candidate + tolerance >= minimum[j] &&
          shortcut_cost <= original_cost + 1e-12L*std::max(1.L,original_cost)) {
        selected = j;
        break;
      }
    }
    output.push_back(path[selected]);
    anchor = selected;
  }
  return true;
}

}  // namespace

std::vector<PathPoint> SimplifyPhaseAwarePath(
    const RequestLocalPlanningView& view, std::span<const PathPoint> raw,
    const SearchControl& control) {
  if (Interrupted(control)) return {};
  // Same-phase runs are checked after collinear compression. Phase-switch
  // edges are never compressed, and short paths need the same certification.
  if (raw.size() < 3) {
    for (std::size_t i = 0; i < raw.size(); ++i) {
      const auto& from = raw[i == 0 ? 0 : i - 1];
      if (!view.CertifySegment(from.pose.position_m, raw[i].pose.position_m,
                               from.phase, raw[i].phase, &control)) return {};
    }
    return {raw.begin(), raw.end()};
  }
  std::vector<PathPoint> output;
  output.reserve(raw.size());
  for (std::size_t begin = 0; begin < raw.size();) {
    if (Interrupted(control)) return {};
    if (begin > 0 && !view.CertifySegment(
            raw[begin - 1].pose.position_m, raw[begin].pose.position_m,
            raw[begin - 1].phase, raw[begin].phase, &control)) return {};
    std::size_t end = begin + 1;
    while (end < raw.size() && raw[end].phase == raw[begin].phase) ++end;
    if (!SimplifyPhase(view, raw.subspan(begin, end - begin), output, control)) {
      return {};
    }
    begin = end;
  }
  return output;
}

std::vector<PathPoint> SimplifyPhaseAwarePath(
    const RequestLocalPlanningView& view, std::span<const PathPoint> raw) {
  return SimplifyPhaseAwarePath(
      view, raw, SearchControl{.deadline = SteadyClock::time_point::max()});
}

}  // namespace lunar::incremental_navigation
