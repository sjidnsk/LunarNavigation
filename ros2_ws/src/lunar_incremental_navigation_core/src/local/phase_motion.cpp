#include "lunar_incremental_navigation_core/local_planning.hpp"

#include <limits>

#include "local/grid_supercover.hpp"

namespace lunar::incremental_navigation {

bool RequestLocalPlanningView::CertifySegment(
    const Vec3 from, const Vec3 to, const StartPhase initial,
    const StartPhase final) const noexcept {
  if (initial == StartPhase::kNormal && final != initial) return false;
  const auto start = local::SupercoverWorldToCell(geometry_, from);
  const auto end = local::SupercoverWorldToCell(geometry_, to);
  if (!start || !end) return false;
  const auto matches = [](LocalCellSource source, StartPhase phase) {
    return source == (phase == StartPhase::kStartPrefix
                          ? LocalCellSource::kStartAssumedFree
                          : LocalCellSource::kEvidenceFree);
  };
  if (!matches(Source(*start), initial) || !matches(Source(*end), final)) {
    return false;
  }
  if (initial == final) {
    return local::VisitSupercoverCells(geometry_, from, to, [&](GridIndex cell) {
      return matches(Source(cell), initial);
    });
  }

  // The supercover enumerates contacts in a canonical geometric order, not
  // travel order. Compare clipped parameter intervals instead of relying on
  // callback order. Zero-length corner contacts are included in the switch.
  const auto origin = geometry_.origin_m();
  const long double resolution = geometry_.resolution_m();
  const long double x0 = (static_cast<long double>(from.x) - origin.x) / resolution;
  const long double y0 = (static_cast<long double>(from.y) - origin.y) / resolution;
  const long double dx = (static_cast<long double>(to.x) - from.x) / resolution;
  const long double dy = (static_cast<long double>(to.y) - from.y) / resolution;
  const long double tolerance = 64 * std::numeric_limits<double>::epsilon() *
      std::max({1.0L, std::abs(x0), std::abs(y0),
                std::abs(x0 + dx), std::abs(y0 + dy)});
  long double last_assumed = 0.0L;
  long double first_evidence = 1.0L;
  const bool covered = local::VisitSupercoverCells(
      geometry_, from, to, [&](GridIndex cell) {
        const auto source = Source(cell);
        if (source != LocalCellSource::kStartAssumedFree &&
            source != LocalCellSource::kEvidenceFree) return false;
        long double entry = 0.0L, exit = 1.0L;
        const auto clip = [&](long double p, long double delta,
                              long double low) {
          if (delta == 0.0L) return;
          auto a = (low - p) / delta;
          auto b = (low + 1.0L - p) / delta;
          if (a > b) std::swap(a, b);
          entry = std::max(entry, a);
          exit = std::min(exit, b);
        };
        clip(x0, dx, cell.x);
        clip(y0, dy, cell.y);
        // Near-corner cells are deliberately included by the supercover.
        // Clamp their numerical contact to the segment parameter range.
        entry = std::clamp(entry, 0.0L, 1.0L);
        exit = std::clamp(exit, 0.0L, 1.0L);
        if (source == LocalCellSource::kStartAssumedFree) {
          last_assumed = std::max(last_assumed, exit);
        } else {
          first_evidence = std::min(first_evidence, entry);
        }
        return true;
      });
  const long double parameter_tolerance =
      tolerance / std::max({1.0L, std::abs(dx), std::abs(dy)});
  return covered && last_assumed <= first_evidence + parameter_tolerance;
}

std::optional<StartPhase> RequestLocalPlanningView::AdvanceGridStep(
    const StartPhase phase, const GridIndex current,
    const GridIndex next) const noexcept {
  const long double dx = static_cast<long double>(next.x) - current.x;
  const long double dy = static_cast<long double>(next.y) - current.y;
  if ((dx == 0.0L && dy == 0.0L) || std::abs(dx) > 1.0L || std::abs(dy) > 1.0L) {
    return std::nullopt;
  }
  const auto next_phase = AdvancePhase(phase, next);
  const auto center = [&](GridIndex index) {
    const auto origin = geometry_.origin_m();
    return Vec3{.x = std::fma(static_cast<double>(index.x) + 0.5,
                            geometry_.resolution_m(), origin.x),
                .y = std::fma(static_cast<double>(index.y) + 0.5,
                            geometry_.resolution_m(), origin.y)};
  };
  return next_phase && CertifySegment(center(current), center(next), phase, *next_phase)
             ? next_phase : std::nullopt;
}

}  // namespace lunar::incremental_navigation
