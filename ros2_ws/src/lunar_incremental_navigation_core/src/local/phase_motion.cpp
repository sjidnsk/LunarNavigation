#include "lunar_incremental_navigation_core/local_planning.hpp"

#include <limits>

#include "local/grid_supercover.hpp"

namespace lunar::incremental_navigation {
namespace {
// Exact squared distance between a closed segment and a unit cell area.
long double SegmentCellDistanceSquared(long double ax, long double ay,
                                      long double bx, long double by) {
  const long double dx=bx-ax, dy=by-ay;
  long double entry=0., exit=1.;
  const auto clip=[&](long double start,long double delta) {
    if (delta==0.) return start>=0. && start<=1.;
    auto lo=-start/delta, hi=(1.-start)/delta;
    if (lo>hi) std::swap(lo,hi);
    entry=std::max(entry,lo);exit=std::min(exit,hi);
    return entry<=exit;
  };
  if (clip(ax,dx) && clip(ay,dy)) return 0.;
  const auto point_box=[](long double x,long double y) {
    const auto ex=std::max({-x,0.L,x-1.L});
    const auto ey=std::max({-y,0.L,y-1.L});
    return ex*ex+ey*ey;
  };
  long double best=std::min(point_box(ax,ay),point_box(bx,by));
  const long double length=dx*dx+dy*dy;
  for (const long double x : {0.L,1.L})
    for (const long double y : {0.L,1.L}) {
      const auto t=length>0. ? std::clamp(((x-ax)*dx+(y-ay)*dy)/length,0.L,1.L) : 0.L;
      const auto ex=x-ax-t*dx,ey=y-ay-t*dy;
      best=std::min(best,ex*ex+ey*ey);
    }
  return best;
}
}  // namespace


bool RequestLocalPlanningView::CertifySegment(
    const Vec3 from, const Vec3 to, const StartPhase initial,
    const StartPhase final, const SearchControl* control) const noexcept {
  return CertifySegmentWithClearance(from,to,initial,final,0.,control);
}

bool RequestLocalPlanningView::CertifySegmentWithClearance(
    const Vec3 from,const Vec3 to,const StartPhase initial,
    const StartPhase final,const double margin_m,
    const SearchControl* control) const noexcept {
  if (!std::isfinite(margin_m) || margin_m<0.) return false;
  std::size_t visited = 0;
  const auto interrupted = [&] {
    return control != nullptr && (control->canceled() || control->expired());
  };
  if (interrupted()) return false;
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
  const long double radius=margin_m/geometry_.resolution_m();
  if (!std::isfinite(radius) || radius>static_cast<long double>(
          std::max(geometry_.width(),geometry_.height()))) return false;
  const auto reach=static_cast<std::int64_t>(std::ceil(radius));
  const auto origin=geometry_.origin_m();
  const long double ax=(static_cast<long double>(from.x)-origin.x)/geometry_.resolution_m();
  const long double ay=(static_cast<long double>(from.y)-origin.y)/geometry_.resolution_m();
  const long double bx=(static_cast<long double>(to.x)-origin.x)/geometry_.resolution_m();
  const long double by=(static_cast<long double>(to.y)-origin.y)/geometry_.resolution_m();
  const auto clear=[&](GridIndex center) {
    if (margin_m==0.) return true;
    const auto low=std::numeric_limits<std::int64_t>::min();
    const auto high=std::numeric_limits<std::int64_t>::max();
    if (center.x<low+reach || center.y<low+reach ||
        center.x>high-reach || center.y>high-reach) return false;
    for (std::int64_t y=-reach;y<=reach;++y)
      for (std::int64_t x=-reach;x<=reach;++x) {
        if (visited++%64==0 && interrupted()) return false;
        const GridIndex cell{center.x+x,center.y+y};
        const auto source=Source(cell);
        if (source==LocalCellSource::kEvidenceFree ||
            source==LocalCellSource::kStartAssumedFree) continue;
        const auto distance=SegmentCellDistanceSquared(
            ax-cell.x,ay-cell.y,bx-cell.x,by-cell.y);
        const auto tolerance=64*std::numeric_limits<double>::epsilon()*std::max(1.L,radius*radius);
        if (distance+tolerance<radius*radius) return false;
      }
    return true;
  };
  if (initial == final) {
    return local::VisitSupercoverCells(geometry_, from, to, [&](GridIndex cell) {
      return (visited++ % 64 != 0 || !interrupted()) &&
             matches(Source(cell), initial) && clear(cell);
    });
  }

  // The supercover enumerates contacts in a canonical geometric order, not
  // travel order. Compare clipped parameter intervals instead of relying on
  // callback order. Zero-length corner contacts are included in the switch.
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
        if (visited++ % 64 == 0 && interrupted()) return false;
        if (!clear(cell)) return false;
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
