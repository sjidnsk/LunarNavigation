#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include "local/grid_supercover.hpp"
#include "lunar_incremental_navigation_core/local_planning.hpp"

namespace lunar::incremental_navigation::local {
// Conservative segment cost used identically for original and replacement
// geometry. Prevent postprocessing from erasing the planner's soft-cost choice.
inline long double TerrainSegmentCost(const RequestLocalPlanningView& view,
                                     Vec3 a, Vec3 b,
                                     const SearchControl& control = {}) {
  long double maximum = 0.;
  const bool valid = VisitSupercoverCells(view.geometry(), a, b, [&](GridIndex cell) {
    if (control.canceled() || control.expired()) return false;
    maximum = std::max(maximum, static_cast<long double>(view.TraversalCost(cell)));
    return true;
  });
  if (!valid) return std::numeric_limits<long double>::infinity();
  return std::hypot(static_cast<long double>(b.x)-a.x,
                    static_cast<long double>(b.y)-a.y) * (1.L + maximum);
}
}  // namespace lunar::incremental_navigation::local
