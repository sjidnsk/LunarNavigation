#pragma once

#include <memory>
#include <string>

#include "lunar_incremental_navigation_core/adaptive_navigation_map.hpp"
#include "lunar_incremental_navigation_core/global_route_planner.hpp"

namespace lunar::incremental_navigation {

struct AdaptiveRouteMetrics final {
  std::size_t built_tiles{};
  std::size_t reused_tiles{};
  std::size_t regions{};
  std::size_t cached_bytes{};
  double build_ms{};
  double search_ms{};
  double length_m{};
  double cost{};
};
struct AdaptiveRouteResult final {
  GlobalRouteResult route;
  AdaptiveRouteMetrics metrics;
  // NO_KNOWN_ROUTE and MAP_INSUFFICIENT describe navigation, never exploration.
  std::string reason;
};
struct AdaptiveRoutePlannerConfig final {
  NowFn now{[] { return SteadyClock::now(); }};
};
class AdaptiveRoutePlanner final {
public:
  explicit AdaptiveRoutePlanner(AdaptiveRoutePlannerConfig config = {});
  ~AdaptiveRoutePlanner();
  AdaptiveRoutePlanner(AdaptiveRoutePlanner &&) noexcept;
  AdaptiveRoutePlanner &operator=(AdaptiveRoutePlanner &&) noexcept;
  AdaptiveRoutePlanner(const AdaptiveRoutePlanner &) = delete;
  AdaptiveRoutePlanner &operator=(const AdaptiveRoutePlanner &) = delete;
  [[nodiscard]] AdaptiveRouteResult Plan(const AdaptiveNavigationSnapshot &,
                                         Point2, Point2, SearchDeadline,
                                         const StopToken &);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace lunar::incremental_navigation
