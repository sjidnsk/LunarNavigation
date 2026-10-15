#pragma once

#include "lunar_incremental_navigation_core/search_control.hpp"
#include "lunar_incremental_navigation_core/traversability_snapshot.hpp"
#include <array>
#include <map>
#include <memory>

namespace lunar::incremental_navigation {
struct AdaptiveNavigationRegion final {
  std::int64_t x{}, y{};
  std::int32_t size{};
  std::array<std::int32_t, 4> children{{-1, -1, -1, -1}};
  double cost{}, minimum_cost{};
  bool free{};
  bool unknown{true};
  bool leaf{true};
  [[nodiscard]] GridIndex Anchor() const {
    return {x + (size - 1) / 2, y + (size - 1) / 2};
  }
};
struct AdaptiveMapMetrics final {
  std::size_t built_tiles{}, reused_tiles{}, regions{}, cached_bytes{};
  double build_ms{};
};
struct AdaptiveNavigationTile;
// A read-only derivative of exactly one fine snapshot. Contains no goal,
// robot location, exploration information or mutable search records.
class AdaptiveNavigationSnapshot final {
public:
  [[nodiscard]] const FineTraversabilitySnapshot &fine() const {
    return *fine_;
  }
  [[nodiscard]] std::shared_ptr<const FineTraversabilitySnapshot>
  fine_ptr() const {
    return fine_;
  }
  [[nodiscard]] const AdaptiveNavigationRegion *RegionAt(GridIndex cell) const;
  [[nodiscard]] const AdaptiveMapMetrics &metrics() const { return metrics_; }

private:
  friend class AdaptiveNavigationMapBuilder;
  std::shared_ptr<const FineTraversabilitySnapshot> fine_;
  std::map<TileIndex, std::shared_ptr<const AdaptiveNavigationTile>> tiles_;
  AdaptiveMapMetrics metrics_;
};
struct AdaptiveMapBuildResult final {
  enum class Status { kReady, kTimeout, kCanceled };
  Status status{Status::kTimeout};
  std::shared_ptr<const AdaptiveNavigationSnapshot> snapshot;
  AdaptiveMapMetrics metrics;
};
class AdaptiveNavigationMapBuilder final {
public:
  explicit AdaptiveNavigationMapBuilder(
      NowFn now = [] { return SteadyClock::now(); },
      double maximum_cost_spread = .05);
  ~AdaptiveNavigationMapBuilder();
  AdaptiveNavigationMapBuilder(const AdaptiveNavigationMapBuilder &) = delete;
  AdaptiveNavigationMapBuilder &
  operator=(const AdaptiveNavigationMapBuilder &) = delete;
  [[nodiscard]] AdaptiveMapBuildResult
  Update(std::shared_ptr<const FineTraversabilitySnapshot> fine,
         SearchDeadline deadline, const StopToken &stop);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace lunar::incremental_navigation
