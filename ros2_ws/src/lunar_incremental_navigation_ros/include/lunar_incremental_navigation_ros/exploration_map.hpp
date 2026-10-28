#pragma once
#include <memory>

#include "lunar_incremental_navigation_ros/exploration_map_projector.hpp"
#include "lunar_incremental_navigation_ros/map_adapters.hpp"
#include "lunar_incremental_navigation_ros/policy_map_exporter.hpp"
namespace lunar::incremental_navigation_ros {
struct ExplorationMapStorageStats final {
  std::size_t policy_cells{};
  std::size_t fragments{};
};
// 单一地图工作线程持有；Capture 返回独立的不可变快照。
class ExplorationMap final {
 public:
  ExplorationMap(lunar::incremental_navigation::PlatformCapability capability,
                 lunar::incremental_navigation::TraversabilityProfile profile,
                 double policy_resolution_m = .2,
                 double coarse_resolution_m = 1.);
  ~ExplorationMap();
  bool Apply(const OwnedElevationEvidence& source, std::int64_t stamp_ns);
  [[nodiscard]] ObservedPolicyMapExportInput Capture(
      std::uint64_t since_revision = 0) const;
  [[nodiscard]] ExplorationMapProjection Project() const;
  [[nodiscard]] ExplorationMapStorageStats StorageStats() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace lunar::incremental_navigation_ros
