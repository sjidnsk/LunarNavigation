#pragma once
#include <memory>
#include <cstdint>
#include <string>
#include <vector>

#include "lunar_incremental_navigation_ros/exploration_map_projector.hpp"
#include "lunar_incremental_navigation_ros/map_adapters.hpp"
#include "lunar_incremental_navigation_ros/policy_map_exporter.hpp"
namespace lunar::incremental_navigation_ros {
struct ExplorationMapStorageStats final {
  std::size_t policy_cells{};
  std::size_t fragments{};
  std::size_t facts_aabb_checks{}, facts_overlap_candidates{};
};
// 世界足迹必须是互不重叠的凸多边形；state来自最新观测，不能用导航历史补全。
// height必须有限；测量不足时slope/relief可为NaN，不能用0冒充完整观测。
struct ExplorationObservationFragment final {
  std::vector<lunar::incremental_navigation::Point2> polygon;
  float height_m{};
  double slope_rad{}, relief_m{};
  lunar::incremental_navigation::IntrinsicCellState state{
      lunar::incremental_navigation::IntrinsicCellState::kUnknown};
};
// epoch+raw_revision标识不可变的完整已提交事实；同epoch的frame固定、revision单调。
// 缺少的足迹被撤销；完整空快照清空派生图。不完整快照原子拒绝。
struct ExplorationMapFacts final {
  double source_resolution_m{};
  std::uint64_t raw_revision{};
  std::string frame, epoch;
  bool complete_snapshot{false};
  std::vector<ExplorationObservationFragment> observation_fragments;
};
struct PublishedExplorationMap final {
  lunar_planning_msgs::srv::GetPolicyMap::Response response;
  std::uint64_t minimum_compatible_revision{};
};
[[nodiscard]] lunar_planning_msgs::srv::GetPolicyMap::Response SelectExplorationMapResponse(
    const PublishedExplorationMap& snapshot, std::uint64_t since_revision,
    std::int64_t minimum_map_stamp_ns = 0);
// worker发布与服务复用同一纯选择逻辑；空完整快照也发布明确的不可用响应。
[[nodiscard]] PublishedExplorationMap BuildExplorationMapPublication(
    const ObservedPolicyMapExportInput& input, const PublishedExplorationMap* previous,
    const PolicyMapExporter& exporter, std::uint64_t minimum_compatible_revision,
    const std::string& empty_frame);
// 单一地图工作线程持有；Capture 返回独立的不可变快照。
class ExplorationMap final {
 public:
  ExplorationMap(lunar::incremental_navigation::PlatformCapability capability,
                 lunar::incremental_navigation::TraversabilityProfile profile,
                 double policy_resolution_m = 0.,
                 double coarse_resolution_m = 0.);
  ~ExplorationMap();
  bool Apply(const OwnedElevationEvidence& source, std::int64_t stamp_ns);
  // 进程内已提交事实的纯派生入口；完整快照替换旧证据，不重新评价源物理。
  bool ApplyFacts(const ExplorationMapFacts& facts, std::int64_t stamp_ns);
  // 来源身份独立于内部派生高程存储的raw_elevation_revision。
  [[nodiscard]] std::string SourceEpoch() const;
  [[nodiscard]] std::uint64_t SourceRawRevision() const;
  [[nodiscard]] ObservedPolicyMapExportInput Capture(
      std::uint64_t since_revision = 0) const;
  [[nodiscard]] ExplorationMapProjection Project() const;
  [[nodiscard]] ExplorationMapStorageStats StorageStats() const;
  [[nodiscard]] std::uint64_t MinimumCompatibleRevision() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace lunar::incremental_navigation_ros
