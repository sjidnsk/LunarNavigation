#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <lunar_planning_msgs/srv/get_policy_map.hpp>

#include "lunar_incremental_navigation_core/fine_traversability_builder.hpp"
#include "lunar_incremental_navigation_core/request_local_start_patch.hpp"

namespace lunar::incremental_navigation_ros {

inline constexpr std::uint8_t kPolicyMapUnknown{0U};
inline constexpr std::uint8_t kPolicyMapFree{1U};
inline constexpr std::uint8_t kPolicyMapBlocked{2U};

[[nodiscard]] std::uint8_t EffectiveObservedState(
    lunar::incremental_navigation::IntrinsicCellState intrinsic,
    float center_elevation_m) noexcept;

struct PolicyMapExportInput final {
  std::shared_ptr<const lunar::incremental_navigation::FineTraversabilitySnapshot> fine;
  std::optional<lunar::incremental_navigation::Pose2> anchor;
  double local_window_size_m{};
  std::string epoch;
  std::int64_t processed_stamp_ns{};
  std::uint64_t base_revision{};
  bool full_snapshot{true};
  std::vector<lunar::incremental_navigation::TileIndex> dirty_tiles;
};

struct ObservedPolicyMapExportInput final {
  PolicyMapExportInput map;
  // 只用于本快照的源格保守分类；不改变原输入的公开布局。
  std::shared_ptr<const lunar::incremental_navigation::PlatformElevationEvaluator> intrinsic;
};

class PolicyMapExporter final {
 public:
  PolicyMapExporter(lunar::incremental_navigation::PlatformCapability capability,
                    lunar::incremental_navigation::TraversabilityProfile profile);

  [[nodiscard]] lunar_planning_msgs::srv::GetPolicyMap::Response Export(
      const PolicyMapExportInput& input, std::uint64_t since_revision,
      std::int64_t minimum_map_stamp_ns = 0) const;

  [[nodiscard]] lunar_planning_msgs::srv::GetPolicyMap::Response ExportObserved(
      const PolicyMapExportInput& input, std::uint64_t since_revision,
      std::int64_t minimum_map_stamp_ns = 0) const;

  [[nodiscard]] lunar_planning_msgs::srv::GetPolicyMap::Response ExportObserved(
      const ObservedPolicyMapExportInput& input, std::uint64_t since_revision,
      std::int64_t minimum_map_stamp_ns = 0) const;

 private:
  [[nodiscard]] lunar_planning_msgs::srv::GetPolicyMap::Response ExportImpl(
      const PolicyMapExportInput& input, std::uint64_t since_revision,
      std::int64_t minimum_map_stamp_ns, bool observed_only,
      const lunar::incremental_navigation::PlatformElevationEvaluator* intrinsic = nullptr) const;
  lunar::incremental_navigation::PlatformCapability capability_;
  lunar::incremental_navigation::TraversabilityProfile profile_;
};

}  // namespace lunar::incremental_navigation_ros
