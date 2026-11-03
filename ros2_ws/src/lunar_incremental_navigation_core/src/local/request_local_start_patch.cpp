#include "lunar_incremental_navigation_core/request_local_start_patch.hpp"

#include "lunar_incremental_navigation_core/local_planning_window.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

namespace lunar::incremental_navigation {
namespace {

[[nodiscard]] std::optional<GridIndex> WorldToCell(
    const SparseGridGeometry& geometry, const Point2 point) noexcept {
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
    return std::nullopt;
  }
  const Vec3 origin = geometry.origin_m();
  const long double x_cells =
      (static_cast<long double>(point.x) - origin.x) /
      geometry.resolution_m();
  const long double y_cells =
      (static_cast<long double>(point.y) - origin.y) /
      geometry.resolution_m();
  const long double floored_x = std::floor(x_cells);
  const long double floored_y = std::floor(y_cells);
  if (!std::isfinite(x_cells) || !std::isfinite(y_cells) ||
      floored_x < static_cast<long double>(geometry.min_inclusive().x) ||
      floored_y < static_cast<long double>(geometry.min_inclusive().y) ||
      floored_x >= static_cast<long double>(geometry.max_exclusive().x) ||
      floored_y >= static_cast<long double>(geometry.max_exclusive().y)) {
    return std::nullopt;
  }
  const GridIndex result{.x = static_cast<std::int64_t>(floored_x),
                         .y = static_cast<std::int64_t>(floored_y)};
  return geometry.Contains(result) ? std::optional<GridIndex>(result)
                                   : std::nullopt;
}

}  // namespace

StartPatchResult RequestLocalStartPatchBuilder::Build(
    std::shared_ptr<const FineTraversabilitySnapshot> fine, const Pose2& p0,
    const PlatformCapability& capability,
    const TraversabilityProfile& profile) const {
  if (!fine) {
    throw std::invalid_argument("start patch requires a fine snapshot");
  }
  return Build(fine, fine->geometry(), p0, capability, profile);
}

StartPatchResult RequestLocalStartPatchBuilder::Build(
    std::shared_ptr<const FineTraversabilitySnapshot> fine,
    const SparseGridGeometry& local_window, const Pose2& p0,
    const PlatformCapability& /*capability*/,
    const TraversabilityProfile& /*profile*/) const {
  if (!fine) {
    throw std::invalid_argument("start patch requires a fine snapshot");
  }
  if (!IsLocalPlanningWindowFor(local_window, fine->geometry())) {
    throw std::invalid_argument(
        "start patch requires a local window on the base lattice");
  }
  if (!std::isfinite(p0.position_m.x) || !std::isfinite(p0.position_m.y) ||
      !std::isfinite(p0.yaw_rad)) {
    throw std::invalid_argument(
        "start view pose must be finite");
  }
  const double patch_radius_m = fine->hard_inflation_radius_m();
  const std::optional<GridIndex> start =
      WorldToCell(local_window, p0.position_m);
  if (!start) {
    return {.status = StartPatchResult::Status::kUnresolved};
  }

  if (fine->State(*start) == FineCellState::kBlocked) {
    return {.status = StartPatchResult::Status::kStartBlocked};
  }
  if (fine->State(*start) == FineCellState::kFree) {
    return {
        .status = StartPatchResult::Status::kNotNeeded,
        .view = std::make_shared<const RequestLocalPlanningView>(
            fine, local_window, p0, patch_radius_m,
            std::vector<LocalCellOverride>{}),
    };
  }

  // 未知地形保留原状态；起点视图不推断高程或补为自由空间。
  return {.status = StartPatchResult::Status::kUnresolved};
}

StartPatchResult RequestLocalStartPatchBuilder::BuildStationary(
    std::shared_ptr<const FineTraversabilitySnapshot> fine, const Pose2& p0,
    const PlatformCapability& capability, const TraversabilityProfile& profile) const {
  if (!fine || !std::isfinite(p0.yaw_rad))
    throw std::invalid_argument("invalid stationary support request");
  const auto* wheel = std::get_if<WheeledCapability>(&capability);
  if (!wheel || wheel->maximum_spin_rate_radps <= 0.0)
    return {.status = StartPatchResult::Status::kUnresolved};
  const auto support = Build(fine, p0, capability, profile);
  if (!support.view) return support;
  return {.status = StartPatchResult::Status::kReady,
          .view = std::make_shared<const RequestLocalPlanningView>(
              fine, fine->geometry(), p0, support.view->start_patch_radius_m(),
              std::vector<LocalCellOverride>{}, true)};
}

StartConnectionsResult RequestLocalStartPatchBuilder::BuildStartConnections(
    std::shared_ptr<const FineTraversabilitySnapshot> fine,
    const SparseGridGeometry& local_window, const Pose2& p0,
    const PlatformCapability& capability,
    const TraversabilityProfile& profile) const {
  const StartPatchResult patch = Build(fine, local_window, p0, capability, profile);
  StartConnectionsResult result{.status = patch.status};
  if (!patch.view || (patch.status != StartPatchResult::Status::kReady &&
                      patch.status != StartPatchResult::Status::kNotNeeded)) {
    return result;
  }
  const std::optional<GridIndex> start = WorldToCell(local_window, p0.position_m);
  if (!start) return result;
  result.connections.push_back({.index = *start, .phase = StartPhase::kNormal});
  return result;
}

}  // namespace lunar::incremental_navigation
