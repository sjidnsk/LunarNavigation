#include "lunar_incremental_navigation_core/planning_cycle.hpp"
#include <algorithm>
#include <cmath>
namespace lunar::incremental_navigation {
namespace {
double GoalPositionTolerance(const FineTraversabilitySnapshot& fine, double tolerance) {
  return std::max(.5 * fine.geometry().resolution_m(), tolerance);
}
}
CycleCandidate ComputeCycle(const CycleRequest& request, PlanningSessionPorts& ports,
                            const NowFn& clock, const StopToken& stop) {
  CycleCandidate output;
  output.request_id = request.request_id;
  output.session_id = request.session_id;
  const auto now = [&] { return clock ? clock() : SteadyClock::now(); };
  const auto& state = request.state;
  const auto& snapshots = request.snapshots;
  const auto deadline = request.deadline;
  const Point2 start = state.base_link_pose.position_m;
  const Point2 final_goal{.x = request.goal.target_x_m,
                          .y = request.goal.target_y_m};
  std::optional<GlobalRoute> guidance_route;
  output.guidance_status = GuidanceStatus::kUnavailable;
  if (snapshots.guidance && ports.plan_global) {
    try {
      const SearchDeadline global_deadline = std::min(
          deadline, now() + request.global_subdeadline);
      GlobalRouteResult global = ports.plan_global(
          *snapshots.guidance, start, final_goal, global_deadline,
          stop);
      output.guidance_status = global.status;
      if (global.status == GuidanceStatus::kAvailable && global.route) {
        guidance_route = std::move(global.route);
        output.global_route = guidance_route;
      }
    } catch (...) {
      output.guidance_status = GuidanceStatus::kUnavailable;
    }
  }
  if (stop.stop_requested()) {
    output.stopped = true;
    return output;
  }

  if (!ports.select_target || !ports.build_start_patch) {
    output.failure = SessionOutcome::kInternalError;
    output.reason_code = "INTERNAL_ERROR";
    return output;
  }

  const LocalPlanningWindowResult local_window = BuildLocalPlanningWindow(
      *snapshots.fine, start, request.local_window_size_m);
  if (!local_window.geometry) {
    const std::string reason =
        local_window.status == LocalPlanningWindowStatus::kCapacityExceeded
            ? "LOCAL_WINDOW_CAPACITY_EXCEEDED"
            : "START_OUTSIDE_FINE_MAP";
    output.failure = SessionOutcome::kNoPath;
    output.reason_code = reason;
    return output;
  }

  std::optional<LocalTarget> target;
  try {
    target = ports.select_target(*snapshots.fine, *local_window.geometry, start,
                                        request.goal, guidance_route);
  } catch (...) {
    output.failure = SessionOutcome::kInternalError;
    output.reason_code = "INTERNAL_ERROR";
    return output;
  }
  if (stop.stop_requested()) {
    output.stopped = true;
    return output;
  }
  if (!target) {
    output.failure = SessionOutcome::kNoPath;
    output.reason_code = "NO_PATH";
    return output;
  }
  // Keep planning termination and execution completion on one internal
  // contract.  Obstacle inflation has already been consumed by the fine-map
  // state and must not enlarge this arrival region.
  target->position_tolerance_m = GoalPositionTolerance(
      *snapshots.fine, request.goal_position_tolerance_m);

  StartPatchResult patch;
  try {
    patch = ports.build_start_patch(
        snapshots.fine, *local_window.geometry, state.base_link_pose, request.capability,
        request.profile);
  } catch (...) {
    output.failure = SessionOutcome::kInternalError;
    output.reason_code = "INTERNAL_ERROR";
    return output;
  }
  if (stop.stop_requested()) {
    output.stopped = true;
    return output;
  }
  if (patch.status == StartPatchResult::Status::kStartBlocked) {
    output.failure = SessionOutcome::kNoPath;
    output.reason_code = "START_BLOCKED";
    return output;
  }
  if (patch.status == StartPatchResult::Status::kUnresolved || !patch.view) {
    output.failure = SessionOutcome::kNoPath;
    output.reason_code = "START_BLIND_ZONE_UNRESOLVED";
    return output;
  }

  LocalPlanResult local;
  const PlatformType platform = CapabilityPlatform(request.capability);
  try {
    if (platform == PlatformType::kWheeled && ports.plan_wheel) {
      local = ports.plan_wheel(
          *patch.view, state.base_link_pose, *target, deadline,
          stop);
    } else if (platform == PlatformType::kLegged &&
               ports.plan_legged) {
      local = ports.plan_legged(
          *patch.view, state.base_link_pose, *target, deadline,
          stop);
    } else {
      output.failure = SessionOutcome::kInternalError;
    output.reason_code = "INTERNAL_ERROR";
    return output;
    }
  } catch (...) {
    output.failure = SessionOutcome::kInternalError;
    output.reason_code = "INTERNAL_ERROR";
    return output;
  }
  if (stop.stop_requested()) {
    output.stopped = true;
    return output;
  }

  try {
    if (now() >= deadline &&
        local.status != LocalPlanResult::Status::kCanceled) {
      local.status = LocalPlanResult::Status::kTimeout;
    }
  } catch (...) {
    local.status = LocalPlanResult::Status::kTimeout;
  }


  output.waiting_for_map = local.status == LocalPlanResult::Status::kNoPath &&
      local.reason_code == "LOCAL_WAITING_FOR_MAP" && target->region;
  output.local = std::move(local);
  output.view = std::move(patch.view);
  return output;
}
}
