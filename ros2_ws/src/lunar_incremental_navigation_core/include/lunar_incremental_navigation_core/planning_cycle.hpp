#pragma once
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

#include "lunar_incremental_navigation_core/global_route_planner.hpp"
#include "lunar_incremental_navigation_core/adaptive_route_planner.hpp"
#include "lunar_incremental_navigation_core/legged_local_planner.hpp"
#include "lunar_incremental_navigation_core/local_planning_window.hpp"
#include "lunar_incremental_navigation_core/local_target_selector.hpp"
#include "lunar_incremental_navigation_core/request_local_start_patch.hpp"
#include "lunar_incremental_navigation_core/types/path_reference.hpp"
#include "lunar_incremental_navigation_core/wheel_local_planner.hpp"

namespace lunar::incremental_navigation {
struct StateInput final {
  Pose2 base_link_pose;
};

struct PlanningSessionPorts final {
  std::function<GlobalRouteResult(
      const GlobalGuidanceSnapshot&, Point2, Point2, SearchDeadline,
      const StopToken&)>
      plan_global;
  std::function<std::optional<LocalTarget>(
      const FineTraversabilitySnapshot&, const SparseGridGeometry&, Point2,
      const FinalGoal&,
      const std::optional<GlobalRoute>&)>
      select_target;
  std::function<StartPatchResult(
      std::shared_ptr<const FineTraversabilitySnapshot>,
      const SparseGridGeometry&, const Pose2&,
      const PlatformCapability&, const TraversabilityProfile&)>
      build_start_patch;
  std::function<LocalPlanResult(
      const RequestLocalPlanningView&, const Pose2&, const LocalTarget&,
      SearchDeadline, const StopToken&)>
      plan_wheel;
  std::function<LocalPlanResult(
      const RequestLocalPlanningView&, const Pose2&, const LocalTarget&,
      SearchDeadline, const StopToken&)>
      plan_legged;
  // Optional known-space navigation; independent of exploration inputs.
  std::function<AdaptiveRouteResult(const AdaptiveNavigationSnapshot&, Point2,
      Point2, SearchDeadline, const StopToken&)> plan_adaptive_global;
  // Appended for callers that can interrupt regional target construction.
  // Existing integrations may keep supplying select_target above.
  std::function<std::optional<LocalTarget>(
      const FineTraversabilitySnapshot&, const SparseGridGeometry&, Point2,
      const FinalGoal&, const std::optional<GlobalRoute>&, const SearchControl&)>
      select_target_controlled;
};


struct CycleRequest final {
  std::uint64_t request_id{};
  SessionId session_id;
  StateInput state;
  FinalGoal goal;
  SnapshotBundle snapshots;
  PlatformCapability capability;
  TraversabilityProfile profile;
  double local_window_size_m{};
  double goal_position_tolerance_m{};
  std::chrono::nanoseconds global_subdeadline{};
  SearchDeadline deadline;
  bool rolling{};
  StopToken stop_token;
};
struct CycleCandidate final {
  std::uint64_t request_id{};
  SessionId session_id;
  LocalPlanResult local;
  GuidanceStatus guidance_status{GuidanceStatus::kUnavailable};
  std::optional<GlobalRoute> global_route;
  std::shared_ptr<const RequestLocalPlanningView> view;
  std::optional<SessionOutcome> failure;
  std::string reason_code;
  std::string global_reason;
  bool waiting_for_map{};
  bool stopped{};
};
[[nodiscard]] CycleCandidate ComputeCycle(const CycleRequest&, PlanningSessionPorts&,
                                         const NowFn&, const StopToken&);
}
