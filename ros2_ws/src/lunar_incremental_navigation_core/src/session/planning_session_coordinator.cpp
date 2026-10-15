#include "lunar_incremental_navigation_core/planning_session_coordinator.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <utility>

#include "local/grid_supercover.hpp"
#include "lunar_incremental_navigation_core/arrival_tolerance.hpp"
#include "lunar_incremental_navigation_core/local_goal_region.hpp"

namespace lunar::incremental_navigation {
namespace {

[[nodiscard]] bool FinitePose(const Pose2& pose) noexcept {
  return std::isfinite(pose.position_m.x) &&
         std::isfinite(pose.position_m.y) && std::isfinite(pose.yaw_rad);
}

[[nodiscard]] double NormalizeAngle(double angle) noexcept {
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kTwoPi = 2.0 * kPi;
  angle = std::fmod(angle + kPi, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle - kPi;
}

[[nodiscard]] double GoalPositionTolerance(
    const FineTraversabilitySnapshot&,
    const double profile_tolerance_m) noexcept {
  return profile_tolerance_m;
}

[[nodiscard]] SessionState FeedbackState(
    const CoordinatorState state) noexcept {
  switch (state) {
    case CoordinatorState::kExecuting:
      return SessionState::kExecuting;
    case CoordinatorState::kReplanning:
      return SessionState::kReplanning;
    case CoordinatorState::kIdle:
    case CoordinatorState::kPlanning:
      return SessionState::kPlanning;
  }
  return SessionState::kPlanning;
}


}  // namespace

struct PlanningSessionCoordinator::Impl final {
  Impl(PlatformCapability capability_value,
       TraversabilityProfile profile_value,
       PlanningSessionCoordinatorConfig config_value,
       PlanningSessionPorts ports_value)
      : capability(std::move(capability_value)),
        profile(std::move(profile_value)),
        config(std::move(config_value)),
        ports(std::move(ports_value)) {}

  PlatformCapability capability;
  TraversabilityProfile profile;
  PlanningSessionCoordinatorConfig config;
  PlanningSessionPorts ports;
  CoordinatorState state{CoordinatorState::kIdle};
  std::optional<SessionId> session_id;
  std::optional<FinalGoal> goal;
  std::uint64_t planning_cycle{};
  std::uint64_t segment_revision{};
  std::uint64_t captured_fine_revision{};
  std::uint64_t last_safety_evaluated_fine_revision{};
  std::optional<PathReference> active_path;
  std::shared_ptr<const RequestLocalPlanningView> active_view;
  std::optional<StateInput> latest_state;
  struct WaitingInput {
    std::uint64_t fine_revision{};
    std::uint64_t guidance_revision{};
    std::string frame_id;
    Pose2 pose;
    SearchDeadline deadline;
  };
  std::optional<WaitingInput> waiting_input;
  std::shared_ptr<const FineTraversabilitySnapshot> waiting_evidence;
  std::optional<SearchDeadline> map_wait_deadline;
  std::optional<SearchDeadline> representation_deadline;
  std::shared_ptr<std::stop_source> current_cycle_stop;
  std::optional<CycleRequest> pending;
  std::uint64_t request_sequence{};
  CycleTrigger pending_trigger{CycleTrigger::kContinue};
  bool final_alignment{};
  unsigned final_pose_corrections{};
  void DropPending() {
    if (auto source = std::atomic_load_explicit(&current_cycle_stop, std::memory_order_acquire))
      source->request_stop();
    pending.reset();
  }

  [[nodiscard]] SteadyClock::time_point Now() const {
    return config.now ? config.now() : SteadyClock::now();
  }

  [[nodiscard]] NavigateToPoseFeedback Feedback(std::string reason = {}) const {
    return NavigateToPoseFeedback{
        .session_state = FeedbackState(state),
        .planning_cycle = planning_cycle,
        .active_segment_revision =
            active_path ? active_path->segment_revision : 0U,
        .reason_code = std::move(reason),
    };
  }

  [[nodiscard]] std::optional<PathReference> Invalidate(
      const std::uint64_t evaluated_fine_revision) {
    DropPending();
    final_alignment = false;
    if (!active_path) {
      return std::nullopt;
    }
    PathReference invalidated{
        .session_id = active_path->session_id,
        .segment_revision = active_path->segment_revision,
        .traversability_revision = evaluated_fine_revision,
        .state = PathState::kInvalidated,
        .reaches_final_goal = false,
    };
    active_path.reset();
    active_view.reset();
    return invalidated;
  }

  [[nodiscard]] SessionTerminal Finish(
      const SessionOutcome outcome, std::string reason_code,
      const std::uint64_t evaluated_fine_revision) {
    SessionTerminal terminal{
        .session_id = session_id.value_or(SessionId{}),
        .result = NavigateToPoseResult{
            .outcome = outcome,
            .reason_code = std::move(reason_code),
            .last_segment_revision = segment_revision,
        },
        .invalidated = Invalidate(evaluated_fine_revision),
    };
    state = CoordinatorState::kIdle;
    session_id.reset();
    goal.reset();
    latest_state.reset();
    waiting_input.reset();
    return terminal;
  }

  [[nodiscard]] bool GoalReached(const StateInput& input,
                                 const FineTraversabilitySnapshot& fine)
      const noexcept {
    if (!active_path || !active_path->reaches_final_goal || !goal) {
      return false;
    }
    const double position_tolerance_m = GoalPositionTolerance(
        fine, config.goal_position_tolerance_m);
    if (!WithinArrivalTolerance(
            std::hypot(input.base_link_pose.position_m.x - goal->target_x_m,
                       input.base_link_pose.position_m.y - goal->target_y_m),
            position_tolerance_m)) {
      return false;
    }
    return !goal->has_target_yaw ||
           std::abs(NormalizeAngle(input.base_link_pose.yaw_rad -
                                   goal->target_yaw_rad)) <=
               config.goal_yaw_tolerance_rad;
  }

  struct RemainingPathEvaluation final {
    bool complete{};
    bool blocked{};
  };

  [[nodiscard]] RemainingPathEvaluation EvaluateRemainingPath(
      const FineTraversabilitySnapshot& fine,
      const PathReference* candidate = nullptr,
      const RequestLocalPlanningView* certified_view = nullptr) const noexcept {
    const auto* reference = candidate ? candidate : (active_path ? &*active_path : nullptr);
    if (!reference || reference->path.poses.empty()) return {};
    const auto& poses = reference->path.poses;
    std::size_t first_segment = 0U;
    double first_fraction = 0.0;
    // Newly computed candidates have not been executed. Validate their whole
    // path so a nearby later branch cannot hide an obstacle on the first one.
    // Only the already active reference may discard its completed prefix.
    if (!candidate && latest_state.has_value()) {
      double nearest = std::numeric_limits<double>::infinity();
      for (std::size_t index = 0U; index + 1U < poses.size(); ++index) {
        const Vec3& begin = poses[index].position_m;
        const Vec3& end = poses[index + 1U].position_m;
        const double dx = end.x - begin.x;
        const double dy = end.y - begin.y;
        const double length_squared = dx * dx + dy * dy;
        const double fraction =
            length_squared > 0.0
                ? std::clamp(((latest_state->base_link_pose.position_m.x -
                               begin.x) *
                                  dx +
                              (latest_state->base_link_pose.position_m.y -
                               begin.y) *
                                  dy) /
                                 length_squared,
                             0.0, 1.0)
                : 0.0;
        const double projected_x = std::lerp(begin.x, end.x, fraction);
        const double projected_y = std::lerp(begin.y, end.y, fraction);
        const double distance = std::hypot(
            projected_x - latest_state->base_link_pose.position_m.x,
            projected_y - latest_state->base_link_pose.position_m.y);
        if (distance < nearest) {
          nearest = distance;
          first_segment = index;
          first_fraction = fraction;
        }
      }
    }

    bool blocked = false;
    const auto visit = [&](const GridIndex index) {
      const auto state = fine.State(index);
      if (state == FineCellState::kBlocked ||
          (candidate && state != FineCellState::kFree &&
           (!certified_view || certified_view->Source(index) != LocalCellSource::kStartAssumedFree))) {
        blocked = true;
        return false;
      }
      return true;
    };
    if (poses.size() == 1U) {
      const bool complete = local::VisitSupercoverCells(
          fine.geometry(), poses.front().position_m,
          poses.front().position_m, visit);
      return {.complete = complete, .blocked = blocked};
    }

    for (std::size_t index = first_segment; index + 1U < poses.size();
         ++index) {
      const Vec3& begin = poses[index].position_m;
      const Vec3& end = poses[index + 1U].position_m;
      const double begin_fraction =
          index == first_segment ? first_fraction : 0.0;
      const Vec3 remaining_begin{
          .x = std::lerp(begin.x, end.x, begin_fraction),
          .y = std::lerp(begin.y, end.y, begin_fraction),
          .z = std::lerp(begin.z, end.z, begin_fraction),
      };
      if (!local::VisitSupercoverCells(fine.geometry(), remaining_begin, end,
                                       visit)) {
        return {.complete = false, .blocked = blocked};
      }
    }
    return {.complete = true, .blocked = false};
  }
};

PlanningSessionCoordinator::PlanningSessionCoordinator(
    PlatformCapability capability, TraversabilityProfile profile,
    PlanningSessionCoordinatorConfig config, PlanningSessionPorts ports)
    : impl_(std::make_unique<Impl>(
          std::move(capability), std::move(profile), std::move(config),
          std::move(ports))) {
  if (!std::isfinite(impl_->config.goal_position_tolerance_m) ||
      impl_->config.goal_position_tolerance_m < 0.0 ||
      !std::isfinite(impl_->config.goal_yaw_tolerance_rad) ||
      impl_->config.goal_yaw_tolerance_rad < 0.0 ||
      !std::isfinite(impl_->config.local_window_size_m) ||
      impl_->config.local_window_size_m <= 0.0 ||
      impl_->config.global_subdeadline < std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument(
        "planning session tolerances, local window, and subdeadline are invalid");
  }
}

PlanningSessionCoordinator::~PlanningSessionCoordinator() = default;
PlanningSessionCoordinator::PlanningSessionCoordinator(
    PlanningSessionCoordinator&&) noexcept = default;
PlanningSessionCoordinator& PlanningSessionCoordinator::operator=(
    PlanningSessionCoordinator&&) noexcept = default;

StartSessionResult PlanningSessionCoordinator::Start(const SessionId id,
                                                     const FinalGoal goal) {
  if (!IsValidFinalGoal(goal)) {
    return StartSessionResult{
        .terminal = SessionTerminal{
            .session_id = id,
            .result = NavigateToPoseResult{
                .outcome = SessionOutcome::kInvalidGoal,
                .reason_code = "INVALID_GOAL",
            },
        },
    };
  }

  StartSessionResult result{.accepted = true};
  if (impl_->state != CoordinatorState::kIdle && impl_->session_id) {
    result.replaced_session = impl_->Finish(
        SessionOutcome::kCanceled, "PREEMPTED",
        impl_->last_safety_evaluated_fine_revision);
  }

  impl_->DropPending();
  impl_->final_alignment = false;
  impl_->final_pose_corrections = 0U;
  impl_->session_id = id;
  impl_->goal = goal;
  impl_->representation_deadline.reset();
  impl_->map_wait_deadline.reset();
  impl_->waiting_evidence.reset();
  impl_->planning_cycle = 0U;
  impl_->segment_revision = 0U;
  impl_->captured_fine_revision = 0U;
  impl_->last_safety_evaluated_fine_revision = 0U;
  impl_->active_path.reset();
  impl_->active_view.reset();
  impl_->latest_state.reset();
  impl_->waiting_input.reset();
  impl_->state = CoordinatorState::kPlanning;
  return result;
}

PreparedCycle PlanningSessionCoordinator::PrepareCycle(
    const StateInput state, const SnapshotBundle snapshots,
    const CycleTrigger trigger, const SearchDeadline deadline, const bool rolling) {
  PreparedCycle prepared;
  auto& output = prepared.immediate;
  if (impl_->state == CoordinatorState::kIdle || !impl_->session_id ||
      !impl_->goal) {
    return prepared;
  }
  ++impl_->planning_cycle;
  impl_->latest_state = state;
  if (snapshots.fine) {
    impl_->captured_fine_revision = std::max(
        impl_->captured_fine_revision,
        snapshots.fine->fine_traversability_revision());
  }

  if (!FinitePose(state.base_link_pose)) {
    output.terminal = impl_->Finish(SessionOutcome::kInternalError,
                                    "INVALID_STATE",
                                    impl_->last_safety_evaluated_fine_revision);
    output.feedback = impl_->Feedback("INVALID_STATE");
    return prepared;
  }

  // A waiting session remains cancelable, but unchanged evidence cannot help search.
  if (impl_->waiting_input && snapshots.fine) {
    const auto& waiting = *impl_->waiting_input;
    const auto& pose = state.base_link_pose;
    const auto guidance_revision = snapshots.guidance
        ? snapshots.guidance->global_guidance_revision() : 0U;
    if (waiting.fine_revision == snapshots.fine->fine_traversability_revision() &&
        waiting.guidance_revision == guidance_revision &&
        waiting.frame_id == snapshots.fine->geometry().frame_id() &&
        WithinArrivalTolerance(std::hypot(pose.position_m.x-waiting.pose.position_m.x,
                                         pose.position_m.y-waiting.pose.position_m.y), 0.0) &&
        WithinArrivalTolerance(std::abs(NormalizeAngle(pose.yaw_rad-waiting.pose.yaw_rad)), 0.0)) {
      if (impl_->Now() >= waiting.deadline) {
        output.terminal = impl_->Finish(SessionOutcome::kMapUnavailable, "MAP_WAIT_TIMEOUT",
            impl_->last_safety_evaluated_fine_revision);
        output.feedback = impl_->Feedback("MAP_WAIT_TIMEOUT");
      } else output.feedback = impl_->Feedback("WAITING_FOR_MAP");
      return prepared;
    }
    impl_->waiting_input.reset();
    impl_->state = CoordinatorState::kReplanning;
  }

  if (trigger != CycleTrigger::kFinalPoseLost &&
      snapshots.fine && impl_->GoalReached(state, *snapshots.fine) &&
      (!impl_->config.require_execution_confirmation || trigger == CycleTrigger::kSegmentEnd)) {
    output.terminal = impl_->Finish(SessionOutcome::kGoalReached,
                                    "GOAL_REACHED",
                                    impl_->last_safety_evaluated_fine_revision);
    output.feedback = impl_->Feedback("GOAL_REACHED");
    return prepared;
  }

  if (trigger == CycleTrigger::kDeviation && impl_->active_path) {
    if (impl_->active_path->reaches_final_goal && impl_->final_pose_corrections != 0U) {
      output.terminal = impl_->Finish(SessionOutcome::kNoPath,
          "FINAL_POSE_NOT_REACHED", impl_->last_safety_evaluated_fine_revision);
      output.feedback = impl_->Feedback("FINAL_POSE_NOT_REACHED");
      return prepared;
    }
    output.path_reference =
        impl_->Invalidate(impl_->last_safety_evaluated_fine_revision);
    impl_->state = CoordinatorState::kReplanning;
    output.feedback = impl_->Feedback("PATH_DEVIATED");
    return prepared;
  }

  const bool planning_required =
      impl_->state == CoordinatorState::kPlanning ||
      impl_->state == CoordinatorState::kReplanning ||
      trigger == CycleTrigger::kSegmentEnd || trigger == CycleTrigger::kFinalPoseLost || rolling;
  if (!planning_required || impl_->pending || (rolling && impl_->final_alignment)) {
    output.feedback = impl_->Feedback("EXECUTING");
    return prepared;
  }
  if (trigger == CycleTrigger::kSegmentEnd || trigger == CycleTrigger::kFinalPoseLost) {
    if (impl_->active_path && impl_->active_path->reaches_final_goal) {
      // One fresh reference may correct residual motion after executor
      // completion. Repeating the same miss must yield a bounded failure.
      if (impl_->final_pose_corrections++ != 0U) {
        output.terminal = impl_->Finish(SessionOutcome::kNoPath,
            "FINAL_POSE_NOT_REACHED", impl_->last_safety_evaluated_fine_revision);
        output.feedback = impl_->Feedback("FINAL_POSE_NOT_REACHED");
        return prepared;
      }
    }
    impl_->final_alignment = false;
    impl_->state = CoordinatorState::kReplanning;
  }

  if (!snapshots.fine) {
    output.terminal = impl_->Finish(SessionOutcome::kMapUnavailable,
                                    "MAP_UNAVAILABLE",
                                    impl_->last_safety_evaluated_fine_revision);
    output.feedback = impl_->Feedback("MAP_UNAVAILABLE");
    return prepared;
  }

  try {
    if (impl_->Now() >= deadline) {
      output.terminal = impl_->Finish(SessionOutcome::kTimeout, "TIMEOUT",
                                      impl_->last_safety_evaluated_fine_revision);
      output.feedback = impl_->Feedback("TIMEOUT");
      return prepared;
    }
  } catch (...) {
    output.terminal = impl_->Finish(SessionOutcome::kInternalError,
                                    "INTERNAL_ERROR",
                                    impl_->last_safety_evaluated_fine_revision);
    output.feedback = impl_->Feedback("INTERNAL_ERROR");
    return prepared;
  }

  if (impl_->ports.plan_adaptive_global || impl_->config.adaptive_global) {
    if (!snapshots.navigation || snapshots.navigation->fine_ptr() != snapshots.fine) {
      if (!impl_->representation_deadline)
        impl_->representation_deadline = std::min(deadline, impl_->Now() + impl_->config.map_wait_timeout);
      output.global_reason = impl_->Now() >= *impl_->representation_deadline
          ? "NAVIGATION_MAP_TIMEOUT" : "NAVIGATION_MAP_PENDING";
      if (output.global_reason == "NAVIGATION_MAP_TIMEOUT")
        output.terminal = impl_->Finish(SessionOutcome::kMapUnavailable, output.global_reason,
            impl_->last_safety_evaluated_fine_revision);
      output.feedback = impl_->Feedback(output.global_reason);
      return prepared;
    }
    impl_->representation_deadline.reset();
  }
  auto source = std::make_shared<std::stop_source>();
  std::atomic_store_explicit(&impl_->current_cycle_stop, source, std::memory_order_release);
  impl_->pending = CycleRequest{
      .request_id = ++impl_->request_sequence,
      .session_id = *impl_->session_id, .state = state, .goal = *impl_->goal,
      .snapshots = snapshots, .capability = impl_->capability, .profile = impl_->profile,
      .local_window_size_m = impl_->config.local_window_size_m,
      .goal_position_tolerance_m = impl_->config.goal_position_tolerance_m,
      .global_subdeadline = impl_->config.global_subdeadline,
      .deadline = deadline, .rolling = rolling, .stop_token = source->get_token()};
  impl_->pending_trigger = trigger;
  prepared.request = impl_->pending;
  return prepared;
}

CycleOutput PlanningSessionCoordinator::PlanCycle(
    StateInput state, SnapshotBundle snapshots, CycleTrigger trigger, SearchDeadline deadline) {
  auto prepared = PrepareCycle(state, snapshots, trigger, deadline);
  if (!prepared.request) return prepared.immediate;
  auto candidate = ComputeCycle(*prepared.request, impl_->ports, impl_->config.now,
                                prepared.request->stop_token);
  return ApplyCycle(std::move(candidate), state, snapshots);
}

CycleOutput PlanningSessionCoordinator::ApplyCycle(
    CycleCandidate candidate, StateInput current_state, SnapshotBundle current_snapshots) {
  CycleOutput output;
  if (!impl_->pending || !impl_->session_id ||
      candidate.session_id != *impl_->session_id ||
      candidate.request_id != impl_->pending->request_id) return output;
  const auto request = *impl_->pending;
  impl_->pending.reset();
  const auto trigger = impl_->pending_trigger;
  const auto& state = request.state;
  const auto& snapshots = request.snapshots;
  const auto deadline = request.deadline;
  const Point2 start = state.base_link_pose.position_m;
  impl_->latest_state = current_state;
  output.global_reason = std::move(candidate.global_reason);
  output.guidance_status = candidate.guidance_status;
  output.global_route = std::move(candidate.global_route);
  if (candidate.stopped || request.stop_token.stop_requested()) {
    output.feedback = impl_->Feedback("CYCLE_STOPPED");
    return output;
  }
  auto fail = [&](SessionOutcome outcome, const std::string& reason) {
    if (!(request.rolling && impl_->active_path)) {
      output.terminal = impl_->Finish(outcome, reason, impl_->last_safety_evaluated_fine_revision);
    }
    output.feedback = impl_->Feedback(reason);
    return output;
  };
  if (!FinitePose(current_state.base_link_pose)) return fail(SessionOutcome::kInternalError, "INVALID_STATE");
  if (candidate.failure) return fail(*candidate.failure, candidate.reason_code);
  auto& local = candidate.local;
  if (request.rolling && impl_->active_path && local.status != LocalPlanResult::Status::kPlanFound) {
    return fail(local.status == LocalPlanResult::Status::kTimeout ? SessionOutcome::kTimeout : SessionOutcome::kNoPath,
                local.status == LocalPlanResult::Status::kTimeout ? "TIMEOUT" :
                (local.reason_code.empty() ? "NO_PATH" : local.reason_code));
  }
  if (local.status == LocalPlanResult::Status::kNoPath &&
      candidate.waiting_for_map) {
    output.path_reference = impl_->Invalidate(impl_->last_safety_evaluated_fine_revision);
    if (impl_->waiting_evidence != snapshots.fine || !impl_->map_wait_deadline) {
      impl_->waiting_evidence = snapshots.fine;
      impl_->map_wait_deadline = impl_->Now() + impl_->config.map_wait_timeout;
    }
    if (impl_->Now() >= *impl_->map_wait_deadline)
      return fail(SessionOutcome::kMapUnavailable, "MAP_WAIT_TIMEOUT");
    impl_->waiting_input = Impl::WaitingInput{
        .fine_revision = snapshots.fine->fine_traversability_revision(),
        .guidance_revision = snapshots.guidance
            ? snapshots.guidance->global_guidance_revision() : 0U,
        .frame_id = snapshots.fine->geometry().frame_id(),
        .pose = state.base_link_pose,
        .deadline = *impl_->map_wait_deadline};
    impl_->state = CoordinatorState::kPlanning;
    output.feedback = impl_->Feedback("WAITING_FOR_MAP");
    return output;
  }

  if (local.status != LocalPlanResult::Status::kPlanFound) {
    SessionOutcome outcome = SessionOutcome::kNoPath;
    std::string reason = local.reason_code.empty() ? "NO_PATH"
                                                    : local.reason_code;
    if (local.status == LocalPlanResult::Status::kTimeout) {
      outcome = SessionOutcome::kTimeout;
      reason = "TIMEOUT";
    } else if (local.status == LocalPlanResult::Status::kCanceled) {
      outcome = SessionOutcome::kCanceled;
      reason = "CANCELED";
    }
    output.terminal =
        impl_->Finish(outcome, reason,
                      impl_->last_safety_evaluated_fine_revision);
    output.feedback = impl_->Feedback(reason);
    return output;
  }

  const std::vector<PathPoint>& selected_path =
      local.path.empty() ? local.raw_path : local.path;
  if (selected_path.empty()) {
    return fail(SessionOutcome::kInternalError, "EMPTY_LOCAL_PATH");
  }

  // A non-final reference consisting only of the current pose cannot advance
  // exploration. Do not endlessly publish new revisions of an already-done segment.
  const bool stationary = std::all_of(selected_path.begin(), selected_path.end(),
      [&](const PathPoint& point) {
        const auto& q = point.pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w*q.z + q.x*q.y),
                                      1.0 - 2.0 * (q.y*q.y + q.z*q.z));
        return std::hypot(point.pose.position_m.x - start.x,
                          point.pose.position_m.y - start.y) <= 1e-6 &&
               std::abs(NormalizeAngle(yaw - state.base_link_pose.yaw_rad)) <= 1e-6;
      });
  bool repeated_completed_target = false;
  if (trigger == CycleTrigger::kSegmentEnd && impl_->active_path &&
      !impl_->active_path->path.poses.empty()) {
    const auto& previous = impl_->active_path->path.poses.back();
    const auto& next = selected_path.back().pose;
    const auto yaw = [](const Quaternion& q) {
      return std::atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z));
    };
    repeated_completed_target =
        std::hypot(previous.position_m.x-next.position_m.x,
                   previous.position_m.y-next.position_m.y) <= 1e-6 &&
        std::abs(NormalizeAngle(yaw(previous.orientation)-yaw(next.orientation))) <= 1e-3 &&
        std::hypot(next.position_m.x-start.x, next.position_m.y-start.y) <=
            GoalPositionTolerance(*snapshots.fine, impl_->config.goal_position_tolerance_m);
  }
  if (!local.reaches_final_goal && (stationary || repeated_completed_target)) {
    return fail(SessionOutcome::kNoPath, "NO_LOCAL_PROGRESS");
  }

  PathReference reference{
      .session_id = *impl_->session_id,
      .segment_revision = impl_->segment_revision + 1U,
      .traversability_revision =
          snapshots.fine->fine_traversability_revision(),
      .state = PathState::kActive,
      .reaches_final_goal = local.reaches_final_goal,
  };
  reference.path.poses.reserve(selected_path.size());
  for (const PathPoint& point : selected_path) {
    reference.path.poses.push_back(point.pose);
  }
  try {
    if (impl_->Now() >= deadline) {
      return fail(SessionOutcome::kTimeout, "TIMEOUT");
    }
  } catch (...) {
    return fail(SessionOutcome::kTimeout, "TIMEOUT");
  }
  if (current_snapshots.fine && current_snapshots.fine->fine_traversability_revision() >
      snapshots.fine->fine_traversability_revision()) {
    const auto evaluation = impl_->EvaluateRemainingPath(
        *current_snapshots.fine, &reference, candidate.view.get());
    if (!evaluation.complete || evaluation.blocked)
      return fail(SessionOutcome::kNoPath, "CANDIDATE_PATH_BLOCKED");
    reference.traversability_revision = current_snapshots.fine->fine_traversability_revision();
  }
  impl_->last_safety_evaluated_fine_revision = reference.traversability_revision;
  impl_->segment_revision = reference.segment_revision;
  impl_->map_wait_deadline.reset();
  impl_->waiting_evidence.reset();
  impl_->active_path = reference;
  impl_->active_view = std::move(candidate.view);
  impl_->state = CoordinatorState::kExecuting;
  output.path_reference = std::move(reference);
  output.feedback = impl_->Feedback("PLAN_FOUND");
  return output;
}

std::optional<PathReference> PlanningSessionCoordinator::OnFineSnapshot(
    std::shared_ptr<const FineTraversabilitySnapshot> fine) {
  if (fine) {
    impl_->captured_fine_revision = std::max(
        impl_->captured_fine_revision,
        fine->fine_traversability_revision());
  }
  if (!fine || impl_->state != CoordinatorState::kExecuting ||
      !impl_->active_path ||
      fine->fine_traversability_revision() <=
          impl_->last_safety_evaluated_fine_revision) {
    return std::nullopt;
  }
  const Impl::RemainingPathEvaluation evaluation =
      impl_->EvaluateRemainingPath(*fine);
  if (!evaluation.complete && !evaluation.blocked) {
    return std::nullopt;
  }
  impl_->last_safety_evaluated_fine_revision =
      fine->fine_traversability_revision();
  if (!evaluation.blocked) {
    return std::nullopt;
  }
  auto invalidated =
      impl_->Invalidate(impl_->last_safety_evaluated_fine_revision);
  impl_->state = CoordinatorState::kReplanning;
  return invalidated;
}

bool PlanningSessionCoordinator::RequestStopCurrentCycle() noexcept {
  const auto source = std::atomic_load_explicit(
      &impl_->current_cycle_stop, std::memory_order_acquire);
  return source && source->request_stop();
}

SessionTerminal PlanningSessionCoordinator::Cancel(
    const CancelReason reason) {
  if (impl_->state == CoordinatorState::kIdle || !impl_->session_id) {
    return SessionTerminal{
        .result = NavigateToPoseResult{
            .outcome = SessionOutcome::kCanceled,
            .reason_code = reason == CancelReason::kPreempted ? "PREEMPTED"
                                                               : "CANCELED"},
    };
  }
  return impl_->Finish(SessionOutcome::kCanceled,
                       reason == CancelReason::kPreempted ? "PREEMPTED"
                                                          : "CANCELED",
                       impl_->last_safety_evaluated_fine_revision);
}

CoordinatorState PlanningSessionCoordinator::state() const noexcept {
  return impl_->state;
}

std::uint64_t PlanningSessionCoordinator::planning_cycle() const noexcept {
  return impl_->planning_cycle;
}

std::uint64_t PlanningSessionCoordinator::active_segment_revision()
    const noexcept {
  return impl_->active_path ? impl_->active_path->segment_revision : 0U;
}

}  // namespace lunar::incremental_navigation

namespace lunar::incremental_navigation {
void PlanningSessionCoordinator::StopPendingCycle() { impl_->DropPending(); }
void PlanningSessionCoordinator::SetFinalAlignment(bool value) {
  // Execution holds supersede speculative rolling updates, but the stopped
  // executor still needs the required replacement/correction computation.
  if (value && impl_->pending && impl_->pending->rolling) impl_->DropPending();
  impl_->final_alignment = value;
}
double PlanningSessionCoordinator::remaining_path_m() const noexcept {
  if (!impl_->active_path || !impl_->latest_state) return 0.;
  const auto& poses = impl_->active_path->path.poses;
  double total=0., progress=0., nearest=std::numeric_limits<double>::infinity();
  const auto p=impl_->latest_state->base_link_pose.position_m;
  for (std::size_t i=1;i<poses.size();++i) {
    const auto a=poses[i-1].position_m, b=poses[i].position_m;
    const double dx=b.x-a.x, dy=b.y-a.y, len=std::hypot(dx,dy);
    const double t=len>0.?std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/(len*len),0.,1.):0.;
    const double d=std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
    if (d<nearest) { nearest=d; progress=total+t*len; }
    total+=len;
  }
  return total-progress;
}
}
