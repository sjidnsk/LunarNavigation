"""Wall-clock journey allowance and independent execution liveness guards.

This module observes execution only. It neither selects exploration targets nor
changes maps, planning costs, arrival tolerances or reward semantics.
"""
from collections import deque
from dataclasses import asdict, dataclass
import math
import statistics

VERSION = 'route_actual_rtf_budget_v1'


@dataclass(frozen=True)
class BudgetPolicy:
    minimum_journey_s: float = 120.
    journey_factor: float = 1.5
    journey_overhead_s: float = 30.
    maximum_speed_mps: float = .2
    action_accept_s: float = 10.
    feedback_gap_s: float = 15.
    no_progress_s: float = 60.
    map_wait_s: float = 15.
    progress_distance_m: float = .1
    progress_angle_rad: float = .1


def polyline_length(points):
    points = tuple(tuple(map(float, p)) for p in points)
    if any(len(p) != 2 or not all(map(math.isfinite, p)) for p in points):
        raise ValueError('finite xy path required')
    return sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))


class NavigationBudget:
    """One goal, one time origin. Route refreshes never restart its clock.

    Keep the largest published route estimate. Do not add all rolling-reference
    lengths, or cumulative driven distance, to the allowance: either would let
    cyclic replanning extend a goal indefinitely. A larger detour or measured
    slower simulation can increase the allowance, while separate liveness
    guards bound stalled execution and unproductive map waiting.
    """
    def __init__(self, *, start_wall_s, start_simulation_s, pose,
                 route_length_m, target_rtf, policy=None):
        self.policy = policy or BudgetPolicy()
        if any(not math.isfinite(v) or v <= 0 for v in asdict(self.policy).values()):
            raise ValueError('positive finite budget policy required')
        if not math.isfinite(route_length_m) or route_length_m < 0:
            raise ValueError('finite nonnegative route length required')
        if not math.isfinite(target_rtf) or target_rtf <= 0:
            raise ValueError('positive finite real-time factor required')
        self.started_wall_s = float(start_wall_s)
        self.route_length_m = float(route_length_m)
        self.target_rtf = self.effective_rtf = float(target_rtf)
        self._anchor = self._pose(pose)
        self._last_progress_s = self._last_feedback_s = self.started_wall_s
        self._map_wait_since = None
        self._rtf_wall_s = self.started_wall_s
        self._rtf_simulation_s = float(start_simulation_s)
        self._rtf_samples = deque(maxlen=5)
        self._last_wall_s = self.started_wall_s
        self.budget_s = self._estimate()

    @staticmethod
    def _pose(pose):
        p = tuple(map(float, pose))
        if len(p) != 3 or not all(map(math.isfinite, p)):
            raise ValueError('finite xy/yaw pose required')
        return p

    def _estimate(self):
        p = self.policy
        return max(p.minimum_journey_s,
            p.journey_factor*self.route_length_m/(p.maximum_speed_mps*self.effective_rtf)
            + p.journey_overhead_s)

    def observe(self, *, wall_s, simulation_s, pose, accepted,
                feedback_wall_s, route_length_m=None, planning_reason=None):
        if not math.isfinite(wall_s) or wall_s < self._last_wall_s:
            raise ValueError('monotonic finite wall time required')
        if not math.isfinite(simulation_s) or simulation_s < self._rtf_simulation_s:
            raise ValueError('monotonic finite simulation time required')
        self._last_wall_s = wall_s
        pose = self._pose(pose)
        if route_length_m is not None:
            if not math.isfinite(route_length_m) or route_length_m < 0:
                raise ValueError('finite nonnegative route length required')
            self.route_length_m = max(self.route_length_m, route_length_m)
        elapsed = wall_s-self._rtf_wall_s
        if elapsed >= 1.:
            advance = simulation_s-self._rtf_simulation_s
            if advance > 0:
                self._rtf_samples.append(min(self.target_rtf, max(.01, advance/elapsed)))
                # A median rejects a single scheduling hiccup. The initial
                # target is only a ceiling, never a claim of achieved speed.
                if len(self._rtf_samples) >= 3:
                    self.effective_rtf = statistics.median(self._rtf_samples)
            self._rtf_wall_s, self._rtf_simulation_s = wall_s, simulation_s
        self.budget_s = max(self.budget_s, self._estimate())
        if math.isfinite(feedback_wall_s):
            self._last_feedback_s = max(self._last_feedback_s, min(wall_s, feedback_wall_s))
        yaw_delta = abs(math.atan2(math.sin(pose[2]-self._anchor[2]),
                                   math.cos(pose[2]-self._anchor[2])))
        if (math.hypot(pose[0]-self._anchor[0], pose[1]-self._anchor[1]) >= self.policy.progress_distance_m
                or yaw_delta >= self.policy.progress_angle_rad):
            self._anchor, self._last_progress_s = pose, wall_s
        if planning_reason in ('WAITING_FOR_MAP', 'MAP_INSUFFICIENT', 'WAITING_FOR_NAVIGATION_MAP'):
            if self._map_wait_since is None:
                self._map_wait_since = wall_s
        else:
            self._map_wait_since = None
        if not accepted and wall_s-self.started_wall_s >= self.policy.action_accept_s:
            return 'ACTION_ACCEPT_TIMEOUT'
        if accepted and wall_s-self._last_feedback_s >= self.policy.feedback_gap_s:
            return 'NAVIGATION_FEEDBACK_TIMEOUT'
        if self._map_wait_since is not None and wall_s-self._map_wait_since >= self.policy.map_wait_s:
            return 'MAP_WAIT_TIMEOUT'
        if wall_s-self._last_progress_s >= self.policy.no_progress_s:
            return 'EXECUTION_NO_PROGRESS'
        if wall_s-self.started_wall_s > self.budget_s:
            return 'JOURNEY_BUDGET_EXCEEDED'
        return None

    def record(self):
        return dict(version=VERSION, policy=asdict(self.policy),
            started_wall_s=self.started_wall_s, elapsed_wall_s=self._last_wall_s-self.started_wall_s,
            budget_s=self.budget_s, deadline_wall_s=self.started_wall_s+self.budget_s,
            route_length_m=self.route_length_m, target_rtf=self.target_rtf,
            effective_rtf=self.effective_rtf,
            no_progress_wall_s=self._last_wall_s-self._last_progress_s,
            feedback_gap_s=self._last_wall_s-self._last_feedback_s,
            map_wait_wall_s=0. if self._map_wait_since is None else self._last_wall_s-self._map_wait_since)
