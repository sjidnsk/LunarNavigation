"""Stateful nonholonomic reference execution, independent of ROS transport."""

from __future__ import annotations

from dataclasses import dataclass
import math
from .path_geometry import Polyline, angle
from .limits import tracking_speed
from .tracking import TrackingCommand, TrackingPolicy, TrackingState, _valid_policy


@dataclass(frozen=True)
class ExecutionResult:
    command: TrackingCommand
    phase: str
    progress_m: float
    cross_track_m: float
    heading_error_rad: float
    direction: int


class PathExecutor:
    def __init__(self, policy: TrackingPolicy):
        if not _valid_policy(policy):
            raise ValueError("invalid execution limits")
        self.policy = policy
        self.identity = None
        self.path = None
        self.phase = "WAITING"
        self.progress_m = 0.0
        self.index = 0
        self.direction = 1
        self.error = 0.0
        self.heading_error = 0.0
        self.failure = None
        self.no_progress_s = 0.0
        self._best_progress = 0.0
        self._best_angle = math.inf
        self._stop_index = 0
        self._chosen = False
        self.final = True

    def clear(self):
        self.path = None
        self.phase = "WAITING"
        self.failure = None
        self._chosen = False

    def set_path(self, points, identity=None, final=True):
        path = Polyline(points)
        if (
            identity is not None
            and self.identity is not None
            and identity[0] == self.identity[0]
        ):
            if identity[1] < self.identity[1]:
                return False
            if identity == self.identity:
                if self.path is not None and path.original != self.path.original:
                    self.failure = "REFERENCE_MUTATED"
                    self.phase = "BRAKING"
                return False
        self.path = path
        self.identity = identity
        self.final = final
        self.phase = "BRAKING"
        self.progress_m = 0.0
        self.index = 0
        self.error = 0.0
        self.heading_error = 0.0
        self.direction = 1
        self._chosen = False
        self._stop_index = 0
        self.failure = None
        self.no_progress_s = 0.0
        self._best_progress = 0.0
        self._best_angle = math.inf
        # Each certified bend is an execution boundary. A shallow bend may
        # still protect an obstacle; lookahead must not skip it across the chord.
        # Collinear sampling vertices remain transparent.
        self.stops = path.stop_arcs(min(self.policy.corner_angle_rad, 1e-6))
        return True

    def _result(self, v=0.0, w=0.0) -> ExecutionResult:
        command = TrackingCommand(
            v,
            w,
            self.phase == "COMPLETED",
            self.failure if self.phase == "FAILED" else None,
        )
        return ExecutionResult(
            command,
            self.phase,
            self.progress_m,
            self.error,
            self.heading_error,
            self.direction,
        )

    def _stopped(self, state):
        p = self.policy
        return (
            abs(state.linear_mps) <= p.stopped_linear_mps
            and abs(state.angular_radps) <= p.stopped_angular_radps
        )

    def _rotation(self, state, error, dt, *, tolerance=None):
        p = self.policy
        self.heading_error = error
        if tolerance is None:
            tolerance = (
                p.goal_yaw_tolerance_rad * (0.5 if self.final else 1.0)
                if self.phase == "FINAL_ALIGN"
                else p.alignment_tolerance_rad
            )
        if abs(error) < tolerance:
            target = 0.0
        else:
            target = math.copysign(
                min(
                    p.max_angular_radps,
                    p.spin_kp * abs(error),
                    math.sqrt(
                        2
                        * p.max_angular_accel_radps2
                        * max(0.0, abs(error) - tolerance)
                    ),
                ),
                error,
            )
        bound = p.max_angular_accel_radps2 * dt
        w = max(state.angular_radps - bound, min(state.angular_radps + bound, target))
        return self._result(0.0, max(-p.max_angular_radps, min(p.max_angular_radps, w)))

    def update(self, state: TrackingState, dt: float) -> ExecutionResult:
        p = self.policy
        if (
            not all(
                math.isfinite(v)
                for v in (
                    state.x_m,
                    state.y_m,
                    state.yaw_rad,
                    state.linear_mps,
                    state.angular_radps,
                    dt,
                )
            )
            or dt <= 0
        ):
            self.failure = "INVALID_STATE"
            self.phase = "FAILED"
            return self._result()
        dt = min(dt, 0.2)
        if self.path is None or self.phase == "FAILED":
            return self._result()
        line = self.path
        if self.phase == "COMPLETED":
            # Completion feedback may be repeated while navigation verifies the
            # latest odometry. Never latch success after measured pose drifts
            # outside the public arrival contract.
            if self.final:
                x, y = line.point_at(line.length)
                if math.hypot(x - state.x_m, y - state.y_m) > p.goal_position_tolerance_m:
                    self.failure = "GOAL_POSITION_LOST"
                elif abs(angle(line.final_yaw - state.yaw_rad)) > p.goal_yaw_tolerance_rad:
                    self.failure = "GOAL_YAW_LOST"
                if self.failure:
                    self.phase = "FAILED" if self._stopped(state) else "BRAKING"
            return self._result()
        projection = line.project(
            state.x_m,
            state.y_m,
            None if not self._chosen else self.index,
            (
                None
                if not self._chosen
                else self.progress_m
                + max(p.lookahead_m, abs(state.linear_mps) * dt)
                + 0.25
            ),
        )
        self.index = projection.index
        self.progress_m = max(self.progress_m, projection.s)
        self.error = projection.error
        if self.error > p.max_cross_track_error_m:
            self.failure = "PATH_DEVIATION"
            self.phase = "BRAKING"
        if self.failure:
            if self._stopped(state):
                self.phase = "FAILED"
            return self._result()
        if not self._chosen:
            heading = line.headings[self.index] if line.headings else line.final_yaw
            self.direction = (
                -1
                if p.allow_reverse
                and abs(angle(heading - state.yaw_rad))
                > p.reverse_heading_threshold_rad
                else 1
            )
            self._chosen = True
            self._stop_index = next(
                (i for i, s in enumerate(self.stops) if s >= self.progress_m - 1e-8),
                len(self.stops) - 1,
            )
        stop_s = self.stops[self._stop_index]
        stop_xy = line.point_at(stop_s)
        at_end = self._stop_index == len(self.stops) - 1
        tolerance = (
            p.goal_position_tolerance_m * 0.5
            if at_end and self.final
            else min(p.corner_position_tolerance_m, .25*p.execution_clearance_m)
        )
        distance = math.hypot(stop_xy[0] - state.x_m, stop_xy[1] - state.y_m)
        arrived = distance <= tolerance and stop_s - self.progress_m <= tolerance
        yaw_tolerance = p.goal_yaw_tolerance_rad * (0.5 if self.final else 1.0)
        if self.phase == "FINAL_ALIGN":
            # Use the public position band as hysteresis during rotation; the
            # tighter control target above leaves room for settling motion.
            retained = (distance <= p.goal_position_tolerance_m and
                        stop_s - self.progress_m <= p.goal_position_tolerance_m)
            if not (retained if self.final else arrived):
                # Localization can move along the path without cross-track
                # error. Stop and request a newly certified reference when
                # the position accepted before final alignment is lost.
                self.failure = "GOAL_POSITION_LOST"
                self.phase = "BRAKING"
                return self._result()
            error = angle(line.final_yaw - state.yaw_rad)
            if abs(error) < self._best_angle - 0.01:
                self._best_angle = abs(error)
                self.no_progress_s = 0.0
            else:
                self.no_progress_s += dt
            if self.no_progress_s > p.no_progress_timeout_s:
                self.failure = "ALIGNMENT_STALLED"
                self.phase = "BRAKING"
                return self._result()
            if abs(error) <= yaw_tolerance and self._stopped(state):
                self.phase = "COMPLETED"
                return self._result()
            return self._rotation(state, error, dt)
        if arrived:
            if not self._stopped(state):
                self.phase = "BRAKING"
                return self._result()
            if at_end:
                error = angle(line.final_yaw - state.yaw_rad)
                if abs(error) <= yaw_tolerance:
                    self.phase = "COMPLETED"
                    return self._result()
                self.phase = "FINAL_ALIGN"
                self._best_angle = math.inf
                self.no_progress_s = 0.0
                return self._rotation(state, error, dt)
            self._stop_index += 1
            stop_s = self.stops[self._stop_index]
            self.phase = "ALIGNING"
            self._best_angle = math.inf
        target = line.point_at(min(stop_s, self.progress_m + p.lookahead_m))
        dx, dy = target[0] - state.x_m, target[1] - state.y_m
        travel_yaw = state.yaw_rad + (math.pi if self.direction < 0 else 0.0)
        # Alignment, tracking re-entry and pursuit use the same chord. Switching
        # back to the path tangent after alignment can immediately demand another
        # rotation at a lateral offset, without making progress along the path.
        self.heading_error = angle(math.atan2(dy, dx) - travel_yaw)
        lateral = -math.sin(travel_yaw) * dx + math.cos(travel_yaw) * dy
        k = 2 * lateral / max(dx * dx + dy * dy, 0.01**2)
        # Tighten short-tail alignment to leave curvature/braking margin. This
        # is a feasibility bound, not a certificate of obstacle clearance.
        alignment_tolerance = min(
            p.alignment_tolerance_rad,
            0.5 * math.asin(min(1.0, p.max_curvature_per_m *
                                max(math.hypot(dx, dy), 0.01) / 2.0)),
        )
        if self.phase == "BRAKING":
            if not self._stopped(state):
                return self._result()
            self.phase = "ALIGNING"
        if self.phase == "TRACKING" and abs(self.heading_error) > p.rotate_enter_rad:
            self.phase = "BRAKING"
            return self._result()
        if self.phase == "ALIGNING":
            if abs(self.heading_error) <= alignment_tolerance and self._stopped(
                state
            ):
                self.phase = "TRACKING"
                self.no_progress_s = 0.0
                self._best_progress = self.progress_m
            else:
                if abs(self.heading_error) < self._best_angle - 0.01:
                    self._best_angle = abs(self.heading_error)
                    self.no_progress_s = 0.0
                else:
                    self.no_progress_s += dt
                if self.no_progress_s > p.no_progress_timeout_s:
                    self.failure = "ALIGNMENT_STALLED"
                    self.phase = "BRAKING"
                    return self._result()
                return self._rotation(state, self.heading_error, dt,
                                      tolerance=alignment_tolerance)
        if self.progress_m > self._best_progress + 0.01:
            self._best_progress = self.progress_m
            self.no_progress_s = 0.0
        else:
            self.no_progress_s += dt
        if self.no_progress_s > p.no_progress_timeout_s:
            self.failure = "TRACKING_STALLED"
            self.phase = "BRAKING"
            return self._result()
        # Pure pursuit in the direction of travel; reverse yaw rate uses |v|.
        if abs(k) > p.max_curvature_per_m:
            self.phase = "BRAKING"
            self._best_angle = math.inf
            # An aligned but infeasible command needs a replacement reference.
            if abs(self.heading_error) <= alignment_tolerance:
                self.failure = "CURVATURE_INFEASIBLE"
            return self._result()
        # Projection can reach the vertex while a small lateral distance remains.
        # Brake to the measured target too, otherwise a tight corner band stalls.
        remaining = max(0.0, max(stop_s-self.progress_m, distance)-tolerance*.5)
        # Allow measured command/odometry response before braking begins.
        # Solve v*tau + v^2/(2*a) <= remaining instead of assuming zero delay.
        response = max(p.control_response_time_s, 3*dt)
        braking_offset = p.max_linear_decel_mps2*response
        braking_speed = (math.sqrt(braking_offset**2 +
                                   2*p.max_linear_decel_mps2*remaining)-braking_offset)
        speed = min(
            p.max_linear_mps if self.direction > 0 else p.max_reverse_mps,
            braking_speed,
        )
        if abs(k) > 1e-9:
            speed = min(
                speed,
                p.max_angular_radps / abs(k),
                math.sqrt(p.max_lateral_accel_mps2 / abs(k)),
            )
        # Slow corrective travel before the estimated lateral braking excursion
        # consumes the reserve. This is a regulation bound, not a swept-volume
        # safety certificate. Keep v/omega coupled through tracking_speed below.
        reserve = max(.1*p.execution_clearance_m,
                      .5*p.execution_clearance_m-self.error)
        lateral_fraction = max(abs(math.sin(self.heading_error)), .01)
        speed = min(speed, math.sqrt(2*p.max_linear_decel_mps2*reserve /
                                     lateral_fraction))
        current = max(0.0, self.direction * state.linear_mps)
        speed = tracking_speed(k, speed, current, state.angular_radps, dt, p)
        if speed is None:
            self.phase = "BRAKING"
            return self._result()
        return self._result(self.direction * speed, speed * k)
