"""Stateful nonholonomic reference execution, independent of ROS transport."""

from __future__ import annotations

from dataclasses import dataclass
import math
from .path_geometry import Polyline, angle
from .limits import rolling_tracking_command, tracking_speed
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
    def __init__(self, policy: TrackingPolicy, *, rolling_updates=False):
        if not _valid_policy(policy):
            raise ValueError("invalid execution limits")
        self.policy = policy
        self.rolling_updates = rolling_updates
        self._motion_anchor = None
        self._replacement_direction = None
        self._reproject = False
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
        self._alignment_anchor = None
        self._alignment_direction = None
        self._stop_index = 0
        self._chosen = False
        self.final = True
        self._last_angular_command = 0.0
        self._last_linear_command = 0.0

    def note_angular_command(self, value):
        """Track emitted commands, including node-level stops on stale input."""
        self._last_angular_command = value

    def note_command(self, linear, angular):
        self._last_linear_command = linear
        self.note_angular_command(angular)

    def clear(self):
        self.note_command(0.0, 0.0)
        self.path = None
        self.phase = "WAITING"
        self.failure = None
        self._chosen = False
        self._motion_anchor = None
        self._alignment_anchor = None
        self._alignment_direction = None
        self.no_progress_s = 0.0

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
        same_session = identity is not None and self.identity is not None and identity[0] == self.identity[0]
        if self.rolling_updates and same_session and self.failure is not None:
            # Navigation may have published this revision before receiving our
            # failure. Keep the fault latched but report it with the current ID;
            # only invalidation/clear may restart execution.
            self.identity = identity
            return True
        hot_update = self.rolling_updates and same_session and self.phase == "TRACKING"
        if not (
            self.rolling_updates and same_session
            and self.phase in ("ALIGNING", "BRAKING")
        ):
            self._alignment_anchor = None
            self._alignment_direction = None
        keep_final_alignment = (
            self.rolling_updates and same_session and self.phase == "FINAL_ALIGN"
            and self.final and final and self.path is not None
            and math.dist(path.points[-1], self.path.points[-1]) <= self.policy.goal_position_tolerance_m
            and abs(angle(path.final_yaw - self.path.final_yaw)) <= self.policy.goal_yaw_tolerance_rad
        )
        self._replacement_direction = self.direction if hot_update else None
        self._reproject = hot_update
        if not (self.rolling_updates and same_session):
            self.no_progress_s = 0.0
            self._motion_anchor = None
        self.path = path
        self.identity = identity
        self.final = final
        self.phase = "FINAL_ALIGN" if keep_final_alignment else ("TRACKING" if hot_update else "BRAKING")
        self.progress_m = 0.0
        self.index = 0
        self.error = 0.0
        self.heading_error = 0.0
        self.direction = 1
        self._chosen = False
        self._stop_index = 0
        self.failure = None
        self._best_progress = 0.0
        if not keep_final_alignment:
            self._best_angle = math.inf
        # Each certified bend is an execution boundary. A shallow bend may
        # still protect an obstacle; lookahead must not skip it across the chord.
        # Collinear sampling vertices remain transparent.
        self.stops = path.stop_arcs(min(self.policy.corner_angle_rad, 1e-6))
        return True

    def _result(self, v=0.0, w=0.0) -> ExecutionResult:
        self.note_command(v, w)
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
            math.hypot(state.linear_mps, state.lateral_mps) <= p.stopped_linear_mps
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
        # Slew the command itself. Anchoring at measured rate can prevent
        # braking when plant gain or feedback lag makes measurement exceed
        # the commanded limit; measured speed still governs _stopped().
        previous = self._last_angular_command
        w = max(previous - bound, min(previous + bound, target))
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
                    state.lateral_mps,
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
        initial_reach = None
        if self._reproject:
            initial_reach = math.hypot(state.x_m-line.points[0][0], state.y_m-line.points[0][1]) + p.lookahead_m
        projection = line.project(
            state.x_m,
            state.y_m,
            None if not self._chosen else self.index,
            (
                initial_reach
                if not self._chosen
                else self.progress_m
                + max(p.lookahead_m, abs(state.linear_mps) * dt)
                + 0.25
            ),
        )
        self._reproject = False
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
            # 同一对齐 episode 的等价几何沿用方向，避免倒车阈值抖动翻转目标。
            alignment_direction = self._alignment_direction
            if (
                alignment_direction is not None
                and abs(angle(heading - alignment_direction[0])) <= p.alignment_tolerance_rad
            ):
                self.direction = alignment_direction[1]
            else:
                self.direction = (
                    -1
                    if p.allow_reverse
                    and abs(angle(heading - state.yaw_rad))
                    > p.reverse_heading_threshold_rad
                    else 1
                )
                self._alignment_direction = (heading, self.direction)
            if self._replacement_direction is not None and self.direction != self._replacement_direction:
                self.phase = "BRAKING"
            self._replacement_direction = None
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
                if self.rolling_updates and not self.final:
                    self.phase = "COMPLETED"
                    return self._result()
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
        desired = angle(math.atan2(dy, dx) - (math.pi if self.direction < 0 else 0.0))
        self.heading_error = angle(desired - state.yaw_rad)
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
                self._alignment_anchor = None
                self._alignment_direction = None
                self.no_progress_s = 0.0
                self._best_progress = self.progress_m
            else:
                # 在当前目标下比较实测航向，避免引用修订或微小切线抖动伪造进展。
                # 超过对齐容差的新目标才重建基准；BRAKING 的停稳门槛仍在上方。
                anchor = self._alignment_anchor
                new_target = (
                    anchor is None
                    or abs(angle(desired - anchor[0])) > p.alignment_tolerance_rad
                )
                if new_target or (
                    abs(self.heading_error) < abs(angle(desired - anchor[1])) - 0.01
                ):
                    self._alignment_anchor = (desired, state.yaw_rad)
                    self.no_progress_s = 0.0
                else:
                    self.no_progress_s += dt
                if self.no_progress_s > p.no_progress_timeout_s:
                    self.failure = "ALIGNMENT_STALLED"
                    self.phase = "BRAKING"
                    return self._result()
                return self._rotation(state, self.heading_error, dt,
                                      tolerance=alignment_tolerance)
        if self.rolling_updates:
            position = (state.x_m, state.y_m)
            if self._motion_anchor is None:
                self._motion_anchor = position
            if math.dist(position, self._motion_anchor) > 0.01:
                self._motion_anchor = position
                self.no_progress_s = 0.0
            else:
                self.no_progress_s += dt
        elif self.progress_m > self._best_progress + 0.01:
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
        correction_speed = math.sqrt(2*p.max_linear_decel_mps2*reserve /
                                     lateral_fraction)
        current = max(0.0, self.direction * self._last_linear_command)
        if self.rolling_updates:
            # The regulator is a soft target throughout rolling tracking.
            # Approach it within the braking limit without dropping tracking;
            # stop/corner and physical speed bounds above remain hard limits.
            correction_speed = max(correction_speed,
                                   current-p.max_linear_decel_mps2*dt)
        speed = min(speed, correction_speed)
        if self.rolling_updates:
            command = rolling_tracking_command(k, speed, current, self._last_angular_command, dt, p)
            if command is None:
                self.phase = "BRAKING"
                return self._result()
            return self._result(self.direction * command[0], command[1])
        speed = tracking_speed(k, speed, current, self._last_angular_command, dt, p)

        if speed is None:
            self.phase = "BRAKING"
            return self._result()
        return self._result(self.direction * speed, speed * k)
