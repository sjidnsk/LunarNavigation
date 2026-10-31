"""标准 Path 的有限参考缓冲；canonical source、物理进度和派生轨迹分离。

只有经过当前图认证的新 worker 结果才能更换 source。普通执行周期只按
实测 pose/yaw 消费固定 source，并生成受限桥接；派生时间和 bridge 永不
成为下一次的来源。这里不代替 certify，调用方仍须认证整段及制动尾段。
"""
from dataclasses import dataclass
from types import MappingProxyType, SimpleNamespace
import math

import numpy as np

from .contracts import Trajectory, VehicleState
from .dynamics import step
from .seeds import brake_candidate
from .trajectory_buffer import TrajectoryBuffer


def _travel(speed, acceleration, duration):
    end = speed + acceleration * duration
    if speed * end < 0 and abs(acceleration) > 1e-15:
        return (speed * speed + end * end) / (2 * abs(acceleration))
    return abs((speed + end) * duration / 2)


def _at(source, phase):
    index = min(max(0, int(np.searchsorted(source.times, phase, side='right') - 1)), len(source.controls) - 1)
    expected = source.states[index].copy()
    duration = float(phase - source.times[index])
    if duration > 1e-12:
        expected = step(expected, source.controls[index], duration)
    return expected, index


def _project(source, observed, floor, tolerance):
    """只以几何/yaw 投影；参考速度不同不代表消费进度不同。"""
    scale = np.maximum([tolerance.position_m, tolerance.position_m, tolerance.yaw_rad], 1e-9)

    def score(phase):
        expected, _ = _at(source, phase)
        delta = observed[:3] - expected[:3]
        delta[2] = math.atan2(math.sin(delta[2]), math.cos(delta[2]))
        return float(np.sum((delta / scale) ** 2))

    best = (float('inf'), floor, 0)
    for index, control in enumerate(source.controls):
        lo, hi = max(float(source.times[index]), floor), float(source.times[index + 1])
        if lo > hi:
            continue
        start, _ = _at(source, lo)
        delta = observed[:3] - start[:3]
        delta[2] = math.atan2(math.sin(delta[2]), math.cos(delta[2]))
        vector = source.states[index + 1, :3] - start[:3]
        vector[2] = math.atan2(math.sin(vector[2]), math.cos(vector[2]))
        vector /= scale
        norm = float(vector @ vector)
        fraction = 0. if norm <= 1e-20 else float(np.clip((delta / scale) @ vector / norm, 0., 1.))
        for phase in (lo, hi, lo + fraction * (hi - lo)):
            value = score(phase)
            if value < best[0] - 1e-12:
                best = value, phase, index
    _, phase, index = best
    lo, hi = max(float(source.times[index]), floor), float(source.times[index + 1])
    # 只细化已选的小段；精确 step 的加速位移不是线性插值。
    ratio = (math.sqrt(5.) - 1.) / 2
    left, right = hi - ratio * (hi - lo), lo + ratio * (hi - lo)
    fl, fr = score(left), score(right)
    for _ in range(24):
        if fl < fr:
            hi, right, fr = right, left, fl
            left = hi - ratio * (hi - lo); fl = score(left)
        else:
            lo, left, fl = left, right, fr
            right = lo + ratio * (hi - lo); fr = score(right)
    refined = (lo + hi) / 2
    return max(floor, refined if score(refined) < best[0] - 1e-12 else phase)


def _remaining(source, phase):
    expected, index = _at(source, phase)
    distance = angle = 0.
    for j in range(index, len(source.controls)):
        start = phase if j == index else float(source.times[j])
        dt = float(source.times[j + 1] - start)
        state = expected if j == index else source.states[j]
        distance += _travel(state[3], source.controls[j, 0], dt)
        angle += _travel(state[4], source.controls[j, 1], dt)
    return distance, angle


@dataclass(frozen=True)
class ReferenceProposal:
    source: object
    identity: object
    phase_s: float
    candidate: object
    alignment: object
    error: object


class ReferenceTrajectoryBuffer:
    def __init__(self, tolerance=None):
        self._conditioner = TrajectoryBuffer(tolerance)
        self.tolerance = self._conditioner.tolerance
        self.clear()

    def clear(self):
        self.source = self.candidate = None
        self.source_identity = MappingProxyType({})
        self.source_phase_s = 0.
        self.source_accepted_s = self.accepted_s = None
        self.last_alignment = self.last_error = None

    def install(self, source, now_s):
        """显式安装一个新来源；仅供已认证调用方，普通 tick 禁止调用。"""
        self.source, self.candidate = source, source
        self.source_identity = MappingProxyType({})
        self.source_phase_s = 0.
        self.source_accepted_s = self.accepted_s = now_s

    def remaining_s(self, now_s):
        return 0. if self.source is None else max(0., float(self.source.times[-1]) - self.source_phase_s)

    def prepare_source(self, source, state, minimum_s, limits, *, identity=None):
        return self._prepare(source, state, 0., minimum_s, limits, identity or {})

    def install_source(self, proposal, candidate, now_s):
        """只在 proposal 的派生结果已经通过当前图认证后更换 canonical。"""
        self.source = proposal.source
        self.source_identity = MappingProxyType(dict(proposal.identity))
        self.source_phase_s = proposal.phase_s
        self.source_accepted_s = now_s
        self.last_alignment, self.last_error = proposal.alignment, proposal.error
        self.update_derived(candidate, now_s)

    def update_derived(self, candidate, now_s):
        self.candidate, self.accepted_s = candidate, now_s

    def current(self, state, now_s, minimum_s, *, limits):
        if self.source is None:
            raise ValueError('BUFFER_EMPTY')
        proposal = self._prepare(self.source, state, self.source_phase_s, minimum_s, limits, self.source_identity)
        self.source_phase_s = proposal.phase_s
        self.last_alignment = dict(proposal.alignment, source_wall_age_s=now_s-self.source_accepted_s)
        self.last_error = proposal.error
        return proposal.candidate

    def _prepare(self, source, state, floor, minimum_s, limits, identity):
        # 复用既有形状/有限值校验；不让 wall-age 改变 source 控制相位。
        if (not np.isfinite(minimum_s) or minimum_s <= 0 or len(source.times) < 2 or
                len(source.times) != len(source.controls) + 1 or
                len(source.states) != len(source.times) or abs(source.times[0]) > 1e-9 or
                not np.all(np.diff(source.times) > 0) or not np.isfinite(source.times).all() or
                not np.isfinite(source.states).all() or not np.isfinite(source.controls).all()):
            raise ValueError('INVALID_BUFFER_TIME_OR_SHAPE')
        observed = np.array([state.x, state.y, state.yaw, state.v, state.w])
        phase = _project(source, observed, floor, self.tolerance)
        if source.times[-1] - phase <= 1e-9:
            raise ValueError('BUFFER_SOURCE_EXHAUSTED')
        distance, angle = _remaining(source, phase)
        active_distance, active_angle = distance > 1e-10, angle > 1e-10
        if not active_distance and not active_angle:
            raise ValueError('BUFFER_SOURCE_EXHAUSTED')
        raw = self._conditioner.reanchor(source, state, 1e-9, limits,
                                        source_phase_s=phase, reference_velocity=True)
        alignment = dict(self._conditioner.last_alignment, request_identity=dict(identity),
                         source_phase_floor_s=floor, remaining_source_distance_m=distance,
                         remaining_source_angle_rad=angle, remaining_source_time_s=float(source.times[-1]-phase))
        derived = self._finite(raw, limits)
        alignment['derived_distance_m'] = sum(_travel(derived.states[i, 3], u[0], dt)
            for i, (u, dt) in enumerate(zip(derived.controls, np.diff(derived.times))))
        alignment['derived_angle_rad'] = sum(_travel(derived.states[i, 4], u[1], dt)
            for i, (u, dt) in enumerate(zip(derived.controls, np.diff(derived.times))))
        return ReferenceProposal(source, dict(identity), phase, derived, alignment, dict(self._conditioner.last_error))

    def _finite(self, raw, limits):
        """保留有限重锚后缀，并显式追加已有制动候选；调用方认证整段。"""
        current = raw.states[-1].copy()
        states = [state.copy() for state in raw.states]
        controls = [control.copy() for control in raw.controls]
        times = list(raw.times)
        tail = brake_candidate(VehicleState(*current[:5]), SimpleNamespace(epoch=raw.map_epoch),
                               SimpleNamespace(route_version=raw.route_version, progress_s=current[5]), limits)
        for control, dt in zip(tail.controls, np.diff(tail.times)):
            current = step(current, control, float(dt))
            states.append(current.copy()); controls.append(control); times.append(times[-1] + float(dt))
        return Trajectory(raw.map_epoch, raw.route_version, times, states, controls, raw.mode, raw.source)
