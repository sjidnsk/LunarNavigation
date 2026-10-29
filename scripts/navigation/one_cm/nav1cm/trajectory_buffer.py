"""按逻辑年龄裁剪控制；反馈只作为重积分初态，不复制预测末态。"""
from dataclasses import dataclass
import numpy as np
from .contracts import Trajectory
from .dynamics import step


@dataclass(frozen=True)
class FeedbackTolerance:
    """离线接续容差，非实车标定参数。"""
    position_m: float = .05
    yaw_rad: float = .1
    speed_mps: float = .2
    angular_speed_rps: float = .2

    def __post_init__(self):
        if not all(np.isfinite(v) and v >= 0 for v in vars(self).values()):
            raise ValueError('invalid feedback tolerance')


class TrajectoryBuffer:
    def __init__(self, tolerance=None):
        self.tolerance = tolerance or FeedbackTolerance()
        self.candidate = None
        self.accepted_s = None
        self.last_error = None
        self.last_alignment = None

    def clear(self):
        self.candidate = self.accepted_s = None

    def remaining_s(self, now_s):
        return 0. if self.candidate is None else max(0., float(self.candidate.times[-1])-(now_s-self.accepted_s))

    def install(self, candidate, now_s):
        """仅安装调用方已整段认证的、以当前反馈重积分的轨迹。"""
        self.candidate, self.accepted_s = candidate, now_s

    def current(self, state, now_s, minimum_s):
        if self.candidate is None:
            raise ValueError('BUFFER_EMPTY')
        return self.rebase(self.candidate, state, now_s-self.accepted_s, minimum_s)

    def rebase(self, candidate, state, age_s, minimum_s):
        t, u = candidate.times, candidate.controls
        if (not np.isfinite(minimum_s) or minimum_s <= 0 or not np.isfinite(age_s) or age_s < -1e-9 or len(t) != len(u)+1
                or len(candidate.states) != len(t)
                or len(t)<2 or abs(t[0])>1e-9 or not np.all(np.diff(t)>0)
                or not np.isfinite(t).all() or not np.isfinite(candidate.states).all()
                or not np.isfinite(u).all()):
            raise ValueError('INVALID_BUFFER_TIME_OR_SHAPE')
        age_s=max(0.,age_s)
        if t[-1]-age_s < minimum_s-1e-9:
            raise ValueError('BUFFER_EXHAUSTED')
        index=min(int(np.searchsorted(t,age_s,side='right')-1),len(u)-1)
        expected=candidate.states[index].copy()
        if age_s>t[index]+1e-12:
            expected=step(expected,u[index],float(age_s-t[index]))
        observed=np.array([state.x,state.y,state.yaw,state.v,state.w])
        delta=observed-expected[:5]
        delta[2]=np.arctan2(np.sin(delta[2]),np.cos(delta[2]))
        self.last_error={'position_m':float(np.linalg.norm(delta[:2])), 'yaw_rad':abs(float(delta[2])),
                         'speed_mps':abs(float(delta[3])), 'angular_speed_rps':abs(float(delta[4]))}
        if any(self.last_error[k]>v+1e-12 for k,v in vars(self.tolerance).items()):
            raise ValueError('FEEDBACK_DISCONTINUITY')
        current=np.r_[observed,expected[5]]
        states, controls, times=[current.copy()],[],[0.]
        for i in range(index,len(u)):
            dt=float(t[i+1]-max(t[i],age_s))
            if dt<=1e-12: continue
            current=step(current,u[i],dt)
            states.append(current.copy()); controls.append(u[i]); times.append(times[-1]+dt)
        return Trajectory(candidate.map_epoch,candidate.route_version,times,states,controls,candidate.mode,candidate.source)

    def reanchor(self, candidate, state, minimum_s, limits, *, source_phase_s=None, reference_velocity=False):
        """未执行候选从实测状态衔接，计算耗时不代表其控制已经执行。

        以物理状态寻找来源相位，再用受限 a/alpha 段达到该相位的 v/w，
        从真实初态重积分剩余控制。这里只生成派生候选，调用方仍须认证
        当前图的完整扫掠、动力学、mode 及制动尾段；不裁任何状态数值。
        已执行缓冲的 current/rebase 时间语义保持独立。
        """
        t, u, source_states = candidate.times, candidate.controls, candidate.states
        if (not np.isfinite(minimum_s) or minimum_s <= 0 or len(t) != len(u) + 1 or
                len(source_states) != len(t) or len(t) < 2 or abs(t[0]) > 1e-9 or
                not np.all(np.diff(t) > 0) or not np.isfinite(t).all() or
                not np.isfinite(source_states).all() or not np.isfinite(u).all()):
            raise ValueError('INVALID_BUFFER_TIME_OR_SHAPE')
        if t[-1] < minimum_s - 1e-9:
            raise ValueError('BUFFER_BRAKING_RESERVE')
        observed = np.array([state.x, state.y, state.yaw, state.v, state.w])
        tolerance = self.tolerance
        scale = np.maximum([tolerance.position_m, tolerance.position_m, tolerance.yaw_rad,
                            tolerance.speed_mps, tolerance.angular_speed_rps], 1e-9)
        if source_phase_s is not None:
            if not np.isfinite(source_phase_s) or source_phase_s < -1e-9:
                raise ValueError('INVALID_BUFFER_TIME_OR_SHAPE')
            phase = max(0., float(source_phase_s))
            if t[-1] - phase < minimum_s - 1e-9:
                raise ValueError('BUFFER_EXHAUSTED')
            phases = {phase}
        else:
            maximum_phase = max(0., float(t[-1] - minimum_s))
            phases = {0., maximum_phase}
            phases.update(float(value) for value in t if value <= maximum_phase)
            # 线段投影只生成少量相位候选，评估用真实 step，不线性插值初态。
            for index in range(len(u)):
                delta = observed - source_states[index, :5]
                delta[2] = np.arctan2(np.sin(delta[2]), np.cos(delta[2]))
                vector = source_states[index + 1, :5] - source_states[index, :5]
                vector[2] = np.arctan2(np.sin(vector[2]), np.cos(vector[2]))
                vector = vector / scale
                norm = float(vector @ vector)
                fraction = 0. if norm <= 1e-20 else float(np.clip((delta / scale) @ vector / norm, 0., 1.))
                phases.add(float(min(t[index] + fraction * (t[index + 1] - t[index]), maximum_phase)))
        best = None
        for phase in sorted(phases):
            index = min(int(np.searchsorted(t, phase, side='right') - 1), len(u) - 1)
            expected = source_states[index].copy()
            if phase > t[index] + 1e-12:
                expected = step(expected, u[index], float(phase - t[index]))
            delta = observed - expected[:5]
            delta[2] = np.arctan2(np.sin(delta[2]), np.cos(delta[2]))
            score = float(np.sum((delta / scale) ** 2))
            if best is None or score < best[0] - 1e-12:
                best = score, phase, index, expected, delta
        _, phase, index, expected, delta = best
        self.last_error = {'position_m': float(np.linalg.norm(delta[:2])), 'yaw_rad': abs(float(delta[2])),
                           'speed_mps': abs(float(delta[3])), 'angular_speed_rps': abs(float(delta[4]))}
        checked_errors = ('position_m', 'yaw_rad') if reference_velocity else tuple(vars(tolerance))
        if any(self.last_error[key] > getattr(tolerance, key) + 1e-12 for key in checked_errors):
            raise ValueError('FEEDBACK_DISCONTINUITY')
        current = np.r_[observed, expected[5]]
        states, controls, times = [current.copy()], [], [0.]
        difference = expected[3:5] - observed[3:5]
        bridge_s = max(abs(difference[0]) / limits.linear_accel_mps2,
                       abs(difference[1]) / limits.angular_accel_rps2)
        bridge_control = np.zeros(3)
        if bridge_s > 1e-12:
            bridge_control[:2] = difference / bridge_s
            current = step(current, bridge_control, bridge_s)
            states.append(current.copy()); controls.append(bridge_control); times.append(bridge_s)
        for source_index in range(index, len(u)):
            dt = float(t[source_index + 1] - max(t[source_index], phase))
            if dt <= 1e-12:
                continue
            current = step(current, u[source_index], dt)
            states.append(current.copy()); controls.append(u[source_index]); times.append(times[-1] + dt)
        derived = Trajectory(candidate.map_epoch, candidate.route_version, times, states, controls,
                             candidate.mode, candidate.source)
        self.last_alignment = {'original_candidate_digest': candidate.digest(), 'source_phase_s': phase,
                               'source_state': expected.tolist(), 'actual_initial': observed.tolist(),
                               'bridge_duration_s': bridge_s, 'bridge_control': bridge_control.tolist()}
        return derived
