"""离线固定周期前缀：仅调用公开动力学 step，不外推制动段控制。"""
import numpy as np
from .contracts import Trajectory, VehicleState
from .dynamics import step


def action_prefix(candidate, measured, duration_s=.05):
    observed = np.array([measured.x, measured.y, measured.yaw, measured.v, measured.w])
    if not np.allclose(candidate.states[0, :5], observed, atol=1e-6, rtol=0):
        raise ValueError('INITIAL_STATE_MISMATCH')
    current = candidate.states[0].copy()
    states, controls, times = [current.copy()], [], [0.]
    for index, control in enumerate(candidate.controls):
        remaining = duration_s-times[-1]
        if remaining <= 1e-12: break
        dt = min(float(candidate.times[index+1]-candidate.times[index]), remaining)
        current = step(current, control, dt)
        states.append(current.copy()); controls.append(control); times.append(times[-1]+dt)
    remaining = duration_s-times[-1]
    if remaining > 1e-12:
        if abs(current[3]) > 1e-10 or abs(current[4]) > 1e-10:
            raise ValueError('moving candidate does not cover execution period')
        control = np.zeros(3)
        current = step(current, control, remaining)
        states.append(current); controls.append(control); times.append(duration_s)
    return Trajectory(candidate.map_epoch, candidate.route_version, times, states,
                      controls, candidate.mode, candidate.source)


def execute_action(action, measured):
    """调用方已认证 action；返回实测模型状态，不直接复制候选末态。"""
    current = np.array([measured.x, measured.y, measured.yaw, measured.v, measured.w, action.states[0, 5]])
    if not np.allclose(current[:5], action.states[0,:5], atol=1e-6, rtol=0):
        raise ValueError('INITIAL_STATE_MISMATCH')
    for index, control in enumerate(action.controls):
        current = step(current, control, float(action.times[index+1]-action.times[index]))
    return VehicleState(*current[:5])
