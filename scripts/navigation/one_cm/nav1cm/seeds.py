"""有界初始化与受限停车候选；停车永远不是 MPCC 成功。"""
import math
import numpy as np
from .contracts import Trajectory
from .dynamics import step


def brake_candidate(state, grid, reference, limits):
    current = np.array([state.x, state.y, state.yaw, state.v, state.w, reference.progress_s])
    mode = 'spin' if abs(state.v) < 1e-10 else ('forward' if state.v > 0 else 'reverse')
    states, controls, times = [current.copy()], [], [0.0]
    while abs(current[3]) > 1e-10 or abs(current[4]) > 1e-10:
        amax = limits.linear_accel_mps2
        if limits.motion_model == "curvature_limited" and abs(current[3]) > 1e-10 and abs(current[4]) > 1e-10:
            amax = min(amax, limits.angular_accel_rps2 * abs(current[3] / current[4]))
        tv = abs(current[3]) / amax if abs(current[3]) > 1e-10 else math.inf
        tw = abs(current[4]) / limits.angular_accel_rps2 if abs(current[4]) > 1e-10 else math.inf
        dt = min(.02, tv, tw)
        u = np.array([-math.copysign(amax, current[3]) if math.isfinite(tv) else 0.,
                      -math.copysign(limits.angular_accel_rps2, current[4]) if math.isfinite(tw) else 0., 0.])
        current = step(current, u, dt)
        controls.append(u)
        states.append(current)
        times.append(times[-1] + dt)
    return Trajectory(grid.epoch, reference.route_version, np.array(times), np.array(states),
                      np.array(controls).reshape(-1, 3), mode, 'brake')


def braking_prefix(state, grid, reference, limits, *, retain_angular=False):
    """真实初态的平移制动；差速车可保留合法角速度，不伪造零速测量。"""
    if retain_angular and limits.motion_model=='differential_drive' and abs(state.v)>1e-10:
        initial=np.array([state.x,state.y,state.yaw,state.v,state.w,reference.progress_s])
        duration=abs(state.v)/limits.linear_accel_mps2
        control=np.array([-state.v/duration,0.,0.])
        endpoint=step(initial,control,duration)
        return Trajectory(grid.epoch,reference.route_version,[0.,duration],
            np.stack((initial,endpoint)),control[None,:],
            'forward' if state.v>0 else 'reverse','brake')
    return brake_candidate(state,grid,reference,limits)


def join_trajectories(prefix, continuation):
    """接续同一身份、同一物理边界的两段；调用者仍须认证完整结果。"""
    if (prefix.map_epoch!=continuation.map_epoch or prefix.route_version!=continuation.route_version or
            not np.allclose(prefix.states[-1],continuation.states[0],atol=1e-10,rtol=0.)):
        raise ValueError('trajectory continuation identity/state mismatch')
    return Trajectory(prefix.map_epoch,prefix.route_version,
        np.concatenate((prefix.times[:-1],prefix.times[-1]+continuation.times)),
        np.vstack((prefix.states[:-1],continuation.states)),np.vstack((prefix.controls,continuation.controls)),
        continuation.mode,continuation.source if continuation.source=='brake' else continuation.source+'+braking-entry')


def rollout(initial, controls, dt=.2):
    states = [np.asarray(initial, dtype=float)]
    for control in controls:
        states.append(step(states[-1], control, dt))
    return np.asarray(states)


def initial_seed(initial, mode, limits, yaw_target, side=0, n=20, dt=.2):
    """动态可行的前进/后退/转向初始化；左右各最多一个。"""
    current = np.asarray(initial, dtype=float)
    states, controls = [current.copy()], []
    sign = -1 if mode == 'reverse' else 1
    for k in range(n):
        target_v = 0. if mode == 'spin' else sign * limits.linear_speed_mps * .85
        heading_error = yaw_target - current[2] + side * .3 * np.sin(np.pi * k / n)
        turn_limit = min(.08, limits.angular_speed_rps)
        target_w = max(-turn_limit, min(turn_limit, heading_error))
        if mode != 'spin' and limits.motion_model == 'curvature_limited':
            target_w = max(-abs(target_v)*limits.curvature_inv_m, min(abs(target_v)*limits.curvature_inv_m, target_w))
        a = max(-limits.linear_accel_mps2*.8, min(limits.linear_accel_mps2*.8, (target_v-current[3])/dt))
        alpha = max(-limits.angular_accel_rps2*.8, min(limits.angular_accel_rps2*.8, (target_w-current[4])/dt))
        if mode != 'spin' and limits.motion_model == 'curvature_limited':
            next_v = current[3]+a*dt
            next_w = max(-abs(next_v)*limits.curvature_inv_m*.95, min(abs(next_v)*limits.curvature_inv_m*.95, current[4]+alpha*dt))
            alpha=(next_w-current[4])/dt
        u=np.array([a, alpha, 0. if mode=='spin' else abs(current[3]+a*dt/2)])
        current=step(current,u,dt)
        controls.append(u)
        states.append(current)
    return np.asarray(states),np.asarray(controls)
