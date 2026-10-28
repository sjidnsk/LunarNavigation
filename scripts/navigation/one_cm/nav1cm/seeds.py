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
        if abs(current[3]) > 1e-10 and abs(current[4]) > 1e-10:
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
        target_w = max(-.08, min(.08, heading_error))
        if mode != 'spin':
            target_w = max(-abs(target_v)*limits.curvature_inv_m, min(abs(target_v)*limits.curvature_inv_m, target_w))
        a = max(-limits.linear_accel_mps2*.8, min(limits.linear_accel_mps2*.8, (target_v-current[3])/dt))
        alpha = max(-limits.angular_accel_rps2*.8, min(limits.angular_accel_rps2*.8, (target_w-current[4])/dt))
        if mode != 'spin':
            next_v = current[3]+a*dt
            next_w = max(-abs(next_v)*limits.curvature_inv_m*.95, min(abs(next_v)*limits.curvature_inv_m*.95, current[4]+alpha*dt))
            alpha=(next_w-current[4])/dt
        u=np.array([a, alpha, 0. if mode=='spin' else abs(current[3]+a*dt/2)])
        current=step(current,u,dt)
        controls.append(u)
        states.append(current)
    return np.asarray(states),np.asarray(controls)
