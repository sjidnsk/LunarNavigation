"""平面目标姿态与受限末端转向；候选仍须经独立整车认证。"""
import math

import numpy as np

from .contracts import Trajectory
from .dynamics import step
from .global_planner import GoalCheck
from .seeds import brake_candidate


def normalize_yaw(yaw):
    """只归一化目标；实测/积分状态的 yaw 保持连续。"""
    yaw = float(yaw)
    if not math.isfinite(yaw):
        raise ValueError('goal yaw must be finite')
    return math.atan2(math.sin(yaw), math.cos(yaw))


def angle_error(target, current):
    """从当前车体朝向到目标的最短有符号转角。"""
    return normalize_yaw(float(target) - float(current))


def check_pose_goal(world, goal_xy, goal_yaw, limits):
    """只因最终矩形车体与已知障碍接触而拒绝，未知/图外均可接收。"""
    xy = tuple(float(value) for value in goal_xy)
    if len(xy) != 2 or not all(math.isfinite(value) for value in xy):
        raise ValueError('goal must contain finite x,y')
    yaw = normalize_yaw(goal_yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = limits.body_length_m / 2, limits.body_width_m / 2
    ex, ey = abs(c) * hl + abs(s) * hw, abs(s) * hl + abs(c) * hw
    for rect in world.blocked:
        x0, y0, x1, y1 = np.asarray((rect.x0, rect.y0, rect.x1, rect.y1)) / 100.
        dx, dy = (x0 + x1) / 2 - xy[0], (y0 + y1) / 2 - xy[1]
        rx, ry = (x1 - x0) / 2, (y1 - y0) / 2
        # 世界两轴与车体两轴的精确矩形分离轴检查；接触亦视为冲突。
        intersects = (abs(dx) <= ex + rx + 1e-12 and abs(dy) <= ey + ry + 1e-12 and
                      abs(c * dx + s * dy) <= hl + abs(c) * rx + abs(s) * ry + 1e-12 and
                      abs(-s * dx + c * dy) <= hw + abs(s) * rx + abs(c) * ry + 1e-12)
        if intersects:
            return GoalCheck(False, 'GOAL_IN_OBSTACLE', world.epoch)
    return GoalCheck(True, 'GOAL_ACCEPTED', world.epoch)


def terminal_candidate(state, grid, reference, limits, *, horizon_s=4.):
    """解析角速度梯形/三角形候选，不把辅助转向冒充 acados 求解成功。

    平移尚未精确停稳时先用实际初态制动。纯转向允许超过滚动时域，
    每段保留真实角速度与角加速度；调用方必须认证整段及停车尾段。
    """
    if reference.terminal_yaw is None:
        raise ValueError('terminal yaw is required')
    if not math.isfinite(horizon_s) or horizon_s <= 0:
        raise ValueError('terminal horizon must be finite and positive')
    error = angle_error(reference.terminal_yaw, state.yaw)
    alpha, maximum = limits.angular_accel_rps2, limits.angular_speed_rps
    direction = 1. if error >= 0 else -1.
    speed = direction * state.w
    distance = abs(error)
    # 停稳判定容差不允许重写实测 v；反向/来不及停下的角速度也先物理制动。
    if (abs(state.v) > 1e-10 or speed < 0 or abs(state.w) > maximum or
            distance <= 1e-12 or speed * speed / (2 * alpha) > distance + 1e-12):
        return brake_candidate(state, grid, reference, limits)
    peak = min(maximum, math.sqrt(alpha * distance + speed * speed / 2))
    accelerate_s = max(0., (peak - speed) / alpha)
    acceleration_distance = (peak * peak - speed * speed) / (2 * alpha)
    deceleration_distance = peak * peak / (2 * alpha)
    cruise_s = max(0., (distance - acceleration_distance - deceleration_distance) / peak)
    phases = ((accelerate_s, direction * alpha), (cruise_s, 0.),
              (peak / alpha, -direction * alpha))
    current = np.array([state.x, state.y, state.yaw, state.v, state.w, reference.progress_s])
    states, controls, times = [current.copy()], [], [0.]
    for duration, acceleration in phases:
        remaining = min(duration, max(0., horizon_s - times[-1]))
        while remaining > 1e-12:
            dt = min(.1, remaining)
            control = np.array([0., acceleration, 0.])
            current = step(current, control, dt)
            states.append(current.copy())
            controls.append(control)
            times.append(times[-1] + dt)
            remaining -= dt
        if times[-1] >= horizon_s - 1e-12:
            break
    return Trajectory(grid.epoch, reference.route_version, times, states,
                      np.asarray(controls).reshape(-1, 3), 'spin', 'terminal-yaw:bounded')
