"""不依赖求解器状态的整车轨迹与停车尾段认证。"""

import math
from dataclasses import dataclass, replace

import numpy as np

from .contracts import Certification, CertificationReason, CellState, GridSnapshot, Trajectory, WheelLimits
from .dynamics import step


def _reject(reason, checked, detail=""):
    return Certification(False, reason, checked_poses=checked, detail=detail)


@dataclass(frozen=True)
class _CollisionContext:
    """同一完整地图的多次认证共享不可写局部危险格前缀计数。"""

    grid: GridSnapshot
    hazard_prefix: np.ndarray
    sparse_rows: object = None
    sparse_cols: object = None
    backend: object = None
    grid_digest: str = ""

    @classmethod
    def build(cls, grid: GridSnapshot, backend=None):
        hazard = grid.cells == CellState.BLOCKED if grid.allow_unknown else grid.cells != CellState.FREE
        counts = hazard.cumsum(axis=0, dtype=np.int32).cumsum(axis=1, dtype=np.int32)
        padded = np.pad(counts, ((1, 0), (1, 0)))
        prefix = np.frombuffer(padded.tobytes(), dtype=np.int32).reshape(padded.shape)
        # 稀疏薄障碍只枚举一次原始格；密集未知图仍用原有局部面积扫描。
        if int(counts[-1,-1]) <= 4096:
            rows, cols = np.nonzero(hazard)
            rows = np.frombuffer(rows.tobytes(), dtype=rows.dtype)
            cols = np.frombuffer(cols.tobytes(), dtype=cols.dtype)
            return cls(grid, prefix, rows, cols, backend, grid.digest())
        return cls(grid, prefix, backend=backend, grid_digest=grid.digest())

    def has_hazard(self, x0, y0, x1, y1):
        p = self.hazard_prefix
        return bool(p[y1, x1] - p[y0, x1] - p[y1, x0] + p[y0, x0])


def _body_collision(pose, radius_margin, context, limits):
    """矩形车体与候选格面积做分离轴检验，窗口外按阻断处理。"""
    if context.backend is not None:
        return context.backend.check_poses(np.asarray(pose[:3]).reshape(1,3),radius_margin,context,limits)[0]
    grid = context.grid
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = math.cos(yaw), math.sin(yaw)
    half_length = limits.body_length_m / 2 + radius_margin
    half_width = limits.body_width_m / 2 + radius_margin
    extent_x = abs(c) * half_length + abs(s) * half_width
    extent_y = abs(s) * half_length + abs(c) * half_width
    ox, oy = grid.origin_xy
    resolution = grid.resolution_m
    height, width = grid.cells.shape
    if (x - extent_x < ox - 1e-12 or x + extent_x > ox + width * resolution + 1e-12 or
            y - extent_y < oy - 1e-12 or y + extent_y > oy + height * resolution + 1e-12):
        return CertificationReason.OUT_OF_BOUNDS
    ix0 = max(0, int(math.floor(math.nextafter(x - extent_x - ox, -math.inf) / resolution)))
    ix1 = min(width, int(math.floor((x + extent_x - ox) / resolution)) + 1)
    iy0 = max(0, int(math.floor(math.nextafter(y - extent_y - oy, -math.inf) / resolution)))
    iy1 = min(height, int(math.floor((y + extent_y - oy) / resolution)) + 1)
    if not context.has_hazard(ix0, iy0, ix1, iy1):
        return None
    if context.sparse_rows is not None:
        keep = ((context.sparse_rows >= iy0) & (context.sparse_rows < iy1) &
                (context.sparse_cols >= ix0) & (context.sparse_cols < ix1))
        rows, cols = context.sparse_rows[keep], context.sparse_cols[keep]
    else:
        area = grid.cells[iy0:iy1, ix0:ix1]
        suspicious = area != CellState.FREE if not grid.allow_unknown else area == CellState.BLOCKED
        rows, cols = np.nonzero(suspicious)
        rows, cols = rows + iy0, cols + ix0
    if len(rows) == 0:
        return None
    dx = ox + (cols + 0.5) * resolution - x
    dy = oy + (rows + 0.5) * resolution - y
    half_cell = resolution / 2
    # 世界 x/y 两轴已由候选包围盒覆盖；车体两轴完成矩形 SAT。
    projection = half_cell * (abs(c) + abs(s))
    intersects = (np.abs(c * dx + s * dy) <= half_length + projection + 1e-12) & (
        np.abs(-s * dx + c * dy) <= half_width + projection + 1e-12
    )
    if not intersects.any():
        return None
    relevant = grid.cells[rows[intersects], cols[intersects]]
    if np.any(relevant == CellState.BLOCKED):
        return CertificationReason.COLLISION
    return CertificationReason.UNKNOWN


def _batch_body_collision(poses, radius_margin, context, limits):
    """批量执行同一轴对齐粗筛；疑似碰撞仍交给原矩形 SAT。"""
    if context.backend is not None:
        return context.backend.check_poses(poses,radius_margin,context,limits)
    grid = context.grid
    x, y, yaw = poses[:,0], poses[:,1], poses[:,2]
    c, s = np.cos(yaw), np.sin(yaw)
    half_length = limits.body_length_m/2 + radius_margin
    half_width = limits.body_width_m/2 + radius_margin
    # 粗筛包围盒额外覆盖浮点舍入；最终仍用原标量尺寸及阈值。
    ex = np.abs(c)*half_length + np.abs(s)*half_width + 1e-12
    ey = np.abs(s)*half_length + np.abs(c)*half_width + 1e-12
    ox, oy = grid.origin_xy; r = grid.resolution_m
    h, w = grid.cells.shape
    outside = (x-ex < ox-1e-12) | (x+ex > ox+w*r+1e-12) | (y-ey < oy-1e-12) | (y+ey > oy+h*r+1e-12)
    x0 = np.clip(np.floor(np.nextafter(x-ex-ox,-np.inf)/r).astype(np.int64),0,w)
    x1 = np.clip(np.floor((x+ex-ox)/r).astype(np.int64)+1,0,w)
    y0 = np.clip(np.floor(np.nextafter(y-ey-oy,-np.inf)/r).astype(np.int64),0,h)
    y1 = np.clip(np.floor((y+ey-oy)/r).astype(np.int64)+1,0,h)
    p = context.hazard_prefix
    hazard = p[y1,x1]-p[y0,x1]-p[y1,x0]+p[y0,x0] > 0
    for i in np.flatnonzero(outside | hazard):
        # 包括窗口边界也复用标量判定，保留原阈值和返回优先级。
        reason = _body_collision(poses[i],radius_margin,context,limits)
        if reason is not None:
            return reason, int(i)+1
    return None, len(poses)


def _state_limit_reason(state, limits, mode):
    v, w = float(state[3]), float(state[4])
    eps = 1e-9
    if abs(v) > limits.linear_speed_mps + eps or abs(w) > limits.angular_speed_rps + eps:
        return CertificationReason.LIMIT
    if mode == "spin":
        return None if abs(v) <= eps else CertificationReason.MODE
    if mode == "forward" and v < -eps:
        return CertificationReason.MODE
    if mode == "reverse" and v > eps:
        return CertificationReason.MODE
    if abs(w) > limits.curvature_inv_m * abs(v) + eps:
        return CertificationReason.LIMIT
    return None


def _sweep(start, control, dt, context, limits):
    """采样矩形加运动上界外包；两样本之间无未覆盖位姿。"""
    v0, w0 = abs(float(start[3])), abs(float(start[4]))
    vmax = max(v0, abs(float(start[3] + control[0] * dt)))
    wmax = max(w0, abs(float(start[4] + control[1] * dt)))
    body_radius = math.hypot(limits.body_length_m / 2, limits.body_width_m / 2)
    # 轮心速度与角点切向速度之和是车体任一点速度上界。
    # 中点插值位置对时间求导的额外转向项上界为 vmax*dt*wmax/2。
    body_speed = vmax * (1.0 + dt * wmax / 2.0) + body_radius * wmax
    subdivisions = max(1, int(math.ceil(body_speed * dt / 0.005)))
    interval = dt / subdivisions
    # 中点插值路径上任一点距最近样本最多半个小区间的运动上界。
    radius_margin = body_speed * interval / 2 + 1e-6
    if subdivisions > 2:
        # 与 step(start, control, t) 完全相同的中点离散表达式及原采样间隔。
        t = np.arange(subdivisions + 1, dtype=float)*interval
        vm = start[3] + .5*control[0]*t
        wm = start[4] + .5*control[1]*t
        ym = start[2] + .5*wm*t
        poses = np.column_stack((start[0]+vm*np.cos(ym)*t,
                                 start[1]+vm*np.sin(ym)*t,start[2]+wm*t))
        return _batch_body_collision(poses,radius_margin,context,limits)
    for index in range(subdivisions + 1):
        pose = start if index == 0 else step(start, control, index * interval)
        reason = _body_collision(pose, radius_margin, context, limits)
        if reason is not None:
            return reason, index + 1
    return None, subdivisions + 1


def _brake_tail(last, context, limits, mode):
    """用额定最大减速度分别收敛 v/w，尾段每步再做连续扫掠。"""
    current = np.array(last, dtype=np.float64)
    checked = 0
    elapsed = 0.0
    while abs(current[3]) > 1e-10 or abs(current[4]) > 1e-10:
        a_capacity = limits.linear_accel_mps2
        if abs(current[3]) > 1e-10 and abs(current[4]) > 1e-10:
            # 行进时角速度不能在线速度归零后继续存在；限制线制动使 w/v 不增。
            a_capacity = min(a_capacity, limits.angular_accel_rps2 * abs(current[3]) / abs(current[4]))
        v_time = abs(current[3]) / a_capacity if abs(current[3]) > 1e-10 else math.inf
        w_time = abs(current[4]) / limits.angular_accel_rps2 if abs(current[4]) > 1e-10 else math.inf
        dt = min(0.02, v_time, w_time)
        a = -math.copysign(a_capacity, current[3]) if math.isfinite(v_time) else 0.0
        alpha = -math.copysign(limits.angular_accel_rps2, current[4]) if math.isfinite(w_time) else 0.0
        control = np.array([a, alpha, 0.0])
        midpoint = step(current, control, dt / 2)
        endpoint = step(current, control, dt)
        if (_state_limit_reason(midpoint, limits, mode) is not None or
                _state_limit_reason(endpoint, limits, mode) is not None):
            return CertificationReason.LIMIT, elapsed, checked
        reason, count = _sweep(current, control, dt, context, limits)
        checked += count
        if reason is not None:
            return reason, elapsed, checked
        current = endpoint
        if abs(current[3]) < 1e-10:
            current[3] = 0.0
        if abs(current[4]) < 1e-10:
            current[4] = 0.0
        elapsed += dt
    return None, elapsed, checked


def _certify_impl(trajectory: Trajectory, grid: GridSnapshot, limits: WheelLimits,
                  current_route_version: int, backend=None, context=None) -> Certification:
    """版本、数值、动力学、约束、整车连续扫掠与制动尾段全部通过才有效。"""
    if trajectory.map_epoch != grid.epoch:
        return _reject(CertificationReason.EPOCH_MISMATCH, 0)
    if trajectory.route_version != current_route_version:
        return _reject(CertificationReason.ROUTE_VERSION_MISMATCH, 0)
    times, states, controls = trajectory.times, trajectory.states, trajectory.controls
    if len(states) < 1 or len(times) != len(states) or len(controls) != len(states) - 1:
        return _reject(CertificationReason.INVALID_SHAPE, 0)
    if not np.isfinite(times).all() or not np.isfinite(states).all() or not np.isfinite(controls).all():
        return _reject(CertificationReason.NONFINITE, 0)
    if np.any(np.diff(times) <= 0):
        return _reject(CertificationReason.TIME_NONMONOTONIC, 0)
    if trajectory.mode not in ("forward", "reverse", "spin"):
        return _reject(CertificationReason.MODE, 0)
    if np.any(np.abs(controls[:, 0]) > limits.linear_accel_mps2 + 1e-9) or np.any(
        np.abs(controls[:, 1]) > limits.angular_accel_rps2 + 1e-9
    ) or np.any(controls[:, 2] < -1e-9):
        return _reject(CertificationReason.LIMIT, 0)
    if trajectory.mode == "spin" and np.any(np.abs(controls[:, 0]) > 1e-9):
        return _reject(CertificationReason.MODE, 0)
    if context is None:
        context = prepare_certification(grid, backend=backend)
    checked = 0
    for index, state in enumerate(states):
        reason = _state_limit_reason(state, limits, trajectory.mode)
        if reason is not None:
            return _reject(reason, checked)
        if index == len(states) - 1:
            break
        dt = float(times[index + 1] - times[index])
        expected = step(state, controls[index], dt)
        if not np.allclose(expected, states[index + 1], atol=1e-6, rtol=0.0):
            return _reject(CertificationReason.DYNAMICS, checked)
        # v(t) 过零处仍有 w 会违反行进曲率；端点检查不足以发现。
        a = controls[index, 0]
        if trajectory.mode != "spin" and abs(a) > 1e-12:
            crossing = -state[3] / a
            if 0 < crossing < dt and abs(state[4] + controls[index, 1] * crossing) > 1e-9:
                return _reject(CertificationReason.LIMIT, checked)
        midpoint = step(state, controls[index], dt / 2)
        reason = _state_limit_reason(midpoint, limits, trajectory.mode)
        if reason is not None:
            return _reject(reason, checked)
        collision, count = _sweep(state, controls[index], dt, context, limits)
        checked += count
        if collision is not None:
            return _reject(collision, checked)
    if len(states) == 1:
        collision = _body_collision(states[0], 0.0, context, limits)
        checked += 1
        if collision is not None:
            return _reject(collision, checked)
    collision, brake_time, count = _brake_tail(states[-1], context, limits, trajectory.mode)
    checked += count
    if collision is not None:
        braking_reasons = {
            CertificationReason.COLLISION: CertificationReason.BRAKING_COLLISION,
            CertificationReason.UNKNOWN: CertificationReason.BRAKING_UNKNOWN,
            CertificationReason.OUT_OF_BOUNDS: CertificationReason.BRAKING_OUT_OF_BOUNDS,
            CertificationReason.LIMIT: CertificationReason.BRAKING_LIMIT,
        }
        return Certification(False, braking_reasons[collision], brake_time, checked, collision.value)
    return Certification(True, CertificationReason.VALID, brake_time, checked)


def prepare_certification(grid: GridSnapshot, *, backend=None):
    """为同一完整地图和碰撞后端准备不可变危险格索引。"""
    return _CollisionContext.build(grid, backend)


def certify(trajectory: Trajectory, grid: GridSnapshot, limits: WheelLimits,
            current_route_version: int, *, backend=None, context=None) -> Certification:
    """返回与完整地图、车体约束、路线版本和轨迹内容绑定的认证。"""
    if context is not None:
        if not isinstance(context, _CollisionContext) or context.grid_digest != grid.digest():
            raise ValueError("certification context grid mismatch")
        if context.backend is not backend:
            raise ValueError("certification context backend mismatch")
    result = _certify_impl(trajectory, grid, limits, current_route_version, backend, context)
    return replace(result, map_epoch=trajectory.map_epoch,
                   route_version=trajectory.route_version,
                   trajectory_source=trajectory.source,
                   trajectory_digest=trajectory.digest(),
                   grid_digest=grid.digest(), limits_digest=limits.digest())
