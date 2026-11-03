"""不依赖求解器状态的整车轨迹与停车尾段认证。"""

import math
from dataclasses import dataclass, replace

import numpy as np

from .contracts import Certification, CertificationReason, CellState, GridSnapshot, Trajectory, WheelLimits, circumscribed_radius
from .dynamics import step
from .rectangle_distance import rectangle_distance_squared, point_box_distance_squared, segment_clear


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
    """生产圆或零净空原型矩形到原始格面积的距离；窗口外阻断。"""
    if context.grid.obstacle_boxes_m is not None and limits.minimum_clearance_m>0:
        grid=context.grid;x,y=map(float,pose[:2]);ox,oy=grid.origin_xy;h,w=grid.cells.shape
        radius=circumscribed_radius(limits.body_length_m,limits.body_width_m,limits.minimum_clearance_m)+radius_margin
        if min(x-ox,y-oy,ox+w*grid.resolution_m-x,oy+h*grid.resolution_m-y)<radius:
            return CertificationReason.OUT_OF_BOUNDS
        if np.any(point_box_distance_squared((x,y),grid.obstacle_boxes_m)<=radius*radius):
            return CertificationReason.COLLISION
        return None
    if context.backend is not None:
        return context.backend.check_poses(np.asarray(pose[:3]).reshape(1,3),radius_margin,context,limits)[0]
    grid = context.grid
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = math.cos(yaw), math.sin(yaw)
    # 数值扫掠余量在公共几何半径之外单独叠加一次。
    circle = limits.minimum_clearance_m > 0
    radius = (circumscribed_radius(limits.body_length_m, limits.body_width_m,
                                  limits.minimum_clearance_m) + radius_margin) if circle else 0.
    legacy_padding = 0. if circle else radius_margin
    half_length = limits.body_length_m / 2 + legacy_padding
    half_width = limits.body_width_m / 2 + legacy_padding
    extent_x = radius if circle else abs(c) * half_length + abs(s) * half_width
    extent_y = radius if circle else abs(s) * half_length + abs(c) * half_width
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
    if circle:
        qx, qy = np.maximum(np.abs(dx)-half_cell, 0.), np.maximum(np.abs(dy)-half_cell, 0.)
        intersects = qx*qx + qy*qy <= (radius+1e-12)**2
    else:
        distance2=rectangle_distance_squared(dx,dy,half_cell,half_cell,c,s,half_length,half_width)
        intersects=distance2 <= 1e-24
    if not intersects.any():
        return None
    relevant = grid.cells[rows[intersects], cols[intersects]]
    if np.any(relevant == CellState.BLOCKED):
        return CertificationReason.COLLISION
    return CertificationReason.UNKNOWN


def _batch_body_collision(poses, radius_margin, context, limits):
    """批量执行轴对齐粗筛；疑似碰撞复用标量精确几何。"""
    if context.grid.obstacle_boxes_m is not None and limits.minimum_clearance_m>0:
        for i,pose in enumerate(poses):
            reason=_body_collision(pose,radius_margin,context,limits)
            if reason is not None:return reason,i+1
        return None,len(poses)
    if context.backend is not None:
        return context.backend.check_poses(poses,radius_margin,context,limits)
    grid = context.grid
    x, y, yaw = poses[:,0], poses[:,1], poses[:,2]
    c, s = np.cos(yaw), np.sin(yaw)
    legacy_padding=radius_margin if limits.minimum_clearance_m==0 else 0.
    circle = limits.minimum_clearance_m > 0
    radius = (circumscribed_radius(limits.body_length_m, limits.body_width_m,
                                  limits.minimum_clearance_m) + radius_margin) if circle else 0.
    half_length = limits.body_length_m/2 + legacy_padding
    half_width = limits.body_width_m/2 + legacy_padding
    # 粗筛包围盒额外覆盖浮点舍入；最终仍用原标量尺寸及阈值。
    ex = (np.full_like(x, radius) if circle else np.abs(c)*half_length + np.abs(s)*half_width) + 1e-12
    ey = (np.full_like(y, radius) if circle else np.abs(s)*half_length + np.abs(c)*half_width) + 1e-12
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
    if limits.motion_model == "curvature_limited" and abs(w) > limits.curvature_inv_m * abs(v) + eps:
        return CertificationReason.LIMIT
    return None


def _sweep(start, control, dt, context, limits):
    """采样车体包络加运动上界余量；两样本之间无未覆盖位姿。"""
    if context.grid.obstacle_boxes_m is not None and limits.minimum_clearance_m>0:
        # 圆的朝向不改变碰撞形状；直接检查中心运动线段。
        # 弯曲离散积分用二阶导数界覆盖弦偏差，不叠加角点扫掠膨胀。
        v=abs(float(start[3]));a=abs(float(control[0]));alpha=abs(float(control[1]))
        vmax=max(v,abs(float(start[3]+control[0]*dt)))
        wmax=max(abs(float(start[4])),abs(float(start[4]+control[1]*dt)))
        curvature_bound=a+vmax*wmax+vmax*dt*(.5*alpha+.25*wmax*wmax)
        straight=wmax==0 or vmax==0
        count=1 if straight else max(1,int(math.ceil(dt*math.sqrt(curvature_bound/(8e-5)))))
        deviation=0. if straight else curvature_bound*(dt/count)**2/8
        radius=circumscribed_radius(limits.body_length_m,limits.body_width_m,limits.minimum_clearance_m)
        raw=np.asarray(context.grid.obstacle_boxes_m,dtype=float).reshape(-1,4)
        boxes=raw+np.array([-radius-deviation,-radius-deviation,radius+deviation,radius+deviation])
        previous=start
        for i in range(1,count+1):
            current=step(start,control,dt*i/count)
            # 端点/弦到窗口边界也保持同一圆半径。
            for pose in (previous,current):
                reason=_body_collision(pose,deviation,context,limits)
                if reason is not None:return reason,i
            if not segment_clear(previous[:2],current[:2],boxes,radius+deviation):
                return CertificationReason.COLLISION,i
            previous=current
        return None,count+1
    v0, w0 = abs(float(start[3])), abs(float(start[4]))
    vmax = max(v0, abs(float(start[3] + control[0] * dt)))
    wmax = max(w0, abs(float(start[4] + control[1] * dt)))
    body_radius = circumscribed_radius(limits.body_length_m,limits.body_width_m,limits.minimum_clearance_m)
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
        if limits.motion_model == "curvature_limited" and abs(current[3]) > 1e-10 and abs(current[4]) > 1e-10:
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
                  current_route_version: int, backend=None, context=None, check_braking_tail=True) -> Certification:
    """检查版本、数值、动力学、约束与整车连续扫掠；默认还检查末端制动尾。"""
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
    # 模式开始前可含显式、单调且无进展信用的真实制动段。
    # 起点测量不归零；到零速结点后才按主模式检查，所有段仍完整积分和扫掠。
    prefix_end=0;prefix_mode=trajectory.mode
    initial_reason=_state_limit_reason(states[0],limits,trajectory.mode)
    needs_entry=(initial_reason==CertificationReason.MODE or
                 trajectory.mode=='spin' and np.any(np.abs(controls[:,0])>1e-9))
    if needs_entry:
        zero=np.flatnonzero(np.abs(states[1:,3])<=1e-9)
        if states[0,3]==0. or not len(controls):
            return _reject(CertificationReason.MODE,0)
        # 截断段可在制动尚未完成时结束；该段末端和制动尾仍按实际方向认证。
        prefix_end=int(zero[0])+1 if len(zero) else len(states)
        sign=float(np.sign(states[0,3]));speeds=states[:prefix_end+1,3]
        if (np.any(sign*speeds<-1e-9) or np.any(np.diff(np.abs(speeds))>1e-12) or
                np.any(sign*controls[:prefix_end,0]>1e-12) or
                np.any(np.abs(controls[:prefix_end,2])>1e-12)):
            return _reject(CertificationReason.MODE,0)
        prefix_mode='forward' if sign>0 else 'reverse'
        if prefix_end < len(states):
            reason=_state_limit_reason(states[prefix_end],limits,prefix_mode)
            if reason is not None:return _reject(reason,0)
    if trajectory.mode == 'spin' and np.any(np.abs(controls[prefix_end:,0])>1e-9):
        return _reject(CertificationReason.MODE,0)
    if context is None:
        context = prepare_certification(grid, backend=backend)
    checked = 0
    for index, state in enumerate(states):
        mode=prefix_mode if index<prefix_end else trajectory.mode
        reason = _state_limit_reason(state, limits, mode)
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
        if limits.motion_model == "curvature_limited" and mode != "spin" and abs(a) > 1e-12:
            crossing = -state[3] / a
            if 0 < crossing < dt and abs(state[4] + controls[index, 1] * crossing) > 1e-9:
                return _reject(CertificationReason.LIMIT, checked)
        midpoint = step(state, controls[index], dt / 2)
        reason = _state_limit_reason(midpoint, limits, mode)
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
    if not check_braking_tail:
        return Certification(True, CertificationReason.VALID, checked_poses=checked)
    tail_mode = prefix_mode if prefix_end == len(states) else trajectory.mode
    collision, brake_time, count = _brake_tail(states[-1], context, limits, tail_mode)
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
            current_route_version: int, *, backend=None, context=None, check_braking_tail=True) -> Certification:
    """绑定完整地图/约束/轨迹；默认检查停车尾，False仅检查实际轨迹本体。"""
    if type(check_braking_tail) is not bool:
        raise TypeError('check_braking_tail must be bool')
    if context is not None:
        if not isinstance(context, _CollisionContext) or context.grid_digest != grid.digest():
            raise ValueError("certification context grid mismatch")
        if context.backend is not backend:
            raise ValueError("certification context backend mismatch")
    result = _certify_impl(trajectory, grid, limits, current_route_version, backend, context, check_braking_tail)
    return replace(result, braking_tail_checked=check_braking_tail, map_epoch=trajectory.map_epoch,
                   route_version=trajectory.route_version,
                   trajectory_source=trajectory.source,
                   trajectory_digest=trajectory.digest(),
                   grid_digest=grid.digest(), limits_digest=limits.digest())
