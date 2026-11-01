"""相切二次 Bezier 全局参考整形；仅证明膨胀障碍下的二维几何安全。"""
from dataclasses import dataclass
from time import perf_counter

import numpy as np

from .contracts import Route, _readonly_array
from .global_planner import _segment_clear, path_cost


@dataclass(frozen=True)
class ShapedRoute:
    route: Route
    proof_boxes: np.ndarray
    changed: bool
    reason: str

    def __post_init__(self):
        boxes = _boxes(self.proof_boxes)
        object.__setattr__(self, "proof_boxes", _readonly_array(boxes, np.float64, 2, 4))


def _boxes(value, check=None):
    boxes = np.asarray(value, dtype=float)
    if boxes.ndim != 2 or boxes.shape[1] != 4:
        raise ValueError("boxes must be finite ordered (n, 4) bounds")
    for offset in range(0, len(boxes), 1024):
        if check is not None:
            check()
        batch = boxes[offset:offset + 1024]
        invalid = not np.isfinite(batch).all() or np.any(batch[:, :2] > batch[:, 2:])
        if check is not None:
            check()
        if invalid:
            raise ValueError("boxes must be finite ordered (n, 4) bounds")
    return boxes


def _overlaps(box, blocked):
    # 严格分离才算安全，盒的边或顶点接触均拒绝。
    return bool(np.any((blocked[:, 0] <= box[2]) & (blocked[:, 2] >= box[0]) &
                       (blocked[:, 1] <= box[3]) & (blocked[:, 3] >= box[1])))


def _segments_clear(starts, ends, blocked, corner_radius=0.):
    """批量执行与 _segment_clear 相同的闭区间裁剪，触边仍拒绝。"""
    if corner_radius > 0:
        return all(_segment_clear(a,b,blocked,corner_radius=corner_radius) for a,b in zip(starts,ends))
    lo = np.zeros((len(starts), len(blocked)))
    hi = np.ones_like(lo)
    possible = np.ones_like(lo, dtype=bool)
    for axis in (0, 1):
        delta = (ends[:, axis] - starts[:, axis])[:, None]
        origin = starts[:, axis, None]
        fixed = delta == 0
        possible &= (~fixed | ((origin >= blocked[None, :, axis]) &
                               (origin <= blocked[None, :, axis + 2])))
        divisor = np.where(fixed, 1., delta)
        near = (blocked[None, :, axis] - origin) / divisor
        far = (blocked[None, :, axis + 2] - origin) / divisor
        lo = np.maximum(lo, np.where(fixed, -np.inf, np.minimum(near, far)))
        hi = np.minimum(hi, np.where(fixed, np.inf, np.maximum(near, far)))
    return not bool(np.any(possible & (lo <= hi)))


def _overlap_mask(boxes, blocked, corner_radius=0.):
    """每个凸包盒的障碍闭区间相交掩码。"""
    if corner_radius > 0:
        raw = blocked + [corner_radius,corner_radius,-corner_radius,-corner_radius]
        gap = np.maximum(np.maximum(raw[None,:,:2]-boxes[:,None,2:],
                                    boxes[:,None,:2]-raw[None,:,2:]),0.)
        return np.any(np.sum(gap*gap,axis=2) <= (corner_radius+1e-12)**2,axis=1)
    return np.any((blocked[None, :, 0] <= boxes[:, None, 2]) &
                  (blocked[None, :, 2] >= boxes[:, None, 0]) &
                  (blocked[None, :, 1] <= boxes[:, None, 3]) &
                  (blocked[None, :, 3] >= boxes[:, None, 1]), axis=1)


def reference_clear(shaped: ShapedRoute, blocked_boxes, *, corner_radius=0.) -> bool:
    """当前地图必须同时接纳采样折线与全部连续曲线凸包叶证据。"""
    try:
        blocked = _boxes(blocked_boxes)
        proof = _boxes(shaped.proof_boxes)
        points = shaped.route.points
        return bool(np.isfinite(points).all()
                    and all(_segment_clear(a, b, blocked,corner_radius=corner_radius) for a, b in zip(points[:-1], points[1:]))
                    and not np.any(_overlap_mask(proof,blocked,corner_radius)))
    except (ValueError, TypeError, AttributeError):
        return False


class _Exhausted(Exception):
    pass


def shape_route(route: Route, blocked_boxes, *, budget_s=.025, max_trim_m=.75,
                step_m=.05, max_points=4096, clock=perf_counter,
                raw_verified=False, terminal_yaw=None, soft_clearance_m=0.,
                clearance_weight=0., corner_radius=0.) -> ShapedRoute:
    """新全局路线的有界后处理；耗尽回退整条已验证原路线，不留部分曲线。

    terminal_yaw 可选；三次末段连接的终切线按 yaw 模 pi 选择最小转角。
    二次曲线控制点为原拐角，端点分别位于相邻原线段上。每次 De Casteljau
    二分的控制多边形包住整段曲线，叶 AABB 与障碍严格分离才接受该段。
    叶控制多边形长度上限同时限制采样间距；采样折线仍独立连续重验。

    raw_verified=True 仅供已用同一 blocked_boxes 连续认证原路线的调用方复用
    证明，不能用于跳过新图复核。默认先在同一预算内验证原路线；未完成证明
    即耗尽时抛 ValueError，只有已有原路线证明时才可安全整条回退。
    预算是协作式墙钟限制；每个最多 1024 障碍的数值操作前后检查，不是硬实时。
    """
    if (not np.isfinite(budget_s) or budget_s < 0 or not np.isfinite(max_trim_m)
            or max_trim_m < 0 or not np.isfinite(step_m) or step_m <= 0
            or type(max_points) is not int or max_points < 2
            or type(raw_verified) is not bool
            or not np.isfinite(soft_clearance_m) or soft_clearance_m < 0
            or not np.isfinite(clearance_weight) or clearance_weight < 0
            or not np.isfinite(corner_radius) or corner_radius < 0
            or (terminal_yaw is not None and not np.isfinite(terminal_yaw))):
        raise ValueError("invalid smoothing budget, trim, step or point limit")
    began = clock()
    points = route.points

    def fallback(reason):
        return ShapedRoute(route, np.empty((0, 4)), False, reason)

    def check():
        now = clock()
        if not np.isfinite(now) or now - began >= budget_s:
            raise _Exhausted("BUDGET_EXHAUSTED")

    verified_segments = set()  # 仅本次不可变障碍快照内复用精确几何证明。

    def segments_clear(starts, ends):
        check()
        keys = [tuple(a)+tuple(b) for a,b in zip(starts,ends)]
        pending = [i for i,key in enumerate(keys) if key not in verified_segments]
        # 两个维度均分块；每段通过全部障碍检查之后才缓存，不缓存失败/部分证明。
        for first in range(0, len(pending), 64):
            selected=pending[first:first+64]
            for offset in range(0, len(blocked), 1024):
                check()
                clear = _segments_clear(starts[selected], ends[selected],
                                        blocked[offset:offset + 1024],corner_radius=corner_radius)
                check()
                if not clear:
                    return False
            verified_segments.update(keys[i] for i in selected)
        check()
        return True

    def overlaps(boxes):
        check()
        mask = np.zeros(len(boxes), dtype=bool)
        for first in range(0, len(boxes), 64):
            for offset in range(0, len(blocked), 1024):
                check()
                mask[first:first + 64] |= _overlap_mask(boxes[first:first + 64],
                                                       blocked[offset:offset + 1024],corner_radius=corner_radius)
                check()
        return mask

    try:
        # 起钟在障碍准备之前；空障碍无需逐边扫描即可完成原路线证明。
        blocked = _boxes(blocked_boxes, check)
        if not raw_verified and len(blocked):
            if not segments_clear(points[:-1], points[1:]):
                raise ValueError("raw route collision")
        raw_verified = True
    except _Exhausted as exhausted:
        if not raw_verified:
            raise ValueError("raw validation budget exhausted") from exhausted
        return fallback(str(exhausted))

    output = [points[0]]
    proof = []
    changed = False
    subdivisions = 0

    def append(point):
        check()
        if np.array_equal(point, output[-1]):
            return
        if len(output) >= max_points:
            raise _Exhausted("POINT_LIMIT")
        output.append(point)

    def line_to(point):
        # 直线单边就是精确连续表示；仅曲线叶受 step_m 采样长度限制。
        append(point)

    def append_samples(samples):
        check()
        distinct = np.any(np.diff(np.vstack((output[-1], samples)), axis=0) != 0, axis=1)
        if len(output) + int(np.count_nonzero(distinct)) > max_points:
            raise _Exhausted("POINT_LIMIT")
        output.extend(samples[distinct])
        check()

    def certify(*polygon):
        nonlocal subdivisions
        controls = np.asarray([polygon])
        # 用深度17的整数参数区间定位叶，最后恢复原递归从左到右的顺序。
        keys = np.zeros(1, dtype=np.int64)
        accepted_points, accepted_boxes, accepted_keys = [], [], []
        for depth in range(17):
            check()
            subdivisions += len(controls)
            if subdivisions > 16384:
                raise _Exhausted("SUBDIVISION_LIMIT")
            # 与递归版逐节点相同的控制点凸包、32eps外扩和长度判据。
            pad = 32 * np.finfo(float).eps * np.maximum(1., np.max(np.abs(controls), axis=(1, 2)))
            boxes = np.concatenate((np.min(controls, axis=1) - pad[:, None],
                                    np.max(controls, axis=1) + pad[:, None]), axis=1)
            collision = overlaps(boxes)
            if np.any(collision):
                endpoints = controls[collision][:, (0, -1), :].reshape(-1, 2)
                if not segments_clear(endpoints, endpoints):
                    return None
            length_bound = np.linalg.norm(np.diff(controls, axis=1), axis=2).sum(axis=1)
            leaves = ~collision & (length_bound <= step_m)
            if np.any(leaves):
                accepted_points.append(controls[leaves, -1])
                accepted_boxes.append(boxes[leaves])
                accepted_keys.append(keys[leaves])
            pending = ~leaves
            if not np.any(pending):
                order = np.argsort(np.concatenate(accepted_keys), kind="stable")
                check()
                return np.concatenate(accepted_points)[order], np.concatenate(accepted_boxes)[order]
            if depth == 16:
                raise _Exhausted("SUBDIVISION_LIMIT")
            controls = controls[pending]
            keys = keys[pending]
            # 同一个 De Casteljau 细分器服务二次拐角和三次终点连接。
            level = controls
            left, right = [level[:, 0]], [level[:, -1]]
            while level.shape[1] > 1:
                level = (level[:, :-1] + level[:, 1:]) / 2
                left.append(level[:, 0])
                right.append(level[:, -1])
            degree_count = controls.shape[1]
            controls = np.stack((np.stack(left, axis=1),
                                 np.stack(right[::-1], axis=1)), axis=1).reshape(-1, degree_count, 2)
            keys = (keys[:, None] + np.asarray((0, 1 << (16-depth)))).reshape(-1)
            check()
        raise _Exhausted("SUBDIVISION_LIMIT")

    try:
        check()
        if len(points) > max_points:
            raise _Exhausted("POINT_LIMIT")
        for i in range(1, len(points) - 1):
            check()
            previous, corner, following = points[i-1:i+2]
            incoming, outgoing = corner - previous, following - corner
            before, after = np.linalg.norm(incoming), np.linalg.norm(outgoing)
            trim = min(max_trim_m, .75, .25 * before, .25 * after)
            unit_in, unit_out = incoming / before, outgoing / after
            cosine = np.clip(np.dot(unit_in, unit_out), -1., 1.)
            # 近直线、折返和极短段不提供稳定切线/可分辨裁切点，保留原角。
            candidate = None
            if trim > 1e-9 and -1 + 1e-10 < cosine < 1 - 1e-10:
                for _ in range(8):
                    check()
                    a, b = corner - trim * unit_in, corner + trim * unit_out
                    if np.array_equal(a, corner) or np.array_equal(b, corner):
                        break
                    candidate = certify(a, corner, b)
                    if candidate is not None:
                        samples, boxes = candidate
                        if segments_clear(np.vstack((a, samples[:-1])), samples):
                            break
                        candidate = None
                    trim /= 2
            if candidate is None:
                line_to(corner)
            else:
                line_to(a)
                append_samples(samples)
                proof.extend(boxes)
                changed = True
        if terminal_yaw is not None:
            goal = points[-1]
            tail = goal - output[-1]
            available = np.linalg.norm(tail)
            if available > 1e-9:
                unit = tail / available
                # 模 pi 选择最小几何转角，不决定车辆前进或倒车。
                heading = np.arctan2(unit[1], unit[0])
                relative = np.arctan2(np.sin(terminal_yaw - heading),
                                      np.cos(terminal_yaw - heading))
                # 正负90度同长时优先显式yaw；容忍角度运算的舍入误差。
                # 真正更短才取yaw+pi，避免在平局处反转几何末切线。
                if abs(abs(relative) - np.pi/2) <= 64 * np.finfo(float).eps:
                    relative = np.copysign(np.pi/2, relative)
                elif abs(relative) > np.pi/2:
                    relative -= np.copysign(np.pi, relative)
                normal = np.array((-unit[1], unit[0]))
                tangent = max(0., np.cos(relative)) * unit + np.sin(relative) * normal
                # 最多3m接近区，避免750m直线整段弯曲；短尾段保留1/4直线。
                # 这里只限制几何尺度，不证明曲率/车速/角速度可行性。
                trim = min(3., .75 * available)
                if abs(relative) > 1e-10 and trim > 1e-9:
                    for _ in range(8):
                        check()
                        start = goal - trim * unit
                        candidate = certify(start, start + trim/3 * unit,
                                            goal - trim/3 * tangent, goal)
                        if candidate is not None:
                            samples, boxes = candidate
                            if segments_clear(np.vstack((start, samples[:-1])), samples):
                                line_to(start)
                                append_samples(samples)
                                proof.extend(boxes)
                                changed = True
                                break
                        trim /= 2
        if not changed:
            return fallback("NO_ELIGIBLE_CORNER")
        line_to(points[-1])
        check()
        shaped = ShapedRoute(Route(route.version, route.map_epoch, np.asarray(output), route.goal),
                             np.asarray(proof).reshape(-1, 4), True, "SMOOTHED")
        # 不调用无 deadline 的公开新图复核入口；逐边和逐叶共享本次预算。
        if not segments_clear(shaped.route.points[:-1], shaped.route.points[1:]):
            return fallback("GEOMETRY_REJECTED")
        if np.any(overlaps(shaped.proof_boxes)):
            return fallback("GEOMETRY_REJECTED")
        if soft_clearance_m > 0 and clearance_weight > 0:
            # 搜索、拉直与Bezier整形复用同一路径代价，避免安全的剪角重新贴障。
            original_cost = path_cost(points, blocked, soft_clearance_m, clearance_weight, check,corner_radius=corner_radius)
            shaped_cost = path_cost(shaped.route.points, blocked, soft_clearance_m, clearance_weight, check,corner_radius=corner_radius)
            original_length = np.linalg.norm(np.diff(points, axis=0), axis=1).sum()
            shaped_length = np.linalg.norm(np.diff(shaped.route.points, axis=0), axis=1).sum()
            has_clearance_cost = (original_cost > original_length + 1e-9 or
                                  shaped_cost > shaped_length + 1e-9)
            # 无近障代价时仍允许为最终朝向生成稍长的平滑末段。
            if has_clearance_cost and shaped_cost > original_cost + 1e-9:
                return fallback("CLEARANCE_COST")
        check()
        return shaped
    except _Exhausted as exhausted:
        return fallback(str(exhausted))
