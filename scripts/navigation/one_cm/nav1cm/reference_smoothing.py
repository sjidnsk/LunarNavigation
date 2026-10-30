"""相切二次 Bezier 全局参考整形；仅证明膨胀障碍下的二维几何安全。"""
from dataclasses import dataclass
from time import perf_counter

import numpy as np

from .contracts import Route, _readonly_array
from .global_planner import _segment_clear


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


def _segments_clear(starts, ends, blocked):
    """批量执行与 _segment_clear 相同的闭区间裁剪，触边仍拒绝。"""
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


def _overlap_mask(boxes, blocked):
    """每个凸包盒的障碍闭区间相交掩码。"""
    return np.any((blocked[None, :, 0] <= boxes[:, None, 2]) &
                  (blocked[None, :, 2] >= boxes[:, None, 0]) &
                  (blocked[None, :, 1] <= boxes[:, None, 3]) &
                  (blocked[None, :, 3] >= boxes[:, None, 1]), axis=1)


def reference_clear(shaped: ShapedRoute, blocked_boxes) -> bool:
    """当前地图必须同时接纳采样折线与全部连续曲线凸包叶证据。"""
    try:
        blocked = _boxes(blocked_boxes)
        proof = _boxes(shaped.proof_boxes)
        points = shaped.route.points
        return bool(np.isfinite(points).all()
                    and all(_segment_clear(a, b, blocked) for a, b in zip(points[:-1], points[1:]))
                    and all(not _overlaps(box, blocked) for box in proof))
    except (ValueError, TypeError, AttributeError):
        return False


class _Exhausted(Exception):
    pass


def shape_route(route: Route, blocked_boxes, *, budget_s=.025, max_trim_m=.75,
                step_m=.05, max_points=4096, clock=perf_counter,
                raw_verified=False) -> ShapedRoute:
    """新全局路线的有界后处理；耗尽回退整条已验证原路线，不留部分曲线。

    二次曲线控制点为原拐角，端点分别位于相邻原线段上。每次 De Casteljau
    二分的控制三角形包住整段曲线，叶 AABB 与障碍严格分离才接受该段。
    叶控制多边形长度上限同时限制采样间距；采样折线仍独立连续重验。

    raw_verified=True 仅供已用同一 blocked_boxes 连续认证原路线的调用方复用
    证明，不能用于跳过新图复核。默认先在同一预算内验证原路线；未完成证明
    即耗尽时抛 ValueError，只有已有原路线证明时才可安全整条回退。
    预算是协作式墙钟限制；每个最多 1024 障碍的数值操作前后检查，不是硬实时。
    """
    if (not np.isfinite(budget_s) or budget_s < 0 or not np.isfinite(max_trim_m)
            or max_trim_m < 0 or not np.isfinite(step_m) or step_m <= 0
            or type(max_points) is not int or max_points < 2
            or type(raw_verified) is not bool):
        raise ValueError("invalid smoothing budget, trim, step or point limit")
    began = clock()
    points = route.points

    def fallback(reason):
        return ShapedRoute(route, np.empty((0, 4)), False, reason)

    def check():
        now = clock()
        if not np.isfinite(now) or now - began >= budget_s:
            raise _Exhausted("BUDGET_EXHAUSTED")

    def segments_clear(starts, ends):
        check()
        # 两个维度均分块，避免 4096×1024 的大临时矩阵吞掉本次预算。
        for first in range(0, len(starts), 64):
            for offset in range(0, len(blocked), 1024):
                check()
                clear = _segments_clear(starts[first:first + 64], ends[first:first + 64],
                                        blocked[offset:offset + 1024])
                check()
                if not clear:
                    return False
        return True

    def overlaps(boxes):
        check()
        mask = np.zeros(len(boxes), dtype=bool)
        for first in range(0, len(boxes), 64):
            for offset in range(0, len(blocked), 1024):
                check()
                mask[first:first + 64] |= _overlap_mask(boxes[first:first + 64],
                                                       blocked[offset:offset + 1024])
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

    def certify(a, control, b):
        nonlocal subdivisions
        controls = np.asarray([[a, control, b]])
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
                endpoints = controls[collision][:, (0, 2), :].reshape(-1, 2)
                if not segments_clear(endpoints, endpoints):
                    return None
            length_bound = (np.linalg.norm(controls[:, 1] - controls[:, 0], axis=1) +
                            np.linalg.norm(controls[:, 2] - controls[:, 1], axis=1))
            leaves = ~collision & (length_bound <= step_m)
            if np.any(leaves):
                accepted_points.append(controls[leaves, 2])
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
            left = (controls[:, 0] + controls[:, 1]) / 2
            right = (controls[:, 1] + controls[:, 2]) / 2
            middle = (left + right) / 2
            controls = np.stack((np.stack((controls[:, 0], left, middle), axis=1),
                                 np.stack((middle, right, controls[:, 2]), axis=1)), axis=1).reshape(-1, 3, 2)
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
        check()
        return shaped
    except _Exhausted as exhausted:
        return fallback(str(exhausted))
