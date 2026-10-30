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


def _boxes(value):
    boxes = np.asarray(value, dtype=float)
    if (boxes.ndim != 2 or boxes.shape[1] != 4 or not np.isfinite(boxes).all()
            or np.any(boxes[:, :2] > boxes[:, 2:])):
        raise ValueError("boxes must be finite ordered (n, 4) bounds")
    return boxes


def _overlaps(box, blocked):
    # 严格分离才算安全，盒的边或顶点接触均拒绝。
    return bool(np.any((blocked[:, 0] <= box[2]) & (blocked[:, 2] >= box[0]) &
                       (blocked[:, 1] <= box[3]) & (blocked[:, 3] >= box[1])))


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
                step_m=.05, max_points=4096, clock=perf_counter) -> ShapedRoute:
    """新全局路线的有界后处理；耗尽回退整条已验证原路线，不留部分曲线。

    二次曲线控制点为原拐角，端点分别位于相邻原线段上。每次 De Casteljau
    二分的控制三角形包住整段曲线，叶 AABB 与障碍严格分离才接受该段。
    叶控制多边形长度上限同时限制采样间距；采样折线仍独立连续重验。
    """
    if (not np.isfinite(budget_s) or budget_s < 0 or not np.isfinite(max_trim_m)
            or max_trim_m < 0 or not np.isfinite(step_m) or step_m <= 0
            or type(max_points) is not int or max_points < 2):
        raise ValueError("invalid smoothing budget, trim, step or point limit")
    blocked = _boxes(blocked_boxes)
    began = clock()
    points = route.points
    # 即使预算为零，也不能将未经验证或已碰撞的 raw 路线作为安全回退。
    if not all(_segment_clear(a, b, blocked) for a, b in zip(points[:-1], points[1:])):
        raise ValueError("raw route collision")

    def fallback(reason):
        return ShapedRoute(route, np.empty((0, 4)), False, reason)

    def check():
        now = clock()
        if not np.isfinite(now) or now - began >= budget_s:
            raise _Exhausted("BUDGET_EXHAUSTED")

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

    def certify(a, control, b, depth=0):
        nonlocal subdivisions
        check()
        subdivisions += 1
        if subdivisions > 16384 or depth > 16:
            raise _Exhausted("SUBDIVISION_LIMIT")
        controls = np.asarray((a, control, b))
        # 浮点二分和凸包求界向外留余量，不让舍入产生虚假严格分离。
        pad = 32 * np.finfo(float).eps * max(1., float(np.max(np.abs(controls))))
        box = np.r_[np.min(controls, axis=0) - pad, np.max(controls, axis=0) + pad]
        overlaps = _overlaps(box, blocked)
        if overlaps and (not _segment_clear(a, a, blocked) or not _segment_clear(b, b, blocked)):
            return None
        length_bound = np.linalg.norm(control - a) + np.linalg.norm(b - control)
        if not overlaps and length_bound <= step_m:
            return [b], [box]
        left = (a + control) / 2
        right = (control + b) / 2
        middle = (left + right) / 2
        first = certify(a, left, middle, depth + 1)
        if first is None:
            return None
        second = certify(middle, right, b, depth + 1)
        if second is None:
            return None
        return first[0] + second[0], first[1] + second[1]

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
                        if all(_segment_clear(x, y, blocked)
                               for x, y in zip([a] + samples[:-1], samples)):
                            break
                        candidate = None
                    trim /= 2
            if candidate is None:
                line_to(corner)
            else:
                line_to(a)
                for sample in samples:
                    append(sample)
                proof.extend(boxes)
                changed = True
        if not changed:
            return fallback("NO_ELIGIBLE_CORNER")
        line_to(points[-1])
        check()
        shaped = ShapedRoute(Route(route.version, route.map_epoch, np.asarray(output), route.goal),
                             np.asarray(proof).reshape(-1, 4), True, "SMOOTHED")
        if not reference_clear(shaped, blocked):
            return fallback("GEOMETRY_REJECTED")
        check()
        return shaped
    except _Exhausted as exhausted:
        return fallback(str(exhausted))
