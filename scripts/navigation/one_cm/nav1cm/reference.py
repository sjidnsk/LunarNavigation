"""路线弧长进度匹配与首次出窗的连续前向参考。"""

import numpy as np

from .contracts import GridSnapshot, LocalReference, ProgressToken, Route, VehicleState

# 现有默认末段范围；P4的4秒预测与0.2m/s上限对应0.8m。
TERMINAL_APPROACH_DISTANCE_M = .8


def _arc_lengths(points):
    return np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))))


def _point_at(points, arc, s):
    if s == arc[-1]:
        # 最终顶点直接保留，避免负起点加线段的舍入使终端身份丢失。
        return points[-1]
    index = int(np.clip(np.searchsorted(arc, s, side="right") - 1, 0, len(arc) - 2))
    fraction = (s - arc[index]) / (arc[index + 1] - arc[index])
    return points[index] + fraction * (points[index + 1] - points[index])


def _project(points, arc, xy, low, high):
    best = (np.inf, low)
    for index in range(len(points) - 1):
        start = max(low, arc[index])
        end = min(high, arc[index + 1])
        if start > end:
            continue
        vector = points[index + 1] - points[index]
        segment_length = arc[index + 1] - arc[index]
        fraction = float(np.clip(np.dot(xy - points[index], vector) / (segment_length * segment_length),
                                 (start - arc[index]) / segment_length, (end - arc[index]) / segment_length))
        candidate_s = arc[index] + fraction * segment_length
        distance_sq = float(np.sum((points[index] + fraction * vector - xy) ** 2))
        # 等距处保留较早进度，防止交叉点向后续腿跳跃。
        if distance_sq < best[0] - 1e-12 or (abs(distance_sq - best[0]) <= 1e-12 and candidate_s < best[1]):
            best = distance_sq, candidate_s
    return best[1]


def _inside(point, minimum, maximum):
    return bool(np.all(point >= minimum - 1e-12) and np.all(point <= maximum + 1e-12))


def _exit_fraction(start, end, minimum, maximum):
    delta = end - start
    fractions = []
    for axis in range(2):
        if delta[axis] > 0:
            fractions.append((maximum[axis] - start[axis]) / delta[axis])
        elif delta[axis] < 0:
            fractions.append((minimum[axis] - start[axis]) / delta[axis])
    return float(min(fractions))


def project_progress(route: Route, state: VehicleState, previous: ProgressToken | None = None,
                     *, arc=None) -> ProgressToken:
    """与局部参考共用的单调有界投影；同版进度最多向前0.25m。"""
    arc = _arc_lengths(route.points) if arc is None else arc
    total = float(arc[-1])
    if previous is not None and previous.route_version == route.version:
        if not np.isfinite(previous.s) or previous.s < 0 or previous.s > total:
            raise ValueError("invalid previous route progress")
        low = max(0.0, previous.s - 0.05)
        high = min(total, previous.s + 0.25)
    else:
        low, high = 0.0, total
    projected = _project(route.points, arc, np.array([state.x, state.y]), low, high)
    if previous is not None and previous.route_version == route.version:
        projected = max(previous.s, projected)
    return ProgressToken(route.version, projected)


def extract_reference(
    route: Route, state: VehicleState, grid: GridSnapshot, previous: ProgressToken | None = None,
    *, terminal_yaw: float | None = None
) -> tuple[LocalReference, ProgressToken]:
    """同版进度最多向前 0.25 m；换版从整条路线重新投影。"""
    arc = _arc_lengths(route.points)
    total = float(arc[-1])
    token = project_progress(route, state, previous, arc=arc)
    projected = token.s
    minimum = np.array(grid.origin_xy, dtype=float)
    maximum = minimum + np.array([grid.cells.shape[1], grid.cells.shape[0]]) * grid.resolution_m
    start = _point_at(route.points, arc, projected)
    if not _inside(start, minimum, maximum):
        raise ValueError("projected route is outside local window")
    points = [start]
    lengths = [projected]
    for index in range(int(np.searchsorted(arc, projected, side="right")), len(arc)):
        end = route.points[index]
        if _inside(end, minimum, maximum):
            points.append(end)
            lengths.append(float(arc[index]))
            continue
        fraction = _exit_fraction(points[-1], end, minimum, maximum)
        fraction = float(np.clip(fraction, 0.0, 1.0))
        points.append(points[-1] + fraction * (end - points[-1]))
        lengths.append(lengths[-1] + fraction * (arc[index] - lengths[-1]))
        break
    # 起点恰在顶点时不输出零长段。
    keep = [0]
    for index in range(1, len(lengths)):
        if lengths[index] > lengths[keep[-1]] + 1e-12:
            keep.append(index)
    # 路线来源 epoch 可旧于局部图；参考绑定本次局部快照供轨迹求解与认证。
    # 必须沿路线实际走到最后一个顶点；位置容差不能把出窗截点当成最终段。
    reaches_end = lengths[keep[-1]] == total and np.array_equal(points[keep[-1]], route.points[-1])
    reference = LocalReference(route.version, grid.epoch, np.asarray(points)[keep], np.asarray(lengths)[keep],
                               projected, terminal_yaw if reaches_end else None)
    return reference, token
