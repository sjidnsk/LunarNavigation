"""截面优化与原通用几何算法逐行等价；oracle 固定为优化前实现。"""
import math

import numpy as np
import pytest

from nav1cm import p3_map
from nav1cm.contracts import CellState
from nav1cm.p3_map import _polygon, _intersect_intervals
from nav1cm.p3_geometry import continuous_strip_intervals
from nav1cm.terrain_native import TerrainUpdate

def _original_cross_section(polygon, y):
    points = []
    tolerance = 1e-9
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        if abs(end[1] - start[1]) <= tolerance:
            if abs(y - start[1]) <= tolerance:
                points.extend((start[0], end[0]))
        elif min(start[1], end[1]) - tolerance <= y <= max(start[1], end[1]) + tolerance:
            t = min(1., max(0., (y - start[1]) / (end[1] - start[1])))
            points.append(start[0] + t * (end[0] - start[0]))
    return (min(points), max(points)) if points else None

def _original_free_rows(regions, update):
    """自由并集在整条厘米条带内的共同 x 区间，消除来源矩形内部接缝。

    按所有顶点事件分段，并保持同一个自由连通分量的边界对应，证明整个
    条带连续覆盖。两端由不同分量覆盖不能证明中间自由。不密化世界地图。
    """
    bands = {}
    for region in regions:
        if region[4] != CellState.FREE:
            continue
        polygon = _polygon(region, update)
        minimum = min(point[1] for point in polygon)
        maximum = max(point[1] for point in polygon)
        for y in range(math.floor(minimum + 1e-9), math.ceil(maximum - 1e-9)):
            bands.setdefault(y, []).append((polygon, minimum, maximum))
    for y, polygons in bands.items():
        # 与行范围和截面的 1e-9 cm 容差一致：贴近整数边界的浮点顶点
        # 不应再切出无法表示中点的数值薄片；不移动真实顶点或合并内部缺口。
        levels = sorted({float(y), float(y + 1),
                         *(point[1] for polygon, _, _ in polygons for point in polygon
                           if y + 1e-9 < point[1] < y + 1 - 1e-9)})
        covered = None
        for lower_y, upper_y in zip(levels, levels[1:]):
            middle = (lower_y + upper_y) / 2
            active = [polygon for polygon, minimum, maximum in polygons if minimum < middle < maximum]
            if not active:
                covered = []
                break
            sections = [(_original_cross_section(polygon, lower_y), _original_cross_section(polygon, upper_y))
                        for polygon in active]
            intervals = continuous_strip_intervals((lo, hi) for lo, hi in sections
                                                   if lo is not None and hi is not None)
            covered = intervals if covered is None else _intersect_intervals(covered, intervals)
            if not covered:
                break
        for left, right in covered or ():
            x0, x1 = math.ceil(left - 1e-9), math.floor(right + 1e-9)
            if x0 < x1:
                yield y, x0, x1, int(CellState.FREE)

def _original_blocked_rows(region, update):
    polygon = _polygon(region, update)
    minimum_y, maximum_y = min(point[1] for point in polygon), max(point[1] for point in polygon)
    tolerance = 1e-9
    first = math.floor(minimum_y + tolerance)
    last = math.ceil(maximum_y - tolerance)
    for y in range(first, last):
        lo, hi = max(float(y), minimum_y), min(float(y + 1), maximum_y)
        lower, upper = _original_cross_section(polygon, lo), _original_cross_section(polygon, hi)
        if lower is None or upper is None:
            continue
        # 水平条带与矩形相交的所有 x 范围均标障碍，含条带内的顶点。
        xs = [*lower, *upper, *(point[0] for point in polygon if lo < point[1] < hi)]
        left, right = math.floor(min(xs) + tolerance), math.ceil(max(xs) - tolerance)
        if left < right:
            yield y, left, right, int(CellState.BLOCKED)

@pytest.mark.parametrize("origin", [(0., 0.), (-.203, -.017), (.003, .007),
                                    (-.2, -.01), (1e-12, -1e-12)])
@pytest.mark.parametrize("resolution", [.2, .01, .003, .000000000001])
@pytest.mark.parametrize("yaw", [0., 1e-15, .3, math.pi / 2])
def test_sections_match_original_on_boundaries_and_mixed_states(origin, resolution, yaw):
    regions = np.array([[-2, -2, 0, 1, 0], [0, -2, 2, 1, 0],
                        [-2, 1, -1, 3, 0], [0, 1, 2, 3, 0],
                        [-1, 1, 0, 2, -1], [-1, 2, 0, 3, 1]], dtype=np.int64)
    update = TerrainUpdate(resolution, origin, yaw, regions)
    assert list(p3_map._free_rows(regions, update)) == list(_original_free_rows(regions, update))
    for region in regions:
        assert list(p3_map._blocked_rows(region, update)) == list(_original_blocked_rows(region, update))


def test_axis_aligned_sections_do_not_repeat_polygon_intersections(monkeypatch):
    regions = np.array([[0, 0, 2, 2, 0], [2, 0, 3, 1, 0]], dtype=np.int64)
    update = TerrainUpdate(.2, (-.003, .007), 0., regions)
    expected_free = list(_original_free_rows(regions, update))
    expected_blocked = list(_original_blocked_rows(regions[0], update))
    calls = []
    original = p3_map._cross_section
    def counted(polygon, y):
        calls.append(y)
        return original(polygon, y)
    monkeypatch.setattr(p3_map, '_cross_section', counted)
    assert list(p3_map._free_rows(regions, update)) == expected_free
    assert list(p3_map._blocked_rows(regions[0], update)) == expected_blocked
    assert calls == []


def test_nonzero_rotation_uses_original_intersections(monkeypatch):
    region = (0, 0, 2, 2, 0)
    update = TerrainUpdate(.2, (.003, -.007), 1e-15, np.array([region], dtype=np.int64))
    calls = []
    original = p3_map._cross_section
    def counted(polygon, y):
        calls.append(y)
        return original(polygon, y)
    monkeypatch.setattr(p3_map, '_cross_section', counted)
    assert list(p3_map._blocked_rows(region, update)) == list(_original_blocked_rows(region, update))
    assert list(p3_map._free_rows([region], update)) == list(_original_free_rows([region], update))
    assert calls
