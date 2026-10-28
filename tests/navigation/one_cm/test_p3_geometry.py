"""旋转条带和多个内部洞的真实原生几何回归。"""

import math

import numpy as np
import pytest

from test_p3_map import grid_map
from test_p3_terrain_native import _api, compiled


def assert_free_centers_observed(world, elevation, resolution, yaw, origin):
    c, s = math.cos(yaw), math.sin(yaw)
    bad = []
    for rect in world.known_free:
        yy, xx = np.mgrid[rect.y0:rect.y1, rect.x0:rect.x1]
        x, y = (xx + .5) * .01, (yy + .5) * .01
        sx = np.floor((c * x + s * y - origin[0]) / resolution).astype(int)
        sy = np.floor((-s * x + c * y - origin[1]) / resolution).astype(int)
        inside = (0 <= sx) & (sx < elevation.shape[1]) & (0 <= sy) & (sy < elevation.shape[0])
        valid = np.zeros(sx.shape, dtype=bool)
        valid[inside] = np.isfinite(elevation[sy[inside], sx[inside]])
        for iy, ix in zip(*np.nonzero(~valid)):
            bad.append((int(xx[iy, ix]), int(yy[iy, ix]), int(sx[iy, ix]), int(sy[iy, ix])))
    assert not bad, f"FREE centimeter center lies in UNKNOWN: {bad[:12]}"
    unknown = []
    for y, row in enumerate(elevation):
        changes = np.flatnonzero(np.diff(np.r_[False, ~np.isfinite(row), False].astype(np.int8)))
        unknown.extend((int(a), y, int(b), y + 1) for a, b in changes.reshape(-1, 2))
    for rect in world.known_free:
        polygon = [((c * x + s * y - origin[0]) / resolution,
                    (-s * x + c * y - origin[1]) / resolution)
                   for x, y in ((rect.x0 * .01, rect.y0 * .01),
                                (rect.x1 * .01, rect.y0 * .01),
                                (rect.x1 * .01, rect.y1 * .01),
                                (rect.x0 * .01, rect.y1 * .01))]
        assert all(-1e-7 <= x <= elevation.shape[1] + 1e-7 and
                   -1e-7 <= y <= elevation.shape[0] + 1e-7 for x, y in polygon)
        for x0, y0, x1, y1 in unknown:
            clipped = polygon
            for axis, bound, positive in ((0, x0, True), (0, x1, False), (1, y0, True), (1, y1, False)):
                clipped = clip_half_plane(clipped, axis, bound, positive)
            area = abs(sum(a[0] * b[1] - b[0] * a[1]
                           for a, b in zip(clipped, clipped[1:] + clipped[:1]))) / 2
            assert area <= 1e-7, f"FREE rectangle crosses UNKNOWN interior: {rect}, source={(x0,y0,x1,y1)}"


def clip_half_plane(polygon, axis, bound, positive):
    """测试专用独立多边形裁剪；验证整个面积，不用采样点证明覆盖。"""
    if not polygon:
        return []
    result = []
    for previous, point in zip(polygon[-1:] + polygon[:-1], polygon):
        inside_previous = previous[axis] >= bound if positive else previous[axis] <= bound
        inside = point[axis] >= bound if positive else point[axis] <= bound
        if inside_previous != inside:
            t = (bound - previous[axis]) / (point[axis] - previous[axis])
            result.append(tuple(a + t * (b - a) for a, b in zip(previous, point)))
        if inside:
            result.append(point)
    return result


@pytest.mark.parametrize("resolution,yaw", [(.003, .01), (.002, .03), (.001, .1)])
def test_true_native_unknown_stripes_do_not_become_free(compiled, resolution, yaw):
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    elevation = np.zeros((40, 1000), dtype=np.float32)
    elevation[::10, :] = np.nan
    with _api().NativeTerrain(compiled[0]) as terrain:
        world = P3MapStore(terrain=terrain).apply(grid_map(elevation, resolution), (0., 0., yaw))
    assert_free_centers_observed(world, elevation, resolution, yaw, (0., 0.))


def test_true_native_many_holes_and_fractional_phase_do_not_become_free(compiled):
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    elevation = np.zeros((80, 200), dtype=np.float32)
    for row, first, last in ((10, 15, 180), (30, 5, 100), (50, 80, 195), (70, 10, 190)):
        elevation[row, first:last] = np.nan
    resolution, yaw, origin = .003, .07, (.0037, -.0023)
    with _api().NativeTerrain(compiled[0]) as terrain:
        world = P3MapStore(terrain=terrain).apply(grid_map(elevation, resolution, origin), (0., 0., yaw))
    assert_free_centers_observed(world, elevation, resolution, yaw, origin)


def test_slanted_gap_crossing_cell_interior_is_not_covered_by_endpoint_components():
    from scripts.navigation.one_cm.nav1cm.p3_map import _free_rows
    update = _api().TerrainUpdate(.003, (0., 0.), .01, np.array([
        [0, 1, 1000, 9, 0], [0, 11, 1000, 19, 0],
        [0, 21, 1000, 29, 0], [0, 31, 1000, 39, 0]], dtype=np.int64))
    cells = [(y, left, right) for y, left, right, _ in _free_rows(update.regions, update)]
    assert not any(y == 4 and left <= 130 < right for y, left, right in cells)


def test_large_flat_region_still_merges_to_sparse_rectangle():
    from scripts.navigation.one_cm.nav1cm.p3_map import _free_rows, _rectangles
    update = _api().TerrainUpdate(.003, (0., 0.), 0., np.array([[0, 0, 10000, 10000, 0]], dtype=np.int64))
    rows = {y: ((left, right, state),) for y, left, right, state in _free_rows(update.regions, update)}
    assert _rectangles(rows) == [[0, 0, 3000, 3000, 0]]
