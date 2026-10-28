"""无 ROS 的 P3 布局、保守厘米转换与历史覆盖契约。

这里显式注入已经分类的地形区域，只隔离几何。真实物理判定另由
test_p3_terrain_native.py 与既有 C++ core 对照，不能以此替代原生验证。
"""

import importlib
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm.contracts import CellRect, CellState, VehicleState
from scripts.navigation.one_cm.nav1cm.maps import extract_window


def _api():
    try:
        module = importlib.import_module("scripts.navigation.one_cm.nav1cm.p3_map")
        native = importlib.import_module("scripts.navigation.one_cm.nav1cm.terrain_native")
    except ModuleNotFoundError as error:
        if error.name.endswith(("p3_map", "terrain_native")):
            pytest.fail("P3 地图接收与原生地形桥接接口尚未实现")
        raise
    return module.P3MapStore, native.TerrainUpdate


def grid_map(elevation, resolution=.05, origin=(0., 0.), yaw=0., *,
             row_major=False, start=(0, 0), offset=0):
    """与现有 GridMap adapter 一致：矩阵行沿负 x、列沿负 y。"""
    elevation = np.asarray(elevation, dtype=np.float32)
    height, width = elevation.shape
    buffer = np.empty(width * height, dtype=np.float32)
    for y in range(height):
        for x in range(width):
            physical_row = (width - 1 - x + start[0]) % width
            physical_col = (height - 1 - y + start[1]) % height
            k = physical_row * height + physical_col if row_major else physical_col * width + physical_row
            buffer[k] = elevation[y, x]
    dims = ([NS(label="row_index", size=width, stride=width * height),
             NS(label="column_index", size=height, stride=height)] if row_major else
            [NS(label="column_index", size=height, stride=width * height),
             NS(label="row_index", size=width, stride=width)])
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = width * resolution / 2, height * resolution / 2
    pose = NS(position=NS(x=origin[0] + c * hx - s * hy,
                          y=origin[1] + s * hx + c * hy, z=0.),
              orientation=NS(x=0., y=0., z=math.sin(yaw / 2), w=math.cos(yaw / 2)))
    return NS(header=NS(frame_id="sensor_map", stamp=NS(sec=0, nanosec=0)),
              info=NS(resolution=resolution, length_x=width * resolution,
                      length_y=height * resolution, pose=pose),
              layers=["elevation"], basic_layers=[],
              data=[NS(layout=NS(dim=dims, data_offset=offset),
                       data=np.r_[np.zeros(offset, dtype=np.float32), buffer])],
              outer_start_index=start[0], inner_start_index=start[1])


class ScriptedTerrain:
    """直接给定 intrinsic 区域；不包含地形算法或物理阈值。"""

    def __init__(self, regions=None):
        self.regions = regions
        self.calls = []

    def apply(self, elevation_m, *, resolution_m, origin_xy_m, yaw, z_offset_m):
        self.calls.append((elevation_m.copy(), resolution_m, origin_xy_m, yaw, z_offset_m))
        _, update_type = _api()
        height, width = elevation_m.shape
        regions = self.regions
        if regions is None:
            regions = [(0, 0, width, height, CellState.FREE)]
        return update_type(resolution_m, origin_xy_m, yaw, np.asarray(regions, dtype=np.int64).reshape(-1, 5))


def state_at(world, x, y):
    ix, iy = math.floor(x * 100), math.floor(y * 100)
    for rect in world.blocked:
        if rect.x0 <= ix < rect.x1 and rect.y0 <= iy < rect.y1:
            return CellState.BLOCKED
    for rect in world.known_free:
        if rect.x0 <= ix < rect.x1 and rect.y0 <= iy < rect.y1:
            return CellState.FREE
    return CellState.UNKNOWN


@pytest.mark.parametrize("resolution", [.005, .01, .03, .05, .1, .2])
def test_native_resolution_does_not_change_centimeter_window(resolution):
    store_type, _ = _api()
    terrain = ScriptedTerrain()
    store = store_type(terrain=terrain)
    world = store.apply(grid_map(np.zeros((10, 12)), resolution))
    assert world is store.snapshot and world.epoch == 1
    assert store.source_resolution_m == resolution and world.resolution_m == .01
    assert world.known_free == (CellRect(0, 0, round(12 * resolution * 100), round(10 * resolution * 100)),)
    for allow_unknown in (False, True):
        window = extract_window(world, VehicleState(0., 0., 0., 0., 0.), allow_unknown)
        assert window.cells.shape == (500, 500) and window.resolution_m == .01
    assert terrain.calls[0][1] == resolution


@pytest.mark.parametrize("row_major", [False, True])
@pytest.mark.parametrize("start", [(0, 0), (3, 1)])
def test_decodes_asymmetric_gridmap_layout_and_circular_buffer(row_major, start):
    store_type, _ = _api()
    terrain = ScriptedTerrain([])
    elevation = np.arange(12, dtype=np.float32).reshape(3, 4)
    elevation[1, 2] = np.nan
    store_type(terrain=terrain).apply(grid_map(elevation, row_major=row_major, start=start, offset=3))
    np.testing.assert_array_equal(terrain.calls[0][0], elevation)


@pytest.mark.parametrize("metadata", ["empty_dim", "blank_labels", "zero_strides", "defaults"])
def test_default_optional_layout_metadata_uses_standard_column_major(metadata):
    store_type, _ = _api()
    elevation = np.arange(12, dtype=np.float32).reshape(3, 4)
    message = grid_map(elevation, start=(2, 1))
    if metadata == "empty_dim":
        message.data[0].layout.dim = []
    elif metadata == "defaults":
        message.data[0].layout.dim = [NS(label="", size=0, stride=0), NS(label="", size=0, stride=0)]
    else:
        for dim in message.data[0].layout.dim:
            if metadata == "blank_labels": dim.label = ""
            else: dim.stride = 0
    terrain = ScriptedTerrain([])
    store_type(terrain=terrain).apply(message)
    np.testing.assert_array_equal(terrain.calls[0][0], elevation)


def test_explicit_row_major_layout_accepts_unspecified_strides():
    store_type, _ = _api()
    elevation = np.arange(12, dtype=np.float32).reshape(3, 4)
    message = grid_map(elevation, row_major=True, start=(1, 2))
    for dim in message.data[0].layout.dim:
        dim.stride = 0
    terrain = ScriptedTerrain([])
    store_type(terrain=terrain).apply(message)
    np.testing.assert_array_equal(terrain.calls[0][0], elevation)


def test_fractional_centimeter_origin_insets_free_and_expands_obstacle():
    store_type, _ = _api()
    free = store_type(terrain=ScriptedTerrain()).apply(grid_map(np.zeros((2, 2)), .05, (.003, -.007)))
    assert free.known_free == (CellRect(1, 0, 10, 9),)
    blocked = store_type(terrain=ScriptedTerrain([(0, 0, 1, 1, 1)])).apply(
        grid_map(np.zeros((1, 1)), .005, (.008, -.002)))
    assert blocked.blocked == (CellRect(0, -1, 2, 1),)
    assert state_at(free, .005, .005) == CellState.UNKNOWN


def test_pose_and_map_from_source_yaw_translation_are_composed():
    store_type, _ = _api()
    terrain = ScriptedTerrain()
    world = store_type(terrain=terrain).apply(
        grid_map(np.zeros((2, 3)), .1, (.2, -.1), math.pi / 2), (1., 2., math.pi / 2))
    _, resolution, origin, yaw, z = terrain.calls[0]
    assert resolution == .1 and z == 0
    np.testing.assert_allclose(origin, (1.1, 2.2), atol=1e-12)
    assert math.remainder(yaw - math.pi, 2 * math.pi) == pytest.approx(0.)
    assert world.known_free == (CellRect(80, 200, 110, 220),)


def test_transform_stamped_with_default_stamp_is_accepted():
    store_type, _ = _api()
    transform = NS(transform=NS(translation=NS(x=.003, y=-.007, z=.5),
                                rotation=NS(x=0., y=0., z=0., w=1.)))
    terrain = ScriptedTerrain()
    store_type(terrain=terrain).apply(grid_map(np.zeros((2, 2)), .05), transform)
    np.testing.assert_allclose(terrain.calls[0][2], (.003, -.007), atol=1e-12)
    assert terrain.calls[0][4] == .5


def test_rotated_free_contains_only_fully_observed_centimeter_cells():
    store_type, _ = _api()
    angle = .37
    origin = (.003, -.007)
    world = store_type(terrain=ScriptedTerrain()).apply(grid_map(np.zeros((4, 5)), .03, origin), (0., 0., angle))
    c, s = math.cos(angle), math.sin(angle)
    assert world.known_free
    for rect in world.known_free:
        for x in (rect.x0 / 100, rect.x1 / 100):
            for y in (rect.y0 / 100, rect.y1 / 100):
                # 先逆 TF，再减 GridMap 的实际原点。
                px, py = c * x + s * y - origin[0], -s * x + c * y - origin[1]
                assert -1e-10 <= px <= .15 + 1e-10 and -1e-10 <= py <= .12 + 1e-10


def test_rotated_obstacle_covers_every_cell_whose_center_lies_inside_source():
    store_type, _ = _api()
    angle = .62
    world = store_type(terrain=ScriptedTerrain([(0, 0, 1, 1, 1)])).apply(
        grid_map(np.zeros((1, 1)), .1, (.003, -.007)), (0., 0., angle))
    c, s = math.cos(angle), math.sin(angle)
    for iy in range(-20, 20):
        for ix in range(-20, 20):
            x, y = (ix + .5) / 100, (iy + .5) / 100
            px, py = c * x + s * y - .003, -s * x + c * y + .007
            if 0 <= px < .1 and 0 <= py < .1:
                assert state_at(world, x, y) == CellState.BLOCKED


def test_resolution_and_phase_changes_preserve_history_and_clear_obstacles_with_free():
    store_type, _ = _api()
    terrain = ScriptedTerrain([(0, 0, 2, 2, 1)])
    store = store_type(terrain=terrain)
    first = store.apply(grid_map(np.zeros((2, 2)), .05))
    assert state_at(first, .045, .045) == CellState.BLOCKED
    terrain.regions = [(0, 0, 4, 4, 0)]
    second = store.apply(grid_map(np.zeros((4, 4)), .03, (-.007, -.007)))
    assert store.source_resolution_m == .03 and second.epoch == first.epoch + 1
    assert state_at(second, .045, .045) == CellState.FREE
    assert state_at(first, .045, .045) == CellState.BLOCKED
    terrain.regions = [(0, 0, 2, 2, 0)]
    third = store.apply(grid_map(np.zeros((2, 2)), .2, (1., 0.)))
    assert state_at(third, .045, .045) == CellState.FREE
    assert state_at(third, 1.1, .1) == CellState.FREE
    assert state_at(third, .5, .1) == CellState.UNKNOWN


def test_unknown_observation_preserves_history_but_new_obstacle_overrides_free():
    store_type, _ = _api()
    terrain = ScriptedTerrain()
    store = store_type(terrain=terrain)
    store.apply(grid_map(np.zeros((10, 10)), .03))
    terrain.regions = [(0, 0, 10, 10, -1)]
    world = store.apply(grid_map(np.full((10, 10), np.nan), .03))
    assert state_at(world, .1, .1) == CellState.FREE
    terrain.regions = [(1, 1, 2, 2, 1)]
    world = store.apply(grid_map(np.zeros((10, 10)), .03))
    assert state_at(world, .045, .045) == CellState.BLOCKED
    assert state_at(world, .015, .015) == CellState.FREE


@pytest.mark.parametrize("newer_state", [CellState.FREE, CellState.BLOCKED])
def test_historical_lattice_halo_only_updates_cells_still_owned_by_that_lattice(newer_state):
    store_type, _ = _api()
    terrain = ScriptedTerrain([(0, 0, 15, 15, 0)])
    store = store_type(terrain=terrain)
    store.apply(grid_map(np.zeros((15, 15)), .1))
    terrain.regions = [(15, 15, 16, 16, newer_state)]
    store.apply(grid_map(np.zeros((30, 30)), .05))
    neighbor_only = np.full((15, 15), np.nan, dtype=np.float32)
    neighbor_only[6, 6] = .001
    stale_state = CellState.BLOCKED if newer_state == CellState.FREE else CellState.FREE
    terrain.regions = [(5, 5, 8, 8, stale_state)]
    after_halo = store.apply(grid_map(neighbor_only, .1))
    assert state_at(after_halo, .775, .775) == newer_state
    assert state_at(after_halo, .655, .655) == stale_state
    direct = np.full_like(neighbor_only, np.nan)
    direct[7, 7] = 0.
    after_direct = store.apply(grid_map(direct, .1))
    assert state_at(after_direct, .775, .775) == stale_state


def test_partial_direct_free_does_not_grant_old_halo_authority_over_whole_centimeter():
    store_type, _ = _api()
    terrain = ScriptedTerrain([(0, 0, 20, 20, 0)])
    store = store_type(terrain=terrain)
    initial = grid_map(np.zeros((20, 20)), .005, (.003, .003))
    store.apply(initial)
    terrain.regions = [(0, 0, 5, 5, 1)]
    store.apply(grid_map(np.zeros((5, 5)), .003, (.009, .009)))
    terrain.regions = [(0, 0, 20, 20, 0)]
    partial = np.full((20, 20), np.nan, dtype=np.float32)
    for value in (.0001, .0002):
        partial[2, 2] = value
        after = store.apply(grid_map(partial, .005, (.003, .003)))
        assert state_at(after, .015, .015) == CellState.BLOCKED
    fully_observed = store.apply(initial)
    assert state_at(fully_observed, .015, .015) == CellState.FREE


@pytest.mark.parametrize("reverse", [False, True])
def test_fresh_free_coverage_can_reclaim_mixed_cell_across_frames(reverse):
    store_type, _ = _api()
    terrain = ScriptedTerrain([(0, 0, 20, 20, 0)])
    store = store_type(terrain=terrain)
    initial = np.zeros((20, 20), dtype=np.float32)
    store.apply(grid_map(initial, .005, (.003, .003)))
    terrain.regions = [(3, 3, 6, 6, 1)]
    store.apply(grid_map(np.zeros((9, 9)), .01))
    left, right = np.full_like(initial, np.nan), np.full_like(initial, np.nan)
    left[7:10, 7:8] = .0001
    right[7:10, 8:10] = .0002
    pieces = (right, left) if reverse else (left, right)
    for index, piece in enumerate(pieces):
        terrain.regions = [(6, 6, 10, 11, 0)] if index == 0 else [(7, 6, 11, 11, 0)]
        complete = store.apply(grid_map(piece, .005, (.003, .003)))
    assert state_at(complete, .045, .045) == CellState.FREE


@pytest.mark.parametrize("kind", ["duplicate", "unknown", "same_class_height"])
def test_unchanged_three_state_geometry_reuses_snapshot_and_epoch(kind):
    store_type, _ = _api()
    terrain = ScriptedTerrain()
    store = store_type(terrain=terrain)
    initial = store.apply(grid_map(np.zeros((10, 10)), .03))
    elevation = np.zeros((10, 10), dtype=np.float32)
    if kind == "unknown":
        elevation[:] = np.nan
        terrain.regions = [(0, 0, 10, 10, -1)]
    elif kind == "same_class_height":
        elevation[:] = .001
    unchanged = store.apply(grid_map(elevation, .03))
    assert unchanged is initial and unchanged.epoch == initial.epoch
    assert len(terrain.calls) == 2


def test_resolution_updates_without_changing_snapshot_when_geometry_is_equal():
    store_type, _ = _api()
    store = store_type(terrain=ScriptedTerrain())
    initial = store.apply(grid_map(np.zeros((6, 6)), .05))
    equivalent = store.apply(grid_map(np.zeros((10, 10)), .03))
    assert equivalent is initial and store.source_resolution_m == .03


def test_actual_new_obstacle_changes_snapshot_and_increments_epoch():
    store_type, _ = _api()
    terrain = ScriptedTerrain()
    store = store_type(terrain=terrain)
    initial = store.apply(grid_map(np.zeros((10, 10)), .03))
    terrain.regions = [(4, 4, 5, 5, 1)]
    changed = store.apply(grid_map(np.zeros((10, 10)), .03))
    assert changed is not initial and changed.epoch == initial.epoch + 1
    assert state_at(changed, .135, .135) == CellState.BLOCKED


def test_overlapping_free_and_blocked_regions_in_one_update_keep_blocked():
    store_type, _ = _api()
    terrain = ScriptedTerrain([(1, 1, 2, 2, 1), (0, 0, 5, 5, 0)])
    world = store_type(terrain=terrain).apply(grid_map(np.zeros((5, 5)), .03))
    assert state_at(world, .045, .045) == CellState.BLOCKED


@pytest.mark.parametrize("resolution,origin,point", [(.005, (0., 0.), (.005, .005)),
                                                     (.03, (.003, -.007), (.015, .025))])
def test_jointly_observed_free_cell_clears_old_obstacle_across_source_run_seam(resolution, origin, point):
    store_type, _ = _api()
    terrain = ScriptedTerrain([(0, 0, 1, 1, 1)])
    store = store_type(terrain=terrain)
    cell_origin = (math.floor(point[0] * 100) / 100, math.floor(point[1] * 100) / 100)
    store.apply(grid_map(np.zeros((1, 1)), .01, cell_origin))
    terrain.regions = [(0, 0, 3, 1, 0), (0, 1, 2, 2, 0)]
    world = store.apply(grid_map(np.zeros((2, 3)), resolution, origin))
    assert state_at(world, *point) == CellState.FREE


def test_dimension_accepts_float_rounding_around_one_cell():
    store_type, _ = _api()
    message = grid_map(np.zeros((1, 1)), .1)
    message.info.length_x = np.nextafter(.1, 0.)
    world = store_type(terrain=ScriptedTerrain()).apply(message)
    assert world.known_free == (CellRect(0, 0, 10, 10),)


def test_repeated_observations_merge_regions_instead_of_retaining_per_cell_objects():
    store_type, _ = _api()
    terrain = ScriptedTerrain([(x, y, x + 1, y + 1, 0) for y in range(50) for x in range(50)])
    store = store_type(terrain=terrain)
    message = grid_map(np.zeros((50, 50)), .01)
    for _ in range(3):
        world = store.apply(message)
        assert world.known_free == (CellRect(0, 0, 50, 50),)
    assert not world.blocked


@pytest.mark.parametrize("kind", ["resolution", "length", "data", "layout", "start"])
def test_malformed_input_is_rejected_before_mutating_store(kind):
    store_type, _ = _api()
    terrain = ScriptedTerrain()
    store = store_type(terrain=terrain)
    previous = store.apply(grid_map(np.zeros((3, 4))))
    message = grid_map(np.zeros((3, 4)))
    if kind == "resolution": message.info.resolution = 0.
    elif kind == "length": message.info.length_y += .013
    elif kind == "data": message.data[0].data = message.data[0].data[:-1]
    elif kind == "layout": message.data[0].layout.dim[1].stride = 999
    elif kind == "start": message.outer_start_index = 4
    with pytest.raises(ValueError):
        store.apply(message)
    assert store.snapshot is previous and len(terrain.calls) == 1


def test_missing_native_library_fails_explicitly(monkeypatch, tmp_path):
    store_type, _ = _api()
    monkeypatch.setenv("NAV1CM_TERRAIN_LIB", str(tmp_path / "missing-terrain-library"))
    with pytest.raises((OSError, RuntimeError), match="(?i)(native|terrain|library|原生|地形)"):
        store_type()
