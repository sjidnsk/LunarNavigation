"""真实 core intrinsic 分类对照；无编译器时明确标记 NOT_RUN。"""

import ctypes
import importlib
import os
from pathlib import Path
import shutil
import subprocess

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[3]


def _api():
    try:
        return importlib.import_module("scripts.navigation.one_cm.nav1cm.terrain_native")
    except ModuleNotFoundError as error:
        if error.name.endswith("terrain_native"):
            pytest.fail("独立原生地形 ABI 接口尚未实现")
        raise


def test_missing_native_library_has_no_python_terrain_fallback(tmp_path):
    with pytest.raises((OSError, RuntimeError), match="(?i)(terrain|library|原生|地形)"):
        _api().NativeTerrain(tmp_path / "missing-native-library")


def test_capability_is_loaded_from_existing_wheel_configuration():
    api = _api()
    import yaml
    with (ROOT / "config/wheel.yaml").open(encoding="utf-8") as stream:
        expected = yaml.safe_load(stream)["capability"]
    actual = api.load_wheel_capability(ROOT / "config/wheel.yaml")
    for name in ("maximum_slope_rad", "maximum_local_obstacle_relief_m", "minimum_underbody_clearance_m"):
        assert getattr(actual, name) == expected[name]


def test_terrain_update_owns_immutable_validated_regions():
    update_type = _api().TerrainUpdate
    raw = np.array([[0, 0, 3, 2, 1]], dtype=np.int64)
    update = update_type(.03, (.003, -.007), .1, raw)
    raw[0, 2] = 99
    assert update.regions[0, 2] == 3
    with pytest.raises(ValueError): update.regions.setflags(write=True)
    for regions in (np.array([[0, 0, 0, 2, 1]]), np.array([[0, 0, 2, 2, 2]]), np.array([[0, 0, 2, 2, .5]])):
        with pytest.raises(ValueError): update_type(.03, (0., 0.), 0., regions)


@pytest.fixture(scope="module")
def compiled(tmp_path_factory):
    compiler = shutil.which("g++")
    if compiler is None:
        pytest.skip("g++ unavailable; real core terrain comparison NOT_RUN")
    output = tmp_path_factory.mktemp("p3-native-terrain")
    bridge = output / ("nav1cm_terrain.dll" if os.name == "nt" else "libnav1cm_terrain.so")
    oracle = output / ("terrain_reference.dll" if os.name == "nt" else "terrain_reference.so")
    core = ROOT / "ros2_ws/src/lunar_incremental_navigation_core"
    common = [compiler, "-O2", "-std=c++20", "-shared", "-fPIC", "-pthread",
              "-I", str(core / "include"), "-I", str(core / "src")]
    sources = [str(core / "src/map" / name) for name in
               ("persistent_elevation_map.cpp", "platform_elevation_physics.cpp", "platform_terrain_window.cpp", "fine_traversability_builder.cpp")]
    explicit = os.environ.get("NAV1CM_TERRAIN_LIB")
    if not explicit:
        subprocess.run(common + [str(ROOT / "scripts/navigation/one_cm/nav1cm/terrain_bridge.cpp")] + sources + ["-o", str(bridge)], check=True)
    subprocess.run(common + [str(Path(__file__).with_name("p3_terrain_reference.cpp"))] + sources + ["-o", str(oracle)], check=True)
    lib = ctypes.CDLL(str(oracle))
    reference = lib.nav1cm_terrain_reference_v1
    array_f = np.ctypeslib.ndpointer(dtype=np.float32, ndim=2, flags="C_CONTIGUOUS")
    array_b = np.ctypeslib.ndpointer(dtype=np.int8, ndim=2, flags=("C_CONTIGUOUS", "WRITEABLE"))
    reference.argtypes = [array_f, ctypes.c_int, ctypes.c_int] + [ctypes.c_double] * 5 + [array_b, array_b]
    reference.restype = ctypes.c_int
    return str(explicit or bridge), reference


def reference_cells(reference, elevation, resolution, capability):
    elevation = np.ascontiguousarray(elevation, dtype=np.float32)
    intrinsic = np.empty(elevation.shape, dtype=np.int8)
    inflated = np.empty(elevation.shape, dtype=np.int8)
    assert reference(elevation, *elevation.shape, resolution, capability.maximum_slope_rad,
                     capability.maximum_local_obstacle_relief_m,
                     capability.minimum_underbody_clearance_m, capability.body_long_side_m or 0., intrinsic, inflated) == 0
    return intrinsic, inflated


def result_cells(update, shape):
    cells = np.full(shape, -1, dtype=np.int8)
    for x0, y0, x1, y1, state in update.regions:
        cells[max(0, y0):min(shape[0], y1), max(0, x0):min(shape[1], x1)] = state
    return cells


@pytest.mark.parametrize("resolution", [.005, .01, .03, .05, .1, .2])
@pytest.mark.parametrize("kind", ["flat", "nan", "step", "ramp"])
def test_native_matches_direct_core_at_source_resolution(compiled, resolution, kind):
    api = _api()
    elevation = np.zeros((9, 11), dtype=np.float32)
    if kind == "nan": elevation[3:6, 4:7] = np.nan
    elif kind == "step": elevation[4, 5] = .4
    elif kind == "ramp": elevation[:] = np.arange(11)[None, :] * .01
    capability = api.load_wheel_capability()
    with api.NativeTerrain(compiled[0], capability=capability) as terrain:
        update = terrain.apply(elevation, resolution_m=resolution, origin_xy_m=(.003, -.007), yaw=.37, z_offset_m=0.)
    expected, _ = reference_cells(compiled[1], elevation, resolution, capability)
    np.testing.assert_array_equal(result_cells(update, elevation.shape), expected)
    assert update.resolution_m == resolution and update.origin_xy_m == (.003, -.007)
    assert update.yaw == pytest.approx(.37)


def test_intrinsic_output_does_not_apply_vehicle_footprint_twice(compiled):
    api = _api()
    elevation = np.zeros((31, 31), dtype=np.float32)
    elevation[15, 15] = .4
    capability = api.load_wheel_capability()
    expected, inflated = reference_cells(compiled[1], elevation, .1, capability)
    assert np.count_nonzero(inflated == 1) > np.count_nonzero(expected == 1)
    with api.NativeTerrain(compiled[0], capability=capability) as terrain:
        update = terrain.apply(elevation, resolution_m=.1, origin_xy_m=(0., 0.), yaw=0., z_offset_m=0.)
    np.testing.assert_array_equal(result_cells(update, elevation.shape), expected)


def test_native_accumulates_nan_holes_and_reclassifies_changed_neighbor_halo(compiled):
    api = _api()
    first = np.zeros((7, 8), dtype=np.float32)
    first[:, 4:] = np.nan
    second = np.zeros_like(first)
    second[:, :4] = np.nan
    with api.NativeTerrain(compiled[0]) as terrain:
        left = terrain.apply(first, resolution_m=.03, origin_xy_m=(0., 0.), yaw=0., z_offset_m=0.)
        history = result_cells(left, first.shape)
        right = terrain.apply(second, resolution_m=.03, origin_xy_m=(0., 0.), yaw=0., z_offset_m=0.)
        for x0, y0, x1, y1, state in right.regions:
            history[max(0, y0):min(7, y1), max(0, x0):min(8, x1)] = state
        assert terrain.lattice_count == 1
    expected, _ = reference_cells(compiled[1], np.zeros_like(first), .03, api.load_wheel_capability())
    np.testing.assert_array_equal(history, expected)


def test_native_tracks_resolution_yaw_and_fractional_lattice_changes(compiled):
    api = _api()
    data = np.zeros((7, 8), dtype=np.float32)
    with api.NativeTerrain(compiled[0]) as terrain:
        for resolution, origin, yaw in ((.05, (0., 0.), 0.), (.03, (0., 0.), 0.),
                                         (.05, (.1, .05), 0.), (.05, (.003, 0.), 0.),
                                         (.05, (0., 0.), .3)):
            terrain.apply(data, resolution_m=resolution, origin_xy_m=origin, yaw=yaw, z_offset_m=0.)
        assert terrain.lattice_count == 4


def test_duplicate_native_free_observation_can_clear_obstacle_from_other_resolution(compiled):
    api = _api()
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    from test_p3_map import grid_map, state_at
    with api.NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        coarse = grid_map(np.zeros((15, 15)), .1)
        store.apply(coarse)
        elevation = np.zeros((30, 30), dtype=np.float32)
        elevation[15, 15] = .4
        blocked = store.apply(grid_map(elevation, .05))
        assert state_at(blocked, .775, .775) == 1
        cleared = store.apply(coarse)
        assert state_at(cleared, .775, .775) == 0


@pytest.mark.parametrize("newer_obstacle", [False, True])
@pytest.mark.parametrize("resolution,origin,yaw", [(.05, (0., 0.), 0.),
                                                 (.1, (.003, -.007), 0.),
                                                 (.05, (.003, -.007), .03)])
def test_old_lattice_neighbor_halo_cannot_overwrite_newer_direct_observation(
        compiled, newer_obstacle, resolution, origin, yaw):
    """A/B/C：C 只观测旧 A 的邻格，不能把 A 的历史中心冒充新观测。"""
    api = _api()
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    from test_p3_map import grid_map, state_at
    coarse = np.zeros((15, 15), dtype=np.float32)
    coarse[7, 7] = 0. if newer_obstacle else .4
    size = round(1.5 / resolution)
    finer = np.zeros((size, size), dtype=np.float32)
    finer[round(.75 / resolution), round(.75 / resolution)] = .4 if newer_obstacle else 0.
    neighbor_only = np.full((15, 15), np.nan, dtype=np.float32)
    neighbor_only[6, 6] = .001
    with api.NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        store.apply(grid_map(coarse, .1))
        newer = store.apply(grid_map(finer, resolution, origin, yaw))
        expected = 1 if newer_obstacle else 0
        assert state_at(newer, .775, .775) == expected
        after_halo = store.apply(grid_map(neighbor_only, .1))
        assert state_at(after_halo, .775, .775) == expected
        # 真正重新观测该中心仍可覆盖，不能通过冻结障碍掩盖来源问题。
        direct = np.full((15, 15), np.nan, dtype=np.float32)
        direct[7, 7] = 0. if newer_obstacle else .4
        after_direct = store.apply(grid_map(direct, .1))
        assert state_at(after_direct, .775, .775) == (0 if newer_obstacle else 1)


@pytest.mark.parametrize("origin_cells", [(0, 0), (1, 1), (-2, 3)])
@pytest.mark.parametrize("new_height,expected", [(0., 0), (.4, 1)])
def test_same_lattice_neighbor_reclassification_still_updates_world(compiled, new_height, expected, origin_cells):
    api = _api()
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    from test_p3_map import grid_map, state_at
    initial = np.zeros((15, 15), dtype=np.float32)
    initial[6, 6] = .4 if new_height == 0. else 0.
    neighbor_only = np.full_like(initial, np.nan)
    neighbor_only[6 - origin_cells[1], 6 - origin_cells[0]] = new_height
    with api.NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        before = store.apply(grid_map(initial, .1))
        assert state_at(before, .775, .775) == 1 - expected
        after = store.apply(grid_map(neighbor_only, .1, tuple(index * .1 for index in origin_cells)))
        assert state_at(after, .775, .775) == expected


def test_same_lattice_subcentimeter_observations_accumulate_across_frames(compiled):
    api = _api()
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    from test_p3_map import grid_map, state_at
    left = np.zeros((10, 10), dtype=np.float32)
    left[:, 5:] = np.nan
    right = np.zeros_like(left)
    right[:, :5] = np.nan
    with api.NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        initial = store.apply(grid_map(left, .005))
        assert state_at(initial, .025, .025) == -1
        complete = store.apply(grid_map(right, .005))
        assert state_at(complete, .025, .025) == 0


def test_partial_direct_free_cannot_revive_stale_native_halo_authority(compiled):
    api = _api()
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    from test_p3_map import grid_map, state_at
    initial = np.zeros((20, 20), dtype=np.float32)
    newer = np.zeros((5, 5), dtype=np.float32)
    newer[2, 2] = .4
    partial = np.full_like(initial, np.nan)
    with api.NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        store.apply(grid_map(initial, .005, (.003, .003)))
        blocked = store.apply(grid_map(newer, .003, (.009, .009)))
        assert state_at(blocked, .015, .015) == 1
        for height in (.0001, .0002):
            partial[2, 2] = height
            after = store.apply(grid_map(partial, .005, (.003, .003)))
            assert state_at(after, .015, .015) == 1
        cleared = store.apply(grid_map(initial, .005, (.003, .003)))
        assert state_at(cleared, .015, .015) == 0


def fresh_coverage_case():
    initial = np.zeros((20, 20), dtype=np.float32)
    blocked = np.zeros((9, 9), dtype=np.float32)
    blocked[4, 4] = .4
    left, right = np.full_like(initial, np.nan), np.full_like(initial, np.nan)
    left[7:10, 7:8] = .0001
    right[7:10, 8:10] = .0002
    return initial, blocked, left, right


@pytest.mark.parametrize("reverse", [False, True])
def test_post_conflict_subcentimeter_free_accumulates_across_frames(compiled, reverse):
    api = _api()
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    from test_p3_map import grid_map, state_at
    initial, blocked, left, right = fresh_coverage_case()
    with api.NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        store.apply(grid_map(initial, .005, (.003, .003)))
        newer = store.apply(grid_map(blocked, .01))
        assert state_at(newer, .045, .045) == 1
        pieces = (right, left) if reverse else (left, right)
        first = store.apply(grid_map(pieces[0], .005, (.003, .003)))
        assert state_at(first, .045, .045) == 1
        complete = store.apply(grid_map(pieces[1], .005, (.003, .003)))
        assert state_at(complete, .045, .045) == 0


@pytest.mark.parametrize("conflict_time", ["before_initial_conflict", "between_pieces"])
def test_pre_conflict_source_coverage_cannot_help_clear_newer_obstacle(compiled, conflict_time):
    api = _api()
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    from test_p3_map import grid_map, state_at
    initial, blocked, left, right = fresh_coverage_case()
    with api.NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        store.apply(grid_map(initial, .005, (.003, .003)))
        if conflict_time == "between_pieces":
            store.apply(grid_map(blocked, .01))
        store.apply(grid_map(left, .005, (.003, .003)))
        # 新的明确外源观测使此前左侧覆盖过期，包括原生 duplicate 更新。
        store.apply(grid_map(blocked, .01))
        incomplete = store.apply(grid_map(right, .005, (.003, .003)))
        assert state_at(incomplete, .045, .045) == 1
        complete = store.apply(grid_map(left, .005, (.003, .003)))
        assert state_at(complete, .045, .045) == 0


@pytest.mark.parametrize("foreign_observation", ["unknown", "elsewhere"])
def test_unrelated_foreign_update_preserves_pending_fresh_coverage(compiled, foreign_observation):
    api = _api()
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    from test_p3_map import grid_map, state_at
    initial, blocked, left, right = fresh_coverage_case()
    unrelated = np.full_like(blocked, np.nan)
    if foreign_observation == "elsewhere":
        unrelated[0, 0] = .001
    with api.NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        store.apply(grid_map(initial, .005, (.003, .003)))
        store.apply(grid_map(blocked, .01))
        store.apply(grid_map(left, .005, (.003, .003)))
        store.apply(grid_map(unrelated, .01))
        complete = store.apply(grid_map(right, .005, (.003, .003)))
        assert state_at(complete, .045, .045) == 0


def test_native_updates_without_three_state_change_keep_epoch_and_snapshot(compiled):
    api = _api()
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    from test_p3_map import grid_map, state_at
    flat = np.zeros((20, 20), dtype=np.float32)
    with api.NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        initial = store.apply(grid_map(flat, .005))
        for elevation in (flat, np.full_like(flat, np.nan), np.full_like(flat, .001)):
            assert store.apply(grid_map(elevation, .005)) is initial
        coarse = np.full((10, 10), .001, dtype=np.float32)
        assert store.apply(grid_map(coarse, .01)) is initial
        assert store.source_resolution_m == .01
        coarse[4, 4] = .4
        changed = store.apply(grid_map(coarse, .01))
        assert changed is not initial and changed.epoch == initial.epoch + 1
        assert state_at(changed, .045, .045) == 1
        assert store.apply(grid_map(coarse, .01)) is changed


def test_native_nan_update_does_not_erase_historical_obstacle(compiled):
    api = _api()
    elevation = np.zeros((9, 9), dtype=np.float32)
    elevation[4, 4] = .4
    with api.NativeTerrain(compiled[0]) as terrain:
        first = terrain.apply(elevation, resolution_m=.05, origin_xy_m=(0., 0.), yaw=0., z_offset_m=0.)
        unknown = terrain.apply(np.full_like(elevation, np.nan), resolution_m=.05,
                                origin_xy_m=(0., 0.), yaw=0., z_offset_m=0.)
        cleared = terrain.apply(np.zeros_like(elevation), resolution_m=.05,
                                origin_xy_m=(0., 0.), yaw=0., z_offset_m=0.)
    assert result_cells(first, elevation.shape)[4, 4] == 1
    assert unknown.regions.shape == (0, 5)
    assert result_cells(cleared, elevation.shape)[4, 4] == 0



def test_observed_709_cell_disk_projects_without_interior_seams(compiled):
    """固定初帧测量掩码；真实 intrinsic 分类、整厘米投影和硬圆支持。"""
    from scripts.navigation.one_cm.nav1cm.contracts import CellState, VehicleState
    from scripts.navigation.one_cm.nav1cm.maps import build_distance_field, extract_window
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    from test_p3_map import grid_map
    yy, xx = np.mgrid[:31, :31]
    observed = (yy - 15) ** 2 + (xx - 15) ** 2 <= 225
    elevation = np.where(observed, 0., np.nan).astype(np.float32)
    assert int(np.count_nonzero(observed)) == 709
    capability = _api().load_wheel_capability()
    intrinsic, _ = reference_cells(compiled[1], elevation, .2, capability)
    # 原生 .2 m 分类的独立逐格展开，初始 5 m 窗口原点是 (-2.4,-2.4)。
    expected = np.repeat(np.repeat(intrinsic, 20, axis=0), 20, axis=1)[60:560, 60:560]
    with _api().NativeTerrain(compiled[0], capability=capability) as terrain:
        world = P3MapStore(terrain=terrain).apply(grid_map(elevation, .2, (-3., -3.)))
    grid = extract_window(world, VehicleState(.1, .1, 0., 0., 0.), False)
    np.testing.assert_array_equal(grid.cells, expected)
    assert np.all(grid.cells[150:350, 150:350] == CellState.FREE)
    # 固定车体 1.182 x .818、净空 .2 与 solver .003 均不放宽。
    required_circle = np.hypot(1.182, .818) / 2 + .2 + .003
    assert build_distance_field(grid).sample(.1, .1)[0] >= required_circle
