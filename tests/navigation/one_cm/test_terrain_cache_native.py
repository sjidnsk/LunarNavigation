"""真实 core 源缓存容量、LRU 与淘汰后的高程隔离回归。"""

import ctypes
import math

import numpy as np
import pytest

from test_p3_terrain_native import _api, compiled, result_cells


def apply_frame(terrain, *, yaw=0., origin=(0., 0.), elevation=None):
    if elevation is None:
        elevation = np.zeros((9, 9), dtype=np.float32)
    return terrain.apply(elevation, resolution_m=.05, origin_xy_m=origin, yaw=yaw)


def test_terrain_update_metadata_keeps_four_argument_injection_compatible():
    update = _api().TerrainUpdate(.05, (0., 0.), 0., np.empty((0, 5), dtype=np.int64))
    assert getattr(update, "source_id", "missing") is None
    assert getattr(update, "evicted_source_ids", None) == ()
    assert getattr(update, "cache_capacity", None) == 8


def test_native_abi_v2_reports_cache_metadata(compiled):
    library = ctypes.CDLL(compiled[0])
    version = library.nav1cm_terrain_abi_version
    version.argtypes, version.restype = [], ctypes.c_int
    assert version() == 2
    with _api().NativeTerrain(compiled[0]) as terrain:
        assert terrain.cache_capacity == 8
        assert terrain.active_source_ids == ()
        update = apply_frame(terrain)
        assert update.source_id > 0
        assert update.evicted_source_ids == ()
        assert update.cache_capacity == terrain.cache_capacity
        assert terrain.active_source_ids == (update.source_id,)
        with pytest.raises(AttributeError):
            terrain.cache_capacity = 9
        with pytest.raises(AttributeError):
            terrain.active_source_ids = ()


@pytest.mark.parametrize("geometry", ["yaw", "phase"])
def test_native_bounds_different_source_geometries_to_eight(compiled, geometry):
    with _api().NativeTerrain(compiled[0]) as terrain:
        created = []
        updates = []
        for index in range(24):
            update = apply_frame(terrain, yaw=index * 1e-4) if geometry == "yaw" else apply_frame(
                terrain, origin=(index * 1e-4, 0.))
            created.append(getattr(update, "source_id", None))
            updates.append(update)
            assert terrain.lattice_count == min(index + 1, 8)
        for index, update in enumerate(updates):
            assert getattr(update, "evicted_source_ids", None) == (() if index < 8 else (created[index - 8],))
        assert all(right > left > 0 for left, right in zip(created, created[1:]))
        assert terrain.active_source_ids == tuple(created[-8:])


def test_native_touch_refreshes_lru_instead_of_fifo(compiled):
    with _api().NativeTerrain(compiled[0]) as terrain:
        sources = [apply_frame(terrain, yaw=index * .01) for index in range(8)]
        touched = apply_frame(terrain, yaw=0.)
        added = apply_frame(terrain, yaw=.08)
        assert getattr(touched, "source_id", None) == getattr(sources[0], "source_id", "missing")
        assert getattr(added, "evicted_source_ids", None) == (getattr(sources[1], "source_id", "missing"),)
        assert terrain.lattice_count == 8
        assert sources[0].source_id in terrain.active_source_ids
        assert sources[1].source_id not in terrain.active_source_ids


def test_native_integer_rolling_offset_keeps_one_source_and_refreshes_lru(compiled):
    yaw = .37
    origin = (.003, -.007)
    with _api().NativeTerrain(compiled[0]) as terrain:
        first = apply_frame(terrain, yaw=yaw, origin=origin)
        for index in range(1, 8):
            apply_frame(terrain, yaw=yaw + index * .01, origin=origin)
        offset = (4 * .05, -3 * .05)
        moved_origin = (origin[0] + math.cos(yaw) * offset[0] - math.sin(yaw) * offset[1],
                        origin[1] + math.sin(yaw) * offset[0] + math.cos(yaw) * offset[1])
        rolled = apply_frame(terrain, yaw=yaw, origin=moved_origin)
        assert getattr(rolled, "source_id", None) == getattr(first, "source_id", "missing")
        assert rolled.origin_xy_m == origin
        assert terrain.lattice_count == 8
        added = apply_frame(terrain, yaw=yaw + .08, origin=origin)
        assert first.source_id not in added.evicted_source_ids
        assert terrain.active_source_ids[-2:] == (first.source_id, added.source_id)


def test_native_nan_frames_neither_allocate_evict_nor_refresh_sources(compiled):
    unknown = np.full((9, 9), np.nan, dtype=np.float32)
    with _api().NativeTerrain(compiled[0]) as terrain:
        empty = apply_frame(terrain, yaw=.7, elevation=unknown)
        assert terrain.lattice_count == 0
        assert getattr(empty, "source_id", "missing") is None
        sources = [apply_frame(terrain, yaw=index * .01) for index in range(8)]
        active = terrain.active_source_ids
        for yaw in [0., .5, .6]:
            skipped = apply_frame(terrain, yaw=yaw, elevation=unknown)
            assert skipped.source_id is None and skipped.evicted_source_ids == ()
            assert skipped.regions.shape == (0, 5)
            assert terrain.active_source_ids == active
        added = apply_frame(terrain, yaw=.08)
        assert added.evicted_source_ids == (sources[0].source_id,)


def test_native_revisited_evicted_geometry_has_new_identity_without_old_nan_elevation(compiled):
    original = np.zeros((9, 9), dtype=np.float32)
    original[4, 4] = .4
    revisit = np.full_like(original, np.nan)
    revisit[4, 5] = .001
    with _api().NativeTerrain(compiled[0]) as terrain:
        first = apply_frame(terrain, elevation=original)
        assert result_cells(first, original.shape)[4, 4] == 1
        for index in range(1, 9):
            latest = apply_frame(terrain, yaw=index * .01)
        recreated = apply_frame(terrain, elevation=revisit)
        # 只观测相邻格时，已淘汰的中心高程不能从旧 PersistentElevationMap 复活。
        assert result_cells(recreated, original.shape)[4, 4] == -1
        assert not any(x0 <= 4 < x1 and y0 <= 4 < y1 for x0, y0, x1, y1, _ in recreated.regions)
        assert getattr(latest, "evicted_source_ids", None) == (getattr(first, "source_id", "missing"),)
        assert recreated.source_id > latest.source_id > first.source_id
        assert terrain.lattice_count == 8
