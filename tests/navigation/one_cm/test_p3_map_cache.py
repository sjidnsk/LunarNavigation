"""源上下文淘汰和世界地图历史独立的资源生命周期回归。"""

import numpy as np
import pytest

from test_p3_map import ScriptedTerrain, _api, grid_map, state_at
from test_p3_terrain_native import compiled


def assert_cache_bound(store, expected, native=None):
    assert len(store._lattice_ids) == expected
    assert len(store._fresh_observations) == expected
    assert len(store._source_classifications) == expected
    if native is not None:
        assert native.lattice_count == expected
        assert tuple(store._lattice_ids.values()) == native.active_source_ids


def test_injected_sources_have_bounded_cache_and_never_reuse_retired_ids():
    store_type, _ = _api()
    store = store_type(terrain=ScriptedTerrain())
    observed_ids = []
    for index in range(24):
        store.apply(grid_map(np.zeros((5, 5)), .05), (0., 0., index * 1e-4))
        assert_cache_bound(store, min(index + 1, 8))
        observed_ids.append(list(store._lattice_ids.values())[-1])
    store.apply(grid_map(np.zeros((5, 5)), .05))
    assert list(store._lattice_ids.values())[-1] > observed_ids[-1]
    assert_cache_bound(store, 8)
    assert all(right > left for left, right in zip(observed_ids, observed_ids[1:]))


@pytest.mark.parametrize("geometry", ["yaw", "phase"])
def test_true_native_and_python_caches_evict_together(compiled, geometry):
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    from scripts.navigation.one_cm.nav1cm.terrain_native import NativeTerrain
    with NativeTerrain(compiled[0]) as native:
        store = P3MapStore(terrain=native)
        identities = []
        for index in range(24):
            if geometry == "yaw":
                world = store.apply(grid_map(np.zeros((9, 9)), .05), (0., 0., index * 1e-4))
            else:
                world = store.apply(grid_map(np.zeros((9, 9)), .05, (index * 1e-4, 0.)))
            identities.append(native.active_source_ids[-1])
            assert_cache_bound(store, min(index + 1, 8), native)
            active = set(native.active_source_ids)
            assert all(owner == 0 or owner in active for row in store._observation_owners.values()
                       for _, _, owner in row)
        assert world.known_free
        assert identities == sorted(set(identities))


def test_true_native_eviction_keeps_world_history_and_revisit_cannot_revive_old_halo(compiled):
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    from scripts.navigation.one_cm.nav1cm.terrain_native import NativeTerrain
    elevation = np.zeros((9, 9), dtype=np.float32)
    elevation[4, 4] = .4
    with NativeTerrain(compiled[0]) as native:
        store = P3MapStore(terrain=native)
        first = store.apply(grid_map(elevation, .05))
        first_id = native.active_source_ids[-1]
        assert state_at(first, .225, .225) == 1
        for index in range(1, 9):
            # 不同源位于远处，不能用新自由覆盖旧障碍，旧世界区域必须保留。
            last = store.apply(grid_map(np.zeros((9, 9)), .05, (index * 2., 0.)), (0., 0., index * .01))
        assert first_id not in native.active_source_ids
        assert state_at(last, .225, .225) == 1
        assert state_at(last, 2.2, .25) == 0
        revisit = np.full_like(elevation, np.nan)
        revisit[4, 5] = .001
        reentered = store.apply(grid_map(revisit, .05))
        assert native.active_source_ids[-1] > first_id
        assert state_at(reentered, .225, .225) == 1
        assert state_at(reentered, 2.2, .25) == 0
        assert_cache_bound(store, 8, native)
        cleared = store.apply(grid_map(np.zeros_like(elevation), .05))
        assert state_at(cleared, .225, .225) == 0


def test_true_native_integer_rolling_map_keeps_single_context(compiled):
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    from scripts.navigation.one_cm.nav1cm.terrain_native import NativeTerrain
    with NativeTerrain(compiled[0]) as native:
        store = P3MapStore(terrain=native)
        for index in range(24):
            store.apply(grid_map(np.zeros((9, 9)), .05, (index * .05, 0.)))
            assert_cache_bound(store, 1, native)


def test_true_native_and_python_touch_and_nan_lru_order_stay_aligned(compiled):
    from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore
    from scripts.navigation.one_cm.nav1cm.terrain_native import NativeTerrain
    with NativeTerrain(compiled[0]) as native:
        store = P3MapStore(terrain=native)
        message = grid_map(np.zeros((9, 9)), .05)
        for index in range(8):
            store.apply(message, (0., 0., index * .01))
        first, second = native.active_source_ids[:2]
        store.apply(message)
        assert_cache_bound(store, 8, native)
        before = native.active_source_ids
        store.apply(grid_map(np.full((9, 9), np.nan), .05), (0., 0., .9))
        assert native.active_source_ids == before
        assert_cache_bound(store, 8, native)
        store.apply(message, (0., 0., .08))
        assert first in native.active_source_ids and second not in native.active_source_ids
        assert_cache_bound(store, 8, native)
