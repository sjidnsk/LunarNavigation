import numpy as np
import pytest
from dataclasses import replace
from scripts.navigation.one_cm.nav1cm import maps, certification as cert
from scripts.navigation.one_cm.nav1cm.contracts import GridSnapshot, Trajectory, WheelLimits


def grid(**kwargs):
    return GridSnapshot(**(dict(epoch=1, origin_xy=(-2.5, -2.5), resolution_m=.01,
                               cells=np.zeros((500, 500), dtype=np.int8), allow_unknown=False) | kwargs))


def test_geometry_cache_rebinds_and_invalidates():
    cache = maps.DistanceFieldCache()
    a = grid()
    first = cache.get(a)
    b = replace(a, epoch=2, origin_xy=(-2.499, -2.497))
    second = cache.get(b)
    assert cache.last_reused
    assert (second.epoch, second.origin_xy) == (b.epoch, b.origin_xy)
    for name in ('distance_m', 'gradient_x', 'gradient_y'):
        assert np.shares_memory(getattr(first, name), getattr(second, name))
        np.testing.assert_array_equal(getattr(second, name), getattr(maps.build_distance_field(b), name))
        with pytest.raises(ValueError): getattr(second, name).setflags(write=True)
    cells = a.cells.copy(); cells[250, 300] = 1
    for changed in [replace(a, cells=cells), replace(a, resolution_m=.02),
                    replace(a, cells=np.zeros((400, 500), dtype=np.int8)), replace(a, allow_unknown=True)]:
        cache.get(a)
        actual = cache.get(changed)
        assert not cache.last_reused
        expected = maps.build_distance_field(changed)
        for name in ('distance_m', 'gradient_x', 'gradient_y'):
            np.testing.assert_array_equal(getattr(actual, name), getattr(expected, name))


def test_prepared_context_reuses_index_but_checks_each_trajectory(monkeypatch):
    g = grid()
    builds = []
    original = cert._CollisionContext.build.__func__
    def build(cls, *args, **kwargs):
        builds.append(1)
        return original(cls, *args, **kwargs)
    monkeypatch.setattr(cert._CollisionContext, 'build', classmethod(build))
    context = cert.prepare_certification(g)
    t = Trajectory(1, 1, [0.], [[0., 0., 0., 0., 0., 0.]], np.empty((0, 3)), 'spin')
    for _ in range(2):
        result = cert.certify(t, g, WheelLimits(), 1, context=context)
        assert result.matches(t, g, WheelLimits(), 1)
    assert len(builds) == 1
    bad = replace(t, states=[[2.4, 0., 0., 0., 0., 0.]])
    assert not cert.certify(bad, g, WheelLimits(), 1, context=context).valid
    for changed in [replace(g, epoch=2), replace(g, origin_xy=(-2.4, -2.5)), replace(g, allow_unknown=True)]:
        with pytest.raises(ValueError, match='context'):
            cert.certify(t, changed, WheelLimits(), 1, context=context)
    with pytest.raises(ValueError, match='backend'):
        cert.certify(t, g, WheelLimits(), 1, backend=object(), context=context)


def test_coordinator_prepares_one_context_for_candidate_and_action(monkeypatch):
    from scripts.navigation.one_cm.nav1cm import coordinator as module
    from scripts.navigation.one_cm.nav1cm.contracts import CellRect, VehicleState, WorldSnapshot
    from types import SimpleNamespace
    class Runner:
        busy = True
        def poll(self): return None
    calls = []
    original = cert._CollisionContext.build.__func__
    def build(cls, *args, **kwargs):
        calls.append(1)
        return original(cls, *args, **kwargs)
    monkeypatch.setattr(cert._CollisionContext, 'build', classmethod(build))
    c = module.NavigationCoordinator(SimpleNamespace(), runner=Runner())
    result = c.tick(WorldSnapshot(1, (), (CellRect(-2000, -2000, 2000, 2000),)),
                    VehicleState(0, 0, 0, .1, 0), (3, 0), 0)
    assert result.certificate.valid and result.action_certificate.valid
    assert len(calls) == 1


def test_cache_uses_passability_without_freezing_moving_window():
    from scripts.navigation.one_cm.nav1cm.contracts import CellRect, VehicleState, WorldSnapshot
    world = WorldSnapshot(1, (), (CellRect(-2000, -2000, 2000, 2000),))
    cache = maps.DistanceFieldCache()
    for i, x in enumerate((0., .001, .003, .009)):
        current = maps.extract_window(world, VehicleState(x, .002, 0., 0., 0.), False)
        field = cache.get(current)
        assert current.origin_xy == (x - 2.5, .002 - 2.5)
        assert field.origin_xy == current.origin_xy
        assert cache.last_reused == (i > 0)
    cells = np.zeros((500, 500), dtype=np.int8)
    strict = grid(cells=cells)
    cache.get(strict)
    cells[250, 250] = -1
    cache.get(grid(cells=cells))
    assert not cache.last_reused
    cache.get(grid(allow_unknown=True))
    cache.get(grid(cells=cells, allow_unknown=True))
    assert cache.last_reused


@pytest.mark.parametrize('case, expected', [('brake', 'BRAKING_COLLISION'),
                                            ('unknown', 'BRAKING_UNKNOWN'),
                                            ('sweep', 'COLLISION'), ('dynamics', 'DYNAMICS')])
def test_prepared_context_preserves_independent_safety_checks(case, expected):
    from scripts.navigation.one_cm.nav1cm.dynamics import step
    cells = np.zeros((500, 500), dtype=np.int8)
    if case in ('brake', 'unknown'):
        cells[215:285, 312] = -1 if case == 'unknown' else 1
        t = Trajectory(1, 1, [0.], [[0., 0., 0., .2, 0., 0.]], np.empty((0, 3)), 'forward')
    else:
        start = np.array([-.8, 0., 0., .2, 0., 0.])
        control = np.array([0., 0., .1])
        end = step(start, control, 8.)
        if case == 'sweep': cells[250, 250] = 1
        else: end[0] += .01
        t = Trajectory(1, 1, [0., 8.], [start, end], [control], 'forward')
    g = grid(cells=cells)
    prepared = cert.prepare_certification(g)
    result = cert.certify(t, g, WheelLimits(), 1, context=prepared)
    assert result == cert.certify(t, g, WheelLimits(), 1)
    assert result.reason.value == expected
    changed = cells.copy(); changed[0, 0] = 1
    with pytest.raises(ValueError, match='context'):
        cert.certify(t, grid(cells=changed), WheelLimits(), 1, context=prepared)
    for array in (prepared.hazard_prefix, prepared.sparse_rows, prepared.sparse_cols):
        with pytest.raises(ValueError): array.setflags(write=True)
