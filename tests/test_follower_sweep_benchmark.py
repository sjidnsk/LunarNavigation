"""独立 benchmark oracle 的几何契约。"""
import importlib.util
from pathlib import Path
import math
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np


def module():
    path = Path(__file__).resolve().parents[1] / 'scripts/navigation/benchmark_follower_sweep.py'
    spec = importlib.util.spec_from_file_location('sweep_benchmark', path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_closed_corner_contact_and_millimetre_near_miss():
    b = module()
    assert b.arc_hits_box((0., 0., 0.), 1., 0., 1., (1., 0.), (2., 1.))
    assert not b.arc_hits_box((0., -.001, 0.), 1., 0., 1., (1., 0.), (2., 1.))


def test_curved_arc_hit_that_endpoint_chord_misses():
    b = module()
    lo = (.69, .27)
    hi = (.73, .32)
    assert b.arc_hits_box((0., 0., 0.), 1., 1., math.pi/2, lo, hi)
    assert not b.arc_hits_box((0., 0., math.pi/4), 1., 0., math.sqrt(2.), lo, hi)


def test_case09_fixture_records_actual_geometry_and_non_replay_boundary():
    b = module()
    case = next(c for c in b.fixtures() if c['name'] == 'case09_mm_constructed')
    assert case['resolution'] == .2
    assert case['pose'] == (8.9423022725, 4.9381833596, 2.450581676)
    assert case['states'][1, 0] == 2
    assert 'constructed' in case['note']
    assert np.isclose(case['origin'][0]+.2, 8.940919534)
    assert np.isclose(case['origin'][1]+.4, 4.940919534)


def test_timing_warmup_does_not_latch_input_and_event_map_is_applied(monkeypatch):
    b = module()

    @dataclass
    class State:
        x: float
        y: float
        yaw: float
        v: float
        w: float

    class Controller:
        def __init__(self):
            self.phase = 'BRAKING'
            self.failure = None
            self.diagnostics = {}
            self.map = SimpleNamespace(revision=1)

        def set_map(self, grid):
            self.map = grid

        def update(self, *_args, **_kwargs):
            first = self.failure is None
            self.phase = 'FAILED'
            self.failure = 'latched'
            self.diagnostics['selected'] = 'first_rejection' if first else 'latched_fastpath'
            return SimpleNamespace(phase=self.phase)

    def grid(states, _resolution, _origin, revision=1, **_kwargs):
        return SimpleNamespace(states=states, revision=revision)

    monkeypatch.setattr(b, 'make_controller', lambda *_args: Controller())
    api = SimpleNamespace(classes={'baseline': Controller, 'current': Controller},
                          TrackingState=State, GridMap=grid)
    for name, expected in [('map_change', 2), ('map_unknown_change', 0)]:
        case = next(c for c in b.fixtures() if c['name'] == name)
        before = case['states'].copy()
        result = b.timing_comparison(api, case, 4, 17)
        assert result['map_update_phase'] == 'post_event_fixed_initial_state'
        assert result['timed_map_state_counts'][str(expected)] > 0
        for backend in api.classes:
            assert result['input_snapshot'][backend]['failure'] is None
            assert result['input_snapshot'][backend]['map_revision'] == 2
            assert result['observed_phase_and_selection'][backend] == {'FAILED/first_rejection': 4}
        np.testing.assert_array_equal(case['states'], before)
