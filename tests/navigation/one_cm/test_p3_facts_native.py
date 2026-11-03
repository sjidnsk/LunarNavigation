import numpy as np
import pytest

from test_p3_map import grid_map, ScriptedTerrain
from test_p3_terrain_native import compiled, _api
from scripts.navigation.one_cm.nav1cm.p3_map import P3MapStore


def test_native_facts_revision_and_empty_frames(compiled):
    with _api().NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        initial = store.capture_facts()
        assert initial.world is store.snapshot and initial.raw_revision == 0
        store.apply(grid_map(np.full((5, 5), np.nan)))
        assert store.capture_facts() is initial
        store.apply(grid_map(np.zeros((5, 5))))
        flat = store.capture_facts()
        old_bytes = flat.sources[0].measurements.tobytes()
        store.apply(grid_map(np.full((5, 5), .01)))
        shifted = store.capture_facts()
        assert shifted.world is flat.world is store.snapshot
        assert shifted.raw_revision == flat.raw_revision + 1
        assert old_bytes == flat.sources[0].measurements.tobytes()
        assert old_bytes != shifted.sources[0].measurements.tobytes()


def test_geometry_only_fixture_cannot_publish_facts():
    store = P3MapStore(terrain=ScriptedTerrain())
    store.apply(grid_map(np.zeros((3, 3))))
    with pytest.raises(RuntimeError, match="measurements"):
        store.capture_facts()

def test_replay_eviction_and_reactivation(compiled, monkeypatch):
    with _api().NativeTerrain(compiled[0]) as terrain:
        calls = []
        actual = terrain.apply

        def counted(*args, **kwargs):
            calls.append(1)
            return actual(*args, **kwargs)

        monkeypatch.setattr(terrain, "apply", counted)
        store = P3MapStore(terrain=terrain)
        original = grid_map(np.zeros((5, 5)), .05, (-1., -1.))
        store.apply(original)
        first = store.capture_facts()
        store.apply(original)
        assert store.capture_facts() is first and len(calls) == 2
        first_id = first.sources[0].source_id
        for k in range(1, 9):
            store.apply(grid_map(np.zeros((3, 3)), .05 + k * .001, (k, k), k * .03))
        assert first_id not in terrain.active_source_ids
        before = store.capture_facts()
        history = next(s for s in before.sources if s.source_id == first_id)
        assert len(history.observation_fragments) == 25
        partial = np.full((5, 5), np.nan)
        partial[2, 2] = .125
        store.apply(grid_map(partial, .05, (-1., -1.)))
        facts = store.capture_facts()
        old = next(s for s in facts.sources if s.source_id == first_id)
        assert len(old.observation_fragments) == 24
        new = next(s for s in facts.sources if s.source_id == terrain.active_source_ids[-1])
        assert new.source_id != first_id and new.measurements["height_m"].tolist() == [.125]


def test_malformed_native_measurement_is_not_published(compiled, monkeypatch):
    from dataclasses import replace
    with _api().NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        store.apply(grid_map(np.zeros((5, 5))))
        before = store.capture_facts()
        apply = terrain.apply

        def malformed(*args, **kwargs):
            result = apply(*args, **kwargs)
            return replace(result, measurements=result.measurements[1:])

        monkeypatch.setattr(terrain, "apply", malformed)
        with pytest.raises(ValueError, match="measurement"):
            store.apply(grid_map(np.ones((5, 5)), .03))
        assert store.snapshot is before.world and store.capture_facts() is before


def support_at(facts, layer, x, y):
    # 点严格位于待查询片段内部，坐标单位为厘米。
    hits = []
    for source in facts.sources:
        for fragment in getattr(source, layer + "_fragments"):
            p = fragment.polygon_cm
            xs, ys = zip(*p)
            if not (min(xs) <= x <= max(xs) and min(ys) <= y <= max(ys)):
                continue
            if all((b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]) >= 0
                   for a, b in zip(p, p[1:] + p[:1])):
                hits.append(source.measurements[fragment.measurement_index])
    return hits


@pytest.mark.parametrize("obstacle", [False, True])
def test_finite_unknown_changes_height_and_only_retracts_free(compiled, obstacle):
    from test_p3_map import state_at
    with _api().NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        flat = np.zeros((9, 9))
        flat[4, 4] = .4 if obstacle else 0.
        store.apply(grid_map(flat, .05))
        assert state_at(store.snapshot, .225, .225) == int(obstacle)
        store.apply(grid_map([[.75]], .01, (.22, .22)))
        facts = store.capture_facts()
        assert state_at(facts.world, .225, .225) == (1 if obstacle else -1)
        assert [r["height_m"] for r in support_at(facts, "observation", 22.5, 22.5)] == [.75]
        assert [r["state"] for r in support_at(facts, "classification", 22.5, 22.5)] == ([1] if obstacle else [])


def test_partial_subcentimeter_reobservation_preserves_foreign_support(compiled):
    from test_p3_terrain_native import fresh_coverage_case
    from test_p3_map import state_at
    initial, blocked, left, right = fresh_coverage_case()
    with _api().NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        store.apply(grid_map(initial, .005, (.003, .003)))
        store.apply(grid_map(blocked, .01))
        store.apply(grid_map(left, .005, (.003, .003)))
        partial = store.capture_facts()
        assert state_at(partial.world, .045, .045) == 1
        assert [r["state"] for r in support_at(partial, "classification", 4.7, 4.7)] == [1]
        assert [r["height_m"] for r in support_at(partial, "observation", 4.7, 4.7)] == pytest.approx([.4])
        store.apply(grid_map(right, .005, (.003, .003)))
        complete = store.capture_facts()
        assert state_at(complete.world, .045, .045) == 0
        assert [r["state"] for r in support_at(complete, "classification", 4.7, 4.7)] == [0]


def test_rotated_rolling_keeps_signed_source_coordinates(compiled):
    yaw = .23
    with _api().NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        origin = (-1., -.5)
        store.apply(grid_map(np.zeros((3, 3)), .05, origin, yaw))
        first = store.capture_facts()
        shift = (origin[0] - .05 * np.cos(yaw), origin[1] - .05 * np.sin(yaw))
        store.apply(grid_map(np.zeros((3, 3)), .05, shift, yaw))
        facts = store.capture_facts()
        source, = facts.sources
        assert source.source_id == first.sources[0].source_id
        assert set(source.measurements["x"]) == {-1, 0, 1, 2}
        assert facts.world is store.snapshot and facts.raw_revision > first.raw_revision


@pytest.mark.parametrize("capture_between", [False, True])
def test_local_update_reuses_unaffected_sources_without_reencoding(compiled, monkeypatch, capture_between):
    from scripts.navigation.one_cm.nav1cm import p3_evidence
    with _api().NativeTerrain(compiled[0]) as terrain:
        store = P3MapStore(terrain=terrain)
        for k in range(3):
            store.apply(grid_map(np.zeros((20, 20)), .05 + k * .001, (3 * k, 0.)))
            if capture_between:
                store.capture_facts()
        before = store.capture_facts()
        encoded = []
        compact = p3_evidence._compact

        def counted(source, *args):
            encoded.append(source.source_id)
            return compact(source, *args)

        monkeypatch.setattr(p3_evidence, "_compact", counted)
        frame = grid_map([[.01]], .052, (6.52, .52))
        store.apply(frame)
        after = store.capture_facts()
        assert after.raw_revision > before.raw_revision
        for previous, current in zip(before.sources[:2], after.sources[:2]):
            assert current is previous
            assert current.measurements is previous.measurements
            assert previous.source_id not in encoded
        encoded.clear()
        store.apply(frame)
        assert store.capture_facts() is after
        assert encoded == [], "duplicate frame must not reencode retained measurements"
