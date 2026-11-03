"""观测事实协议只编码已有片段，不重新分类或折叠历史。"""
import importlib
import json

import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm.base_map import BaseMapFacts, EvidenceFragment, SourceEvidence
from scripts.navigation.one_cm.nav1cm.contracts import WorldSnapshot
from scripts.navigation.one_cm.nav1cm.terrain_native import MEASUREMENT_DTYPE


def codec():
    try:
        return importlib.import_module("scripts.navigation.one_cm.nav1cm.facts_codec")
    except ModuleNotFoundError:
        pytest.fail("observation facts JSON protocol is missing")


def facts():
    values = np.array([(3, 4, 1., 1, .2, .3),
                       (3, 4, 2., -1, float("nan"), float("nan"))], dtype=MEASUREMENT_DTYPE)
    old = EvidenceFragment(0, ((-100., 0.), (0., 100.), (100., 0.), (0., -100.)))
    current = EvidenceFragment(1, ((-50., 0.), (0., 50.), (50., 0.)))
    one = SourceEvidence(17, .2, (-1., 2.), .7, values, (old, current, current), (old,))
    two = SourceEvidence(99, .05, (3., 4.), -.2, values[:1], (old,), ())
    return BaseMapFacts(WorldSnapshot(0, (), ()), 2**63 + 123, (one, two))


def encoded(value=None, **kwargs):
    return codec().encode_observation_facts(facts() if value is None else value,
        source_resolution_m=.2, frame_id="map", epoch=7, **kwargs)


def test_roundtrip_preserves_observation_order_history_units_and_identity():
    value = facts()
    before = [source.measurements.tobytes() for source in value.sources]
    text = encoded(value)
    parsed = json.loads(text)
    assert parsed["schema"] == "lunar-observation-facts/v1"
    assert parsed["raw_revision"] == value.raw_revision
    assert parsed["frame"] == "map" and parsed["epoch"] == 7
    assert parsed["source_resolution_m"] == .2 and parsed["complete_snapshot"] is True
    fragments = parsed["observation_fragments"]
    assert len(fragments) == 4
    assert [f["state"] for f in fragments] == [1, -1, -1, 1]
    assert [f["height_m"] for f in fragments] == [1., 2., 2., 1.]
    assert fragments[0]["polygon"] == [[-1., 0.], [0., 1.], [1., 0.], [0., -1.]]
    assert fragments[1]["slope_rad"] is None and fragments[1]["relief_m"] is None
    assert "source_id" not in text and "classification" not in text
    assert "NaN" not in text and "Infinity" not in text
    assert codec().decode_observation_facts(text) == parsed
    assert [source.measurements.tobytes() for source in value.sources] == before
    assert all(not source.measurements.flags.writeable for source in value.sources)


@pytest.mark.parametrize("observed_state", [-1, 0, 1])
def test_observation_state_is_independent_of_retained_blocked_classification(observed_state):
    values = np.array([(0, 0, 1., 1, .2, .3),
                       (0, 0, 2., observed_state, float("nan"), float("nan"))],
                      dtype=MEASUREMENT_DTYPE)
    polygon = ((0., 0.), (10., 0.), (0., 10.))
    old = EvidenceFragment(0, polygon)
    new = EvidenceFragment(1, polygon)
    source = SourceEvidence(1, .05, (0., 0.), 0., values, (new,), (old,))
    value = BaseMapFacts(WorldSnapshot(0, (), ()), 9, (source,))
    fragments = codec().decode_observation_facts(encoded(value))["observation_fragments"]
    assert len(fragments) == 1
    assert fragments[0]["state"] == observed_state
    assert fragments[0]["height_m"] == 2.


def test_complete_empty_snapshot_is_a_real_snapshot():
    empty = BaseMapFacts(WorldSnapshot(0, (), ()), 0, ())
    parsed = codec().decode_observation_facts(encoded(empty))
    assert parsed["observation_fragments"] == []
    assert parsed["complete_snapshot"] is True
    assert parsed["raw_revision"] == 0


@pytest.mark.parametrize("key,bad", [
    ("schema", "lunar-observation-facts/v2"), ("complete_snapshot", False),
    ("complete_snapshot", 1), ("source_resolution_m", 0),
    ("source_resolution_m", -1), ("source_resolution_m", float("nan")),
    ("source_resolution_m", True), ("raw_revision", 1.5), ("raw_revision", -1),
    ("raw_revision", True), ("frame", ""), ("frame", None), ("epoch", -1),
    ("epoch", False), ("observation_fragments", {}),
])
def test_rejects_invalid_envelope(key, bad):
    data = json.loads(encoded())
    data[key] = bad
    with pytest.raises(ValueError):
        codec().decode_observation_facts(json.dumps(data))


@pytest.mark.parametrize("key,bad", [
    ("height_m", float("nan")), ("height_m", float("inf")),
    ("height_m", None), ("height_m", True), ("state", 2), ("state", False),
    ("state", 0.0), ("slope_rad", float("inf")), ("slope_rad", -.1),
    ("relief_m", float("nan")), ("polygon", [[0., 0.], [1., 1.]]),
    ("polygon", [[0., 0.], [1., 1.], [float("inf"), 1.]]),
    ("polygon", [[0., 0.], [1., 1.], [2., 3., 4.]]),
])
def test_rejects_invalid_fragment(key, bad):
    data = json.loads(encoded())
    data["observation_fragments"][0][key] = bad
    with pytest.raises(ValueError):
        codec().decode_observation_facts(json.dumps(data))


def test_rejects_missing_fields_extra_native_identity_and_duplicate_keys():
    data = json.loads(encoded())
    for key in tuple(data):
        invalid = dict(data)
        del invalid[key]
        with pytest.raises(ValueError):
            codec().decode_observation_facts(json.dumps(invalid))
    for key in tuple(data["observation_fragments"][0]):
        invalid = json.loads(encoded())
        del invalid["observation_fragments"][0][key]
        with pytest.raises(ValueError):
            codec().decode_observation_facts(json.dumps(invalid))
    data["observation_fragments"][0]["source_id"] = 17
    with pytest.raises(ValueError):
        codec().decode_observation_facts(json.dumps(data))
    with pytest.raises(ValueError):
        codec().decode_observation_facts('{"schema":"a","schema":"b"}')


@pytest.mark.parametrize("resolution", [0., -.1, float("inf"), float("nan")])
def test_encoder_rejects_nonpositive_or_nonfinite_source_resolution(resolution):
    with pytest.raises(ValueError):
        codec().encode_observation_facts(facts(), source_resolution_m=resolution,
                                        frame_id="map", epoch=0)


def test_processed_stamp_defaults_to_zero_and_preserves_large_integer():
    assert codec().decode_observation_facts(encoded())["processed_stamp_ns"] == 0
    stamp = 2**31 * 10**9 - 1
    parsed = codec().decode_observation_facts(encoded(processed_stamp_ns=stamp))
    assert parsed["processed_stamp_ns"] == stamp
    for invalid in (-1, 2**63, 1.5, True):
        with pytest.raises(ValueError):
            encoded(processed_stamp_ns=invalid)


@pytest.mark.parametrize("field", ["raw_revision", "epoch"])
def test_protocol_integer_capacity_matches_uint64_consumer(field):
    data = json.loads(encoded())
    data[field] = 2**64 - 1
    assert codec().decode_observation_facts(json.dumps(data))[field] == 2**64 - 1
    data[field] = 2**64
    with pytest.raises(ValueError):
        codec().decode_observation_facts(json.dumps(data))
    original = facts()
    if field == "raw_revision":
        oversized = BaseMapFacts(original.world, 2**80, original.sources)
        with pytest.raises(ValueError):
            encoded(oversized)
    else:
        with pytest.raises(ValueError):
            codec().encode_observation_facts(original, source_resolution_m=.2,
                                            frame_id="map", epoch=2**80)


def test_height_capacity_matches_existing_float32_elevation_storage():
    data = json.loads(encoded())
    limit = float(np.finfo(np.float32).max)
    data["observation_fragments"][0]["height_m"] = limit
    assert codec().decode_observation_facts(json.dumps(data))["observation_fragments"][0]["height_m"] == limit
    for height in (limit * 2, -limit * 2):
        data["observation_fragments"][0]["height_m"] = height
        with pytest.raises(ValueError):
            codec().decode_observation_facts(json.dumps(data))
    values = np.array([(0, 0, 1e39, -1, float("nan"), float("nan"))], dtype=MEASUREMENT_DTYPE)
    fragment = EvidenceFragment(0, ((0., 0.), (1., 0.), (0., 1.)))
    source = SourceEvidence(1, .05, (0., 0.), 0., values, (fragment,), ())
    with pytest.raises(ValueError):
        encoded(BaseMapFacts(WorldSnapshot(0, (), ()), 1, (source,)))

@pytest.mark.parametrize("stamp", [2147483648000000000, 2**63-17])
def test_stamp_outside_ros_time_cannot_enter_shared_protocol(stamp):
    with pytest.raises(ValueError, match="stamp"):
        encoded(processed_stamp_ns=stamp)
