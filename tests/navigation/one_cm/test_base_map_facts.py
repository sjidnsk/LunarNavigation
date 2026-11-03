"""Immutable provenance value contract; integration tests follow in Task 1B.1."""
import importlib
from dataclasses import FrozenInstanceError
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import WorldSnapshot
from scripts.navigation.one_cm.nav1cm.terrain_native import MEASUREMENT_DTYPE


def api():
    try:
        return importlib.import_module('scripts.navigation.one_cm.nav1cm.base_map')
    except ModuleNotFoundError:
        pytest.fail('immutable canonical evidence value contract is missing')


def test_evidence_owns_immutable_measurement_bytes_and_polygon_fragments():
    module = api()
    measurements = np.array([(-3, 4, 2., -1, float('nan'), float('nan'))], dtype=MEASUREMENT_DTYPE)
    fragment = module.EvidenceFragment(0, ((0., 0.), (.5, 0.), (.5, 1.), (0., 1.)))
    source = module.SourceEvidence(1, .005, (0., 0.), 0., measurements, (fragment,))
    facts = module.BaseMapFacts(WorldSnapshot(0, (), ()), 1, (source,))
    measurements['height_m'] = 8.
    assert facts.sources[0].measurements['height_m'][0] == 2.
    assert facts.sources[0].measurements['x'][0] == -3
    with pytest.raises(ValueError):
        source.measurements.setflags(write=True)
    with pytest.raises(FrozenInstanceError):
        facts.raw_revision = 2
    assert source.observation_fragments[0].polygon_cm[1] == (.5, 0.)
    assert not hasattr(source, "classification_fragments")


def test_fragment_reference_validation():
    module = api()
    fragment = module.EvidenceFragment(0, ((0., 0.), (1., 0.), (1., 1.)))
    with pytest.raises(ValueError, match='measurement'):
        module.SourceEvidence(1, .01, (0., 0.), 0., np.empty(0, dtype=MEASUREMENT_DTYPE), (fragment,))


def test_same_cell_historical_values_have_independent_fragment_references():
    module = api()
    values = np.array([(-3, 4, 1., 1, .2, .3),
                       (-3, 4, 2., -1, float('nan'), float('nan'))], dtype=MEASUREMENT_DTYPE)
    old_fragment = module.EvidenceFragment(0, ((0., 0.), (.5, 0.), (.5, 1.), (0., 1.)))
    new_fragment = module.EvidenceFragment(1, ((.5, 0.), (1., 0.), (1., 1.), (.5, 1.)))
    source = module.SourceEvidence(1, .01, (0., 0.), 0., values,
                                   (old_fragment, new_fragment))
    facts = module.BaseMapFacts(WorldSnapshot(0, (), ()), 1, (source,))
    values['height_m'] = 99.
    retained = facts.sources[0]
    assert [retained.measurements[f.measurement_index]['height_m']
            for f in retained.observation_fragments] == [1., 2.]
    assert list(retained.measurements['x']) == [-3, -3]
    with pytest.raises(ValueError):
        retained.measurements.setflags(write=True)

def test_small_triangle_at_negative_world_coordinates():
    from scripts.navigation.one_cm.nav1cm.base_map import EvidenceFragment
    x = -10000.
    p = ((x, x), (x + 1e-5, x), (x, x + 1e-5))
    assert EvidenceFragment(0, p).polygon_cm == p
