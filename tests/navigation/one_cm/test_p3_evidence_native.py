import math

import numpy as np
import pytest

from test_p3_terrain_native import compiled, _api
from test_convex_polygon_native import area
from scripts.navigation.one_cm.nav1cm.base_map import SourceEvidence, MEASUREMENT_DTYPE
from scripts.navigation.one_cm.nav1cm.contracts import CellRect


def update_evidence(*args):
    try:
        from scripts.navigation.one_cm.nav1cm.p3_evidence import update_evidence as kernel
    except ModuleNotFoundError as error:
        if error.name != 'scripts.navigation.one_cm.nav1cm.p3_evidence':
            raise
        pytest.fail('canonical evidence update kernel is missing')
    return kernel(*args)


def source(identity, state, height, origin=(0., 0.), yaw=0.):
    records = np.array([(0, 0, height, state, np.nan, np.nan)], dtype=MEASUREMENT_DTYPE)
    return SourceEvidence(identity, .02, origin, yaw, records, (), ())


def size(source, kind):
    return area([f.polygon_cm for f in getattr(source, kind + '_fragments')])


@pytest.mark.parametrize('state', [0, 1])
def test_partial_unknown_and_halo(compiled, state):
    with _api().NativeTerrain(compiled[0]) as ops:
        old = source(1, state, 2.)
        whole = CellRect(0, 0, 2, 2)
        a, = update_evidence((), old, (0,), ((state, whole),), ops)
        saved = a.measurements.tobytes()
        b = source(2, -1, 3., (.01, 0.))
        spans = ((-1, CellRect(1, 0, 2, 2)),)
        left, new = update_evidence((a,), b, (0,), spans, ops)
        assert size(left, 'observation') == pytest.approx(2)
        assert size(left, 'classification') == pytest.approx(4 if state else 2)
        assert size(new, 'observation') == pytest.approx(4)
        assert not new.classification_fragments
        halo = source(1, state, 9.)
        update = ((state, CellRect(0, 0, 1, 2)),)
        revised = update_evidence((left, new), halo, (), update, ops)
        replay = update_evidence(revised, halo, (), update, ops)
        assert revised[0].observation_fragments == left.observation_fragments
        assert set(revised[0].measurements['height_m']) == {2., 9.}
        assert revised[0].classification_fragments == replay[0].classification_fragments
        assert revised[0].measurements.tobytes() == replay[0].measurements.tobytes()
        assert a.measurements.tobytes() == saved and size(a, 'observation') == 4


def test_rotated_and_distant_sources(compiled):
    with _api().NativeTerrain(compiled[0]) as ops:
        a, = update_evidence((), source(1, 0, 1.), (0,), (), ops)
        rotated = source(2, 0, 2., yaw=math.pi / 4)
        result = update_evidence((a,), rotated, (0,), (), ops)
        assert size(result[0], 'observation') == pytest.approx(4 - (4 * math.sqrt(2) - 4))
        distant = source(3, 0, 2., (100., 100.))
        retained = update_evidence(result, distant, (0,), (), ops)
        assert retained[0].observation_fragments == result[0].observation_fragments
        with pytest.raises(ValueError):
            update_evidence(result, source(1, 0, 1., (.1, 0.)), (), (), ops)
        with pytest.raises(ValueError):
            update_evidence(result, distant, (1,), (), ops)


def test_interior_cut_retains_multiple_fragments_of_one_measurement(compiled):
    from dataclasses import replace

    with _api().NativeTerrain(compiled[0]) as ops:
        a, = update_evidence((), source(1, 0, 1.), (0,), (), ops)
        small = replace(source(2, -1, 3., (.005, .005)), resolution_m=.01)
        result = update_evidence((a,), small, (0,), (), ops)
        assert size(result[0], 'observation') == pytest.approx(3.)
        assert len(result[0].observation_fragments) >= 4
        assert {f.measurement_index for f in result[0].observation_fragments} == {0}
        replay = update_evidence(result, small, (0,), (), ops)
        assert replay[0].observation_fragments == result[0].observation_fragments


def test_unreferenced_incoming_measurements_do_not_create_history(compiled):
    with _api().NativeTerrain(compiled[0]) as ops:
        assert update_evidence((), source(1, 0, 1.), (), (), ops) == ()
