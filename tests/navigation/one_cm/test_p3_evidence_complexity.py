"""真实几何差分与工作量回归：防止恢复逐 span 扫描全部证据。"""

from collections import Counter, defaultdict
from dataclasses import replace
import math
import random
import sys

import numpy as np
import pytest

from test_p3_terrain_native import compiled, _api
from test_convex_polygon_native import area
from scripts.navigation.one_cm.nav1cm import p3_evidence as kernel
from scripts.navigation.one_cm.nav1cm.base_map import EvidenceFragment, SourceEvidence, MEASUREMENT_DTYPE
from scripts.navigation.one_cm.nav1cm.contracts import CellRect


def _source(identity, states, resolution=.01, origin=(0., 0.), yaw=0.):
    rows = [(x, y, float(identity), int(state), np.nan, np.nan)
            for y, row in enumerate(states) for x, state in enumerate(row)]
    return SourceEvidence(identity, resolution, origin, yaw,
                          np.array(rows, dtype=MEASUREMENT_DTYPE), (), ())


def _cell_polygon(source, record):
    # 独立坐标变换，不复用被测模块的候选生成和空间筛选。
    scale = source.resolution_m * 100
    c, s = math.cos(source.yaw), math.sin(source.yaw)
    x, y = int(record['x']), int(record['y'])
    return tuple((source.origin_xy_m[0] * 100 + scale * (c * u - s * v),
                  source.origin_xy_m[1] * 100 + scale * (s * u + c * v))
                 for u, v in ((x, y), (x + 1, y), (x + 1, y + 1), (x, y + 1)))


def _slow_new_source(previous, incoming, direct, spans, ops):
    """小输入的顺序语义 oracle；仅支持新 source，不复制生产内核的记录管理。"""
    assert incoming.source_id not in {s.source_id for s in previous}
    polygons = [_cell_polygon(incoming, r) for r in incoming.measurements]
    layers = {s.source_id: [s, list(s.observation_fragments), list(s.classification_fragments)]
              for s in previous}
    layers[incoming.source_id] = [incoming, [EvidenceFragment(i, polygons[i]) for i in direct], []]

    def subtract(fragment, clip):
        # Native ABI 对不相交/极小舍入碎片的重复差集会放大数值噪声。
        if not ops.polygon_intersection(fragment.polygon_cm, clip):
            return [fragment]
        return [EvidenceFragment(fragment.measurement_index, p)
                for p in ops.polygon_difference(fragment.polygon_cm, clip) if area([p]) > 1e-10]

    for source, observations, _ in list(layers.values())[:-1]:
        for i in direct:
            observations[:] = [p for f in observations for p in subtract(f, polygons[i])]
    for state, rect in spans:
        clip = ((rect.x0, rect.y0), (rect.x1, rect.y0),
                (rect.x1, rect.y1), (rect.x0, rect.y1))
        for source, _, classes in layers.values():
            remaining = []
            for f in classes:
                if state == -1 and source.measurements[f.measurement_index]['state'] != 0:
                    remaining.append(f)
                else:
                    remaining.extend(subtract(f, clip))
            classes[:] = remaining
        if state != -1:
            for i, record in enumerate(incoming.measurements):
                if record['state'] == state:
                    layers[incoming.source_id][2].extend(EvidenceFragment(i, p) for p in
                                                        ops.polygon_intersection(polygons[i], clip))
    return tuple(replace(s, observation_fragments=tuple(o), classification_fragments=tuple(c))
                 for s, o, c in layers.values() if o or c)


def _supports(sources, layer):
    result = defaultdict(list)
    for source in sources:
        for fragment in getattr(source, layer + '_fragments'):
            if area([fragment.polygon_cm]) <= 1e-10:
                continue
            record = source.measurements[fragment.measurement_index]
            key = (source.source_id, int(record['x']), int(record['y']),
                   float(record['height_m']), int(record['state']))
            result[key].append(fragment.polygon_cm)
    return result


def _assert_same_support(actual, expected, ops):
    for layer in ('observation', 'classification'):
        left, right = _supports(actual, layer), _supports(expected, layer)
        assert left.keys() == right.keys()
        for key in left:
            common = sum(area(ops.polygon_intersection(a, b)) for a in left[key] for b in right[key])
            assert area(left[key]) == pytest.approx(area(right[key]), abs=1e-8)
            assert common == pytest.approx(area(right[key]), abs=1e-8)


@pytest.mark.parametrize('seed', [3, 17, 91])
@pytest.mark.parametrize('reverse', [False, True])
def test_spatial_update_matches_sequential_native_geometry_with_rotated_subcm_sources(compiled, seed, reverse):
    rng = random.Random(seed)
    with _api().NativeTerrain(compiled[0]) as ops:
        actual = expected = ()
        for identity in range(1, 7):
            states = [[rng.choice((-1, 0, 1)) for _ in range(3)] for _ in range(2)]
            incoming = _source(identity, states, rng.choice((.005, .013, .02)),
                               (rng.uniform(-.01, .01), rng.uniform(-.01, .01)),
                               rng.choice((0., .37, -.81)))
            spans = [(rng.choice((-1, 0, 1)), CellRect(x, y, x + 2, y + 2))
                     for x, y in ((-1, -1), (0, 0), (1, 0), (0, 1), (-1, 0))]
            rng.shuffle(spans)
            if reverse:
                spans.reverse()
            direct = tuple(i for i in range(6) if rng.random() < .75)
            frozen = [(s.measurements.tobytes(), s.observation_fragments, s.classification_fragments)
                      for s in actual]
            saved = actual
            actual = kernel.update_evidence(actual, incoming, direct, tuple(spans), ops)
            expected = _slow_new_source(expected, incoming, direct, spans, ops)
            _assert_same_support(actual, expected, ops)
            assert frozen == [(s.measurements.tobytes(), s.observation_fragments,
                               s.classification_fragments) for s in saved]


def test_unknown_keeps_old_blocked_until_later_clear_span(compiled):
    with _api().NativeTerrain(compiled[0]) as ops:
        blocked = _source(1, [[1]], .02)
        old = kernel.update_evidence((), blocked, (0,), ((1, CellRect(0, 0, 2, 2)),), ops)
        unknown = _source(2, [[-1]], .02)
        retained = kernel.update_evidence(old, unknown, (0,), ((-1, CellRect(0, 0, 2, 2)),), ops)
        assert area(_supports(retained, 'classification')[(1, 0, 0, 1., 1)]) == pytest.approx(4.)
        clear = _source(3, [[0]], .02)
        spans = ((0, CellRect(0, 0, 2, 2)), (-1, CellRect(1, 0, 2, 2)))
        result = kernel.update_evidence(retained, clear, (0,), spans, ops)
        support = _supports(result, 'classification')
        assert set(support) == {(3, 0, 0, 3., 0)}
        assert area(support[(3, 0, 0, 3., 0)]) == pytest.approx(2.)
        _assert_same_support(result, _slow_new_source(retained, clear, (0,), spans, ops), ops)


@pytest.mark.parametrize('with_history', [False, True])
def test_disjoint_grid_spans_do_not_scan_or_rebuild_all_fragments(compiled, monkeypatch, with_history):
    # 棋盘格不允许矩形合并掩盖 S×F/C；旧实现达到预算即失败，不运行完平方循环。
    side = 64
    count = side * side
    states = [[(x + y) % 2 for x in range(side)] for y in range(side)]
    incoming = _source(2, states)
    spans = tuple(((x + y) % 2, CellRect(x, y, x + 1, y + 1))
                  for y in range(side) for x in range(side))
    work = Counter()
    previous = ()
    if with_history:
        old = _source(1, states)
        fragments = tuple(EvidenceFragment(i, _cell_polygon(old, r)) for i, r in enumerate(old.measurements))
        previous = (replace(old, observation_fragments=fragments, classification_fragments=fragments),)

    def counted(name, function):
        def call(*args, **kwargs):
            work[name] += 1
            assert work[name] <= 160 * count, f'{name} exceeds local-overlap work budget: {work}'
            return function(*args, **kwargs)
        return call

    for name in ('_bounds', '_overlaps', 'EvidenceFragment'):
        monkeypatch.setattr(kernel, name, counted(name, getattr(kernel, name)))

    class Geometry:
        def __init__(self, native):
            self.polygon_intersection = counted('intersection', native.polygon_intersection)
            self.polygon_difference = counted('difference', native.polygon_difference)

    def trace(frame, event, arg):
        if event == 'line' and frame.f_globals.get('__name__') == kernel.__name__:
            work['python_lines'] += 1
            assert work['python_lines'] <= 1000 * count, f'Python full-scan budget exceeded: {work}'
        return trace if frame.f_globals.get('__name__') == kernel.__name__ else None

    with _api().NativeTerrain(compiled[0]) as native:
        prior_trace = sys.gettrace()
        try:
            sys.settrace(trace)
            result = kernel.update_evidence(previous, incoming, tuple(range(count)), spans, Geometry(native))
        finally:
            sys.settrace(prior_trace)
    assert len(result) == 1
    assert len(result[0].observation_fragments) == count
    assert len(result[0].classification_fragments) <= 2 * count
    assert area(f.polygon_cm for f in result[0].classification_fragments) == pytest.approx(count)
