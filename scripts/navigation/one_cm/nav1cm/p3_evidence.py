"""按已获准的世界写入范围更新不可变源测量证据。"""

from collections import defaultdict
from dataclasses import replace
import math
import operator

import numpy as np

from .base_map import EvidenceFragment, SourceEvidence
from .contracts import CellRect
from .p3_map import _polygon, _rectangles
from .terrain_native import NativeTerrain


def _record_key(record):
    return tuple(None if isinstance(v, float) and math.isnan(v) else v for v in record.tolist())


def _cell(record):
    return int(record['x']), int(record['y'])


def _bounds(poly):
    xs, ys = zip(*poly)
    return min(xs), min(ys), max(xs), max(ys)


def _overlaps(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def _contains(outer, inner):
    return outer[0] <= inner[0] and outer[1] <= inner[1] and outer[2] >= inner[2] and outer[3] >= inner[3]


class _BoxIndex:
    """静态包围盒树；查询恢复输入顺序，几何精裁剪仍由原内核负责。"""

    def __init__(self, boxes):
        self.boxes = tuple(boxes)

        def build(indices):
            bounds = (min(self.boxes[i][0] for i in indices), min(self.boxes[i][1] for i in indices),
                      max(self.boxes[i][2] for i in indices), max(self.boxes[i][3] for i in indices))
            if len(indices) <= 8:
                return bounds, indices, None, None
            axis = 0 if bounds[2] - bounds[0] >= bounds[3] - bounds[1] else 1
            ordered = sorted(indices, key=lambda i: self.boxes[i][axis] + self.boxes[i][axis + 2])
            middle = len(ordered) // 2
            return bounds, None, build(ordered[:middle]), build(ordered[middle:])

        self.root = build(list(range(len(self.boxes)))) if self.boxes else None

    def query(self, bounds):
        result = []
        pending = [self.root] if self.root is not None else []
        while pending:
            box, indices, left, right = pending.pop()
            if not _overlaps(box, bounds):
                continue
            if indices is not None:
                result.extend(i for i in indices if _overlaps(self.boxes[i], bounds))
            else:
                pending.extend((left, right))
        return sorted(result)


def _coverage(cells, source):
    rows = defaultdict(list)
    for x, y in sorted(cells, key=lambda p: (p[1], p[0])):
        row = rows[y]
        if row and row[-1][1] == x:
            row[-1][1] += 1
        else:
            row.append([x, x + 1, 0])
    for region in _rectangles(rows):
        poly = tuple(_polygon(region, source))
        yield _bounds(poly), poly


def _cut(fragment, cutters, ops, *, rectangles=False):
    pieces = [fragment.polygon_cm]
    original_bounds = _bounds(fragment.polygon_cm)
    changed = False
    for bounds, poly in cutters:
        if not _overlaps(original_bounds, bounds):
            continue
        remaining = []
        for piece in pieces:
            piece_bounds = _bounds(piece)
            if rectangles and _contains(bounds, piece_bounds):
                changed = True
            elif _overlaps(piece_bounds, bounds) and ops.polygon_intersection(piece, poly):
                changed = True
                remaining.extend(ops.polygon_difference(piece, poly))
            else:
                remaining.append(piece)
        pieces = remaining
        if not pieces:
            break
    return [EvidenceFragment(fragment.measurement_index, p) for p in pieces] if changed else [fragment]


def _compact(source, records, observations, classes):
    used = {f.measurement_index for layer in (observations, classes) for f in layer}
    if not used:
        return None
    # 相同记录以最后一次出现排序，保留同格历史与最新可用记录的顺序。
    keys = {}
    canonical = {}
    for index in sorted(used):
        row = records[index].tolist()
        key = tuple(None if isinstance(v, float) and math.isnan(v) else v for v in row)
        keys[index] = key
        canonical[key] = index
    ordered = sorted(canonical, key=canonical.get)
    indices = {key: i for i, key in enumerate(ordered)}

    def remap(layer):
        return tuple(EvidenceFragment(indices[keys[f.measurement_index]], f.polygon_cm) for f in layer)

    return replace(source, measurements=records[[canonical[k] for k in ordered]],
                   observation_fragments=remap(observations), classification_fragments=remap(classes))


def update_evidence(previous_sources: tuple[SourceEvidence, ...], incoming_source: SourceEvidence,
                    direct_measurement_indices: tuple[int, ...], accepted_spans: tuple[tuple[int, CellRect], ...],
                    geometry_ops: NativeTerrain) -> tuple[SourceEvidence, ...]:
    """只使用已融合的测量和实际获准的范围，不改变输入快照。"""
    incoming = incoming_source
    sources = {s.source_id: s for s in previous_sources}
    if len(sources) != len(previous_sources) or incoming.observation_fragments or incoming.classification_fragments:
        raise ValueError('expected unique history sources and uncommitted incoming measurements')
    old = sources.get(incoming.source_id)
    lattice = ('resolution_m', 'origin_xy_m', 'yaw')
    if old and any(getattr(old, name) != getattr(incoming, name) for name in lattice):
        raise ValueError('source lattice changed')
    indices = tuple(operator.index(i) for i in direct_measurement_indices)
    cells = [_cell(r) for r in incoming.measurements]
    if len(set(cells)) != len(cells) or len(set(indices)) != len(indices):
        raise ValueError('duplicate incoming cell or direct index')
    if any(i < 0 or i >= len(cells) for i in indices):
        raise ValueError('invalid direct measurement index')
    direct = {cells[i] for i in indices}
    coverage = tuple(_coverage(direct, incoming))
    coverage_index = _BoxIndex(box for box, _ in coverage)
    sources.setdefault(incoming.source_id, incoming)
    # 只处理与直接观测或已获准分类范围相交的源，远处源原样共享。
    bounds = [box for box, _ in coverage]
    for state, rect in accepted_spans:
        if state not in (-1, 0, 1) or not isinstance(rect, CellRect):
            raise ValueError('invalid accepted classification span')
        bounds.append((rect.x0, rect.y0, rect.x1, rect.y1))
    active = {sid: source for sid, source in sources.items()
              if sid == incoming.source_id or (source._support_bounds_cm is not None and
                  any(_overlaps(source._support_bounds_cm, box) for box in bounds))}
    observations, classes, records = {}, {}, {}
    for sid, source in active.items():
        same = sid == incoming.source_id
        records[sid] = source.measurements
        observations[sid] = []
        classes[sid] = list(source.classification_fragments)
        for f in source.observation_fragments:
            if same:
                if _cell(source.measurements[f.measurement_index]) not in direct:
                    observations[sid].append(f)
            else:
                cutters = (coverage[i] for i in coverage_index.query(_bounds(f.polygon_cm)))
                observations[sid].extend(_cut(f, cutters, geometry_ops))

    sid = incoming.source_id
    if old:
        latest = {_cell(r): i for i, r in enumerate(old.measurements)}
        added = [r for r in incoming.measurements
                 if _cell(r) not in latest or _record_key(r) != _record_key(old.measurements[latest[_cell(r)]])]
        if added:
            records[sid] = np.concatenate((old.measurements, np.array(added, dtype=old.measurements.dtype)))
    latest = {_cell(r): i for i, r in enumerate(records[sid])}
    candidates = []
    for (x, y), index in latest.items():
        polygon = tuple(_polygon((x, y, x + 1, y + 1, 0), incoming))
        candidates.append((index, polygon, _bounds(polygon)))
        if (x, y) in direct:
            observations[sid].append(EvidenceFragment(index, polygon))

    spans = tuple((state, (rect.x0, rect.y0, rect.x1, rect.y1),
                   ((rect.x0, rect.y0), (rect.x1, rect.y0), (rect.x1, rect.y1), (rect.x0, rect.y1)))
                  for state, rect in accepted_spans)
    span_index = _BoxIndex(box for _, box, _ in spans)

    def later_cutters(box, intrinsic_state, after=-1):
        # UNKNOWN 只能清除 FREE；保留旧 BLOCKED，与逐 span 更新完全一致。
        return ((spans[i][1], spans[i][2]) for i in span_index.query(box)
                if i > after and (spans[i][0] != -1 or intrinsic_state == 0))

    # 每个旧片段只扫描相交的 spans；片段及裁剪器都保持原有顺序。
    for owner, fragments in classes.items():
        remaining = []
        for f in fragments:
            cutters = later_cutters(_bounds(f.polygon_cm), records[owner][f.measurement_index]['state'])
            remaining.extend(_cut(f, cutters, geometry_ops, rectangles=True))
        classes[owner] = remaining

    candidate_index = _BoxIndex(box for _, _, box in candidates)
    for span_number, (state, bounds, clip) in enumerate(spans):
        if state != -1:
            for candidate_number in candidate_index.query(bounds):
                index, polygon, box = candidates[candidate_number]
                if records[sid][index]['state'] == state:
                    pieces = (polygon,) if _contains(bounds, box) else geometry_ops.polygon_intersection(polygon, clip)
                    for piece in pieces:
                        fragment = EvidenceFragment(index, piece)
                        cutters = later_cutters(_bounds(piece), state, span_number)
                        classes[sid].extend(_cut(fragment, cutters, geometry_ops, rectangles=True))

    result = []
    for owner, source in sources.items():
        if owner in active and not observations[owner] and not classes[owner]:
            continue
        if owner not in active or (records[owner] is source.measurements and
                set(observations[owner]) == set(source.observation_fragments) and
                set(classes[owner]) == set(source.classification_fragments)):
            result.append(source)
        else:
            value = _compact(source, records[owner], observations[owner], classes[owner])
            if value is not None:
                result.append(value)
    return tuple(result)
