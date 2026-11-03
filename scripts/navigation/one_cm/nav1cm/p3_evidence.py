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


def _compact(source, records, observations):
    """仅保留有效观测引用的记录；不再为导航分类复制一份历史。"""
    used=sorted({f.measurement_index for f in observations})
    if not used:return None
    indices={old:new for new,old in enumerate(used)}
    fragments=tuple(f if indices[f.measurement_index]==f.measurement_index else
                    EvidenceFragment(indices[f.measurement_index],f.polygon_cm) for f in observations)
    return replace(source,measurements=records[used],observation_fragments=fragments)



def _update_source(old,incoming,indices,cells):
    """同格网值更新保留几何索引，只有覆盖形状变化才重建相应片段。"""
    support={} if old is None else old._observation_index
    old_indices=np.array([support[cells[i]][0][0].measurement_index if cells[i] in support else -1 for i in indices])
    present=old_indices>=0
    same=np.zeros(len(indices),dtype=bool)
    if old is not None and present.any():
        a=old.measurements[old_indices[present]];b=incoming.measurements[np.asarray(indices)[present]]
        equal=np.ones(len(a),dtype=bool)
        for name in a.dtype.names:
            equal &= (a[name]==b[name]) | (np.isnan(a[name]) & np.isnan(b[name]))
        same[present]=equal
    geometry={cells[i] for i in indices if cells[i] not in support or not support[cells[i]][1]}
    if old is not None and same.all() and not geometry:return old,geometry
    records=np.empty(0,dtype=incoming.measurements.dtype) if old is None else old.measurements.copy()
    fragments=[] if old is None else [f for f in old.observation_fragments
        if _cell(old.measurements[f.measurement_index]) not in geometry]
    index=support.copy();added=[]
    for position,i in enumerate(indices):
        cell=cells[i];record_index=int(old_indices[position])
        if record_index<0:
            record_index=len(records)+len(added);added.append(incoming.measurements[i])
        elif not same[position]:records[record_index]=incoming.measurements[i]
        if cell in geometry:
            x,y=cell;f=EvidenceFragment(record_index,tuple(_polygon((x,y,x+1,y+1,0),incoming)))
            fragments.append(f);index[cell]=((f,),True)
    if added:records=np.concatenate((records,np.array(added,dtype=records.dtype)))
    geometry_unchanged=old is not None and not geometry
    fs=old.observation_fragments if geometry_unchanged else tuple(fragments)
    value=replace(incoming,measurements=records,observation_fragments=fs)
    if len({f.measurement_index for f in fs})!=len(records):
        value=_compact(value,records,fs)
    else:
        value.__dict__['_observation_index']=index
        if geometry_unchanged and '_support_bounds_cm' in old.__dict__:
            value.__dict__['_support_bounds_cm']=old._support_bounds_cm
    return value,geometry

def update_evidence(previous_sources: tuple[SourceEvidence,...], incoming_source: SourceEvidence,
                    direct_measurement_indices: tuple[int,...], geometry_ops: NativeTerrain) -> tuple[SourceEvidence,...]:
    """只维护当前有效观测；导航分类由P3MapStore既有存储独立维护。"""
    incoming=incoming_source
    sources={s.source_id:s for s in previous_sources}
    if len(sources)!=len(previous_sources) or incoming.observation_fragments:
        raise ValueError('expected unique history sources and uncommitted incoming measurements')
    old=sources.get(incoming.source_id)
    if old and any(getattr(old,n)!=getattr(incoming,n) for n in ('resolution_m','origin_xy_m','yaw')):
        raise ValueError('source lattice changed')
    indices=tuple(operator.index(i) for i in direct_measurement_indices)
    cells=[_cell(r) for r in incoming.measurements]
    if len(set(cells))!=len(cells) or len(set(indices))!=len(indices):raise ValueError('duplicate incoming cell or direct index')
    if any(i<0 or i>=len(cells) for i in indices):raise ValueError('invalid direct measurement index')
    if not indices:return previous_sources
    value,geometry_cells=_update_source(old,incoming,indices,cells)
    if old is value:return previous_sources
    # 同格网只有测量值变化时，覆盖没有变化，不必裁剪任何其他来源。
    coverage=tuple(_coverage(geometry_cells,incoming))
    coverage_index=_BoxIndex(box for box,_ in coverage)
    result=[]
    for source in previous_sources:
        if source.source_id==incoming.source_id:continue
        if source._support_bounds_cm is None or not any(_overlaps(source._support_bounds_cm,b) for b,_ in coverage):
            result.append(source);continue
        fragments=[]
        for f in source.observation_fragments:
            cutters=(coverage[i] for i in coverage_index.query(_bounds(f.polygon_cm)))
            fragments.extend(_cut(f,cutters,geometry_ops))
        if tuple(fragments)==source.observation_fragments:result.append(source)
        else:
            trimmed=_compact(source,source.measurements,fragments)
            if trimmed is not None:result.append(trimmed)
    # 保持源顺序稳定，来源身份不是本次更新的顺序。
    by_id={s.source_id:s for s in result};by_id[value.source_id]=value
    return tuple(by_id[s.source_id] for s in previous_sources if s.source_id in by_id)+(() if old else (value,))
