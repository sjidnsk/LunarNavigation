"""Immutable source evidence values, independent of native accumulator lifetime.

Polygons use world centimeter coordinates and preserve sub-centimeter support.
Observation support and retained classification support are separate: a finite
UNKNOWN observation can coexist with an older BLOCKED classification.
"""
from dataclasses import dataclass, field
from functools import cached_property
import math
import operator
import numpy as np
from .contracts import WorldSnapshot
from .terrain_native import MEASUREMENT_DTYPE


@dataclass(frozen=True)
class EvidenceFragment:
    measurement_index: int
    polygon_cm: tuple[tuple[float, float], ...]

    def __post_init__(self):
        index = operator.index(self.measurement_index)
        polygon = tuple(tuple(float(v) for v in point) for point in self.polygon_cm)
        if index < 0 or len(polygon) < 3 or any(len(p) != 2 or not all(map(math.isfinite, p)) for p in polygon):
            raise ValueError('fragment requires nonnegative measurement index and finite polygon')
        x, y = polygon[0]
        area = sum((a[0] - x) * (b[1] - y) - (b[0] - x) * (a[1] - y)
                   for a, b in zip(polygon[1:], polygon[2:]))
        if area == 0 or not math.isfinite(area):
            raise ValueError('fragment polygon must have nonzero finite area')
        object.__setattr__(self, 'measurement_index', index)
        object.__setattr__(self, 'polygon_cm', polygon)


@dataclass(frozen=True, eq=False)
class SourceEvidence:
    source_id: int
    resolution_m: float
    origin_xy_m: tuple[float, float]
    yaw: float
    measurements: np.ndarray = field(repr=False)
    observation_fragments: tuple[EvidenceFragment, ...]

    def __post_init__(self):
        identity = operator.index(self.source_id)
        origin = tuple(float(v) for v in self.origin_xy_m)
        if identity <= 0 or len(origin) != 2 or not all(map(math.isfinite, origin)) or not math.isfinite(self.yaw) or not math.isfinite(self.resolution_m) or self.resolution_m <= 0:
            raise ValueError('source identity and geometry must be valid')
        values = np.asarray(self.measurements)
        if values.dtype != MEASUREMENT_DTYPE or values.ndim != 1:
            raise ValueError('measurements require exact structured source dtype')
        # 同一源格的历史值可由不同片段引用；数组索引标识记录，而非坐标。
        if not np.isfinite(values['height_m']).all() or not np.isin(values['state'], [-1, 0, 1]).all():
            raise ValueError('measurements require finite observed height and intrinsic state')
        for name in ('slope_rad', 'relief_m'):
            if np.isinf(values[name]).any() or (values[name] < 0).any():
                raise ValueError('physical measurements must be nonnegative or unavailable (NaN)')
        object.__setattr__(self, 'source_id', identity)
        object.__setattr__(self, 'origin_xy_m', origin)
        object.__setattr__(self, 'measurements', np.frombuffer(values.tobytes(), dtype=MEASUREMENT_DTYPE))
        for name in ('observation_fragments',):
            fragments = tuple(getattr(self, name))
            if any(not isinstance(f, EvidenceFragment) or f.measurement_index >= len(values) for f in fragments):
                raise ValueError('fragment references a missing measurement')
            object.__setattr__(self, name, fragments)
    @cached_property
    def _support_bounds_cm(self):
        points=[p for f in self.observation_fragments for p in f.polygon_cm]
        if not points:return None
        xs,ys=zip(*points)
        return min(xs),min(ys),max(xs),max(ys)

    @cached_property
    def _observation_index(self):
        # 私有几何缓存，不记录测量历史；快照发布后不会修改。
        from .p3_map import _polygon
        groups={}
        for f in self.observation_fragments:
            r=self.measurements[f.measurement_index];cell=int(r['x']),int(r['y'])
            groups.setdefault(cell,[]).append(f)
        return {cell:(tuple(fs),len(fs)==1 and fs[0].polygon_cm==tuple(_polygon((*cell,cell[0]+1,cell[1]+1,0),self)))
                for cell,fs in groups.items()}


@dataclass(frozen=True)
class BaseMapFacts:
    world: WorldSnapshot
    raw_revision: int
    sources: tuple[SourceEvidence, ...]

    def __post_init__(self):
        revision = operator.index(self.raw_revision)
        sources = tuple(self.sources)
        if not isinstance(self.world, WorldSnapshot) or revision < 0:
            raise ValueError('facts require a world snapshot and nonnegative revision')
        if any(not isinstance(s, SourceEvidence) for s in sources) or len({s.source_id for s in sources}) != len(sources):
            raise ValueError('facts require unique immutable sources')
        object.__setattr__(self, 'raw_revision', revision)
        object.__setattr__(self, 'sources', sources)
