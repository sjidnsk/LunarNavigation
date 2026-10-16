"""Training-owned truth skeleton and frozen candidate descriptors.

This module is never needed by the online decision path. Terrain and temporary
ray results stay with the environment; replay owns only raw removal counts and sparse
read-only query supports. Truth does not add or remove measured actions.
"""
from collections import OrderedDict
import math
from numbers import Integral
from typing import TYPE_CHECKING
import numpy as np
from scipy.spatial import cKDTree
from .config import GraphConfig
from .contracts import PrivilegedScene, PrivilegedState, PrivilegedActionContext, Pose, SensorSpec
from .geometry import lattice_displacement, pack_cell_ids
from .frontiers import FrontierSnapshot
from .maps import FREE, UNKNOWN
from .metric_graph import MetricGrid, greedy_spanner
from .sensor import visible_cells, directional_visibility, validate_sensor

if TYPE_CHECKING:
    from .scene import TerrainGrid


def build_truth(terrain, reference, config=None):
    """Build the scene-owned static metric skeleton once per environment reset."""
    config = config or GraphConfig()
    reachable = reference.unpack(reference.reachable_bits)
    grid = MetricGrid(reachable, terrain.resolution_m)
    cells = grid.cover(radius=config.coverage_radius_m)
    points = (cells + 0.5) * terrain.resolution_m + np.asarray(terrain.origin[:2])
    candidates, distances = grid.connections(cells, limit=config.connection_limit_m)
    edges, lengths = greedy_spanner(len(cells), candidates, distances, config.stretch)
    indices = np.flatnonzero(reference.unpack(reference.packed_mask).ravel())
    if len(points):
        owners = np.empty(len(indices), np.int64)
        tree = cKDTree(points)
        for first in range(0, len(indices), 65536):
            chunk = indices[first:first + 65536]
            xy = np.column_stack((chunk % reference.shape[1], chunk // reference.shape[1]))
            xy = (xy + 0.5) * terrain.resolution_m + np.asarray(terrain.origin[:2])
            owners[first:first + len(chunk)] = tree.query(xy)[1]
        order = np.argsort(owners, kind='stable')
        indices = indices[order]
        offsets = np.r_[0, np.cumsum(np.bincount(owners, minlength=len(points)))]
    elif len(indices):
        raise ValueError('reference cells require reachable truth skeleton')
    else:
        offsets = np.array([0], np.int64)
    descriptor = getattr(terrain, 'generator_descriptor', None)
    if descriptor is None:
        descriptor = dict(resolution_m=terrain.resolution_m, origin=tuple(terrain.origin),
                          shape=tuple(terrain.shape))
    descriptor = dict(descriptor, terrain_id=terrain.terrain_id,
                      graph_schema='metric_cover_spanner_v1')
    return PrivilegedScene(reference.reference_id, points, edges, lengths, offsets,
        indices, reference.packed_mask, tuple(reference.shape), descriptor)


def _sorted_membership(values, sorted_ids):
    """Probe canonical snapshot IDs without scanning/copying the known map."""
    indices = np.searchsorted(sorted_ids, values)
    result = np.zeros(len(values), bool)
    valid = indices < len(sorted_ids)
    result[valid] = sorted_ids[indices[valid]] == values[valid]
    return result


class _RemovalPrediction:
    """Per-build projection of frozen inside demands; never retained across K."""
    def __init__(self, before, terrain):
        if terrain.resolution_m != before.resolution_m:
            raise ValueError('native/reference resolution mismatch')
        self.offset = lattice_displacement(terrain.origin, before.lattice_origin,
                                           before.resolution_m)
        if self.offset is None:
            raise ValueError('native/reference lattice mismatch')
        self.old = before.inside
        self.known_demands = _sorted_membership(self.old.demand_ids, before.known_ids)
        self.free_centers = _sorted_membership(self.old.cell_ids, before.free_ids)

    def predict(self, terrain, pose, sensor):
        # Keep the native reveal and its sensor/pose validation even for no fronts.
        rows, cols = visible_cells(terrain.intrinsic, terrain.origin,
                                   terrain.resolution_m, pose, sensor)
        old = self.old
        if not len(old.cell_ids):
            return 0
        classes = terrain.intrinsic[rows, cols]
        valid = np.isfinite(terrain.heights[rows, cols]) & (classes != UNKNOWN)
        hits = pack_cell_ids(cols[valid]+self.offset[0], rows[valid]+self.offset[1])
        solved = np.logical_and.reduceat(
            self.known_demands | np.isin(old.demand_ids, hits), old.demand_offsets[:-1])
        # A revealed BLOCKED center cannot earn removal. Unrevealed centers
        # retain measured FREE membership, exactly as the virtual snapshot did.
        free = ((self.free_centers & ~np.isin(old.cell_ids, hits)) |
                np.isin(old.cell_ids, hits[classes[valid] == FREE]))
        # All demands known implies the old center is absent from the projected
        # frontier set. New fronts and outside demands cannot affect this count.
        return int(np.count_nonzero(solved & free))

    def predict_many(self, terrain, goals, sensor):
        """Same endpoint projection, querying only the frozen relevant cells.

        Full-map revelations do not affect removal of these old fronts. Share
        the exact native directional query across headings at one sensor cell;
        retain current known demands and the FREE-center gate per action.
        """
        validate_sensor(sensor)
        goals = np.asarray(goals, float).reshape(-1, 3)
        if not np.isfinite(goals).all():raise ValueError('invalid prediction pose')
        cells = np.floor((goals[:, :2]-terrain.origin[:2])/terrain.resolution_m).astype(np.int64)
        h,w = terrain.shape
        if np.any((cells[:,0]<0)|(cells[:,0]>=w)|(cells[:,1]<0)|(cells[:,1]>=h)):
            raise ValueError('sensor outside provided grid')
        result = np.zeros(len(goals), np.int64)
        old = self.old
        if not len(old.cell_ids) or not len(goals):return result
        ids, restore = np.unique(np.r_[old.cell_ids,old.demand_ids], return_inverse=True)
        targets = np.column_stack((ids>>32,(ids<<32)>>32))-self.offset
        valid = ((targets[:,0]>=0)&(targets[:,0]<w)&(targets[:,1]>=0)&(targets[:,1]<h))
        classes = np.zeros(len(ids),np.uint8)
        good = np.flatnonzero(valid)
        x,y = targets[good].T
        classes[good] = terrain.intrinsic[y,x]
        valid[good] &= np.isfinite(terrain.heights[y,x]) & (classes[good]!=UNKNOWN)
        count = len(old.cell_ids)
        center_free = (classes[restore[:count]]==FREE)[:,None]
        positions, inverse = np.unique(cells,axis=0,return_inverse=True)
        for position,source in enumerate(positions):
            actions = np.flatnonzero(inverse==position)
            # At 360 degrees every heading has the same reveal. Keep the
            # original action order/yaws in the returned context nonetheless.
            yaws, yaw_inverse = np.unique(np.zeros(len(actions)) if sensor.fov_deg==360.
                else goals[actions,2], return_inverse=True)
            for start in range(0,len(yaws),8):
                headings = yaws[start:start+8]
                padded = np.pad(headings,(0,8-len(headings)),mode='edge')
                seen = np.zeros((len(ids),8),bool)
                seen[valid] = directional_visibility(terrain.intrinsic,source,targets[valid],
                    sensor.range_m/terrain.resolution_m,padded+sensor.offset_yaw_rad,
                    math.radians(sensor.fov_deg))
                hit = seen[restore]
                solved = np.logical_and.reduceat(self.known_demands[:,None]|hit[count:],
                                                 old.demand_offsets[:-1],axis=0)
                free = (self.free_centers[:,None]&~hit[:count]) | (hit[:count]&center_free)
                removed = np.count_nonzero(solved&free,axis=0)
                selected = (yaw_inverse>=start)&(yaw_inverse<start+len(headings))
                result[actions[selected]] = removed[yaw_inverse[selected]-start]
        return result


def predict_removed(before: FrontierSnapshot, terrain: 'TerrainGrid', pose: Pose,
                    sensor: SensorSpec) -> int:
    """Exact old inside-front elimination after one endpoint truth observation.

    This projects only the frozen inside demands and current FREE centers,
    without constructing/copying whole-map virtual snapshots. It intentionally
    differs from the real action's accumulated route reward.
    """
    return _RemovalPrediction(before, terrain).predict(terrain, pose, sensor)


class PrivilegedBuilder:
    """One environment's fixed truth geometry; never cache state-dependent gains.

    The exact-XY support cache belongs to this builder's terrain/scene/config,
    and is discarded with it at reset. Its values contain only static query
    geometry; observed bits and frozen frontier demands are rebuilt each call.
    """
    def __init__(self, terrain, scene, task, sensor, config=None, *, support_cache_size=4096):
        if (isinstance(support_cache_size, bool) or not isinstance(support_cache_size, Integral)
                or support_cache_size < 0):
            raise ValueError('support cache size must be a nonnegative integer')
        self.terrain, self.scene, self.sensor = terrain, scene, sensor
        self.config = config or GraphConfig()
        self.grid = MetricGrid(terrain.navigation == 1, terrain.resolution_m)
        self.cells = np.floor((scene.positions.astype(float) - terrain.origin[:2]) /
                              terrain.resolution_m).astype(np.int64)
        self._support_cache_size = int(support_cache_size)
        self._support_cache = OrderedDict()

    def build(self, observation, observed, *, frontiers: FrontierSnapshot):
        """Freeze current actions against the measured pre-action frontier snapshot."""
        positions, inverse = np.unique(observation.goals[:, :2], axis=0, return_inverse=True)
        offsets = [0]
        supports, support_distances = [], []
        terrain = self.terrain
        resolution = terrain.resolution_m
        origin = np.asarray(terrain.origin[:2])
        for xy in positions:
            cell = np.floor((xy - origin) / resolution).astype(np.int64)
            x, y = map(int, cell)
            if not (0 <= y < terrain.shape[0] and 0 <= x < terrain.shape[1]
                    and terrain.navigation[y, x] == 1):
                raise ValueError('candidate query lacks native truth FREE start connection')
            # Continuous entry costs depend on exact XY, even within one cell.
            key = tuple(map(float, xy))
            cached = self._support_cache.get(key)
            if cached is None:
                # Formal BuildStartConnections returns the containing grid cell
                # in the evidence-FREE case, also the plant's truth precondition.
                entry = float(np.linalg.norm((cell + .5) * resolution + origin - xy))
                distance = self.grid.distances(cell.reshape(1, 2), np.array([entry]),
                    self.config.coverage_radius_m + entry, self.cells)
                selected = np.flatnonzero(np.isfinite(distance))
                if not len(selected):
                    raise ValueError('truth cover provides no legal candidate query support')
                # Immutable tuples cannot alias or be changed by output arrays.
                cached = (tuple(selected.tolist()), tuple(distance[selected].tolist()))
                if self._support_cache_size:
                    self._support_cache[key] = cached
                    if len(self._support_cache) > self._support_cache_size:
                        self._support_cache.popitem(last=False)
            else:
                self._support_cache.move_to_end(key)
            supports.extend(cached[0])
            support_distances.extend(cached[1])
            offsets.append(len(supports))
        observed = np.asarray(observed, dtype=np.uint8)
        prediction = _RemovalPrediction(frontiers, terrain) if len(observation.goals) else None
        removed = (prediction.predict_many(terrain, observation.goals, self.sensor)
                   if prediction is not None else np.empty(0, np.int64))
        actions = PrivilegedActionContext(positions, inverse, observation.goals[:, 2], np.asarray(offsets),
            np.asarray(supports, np.int64), np.asarray(support_distances, np.float32), removed)
        return PrivilegedState(self.scene.scene_id, observed, actions)
