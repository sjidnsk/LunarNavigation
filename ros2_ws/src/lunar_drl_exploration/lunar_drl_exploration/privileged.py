"""Training-owned truth skeleton and frozen candidate descriptors.

This module is never needed by the online decision path. Terrain and temporary
ray results stay with the environment; replay owns only raw removal counts and sparse
read-only query supports. Truth does not add or remove measured actions.
"""
from typing import TYPE_CHECKING
import numpy as np
from scipy.spatial import cKDTree
from .config import GraphConfig
from .contracts import PrivilegedScene, PrivilegedState, PrivilegedActionContext, Pose, SensorSpec
from .geometry import lattice_displacement, pack_cell_ids
from .frontiers import FrontierSnapshot
from .maps import FREE, UNKNOWN
from .metric_graph import MetricGrid, greedy_spanner
from .sensor import visible_cells

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


def predict_removed(before: FrontierSnapshot, terrain: 'TerrainGrid', pose: Pose,
                    sensor: SensorSpec) -> int:
    """Exact old inside-front elimination after one endpoint truth observation.

    This projects only the frozen inside demands and current FREE centers,
    without constructing/copying whole-map virtual snapshots. It intentionally
    differs from the real action's accumulated route reward.
    """
    return _RemovalPrediction(before, terrain).predict(terrain, pose, sensor)


class PrivilegedBuilder:
    """One environment's truth geometry; state-dependent descriptors never cache K."""
    def __init__(self, terrain, scene, task, sensor, config=None):
        self.terrain, self.scene, self.sensor = terrain, scene, sensor
        self.config = config or GraphConfig()
        self.grid = MetricGrid(terrain.navigation == 1, terrain.resolution_m)
        self.cells = np.floor((scene.positions.astype(float) - terrain.origin[:2]) /
                              terrain.resolution_m).astype(np.int64)

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
            # Formal BuildStartConnections returns the containing grid cell in
            # the evidence-FREE case. This is also the plant's truth precondition.
            entry = float(np.linalg.norm((cell + .5) * resolution + origin - xy))
            distance = self.grid.distances(cell.reshape(1, 2), np.array([entry]),
                self.config.coverage_radius_m + entry, self.cells)
            selected = np.flatnonzero(np.isfinite(distance))
            if not len(selected):
                raise ValueError('truth cover provides no legal candidate query support')
            supports.extend(selected.tolist())
            support_distances.extend(distance[selected].tolist())
            offsets.append(len(supports))
        observed = np.asarray(observed, dtype=np.uint8)
        prediction = _RemovalPrediction(frontiers, terrain) if len(observation.goals) else None
        removed = np.asarray([prediction.predict(terrain, Pose(*goal), self.sensor)
                              for goal in observation.goals], dtype=np.int64)
        actions = PrivilegedActionContext(positions, inverse, observation.goals[:, 2], np.asarray(offsets),
            np.asarray(supports, np.int64), np.asarray(support_distances, np.float32), removed)
        return PrivilegedState(self.scene.scene_id, observed, actions)
