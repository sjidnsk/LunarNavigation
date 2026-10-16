"""Frozen candidate geometry protocol; no Torch, ROS or privileged inputs.

The native fine graph remains the navigation authority. A2 hubs only carry
connectivity, while retained real nodes keep their pre-contraction path cost.
"""
from dataclasses import dataclass, field
from typing import Mapping
import math
import numpy as np

from .contracts import Pose, freeze_array, _freeze_descriptor
from .feature_schema import IN_GAIN_SLICE, OUT_GAIN_SLICE, HISTORY_SLICE
from .policy_graph import (PolicyGraphResult, _known_graph, _distances,
                           _expand_actions, _contract_graph, _validate_cap)

OBSERVATION_SCHEMA = 'task_graph_v5'
ARCHITECTURE_VERSION = 'candidate_geometry_v3'
HEADINGS = np.arange(8, dtype=np.float64) * (math.pi / 4)


def body_vectors(vectors, yaw):
    """Rotate row-vector XY into the frozen robot frame (no translation)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(vectors, np.float64) @ np.array([[c, -s], [s, c]])


def nearest_region_vectors(points, polygon):
    """Vector to the closed polygon; inside/boundary is zero.

    Equal nearest points use lexicographic XY in the body frame, independent
    of polygon start/winding. Processing one edge at a time bounds memory by N.
    """
    points, polygon = np.asarray(points, np.float64), np.asarray(polygon, np.float64)
    if (points.ndim != 2 or points.shape[1] != 2 or polygon.ndim != 2 or
            polygon.shape[1] != 2 or len(polygon) < 3 or
            not np.isfinite(points).all() or not np.isfinite(polygon).all()):
        raise ValueError('finite points and polygon with at least three vertices required')
    best = np.full(len(points), np.inf)
    nearest = np.full_like(points, np.inf)
    inside, boundary = np.zeros(len(points), bool), np.zeros(len(points), bool)
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        delta = b - a
        square = float(delta @ delta)
        t = np.clip((points-a) @ delta / square, 0., 1.) if square else np.zeros(len(points))
        q = a + t[:, None] * delta
        distance = np.sum((q-points)**2, axis=1)
        tied = np.isclose(distance, best, rtol=1e-12, atol=1e-14)
        lex = (q[:, 0] < nearest[:, 0]) | ((q[:, 0] == nearest[:, 0]) & (q[:, 1] < nearest[:, 1]))
        take = ((distance < best) & ~tied) | (tied & lex)
        best[take], nearest[take] = distance[take], q[take]
        boundary |= distance <= 1e-20
        if delta[1]:
            inside ^= ((a[1] > points[:, 1]) != (b[1] > points[:, 1])) & (
                points[:, 0] < delta[0]*(points[:, 1]-a[1])/delta[1]+a[0])
    vectors = nearest-points
    vectors[inside | boundary] = 0.
    return vectors


@dataclass(frozen=True)
class CandidateObservation:
    node_ids: np.ndarray
    node_geometry: np.ndarray
    node_views: np.ndarray
    node_is_real: np.ndarray
    heading_features: np.ndarray
    edges: np.ndarray
    edge_lengths: np.ndarray
    current_index: int
    polygon: np.ndarray
    context: np.ndarray
    action_pairs: np.ndarray
    goals: np.ndarray
    build_pose: Pose
    epoch: str
    revision: int
    metadata: Mapping = field(default_factory=dict)
    schema: str = OBSERVATION_SCHEMA

    def __post_init__(self):
        if self.schema != OBSERVATION_SCHEMA:
            raise ValueError('candidate observation schema must be task_graph_v5')
        for name, dtype in (('node_ids', np.int64), ('node_geometry', np.float32),
                ('node_views', np.float32), ('node_is_real', np.bool_),
                ('heading_features', np.float32), ('edges', np.int64),
                ('edge_lengths', np.float32), ('polygon', np.float32), ('context', np.float32),
                ('action_pairs', np.int64), ('goals', np.float64)):
            original = np.asarray(getattr(self, name))
            if dtype == np.int64 and (original.dtype.kind not in 'iu' and original.size):
                raise ValueError(f'{name} requires integer indices')
            object.__setattr__(self, name, freeze_array(original, dtype=dtype))
            if not np.isfinite(getattr(self, name)).all():
                raise ValueError(f'{name} must be finite')
        object.__setattr__(self, 'metadata', _freeze_descriptor(self.metadata))
        n, a = len(self.node_ids), len(self.action_pairs)
        if (self.node_ids.shape != (n,) or len(np.unique(self.node_ids)) != n or
                self.node_geometry.shape != (n, 5) or self.node_views.shape != (n, 8, 3) or
                self.node_is_real.shape != (n,)):
            raise ValueError('candidate node fields must align with N x 5 geometry and N x 8 x 3 views')
        if (not isinstance(self.current_index, (int, np.integer)) or
                not 0 <= self.current_index < n or not self.node_is_real[self.current_index]):
            raise ValueError('current index must identify a real node')
        if self.context.shape != (4,) or self.heading_features.shape != (8, 2):
            raise ValueError('four dynamic inputs and eight relative heading labels required')
        if self.action_pairs.shape != (a, 2) or self.goals.shape != (a, 3):
            raise ValueError('each candidate action requires a node-heading pair and world goal')
        if (np.any(self.action_nodes < 0) or np.any(self.action_nodes >= n) or
                np.any(self.action_pairs[:, 1] < 0) or np.any(self.action_pairs[:, 1] >= 8) or
                not self.node_is_real[self.action_nodes].all()):
            raise ValueError('actions must index real nodes and physical headings')
        if len(np.unique(self.action_pairs, axis=0)) != a:
            raise ValueError('candidate actions must be unique')
        if (self.edges.shape != (len(self.edges), 2) or
                self.edge_lengths.shape != (len(self.edges),) or np.any(self.edges < 0) or
                np.any(self.edges >= n) or np.any(self.edge_lengths < 0)):
            raise ValueError('invalid candidate graph edges/costs')
        if (self.polygon.ndim != 2 or self.polygon.shape[1] != 2 or len(self.polygon) < 3 or
                abs(np.sum(self.polygon[:, 0]*np.roll(self.polygon[:, 1], -1) -
                           self.polygon[:, 1]*np.roll(self.polygon[:, 0], -1))) < 1e-12):
            raise ValueError('nondegenerate complete task polygon required')
        if not isinstance(self.build_pose, Pose) or not all(math.isfinite(x) for x in (
                self.build_pose.x, self.build_pose.y, self.build_pose.yaw)):
            raise ValueError('finite frozen build pose required')
        if (np.any(self.node_geometry[~self.node_is_real]) or
                np.any(self.node_views[~self.node_is_real]) or
                np.any(self.node_geometry[:, 4] < 0) or np.any(self.node_views[:, :, :2] < 0) or
                np.any((self.node_views[:, :, 2] != 0) & (self.node_views[:, :, 2] != 1))):
            raise ValueError('virtual placeholders must be zero; real gains/costs nonnegative and history binary')
        if not np.allclose(self.node_geometry[self.current_index, [0, 1, 4]], 0., atol=1e-6):
            raise ValueError('current node must be at body origin with zero fine distance')
        expected = np.column_stack((np.cos(HEADINGS-self.build_pose.yaw), np.sin(HEADINGS-self.build_pose.yaw)))
        if not np.allclose(self.heading_features, expected, atol=2e-6, rtol=0):
            raise ValueError('heading labels must match frozen pose and physical catalog')
        yaw_error = self.goals[:, 2]-HEADINGS[self.action_pairs[:, 1]]
        if np.any(np.abs(np.arctan2(np.sin(yaw_error), np.cos(yaw_error))) > 2e-6):
            raise ValueError('world goals must match candidate heading identity')
        goal_xy = body_vectors(self.goals[:, :2]-[self.build_pose.x, self.build_pose.y], self.build_pose.yaw)/10.
        if not np.allclose(goal_xy, self.node_geometry[self.action_nodes, :2], atol=2e-5, rtol=0):
            raise ValueError('world goals must match candidate position identity')

    @property
    def action_nodes(self):
        return self.action_pairs[:, 0]

    @property
    def action_yaws(self):
        return self.goals[:, 2]


def prepare_candidate_observation(observation, pose, *, contract=True,
                                  max_remote_positions=32, metadata=None):
    """Build v5 only from an authoritative uncontracted measured observation."""
    if observation.schema != 'task_graph_v4':
        raise ValueError('v5 construction requires the original measured fine graph')
    _validate_cap(max_remote_positions)
    adjacency, costs = _known_graph(observation)
    distances = _distances(adjacency, observation.current_index)
    if len(distances) != len(observation.node_ids):
        raise ValueError('all fine graph nodes must be reachable from the current node')
    fine = _expand_actions(observation, max_remote_positions, adjacency, distances=distances)
    if contract:
        result = _contract_graph(fine, adjacency, costs)
        coarse, mapping, virtual, diagnostics = (result.observation, result.fine_to_coarse,
            result.virtual_mask, result.diagnostics)
    else:
        coarse, mapping = fine, np.arange(len(fine.node_ids), dtype=np.int64)
        virtual, diagnostics = np.zeros(len(fine.node_ids), bool), {}
    # Real-node rows are unambiguous; hubs intentionally do not inherit their
    # representative's position, gains or observation history.
    rows = np.flatnonzero(~virtual[mapping])
    retained = mapping[rows]
    xy = body_vectors(fine.positions.astype(np.float64)-[pose.x, pose.y], pose.yaw)/10.
    xy[fine.current_index] = 0.
    polygon = body_vectors(fine.polygon, pose.yaw)
    geometry = np.zeros((len(coarse.node_ids), 5), np.float32)
    geometry[retained, :2] = xy[rows]
    geometry[retained, 2:4] = nearest_region_vectors(xy[rows], polygon)
    geometry[retained, 4] = [distances[int(row)]/10. for row in rows]
    views = np.zeros((len(coarse.node_ids), 8, 3), np.float32)
    views[retained] = np.stack((fine.features[rows, IN_GAIN_SLICE],
        fine.features[rows, OUT_GAIN_SLICE], fine.features[rows, HISTORY_SLICE]), axis=-1)
    heading = np.rint(coarse.goals[:, 2]/(math.pi/4)).astype(np.int64) % 8
    observed = CandidateObservation(coarse.node_ids, geometry, views, ~virtual,
        np.column_stack((np.cos(HEADINGS-pose.yaw), np.sin(HEADINGS-pose.yaw))),
        coarse.edges, coarse.edge_lengths, coarse.current_index, polygon,
        fine.context[2:6], np.column_stack((coarse.action_nodes, heading)), coarse.goals,
        pose, fine.epoch, fine.revision, {} if metadata is None else metadata)
    return PolicyGraphResult(observed, mapping, virtual, diagnostics, fine)
