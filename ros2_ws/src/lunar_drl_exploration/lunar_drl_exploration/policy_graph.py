"""A2 policy adapter over an authoritative measured fine graph.

The output graph is an Actor message-passing graph. Its virtual hubs are never
navigation goals, and its hub edge costs are representative-to-interface
costs rather than exact distances for every pair of retained boundary nodes.
The original fine graph and all native navigation code remain authoritative.
"""

from dataclasses import dataclass, replace
import heapq
import math
from numbers import Integral

import numpy as np

from lunar_drl_exploration.contracts import DecisionObservation, freeze_array
from lunar_drl_exploration.feature_schema import (
    HISTORY_SLICE, IN_GAIN_SLICE, OUT_GAIN_SLICE)


# Keep this pure graph adapter independent of history's native sensor import.
# These are the same eight optical-world action headings used by GraphBuilder.
_HEADINGS = np.arange(8, dtype=np.float64) * (math.pi / 4)


@dataclass(frozen=True)
class ContractionResult:
    observation: DecisionObservation
    fine_to_coarse: np.ndarray
    virtual_mask: np.ndarray
    diagnostics: dict

    def __post_init__(self):
        object.__setattr__(self, "fine_to_coarse",
                           freeze_array(self.fine_to_coarse, dtype=np.int64))
        object.__setattr__(self, "virtual_mask",
                           freeze_array(self.virtual_mask, dtype=np.bool_))


def _known_graph(observation):
    """Canonical undirected known edges, with the minimum cost for duplicates."""
    count = len(observation.node_ids)
    edges, lengths = observation.edges, observation.edge_lengths
    if edges.shape != (len(edges), 2) or lengths.shape != (len(edges),):
        raise ValueError("known graph edges and edge lengths must align")
    if np.any(edges < 0) or np.any(edges >= count):
        raise ValueError("known graph edge endpoint is outside graph nodes")
    if not np.isfinite(lengths).all() or np.any(lengths < 0):
        raise ValueError("known graph edge costs must be finite and nonnegative")
    if observation.positions.shape != (count, 2) or not np.isfinite(observation.positions).all():
        raise ValueError("known graph positions must be finite N x 2 coordinates")
    costs = {}
    for (a, b), length in zip(edges, lengths):
        a, b = sorted((int(a), int(b)))
        if a == b:
            continue
        key = (a, b)
        costs[key] = min(costs.get(key, math.inf), float(length))
    adjacency = [[] for _ in range(count)]
    for (a, b), length in sorted(costs.items()):
        adjacency[a].append((b, length))
        adjacency[b].append((a, length))
    return adjacency, costs


def _distances(adjacency, source, allowed=None):
    """Dijkstra over the original known graph, optionally within one component."""
    distances = {int(source): 0.0}
    pending = [(0.0, int(source))]
    while pending:
        distance, row = heapq.heappop(pending)
        if distance != distances[row]:
            continue
        for neighbor, cost in adjacency[row]:
            if allowed is not None and neighbor not in allowed:
                continue
            candidate = distance + cost
            if candidate < distances.get(neighbor, math.inf):
                distances[neighbor] = candidate
                heapq.heappush(pending, (candidate, neighbor))
    return distances


def _groups(observation, adjacency):
    features = observation.features
    eligible = (np.all(features[:, IN_GAIN_SLICE] == 0, axis=1)
                & np.all(features[:, OUT_GAIN_SLICE] == 0, axis=1))
    eligible[observation.current_index] = False
    eligible[observation.action_nodes] = False
    visited = ~eligible
    groups = [[int(row)] for row in np.flatnonzero(~eligible)]
    for start in np.flatnonzero(eligible):
        if visited[start]:
            continue
        members, pending = [], [int(start)]
        visited[start] = True
        while pending:
            row = pending.pop()
            members.append(row)
            for neighbor, _ in adjacency[row]:
                if not visited[neighbor] and features[neighbor, 2] == features[row, 2]:
                    visited[neighbor] = True
                    pending.append(neighbor)
        groups.append(sorted(members))
    return sorted(groups, key=lambda members: members[0])


def contract_observation(observation: DecisionObservation) -> ContractionResult:
    """Replace zero-gain, same-domain connected components by virtual hubs.

    Current, action and any nonzero gain nodes are retained. Singletons retain
    their original identity and features. ``fine_to_coarse`` addresses graph
    rows, so original action indices can be translated without changing their
    order, world goals or headings. Diagnostics contain only JSON primitives.
    """
    adjacency, known_costs = _known_graph(observation)
    return _contract_graph(observation, adjacency, known_costs)


def _contract_graph(observation, adjacency, known_costs):
    groups = _groups(observation, adjacency)
    count = len(observation.node_ids)
    mapping = np.empty(count, dtype=np.int64)
    virtual = np.asarray([len(members) > 1 for members in groups])
    representatives, regions = [], []
    ids, internal_costs = [], np.zeros(count, np.float64)
    used_ids = set(map(int, observation.node_ids))
    next_virtual_id = np.iinfo(np.int64).max
    for coarse_row, members in enumerate(groups):
        mapping[members] = coarse_row
        if len(members) == 1:
            representative = members[0]
            node_id = int(observation.node_ids[representative])
        else:
            points = observation.positions[members].astype(np.float64)
            center = points.mean(axis=0)
            distance2 = np.sum((points - center) ** 2, axis=1)
            representative = min(zip(members, distance2), key=lambda item:
                                 (float(item[1]), int(observation.node_ids[item[0]]), item[0]))[0]
            while next_virtual_id in used_ids:
                next_virtual_id -= 1
            node_id = int(next_virtual_id)
            used_ids.add(node_id)
            next_virtual_id -= 1
            distances = _distances(adjacency, representative, set(members))
            for row in members:
                internal_costs[row] = distances[row]
            regions.append({
                "coarse_index": int(coarse_row), "node_id": node_id,
                "member_indices": list(map(int, members)),
                "member_node_ids": [int(observation.node_ids[row]) for row in members],
                "representative_index": int(representative),
                "representative_node_id": int(observation.node_ids[representative]),
                "inside": bool(observation.features[representative, 2] > .5),
                "size": len(members),
            })
        representatives.append(representative)
        ids.append(node_id)

    if not virtual.any():
        canonical_edges = np.asarray(sorted(known_costs), np.int64).reshape(-1, 2)
        canonical_lengths = np.asarray([known_costs[tuple(edge)] for edge in canonical_edges],
                                       np.float32)
        coarse = (observation if np.array_equal(canonical_edges, observation.edges)
                  and np.array_equal(canonical_lengths, observation.edge_lengths)
                  else replace(observation, edges=canonical_edges, edge_lengths=canonical_lengths))
    else:
        features = observation.features[representatives].copy()
        features[virtual, IN_GAIN_SLICE] = 0
        features[virtual, OUT_GAIN_SLICE] = 0
        features[virtual, HISTORY_SLICE] = 0
        coarse_costs = {}
        for (a, b), cost in known_costs.items():
            ca, cb = sorted((int(mapping[a]), int(mapping[b])))
            if ca == cb:
                continue
            total = internal_costs[a] + cost + internal_costs[b]
            key = (ca, cb)
            coarse_costs[key] = min(coarse_costs.get(key, math.inf), float(total))
        coarse_edges = sorted(coarse_costs)
        lengths = np.asarray([coarse_costs[edge] for edge in coarse_edges], np.float64)
        if np.any(lengths > np.finfo(np.float32).max):
            raise ValueError("contracted edge costs must remain finite in float32")
        coarse = replace(
            observation, node_ids=np.asarray(ids, np.int64),
            positions=observation.positions[representatives], features=features,
            edges=np.asarray(coarse_edges, np.int64).reshape(-1, 2),
            edge_lengths=lengths, current_index=int(mapping[observation.current_index]),
            action_nodes=mapping[observation.action_nodes])
    if np.any(virtual[coarse.action_nodes]):
        raise ValueError("virtual region nodes cannot be action goals")
    diagnostics = {
        "original_nodes": count, "coarse_nodes": len(coarse.node_ids),
        "original_edges": len(observation.edges), "coarse_edges": len(coarse.edges),
        "node_compression_ratio": len(coarse.node_ids) / count,
        "edge_compression_ratio": (len(coarse.edges) / len(observation.edges)
                                   if len(observation.edges) else 1.0),
        "virtual_nodes": int(virtual.sum()), "regions": regions,
    }
    return ContractionResult(coarse, mapping, virtual, diagnostics)


def expand_frontier_actions(observation: DecisionObservation,
                            max_positions: int = 32) -> DecisionObservation:
    """Append headings at reachable real stances observing task-inside frontiers.

    Run this on the original fine graph, before contraction. Existing actions
    remain an exact prefix. The current stance is excluded to retain the
    original current-turn validity filtering. Only positions with newly added
    actions count toward the cap; fully represented positions consume no cap.
    """
    _validate_cap(max_positions)
    adjacency, _ = _known_graph(observation)
    return _expand_actions(observation, max_positions, adjacency)


def _validate_cap(max_positions):
    if (not isinstance(max_positions, Integral) or isinstance(max_positions, bool)
            or max_positions < 0):
        raise ValueError("max_positions must be a nonnegative integer")


def _expand_actions(observation, max_positions, adjacency):
    if max_positions == 0:
        return observation
    distances = _distances(adjacency, observation.current_index)
    positive_inside = np.any(observation.features[:, IN_GAIN_SLICE] > 0, axis=1)
    candidates = sorted(
        (row for row in distances if row != observation.current_index and positive_inside[row]),
        key=lambda row: (distances[row], int(observation.node_ids[row]), row))
    existing = set(zip(map(int, observation.action_nodes), map(float, observation.action_yaws)))
    new_nodes, new_yaws, new_goals = [], [], []
    positions_added = 0
    for row in candidates:
        missing = [float(yaw) for yaw in _HEADINGS
                   if (row, float(np.float32(yaw))) not in existing]
        if not missing:
            continue
        for yaw in missing:
            new_nodes.append(row)
            new_yaws.append(yaw)
            new_goals.append([float(observation.positions[row, 0]),
                              float(observation.positions[row, 1]), yaw])
        positions_added += 1
        if positions_added == max_positions:
            break
    if not new_nodes:
        return observation
    return replace(
        observation, action_nodes=np.r_[observation.action_nodes, np.asarray(new_nodes, np.int64)],
        action_yaws=np.r_[observation.action_yaws, np.asarray(new_yaws, np.float32)],
        goals=np.vstack((observation.goals, np.asarray(new_goals, np.float64))))


@dataclass(frozen=True)
class PolicyGraphResult(ContractionResult):
    """Policy graph plus the expanded real-node graph used by the Critic."""
    fine_observation: DecisionObservation


def prepare_policy_graph(observation: DecisionObservation,
                         max_remote_positions: int = 32) -> PolicyGraphResult:
    """Expand real frontier actions and contract using one canonical graph.

    Canonical adjacency and minimum duplicate costs are local to this decision.
    Expansion preserves every fine graph array, so both stages can share them.
    The action order and world goals also match ``fine_observation`` exactly;
    virtual policy hubs never become executable goals or Critic query stances.
    """
    _validate_cap(max_remote_positions)
    adjacency, costs = _known_graph(observation)
    fine = _expand_actions(observation, max_remote_positions, adjacency)
    contracted = _contract_graph(fine, adjacency, costs)
    return PolicyGraphResult(contracted.observation, contracted.fine_to_coarse,
                             contracted.virtual_mask, contracted.diagnostics, fine)
