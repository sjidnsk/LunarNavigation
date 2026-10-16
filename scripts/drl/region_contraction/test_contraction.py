"""Behavioral tests for the experiment-only observation transformation."""

from dataclasses import replace
import importlib
import json
import math

import numpy as np
import pytest

from lunar_drl_exploration.contracts import DecisionObservation

# Optical headings are a fixed public convention. Importing history would also
# load the native sensor extension, which this graph-only unit suite does not use.
HEADINGS = np.asarray([i * (math.pi / 4) for i in range(8)])


def implementation():
    try:
        return importlib.import_module("contraction")
    except ImportError:
        pytest.fail("the region-contraction implementation is missing")


def observation(positions, edges=(), *, lengths=None, inside=None, gains=(),
                outside_gains=(), actions=(), yaws=None, current=0, ids=None):
    positions = np.asarray(positions, np.float32)
    features = np.zeros((len(positions), 27), np.float32)
    features[:, :2] = (positions - positions[current]) / 10
    features[:, 2] = 1 if inside is None else inside
    features[:, 19:27] = .75  # Virtual hubs must not inherit observed headings.
    for index in gains:
        features[index, 3] = .25
    for index in outside_gains:
        features[index, 18] = .125
    actions = np.asarray(actions, np.int64)
    yaws = np.zeros(len(actions)) if yaws is None else np.asarray(yaws)
    goals = np.column_stack((positions[actions], yaws))
    edges = np.asarray(edges, np.int64).reshape(-1, 2)
    return DecisionObservation(
        node_ids=np.arange(10, 10 + len(positions)) if ids is None else ids,
        positions=positions, features=features, edges=edges,
        edge_lengths=np.ones(len(edges)) if lengths is None else lengths,
        current_index=current, polygon=np.array([[0, 0], [5, 0], [0, 5]]),
        context=np.arange(8), action_nodes=actions, action_yaws=yaws,
        goals=goals, epoch="frozen-epoch", revision=7)


def edge_cost(result, fine_a, fine_b):
    a, b = sorted((int(result.fine_to_coarse[fine_a]),
                   int(result.fine_to_coarse[fine_b])))
    edge = np.flatnonzero(np.all(result.observation.edges == [a, b], axis=1))
    assert len(edge) == 1
    return result.observation.edge_lengths[edge[0]]


def test_only_known_edges_form_components_even_with_coincident_positions():
    # Spatial clustering would fabricate a connection across a wall.
    obs = observation([(10, 10), (0, 0), (1, 0), (0, 0), (1, 0), (2, 0)],
                      [(1, 2), (3, 4)])
    result = implementation().contract_observation(obs)
    assert result.fine_to_coarse[1] == result.fine_to_coarse[2]
    assert result.fine_to_coarse[3] == result.fine_to_coarse[4]
    assert result.fine_to_coarse[1] != result.fine_to_coarse[3]
    assert len(result.observation.node_ids) == 4
    assert not result.virtual_mask[result.fine_to_coarse[5]]
    assert len(result.observation.edges) == 0


def test_task_inside_and_outside_zero_gain_components_stay_separate():
    # Ignoring membership would conflate explored areas across the task border.
    obs = observation([(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)],
                      [(0, 1), (1, 2), (2, 3), (3, 4)],
                      inside=[1, 1, 1, 0, 0])
    result = implementation().contract_observation(obs)
    assert result.fine_to_coarse[1] == result.fine_to_coarse[2]
    assert result.fine_to_coarse[3] == result.fine_to_coarse[4]
    assert result.fine_to_coarse[2] != result.fine_to_coarse[3]
    assert result.observation.features[result.fine_to_coarse[1], 2] == 1
    assert result.observation.features[result.fine_to_coarse[3], 2] == 0
    assert edge_cost(result, 2, 3) == 2  # Inside representative-to-interface.


def test_positive_gains_current_and_original_action_nodes_survive_bytewise():
    # Dropping out-gain nodes or zero-gain targets changes the frozen actor task.
    obs = observation([(i, 0) for i in range(8)],
                      [(i, i + 1) for i in range(7)], gains=[4], outside_gains=[5],
                      actions=[6, 0, 6], yaws=[1.3, .7, 2.2])
    result = implementation().contract_observation(obs)
    coarse = result.observation
    for fine in [0, 4, 5, 6, 7]:
        row = result.fine_to_coarse[fine]
        assert coarse.node_ids[row] == obs.node_ids[fine]
        assert coarse.features[row].tobytes() == obs.features[fine].tobytes()
        assert coarse.positions[row].tobytes() == obs.positions[fine].tobytes()
        assert not result.virtual_mask[row]
    assert coarse.current_index == result.fine_to_coarse[0]
    assert coarse.action_nodes.tolist() == result.fine_to_coarse[obs.action_nodes].tolist()
    assert not np.any(result.virtual_mask[coarse.action_nodes])
    for field in ["action_yaws", "goals", "polygon", "context"]:
        assert getattr(coarse, field).tobytes() == getattr(obs, field).tobytes()
    assert (coarse.epoch, coarse.revision, coarse.schema) == (obs.epoch, obs.revision, obs.schema)


def test_representative_is_actual_centroid_nearest_member_with_node_id_tie_break():
    # A made-up centroid is not a measured graph stance; index ties are unstable.
    obs = observation([(9, 9), (0, 0), (2, 0)], [(1, 2)], ids=[-1, 100, 20])
    result = implementation().contract_observation(obs)
    hub = result.fine_to_coarse[1]
    assert result.observation.positions[hub].tolist() == [2, 0]
    assert result.observation.features[hub, :3].tobytes() == obs.features[2, :3].tobytes()
    assert np.all(result.observation.features[hub, 3:27] == 0)
    assert result.observation.node_ids[hub] not in obs.node_ids
    assert len(np.unique(result.observation.node_ids)) == len(result.observation.node_ids)


def test_long_chain_edges_include_internal_known_graph_distances():
    # Mapping only endpoint edge costs would turn a long explored chain into a shortcut.
    obs = observation([(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)],
                      [(0, 1), (1, 2), (2, 3), (3, 4)], lengths=[2, 10, 20, 3],
                      gains=[4])
    result = implementation().contract_observation(obs)
    assert result.observation.positions[result.fine_to_coarse[1]].tolist() == [2, 0]
    assert edge_cost(result, 0, 1) == 12
    assert edge_cost(result, 3, 4) == 23


def test_internal_dijkstra_uses_weighted_shortest_path_and_minimum_duplicate_edge():
    # Hop count, Euclidean distance or last duplicate cost would lose measured route costs.
    obs = observation([(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)],
                      [(0, 1), (0, 3), (3, 0), (1, 2), (2, 3), (1, 3), (3, 4)],
                      lengths=[8, 2, 10, 10, 1, 1, 5], gains=[4])
    result = implementation().contract_observation(obs)
    assert edge_cost(result, 0, 1) == 3  # 0->3 (2), 3->representative 2 (1).
    assert edge_cost(result, 3, 4) == 6
    assert not np.any(result.observation.edges[:, 0] == result.observation.edges[:, 1])


def test_zero_cost_known_edges_are_eligible_and_connected():
    # Filtering out zero lengths wrongly disconnects a valid known graph.
    obs = observation([(0, 0), (1, 0), (2, 0)], [(0, 1), (1, 2)], lengths=[0, 0])
    result = implementation().contract_observation(obs)
    assert result.fine_to_coarse[1] == result.fine_to_coarse[2]
    assert edge_cost(result, 0, 1) == 0


def test_deterministic_output_diagnostics_and_input_immutability():
    # Order-dependent traversal or mutable staging arrays make snapshots irreproducible.
    obs = observation([(i, 0) for i in range(6)], [(0, 1), (1, 2), (3, 4)], gains=[5])
    original = {name: getattr(obs, name).tobytes() for name in
                ["node_ids", "positions", "features", "edges", "edge_lengths", "goals"]}
    result = implementation().contract_observation(obs)
    reversed_edges = replace(obs, edges=obs.edges[::-1, ::-1], edge_lengths=obs.edge_lengths[::-1])
    other = implementation().contract_observation(reversed_edges)
    for field in ["node_ids", "positions", "features", "edges", "edge_lengths", "action_nodes"]:
        assert getattr(result.observation, field).tobytes() == getattr(other.observation, field).tobytes()
    assert result.fine_to_coarse.tolist() == other.fine_to_coarse.tolist()
    assert result.diagnostics == other.diagnostics
    assert result.diagnostics["original_nodes"] == 6
    assert result.diagnostics["coarse_nodes"] == 4
    assert result.diagnostics["original_edges"] == 3
    assert result.diagnostics["coarse_edges"] == 1
    assert len(result.diagnostics["regions"]) == 2
    json.dumps(result.diagnostics, allow_nan=False)
    assert not result.virtual_mask.flags.writeable
    assert not result.fine_to_coarse.flags.writeable
    for name, value in original.items():
        assert getattr(obs, name).tobytes() == value


def test_no_eligible_multi_node_component_preserves_graph_content():
    # An unnecessary transform must not perturb a fully retained graph's costs.
    obs = observation([(0, 0), (1, 0), (2, 0)], [(0, 1), (1, 2)], gains=[1, 2], actions=[1])
    result = implementation().contract_observation(obs)
    assert result.fine_to_coarse.tolist() == [0, 1, 2]
    assert not result.virtual_mask.any()
    for name in ["node_ids", "features", "positions", "edges", "edge_lengths", "action_nodes", "goals"]:
        assert getattr(result.observation, name).tobytes() == getattr(obs, name).tobytes()


def test_duplicate_and_self_edges_are_removed_even_without_a_region():
    # The graph identity fallback cannot retain duplicate abstractions or self loops.
    obs = observation([(0, 0), (1, 0)], [(1, 0), (0, 1), (1, 1)],
                      lengths=[4, 2, 1], gains=[1])
    result = implementation().contract_observation(obs)
    assert result.observation.edges.tolist() == [[0, 1]]
    assert result.observation.edge_lengths.tolist() == [2]
    assert result.diagnostics["original_edges"] == 3
    assert result.diagnostics["coarse_edges"] == 1


@pytest.mark.parametrize("bad_cost", [-1, np.nan, np.inf])
def test_invalid_graph_costs_fail_before_any_fabricated_connectivity(bad_cost):
    # Clamping invalid costs into an apparently valid abstract graph is unsafe.
    obs = observation([(0, 0), (1, 0)], [(0, 1)], lengths=[bad_cost])
    with pytest.raises(ValueError, match="nonnegative|finite"):
        implementation().contract_observation(obs)
    with pytest.raises(ValueError, match="nonnegative|finite"):
        implementation().expand_frontier_actions(obs)


def test_remote_actions_use_reachable_inside_gain_and_weighted_distance_then_node_id():
    # Euclidean rank, pure task-outside gain or disconnected stances change A2's domain.
    obs = observation([(0, 0), (1, 0), (30, 0), (2, 0), (4, 0), (50, 0)],
                      [(0, 1), (0, 2), (0, 3), (0, 4)], lengths=[9, 2, 2, 1],
                      inside=[1, 1, 1, 1, 0, 1], gains=[1, 2, 3, 5], outside_gains=[4],
                      actions=[0], yaws=[.5], ids=[0, 50, 20, 10, 40, 30])
    expanded = implementation().expand_frontier_actions(obs, max_positions=2)
    assert expanded.action_nodes.tolist() == [0] + [3] * 8 + [2] * 8
    assert expanded.goals[1:9, :2].tolist() == [[2, 0]] * 8
    assert expanded.goals[9:, :2].tolist() == [[30, 0]] * 8
    assert expanded.action_yaws[1:9].tolist() == HEADINGS.astype(np.float32).tolist()
    assert expanded.goals[1:9, 2].tolist() == HEADINGS.tolist()
    for name in ["node_ids", "positions", "features", "edges", "edge_lengths", "polygon", "context"]:
        assert getattr(expanded, name).tobytes() == getattr(obs, name).tobytes()
    assert expanded.current_index == obs.current_index
    assert expanded.goals[:1].tobytes() == obs.goals.tobytes()
    assert expanded.action_yaws[:1].tobytes() == obs.action_yaws.tobytes()


def test_remote_outside_stance_observing_task_inside_frontier_is_admitted():
    # Filtering by stance membership hides legitimate task-inside observations.
    obs = observation([(0, 0), (1, 0), (2, 0)], [(0, 1), (0, 2)],
                      inside=[1, 0, 0], gains=[1], outside_gains=[2])
    expanded = implementation().expand_frontier_actions(obs)
    assert expanded.action_nodes.tolist() == [1] * 8
    assert expanded.goals[:, :2].tolist() == [[1, 0]] * 8


def test_remote_actions_fill_missing_headings_without_duplicate_pairs_or_reordering_prefix():
    # Blindly appending all headings duplicates a legal original action and changes its index.
    obs = observation([(0, 0), (1, 0), (2, 0)], [(0, 1), (1, 2)], gains=[1, 2],
                      actions=[1, 0, 1], yaws=[HEADINGS[2], .1, HEADINGS[0]])
    expanded = implementation().expand_frontier_actions(obs, max_positions=1)
    assert expanded.action_nodes.tolist() == [1, 0, 1] + [1] * 6
    assert expanded.action_yaws[3:].tolist() == HEADINGS[[1, 3, 4, 5, 6, 7]].astype(np.float32).tolist()
    assert expanded.goals[:3].tobytes() == obs.goals.tobytes()
    pairs = list(zip(expanded.action_nodes.tolist(), expanded.action_yaws.tolist()))
    assert len(pairs) == len(set(pairs))


def test_remote_position_cap_skips_already_complete_positions():
    # Counting an already present position toward the cap prevents remote discovery.
    obs = observation([(0, 0), (1, 0), (2, 0)], [(0, 1), (1, 2)], gains=[1, 2],
                      actions=[1] * 8, yaws=HEADINGS)
    expanded = implementation().expand_frontier_actions(obs, max_positions=1)
    assert expanded.action_nodes.tolist() == [1] * 8 + [2] * 8


def test_remote_default_limit_is_32_positions():
    # An unbounded action set would change the specified experiment budget.
    obs = observation([(i, 0) for i in range(35)], [(0, i) for i in range(1, 35)],
                      lengths=list(range(1, 35)), gains=list(range(1, 35)))
    expanded = implementation().expand_frontier_actions(obs)
    assert len(expanded.action_nodes) == 256
    assert np.unique(expanded.action_nodes).tolist() == list(range(1, 33))


@pytest.mark.parametrize("limit", [0, 32])
def test_remote_no_positive_inside_candidate_falls_back_bytewise(limit):
    # Out-only gains and positive but unreachable candidates cannot invent remote actions.
    obs = observation([(0, 0), (1, 0), (2, 0)], [(0, 1)], outside_gains=[1],
                      gains=[2], actions=[1], yaws=[.25])
    expanded = implementation().expand_frontier_actions(obs, max_positions=limit)
    for name in ["action_nodes", "action_yaws", "goals", "node_ids", "features"]:
        assert getattr(expanded, name).tobytes() == getattr(obs, name).tobytes()


def test_remote_current_stance_keeps_original_turn_validity_filtering():
    # Adding missing current turns bypasses the navigation arrival/yaw filters.
    obs = observation([(0, 0)], gains=[0], actions=[0], yaws=[HEADINGS[0]])
    expanded = implementation().expand_frontier_actions(obs)
    assert expanded.action_nodes.tolist() == [0]
    assert expanded.goals.tobytes() == obs.goals.tobytes()


def test_expanded_actions_are_retained_as_real_nodes_after_contraction():
    # Contracting A2 before expansion could make a virtual hub an executable target.
    obs = observation([(i, 0) for i in range(6)], [(i, i + 1) for i in range(5)], gains=[5])
    expanded = implementation().expand_frontier_actions(obs)
    result = implementation().contract_observation(expanded)
    assert len(result.observation.action_nodes) == 8
    assert not result.virtual_mask[result.observation.action_nodes].any()
    assert result.observation.goals.tobytes() == expanded.goals.tobytes()
    assert result.observation.positions[result.observation.action_nodes].tolist() == [[5, 0]] * 8
