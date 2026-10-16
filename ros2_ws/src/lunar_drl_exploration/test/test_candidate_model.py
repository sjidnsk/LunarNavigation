"""Behavioral invariants of indexed position-heading scores and both Qs."""
from dataclasses import replace
import importlib
import numpy as np
import pytest
import torch

from lunar_drl_exploration.config import ModelConfig
from lunar_drl_exploration.model import Actor, Critic
from lunar_drl_exploration.batch import pack_observations, pack_privileged
from lunar_drl_exploration.contracts import Pose
from test_candidate_observation import fine_observation, implementation
from test_model import scene, state

torch.set_num_threads(2)


def model_config():
    return ModelConfig(architecture_version='candidate_geometry_v3')


def observed():
    raw = fine_observation()
    f = raw.features.copy()
    f[4, 3:11] = np.arange(8) / 100.
    f[4, 11:19] = np.arange(8)[::-1] / 100.
    return implementation().prepare_candidate_observation(
        replace(raw, features=f), Pose(0., 0., 0.)).observation


def test_ragged_node_action_permutations_single_and_empty_states():
    torch.manual_seed(31)
    actor = Actor(model_config()).eval()
    obs = observed()
    singleton = replace(obs, action_pairs=obs.action_pairs[:1], goals=obs.goals[:1])
    empty = replace(obs, action_pairs=np.empty((0, 2), int), goals=np.empty((0, 3)))
    with torch.no_grad():
        outputs = actor([obs, singleton, empty])
        torch.testing.assert_close(outputs[0].probs, actor([obs])[0].probs)
        assert outputs[1].probs.tolist() == [1.] and outputs[2].probs.numel() == 0
        order = np.arange(len(obs.node_ids))[::-1].copy(); inverse = np.argsort(order)
        actions = np.arange(len(obs.goals))[::-1].copy()
        shuffled = replace(obs, node_ids=obs.node_ids[order], node_geometry=obs.node_geometry[order],
            node_views=obs.node_views[order], node_is_real=obs.node_is_real[order],
            edges=inverse[obs.edges[::-1]], edge_lengths=obs.edge_lengths[::-1],
            current_index=int(inverse[obs.current_index]),
            action_pairs=np.column_stack((inverse[obs.action_nodes[actions]], obs.action_pairs[actions, 1])),
            goals=obs.goals[actions])
        torch.testing.assert_close(actor([shuffled])[0].probs, outputs[0].probs[actions], atol=2e-6, rtol=2e-5)
        critic = Critic(model_config()).eval(); truth = scene()
        q = critic([obs], [state(obs=obs)], {'s': truth})[0]
        qs = critic([shuffled], [state(obs=shuffled)], {'s': truth})[0]
        torch.testing.assert_close(qs, q[actions], atol=2e-6, rtol=2e-5)


def test_polygon_winding_and_start_leave_actor_and_critic_unchanged():
    torch.manual_seed(8)
    obs = observed(); actor = Actor(model_config()).eval(); critic = Critic(model_config()).eval()
    with torch.no_grad():
        original = actor([obs])[0].logits
        q = critic([obs], [state(obs=obs)], {'s': scene()})[0]
        for polygon in (obs.polygon[::-1], np.roll(obs.polygon, 2, axis=0)):
            other = replace(obs, polygon=polygon)
            torch.testing.assert_close(actor([other])[0].logits, original, atol=2e-6, rtol=2e-5)
            torch.testing.assert_close(critic([other], [state(obs=other)], {'s': scene()})[0], q, atol=2e-6, rtol=2e-5)


def test_virtual_rows_are_not_encoded_and_each_physical_view_reaches_scores():
    actor = Actor(model_config())
    obs = observed(); batch = pack_observations([obs], 'cpu')
    rows = []
    handles = [actor.encoder.position.register_forward_pre_hook(lambda _, args: rows.append(len(args[0]))),
               actor.encoder.view.register_forward_pre_hook(lambda _, args: rows.append(len(args[0])))]
    encoded = actor.encoder(batch)
    for h in handles: h.remove()
    assert rows == [int(obs.node_is_real.sum()), int(obs.node_is_real.sum())*8]
    # Swapping east/west directional gains must remain distinguishable before
    # pooling and at the selected action head; geometry has a direct gradient.
    view = obs.node_views.copy(); node = obs.action_nodes[0]
    view[node, [0, 4]] = view[node, [4, 0]]
    changed = pack_observations([replace(obs, node_views=view)], 'cpu')
    assert not torch.allclose(actor.encoder(changed).views, encoded.views)
    batch.graph.features.requires_grad_()
    result = actor.forward_packed(batch)
    (result.logits[0]-result.logits[4]).backward()
    assert torch.isfinite(batch.graph.features.grad).all()
    assert batch.graph.features.grad[batch.real_nodes, 2:4].abs().sum() > 0
    assert batch.graph.features.grad[~torch.tensor(obs.node_is_real)].abs().sum() == 0


@pytest.mark.parametrize('widths', [(128,)*4, (128,)*6+(1,)])
@pytest.mark.parametrize('dtype', [torch.float32, torch.float64])
def test_cached_head_matches_concat_outputs_and_all_gradients(widths, dtype):
    try:
        head_type = importlib.import_module('lunar_drl_exploration.candidate_model').FactoredScoreHead
    except ImportError:
        pytest.fail('candidate model implementation is missing')
    torch.manual_seed(12)
    head = head_type(widths, 128).to(dtype=dtype)
    sizes = [7, 7, 34, 2, 2, 7, 34][:len(widths)]
    parts = [torch.randn(n, d, dtype=dtype, requires_grad=True) for n, d in zip(sizes, widths)]
    indices = [torch.arange(34) % n for n in sizes]
    actual = head(parts, indices)
    full = head.output(torch.nn.functional.gelu(head.first(torch.cat(
        [v[i] for v, i in zip(parts, indices)], dim=-1)))).squeeze(-1)
    variables = parts + list(head.parameters())
    a = torch.autograd.grad(actual.square().sum(), variables, retain_graph=True)
    b = torch.autograd.grad(full.square().sum(), variables)
    tolerance = 2e-5 if dtype == torch.float32 else 1e-11
    torch.testing.assert_close(actual, full, atol=tolerance, rtol=tolerance)
    for x, y in zip(a, b): torch.testing.assert_close(x, y, atol=tolerance, rtol=tolerance)


def test_truth_geometry_uses_build_pose_and_actor_does_not_require_truth():
    from lunar_drl_exploration.candidate_observation import body_vectors
    obs = observed()
    # Keep relative observation identical, move world goals and truth together.
    angle = np.pi/2
    pose = Pose(10., 20., angle)
    goals = obs.goals.copy()
    goals[:, :2] = body_vectors(goals[:, :2], -angle) + [pose.x, pose.y]
    goals[:, 2] = np.remainder(goals[:, 2]+angle, 2*np.pi)
    pairs = obs.action_pairs.copy(); pairs[:, 1] = (pairs[:, 1]+2)%8
    headings = np.arange(8)*np.pi/4-angle
    rotated = replace(obs, build_pose=pose, goals=goals, action_pairs=pairs,
        heading_features=np.column_stack((np.cos(headings), np.sin(headings))),
        node_views=np.roll(obs.node_views, 2, axis=1))
    truth = scene()
    rotated_truth = replace(truth, positions=body_vectors(truth.positions, -angle)+[pose.x, pose.y])
    original_state = state(obs=obs)
    rotated_state = replace(original_state, actions=replace(original_state.actions,
        positions=body_vectors(original_state.actions.positions, -angle)+[pose.x, pose.y],
        action_yaws=goals[:, 2]))
    a = pack_privileged([obs], [original_state], {'s': truth}, 'cpu')
    b = pack_privileged([rotated], [rotated_state], {'s': rotated_truth}, 'cpu')
    torch.testing.assert_close(a.graph.features, b.graph.features, atol=2e-6, rtol=2e-5)
    torch.testing.assert_close(a.support_geometry, b.support_geometry, atol=2e-6, rtol=2e-5)
    actor = Actor(model_config()).eval(); critic = Critic(model_config()).eval()
    with torch.no_grad():
        torch.testing.assert_close(actor([obs])[0].probs, actor([rotated])[0].probs, atol=2e-6, rtol=2e-5)
        torch.testing.assert_close(critic([obs], [original_state], {'s': truth})[0],
            critic([rotated], [rotated_state], {'s': rotated_truth})[0], atol=2e-6, rtol=2e-5)


def test_mixed_batch_and_wrong_architecture_fail_explicitly():
    obs = observed()
    with pytest.raises(ValueError, match='schema'):
        pack_observations([obs, fine_observation()], 'cpu')
    with pytest.raises(ValueError, match='schema'):
        Actor(model_config())([fine_observation()])
    with pytest.raises(ValueError, match='schema'):
        Actor(ModelConfig())([obs])
