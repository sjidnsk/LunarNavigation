"""Finite soft Retrace: hand-derived targets and real learner integration."""
from dataclasses import replace
import copy
import math

import numpy as np
import pytest

torch = pytest.importorskip('torch')

from lunar_drl_exploration.config import LearningConfig, ModelConfig
from lunar_drl_exploration.contracts import Transition, RewardParts
from lunar_drl_exploration.sac import SACLearner, soft_values
from test_model import observation, scene, state


def target(rewards, discounts, values, selected, coefficients):
    from lunar_drl_exploration.sac import soft_retrace_target
    tensors = [torch.tensor(x, dtype=torch.float64, requires_grad=True)
               for x in (rewards, discounts, values, selected, coefficients)]
    result = soft_retrace_target(*tensors)
    assert result.ndim == 0 and not result.requires_grad
    return result.item()


@pytest.mark.parametrize('rewards,discounts,values,selected,coefficients,expected', [
    ([1.], [1.], [4.], [], [], 5.),
    ([1.], [0.], [4.], [], [], 1.),
    ([1., 2.], [1., 1.], [4., 10.], [6.], [.5], 8.),
    ([1., 2.], [1., 1.], [4., 10.], [6.], [0.], 5.),
    ([1., 200.], [1., 1.], [4., 1000.], [6.], [0.], 5.),
    ([1., 2.], [1., 0.], [4., 10.], [6.], [.5], 3.),
    ([1., 2., 3.], [1., 1., 1.], [4., 5., 6.], [7., 8.], [.5, .25], 5.125),
    ([-.1] * 8, [1.] * 8, [10.] * 8, [10.] * 7, [1.] * 7, 9.2),
    ([-.1] * 7 + [5.], [1.] * 7 + [0.], [10.] * 8, [10.] * 7, [1.] * 7, 4.3),
])
def test_retrace_hand_values_and_zero_trace_cut(rewards, discounts, values, selected, coefficients, expected):
    assert target(rewards, discounts, values, selected, coefficients) == pytest.approx(expected, abs=1e-12)


def test_clipped_ratios_use_joint_probability_and_stable_log_space():
    from lunar_drl_exploration.sac import retrace_coefficients
    # The first two actions may share a position: their probabilities must not
    # be combined. Very unlikely current actions cut the trace without epsilon.
    log_pi = torch.tensor([math.log(.2), math.log(.8), -1000.], requires_grad=True)
    log_mu = torch.tensor([math.log(.8), math.log(.2), math.log(.5)])
    actual = retrace_coefficients(log_pi, log_mu)
    torch.testing.assert_close(actual, torch.tensor([.25, 1., 0.]), atol=0, rtol=0)
    assert not actual.requires_grad


def test_entropy_is_in_successor_value_once_and_on_policy_returns_agree_in_expectation():
    probs = torch.tensor([.5, .5], dtype=torch.float64)
    q = torch.tensor([6., 0.], dtype=torch.float64)
    value = soft_values(probs, probs.log(), q, q, 1 / math.log(2),
                        torch.tensor([0, 0]), 1).item()
    assert value == pytest.approx(4.)
    assert target([1., 2.], [1., 1.], [value, 10.], [6.], [.5]) == pytest.approx(8.)
    # pi=mu, lambda=1: a sampled advantage remains, but it has zero mean.
    first = target([1., 2.], [1., 1.], [value, 10.], [6.], [1.])
    second = target([1., 2.], [1., 1.], [value, 10.], [0.], [1.])
    assert first == 11. and second == 17.
    assert .5 * first + .5 * second == 14.  # r0 + H1*alpha + r1 + V2


def sequence(length, *, episode='e', terminal=False, truncated=False, legacy=False, seed=1):
    observations = [observation(3 + j % 2, 1 + j % 2, seed=seed + j)
                    for j in range(length + 1)]
    if terminal:
        observations[-1] = replace(observations[-1], action_nodes=np.empty(0, int),
            action_yaws=np.empty(0), goals=np.empty((0, 3)))
    states = [state(obs=obs, full=j % 2 == 0) for j, obs in enumerate(observations)]
    result = []
    for j in range(length):
        metadata = {} if legacy else dict(step_index=j,
            behavior_log_prob=-math.log(len(observations[j].action_nodes)))
        result.append(Transition(observations[j], j % len(observations[j].action_nodes),
            (j - 2) / 10, observations[j + 1], states[j], states[j + 1], RewardParts(0, 0, 0),
            terminal and j == length - 1, truncated and j == length - 1, episode, j // 2, **metadata))
    return tuple(result)


def assert_tree_equal(left, right):
    if isinstance(left, torch.Tensor):
        torch.testing.assert_close(left, right, atol=0, rtol=0)
    elif isinstance(left, dict):
        assert left.keys() == right.keys()
        for key in left:
            assert_tree_equal(left[key], right[key])
    elif isinstance(left, (list, tuple)):
        assert len(left) == len(right)
        for a, b in zip(left, right):
            assert_tree_equal(a, b)
    else:
        assert left == right


def test_one_step_sequences_use_unchanged_targets_and_exact_update(monkeypatch):
    torch.set_num_threads(1)
    torch.manual_seed(7)
    config = LearningConfig(microbatch_size=8, return_steps=1)
    first = SACLearner(ModelConfig(width=16, heads=2, layers=1), config)
    second = SACLearner(first.model_config, config)
    second.load_state_dict(first.state_dict())
    sequences = [sequence(1, episode=str(i), terminal=i % 3 == 0,
        truncated=i % 3 == 1, legacy=True, seed=i + 1) for i in range(64)]
    anchors = [s[0] for s in sequences]
    scenes = {'s': scene()}
    actual, mass = second.retrace_targets(sequences[:8], scenes)
    torch.testing.assert_close(actual, first.td_targets(anchors[:8], scenes), atol=0, rtol=0)
    torch.testing.assert_close(mass, torch.ones(8), atol=0, rtol=0)
    def reject_multistep(*args):
        raise AssertionError('return_steps=1 must retain the original td_targets update path')
    monkeypatch.setattr(first, 'retrace_targets', reject_multistep)
    monkeypatch.setattr(second, 'retrace_targets', reject_multistep)
    before_rng = torch.get_rng_state().clone()
    ma, mb = first.update(anchors, scenes), second.update(sequences, scenes)
    assert_tree_equal(first.state_dict(), second.state_dict())
    assert ma == mb
    torch.testing.assert_close(torch.get_rng_state(), before_rng, atol=0, rtol=0)
    assert ma['trace_samples'] == ma['trace_length_sum'] == ma['trace_legacy_count'] == 64
    assert ma['trace_multi_count'] == 0 and ma['trace_coefficient_mass_sum'] == 64.


def test_real_network_retrace_uses_target_min_current_policy_and_terminal_masks():
    torch.set_num_threads(1)
    torch.manual_seed(13)
    learner = SACLearner(ModelConfig(width=16, heads=2, layers=1),
        LearningConfig(microbatch_size=2, return_steps=8))
    sequences = [sequence(3, terminal=True), sequence(3, truncated=True, seed=9)]
    scenes = {'s': scene()}
    actual, masses = learner.retrace_targets(sequences, scenes)
    assert not actual.requires_grad and not masses.requires_grad
    expected, expected_mass = [], []
    with torch.no_grad():
        for seq in sequences:
            value, correction_q, coefficients = [], [], []
            for i, transition in enumerate(seq):
                if transition.terminated:
                    value.append(0.)
                    continue
                obs, privileged = [transition.next_observation], [transition.next_privileged]
                policy = learner.actor(obs)[0]
                q1 = learner.target1(obs, privileged, scenes)[0]
                q2 = learner.target2(obs, privileged, scenes)[0]
                minimum = torch.minimum(q1, q2)
                value.append(float((policy.probs * (minimum - learner.alpha * policy.log_probs)).sum()))
                if i + 1 < len(seq):
                    action = seq[i + 1].action
                    correction_q.append(float(minimum[action]))
                    coefficients.append(min(1., math.exp(float(policy.log_probs[action]) -
                        seq[i + 1].behavior_log_prob)))
            # Independent three-step expansion, no production recursion helper.
            r0, r1, r2 = (t.reward for t in seq)
            c1, c2 = coefficients
            q1, q2 = correction_q
            expected.append(r0 + value[0] + c1 * (r1 + value[1] - q1) +
                            c1 * c2 * (r2 + value[2] - q2))
            expected_mass.append(1. + c1 + c1 * c2)
    torch.testing.assert_close(actual, torch.tensor(expected), atol=2e-6, rtol=2e-5)
    torch.testing.assert_close(masses, torch.tensor(expected_mass), atol=2e-6, rtol=2e-5)
    assert all(p.grad is None for net in (learner.actor, learner.target1, learner.target2)
               for p in net.parameters())


def test_multistep_update_is_finite_microbatch_equivalent_and_only_trains_anchors(monkeypatch):
    torch.set_num_threads(1)
    torch.manual_seed(17)
    model = ModelConfig(width=16, heads=2, layers=1)
    a = SACLearner(model, LearningConfig(microbatch_size=8, return_steps=8))
    b = SACLearner(model, LearningConfig(microbatch_size=4, return_steps=8))
    b.load_state_dict(a.state_dict())
    samples = [sequence((1, 2, 3, 8)[i % 4], episode=str(i), legacy=i % 8 == 0,
        terminal=i % 5 == 0, truncated=i % 5 == 1, seed=i + 1) for i in range(64)]
    target_sizes, actor_grad_sizes, critic_grad_sizes = [], [], []
    def observe_forward(network, *, target_network=False, actor=False):
        original = network.forward_packed
        def wrapped(batch, *args, **kwargs):
            assert batch.size <= 8, 'graph inference exceeded microbatch bound'
            if target_network:
                target_sizes.append(batch.size)
                assert not torch.is_grad_enabled()
            elif torch.is_grad_enabled():
                (actor_grad_sizes if actor else critic_grad_sizes).append(batch.size)
            return original(batch, *args, **kwargs)
        monkeypatch.setattr(network, 'forward_packed', wrapped)
    for net in (a.target1, a.target2):
        observe_forward(net, target_network=True)
    observe_forward(a.actor, actor=True)
    for net in (a.q1, a.q2):
        observe_forward(net)
    ma, mb = a.update(samples, {'s': scene()}), b.update(samples, {'s': scene()})
    assert target_sizes and sum(actor_grad_sizes) == 64 and sum(critic_grad_sizes) == 128
    assert ma['trace_samples'] == 64 and ma['trace_length_sum'] == 224
    assert ma['trace_multi_count'] == 48 and ma['trace_legacy_count'] == 8
    assert 64. <= ma['trace_coefficient_mass_sum'] <= 224.
    for metrics in (ma, mb):
        assert all(math.isfinite(value) for value in metrics.values())
    for name in ('actor', 'q1', 'q2', 'target1', 'target2'):
        for key, value in getattr(a, name).state_dict().items():
            torch.testing.assert_close(value, getattr(b, name).state_dict()[key], atol=3e-6, rtol=3e-5)
    torch.testing.assert_close(a.log_alpha, b.log_alpha, atol=1e-7, rtol=1e-6)
    for name in ('actor_optimizer', 'q1_optimizer', 'q2_optimizer', 'alpha_optimizer'):
        for values in getattr(a, name).state.values():
            assert values['step'] == 1
            assert all(torch.isfinite(value).all() for value in values.values() if isinstance(value, torch.Tensor))


def test_saved_legacy_learner_defaults_to_one_step_but_cannot_resume_as_eight():
    model = ModelConfig(width=16, heads=2, layers=1)
    a = SACLearner(model)
    saved = a.state_dict()
    saved['learning_config'].pop('return_steps', None)
    original = copy.deepcopy(saved)
    a.load_state_dict(saved)
    b = SACLearner(model, LearningConfig(return_steps=8))
    with pytest.raises(ValueError, match='incompatible SAC semantics'):
        b.load_state_dict(saved)
    assert_tree_equal(saved, original)
    migrated = copy.deepcopy(saved)
    migrated['learning_config']['return_steps'] = 8
    b.load_state_dict(migrated)
    assert b.state_dict()['learning_config']['return_steps'] == 8


@pytest.mark.parametrize('kind', ['empty', 'too_long', 'boundary', 'missing_metadata'])
def test_invalid_retrace_sequences_fail_before_optimizer_changes(kind):
    learner = SACLearner(ModelConfig(width=16, heads=2, layers=1), LearningConfig(return_steps=8))
    good = sequence(2)
    if kind == 'empty':
        bad = ()
    elif kind == 'too_long':
        bad = sequence(9)
    elif kind == 'boundary':
        bad = (replace(good[0], truncated=True), good[1])
    else:
        bad = (good[0], replace(good[1], step_index=None, behavior_log_prob=None))
    before = learner.state_dict()
    with pytest.raises(ValueError, match='sequence|metadata|boundary'):
        learner.update([bad] + [good] * 63, {'s': scene()})
    assert_tree_equal(before, learner.state_dict())
