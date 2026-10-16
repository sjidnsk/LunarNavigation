"""Independent position/conditional-heading temperatures on the existing joint actor."""
from dataclasses import asdict, replace
import math
from types import SimpleNamespace

import pytest
torch = pytest.importorskip('torch')

from lunar_drl_exploration import sac
from lunar_drl_exploration.batch import pack_observations
from lunar_drl_exploration.config import (LearningConfig, ModelConfig, TrainingConfig,
    canonical_learning_config, resume_semantics)
from lunar_drl_exploration.model import segment_log_softmax
from test_model import scene
from test_sac import transitions
from test_soft_retrace import sequence, assert_tree_equal


MODEL = ModelConfig(width=16, heads=4, layers=1)


def policy_batch(logits, nodes, groups, graph_groups):
    logits = torch.tensor(logits, dtype=torch.float64, requires_grad=True)
    groups = torch.tensor(groups)
    count = max(graph_groups) + 1
    logp = segment_log_softmax(logits, groups, count)
    policy = SimpleNamespace(log_probs=logp, probs=logp.exp())
    batch = SimpleNamespace(action_nodes=torch.tensor(nodes), action_groups=groups,
        size=count, graph=SimpleNamespace(features=torch.empty(len(graph_groups), 0),
            groups=torch.tensor(graph_groups)))
    return logits, policy, batch


def test_ragged_reference_targets_preserve_joint_target_and_global_node_offsets():
    # State 0: position masses .2/.8, with 1/3 headings. State 1: one
    # position and two headings. Nodes 0,2,4 are graph-only (not actions).
    _, policy, batch = policy_batch([math.log(x) for x in (.2,.4,.2,.2,.25,.75)],
        [1,3,3,3,5,5], [0,0,0,0,1,1], [0,0,0,0,1,1])
    h = sac.policy_entropies(policy, batch, position_factor=.1, heading_factor=.1)
    hp = -(.2*math.log(.2)+.8*math.log(.8))
    hh0 = -.8*(.5*math.log(.5)+.25*math.log(.25)*2)
    hh1 = -(.25*math.log(.25)+.75*math.log(.75))
    torch.testing.assert_close(h.position_entropy, torch.tensor([hp,0.], dtype=torch.float64))
    torch.testing.assert_close(h.heading_entropy, torch.tensor([hh0,hh1], dtype=torch.float64))
    torch.testing.assert_close(h.position_target, torch.tensor([
        -.1*(.25*math.log(.25)+.75*math.log(.75)),0.], dtype=torch.float64))
    torch.testing.assert_close(h.heading_target, torch.tensor([
        .1*.75*math.log(3),.1*math.log(2)], dtype=torch.float64))
    torch.testing.assert_close(h.position_target+h.heading_target,
        torch.tensor([.1*math.log(4),.1*math.log(2)], dtype=torch.float64))
    assert not h.position_target.requires_grad and not h.heading_target.requires_grad
    torch.testing.assert_close(h.position_log_probs+h.heading_log_probs, policy.log_probs)


def test_equal_temperatures_match_joint_soft_value_actor_loss_and_gradient():
    logits, policy, batch = policy_batch([1.,-2.,3.,.5,2.], [0,0,2,2,2], [0]*5, [0]*4)
    h = sac.policy_entropies(policy, batch)
    bonus = sac.factorized_entropy_bonus(h, .17, .17)
    q1 = torch.tensor([.3,1.,-.2,.7,.4], requires_grad=True)
    q2 = torch.tensor([.4,1.2,-.3,.8,.1], requires_grad=True)
    joint = sac.policy_loss(policy.probs, policy.log_probs, q1, q2, .17, batch.action_groups, 1)
    split = sac.policy_loss(policy.probs, policy.log_probs, q1, q2, .17,
        batch.action_groups, 1, entropy_bonus=bonus)
    torch.testing.assert_close(split, joint, atol=1e-14, rtol=1e-14)
    a, = torch.autograd.grad(joint, logits, retain_graph=True)
    b, = torch.autograd.grad(split, logits)
    torch.testing.assert_close(a, b, atol=1e-14, rtol=1e-14)
    assert q1.grad is None and q2.grad is None
    torch.testing.assert_close(sac.soft_values(policy.probs, policy.log_probs, q1, q2, .17,
        batch.action_groups, 1, entropy_bonus=bonus), -split.reshape(1))


def test_position_concentration_moves_position_temperature_independently_of_heading():
    # Two equally likely headings at each position, but 99.9% on one position.
    logits, policy, batch = policy_batch([math.log(x) for x in (.4995,.4995,.0005,.0005)],
        [0,0,1,1], [0]*4, [0,0])
    h = sac.policy_entropies(policy, batch)
    position_grad, = torch.autograd.grad(h.position_entropy.sum(), logits, retain_graph=True)
    heading_grad, = torch.autograd.grad(h.heading_entropy.sum(), logits, retain_graph=True)
    assert position_grad[0] < 0 < position_grad[2]
    torch.testing.assert_close(heading_grad, torch.zeros_like(heading_grad), atol=1e-15, rtol=0)
    lp, lh = (torch.tensor(-5., requires_grad=True) for _ in range(2))
    loss = sac.temperature_loss(lp, h.position_entropy, h.position_target)
    loss += sac.temperature_loss(lh, h.heading_entropy, h.heading_target)
    loss.backward()
    assert lp.grad < 0 < lh.grad
    assert logits.grad is None


@pytest.mark.parametrize('logits,nodes', [([1000.,-1000.,-1001.],[0,1,1]),
    ([-1000.,-1001.,1000.],[0,0,1]), ([0.],[1]), ([2.,-3.],[1,1]), ([2.,-3.],[0,1])])
def test_extreme_logits_and_singleton_components_keep_values_and_gradients_finite(logits,nodes):
    raw, policy, batch = policy_batch(logits, nodes, [0]*len(nodes), [0,0,0])
    h = sac.policy_entropies(policy, batch)
    bonus = sac.factorized_entropy_bonus(h, .3, .07)
    loss = (policy.probs*bonus).sum()
    loss.backward()
    assert all(torch.isfinite(value).all() for value in vars(h).values())
    assert torch.isfinite(raw.grad).all()
    if len(set(nodes)) == 1:
        torch.testing.assert_close(h.position_entropy, torch.zeros(1,dtype=torch.float64), atol=1e-15,rtol=0)
        assert h.position_target.item() == 0.
    if len(set(nodes)) == len(nodes):
        assert h.heading_entropy.item() == h.heading_target.item() == 0.


@pytest.mark.parametrize('name,value', [('entropy_mode','unknown'),
    ('position_target_entropy_factor',-1.),('heading_target_entropy_factor',-1.),
    ('position_target_entropy_factor',float('nan')),('heading_target_entropy_factor',float('inf')),
    ('temperature_learning_rate',0.),('temperature_learning_rate',float('nan')),
    ('temperature_learning_rate',float('inf')),('learning_rate',float('inf')),
    ('maximum_alpha',float('inf')),('initial_alpha',float('nan')),
    ('target_entropy_factor',float('inf'))])
def test_invalid_entropy_and_nonfinite_optimization_configs_rejected(name,value):
    with pytest.raises(ValueError):
        LearningConfig(**{name:value})
    with pytest.raises(ValueError):
        canonical_learning_config(dict(asdict(LearningConfig()), **{name:value}))


def test_legacy_config_canonicalization_and_temperature_rate_is_only_an_optimizer_control():
    defaults = asdict(LearningConfig())
    legacy = {k:v for k,v in defaults.items() if k not in ('return_steps','entropy_mode',
        'position_target_entropy_factor','heading_target_entropy_factor','temperature_learning_rate')}
    assert canonical_learning_config(legacy) == defaults
    base = TrainingConfig()
    changed = replace(base, learning=replace(base.learning, temperature_learning_rate=.003))
    assert resume_semantics(base) == resume_semantics(changed)


@pytest.mark.parametrize('mode,legacy', [('joint',True),('joint',False),('position_heading',False)])
def test_low_cap_checkpoint_semantics_validate_without_absent_initial_alpha(mode,legacy):
    from lunar_drl_exploration.checkpoint import UpdateBoundary
    from lunar_drl_exploration.replay import ReplayBuffer
    from lunar_drl_exploration.schedule import UpdateSchedule
    from test_replay_checkpoint import capture_state, scene as replay_scene, transition
    config = replace(TrainingConfig(),model=MODEL,learning=LearningConfig(
        entropy_mode=mode,initial_alpha=1e-6,maximum_alpha=1e-5))
    learner = sac.SACLearner(MODEL,config.learning)
    replay = ReplayBuffer(1000000)
    replay.add(transition(),{'s':replay_scene()})
    schedule = UpdateSchedule()
    schedule.collected()
    saved = capture_state(learner,config,replay,schedule,UpdateBoundary())
    if legacy:
        for record in (saved.record['semantics']['learning'],saved.record['config']['learning'],
                       saved.record['learner']['learning_config']):
            for field in ('entropy_mode','position_target_entropy_factor',
                          'heading_target_entropy_factor','temperature_learning_rate'):
                record.pop(field,None)
    semantic_record = canonical_learning_config(saved.record['semantics']['learning'])
    assert semantic_record['maximum_alpha'] == 1e-5
    assert 'initial_alpha' not in semantic_record and 'learning_rate' not in semantic_record
    saved.validate()
    saved.validate(config)
    restored = sac.SACLearner(MODEL,config.learning)
    saved.restore(restored,config)
    torch.testing.assert_close(restored.alpha,learner.alpha,atol=0,rtol=0)


def test_canonical_full_config_still_rejects_initial_temperature_above_cap():
    full = asdict(LearningConfig())
    with pytest.raises(ValueError,match='optimization'):
        canonical_learning_config(dict(full,maximum_alpha=1e-5))


def learner_config(**kwargs):
    return LearningConfig(entropy_mode='position_heading', **kwargs)


def test_split_update_microbatch_weights_roundtrip_and_current_temperature_learning_rate():
    torch.manual_seed(25)
    config = learner_config(microbatch_size=64, temperature_learning_rate=.003)
    full = sac.SACLearner(MODEL, config)
    micro = sac.SACLearner(MODEL, replace(config, microbatch_size=13))
    micro.load_state_dict(full.state_dict())
    samples, scenes = transitions(), {'s':scene()}
    a, b = full.update(samples,scenes), micro.update(samples,scenes)
    for key in a:
        assert math.isfinite(a[key])
        assert a[key] == pytest.approx(b[key], abs=3e-6, rel=3e-5), key
    for key in ('alpha_position','alpha_heading','position_entropy_target','heading_entropy_target'):
        assert key in a
    saved = full.state_dict()
    restored = sac.SACLearner(MODEL, replace(config, learning_rate=2e-5, temperature_learning_rate=.004))
    restored.load_state_dict(saved)
    for name in ('alpha_optimizer','heading_alpha_optimizer'):
        assert_tree_equal(getattr(restored,name).state_dict()['state'],saved[name]['state'])
        assert getattr(restored,name).param_groups[0]['lr'] == .004
    assert restored.actor_optimizer.param_groups[0]['lr'] == 2e-5
    torch.testing.assert_close(restored.log_alpha_heading, saved['log_alpha_heading'],atol=0,rtol=0)
    full.update(samples,scenes)
    micro.update(samples,scenes)
    for name in ('actor','q1','q2','target1','target2'):
        for key,value in getattr(full,name).state_dict().items():
            torch.testing.assert_close(value,getattr(micro,name).state_dict()[key],atol=3e-6,rtol=3e-5)
    assert full.log_alpha == micro.log_alpha
    assert full.log_alpha_heading == micro.log_alpha_heading
    # Both trainable values, not only exposed metrics, obey the shared cap.
    with torch.no_grad():
        full.log_alpha.fill_(math.log(config.maximum_alpha)+1.)
        full.log_alpha_heading.fill_(math.log(config.maximum_alpha)+1.)
    full.update(samples,scenes)
    assert full.alpha_position <= config.maximum_alpha*1.000001
    assert full.alpha_heading <= config.maximum_alpha*1.000001


def test_resume_never_implicitly_migrates_modes_and_requires_all_split_state():
    joint = sac.SACLearner(MODEL, LearningConfig())
    split = sac.SACLearner(MODEL, learner_config())
    assert 'log_alpha_heading' not in joint.state_dict()
    assert 'heading_alpha_optimizer' not in joint.state_dict()
    assert joint.alpha_heading == joint.alpha_position == joint.alpha
    with pytest.raises(ValueError,match='incompatible SAC semantics'):
        split.load_state_dict(joint.state_dict())
    with pytest.raises(ValueError,match='incompatible SAC semantics'):
        joint.load_state_dict(split.state_dict())
    for key in ('log_alpha_heading','heading_alpha_optimizer'):
        saved = split.state_dict()
        del saved[key]
        with pytest.raises(ValueError,match='heading'):
            split.load_state_dict(saved)


def test_single_position_has_exactly_zero_temperature_gradient_in_float32():
    batch = pack_observations([sequence(1)[0].observation], 'cpu')
    logits = torch.randn(8,generator=torch.Generator().manual_seed(4),requires_grad=True)
    logp = segment_log_softmax(logits, batch.action_groups, batch.size)
    h = sac.policy_entropies(SimpleNamespace(log_probs=logp,probs=logp.exp()),batch)
    logalpha = torch.tensor(-5., requires_grad=True)
    sac.temperature_loss(logalpha,h.position_entropy,h.position_target).backward()
    assert h.position_entropy.item() == h.position_target.item() == logalpha.grad.item() == 0.


@pytest.mark.skipif(not torch.cuda.is_available(),reason='CUDA required for synchronization audit')
def test_split_cuda_update_has_bounded_phase_sync_and_restores_both_adam_states_on_cpu():
    from torch.utils._python_dispatch import TorchDispatchMode
    scalar_reads = []
    class PhaseDeviceSync(TorchDispatchMode):
        def __torch_dispatch__(self,func,types,args=(),kwargs=None):
            if func == torch.ops.aten._local_scalar_dense.default:
                if args[0].is_cuda:
                    scalar_reads.append(args[0].numel())
            return func(*args,**(kwargs or {}))
    learner = sac.SACLearner(MODEL,learner_config(),device='cuda')
    with PhaseDeviceSync():
        metrics = learner.update(transitions(),{'s':scene()})
    assert all(math.isfinite(value) for value in metrics.values())
    # Four optimizer phases and parameter gates, independent of transitions.
    assert len(scalar_reads) <= 12 and all(size == 1 for size in scalar_reads)
    assert metrics['numeric_guard_version'] == 1 and metrics['numeric_checked_updates'] == 1
    for key in ('actor_grad_norm', 'q1_grad_norm', 'q2_grad_norm', 'temperature_grad_norm',
                'log_alpha_position', 'log_alpha_heading'):
        assert math.isfinite(metrics[key])
    cpu = sac.SACLearner(MODEL,learner_config())
    cpu.load_state_dict(learner.state_dict())
    for optimizer in (cpu.alpha_optimizer,cpu.heading_alpha_optimizer):
        assert all(value['exp_avg'].device.type=='cpu' for value in optimizer.state.values())


@pytest.mark.parametrize('terminal,truncated', [(True,False),(False,True),(False,False)])
def test_split_td_and_retrace_use_both_temperatures_and_respect_terminal_truncation(terminal,truncated):
    learner = sac.SACLearner(MODEL, learner_config(return_steps=3, gamma=.95))
    with torch.no_grad():
        learner.log_alpha.fill_(math.log(.4))
        learner.log_alpha_heading.fill_(math.log(.07))
    seq = sequence(3,terminal=terminal,truncated=truncated)
    scenes = {'s':scene()}
    expected_values, selected, ratios = [], [], []
    with torch.no_grad():
        for i,t in enumerate(seq):
            if t.terminated:
                expected_values.append(torch.tensor(0.))
                continue
            p = learner.actor([t.next_observation])[0]
            q = torch.minimum(learner.target1([t.next_observation],[t.next_privileged],scenes)[0],
                learner.target2([t.next_observation],[t.next_privileged],scenes)[0])
            # Independent enumeration on each observation, using logsumexp.
            nodes = t.next_observation.action_nodes
            position_logp = torch.stack([torch.logsumexp(p.log_probs[nodes==node],0) for node in nodes])
            value = (p.probs*(q-.4*position_logp-.07*(p.log_probs-position_logp))).sum()
            expected_values.append(value)
            if i+1<len(seq):
                selected.append(q[seq[i+1].action])
                ratios.append((p.log_probs[seq[i+1].action]-seq[i+1].behavior_log_prob).clamp(max=0).exp())
        want_td = torch.stack([t.reward+.95*(not t.terminated)*v for t,v in zip(seq,expected_values)])
        want_retrace = want_td[-1]
        for i in range(1,-1,-1):
            want_retrace = seq[i].reward+.95*(expected_values[i]+ratios[i]*(want_retrace-selected[i]))
    torch.testing.assert_close(learner.td_targets(seq,scenes),want_td,atol=2e-6,rtol=2e-5)
    got,_ = learner.retrace_targets([seq],scenes)
    torch.testing.assert_close(got[0],want_retrace,atol=2e-6,rtol=2e-5)
    assert not got.requires_grad
