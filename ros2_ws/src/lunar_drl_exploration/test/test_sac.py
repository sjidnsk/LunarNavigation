"""Numerical SAC invariants use full action expectations and independent weights."""
from dataclasses import replace
import copy
import math
import numpy as np
import pytest
torch = pytest.importorskip("torch")
from lunar_drl_exploration.config import ModelConfig, LearningConfig
from lunar_drl_exploration.contracts import Transition, RewardParts
from lunar_drl_exploration.sac import SACLearner, soft_values, policy_loss, temperature_loss
from test_model import observation, scene, state


def transitions():
    result=[]
    for i in range(64):
        obs = observation(3+i%4, 1+i%3, seed=i+1)
        next_obs = observation(3+(i+1)%4, 1+(i+1)%3, seed=i+2)
        terminal = i%5==0
        if terminal:
            next_obs=replace(next_obs, action_nodes=np.empty(0,int),
                action_yaws=np.empty(0),goals=np.empty((0,3)))
        result.append(Transition(obs, i%len(obs.goals), (i-30)/100., next_obs,
            state(obs=obs),state(full=i%2==0,obs=next_obs),RewardParts(0,0,0,0,0),terminal,i%7==0 and not terminal,'e',0))
    return result


def test_enumerated_math_and_temperature_direction():
    logp=torch.tensor([.25,.75,.1,.2,.7]).log().requires_grad_()
    probs=logp.exp()
    q1=torch.tensor([2.,4.,1.,2.,3.],requires_grad=True)
    q2=torch.tensor([3.,1.,2.,1.,4.],requires_grad=True)
    groups=torch.tensor([0,0,1,1,1])
    expected=torch.tensor([.25*2+.75*1,.1*1+.2*1+.7*3])
    entropy=torch.tensor([-(.25*math.log(.25)+.75*math.log(.75)),
        -(.1*math.log(.1)+.2*math.log(.2)+.7*math.log(.7))])
    torch.testing.assert_close(soft_values(probs,logp,q1,q2,.1,groups,2),expected+.1*entropy)
    loss=policy_loss(probs,logp,q1,q2,.1,groups,2)
    torch.testing.assert_close(loss,(-expected-.1*entropy).mean())
    loss.backward()
    assert q1.grad is None and q2.grad is None and logp.grad is not None
    for measured, sign in ((1.,1), (0.,-1)):
        logalpha=torch.tensor(math.log(5e-5),requires_grad=True)
        h=torch.tensor([measured],requires_grad=True)
        temperature_loss(logalpha,h,torch.tensor([.1])).backward()
        assert logalpha.grad*sign > 0 and h.grad is None


@pytest.mark.parametrize('gamma', [.995, 1.0])
def test_terminal_skips_empty_actions_truncated_bootstraps_and_target_detaches(gamma):
    learner=SACLearner(ModelConfig(),LearningConfig(gamma=gamma),device='cpu')
    samples=transitions()[:3]
    samples[1]=replace(samples[1],truncated=True)
    actual=learner.td_targets(samples,{'s':scene()})
    assert not actual.requires_grad
    with torch.no_grad():
        obs=[t.next_observation for t in samples[1:]]
        states=[t.next_privileged for t in samples[1:]]
        policies=learner.actor(obs)
        q1=learner.target1(obs,states,{'s':scene()}); q2=learner.target2(obs,states,{'s':scene()})
        expected=[torch.tensor(samples[0].reward)]
        for t,p,a,b in zip(samples[1:],policies,q1,q2):
            expected.append(t.reward+learner.learning_config.gamma*(p.probs*(torch.minimum(a,b)-learner.alpha*p.log_probs)).sum())
    torch.testing.assert_close(actual,torch.stack(expected))


def test_micro64_and_16x4_match_steps_restore_publication_and_independence():
    torch.manual_seed(31)
    a=SACLearner(ModelConfig(),LearningConfig(microbatch_size=64),device='cpu')
    b=SACLearner(ModelConfig(),LearningConfig(microbatch_size=16),device='cpu')
    b.load_state_dict(a.state_dict())
    # Eval uses the identical deterministic network without activation recomputation.
    for network in (a.actor, a.q1, a.q2):
        network.eval()
    storages=[{p.data_ptr() for p in m.parameters()} for m in
        (a.actor,a.q1,a.q2,a.target1,a.target2)]
    for i, left in enumerate(storages):
        for right in storages[i+1:]: assert left.isdisjoint(right)
    before=copy.deepcopy(a.state_dict())
    publication=a.actor_state()
    assert publication['version']==0 and all(v.device.type=='cpu' for v in publication['state_dict'].values())
    samples=transitions(); scenes={'s':scene()}
    ma=a.update(samples,scenes); mb=b.update(samples,scenes)
    assert a.updates==b.updates==1
    assert a.alpha < a.learning_config.initial_alpha
    for key in ('critic1_loss','critic2_loss','actor_loss','alpha_loss','entropy','target_mean'):
        assert ma[key]==pytest.approx(mb[key],abs=2e-6,rel=2e-5)
    for name in ('actor','q1','q2','target1','target2'):
        for key,value in getattr(a,name).state_dict().items():
            torch.testing.assert_close(value,getattr(b,name).state_dict()[key],atol=2e-6,rtol=2e-5)
    for net in (a.q1,a.q2,a.target1,a.target2):
        assert all(p.grad is None for p in net.parameters())
    for opt in (a.actor_optimizer,a.q1_optimizer,a.q2_optimizer,a.alpha_optimizer):
        assert all(s['step']==1 for s in opt.state.values())
    for key,value in a.target1.state_dict().items():
        torch.testing.assert_close(value,before['target1'][key]*.995+a.q1.state_dict()[key]*.005)
    assert a.actor_state()['version']==1
    assert any(not torch.equal(publication['state_dict'][k],v) for k,v in a.actor.state_dict().items())
    import io
    stream=io.BytesIO()
    torch.save(a.state_dict(),stream)
    stream.seek(0)
    checkpoint=torch.load(stream,map_location='cpu',weights_only=True)
    c=SACLearner(ModelConfig(),LearningConfig(),device='cpu'); c.load_state_dict(checkpoint)
    assert c.updates==1
    a.update(samples,scenes); c.update(samples,scenes)
    for name in ('actor','q1','q2','target1','target2'):
        for key,value in getattr(a,name).state_dict().items():
            torch.testing.assert_close(value,getattr(c,name).state_dict()[key],atol=3e-6,rtol=3e-5)
    assert c.log_alpha==a.log_alpha
    with pytest.raises(ValueError): c.update(samples[:16],scenes)


def test_temperature_caps_trainable_value_without_windup():
    learner=SACLearner(ModelConfig(),LearningConfig(),device='cpu')
    with torch.no_grad(): learner.log_alpha.fill_(math.log(1e-4)+.1)
    learner.update(transitions(),{'s':scene()})
    assert learner.log_alpha <= math.log(1e-4)
    assert learner.alpha <= 1.000001e-4


def test_saved_state_owns_tensors_and_rejects_old_schema():
    learner=SACLearner(ModelConfig(),LearningConfig(),device='cpu')
    saved=learner.state_dict()
    first=next(iter(saved['actor']))
    snapshot=saved['actor'][first].clone()
    with torch.no_grad(): next(learner.actor.parameters()).add_(1)
    torch.testing.assert_close(saved['actor'][first],snapshot,atol=0,rtol=0)
    with pytest.raises(ValueError): learner.load_state_dict(dict(saved,schema='old_gru'))


@pytest.mark.skipif(not torch.cuda.is_available(),reason='CUDA required for synchronization audit')
def test_cuda_complete_update_has_only_bounded_phase_synchronization():
    from torch.utils._python_dispatch import TorchDispatchMode
    scalar_reads=[]
    class PhaseDeviceSync(TorchDispatchMode):
        def __torch_dispatch__(self,func,types,args=(),kwargs=None):
            if func == torch.ops.aten._local_scalar_dense.default:
                if args[0].is_cuda: scalar_reads.append(args[0].numel())
            return func(*args,**(kwargs or {}))
    learner=SACLearner(ModelConfig(),LearningConfig(),device='cuda')
    with PhaseDeviceSync():
        metrics=learner.update(transitions(),{'s':scene()})
    # Only aggregate phase gates synchronize; no transition/node/edge loops read scalars.
    assert len(scalar_reads) <= 12 and all(size==1 for size in scalar_reads)
    assert metrics['updates']==1 and all(math.isfinite(v) for v in metrics.values())
    assert metrics['numeric_guard_version']==1 and metrics['numeric_checked_updates']==1
    restored=SACLearner(ModelConfig(),LearningConfig(),device='cpu')
    restored.load_state_dict(learner.state_dict())
    assert restored.updates==1
    gpu_publication=learner.actor_state()['state_dict']
    for key,value in restored.actor_state()['state_dict'].items():
        torch.testing.assert_close(value,gpu_publication[key])
    for optimizer in (restored.actor_optimizer,restored.q1_optimizer,
                      restored.q2_optimizer,restored.alpha_optimizer):
        assert all(state['exp_avg'].device.type=='cpu' for state in optimizer.state.values())


def test_restore_keeps_adam_history_and_uses_requested_learning_rate_and_polyak():
    torch.manual_seed(45)
    original = SACLearner(ModelConfig(), LearningConfig(), device='cpu')
    samples, scenes = transitions(), {'s': scene()}
    original.update(samples, scenes)
    saved = original.state_dict()
    requested = LearningConfig(microbatch_size=32, learning_rate=3e-5,
                               polyak=0.1, initial_alpha=2e-5)
    restored = SACLearner(ModelConfig(), requested, device='cpu')
    restored.load_state_dict(saved)
    assert restored.updates == 1
    torch.testing.assert_close(restored.log_alpha, saved['log_alpha'], atol=0, rtol=0)
    assert restored.alpha != requested.initial_alpha
    for name in ('actor', 'q1', 'q2', 'target1', 'target2'):
        for key, value in getattr(restored, name).state_dict().items():
            torch.testing.assert_close(value, saved[name][key], atol=0, rtol=0)
    optimizer_names = ('actor_optimizer', 'q1_optimizer', 'q2_optimizer', 'alpha_optimizer')
    parameters_before = {}
    for name in optimizer_names:
        optimizer = getattr(restored, name)
        state = optimizer.state_dict()
        assert all(group['lr'] == requested.learning_rate for group in state['param_groups'])
        for index, values in saved[name]['state'].items():
            for key, value in values.items():
                torch.testing.assert_close(state['state'][index][key], value, atol=0, rtol=0)
        parameters_before[name] = optimizer.param_groups[0]['params'][0].detach().clone()
    targets_before = {name: copy.deepcopy(getattr(restored, name).state_dict())
                      for name in ('target1', 'target2')}
    restored.update(samples, scenes)
    assert restored.updates == 2
    for name in optimizer_names:
        optimizer = getattr(restored, name)
        assert all(value['step'] == 2 for value in optimizer.state.values())
        group = optimizer.param_groups[0]
        parameter = group['params'][0]
        state = optimizer.state[parameter]
        # Independent Adam equation verifies the requested lr actually controls
        # the next update, with the continued (not reset) second-step moments.
        beta1, beta2 = group['betas']
        mean = state['exp_avg'] / (1 - beta1 ** 2)
        variance = state['exp_avg_sq'] / (1 - beta2 ** 2)
        expected = parameters_before[name] - requested.learning_rate * mean / (
            variance.sqrt() + group['eps'])
        torch.testing.assert_close(parameter, expected, atol=1e-7, rtol=1e-6)
    for target, source in (('target1', 'q1'), ('target2', 'q2')):
        for key, value in getattr(restored, target).state_dict().items():
            expected = targets_before[target][key] * 0.9 + getattr(restored, source).state_dict()[key] * 0.1
            torch.testing.assert_close(value, expected, atol=1e-7, rtol=1e-6)


@pytest.mark.parametrize('field,value', [('batch_size', 32), ('gamma', 0.99),
    ('target_entropy_factor', 0.02), ('maximum_alpha', 2e-4)])
def test_restore_still_rejects_changed_experience_objective(field, value):
    learner = SACLearner(ModelConfig(), LearningConfig(), device='cpu')
    saved = learner.state_dict()
    saved['learning_config'][field] = value
    with pytest.raises(ValueError, match='incompatible SAC semantics'):
        learner.load_state_dict(saved)


def test_restore_still_rejects_changed_model_shape():
    learner = SACLearner(ModelConfig(), LearningConfig(), device='cpu')
    saved = learner.state_dict()
    saved['model_config']['width'] = 64
    with pytest.raises(ValueError, match='incompatible learner model schema'):
        learner.load_state_dict(saved)


def test_v2_optimization_phases_isolate_actor_critics_and_temperature(monkeypatch):
    learner=SACLearner(ModelConfig(width=16,heads=2,layers=1),LearningConfig(microbatch_size=64))
    actor_before=[p.detach().clone() for p in learner.actor.parameters()]
    critic_after=[]; actor_grad_after=[]; phases=[]
    for name in ('q1','q2'):
        optimizer=getattr(learner,name+'_optimizer'); original=optimizer.step
        def critic_step(*args,_original=original,_name=name,**kwargs):
            for p,before in zip(learner.actor.parameters(),actor_before):
                assert torch.equal(p,before) and p.grad is None
            result=_original(*args,**kwargs); phases.append(_name)
            if _name=='q2':
                critic_after.extend(p.detach().clone() for m in (learner.q1,learner.q2) for p in m.parameters())
            return result
        monkeypatch.setattr(optimizer,'step',critic_step)
    actor_step=learner.actor_optimizer.step
    def step_actor(*args,**kwargs):
        for p,before in zip((p for m in (learner.q1,learner.q2) for p in m.parameters()),critic_after):
            assert torch.equal(p,before) and p.grad is None
        result=actor_step(*args,**kwargs)
        actor_grad_after.extend(p.grad.clone() for p in learner.actor.parameters())
        phases.append('actor'); return result
    monkeypatch.setattr(learner.actor_optimizer,'step',step_actor)
    alpha_step=learner.alpha_optimizer.step
    def step_alpha(*args,**kwargs):
        for p,before in zip(learner.actor.parameters(),actor_grad_after):
            assert torch.equal(p.grad,before)
        phases.append('alpha'); return alpha_step(*args,**kwargs)
    monkeypatch.setattr(learner.alpha_optimizer,'step',step_alpha)
    learner.update(transitions(),{'s':scene()})
    assert phases==['q1','q2','actor','alpha']
    assert all(p.grad is None for m in (learner.target1,learner.target2) for p in m.parameters())


def test_v2_nonterminal_empty_successor_rejected_and_single_action_is_one():
    learner=SACLearner(ModelConfig(width=16,heads=2,layers=1))
    sample=transitions()[0]
    assert sample.terminated and len(sample.next_observation.action_nodes)==0
    with pytest.raises(ValueError,match='nonterminal successor'):
        learner.td_targets([replace(sample,terminated=False)],{'s':scene()})
    obs=sample.observation
    single=replace(obs,action_nodes=obs.action_nodes[:1],action_yaws=obs.action_yaws[:1],goals=obs.goals[:1])
    output=learner.actor([single])[0]
    torch.testing.assert_close(output.probs,torch.ones(1),atol=0,rtol=0)
    torch.testing.assert_close(output.log_probs,torch.zeros(1),atol=0,rtol=0)


@pytest.mark.parametrize('phase',['q1','q2','actor','temperature'])
def test_nonfinite_gradients_fail_before_phase_step_and_update_counter(monkeypatch,phase):
    learner=SACLearner(ModelConfig(width=16,heads=2,layers=1),LearningConfig(microbatch_size=64))
    parameter=learner.log_alpha if phase=='temperature' else next(getattr(learner,phase).parameters())
    parameter.register_hook(lambda grad:torch.full_like(grad,float('nan')))
    optimizer=learner.alpha_optimizer if phase=='temperature' else getattr(learner,phase+'_optimizer')
    called=[]; original=optimizer.step
    monkeypatch.setattr(optimizer,'step',lambda *a,**k:(called.append(True),original(*a,**k))[-1])
    with pytest.raises(FloatingPointError,match=phase): learner.update(transitions(),{'s':scene()})
    assert not called and learner.updates==0


@pytest.mark.parametrize('phase',['q1','q2','actor','temperature'])
def test_nonfinite_parameters_after_step_fail_without_admission(monkeypatch,phase):
    learner=SACLearner(ModelConfig(width=16,heads=2,layers=1),LearningConfig(microbatch_size=64))
    parameter=learner.log_alpha if phase=='temperature' else next(getattr(learner,phase).parameters())
    optimizer=learner.alpha_optimizer if phase=='temperature' else getattr(learner,phase+'_optimizer')
    original=optimizer.step
    def corrupt(*args,**kwargs):
        result=original(*args,**kwargs)
        with torch.no_grad(): parameter.fill_(float('nan'))
        return result
    monkeypatch.setattr(optimizer,'step',corrupt)
    with pytest.raises(FloatingPointError,match=phase): learner.update(transitions(),{'s':scene()})
    assert learner.updates==0


def test_numerical_diagnostics_are_finite_and_alpha_zero_has_no_floor():
    learner=SACLearner(ModelConfig(width=16,heads=2,layers=1),LearningConfig(microbatch_size=64))
    with torch.no_grad(): learner.log_alpha.fill_(-1000.)
    metrics=learner.update(transitions(),{'s':scene()})
    for key in ('actor_grad_norm','q1_grad_norm','q2_grad_norm','temperature_grad_norm',
        'q1_selected_std','q2_selected_std','td_target_std','q1_td_error_std','q2_td_error_std'):
        assert np.isfinite(metrics[key]) and metrics[key]>=0
    assert metrics['alpha']==0.0 and metrics['alpha_position_underflow']==1.0
    assert metrics['log_alpha_position'] < -900


def test_q_and_td_quantiles_match_saved_update_samples(monkeypatch):
    learner=SACLearner(ModelConfig(width=16,heads=2,layers=1),LearningConfig(microbatch_size=64))
    saved={}; original=learner.td_targets
    def capture(samples,scenes):
        targets=original(samples,scenes); saved['targets']=targets.clone(); return targets
    monkeypatch.setattr(learner,'td_targets',capture)
    for name in ('q1','q2'):
        network=getattr(learner,name); forward=network.forward_packed
        def captured(batch,truth,_forward=forward,_name=name):
            output=_forward(batch,truth)
            if _name not in saved and torch.is_grad_enabled():
                indices=torch.as_tensor(np.cumsum((0,)+batch.action_counts[:-1])+[t.action for t in transitions()])
                saved[_name]=output[indices].detach().clone()
            return output
        monkeypatch.setattr(network,'forward_packed',captured)
    metrics=learner.update(transitions(),{'s':scene()})
    streams={'q1':saved['q1'],'q2':saved['q2'],'td_target':saved['targets'],
        'td':torch.cat(((saved['q1']-saved['targets']).abs(),(saved['q2']-saved['targets']).abs()))}
    for name,values in streams.items():
        expected=torch.quantile(values,torch.tensor([.5,.95,.99])).tolist()
        for percentile,value in zip(('p50','p95','p99'),expected):
            assert metrics[name+'_'+percentile]==pytest.approx(value,abs=1e-6)


def test_nonfinite_loss_rejected_even_if_gradient_hooks_return_finite_values(monkeypatch):
    learner=SACLearner(ModelConfig(width=16,heads=2,layers=1),LearningConfig(microbatch_size=64))
    monkeypatch.setattr(learner,'td_targets',lambda rows,scenes:torch.full((len(rows),),float('nan')))
    for network in (learner.q1,learner.q2):
        for parameter in network.parameters(): parameter.register_hook(lambda grad:torch.zeros_like(grad))
    with pytest.raises(FloatingPointError,match='q1.*loss'):
        learner.update(transitions(),{'s':scene()})
    assert learner.updates==0


@pytest.mark.parametrize('after_step',[False,True])
def test_split_heading_temperature_is_guarded(monkeypatch,after_step):
    learner=SACLearner(ModelConfig(width=16,heads=2,layers=1),
        LearningConfig(microbatch_size=64,entropy_mode='position_heading'))
    if after_step:
        original=learner.heading_alpha_optimizer.step
        def corrupt(*args,**kwargs):
            result=original(*args,**kwargs)
            with torch.no_grad(): learner.log_alpha_heading.fill_(float('nan'))
            return result
        monkeypatch.setattr(learner.heading_alpha_optimizer,'step',corrupt)
    else: learner.log_alpha_heading.register_hook(lambda grad:torch.full_like(grad,float('nan')))
    with pytest.raises(FloatingPointError,match='temperature'):
        learner.update(transitions(),{'s':scene()})
    assert learner.updates==0
