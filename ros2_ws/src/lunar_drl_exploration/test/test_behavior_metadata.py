"""Behavior metadata tests use real sampling and IPC, without ROS simulation."""
from dataclasses import replace
import math
import multiprocessing as mp
from pathlib import Path
import pickle
import time

import pytest

from worker_fixtures import ControlledEnv


@pytest.mark.parametrize('scale', [1., 2.**-100, 2.**100])
def test_sampled_joint_probability_is_normalized_without_extra_rng(scale):
    import torch
    from lunar_drl_exploration.collector import sample_behavior_action
    weights = torch.tensor([scale, 0., 3.*scale], dtype=torch.float32)
    actual = torch.Generator().manual_seed(20260927)
    reference = torch.Generator().manual_seed(20260927)
    for _ in range(24):
        expected_action = int(torch.multinomial(weights, 1, generator=reference))
        action, log_mu = sample_behavior_action(weights, actual)
        assert action == expected_action
        assert log_mu == pytest.approx(math.log({0: .25, 2: .75}[action]), abs=2e-14)
        assert torch.equal(actual.get_state(), reference.get_state())


def prepared_env(seed=1):
    env = ControlledEnv(None, 1)
    env.reset(seed, 'moon', 40., episode_budget=3)
    return env


def test_worker_metadata_uses_real_zero_based_step_and_preserves_legacy_unknown():
    from lunar_drl_exploration.worker import step_with_behavior_metadata
    env = prepared_env()
    command = dict(action=1, actor_version=7, behavior_log_prob=math.log(.25))
    first = step_with_behavior_metadata(env, command)
    second = step_with_behavior_metadata(env, dict(command, action=0, actor_version=9,
                                                  behavior_log_prob=math.log(.75)))
    assert (first.step_index, second.step_index) == (0, 1)
    assert first.episode_id == second.episode_id == env.episode_id
    assert first.behavior_log_prob == math.log(.25)
    assert second.behavior_log_prob == math.log(.75)
    env.reset(2, 'moon', 40., episode_budget=3)
    legacy = step_with_behavior_metadata(env, dict(action=0, actor_version=11))
    assert legacy.episode_id != first.episode_id
    assert legacy.step_index is None and legacy.behavior_log_prob is None


@pytest.mark.parametrize('value', [None, True, '0', float('nan'), float('inf'), -float('inf'), .01])
def test_invalid_provided_probability_is_rejected_before_execution(value):
    from lunar_drl_exploration.worker import step_with_behavior_metadata
    env = prepared_env()
    with pytest.raises(ValueError):
        step_with_behavior_metadata(env, dict(action=0, actor_version=7, behavior_log_prob=value))
    assert env.steps == 0


@pytest.mark.parametrize('value', [None, True, -1, .5])
def test_invalid_real_step_is_rejected_before_execution(value):
    from lunar_drl_exploration.worker import step_with_behavior_metadata
    env = prepared_env()
    env.steps = value
    with pytest.raises(ValueError):
        step_with_behavior_metadata(env, dict(action=0, actor_version=7, behavior_log_prob=0.))


@pytest.mark.parametrize('mismatch', ['action', 'version', 'episode', 'live_episode', 'step'])
def test_completed_transition_must_match_dispatched_action_and_episode(mismatch):
    from lunar_drl_exploration.worker import step_with_behavior_metadata
    env = prepared_env()
    original = env.step
    def incorrect_step(action, *, actor_version):
        transition = original(action, actor_version=actor_version)
        if mismatch == 'action':return replace(transition, action=1)
        if mismatch == 'version':return replace(transition, actor_version=actor_version+1)
        if mismatch == 'episode':return replace(transition, episode_id='another-episode')
        if mismatch == 'live_episode':env.episode_id = 'another-episode'
        if mismatch == 'step':env.steps += 1
        return transition
    env.step = incorrect_step
    with pytest.raises(ValueError):
        step_with_behavior_metadata(env, dict(action=0, actor_version=7, behavior_log_prob=0.))


def receive(pipe, kind, timeout=12.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if pipe.poll(.05):
            message = pipe.recv()
            if message['kind'] == kind:return message
            if message['kind'] in ('ERROR', 'FATAL'):pytest.fail(str(message))
    raise AssertionError('missing '+kind)


def close_child(process, pipe):
    """Failure cleanup must ACK retained completions before requesting exit."""
    if process.is_alive():
        try:pipe.send(dict(kind='STOP'))
        except (EOFError, BrokenPipeError):pass
        deadline=time.monotonic()+5.
        while process.is_alive() and time.monotonic()<deadline:
            if pipe.poll(.05):
                try:message=pipe.recv()
                except EOFError:break
                if message['kind']=='DONE':
                    pipe.send(dict(kind='ACK',env=message.get('env'),token=message['token']))
            process.join(.01)
    if process.is_alive():process.kill()
    process.join(5);pipe.close()


def test_worker_retry_is_identical_one_admission_and_cancel_has_no_transition():
    from lunar_drl_exploration.config import TrainingConfig, config_record
    from lunar_drl_exploration.replay import loads_transport, ReplayBuffer
    from lunar_drl_exploration.schedule import UpdateSchedule
    from lunar_drl_exploration.training import AdmissionLedger
    from lunar_drl_exploration.worker import worker_main
    context = mp.get_context('spawn')
    parent, child = context.Pipe()
    process = context.Process(target=worker_main, args=(child, config_record(TrainingConfig()), 1),
                              kwargs={'env_factory': 'worker_fixtures:ControlledEnv'})
    process.start();child.close()
    try:
        assert receive(parent, 'HELLO')['torch_loaded'] is False
        parent.send(dict(kind='RESET', spec=dict(seed=1, family='moon', extent=40, episode_budget=3)))
        scene = loads_transport(receive(parent, 'SCENE')['payload'])
        receive(parent, 'READY')
        parent.send(dict(kind='STEP', token=11, action=1, actor_version=7, behavior_log_prob=math.log(.25)))
        done = receive(parent, 'DONE')
        retry = receive(parent, 'DONE', timeout=4.)
        assert retry == done
        transition = loads_transport(done['payload'])
        assert transition.step_index == 0 and transition.behavior_log_prob == math.log(.25)
        assert transition.action == 1 and transition.actor_version == 7
        ledger = AdmissionLedger(ReplayBuffer(), UpdateSchedule(warmup=0))
        ledger.scene(1, scene);ledger.schedule.reserve_dispatch()
        assert ledger.admit(1, 11, transition)
        assert not ledger.admit(1, 11, loads_transport(retry['payload']))
        assert len(ledger.replay) == ledger.schedule.transitions == 1
        assert ledger.schedule.credit == .25
        parent.send(dict(kind='ACK', token=11));receive(parent, 'READY')
        parent.send(dict(kind='RESET', spec=dict(seed=-1, family='moon', extent=40, episode_budget=3)))
        receive(parent, 'SCENE');receive(parent, 'READY')
        parent.send(dict(kind='STEP', token=12, action=0, actor_version=9, behavior_log_prob=math.log(.75)))
        parent.send(dict(kind='STOP'))
        messages=[];deadline=time.monotonic()+8.
        while time.monotonic()<deadline:
            if not parent.poll(.05):continue
            message=parent.recv();messages.append(message)
            assert message['kind']!='DONE', 'canceled action became a finished transition'
            if message['kind']=='CLOSED':break
        assert any(m['kind']=='ABORTED' and m['token']==12 for m in messages)
        assert messages[-1]['kind']=='CLOSED'
        process.join(5)
        assert process.exitcode == 0
    finally:
        close_child(process,parent)


class PublicationGateEnv(ControlledEnv):
    """Hold the first external action while the real collector receives PUBLISH."""
    def __init__(self, config, env_id, **kwargs):
        super().__init__(config, env_id, **kwargs)
        self.release_path = Path(config.output_dir)/'release-action'
        self.waiting = False

    def step(self, *args, **kwargs):
        if self.steps == 0:
            self.waiting = True
            deadline = time.monotonic()+10.
            while not self.release_path.exists():
                if self.stop.wait(.01):
                    class TaskCanceled(RuntimeError):pass
                    raise TaskCanceled('cancel gated action')
                if time.monotonic() > deadline:raise RuntimeError('publication gate timed out')
            self.waiting = False
        return super().step(*args, **kwargs)

    def progress(self):
        return dict(super().progress(), behavior_test_waiting=self.waiting)


def test_collector_preserves_sample_probability_across_inflight_publication(tmp_path):
    import torch
    from lunar_drl_exploration.collector import collector_main
    from lunar_drl_exploration.config import TrainingConfig, ModelConfig, config_record
    from lunar_drl_exploration.model import Actor
    from lunar_drl_exploration.replay import loads_transport
    from worker_fixtures import observation
    torch.set_num_threads(1)
    torch.manual_seed(123)
    model_config = ModelConfig(width=16, heads=4, layers=1)
    actor = Actor(model_config).eval()
    with torch.inference_mode():old_weights = actor([observation()])[0].probs.clone()
    old_record = dict(schema='task_graph_v3', model_config=config_record(model_config), version=7,
                      state_dict={key:value.clone() for key,value in actor.state_dict().items()})
    with torch.no_grad():actor.head[-1].weight.mul_(-4.)
    with torch.inference_mode():new_weights = actor([observation()])[0].probs.clone()
    publication = dict(old_record, version=16, state_dict=actor.state_dict())
    generator = torch.Generator().manual_seed(77)
    state = pickle.dumps(dict(actor_record=old_record, actor_version=7, policy_rng=generator.get_state()))
    expected = []
    for weights in (old_weights, new_weights):
        action = int(torch.multinomial(weights, 1, generator=generator))
        expected.append((action, math.log(float(weights[action]))-math.log(float(weights.double().sum()))))
    config = replace(TrainingConfig(), environments=1, collector_threads=1, output_dir=str(tmp_path))
    parent, child = mp.get_context('spawn').Pipe()
    process = mp.get_context('spawn').Process(target=collector_main,
        args=(child, config_record(config), state),
        kwargs={'env_factory': 'test_behavior_metadata:PublicationGateEnv'})
    process.start();child.close()
    completed=[];published=False;stopped=None;grants=0
    try:
        deadline=time.monotonic()+25.
        while stopped is None and time.monotonic()<deadline:
            if not parent.poll(.1):continue
            message=parent.recv();kind=message['kind']
            if kind=='NEED_RESET':
                parent.send(dict(kind='RESET',env=0,spec=dict(seed=1,family='moon',extent=40.,episode_budget=10)))
            elif kind=='RESERVE' and grants<2:
                grants+=1;parent.send(dict(kind='GRANT',env=0,token=message['token']))
            elif kind=='STATUS' and message['progress'].get('behavior_test_waiting') and not published:
                published=True;parent.send(dict(kind='PUBLISH',payload=pickle.dumps(publication)))
            elif kind=='PUBLISHED':
                assert message['version']==16
                (tmp_path/'release-action').write_text('release')
            elif kind=='DONE':
                completed.append(loads_transport(message['payload']))
                parent.send(dict(kind='ACK',env=0,token=message['token']))
                if len(completed)==2:parent.send(dict(kind='STOP'))
            elif kind=='STOPPED':stopped=pickle.loads(message['payload'])
            elif kind=='FATAL':pytest.fail(message['reason'])
        assert stopped is not None and published and len(completed)==2
        assert [t.actor_version for t in completed]==[7,16]
        assert [t.step_index for t in completed]==[0,1]
        assert completed[0].episode_id==completed[1].episode_id
        for transition,(action,log_mu) in zip(completed,expected):
            assert transition.action==action
            assert transition.behavior_log_prob==pytest.approx(log_mu,abs=1e-7)
        assert torch.equal(stopped['policy_rng'],generator.get_state())
        process.join(5);assert process.exitcode==0
    finally:
        close_child(process,parent)
