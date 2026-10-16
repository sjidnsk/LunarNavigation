"""Real collector/worker and optimizer boundaries; no formal scene training."""
from dataclasses import replace
from fractions import Fraction
import multiprocessing as mp
import os
import signal
import threading

import numpy as np
import pytest
import torch

from lunar_drl_exploration.checkpoint import CheckpointManager, TrainingState, UpdateBoundary
from lunar_drl_exploration.config import TrainingConfig, config_record, training_config_from_record
from lunar_drl_exploration.replay import ReplayBuffer, loads_transport, snapshot_sample_count
from lunar_drl_exploration.sac import SACLearner
from lunar_drl_exploration.schedule import UpdateSchedule
from lunar_drl_exploration.training import AdmissionLedger, run_training


def config_for(path, **kwargs):
    base = TrainingConfig()
    return replace(base, output_dir=str(path), environments=2, warmup=0,
        system_reserve_bytes=0, save_interval_s=1800,
        learning=replace(base.learning, microbatch_size=64), **kwargs)


def seed_debt(config, count):
    """Validated recovery initial state: actual ledger admissions, no optimizer yet."""
    from worker_fixtures import ControlledEnv
    torch.set_num_threads(2)
    learner = SACLearner(config.model, config.learning, device='cpu')
    replay = ReplayBuffer(config.replay_max_bytes)
    schedule = UpdateSchedule(config.warmup, config.update_ratio, config.max_update_credit)
    ledger = AdmissionLedger(replay, schedule)
    env = ControlledEnv(config, 1)
    env.reset(7, 'moon', 40, episode_budget=512)
    ledger.scene(0, env.static_scenes)
    for token in range(1, count + 1):
        schedule.reserve_dispatch()
        assert ledger.admit(0, token, env.step(1, actor_version=0))
    state = TrainingState.capture(learner=learner, config=config, replay=replay,
        schedule=schedule, boundary=UpdateBoundary(), replay_rng=np.random.default_rng(3),
        curriculum_rng=np.random.default_rng(4), curriculum={'episodes_issued': 1},
        collector_state=dict(actor_record=learner.actor_state(), actor_version=0,
            policy_rng=torch.Generator().manual_seed(5).get_state(), sequences={0: count},
            acknowledged={0: count}),
        counters=dict(transitions=count, aborted=0, recoveries=0, last_admitted={0: count}))
    CheckpointManager(config.output_dir).save(state)


def run(config, **kwargs):
    return run_training(config, device='cpu', env_factory='worker_fixtures:ControlledEnv',
        probe_extent=40, probe_budget=512, **kwargs)


def assert_boundary(config, transitions, updates, credit=0):
    saved = CheckpointManager(config.output_dir).load(config).record
    assert saved['schedule']['transitions'] == transitions
    assert snapshot_sample_count(saved['replay']) == transitions
    assert saved['schedule']['updates'] == saved['learner']['updates'] == updates
    assert Fraction(saved['schedule']['credit']) == credit
    assert saved['schedule']['unfinished_reservations'] == 0
    assert loads_transport(saved['counters'])['transitions'] == transitions
    assert saved['collector_state']['actor_version'] == updates
    assert saved['collector_state']['actor_record']['version'] == updates
    for key, value in saved['learner']['actor'].items():
        torch.testing.assert_close(value, saved['collector_state']['actor_record']['state_dict'][key], rtol=0, atol=0)
    assert saved['collector_state']['acknowledged'] == loads_transport(saved['counters'])['last_admitted']
    assert not mp.active_children()
    assert not [t for t in threading.enumerate() if t.name.startswith('drl-ipc-')]
    return saved


def test_exact_two_stages_drain_and_final_publication(tmp_path, monkeypatch):
    from lunar_drl_exploration.ipc import Duplex
    config = replace(config_for(tmp_path), warmup=4)
    commands = []
    original = Duplex.send
    def send(self, message):
        commands.append(message['kind'])
        return original(self, message)
    monkeypatch.setattr(Duplex, 'send', send)
    for stage, maximum in ((1, 12), (2, 8)):
        commands.clear()
        result = run(config, resume=stage > 1, max_transitions=maximum)
        assert result['new_transitions'] == maximum
        assert_boundary(config, 4 + stage * 8, stage * 2)
        assert commands[-3:] == ['PUBLISH', 'BARRIER', 'STOP']
        assert commands.count('GRANT') == maximum


@pytest.mark.parametrize('count', [1, 4, 128])
def test_zero_admission_resume_drains_integral_debt_only(tmp_path, monkeypatch, count):
    from lunar_drl_exploration.ipc import Duplex
    config = config_for(tmp_path)
    seed_debt(config, count)
    commands = []
    original = Duplex.send
    def send(self, message):
        commands.append(message['kind'])
        return original(self, message)
    monkeypatch.setattr(Duplex, 'send', send)
    result = run(config, resume=True, max_transitions=0)
    assert result['new_transitions'] == 0
    assert 'GRANT' not in commands
    assert_boundary(config, count, count // 4, Fraction(count % 4, 4))
    if count == 1:
        run(config, resume=True, max_transitions=3)
        assert_boundary(config, 4, 1)


def test_production_stage_arithmetic_without_formal_training():
    schedule = UpdateSchedule()
    for stage in range(1, 11):
        for _ in range(9216 if stage == 1 else 8192):
            schedule.reserve_dispatch()
            schedule.collected(reserved=True)
            if schedule.can_update:
                schedule.updated()
        assert (schedule.transitions, schedule.updates, schedule.credit) == (1024 + stage*8192, stage*2048, 0)


def gated_completion_collector(child, record, state, ready, count, **kwargs):
    """Withhold first DONE transport until all eight slots have finished."""
    from lunar_drl_exploration.collector import collector_main
    class GatePipe:
        pending = {}
        def send(self, message):
            if message['kind'] == 'DONE' and not ready.is_set():
                # Retries do not count as another worker. Buffer only DONE so
                # later SCENE/READY messages can still initialize the eighth slot.
                self.pending[message['env']] = message
                with count.get_lock():
                    count.value = len(self.pending)
                if len(self.pending) < 8:
                    return
                ready.set()
                for completed in self.pending.values():
                    child.send(completed)
                self.pending.clear()
                return
            return child.send(message)
        def __getattr__(self, name): return getattr(child, name)
    collector_main(GatePipe(), record, state, **kwargs)


def test_all_eight_reservations_and_projected_credit_stay_bounded(tmp_path, monkeypatch):
    from functools import partial
    from lunar_drl_exploration import collector
    config = replace(config_for(tmp_path), environments=8, warmup=4)
    context = mp.get_context('spawn')
    ready, count = context.Event(), context.Value('i', 0)
    monkeypatch.setattr(collector, 'collector_main', partial(gated_completion_collector, ready=ready, count=count))
    inflight, projected = [], []
    original = UpdateSchedule.reserve_dispatch
    def reserve(self, count=1):
        original(self, count)
        inflight.append(self.inflight)
        projected.append(max(0, self.transitions + self.inflight - self.warmup) * self.ratio - self.updates)
    monkeypatch.setattr(UpdateSchedule, 'reserve_dispatch', reserve)
    result = run(config, max_transitions=12)
    assert ready.is_set() and max(inflight) == 8
    assert max(projected) <= 32 and all(value >= 0 for value in projected)
    assert result['new_transitions'] == 12
    assert_boundary(config, 12, 2)


def signal_boundary_child(record, boundary, handshake):
    """Parent sends actual SIGINT only after an explicit update/IPC handshake."""
    from lunar_drl_exploration.ipc import Duplex
    config = training_config_from_record(record)
    fired = False
    accounted = 0
    events = []
    def gate(where):
        nonlocal fired
        if boundary == where and not fired:
            fired = True
            handshake.send(dict(boundary=where, accounted=accounted))
            # Spawned fixtures also run with production PYTHONOPTIMIZE=1.
            # Blocking protocol operations cannot live inside stripped asserts.
            if not handshake.poll(20):
                raise AssertionError('parent must release signal boundary')
            release = handshake.recv()
            if release != 'release':
                raise AssertionError('unexpected signal boundary release')
    original_update = SACLearner.update
    original_updated = UpdateSchedule.updated
    original_send, original_recv, original_poll = Duplex.send, Duplex.recv, Duplex.poll
    def update(self, *args, **kwargs):
        gate('before_update')
        result = original_update(self, *args, **kwargs)
        gate('after_update')
        return result
    def updated(self):
        nonlocal accounted
        original_updated(self)
        accounted = self.updates
        gate('after_account')
    def send(self, message):
        if message['kind'] == 'PUBLISH': gate('before_publish')
        events.append(message['kind'])
        original_send(self, message)
        if message['kind'] == 'PUBLISH': gate('after_publish')
    def recv(self):
        message = original_recv(self)
        if message['kind'] == 'PUBLISHED': gate('after_published')
        return message
    def poll(self, timeout=0):
        gate('before_loop')
        return original_poll(self, timeout)
    SACLearner.update, UpdateSchedule.updated = update, updated
    Duplex.send, Duplex.recv, Duplex.poll = send, recv, poll
    result = run(config, resume=True, max_transitions=0)
    handshake.send(dict(result=result, events=events, children=[p.pid for p in mp.active_children()]))


@pytest.mark.parametrize('count,boundary', [
    (1, 'before_loop'), (4, 'before_loop'), (128, 'before_loop'),
    (4, 'before_update'), (128, 'before_update'),
    (4, 'after_update'), (128, 'after_update'),
    (4, 'after_account'), (128, 'after_account'),
    (4, 'before_publish'), (128, 'before_publish'),
    (4, 'after_publish'), (128, 'after_publish'),
    (4, 'after_published'), (128, 'after_published'),
])
def test_sigint_atomic_update_and_publication_recovery(tmp_path, count, boundary):
    config = replace(config_for(tmp_path), actor_publish_updates=1)
    seed_debt(config, count)
    context = mp.get_context('spawn')
    parent, child = context.Pipe()
    process = context.Process(target=signal_boundary_child, args=(config_record(config), boundary, child))
    process.start(); child.close()
    try:
        assert parent.poll(30), 'learner failed to reach deterministic signal gate'
        gate = parent.recv()
        assert gate['boundary'] == boundary
        os.kill(process.pid, signal.SIGINT)
        parent.send('release')
        assert parent.poll(45), 'signal did not produce bounded orderly exit'
        evidence = parent.recv()
        process.join(10)
        assert process.exitcode == 0 and evidence['children'] == []
        assert 'GRANT' not in evidence['events']
        completed = (0 if boundary == 'before_loop' else gate['accounted']
            if boundary == 'after_published' else 1)
        saved = assert_boundary(config, count, completed, Fraction(count, 4) - completed)
        original_replay = saved['replay']
        result = run(config, resume=True, max_transitions=0)
        assert result['new_transitions'] == 0
        restored = assert_boundary(config, count, count // 4, Fraction(count % 4, 4))
        assert restored['replay'] == original_replay
    finally:
        if process.is_alive(): process.kill(); process.join(5)
        parent.close()


def test_positive_resume_pays_old_debt_before_first_grant(tmp_path, monkeypatch):
    from lunar_drl_exploration.ipc import Duplex
    config = config_for(tmp_path)
    seed_debt(config, 4)
    events = []
    original_updated, original_send = UpdateSchedule.updated, Duplex.send
    def updated(self):
        original_updated(self)
        events.append(('update', self.updates))
    def send(self, message):
        events.append((message['kind'], None))
        return original_send(self, message)
    monkeypatch.setattr(UpdateSchedule, 'updated', updated)
    monkeypatch.setattr(Duplex, 'send', send)
    run(config, resume=True, max_transitions=4)
    assert events.index(('update', 1)) < events.index(('GRANT', None))
    assert_boundary(config, 8, 2)


@pytest.mark.parametrize('missing', ['PUBLISHED', 'BARRIER'])
def test_missing_final_confirmation_is_bounded_and_preserves_checkpoint(tmp_path, monkeypatch, missing):
    import hashlib
    import time
    from lunar_drl_exploration import ipc, training
    config = config_for(tmp_path)
    seed_debt(config, 4)
    path = tmp_path / 'resume.pt'
    before = hashlib.sha256(path.read_bytes()).hexdigest()
    omitted = threading.Event()
    commands = []
    real_duplex = ipc.Duplex
    class MissingFrame:
        def __init__(self, raw): self.raw = raw
        def recv(self):
            while True:
                message = self.raw.recv()
                if message['kind'] != missing: return message
                omitted.set()
        def __getattr__(self, name): return getattr(self.raw, name)
    class MissingDuplex(real_duplex):
        def __init__(self, raw, *args, **kwargs):
            super().__init__(MissingFrame(raw), *args, **kwargs)
        def send(self, message):
            # STOP is only allowed by failure cleanup after confirmation timeout.
            if message['kind'] == 'STOP': assert omitted.is_set()
            commands.append(message['kind'])
            return super().send(message)
    monkeypatch.setattr(ipc, 'Duplex', MissingDuplex)
    monkeypatch.setattr(training, 'FINAL_CONFIRM_TIMEOUT_S', 5.)
    started = time.monotonic()
    with pytest.raises(RuntimeError, match=f'final {missing} confirmation timed out'):
        run(config, resume=True, max_transitions=0)
    assert omitted.is_set() and time.monotonic() - started < 15
    assert hashlib.sha256(path.read_bytes()).hexdigest() == before
    if missing == 'PUBLISHED': assert 'BARRIER' not in commands
    assert not mp.active_children()
    assert not [t for t in threading.enumerate() if t.name.startswith('drl-ipc-')]


def test_terminal_zero_target_reports_zero_new_admissions(tmp_path, capsys):
    import json
    config = config_for(tmp_path)
    seed_debt(config, 4)
    run(config, resume=True, max_transitions=0)
    assert '本次采集 0 / 0' in capsys.readouterr().out
    final = json.loads((tmp_path / 'run.json').read_text())['observations']['final']
    assert final['new_transitions'] == final['overshoot'] == final['inflight'] == final['credit'] == 0


@pytest.mark.parametrize('resume,budget', [(False, 0), (False, -1), (True, -1)])
def test_api_rejects_invalid_admission_budget_before_artifacts(tmp_path, resume, budget):
    config = config_for(tmp_path / 'uncreated')
    with pytest.raises(ValueError, match='max_transitions'):
        run(config, resume=resume, max_transitions=budget)
    assert not (tmp_path / 'uncreated').exists()
