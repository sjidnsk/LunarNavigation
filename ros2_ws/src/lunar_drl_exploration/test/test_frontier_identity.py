"""Frontier meanings survive every boundary; incompatible artifacts stay untouched."""
import copy
from dataclasses import replace
import math
import pickle
import random
from pathlib import Path

import numpy as np
import pytest
import torch

from lunar_drl_exploration.checkpoint import CheckpointManager, TrainingState, UpdateBoundary
from lunar_drl_exploration.config import LearningConfig, ModelConfig, TrainingConfig, resume_semantics
from lunar_drl_exploration.evaluation import export_actor
from lunar_drl_exploration.replay import ReplayBuffer, dumps_transport, loads_transport
from lunar_drl_exploration.runtime import ActorPolicy
from lunar_drl_exploration.sac import SACLearner
from lunar_drl_exploration.schedule import UpdateSchedule
from test_replay_checkpoint import capture_state, observation, scene, transition


def config():
    return replace(TrainingConfig(), model=ModelConfig(width=16, heads=2, layers=1),
        learning=LearningConfig(learning_rate=1e-5, temperature_learning_rate=.003,
            entropy_mode='position_heading', return_steps=1, position_aux_weight=0), warmup=0)


def captured(updated=False):
    cfg = config()
    learner = SACLearner(cfg.model, cfg.learning)
    replay = ReplayBuffer(1_000_000)
    replay.add(transition(), {'s': scene()})
    schedule = UpdateSchedule(warmup=0)
    for _ in range(8): schedule.collected()
    boundary = UpdateBoundary()
    if updated:
        with boundary.update():
            learner.update(replay.sample(64, np.random.default_rng(3)), replay.scenes)
            schedule.updated()
    return learner, capture_state(learner, cfg, replay, schedule, boundary)


def test_three_new_schemas_and_complete_frontier_formula_identity():
    from lunar_drl_exploration.feature_schema import (
        OBSERVATION_SCHEMA, PRIVILEGED_SCHEMA, LEARNER_SCHEMA, FRONTIER_SEMANTICS)
    assert (OBSERVATION_SCHEMA, PRIVILEGED_SCHEMA, LEARNER_SCHEMA) == (
        'task_graph_v4', 'candidate_frontier_truth_v2', 'candidate_graph_sac_v3')
    semantic = resume_semantics(config())
    assert semantic['frontier_semantics'] == FRONTIER_SEMANTICS
    assert semantic['observation_schema'] == OBSERVATION_SCHEMA
    assert semantic['privileged_schema'] == PRIVILEGED_SCHEMA
    assert semantic['learner_schema'] == LEARNER_SCHEMA
    assert 'removed_frontiers_in/50' in semantic['reward']
    assert FRONTIER_SEMANTICS['normalization'] == dict(
        node_inside_count=100., node_outside_count=100., reward_inside_count=50.,
        critic_expected_removed_count=50.)


def test_wrong_same_shape_observation_rejected_at_constructor_transport_replay_runtime():
    obs = observation()
    with pytest.raises(ValueError, match='schema'): replace(obs, schema='task_graph_v3')
    forged = observation()
    object.__setattr__(forged, 'schema', 'wrong_27d_meaning')
    with pytest.raises(ValueError, match='schema'): loads_transport(dumps_transport(forged))
    replay = ReplayBuffer(1_000_000)
    with pytest.raises(ValueError, match='schema'): replay.add(transition(obs=forged), {'s': scene()})
    assert not len(replay)
    policy = ActorPolicy(SACLearner(config().model, config().learning).actor)
    with pytest.raises(ValueError, match='schema'): policy(forged)


@pytest.mark.parametrize('mutation', ['schema', 'reward', 'scale', 'missing'])
def test_actor_runtime_export_and_publication_reject_wrong_meaning_preserve_output(tmp_path, mutation):
    from lunar_drl_exploration.ipc import load_actor_publication
    learner = SACLearner(config().model, config().learning)
    valid = learner.actor_state()
    assert valid['schema'] == 'task_graph_v4'
    bad = copy.deepcopy(valid)
    if mutation == 'schema': bad['schema'] = 'task_graph_v3'
    elif mutation == 'missing': bad.pop('frontier_semantics')
    elif mutation == 'reward': bad['frontier_semantics']['reward_formula'] = 'delta_area/100'
    else: bad['frontier_semantics']['normalization']['critic_expected_removed_count'] = 100.
    source, target = tmp_path/'source.pt', tmp_path/'actor.pt'
    torch.save(valid, target)
    good = target.read_bytes()
    torch.save(bad, source)
    with pytest.raises(ValueError): ActorPolicy.load(source)
    with pytest.raises(ValueError): load_actor_publication(pickle.dumps(bad))
    with pytest.raises(ValueError): export_actor(source, target)
    assert target.read_bytes() == good
    ActorPolicy.load(target)


def test_legacy_19d_actor_and_area_privileged_transport_are_rejected(tmp_path):
    learner = SACLearner(config().model, config().learning)
    legacy = learner.actor_state()
    legacy['schema'] = 'task_graph_v3'
    # The former Actor encoder really had 19 feature columns.
    legacy['state_dict']['encoder.graph.input.weight'] = legacy['state_dict']['encoder.graph.input.weight'][:, :19]
    artifact = tmp_path/'legacy.pt'
    torch.save(legacy, artifact)
    before = artifact.read_bytes()
    with pytest.raises(ValueError): ActorPolicy.load(artifact)
    assert artifact.read_bytes() == before
    encoded = pickle.loads(dumps_transport(transition()))
    for node in encoded['nodes']:
        if node[0] == 'contract' and node[1] == 'PrivilegedActionContext':
            node[2]['expected_gain_m2'] = node[2].pop('removed_frontiers')
    with pytest.raises(ValueError): loads_transport(pickle.dumps(encoded))


@pytest.mark.parametrize('mutation', ['observation', 'privileged', 'learner', 'reward', 'collector', 'replay', 'replay_reward', 'replay_privileged'])
def test_checkpoint_validation_without_config_rejects_before_replacing_good(tmp_path, mutation):
    _, state = captured()
    manager = CheckpointManager(tmp_path, snapshot_max_bytes=1_000_000)
    manager.save(state)
    good = (tmp_path/'resume.pt').read_bytes()
    bad = copy.deepcopy(state.record)
    if mutation == 'observation': bad['semantics']['observation_schema'] = 'task_graph_v3'
    elif mutation == 'privileged': bad['semantics']['privileged_schema'] = 'candidate_truth_v1'
    elif mutation == 'learner': bad['learner']['schema'] = 'candidate_graph_sac_v2'
    elif mutation == 'reward': bad['semantics']['reward'] = 'delta_area/100'
    elif mutation == 'collector': bad['collector_state']['actor_record']['schema'] = 'task_graph_v3'
    elif mutation == 'replay': bad['replay'] = b'LDRLRP2\0' + bad['replay'][8:]
    elif mutation == 'replay_reward':
        replay_record = loads_transport(bad['replay'][16:])
        replay_record['frontier_semantics']['reward_formula'] = 'delta_area/100'
        bad['replay'] = bad['replay'][:16] + dumps_transport(replay_record)
    else:
        raw = pickle.loads(bad['replay'][16:])
        for node in raw['nodes']:
            if node[0] == 'contract' and node[1] == 'PrivilegedActionContext':
                node[2]['schema'] = ('literal', 'candidate_truth_v1')
        bad['replay'] = bad['replay'][:16] + pickle.dumps(raw, protocol=5)
    with pytest.raises(ValueError): TrainingState(bad).validate()
    with pytest.raises(ValueError): manager.save(TrainingState(bad))
    assert (tmp_path/'resume.pt').read_bytes() == good
    invalid = tmp_path/'invalid.pt'; torch.save(bad, invalid)
    actor = tmp_path/'actor.pt'; actor.write_bytes(b'previous-valid-file')
    with pytest.raises(ValueError): export_actor(invalid, actor)
    assert actor.read_bytes() == b'previous-valid-file'


def test_transport_and_replay_preserve_raw_integer_counts_and_normalized_features():
    obs = observation()
    features = np.array(obs.features)
    features[:, 3:11] = .13
    features[:, 11:19] = .29
    obs = replace(obs, features=features)
    sample = transition(obs=obs)
    actions = replace(sample.privileged.actions, removed_frontiers=np.array([7, 61], np.int64))
    sample = replace(sample, privileged=replace(sample.privileged, actions=actions),
                     parts=replace(sample.parts, removed_frontiers_in=7, removed_frontiers_out=61))
    transported = loads_transport(dumps_transport(sample))
    replay = ReplayBuffer(1_000_000); replay.add(transported, {'s':scene()})
    restored = ReplayBuffer.from_snapshot(replay.snapshot(100_000))
    actual = restored.sample(1, np.random.default_rng(1))[0]
    np.testing.assert_array_equal(actual.observation.features, obs.features)
    np.testing.assert_array_equal(actual.privileged.actions.removed_frontiers, [7, 61])
    assert actual.privileged.actions.removed_frontiers.dtype == np.int64
    assert actual.parts.removed_frontiers_in == 7 and actual.parts.removed_frontiers_out == 61
    assert not actual.privileged.actions.removed_frontiers.flags.writeable
    raw = pickle.loads(dumps_transport(sample)); raw.pop('frontier_semantics', None)
    with pytest.raises(ValueError): loads_transport(pickle.dumps(raw))


def test_new_replay_rejects_wrong_same_shape_reward_identity():
    replay = ReplayBuffer(1_000_000); replay.add(transition(), {'s':scene()})
    saved = replay.snapshot(100_000)
    record = loads_transport(saved[16:])
    record['frontier_semantics']['reward_formula'] = 'delta_area/100'
    bad = saved[:16] + dumps_transport(record)
    with pytest.raises(ValueError): ReplayBuffer.from_snapshot(bad)


def equal_state(left, right):
    if isinstance(left, torch.Tensor): torch.testing.assert_close(left, right, rtol=0, atol=0)
    elif isinstance(left, np.ndarray): np.testing.assert_array_equal(left, right)
    elif isinstance(left, dict):
        assert left.keys() == right.keys()
        for key in left: equal_state(left[key], right[key])
    elif isinstance(left, (tuple, list)):
        assert len(left) == len(right)
        for a, b in zip(left, right): equal_state(a, b)
    else: assert left == right


def test_actual_split_sac_update_full_resume_and_actor_export_equal(tmp_path):
    cfg = config()
    learner, saved = captured(updated=True)
    assert all(getattr(learner, name).state for name in (
        'actor_optimizer', 'q1_optimizer', 'q2_optimizer', 'alpha_optimizer', 'heading_alpha_optimizer'))
    random.seed(41); np.random.seed(42); torch.manual_seed(43)
    saved.record['rng'].update(python=random.getstate(), numpy=dumps_transport(np.random.get_state()),
                              torch_cpu=torch.get_rng_state())
    manager = CheckpointManager(tmp_path, snapshot_max_bytes=1_000_000)
    checkpoint = manager.save(saved)
    expected_random = (random.random(), np.random.random(), torch.rand(3))
    expected_state = copy.deepcopy(learner.state_dict())
    restored = SACLearner(cfg.model, cfg.learning)
    result = manager.load(cfg).restore(restored, cfg)
    equal_state(restored.state_dict(), expected_state)
    equal_state((random.random(), np.random.random(), torch.rand(3)), expected_random)
    equal_state(result.schedule.state_dict(), saved.record['schedule'])
    equal_state(result.collector_state, saved.record['collector_state'])
    equal_state(result.counters, loads_transport(saved.record['counters']))
    equal_state(result.curriculum, loads_transport(saved.record['curriculum']))
    equal_state(result.replay_rng.bit_generator.state, loads_transport(saved.record['rng']['replay']))
    equal_state(result.curriculum_rng.bit_generator.state, loads_transport(saved.record['rng']['curriculum']))
    assert result.replay.snapshot(1_000_000, require_all=True) == saved.record['replay']
    artifact = tmp_path/'actor.pt'; export_actor(checkpoint, artifact)
    policy = ActorPolicy.load(artifact)
    with torch.inference_mode():
        equal_state(policy.actor([observation()])[0].probs, learner.actor([observation()])[0].probs)
    assert policy(observation()) == int(learner.actor([observation()])[0].probs.argmax())


def test_fresh_construction_never_reads_source_checkpoint_and_starts_all_state_zero(monkeypatch):
    def forbidden_load(*args, **kwargs): raise AssertionError('fresh init read checkpoint')
    monkeypatch.setattr(torch, 'load', forbidden_load)
    cfg = config()
    random.seed(cfg.seed); np.random.seed(cfg.seed); torch.manual_seed(cfg.seed)
    learner = SACLearner(cfg.model, cfg.learning)
    assert learner.updates == 0
    for q, target in ((learner.q1, learner.target1), (learner.q2, learner.target2)):
        equal_state(q.state_dict(), target.state_dict())
        assert {p.data_ptr() for p in q.parameters()}.isdisjoint(p.data_ptr() for p in target.parameters())
    for name in ('actor_optimizer', 'q1_optimizer', 'q2_optimizer', 'alpha_optimizer', 'heading_alpha_optimizer'):
        assert not getattr(learner, name).state
    assert learner.log_alpha.item() == pytest.approx(math.log(5e-5))
    assert learner.log_alpha_heading.item() == learner.log_alpha.item()
    replay = ReplayBuffer(cfg.replay_max_bytes)
    schedule = UpdateSchedule(cfg.warmup, cfg.update_ratio, cfg.max_update_credit)
    assert len(replay) == 0 and replay.bytes_used == 0
    assert schedule.transitions == schedule.updates == schedule.inflight == schedule.credit == 0


def test_fresh_runner_uses_zero_state_and_seeded_rng_without_checkpoint_read(tmp_path, monkeypatch):
    # ControlledEnv is a deterministic test double; no scene training job is launched.
    from lunar_drl_exploration import training
    cfg = replace(TrainingConfig(), environments=1, warmup=1024, paired_scene_sequence=True,
                  output_dir=str(tmp_path), system_reserve_bytes=0)
    original = training.initial_training_identity
    seen = []
    def inspect(learner, **kwargs):
        assert learner.updates == 0
        assert not len(kwargs['replay'])
        assert kwargs['schedule'].transitions == kwargs['schedule'].updates == 0
        assert kwargs['counters'] == dict(transitions=0, aborted=0, recoveries=0)
        assert kwargs['curriculum_state'] is None
        assert kwargs['source_replay_sha256'] is None
        for key, seed in (('replay_rng', cfg.seed), ('curriculum_rng', cfg.seed+1)):
            equal_state(kwargs[key].bit_generator.state, np.random.default_rng(seed).bit_generator.state)
        equal_state(kwargs['collector_state']['policy_rng'],
                    torch.Generator().manual_seed(cfg.seed+2).get_state())
        assert kwargs['collector_state']['actor_version'] == 0
        for q, target in ((learner.q1, learner.target1), (learner.q2, learner.target2)):
            equal_state(q.state_dict(), target.state_dict())
        for name in ('actor_optimizer', 'q1_optimizer', 'q2_optimizer', 'alpha_optimizer'):
            assert not getattr(learner, name).state
        seen.append(True)
        return original(learner, **kwargs)
    original_load = torch.load
    def forbid_checkpoint_file(source, *args, **kwargs):
        if isinstance(source, (str, Path)):
            raise AssertionError('fresh runner read checkpoint file')
        # Torch tensor unpickling over owned IPC uses in-memory BytesIO storage.
        return original_load(source, *args, **kwargs)
    monkeypatch.setattr(torch, 'load', forbid_checkpoint_file)
    monkeypatch.setattr(training, 'initial_training_identity', inspect)
    training.run_training(cfg, max_transitions=1, device='cpu',
        env_factory='worker_fixtures:ControlledEnv', probe_extent=40, probe_budget=3)
    assert seen == [True]
