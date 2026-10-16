"""The v5 training boundary retains full experience and rejects legacy mixing."""
from dataclasses import replace
from types import SimpleNamespace
import hashlib
import pickle
import numpy as np
import pytest
import torch

from lunar_drl_exploration.config import TrainingConfig, LearningConfig, resume_semantics
from lunar_drl_exploration.contracts import Pose, SensorSpec, Transition, RewardParts
from lunar_drl_exploration.replay import ReplayBuffer, _same_content, snapshot_sample_count
from lunar_drl_exploration.sac import SACLearner
from lunar_drl_exploration.checkpoint import UpdateBoundary, TrainingState
from lunar_drl_exploration.schedule import UpdateSchedule
from test_candidate_model import model_config, observed
from test_candidate_observation import fine_observation
from test_model import scene, state
from test_replay_checkpoint import capture_state, transition as legacy_transition, scene as legacy_scene


def samples():
    obs = observed()
    empty = replace(obs, action_pairs=np.empty((0, 2), int), goals=np.empty((0, 3)))
    result = []
    for i in range(64):
        nxt = empty if i % 5 == 0 else obs
        result.append(Transition(obs, i % len(obs.goals), (i-30)/100., nxt,
            state(obs=obs), state(full=bool(i % 2), obs=nxt), RewardParts(0, 0, 0, 0, 0),
            i % 5 == 0, i % 7 == 0 and i % 5 != 0, f'episode-{i}', 0,
            step_index=0, behavior_log_prob=-np.log(len(obs.goals))))
    return result


def test_environment_converts_original_fine_graph_before_privileged_query():
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    env = object.__new__(RosExplorationEnv)
    env.config = replace(TrainingConfig(), model=model_config(), policy_graph_mode='a2')
    env.sensor = SensorSpec(range_m=20.)
    env.task = SimpleNamespace(task_id='task')
    snap = SimpleNamespace(pose=Pose(0., 0., 0.), profile_hash='wheel-profile', resolution_m=.2)
    raw = fine_observation()
    obs, fine, _, stats = env._prepare_policy_state(raw, snapshot=snap)
    assert obs.schema == 'task_graph_v5' and fine.schema == 'task_graph_v4'
    np.testing.assert_array_equal(obs.goals, fine.goals)
    assert obs.metadata['map_resolution_m'] == .2
    assert obs.metadata['platform_profile_hash'] == 'wheel-profile'
    assert stats['virtual_nodes'] == 1
    assert resume_semantics(env.config)['observation_schema'] == 'task_graph_v5'
    assert resume_semantics(env.config) != resume_semantics(replace(env.config, model=TrainingConfig().model))


def test_sac_update_publication_and_complete_disk_roundtrip(tmp_path):
    from lunar_drl_exploration.ipc import load_actor_publication
    torch.manual_seed(19)
    config = replace(TrainingConfig(), model=model_config(),
        learning=LearningConfig(entropy_mode='position_heading'), warmup=0, policy_graph_mode='a2',
        snapshot_max_bytes=64*1024**2, replay_max_bytes=64*1024**2)
    learner = SACLearner(config.model, config.learning)
    batch = samples(); scenes = {'s': scene()}
    replay = ReplayBuffer(config.replay_max_bytes); schedule = UpdateSchedule(warmup=0)
    for transition in batch:
        replay.add(transition, scenes); schedule.collected()
    # All five networks own separate weights; targets are reference copies only.
    storages = [{p.data_ptr() for p in network.parameters()} for network in
        (learner.actor, learner.q1, learner.q2, learner.target1, learner.target2)]
    for i, left in enumerate(storages):
        for right in storages[i+1:]: assert left.isdisjoint(right)
    for network in (learner.target1, learner.target2):
        assert all(not p.requires_grad for p in network.parameters())
    initial = {key: value.clone() for key, value in learner.actor.state_dict().items()}
    metrics = learner.update(batch, scenes); schedule.updated()
    assert metrics['updates'] == 1 and np.isfinite(metrics['actor_loss'])
    assert any(not torch.equal(initial[k], v) for k, v in learner.actor.state_dict().items())
    publication = learner.actor_state()
    assert publication['schema'] == 'task_graph_v5'
    _, cpu_actor = load_actor_publication(pickle.dumps(publication))
    with torch.no_grad():
        torch.testing.assert_close(cpu_actor([batch[0].observation])[0].probs,
            learner.actor([batch[0].observation])[0].probs)
    saved = capture_state(learner, config, replay, schedule, UpdateBoundary())
    saved.validate(config)
    path = tmp_path/'checkpoint.pt'; torch.save(saved.record, path)
    from lunar_drl_exploration.evaluation import export_actor
    from lunar_drl_exploration.runtime import ActorPolicy
    artifact = tmp_path/'actor.pt'
    export_actor(path, artifact)
    exported = ActorPolicy.load(artifact)
    with torch.no_grad():
        torch.testing.assert_close(exported.actor([batch[0].observation])[0].probs,
            learner.actor([batch[0].observation])[0].probs, rtol=0, atol=0)
    loaded = TrainingState(torch.load(path, weights_only=True, map_location='cpu'))
    other = SACLearner(config.model, config.learning)
    restored = loaded.restore(other, config)
    assert len(restored.replay) == len(replay) == 64 == snapshot_sample_count(saved.record['replay'])
    assert all(_same_content(a[0], b[0]) for a, b in zip(replay._entries.values(), restored.replay._entries.values()))
    assert restored.replay.scene_refcounts == replay.scene_refcounts
    for i in range(5):
        torch.testing.assert_close(list((learner.actor, learner.q1, learner.q2, learner.target1, learner.target2)[i].parameters())[0],
            list((other.actor, other.q1, other.q2, other.target1, other.target2)[i].parameters())[0])
    a = replay.sample(64, np.random.default_rng(15))
    b = restored.replay.sample(64, np.random.default_rng(15))
    assert all(_same_content(x, y) for x, y in zip(a, b))
    assert other.update(batch, scenes)['updates'] == 2


def test_legacy_replay_cannot_be_captured_as_new_protocol_or_hidden_by_envelope():
    config = replace(TrainingConfig(), model=model_config())
    learner = SACLearner(config.model, config.learning)
    old = ReplayBuffer(1024**2); old.add(legacy_transition(), {'s': legacy_scene()})
    schedule = UpdateSchedule(); schedule.collected()
    with pytest.raises(ValueError, match='schema'):
        capture_state(learner, config, old, schedule, UpdateBoundary())
    fresh = ReplayBuffer(1024**2); fresh.add(samples()[0], {'s': scene()})
    saved = capture_state(learner, config, fresh, schedule, UpdateBoundary())
    payload = old.snapshot(1024**2, require_all=True)
    saved.record['replay'] = payload
    saved.record['replay_integrity'] = dict(mode='complete_v1', count=1, sha256=hashlib.sha256(payload).hexdigest())
    with pytest.raises(ValueError, match='schema'):
        saved.validate(config)


def test_legacy_weights_and_wrong_actor_schema_fail_before_replacement():
    from lunar_drl_exploration.runtime import ActorPolicy
    learner = SACLearner(model_config(), LearningConfig())
    old = SACLearner(TrainingConfig().model, LearningConfig())
    with pytest.raises(ValueError, match='model schema'):
        learner.load_state_dict(old.state_dict())
    record = learner.actor_state(); record['schema'] = 'task_graph_v4'
    with pytest.raises(ValueError, match='schema'):
        ActorPolicy.from_record(record)
