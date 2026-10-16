"""Explicit arrival contracts and lossless, narrowly scoped continuation."""
from dataclasses import replace
import math
import pytest

from lunar_drl_exploration.config import (
    TrainingConfig, ModelConfig, config_record, resume_semantics,
    training_config_from_record)


def test_explicit_yaw_roundtrip_and_legacy_checkpoint_semantics():
    legacy = config_record(TrainingConfig())
    legacy.pop('goal_yaw_tolerance_rad', None)
    old = training_config_from_record(legacy)
    assert old.goal_yaw_tolerance_rad == math.pi / 12
    assert 'goal_yaw_tolerance_rad' not in resume_semantics(old)
    aligned = replace(old, goal_position_tolerance_m=.10, goal_yaw_tolerance_rad=.05)
    assert training_config_from_record(config_record(aligned)) == aligned
    assert resume_semantics(aligned)['goal_yaw_tolerance_rad'] == .05


@pytest.mark.parametrize('value', [0., -.05, math.nan, math.inf])
def test_yaw_tolerance_must_be_positive_finite(value):
    with pytest.raises(ValueError, match='goal_yaw_tolerance_rad'):
        replace(TrainingConfig(), goal_yaw_tolerance_rad=value)


def test_arrival_migration_preserves_all_training_payload_and_rejects_other_changes(tmp_path):
    import numpy as np
    from lunar_drl_exploration.arrival_contract import migrate_arrival_contract
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.replay import ReplayBuffer, snapshot_sample_count
    from lunar_drl_exploration.schedule import UpdateSchedule
    from lunar_drl_exploration.checkpoint import UpdateBoundary, CheckpointManager
    from test_replay_checkpoint import capture_state, transition, scene
    old = replace(TrainingConfig(), output_dir=str(tmp_path),
                  model=ModelConfig(width=16, heads=2, layers=1), warmup=0)
    learner = SACLearner(old.model, old.learning)
    replay = ReplayBuffer(1000000)
    replay.add(transition(), {'s': scene()})
    schedule = UpdateSchedule(warmup=0)
    for _ in range(7): schedule.collected()
    boundary = UpdateBoundary()
    with boundary.update():
        learner.update(replay.sample(64, np.random.default_rng(2)), replay.scenes)
        schedule.updated()
    original = capture_state(learner, old, replay, schedule, boundary)
    new = replace(old, goal_position_tolerance_m=.10, goal_yaw_tolerance_rad=.05)
    with pytest.raises(ValueError, match='incompatible full resume semantics'):
        original.validate(new)
    migrated = migrate_arrival_contract(original, new)
    assert migrated is not original
    for key in original.record.keys() - {'config', 'semantics'}:
        assert migrated.record[key] is original.record[key], key
    assert original.record['config']['goal_position_tolerance_m'] == .05
    migrated.validate(new)
    manager = CheckpointManager(tmp_path)
    manager.save(migrated)
    restored = manager.load(new)
    assert restored.record['replay'] == original.record['replay']
    assert snapshot_sample_count(restored.record['replay']) == 1
    assert restored.record['schedule'] == original.record['schedule']
    restored_state = restored.restore(learner, new)
    from lunar_drl_exploration.checkpoint import TrainingState
    recaptured = TrainingState.capture(learner=learner, config=new,
        replay=restored_state.replay, schedule=restored_state.schedule, boundary=boundary,
        replay_rng=restored_state.replay_rng, curriculum_rng=restored_state.curriculum_rng,
        curriculum=restored_state.curriculum, collector_state=restored_state.collector_state,
        counters=restored_state.counters,
        arrival_contract_migrations=restored.record['arrival_contract_migrations'])
    manager.save(recaptured)
    assert manager.load(new).record['arrival_contract_migrations'] == migrated.record['arrival_contract_migrations']
    with pytest.raises(ValueError, match='only arrival tolerances'):
        migrate_arrival_contract(original, replace(new, target_rtf=2.))
