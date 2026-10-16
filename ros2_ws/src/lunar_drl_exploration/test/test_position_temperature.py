"""Position-temperature caps change only the explicitly configured SAC component."""
from dataclasses import asdict, replace
import math

import pytest
import torch

from lunar_drl_exploration.config import (LearningConfig, ModelConfig, TrainingConfig,
    canonical_learning_config, resume_semantics)
from lunar_drl_exploration.sac import SACLearner
from test_model import scene
from test_sac import transitions
from test_soft_retrace import assert_tree_equal


MODEL = ModelConfig(width=16, heads=4, layers=1)


@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf'), -float('inf'), True, '0.001'])
def test_position_cap_requires_a_positive_finite_number(value):
    with pytest.raises(ValueError):
        LearningConfig(maximum_position_alpha=value)
    with pytest.raises(ValueError):
        canonical_learning_config(dict(asdict(LearningConfig()), maximum_position_alpha=value))


def test_initial_temperature_obeys_each_applicable_cap():
    with pytest.raises(ValueError, match='optimization'):
        LearningConfig(entropy_mode='position_heading', initial_alpha=5e-5,
                       maximum_position_alpha=1e-5)
    # A larger position limit does not relax the initial heading limit.
    with pytest.raises(ValueError, match='optimization'):
        LearningConfig(entropy_mode='position_heading', initial_alpha=2e-4,
                       maximum_position_alpha=1e-3)
    # Joint mode never uses the independent position cap.
    LearningConfig(entropy_mode='joint', initial_alpha=5e-5, maximum_position_alpha=1e-6)


def test_legacy_and_explicit_unchanged_caps_have_the_same_resume_semantics():
    original = LearningConfig(entropy_mode='position_heading')
    old_record = asdict(original)
    old_record.pop('maximum_position_alpha', None)
    explicit = replace(original, maximum_position_alpha=original.maximum_alpha)
    assert canonical_learning_config(old_record) == asdict(original)
    assert canonical_learning_config(asdict(explicit)) == asdict(original)
    config = replace(TrainingConfig(), model=MODEL, learning=original)
    assert resume_semantics(config) == resume_semantics(replace(config, learning=explicit))
    changed = replace(config, learning=replace(original, maximum_position_alpha=1e-3))
    assert resume_semantics(config) != resume_semantics(changed)


def test_semantic_record_without_initial_alpha_validates_a_lower_position_cap():
    config = LearningConfig(entropy_mode='position_heading', initial_alpha=1e-6,
                            maximum_position_alpha=1e-5)
    semantics = resume_semantics(replace(TrainingConfig(), learning=config))['learning']
    assert 'initial_alpha' not in semantics
    canonical = canonical_learning_config(semantics)
    assert canonical['maximum_position_alpha'] == 1e-5
    assert 'initial_alpha' not in canonical


def test_real_update_can_raise_position_temperature_while_heading_stays_capped():
    torch.manual_seed(83)
    config = LearningConfig(entropy_mode='position_heading', microbatch_size=64,
        initial_alpha=1e-4, maximum_alpha=1e-4, maximum_position_alpha=1e-3,
        temperature_learning_rate=.05, position_target_entropy_factor=10.,
        heading_target_entropy_factor=10.)
    learner = SACLearner(MODEL, config, device='cpu')
    metrics = learner.update(transitions(), {'s': scene()})
    assert learner.updates == 1
    assert 1.001e-4 < learner.alpha_position.item() <= 1e-3
    assert learner.alpha_heading.item() <= 1e-4 * 1.000001
    assert metrics['alpha_position'] == learner.alpha_position.item()
    assert metrics['alpha_heading'] == learner.alpha_heading.item()
    assert all(math.isfinite(value) for value in metrics.values())
    # A complete second update clamps the position at its own upper limit.
    with torch.no_grad():
        learner.log_alpha.fill_(math.log(1e-3) + 1.)
        learner.log_alpha_heading.fill_(math.log(1e-4) + 1.)
    learner.update(transitions(), {'s': scene()})
    assert learner.alpha_position.item() == pytest.approx(1e-3, rel=1e-6)
    assert learner.alpha_heading.item() == pytest.approx(1e-4, rel=1e-6)


def test_joint_mode_keeps_its_original_cap_and_identical_full_update():
    torch.manual_seed(84)
    config = LearningConfig(microbatch_size=64, initial_alpha=1e-4,
                            target_entropy_factor=10., temperature_learning_rate=.05)
    original = SACLearner(MODEL, config)
    unused = SACLearner(MODEL, replace(config, maximum_position_alpha=1e-3))
    unused.load_state_dict(original.state_dict())
    samples, scenes = transitions(), {'s': scene()}
    original.update(samples, scenes)
    unused.update(samples, scenes)
    assert original.alpha.item() == pytest.approx(1e-4, rel=1e-6)
    assert unused.alpha.item() == original.alpha.item()
    for key in ('actor', 'q1', 'q2', 'target1', 'target2', 'actor_optimizer',
                'q1_optimizer', 'q2_optimizer', 'alpha_optimizer', 'log_alpha'):
        assert_tree_equal(original.state_dict()[key], unused.state_dict()[key])
    first = replace(TrainingConfig(), model=MODEL, learning=config)
    assert resume_semantics(first) == resume_semantics(
        replace(first, learning=unused.learning_config))


def test_ordinary_learner_load_accepts_old_cap_but_rejects_changed_actual_cap():
    config = LearningConfig(entropy_mode='position_heading')
    original = SACLearner(MODEL, config)
    saved = original.state_dict()
    saved['learning_config'].pop('maximum_position_alpha', None)
    compatible = SACLearner(MODEL, replace(config, maximum_position_alpha=1e-4))
    compatible.load_state_dict(saved)
    for key in ('actor', 'q1', 'q2', 'target1', 'target2', 'log_alpha', 'log_alpha_heading'):
        assert_tree_equal(compatible.state_dict()[key], saved[key])
    for cap in (1e-3, 7e-5):
        changed = SACLearner(MODEL, replace(config, maximum_position_alpha=cap))
        with pytest.raises(ValueError, match='incompatible SAC semantics'):
            changed.load_state_dict(saved)
        with pytest.raises(ValueError, match='incompatible SAC semantics'):
            compatible.load_state_dict(changed.state_dict())


def test_full_legacy_checkpoint_validates_and_actual_cap_change_requires_migration():
    from lunar_drl_exploration.checkpoint import UpdateBoundary
    from lunar_drl_exploration.replay import ReplayBuffer
    from lunar_drl_exploration.schedule import UpdateSchedule
    from test_replay_checkpoint import capture_state, scene as replay_scene, transition
    config = replace(TrainingConfig(), model=MODEL,
                     learning=LearningConfig(entropy_mode='position_heading'))
    learner = SACLearner(MODEL, config.learning)
    replay = ReplayBuffer(1000000)
    replay.add(transition(), {'s': replay_scene()})
    schedule = UpdateSchedule()
    schedule.collected()
    checkpoint = capture_state(learner, config, replay, schedule, UpdateBoundary())
    for record in (checkpoint.record['semantics']['learning'],
                   checkpoint.record['config']['learning'],
                   checkpoint.record['learner']['learning_config']):
        record.pop('maximum_position_alpha', None)
    checkpoint.validate()
    checkpoint.validate(config)
    checkpoint.validate(replace(config, learning=replace(config.learning, maximum_position_alpha=1e-4)))
    with pytest.raises(ValueError, match='incompatible full resume semantics'):
        checkpoint.validate(replace(config, learning=replace(config.learning, maximum_position_alpha=1e-3)))
