"""Entropy comparisons preserve the common state and explicitly clone temperature history."""
import copy
from dataclasses import replace
import hashlib

import pytest
import torch

from test_training_fork import learned_checkpoint, assert_state_equal


def requested(config, tmp_path, mode):
    return replace(config, output_dir=str(tmp_path / mode), paired_scene_sequence=True,
        snapshot_max_bytes=256 * 1024**2,
        learning=replace(config.learning, entropy_mode=mode, temperature_learning_rate=.003))


@pytest.mark.parametrize('mode', ['joint', 'position_heading'])
def test_entropy_fork_preserves_source_and_clones_temperature_history(learned_checkpoint, tmp_path, mode):
    from lunar_drl_exploration.checkpoint import TrainingState
    from lunar_drl_exploration.sac import SACLearner
    config, source, original = learned_checkpoint
    digest = hashlib.sha256(source.read_bytes()).digest()
    changed = requested(config, tmp_path, mode)
    fork = TrainingState.fork_from(source, changed, experiment='paired-entropy')
    for key in ('replay', 'schedule', 'rng', 'curriculum', 'collector_state', 'counters'):
        assert_state_equal(fork.record[key], original.record[key])
    for key in ('actor', 'q1', 'q2', 'target1', 'target2', 'actor_optimizer',
                'q1_optimizer', 'q2_optimizer', 'log_alpha', 'alpha_optimizer'):
        assert_state_equal(fork.record['learner'][key], original.record['learner'][key])
    if mode == 'position_heading':
        saved = fork.record['learner']
        assert_state_equal(saved['log_alpha_heading'], saved['log_alpha'])
        assert saved['log_alpha_heading'].data_ptr() != saved['log_alpha'].data_ptr()
        assert_state_equal(saved['heading_alpha_optimizer'], saved['alpha_optimizer'])
        for index, values in saved['alpha_optimizer']['state'].items():
            for name, value in values.items():
                if isinstance(value, torch.Tensor):
                    assert value.data_ptr() != saved['heading_alpha_optimizer']['state'][index][name].data_ptr()
    learner = SACLearner(config.model, changed.learning)
    restored = fork.restore(learner, changed)
    assert len(restored.replay) == 1
    assert_state_equal(learner.alpha_optimizer.state_dict()['state'],
                       original.record['learner']['alpha_optimizer']['state'])
    assert learner.alpha_optimizer.param_groups[0]['lr'] == .003
    assert learner.actor_optimizer.param_groups[0]['lr'] == config.learning.learning_rate
    if mode == 'position_heading':
        assert learner.heading_alpha_optimizer.param_groups[0]['lr'] == .003
    assert hashlib.sha256(source.read_bytes()).digest() == digest
    changes = fork.record['fork']['changes']['learning']
    assert changes['temperature_learning_rate'] == {'before': None, 'after': .003}
    assert ('entropy_mode' in changes) == (mode == 'position_heading')


@pytest.mark.parametrize('experiment', [None, 'paired-polyak', 'paired-multistep'])
def test_other_forks_cannot_migrate_entropy_objective(learned_checkpoint, tmp_path, experiment):
    from lunar_drl_exploration.checkpoint import TrainingState
    config, source, _ = learned_checkpoint
    with pytest.raises(ValueError, match='learning'):
        TrainingState.fork_from(source, requested(config, tmp_path, 'position_heading'), experiment=experiment)


@pytest.mark.parametrize('field,value', [('return_steps', 8), ('gamma', .99),
    ('polyak', .02), ('learning_rate', .001), ('initial_alpha', .0001),
    ('maximum_alpha', .001), ('position_target_entropy_factor', .3)])
def test_entropy_fork_rejects_unrelated_learning_changes(learned_checkpoint, tmp_path, field, value):
    from lunar_drl_exploration.checkpoint import TrainingState
    config, source, _ = learned_checkpoint
    changed = requested(config, tmp_path, 'position_heading')
    changed = replace(changed, learning=replace(changed.learning, **{field: value}))
    with pytest.raises(ValueError, match='learning.' + field):
        TrainingState.fork_from(source, changed, experiment='paired-entropy')


def test_entropy_resume_rejects_missing_or_nonfinite_heading_temperature(learned_checkpoint, tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState
    config, source, _ = learned_checkpoint
    changed = requested(config, tmp_path, 'position_heading')
    state = TrainingState.fork_from(source, changed, experiment='paired-entropy')
    for field in ('log_alpha_heading', 'heading_alpha_optimizer'):
        broken = copy.deepcopy(state)
        del broken.record['learner'][field]
        with pytest.raises(ValueError, match='entropy|temperature'):
            broken.validate(changed)
    broken = copy.deepcopy(state)
    broken.record['learner']['log_alpha_heading'].fill_(float('nan'))
    with pytest.raises(ValueError, match='entropy|temperature'):
        broken.validate(changed)


def test_entropy_initial_identity_checks_duplicate_state_before_normalizing(learned_checkpoint, tmp_path):
    from lunar_drl_exploration.checkpoint import TrainingState
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.training import initial_training_identity
    config, source, _ = learned_checkpoint
    identities = []
    for mode in ('joint', 'position_heading'):
        changed = requested(config, tmp_path, mode)
        state = TrainingState.fork_from(source, changed, experiment='paired-entropy')
        learner = SACLearner(config.model, changed.learning)
        restored = state.restore(learner, changed)
        kwargs = dict(replay=restored.replay, schedule=restored.schedule,
            replay_rng=restored.replay_rng, curriculum_rng=restored.curriculum_rng,
            curriculum_state=restored.curriculum, collector_state=restored.collector_state,
            counters=restored.counters, source_replay_sha256='source',
            excluded_learning_fields=('entropy_mode',))
        identities.append(initial_training_identity(learner, **kwargs))
    assert identities[0] == identities[1]
    with torch.no_grad():
        learner.log_alpha_heading.add_(.01)
    with pytest.raises(ValueError, match='initial.*temperature'):
        initial_training_identity(learner, **kwargs)
    # A later ordinary resume must identify, rather than equate, temperatures
    # which legitimately diverged after learning. No paired normalization then.
    kwargs['excluded_learning_fields'] = ()
    before = initial_training_identity(learner, **kwargs)
    with torch.no_grad():
        learner.log_alpha_heading.add_(.01)
    assert initial_training_identity(learner, **kwargs)['learner_sha256'] != before['learner_sha256']


def test_entropy_cli_requires_explicit_fork():
    from lunar_drl_exploration.cli import parser, load_config
    args = parser().parse_args(['train', '--fork-experiment', 'paired-entropy'])
    with pytest.raises(ValueError, match='requires --fork-from'):
        load_config(args)


def test_learned_split_fork_can_resume_with_distinct_temperatures(tmp_path):
    import json
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.training import run_training
    from lunar_drl_exploration.checkpoint import CheckpointManager
    config = replace(TrainingConfig(), environments=1, warmup=0,
        output_dir=str(tmp_path / 'source'), system_reserve_bytes=0)
    kwargs = dict(device='cpu', env_factory='worker_fixtures:ControlledEnv',
        probe_extent=40, probe_budget=3)
    run_training(config, max_transitions=8, **kwargs)
    changed = requested(config, tmp_path, 'position_heading')
    learned = run_training(changed, fork_from=tmp_path / 'source' / 'resume.pt',
        fork_experiment='paired-entropy', max_transitions=8, **kwargs)
    saved = CheckpointManager(changed.output_dir).load(changed).record['learner']
    assert not torch.equal(saved['log_alpha'], saved['log_alpha_heading'])
    result = run_training(changed, resume=True, max_transitions=4, **kwargs)
    assert result['updates'] > learned['updates']
    observations = json.loads((tmp_path / 'position_heading' / 'run.json').read_text())['observations']
    assert observations['initial_state_identity']['excluded_learning_fields'] == []
