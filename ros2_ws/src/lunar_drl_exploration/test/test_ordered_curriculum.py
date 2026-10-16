"""Completed admissions, rather than episode lengths, determine course shares."""
from collections import Counter
from dataclasses import replace

import numpy as np
import pytest


def test_four_tasks_and_prior_task_rehearsal():
    from lunar_drl_exploration.ordered_curriculum import task_weights
    assert task_weights(0) == {'moon_inside': 1.}
    assert task_weights(1) == {'moon_inside': .2, 'cave_inside': .8}
    assert task_weights(3) == pytest.approx({'moon_inside': 1/15, 'cave_inside': 1/15,
        'moon_outside': 1/15, 'cave_outside': .8})
    for phase in (-1, 4, True):
        with pytest.raises(ValueError): task_weights(phase)


def test_course_resumes_same_or_next_phase_only_and_keeps_totals():
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    course = OrderedCourse(0)
    course.activate(0, 'moon_inside')
    course.admitted(0)
    course.observe(0, terminated=True, truncated=False, coverage=.995, collision=False)
    course.sampled(['moon_inside'] * 64)
    restored = OrderedCourse(1, state=course.state_dict())
    assert restored.summary()['by_task']['moon_inside'] == dict(
        transitions=1, samples=64, ended=1, successes=1, episode_steps=1)
    assert restored.summary()['phase_transitions'] == {'moon_inside': 0, 'cave_inside': 0}
    for phase in (2, 3):
        with pytest.raises(ValueError, match='phase'):
            OrderedCourse(phase, state=course.state_dict())
    with pytest.raises(ValueError, match='phase'):
        OrderedCourse(0, state=restored.state_dict())


def test_grants_balance_completed_transitions_with_long_old_episodes():
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    course = OrderedCourse(1)
    for env in range(8):
        course.activate(env, course.next_task())
    counts = Counter()
    pending = set(range(8))
    # Moon stays in one long episode while cave ends every 3 decisions.
    steps = Counter()
    for _ in range(8192):
        env = course.select_pending(pending, {})
        assert env is not None
        task = course.active[env]
        course.admitted(env)
        counts[task] += 1
        steps[env] += 1
        terminal = steps[env] % (400 if task == 'moon_inside' else 3) == 0
        course.observe(env, terminated=False, truncated=terminal, coverage=.5, collision=False)
        if terminal:
            course.activate(env, course.next_task())
    assert abs(counts['cave_inside'] - 6553.6) <= 1
    assert abs(counts['moon_inside'] - 1638.4) <= 1
    stats = course.summary()['by_task']
    assert stats['cave_inside']['ended'] > stats['moon_inside']['ended'] * 100


def test_reservations_are_counted_and_fast_task_cannot_run_ahead():
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    course = OrderedCourse(1)
    for env in range(4):
        course.activate(env, 'cave_inside')
    course.activate(4, 'moon_inside')
    inflight = {env: 11 for env in range(4)}
    # Four cave actions are reserved, so the fifth admission belongs to moon.
    assert course.select_pending({4}, inflight) == 4
    course.admitted(4)
    # Slow cave remains in flight; moon cannot take all remaining credit.
    assert course.select_pending({4}, inflight) is None


def test_bootstrap_samples_available_tasks_and_reports_actual_shares():
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    course = OrderedCourse(1)
    assert course.sampling_weights({'moon_inside': 12}) == {'moon_inside': 1.}
    course.sampled(['moon_inside'] * 64)
    assert course.summary()['bootstrap_updates'] == 1
    assert course.summary()['missing_replay_tasks'] == ['cave_inside']
    assert course.sampling_weights({'moon_inside': 12, 'cave_inside': 1}) == {
        'moon_inside': .2, 'cave_inside': .8}
    course.sampled(['moon_inside'] * 13 + ['cave_inside'] * 51)
    assert course.summary()['by_task']['cave_inside']['samples'] == 51


def test_interrupt_is_excluded_from_ended_and_success_requires_collision_free_99():
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    course = OrderedCourse(0)
    course.activate(0, 'moon_inside')
    for _ in range(3):
        course.admitted(0)
        course.observe(0, terminated=False, truncated=False, coverage=.5, collision=False)
    restored = OrderedCourse(0, state=course.state_dict())
    restored.activate(0, 'moon_inside')
    restored.admitted(0)
    restored.observe(0, terminated=True, truncated=False, coverage=1., collision=True)
    assert restored.summary()['by_task']['moon_inside'] == dict(
        transitions=4, samples=0, ended=1, successes=0, episode_steps=1)


@pytest.mark.parametrize('extent,want', [(50,256),(61.111,288),(72.222,352),(88.889,448),(100,512)])
def test_budget_preserves_original_area_interpolation(extent, want):
    from lunar_drl_exploration.ordered_curriculum import area_budget
    assert area_budget(extent) == want


def test_curriculum_freezes_label_and_does_not_use_slot_parity(monkeypatch):
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.training import Curriculum
    monkeypatch.setenv('DRL_COURSE_PHASE', '0')
    config = replace(TrainingConfig(), ordered_curriculum=True, environments=8)
    curriculum = Curriculum(config, np.random.default_rng(4))
    specs = [curriculum.next(env, 0) for env in range(8)]
    assert all(spec['family'] == 'moon' and spec['start_mode'] == 'inside'
        and spec['curriculum_task'] == 'moon_inside' for spec in specs)
    assert all(256 <= spec['episode_budget'] <= 512 for spec in specs)
    # A saved old mixed curriculum is incompatible, even at phase zero.
    with pytest.raises(ValueError, match='ordered'):
        Curriculum(config, np.random.default_rng(4), {'episodes_issued': 99})


# External environment double: real collector, replay, SAC and checkpoint owners
# still run. Navigation is the only controlled dependency in this bounded test.
from worker_fixtures import ControlledEnv


class OrderedControlledEnv(ControlledEnv):
    def reset(self, *args, start_mode='any', curriculum_task=None, **kwargs):
        result = super().reset(*args, **kwargs)
        self.static_scenes = {key: replace(scene,
            generator_descriptor=dict(scene.generator_descriptor, curriculum_task=curriculum_task,
                start_mode=start_mode)) for key, scene in self.static_scenes.items()}
        self.episode_metadata.update(curriculum_task=curriculum_task, start_mode=start_mode)
        return result


def test_bounded_course_updates_and_next_phase_resume_preserve_durable_counters(tmp_path, monkeypatch):
    import json
    pytest.importorskip('torch')
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.training import run_training
    from lunar_drl_exploration.checkpoint import CheckpointManager
    from lunar_drl_exploration.replay import loads_transport
    config = replace(TrainingConfig(), environments=2, warmup=0,
        ordered_curriculum=True, output_dir=str(tmp_path/'run'), artifact_root=str(tmp_path),
        total_output_max_bytes=512*1024**2, system_reserve_bytes=0, save_interval_s=1800.)
    monkeypatch.setenv('DRL_COURSE_PHASE', '0')
    result = run_training(config, device='cpu', max_transitions=32,
        env_factory='test_ordered_curriculum:OrderedControlledEnv', probe_extent=60, probe_budget=3)
    assert result['new_transitions'] == 32
    summary = json.loads((tmp_path/'run'/'course-summary.json').read_text())
    assert summary['by_task']['moon_inside']['transitions'] == 32
    assert summary['by_task']['moon_inside']['samples'] == 512
    checkpoint = CheckpointManager(config.output_dir).load(config).record
    assert loads_transport(checkpoint['counters'])['course']['by_task'] == summary['by_task']
    assert checkpoint['schedule']['unfinished_reservations'] == 0
    monkeypatch.setenv('DRL_COURSE_PHASE', '1')
    second = run_training(config, resume=True, device='cpu', max_transitions=32,
        env_factory='test_ordered_curriculum:OrderedControlledEnv', probe_extent=60, probe_budget=3)
    assert second['new_transitions'] == 32
    summary = json.loads((tmp_path/'run'/'course-summary.json').read_text())
    assert summary['phase'] == 1
    assert summary['by_task']['moon_inside']['transitions'] == 38
    assert summary['by_task']['cave_inside']['transitions'] == 26
    assert sum(stats['samples'] for stats in summary['by_task'].values()) == 1024
    assert summary['phase_transitions'] == {'moon_inside': 6, 'cave_inside': 26}
    monkeypatch.setenv('DRL_COURSE_PHASE', '3')
    with pytest.raises(ValueError, match='phase'):
        run_training(config, resume=True, device='cpu', max_transitions=1,
            env_factory='test_ordered_curriculum:OrderedControlledEnv')


def test_phase_sample_shares_reset_only_on_promotion():
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    first = OrderedCourse(0)
    first.sampled(['moon_inside'] * 64)
    same = OrderedCourse(0, state=first.state_dict())
    assert same.summary()['phase_samples'] == {'moon_inside': 64}
    second = OrderedCourse(1, state=same.state_dict())
    assert second.summary()['phase_samples'] == {'moon_inside': 0, 'cave_inside': 0}
    second.sampled(['cave_inside'] * 51 + ['moon_inside'] * 13)
    assert second.summary()['phase_samples'] == {'moon_inside': 13, 'cave_inside': 51}
    assert second.summary()['by_task']['moon_inside']['samples'] == 77


def test_recovery_does_not_replace_only_live_rehearsal_slot(monkeypatch):
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.training import Curriculum
    monkeypatch.setenv('DRL_COURSE_PHASE', '1')
    curriculum = Curriculum(replace(TrainingConfig(), ordered_curriculum=True, environments=2),
        np.random.default_rng(1))
    assert curriculum.next(0, 0)['curriculum_task'] == 'cave_inside'
    assert curriculum.next(1, 0)['curriculum_task'] == 'moon_inside'
    # Infrastructure restart requests a reset without a terminal transition.
    assert curriculum.next(1, 0)['curriculum_task'] == 'moon_inside'


@pytest.mark.parametrize('field', ['phase_transitions', 'phase_samples'])
def test_invalid_persisted_phase_counts_are_rejected(field):
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    state = OrderedCourse(0).state_dict()
    state[field]['moon_inside'] = -1
    with pytest.raises(ValueError, match='counter'):
        OrderedCourse(0, state=state)


def test_promotion_does_not_publish_previous_phase_sampler_weights():
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    first = OrderedCourse(0)
    first.sampling_weights({'moon_inside': 20})
    promoted = OrderedCourse(1, state=first.state_dict())
    assert promoted.summary()['sampling_weights'] == {}

def test_missing_retained_task_is_not_new_task_bootstrap():
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    course=OrderedCourse(1)
    with pytest.raises(ValueError,match='retained'):
        course.sampling_weights({'cave_inside':10})
    assert course.sampling_weights({'moon_inside':10})=={'moon_inside':1.}
    assert course.missing_replay_tasks==['cave_inside']

def test_current_task_eviction_is_not_initial_bootstrap():
    from lunar_drl_exploration.ordered_curriculum import OrderedCourse
    course=OrderedCourse(1)
    course.by_task['cave_inside']['transitions']=1
    with pytest.raises(ValueError,match='current'):
        course.sampling_weights({'moon_inside':10})
