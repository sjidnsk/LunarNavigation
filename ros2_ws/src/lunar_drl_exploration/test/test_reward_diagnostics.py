"""Reward accounting stays bounded and separates completed from censored tails."""
import importlib
import json
import math
from types import SimpleNamespace

import pytest


def diagnostics(environments=2, **kwargs):
    name = 'lunar_drl_exploration.reward_diagnostics'
    assert importlib.util.find_spec(name) is not None, 'bounded reward diagnostics are missing'
    return importlib.import_module(name).RewardDiagnostics(environments, **kwargs)


def transition(episode='one', *, area=0., removed=0, outside=0, distance=2., turn=math.pi,
               reward=None, terminated=False, truncated=False, actor=10):
    if reward is None:
        reward = removed / 50. - .002 * distance - .005 * turn / math.pi - .001
        if terminated:
            reward += 5.
    return SimpleNamespace(episode_id=episode, actor_version=actor, reward=reward,
        terminated=terminated, truncated=truncated,
        parts=SimpleNamespace(new_area_m2=area, distance_m=distance, turn_rad=turn, removed_frontiers_in=removed, removed_frontiers_out=outside))


def observe(diag, item=None, **kwargs):
    defaults = dict(before_coverage=.96, after_coverage=.96, family='cave', extent_m=60.,
                    execution={'wall_s': 2., 'simulation_s': 5.})
    defaults.update(kwargs)
    diag.observe(0, item or transition(), **defaults)


def test_reward_accounting_uses_before_coverage_and_keeps_components():
    d = diagnostics()
    observe(d, transition(area=.4, removed=0), before_coverage=.799, after_coverage=.81)
    s = d.summary()
    assert s['totals']['admitted'] == 1
    assert s['totals']['positive_gain'] == 1
    assert s['totals']['positive_gain_negative_reward'] == 1
    assert s['totals']['reward_sum'] == pytest.approx(-.01)
    components = s['totals']['reward_components']
    assert components == pytest.approx(dict(exploration=0., motion=-.009, step=-.001, completion=0.))
    assert sum(components.values()) == pytest.approx(s['totals']['reward_sum'])
    assert s['by_coverage']['below_80']['admitted'] == 1
    assert s['by_coverage']['80_to_95']['admitted'] == 0
    assert s['totals']['wall_s'] == 2.
    assert s['totals']['simulation_s'] == 5.


def test_success_tail_and_truncated_tail_are_separate_and_actor_range_is_visible():
    d = diagnostics()
    observe(d, transition(actor=10))
    observe(d, transition(area=1., terminated=True, actor=14), after_coverage=.99)
    observe(d, transition('two', truncated=True, actor=20), truncation_reason='STAGNATION')
    summary = d.summary()
    tails = summary['late_episode_segments']
    assert tails['success']['segments'] == 1
    assert tails['success']['steps'] == 2
    assert tails['success']['reward_sum'] == pytest.approx(4.98)
    assert tails['success']['actor_version_min'] == 10
    assert tails['success']['actor_version_max'] == 14
    assert tails['truncated']['segments'] == 1
    assert tails['truncated']['reward_sum'] == pytest.approx(-.01)
    assert summary['totals']['success'] == 1
    assert summary['totals']['truncation_reasons']['STAGNATION'] == 1
    assert 'not a frozen-policy return' in summary['late_return_scope']


def test_termination_wins_and_exhaustion_does_not_receive_success_credit():
    d = diagnostics()
    observe(d, transition(terminated=True, truncated=True), after_coverage=.995,
            truncation_reason='STAGNATION')
    observe(d, transition('exhausted', reward=-.01, terminated=True), after_coverage=.97)
    totals = d.summary()['totals']
    assert totals['success'] == 1
    assert totals['terminated_other'] == 1
    assert totals['truncated'] == 0
    assert totals['reward_components']['completion'] == 5.
    assert totals['reward_mismatch_count'] == 0


def test_collision_at_success_coverage_is_not_a_completed_exploration():
    d = diagnostics()
    observe(d, transition(terminated=True, reward=-.01), after_coverage=.999,
            execution=dict(reason_code='COLLISION'))
    totals = d.summary()['totals']
    assert totals['success'] == 0
    assert totals['terminated_other'] == 1
    assert totals['reward_components']['completion'] == 0.
    assert totals['reward_mismatch_count'] == 0
    assert d.cases()['success'] == []


def test_fixed_case_reservations_and_step_rings_do_not_grow_with_episodes():
    d = diagnostics()
    observe(d, transition('success', terminated=True), after_coverage=.99)
    for episode in range(40):
        for step in range(150):
            observe(d, transition(str(episode), truncated=step == 149, actor=step),
                    family='unbounded-scene-' + str(episode), extent_m=1e4,
                    truncation_reason='STAGNATION' if step == 149 else None)
    cases = d.cases()
    assert len(cases['success']) == 1
    assert len(cases['stagnation']) == 8
    assert all(len(case['steps']) == 128 for case in cases['stagnation'])
    assert cases['stagnation'][-1]['steps'][0]['actor_version'] == 22
    assert cases['stagnation'][-1]['segment_steps'] == 150
    assert d.summary()['totals']['admitted'] == 6001
    assert set(d.summary()['by_family']) == {'moon', 'cave', 'other'}
    assert d.summary()['by_family']['other']['admitted'] == 6000
    assert len(json.dumps(d.summary()).encode()) < 1024**2
    assert len(json.dumps(cases).encode()) < 16 * 1024**2
    assert 'cannot estimate population rates' in cases['scope']


def test_restore_keeps_cumulative_counts_and_marks_interrupted_episode_segment():
    d = diagnostics()
    observe(d, transition(actor=4))
    state = d.state_dict()
    # Round-tripping is primitive-only: no tensor, graph, transition or raw map.
    restored = diagnostics(state=json.loads(json.dumps(state)))
    assert restored.summary()['totals']['admitted'] == 1
    assert restored.summary()['late_episode_segments']['interrupted']['steps'] == 1
    assert restored.summary()['segments_closed_by_reason']['RESUME'] == 1
    observe(restored, transition('new', terminated=True, actor=8), after_coverage=.99)
    tails = restored.summary()['late_episode_segments']
    assert tails['success']['steps'] == 1
    assert tails['success']['actor_version_min'] == 8
    assert restored.summary()['totals']['admitted'] == 2
    # Exported state is detached; callers cannot mutate live diagnostics.
    state['totals']['admitted'] = 999
    assert d.summary()['totals']['admitted'] == 1


def test_reset_preserves_censored_tail_and_open_tail_is_never_counted_as_success():
    d = diagnostics()
    observe(d)
    assert d.summary()['late_episode_segments']['active_incomplete']['steps'] == 1
    d.reset(0, reason='RECOVERY')
    assert d.summary()['late_episode_segments']['active_incomplete']['steps'] == 0
    assert d.summary()['late_episode_segments']['interrupted']['steps'] == 1
    assert d.summary()['segments_closed_by_reason']['RECOVERY'] == 1
    assert d.summary()['totals']['success'] == 0


def test_state_rejects_oversized_rings_and_wrong_environment_count():
    d = diagnostics()
    observe(d)
    state = d.state_dict()
    state['active'][0]['steps'] *= 129
    with pytest.raises(ValueError, match='bounded'):
        diagnostics(state=state)
    with pytest.raises(ValueError, match='environments'):
        diagnostics(environments=3, state=d.state_dict())


@pytest.mark.parametrize('corrupt', ['unknown_payload', 'unknown_counter', 'long_episode_id'])
def test_restore_rejects_hidden_payloads_and_unbounded_identifiers(corrupt):
    d = diagnostics()
    observe(d)
    state = d.state_dict()
    if corrupt == 'unknown_payload':
        state['active'][0]['steps'][0]['raw_observation'] = [[0.] * 100]
    elif corrupt == 'unknown_counter':
        state['totals']['per_seed'] = {'12345': 1}
    else:
        state['active'][0]['episode_id'] = 'x' * 129
    with pytest.raises(ValueError, match='bounded|schema'):
        diagnostics(state=state)


def test_both_case_groups_retain_only_their_latest_eight_examples_and_seed():
    d = diagnostics()
    for index in range(25):
        observe(d, transition(f'success-{index}', terminated=True), after_coverage=.99,
                seed=index)
        observe(d, transition(f'loop-{index}', truncated=True), truncation_reason='STAGNATION',
                seed=index)
    cases = d.cases()
    assert len(cases['success']) + len(cases['stagnation']) == 16
    assert [case['seed'] for case in cases['success']] == list(range(17, 25))
    assert [case['seed'] for case in cases['stagnation']] == list(range(17, 25))
    assert diagnostics(state=d.state_dict()).summary()['totals']['admitted'] == 50


@pytest.mark.parametrize('overrides', [dict(before_coverage=float('nan')),
    dict(after_coverage=float('inf')), dict(extent_m=-1.)])
def test_invalid_metadata_does_not_partially_mutate_counts(overrides):
    d = diagnostics()
    with pytest.raises(ValueError):
        observe(d, **overrides)
    assert d.summary()['totals']['admitted'] == 0


def test_frontier_reward_uses_integer_removals_and_area_zero_stays_area_zero():
    d = diagnostics()
    observe(d, transition(area=0., removed=5, outside=99))
    totals = d.summary()['totals']
    assert totals['zero_gain'] == 1
    assert totals['removed_frontiers_in'] == 5
    assert totals['removed_frontiers_out'] == 99
    assert totals['reward_components'] == pytest.approx(dict(exploration=.1, motion=-.009, step=-.001, completion=0.))
    assert totals['reward_sum'] == pytest.approx(.09)
    assert totals['reward_mismatch_count'] == 0
    observe(d, transition(area=3., removed=0, outside=500))
    assert d.summary()['totals']['reward_components']['exploration'] == .1
