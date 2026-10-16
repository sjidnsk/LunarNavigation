"""A2 entry assessment is a promotion decision, never a completed training block."""
import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts/drl/task_frontier'))


def source_state():
    return dict(phase=1, blocks=18, phase_blocks=6, streak=0,
                warmup_complete=True, status='running', history=[{'block': 18}],
                a2_entry_screen=dict(status='pending'))


def result_rows(cave=4, moon=4):
    return [dict(curriculum_task=task, completed=i < success, decisions=100,
                 zero_gain_two_point_loop_steps=0, collisions=0)
            for task, success in [('cave_inside', cave), ('moon_inside', moon)]
            for i in range(5)]


def test_entry_pass_promotes_without_consuming_block_or_altering_history():
    from course_controller import apply_a2_entry_result, default_plan
    state = source_state()
    history = copy.deepcopy(state['history'])
    apply_a2_entry_result(state, result_rows(), default_plan(), 'actor', 'checkpoint')
    assert (state['phase'], state['phase_blocks'], state['streak']) == (2, 0, 0)
    assert state['blocks'] == 18 and state['history'] == history
    assert state['a2_entry_screen']['status'] == 'finished'
    assert state['a2_entry_screen']['gate']['passed']
    assert state['a2_entry_screen']['actor_sha'] == 'actor'


def test_entry_failure_keeps_cave_phase_and_retention_gate():
    from course_controller import apply_a2_entry_result, default_plan
    state = source_state()
    apply_a2_entry_result(state, result_rows(moon=3), default_plan(), 'actor', 'checkpoint')
    assert (state['phase'], state['blocks'], state['phase_blocks']) == (1, 18, 6)
    assert not state['a2_entry_screen']['gate']['passed']


def test_incomplete_or_repeated_entry_is_rejected_before_state_changes():
    from course_controller import apply_a2_entry_result, default_plan
    state = source_state()
    before = copy.deepcopy(state)
    with pytest.raises(ValueError):
        apply_a2_entry_result(state, result_rows()[:-1], default_plan(), 'actor', 'checkpoint')
    assert state == before
    apply_a2_entry_result(state, result_rows(), default_plan(), 'actor', 'checkpoint')
    after = copy.deepcopy(state)
    with pytest.raises(ValueError):
        apply_a2_entry_result(state, result_rows(), default_plan(), 'actor', 'checkpoint')
    assert state == after


def test_entry_cases_are_five_inside_caves_and_five_inside_moons():
    from course_controller import course_cases, default_plan
    cases = course_cases(1, 7, default_plan())
    assert [c['curriculum_task'] for c in cases] == ['cave_inside'] * 5 + ['moon_inside'] * 5
    assert all(c['start_mode'] == 'inside' for c in cases)
    assert [c['budget'] for c in cases[:5]] == [256, 288, 352, 448, 512]
