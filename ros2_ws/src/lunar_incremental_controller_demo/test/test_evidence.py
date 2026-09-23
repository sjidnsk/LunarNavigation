"""Acceptance must reject cross-session planning and premature completion."""
import math
import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lunar_incremental_controller_demo.evidence import (
    plan_found_with_active_reference, matching_stopped_completion,
)

REFS = {('current', 3): dict(session_id='current', segment_revision=3,
                            reaches_final_goal=True)}


@pytest.mark.parametrize('change', [dict(session_id='old'), dict(segment_revision='2'),
                                    dict(path_state='INVALIDATED'), dict(path_state='NONE')])
def test_diagnostic_plan_proof_must_match_active_session_revision(change):
    diagnostic = dict(session_id='current', segment_revision='3', path_state='ACTIVE',
                      cycle_result='PLAN_FOUND')
    assert plan_found_with_active_reference('current', REFS, [], [diagnostic])
    diagnostic.update(change)
    assert not plan_found_with_active_reference('current', REFS, [], [diagnostic])


def test_feedback_plan_proof_must_match_received_reference_revision():
    feedback = dict(reason_code='PLAN_FOUND', active_segment_revision=3)
    assert plan_found_with_active_reference('current', REFS, [feedback], [])
    feedback['active_segment_revision'] = 2
    assert not plan_found_with_active_reference('current', REFS, [feedback], [])
    assert not plan_found_with_active_reference('other', REFS, [feedback], [])


@pytest.mark.parametrize('linear,angular', [(0.2, 0.), (0., .2), (math.nan, 0.),
                                            (0., math.nan), (math.inf, 0.), (0., -math.inf)])
def test_completion_requires_finite_measured_stop(linear, angular):
    item = dict(session_id='current', segment_revision=3, state=5,
                linear_speed_mps=0., angular_speed_radps=0.)
    assert matching_stopped_completion(item, 'current', 3, REFS, 5)
    item.update(linear_speed_mps=linear, angular_speed_radps=angular)
    assert not matching_stopped_completion(item, 'current', 3, REFS, 5)


@pytest.mark.parametrize('change', [dict(session_id='old'), dict(segment_revision=2), dict(state=3)])
def test_completion_identity_and_phase_remain_required(change):
    item = dict(session_id='current', segment_revision=3, state=5,
                linear_speed_mps=0., angular_speed_radps=0.)
    item.update(change)
    assert not matching_stopped_completion(item, 'current', 3, REFS, 5)


def test_moving_update_has_no_stop_and_deduplicates_feedback():
    from lunar_incremental_controller_demo.evidence import summarize_rolling_events
    events = [dict(t=0., kind='tracking', session='a', revision=1, speed=.1, phase='TRACKING'),
              dict(t=.1, kind='reference', session='a', revision=2, final=False),
              dict(t=.2, kind='tracking', session='a', revision=2, speed=.1, phase='TRACKING'),
              dict(t=.3, kind='tracking', session='a', revision=2, speed=.1, phase='TRACKING')]
    result = summarize_rolling_events(events)
    assert result['moving_updates'] == 1
    assert result['update_stops'] == 0
    events.extend([dict(t=.4, kind='tracking', session='a', revision=2, speed=0., phase='BRAKING'),
                   dict(t=.5, kind='tracking', session='a', revision=2, speed=0., phase='BRAKING')])
    assert summarize_rolling_events(events)['update_stops'] == 1


def test_old_session_and_final_alignment_are_not_update_stops():
    from lunar_incremental_controller_demo.evidence import summarize_rolling_events
    events = [dict(t=0., kind='tracking', session='old', revision=1, speed=.1, phase='TRACKING'),
              dict(t=.1, kind='reference', session='new', revision=1, final=True),
              dict(t=.2, kind='tracking', session='new', revision=1, speed=0., phase='FINAL_ALIGN')]
    result=summarize_rolling_events(events)
    assert result['moving_updates']==0
    assert result['update_stops']==0


def test_braking_at_final_endpoint_is_not_an_update_induced_stop():
    from lunar_incremental_controller_demo.evidence import summarize_rolling_events
    events = [dict(t=0.,kind='tracking',session='a',revision=1,speed=.1,phase='TRACKING'),
              dict(t=.1,kind='reference',session='a',revision=2,final=True,length_m=.2),
              dict(t=.2,kind='tracking',session='a',revision=2,speed=.1,phase='BRAKING',progress=.15)]
    assert summarize_rolling_events(events)['update_stops']==0
