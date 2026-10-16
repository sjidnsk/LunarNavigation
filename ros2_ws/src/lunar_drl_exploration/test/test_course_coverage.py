from dataclasses import replace
from copy import deepcopy
import numpy as np
import pytest
from lunar_drl_exploration.ordered_curriculum import OrderedCourse
from lunar_drl_exploration.replay import ReplayBuffer
from test_replay_checkpoint import scene, transition


def end(course,coverage,env=0):
    course.activate(env,'moon_inside');course.admitted(env)
    course.observe(env,terminated=False,truncated=True,coverage=coverage,collision=False)


def test_mean_is_episode_weighted_and_ignores_inflight_and_interruptions():
    c=OrderedCourse(0);end(c,.2);end(c,.8)
    c.activate(1,'moon_inside');c.admitted(1)
    c.observe(1,terminated=False,truncated=False,coverage=.01,collision=False)
    stats=c.summary()['coverage_stats']['moon_inside']
    assert stats['count']==2 and stats['sum']/stats['count']==pytest.approx(.5)
    restored=OrderedCourse(0,state=c.state_dict())
    assert restored.summary()['coverage_stats']==c.summary()['coverage_stats']


def test_recent_window_is_bounded_and_promotion_keeps_task_history():
    c=OrderedCourse(0)
    for n in range(25):end(c,n/25)
    stats=c.summary()['coverage_stats']['moon_inside']
    assert stats['count']==25 and len(stats['recent'])==20
    assert stats['recent']==pytest.approx([n/25 for n in range(5,25)])
    assert OrderedCourse(1,state=c.state_dict()).summary()['coverage_stats']==c.summary()['coverage_stats']


def test_bad_terminal_coverage_cannot_increment_counters():
    c=OrderedCourse(0);c.activate(0,'moon_inside');c.admitted(0)
    before=deepcopy(c.summary())
    with pytest.raises(ValueError,match='coverage'):
        c.observe(0,terminated=True,truncated=False,coverage=float('nan'),collision=False)
    assert c.summary()==before


def legacy_course(ended):
    c=OrderedCourse(0)
    for _ in range(ended):end(c,.5)
    state=c.state_dict();state.pop('coverage_stats',None)
    return OrderedCourse(0,state=state)


def test_legacy_backfill_reports_partial_history_and_never_uses_current_steps():
    c=legacy_course(6);r=ReplayBuffer(10_000_000)
    truth=replace(scene(),generator_descriptor={'curriculum_task':'moon_inside'})
    tr=transition()
    r.add(replace(tr,truncated=True,episode_id='ended-a'),{'s':truth})
    r.add(replace(tr,truncated=True,episode_id='ended-b'),{'s':truth})
    r.add(replace(tr,episode_id='inflight'),{'s':truth})
    c.backfill_coverage(r)
    stats=c.summary()['coverage_stats']['moon_inside']
    assert stats['count']==2 and c.by_task['moon_inside']['ended']==6
    assert stats['sum']==pytest.approx(1.)  # each fixture observes one of two reference cells
    c.backfill_coverage(r)
    assert c.summary()['coverage_stats']['moon_inside']['count']==2
    assert OrderedCourse(0,state=c.state_dict()).summary()['coverage_stats']==c.summary()['coverage_stats']


def test_invalid_saved_coverage_is_rejected():
    c=OrderedCourse(0);end(c,.5);state=c.state_dict()
    state['coverage_stats']['moon_inside']['count']=2
    with pytest.raises(ValueError,match='coverage'):OrderedCourse(0,state=state)


def test_dashboard_has_current_task_mean_recent_mean_and_missing_history(tmp_path):
    import sys
    from pathlib import Path
    sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts/drl/task_frontier'))
    from course_dashboard import lines
    c=OrderedCourse(0);end(c,.2);end(c,.8);s=c.summary()
    s['by_task']['moon_inside']['ended']=6
    value='\n'.join(lines(tmp_path,dict(course_phase=0,phase='training'),dict(course=s,environments={}),s,{}))
    assert '平均最终覆盖率' in value and '50.00%' in value and '最近20' in value
    assert '历史缺4' in value

def test_restore_rejects_inconsistent_short_window_sum():
    c=OrderedCourse(0);end(c,.1);state=c.state_dict()
    state['coverage_stats']['moon_inside']['sum']=.9
    with pytest.raises(ValueError,match='coverage'):OrderedCourse(0,state=state)


def test_old_task_partial_mean_shows_its_own_sample_count(tmp_path):
    import sys
    from pathlib import Path
    sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts/drl/task_frontier'))
    from course_dashboard import lines
    c=OrderedCourse(0);end(c,.5);state=c.state_dict()
    state['by_task']['moon_inside']['ended']=6
    old=OrderedCourse(1,state=state);s=old.summary()
    value='\n'.join(lines(tmp_path,dict(course_phase=1,phase='training'),dict(course=s,environments={}),s,{}))
    assert '50.00%(1)' in value
