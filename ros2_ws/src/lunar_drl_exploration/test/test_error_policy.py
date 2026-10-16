import copy
import pytest
from test_course_two_parallel import pool,assert_reaped
from test_region_relation_evaluation import fixture
from lunar_drl_exploration import course_two_control as ctl
from lunar_drl_exploration.region_relation_evaluation import summarize


def test_worker_exit_counts_failure_and_pool_continues_then_resumes(pool):
    parallel,controller,actor,cases,commands=pool
    ctl.write(controller.root/'evaluation-settings.json',dict(workers=2,error_policy='count_failure'))
    cases[1]['case_id']='bad'
    rows=controller.screen(actor,cases,'round')
    assert len(rows)==6
    failed=next(row for row in rows if row['case_id']=='bad')
    assert failed['completed'] is False and failed['evaluation_failure_kind']=='infrastructure'
    assert failed['final_coverage'] is None and '7' in failed['error']
    assert (controller.root/'c5.started').exists()
    assert_reaped(controller.root)
    commands.clear();assert len(controller.screen(actor,cases,'round'))==6
    assert not commands


def test_timeout_result_is_counted_without_rewriting_original_proof(pool):
    parallel,controller,actor,cases,_=pool
    ctl.write(controller.root/'evaluation-settings.json',dict(workers=2,error_policy='count_failure'))
    proof=dict(actor_sha=ctl.sha(actor),case=cases[0],config_sha=ctl.sha(controller.root/'config.json'),runtime='test-runtime')
    record=dict(status='failed',proof=proof,row=dict(case_id='c0',kind='new',completed=False,collisions=0,error='InfrastructureError: navigation watchdog',trajectory=[[0,0,0,.37]],initial_completed=False))
    path=controller.root/'evaluation/round/c0.json';ctl.write(path,record);before=ctl.sha(path)
    result=parallel.cached_result(controller.root,path,proof)
    assert result['row']['evaluation_failure_kind']=='infrastructure'
    assert result['row']['final_coverage']==.37
    assert ctl.sha(path)==before
    with pytest.raises(ValueError,match='identity'):
        parallel.cached_result(controller.root,path,dict(proof,actor_sha='foreign'))


def test_failure_stays_in_success_denominator_and_reports_error():
    from lunar_drl_exploration.evaluation_error_policy import counted_failure
    manifest,rows=fixture()
    rows[0]=counted_failure(dict(rows[0],error='InfrastructureError: watchdog',completed=False,final_coverage=.42))
    result=summarize(manifest,rows)
    assert result['by_task']['moon_inside']['episodes']==60
    assert result['by_task']['moon_inside']['successes']==59
    assert result['by_task']['moon_inside']['infrastructure_failures']==1
    assert result['by_task']['moon_inside']['success_rate']==59/60


def test_failure_before_observation_does_not_invent_coverage_or_collision():
    from lunar_drl_exploration.evaluation_error_policy import counted_failure
    manifest,rows=fixture();row=copy.deepcopy(rows[0]);row.pop('final_coverage');row['collisions']=None;row['error']='WorkerExit: 7'
    rows[0]=counted_failure(row)
    result=summarize(manifest,rows)['by_task']['moon_inside']
    assert result['successes']==59 and result['episodes']==60
    assert result['coverage_known_episodes']==59 and result['unknown_collision_episodes']==1
