"""Journey duration, execution liveness and map waiting have different clocks."""
import importlib.util
import math
import pytest


def test_navigation_budget_is_available():
    assert importlib.util.find_spec('lunar_drl_exploration.navigation_budget') is not None


def make(length=100.,rtf=30.):
    from lunar_drl_exploration.navigation_budget import NavigationBudget
    return NavigationBudget(start_wall_s=0.,start_simulation_s=0.,pose=(0.,0.,0.),
                            route_length_m=length,target_rtf=rtf)


@pytest.mark.parametrize('length,rtf,expected',[(30.,30.,120.),(100.,4.,217.5),
    (900.,30.,255.),(900.,4.,1717.5),(0.,30.,120.)])
def test_confirmed_journey_formula(length,rtf,expected):
    assert make(length,rtf).budget_s==pytest.approx(expected)


def tick(b,wall,sim,pose,**kw):
    return b.observe(wall_s=wall,simulation_s=sim,pose=pose,
                     accepted=kw.pop('accepted',True),feedback_wall_s=kw.pop('feedback_wall_s',wall),**kw)


def test_healthy_long_navigation_survives_the_old_120_second_cutoff():
    b=make(900.)
    for wall in range(1,159):
        assert tick(b,wall,wall*30,(wall*5.7,0.,0.)) is None
    assert b.budget_s==pytest.approx(255.)


def test_observed_slow_simulation_expands_budget_using_actual_not_target_rtf():
    b=make(100.)
    for wall in range(1,130):
        assert tick(b,wall,wall*4,(wall*.8,0.,0.)) is None
    assert b.effective_rtf==pytest.approx(4.)
    assert b.budget_s==pytest.approx(217.5)


def test_detour_route_is_used_instead_of_straight_line_distance():
    b=make(10.)
    assert tick(b,1.,30.,(.2,0.,0.),route_length_m=900.) is None
    assert b.route_length_m==900. and b.budget_s==255.


def test_repeated_route_publications_do_not_reset_journey_origin():
    b=make(900.)
    for wall in range(1,256):
        assert tick(b,wall,wall*30,(wall*.3,0.,0.),route_length_m=900.) is None
    assert tick(b,256.,7680.,(77.,0.,0.),route_length_m=900.)=='JOURNEY_BUDGET_EXCEEDED'


def test_map_updates_cannot_keep_a_waiting_goal_alive_forever():
    b=make()
    for wall in range(1,16):
        assert tick(b,wall,wall*30,(0.,0.,0.),planning_reason='WAITING_FOR_MAP') is None
    assert tick(b,16.,480.,(0.,0.,0.),planning_reason='WAITING_FOR_MAP')=='MAP_WAIT_TIMEOUT'


def test_pose_jitter_does_not_count_as_execution_progress():
    b=make()
    for wall in range(1,60):
        assert tick(b,wall,wall*30,((wall%2)*.001,0.,(wall%2)*.001)) is None
    assert tick(b,60.,1800.,(0.,0.,0.))=='EXECUTION_NO_PROGRESS'


def test_legitimate_turning_counts_as_progress_even_without_translation():
    b=make()
    for wall in range(1,100):
        yaw=math.atan2(math.sin(wall*.12),math.cos(wall*.12))
        assert tick(b,wall,wall*30,(0.,0.,yaw)) is None


def test_angle_wrap_does_not_fake_large_turning_progress():
    from lunar_drl_exploration.navigation_budget import NavigationBudget
    b=NavigationBudget(start_wall_s=0.,start_simulation_s=0.,pose=(0.,0.,math.pi-.001),
                       route_length_m=10.,target_rtf=30.)
    for wall in range(1,60):
        assert tick(b,wall,wall*30,(0.,0.,(-math.pi+.001) if wall%2 else (math.pi-.001))) is None
    assert tick(b,60.,1800.,(0.,0.,math.pi-.001))=='EXECUTION_NO_PROGRESS'


def test_acceptance_and_feedback_timeouts_are_not_journey_timeouts():
    b=make(900.)
    assert tick(b,10.,300.,(0.,0.,0.),accepted=False)=='ACTION_ACCEPT_TIMEOUT'
    b=make(900.)
    assert tick(b,1.,30.,(1.,0.,0.),feedback_wall_s=0.) is None
    assert tick(b,16.,480.,(2.,0.,0.),feedback_wall_s=0.)=='NAVIGATION_FEEDBACK_TIMEOUT'


def test_route_refresh_without_motion_cannot_hide_stall():
    b=make(900.)
    for wall in range(1,60):
        assert tick(b,wall,wall*30,(0.,0.,0.),route_length_m=900.+wall) is None
    assert tick(b,60.,1800.,(0.,0.,0.),route_length_m=960.)=='EXECUTION_NO_PROGRESS'


def test_incomplete_goal_has_bounded_deadline_even_if_robot_keeps_moving_in_circles():
    b=make(10.)
    for wall in range(1,121):
        assert tick(b,wall,wall*30,(math.sin(wall),math.cos(wall),0.),route_length_m=10.) is None
    assert tick(b,121.,3630.,(1.,0.,0.),route_length_m=10.)=='JOURNEY_BUDGET_EXCEEDED'


@pytest.mark.parametrize('length,rtf',[(math.nan,30.),(10.,0.),(-1.,30.),(10.,math.inf)])
def test_invalid_estimates_cannot_disable_protection(length,rtf):
    with pytest.raises(ValueError):make(length,rtf)


def adapter_fixture(monkeypatch):
    from collections import deque
    from types import SimpleNamespace as NS
    from lunar_drl_exploration.ros_messages import RosNavigationAdapter
    from threading import Lock
    a=RosNavigationAdapter.__new__(RosNavigationAdapter)
    a._execution_lock=Lock();a.diagnostics=deque(maxlen=64);a.planning_evidence=deque(maxlen=64)
    a.references=deque(maxlen=64);a.sent_goal=(10.,0.,0.)
    a._execution_feedback_wall_s=0.;a._execution_planning=None;a._execution_tracking=None
    a._execution_reference=None;a._execution_route=None
    a._handle=NS(accepted=True,goal_id=NS(uuid=[1]*16),status=2)
    a._send_future=NS(done=lambda:True);a._result_future=NS(done=lambda:False)
    monkeypatch.setattr('lunar_drl_exploration.ros_messages.time.monotonic',lambda:5.)
    return a


def test_full_route_length_not_just_local_reference_reaches_budget(monkeypatch):
    from types import SimpleNamespace as NS
    a=adapter_fixture(monkeypatch)
    point=lambda x,y:NS(pose=NS(position=NS(x=x,y=y)))
    a._route(NS(poses=[point(0,0),point(0,100),point(10,100),point(10,0)],header=NS(frame_id='map')))
    state=a.execution_state()
    assert state['route_length_m']==pytest.approx(210.)


def test_route_for_previous_target_cannot_inflate_new_goal_budget(monkeypatch):
    from types import SimpleNamespace as NS
    a=adapter_fixture(monkeypatch)
    point=lambda x,y:NS(pose=NS(position=NS(x=x,y=y)))
    a._route(NS(poses=[point(0,0),point(900,0)],header=NS(frame_id='map')))
    assert a.execution_state()['route_length_m'] is None


def test_previous_goal_map_wait_diagnostic_is_not_current_goal_wait(monkeypatch):
    from types import SimpleNamespace as NS
    a=adapter_fixture(monkeypatch)
    fields=[NS(key='session_id',value='02'*16),NS(key='cycle_result',value='WAITING_FOR_MAP')]
    a._diagnostic(NS(status=[NS(name='planning_cycle',values=fields)]))
    assert a.execution_state()['planning_reason'] is None
    fields[0].value='01'*16
    a._diagnostic(NS(status=[NS(name='planning_cycle',values=fields)]))
    assert a.execution_state()['planning_reason']=='WAITING_FOR_MAP'


def test_route_republish_does_not_fake_fresh_navigation_feedback(monkeypatch):
    from types import SimpleNamespace as NS
    a=adapter_fixture(monkeypatch)
    point=lambda x,y:NS(pose=NS(position=NS(x=x,y=y)))
    a._route(NS(poses=[point(0,0),point(10,0)],header=NS(frame_id='map')))
    assert a.execution_state()['feedback_wall_s']==0.


def test_one_slow_initial_tick_does_not_permanently_multiply_the_allowance():
    b=make(900.)
    assert tick(b,1.,.05,(.2,0.,0.)) is None
    assert tick(b,2.,30.05,(.4,0.,0.)) is None
    assert tick(b,3.,60.05,(.6,0.,0.)) is None
    assert b.budget_s==pytest.approx(255.)


def execution_fixture(monkeypatch,tmp_path,mode):
    from types import SimpleNamespace as NS
    from threading import Event
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    from lunar_drl_exploration.contracts import Pose
    from lunar_drl_exploration.ros_messages import NavigationResult
    env=RosExplorationEnv.__new__(RosExplorationEnv)
    clock=[100.]
    env.wall_timeout_s=120.;env.env_id=157;env._cancel=Event()
    env.config=NS(target_rtf=30.,artifact_root=str(tmp_path))
    env.owned=NS(error_tail=lambda:'',pids=[])
    env.plant=NS(pose=Pose(0.,0.,0.),simulation_s=0.,collision=False,linear_mps=0.,angular_radps=0.)
    def pump():
        clock[0]+=1.;env.plant.simulation_s+=30.
        if mode=='moving':env.plant.pose=Pose(min(900.,env.plant.pose.x+6.),0.,0.)
    env._pump=pump
    env.adapter=NS(begin_goal=lambda g:None,cancel_goal=lambda:None,
        poll_goal=lambda:NavigationResult(0,'GOAL_REACHED',1) if env.plant.pose.x>=900. else None,
        execution_state=lambda:dict(accepted=True,feedback_wall_s=clock[0],route_length_m=900.,
            planning_reason='WAITING_FOR_MAP' if mode=='waiting' else 'PLAN_FOUND'))
    monkeypatch.setattr('lunar_drl_exploration.ros_env.time.monotonic',lambda:clock[0])
    return env


def test_actual_execution_loop_survives_120_seconds_of_healthy_long_drive(monkeypatch,tmp_path):
    env=execution_fixture(monkeypatch,tmp_path,'moving')
    result=env._execute_goal((900.,0.,0.))
    assert result.reason_code=='GOAL_REACHED'
    assert env.last_navigation_budget['elapsed_wall_s']>=149.
    assert env.last_navigation_budget['budget_s']==pytest.approx(255.)


@pytest.mark.parametrize('mode,reason',[('waiting','MAP_WAIT_TIMEOUT'),('stalled','EXECUTION_NO_PROGRESS')])
def test_execution_guard_preserves_failure_snapshot(monkeypatch,tmp_path,mode,reason):
    import json
    from lunar_drl_exploration.ros_env import InfrastructureError
    env=execution_fixture(monkeypatch,tmp_path,mode)
    with pytest.raises(InfrastructureError,match=reason):env._execute_goal((900.,0.,0.))
    paths=list((tmp_path/'navigation-watchdogs').glob('*.json'))
    assert len(paths)==1
    evidence=json.loads(paths[0].read_text())
    assert evidence['reason']==reason
    assert evidence['goal']==[900.,0.,0.]
    assert evidence['budget']['route_length_m']==900.
    assert evidence['actual_pose']==[0.,0.,0.]


def test_unrelated_diagnostics_do_not_refresh_current_goal_heartbeat(monkeypatch):
    from types import SimpleNamespace as NS
    a=adapter_fixture(monkeypatch)
    a._diagnostic(NS(status=[NS(values=[NS(key='applied_map_count',value='20')])]))
    assert a.execution_state()['feedback_wall_s']==0.
    a._diagnostic(NS(status=[NS(values=[NS(key='session_id',value='02'*16),NS(key='cycle_result',value='PLAN_FOUND')])]))
    assert a.execution_state()['feedback_wall_s']==0.


def test_old_tracking_does_not_hide_newer_map_wait(monkeypatch):
    from types import SimpleNamespace as NS
    a=adapter_fixture(monkeypatch)
    a._execution_tracking=dict(session_id='01'*16,reason='TRACKING',received_wall_s=4.)
    fields=[NS(key='session_id',value='01'*16),NS(key='cycle_result',value='WAITING_FOR_MAP')]
    a._diagnostic(NS(status=[NS(values=fields)]))
    assert a.execution_state()['planning_reason']=='WAITING_FOR_MAP'
