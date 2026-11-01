"""标准Path发布必须有界，真实耗时不能靠重复认证追赶。"""
from types import SimpleNamespace as S
import pytest
from threading import Lock
from lunar_pure_wheeled_controller.node import PureWheeledControllerNode

@pytest.mark.parametrize('case', ['slow_certificate', 'map_changes', 'map_missing', 'clock_regressed', 'unsafe', 'overflow', 'valid'])
def test_publication_certification_is_single_pass_and_checks_completion(case):
    clock=[10_000_000_000]
    grid=S(revision=1,epoch='1',origin=(0.,0.))
    newer=S(revision=2,epoch='1',origin=(0.,0.))
    calls=[];published=[];unavailable=[]
    def update(*args,**kwargs):
        clock[0]+=60_000_000
        if case=='overflow':node._execution_ingress_overflow=True
        return S(command=S(linear_x_mps=.1,angular_z_radps=0.),phase='TRACKING')
    def certify(*args):
        calls.append(args)
        # 第一轮慢，第二轮快：旧循环会追赶成功并错误发布。
        if case=='slow_certificate' and len(calls)==1:clock[0]+=60_000_000
        elif case=='clock_regressed':clock[0]-=1_000_000_000
        else:clock[0]+=1_000_000
        return case!='unsafe'
    def latest(state):
        if case=='map_missing' and calls:return None
        return newer if case=='map_changes' and calls else grid
    executor=S(config=S(compute_budget_s=.05),request_stop=lambda *a:None,
        set_map=lambda *a:None,update=update,certify_publication=certify,
        diagnostics={},_stopped=lambda *a:False)
    node=S(_feedback_lock=Lock(),_execution_ingress_overflow=False,_odometry=object(),_odometry_received_at=0.,_odometry_timeout_s=.5,
        _active=object(),_path_received_at=0.,_path_timeout_s=.35,
        _standard_state=object(),_standard_state_ns=clock[0],_last_control_ns=None,
        _executor=executor,_input_is_stale=lambda *a:False,_standard_grid=latest,
        get_clock=lambda:S(now=lambda:S(nanoseconds=clock[0])),
        _standard_unavailable=lambda reason,**kw:unavailable.append((reason,kw)),_mppi_diagnostics=S(publish=lambda *a:None),
        _publish_twist=lambda *a:published.append(a),_publish_execution_status=lambda:None)
    PureWheeledControllerNode._tick_standard_path(node)
    assert len(calls)==1
    if case=='valid':
        assert published==[(.1,0.)] and not unavailable
    else:
        assert not published and unavailable
        if case=='overflow':assert unavailable==[('execution_ingress_overflow',{})]
        if case=='map_missing':assert unavailable==[('stale_map_after_certification',{})]
        if case=='clock_regressed':assert unavailable==[('clock_regressed_during_certification',{})]


def test_new_feedback_cannot_renew_the_state_used_by_this_calculation():
    clock=[10_000_000_000];received=[0.];reasons=[];published=[]
    grid=S(revision=1,epoch='1',origin=(0.,0.))
    def update(*a,**kw):
        clock[0]+=600_000_000
        node._odometry_received_at=.6  # 模拟新消息；本轮state仍来自0秒。
        return S(command=S(linear_x_mps=.1,angular_z_radps=0.),phase='TRACKING')
    node=S(_feedback_lock=Lock(),_execution_ingress_overflow=False,_odometry=object(),_odometry_received_at=0.,_odometry_timeout_s=.5,
        _active=object(),_path_received_at=0.,_path_timeout_s=.35,
        _standard_state=object(),_standard_state_ns=clock[0],_last_control_ns=None,
        _executor=S(config=S(compute_budget_s=.05),set_map=lambda *a:None,update=update,
            diagnostics={},_stopped=lambda *a:False,certify_publication=lambda *a:True),
        _input_is_stale=lambda stamp,limit:(clock[0]-10_000_000_000)/1e9-stamp>limit,
        _standard_grid=lambda state:grid,get_clock=lambda:S(now=lambda:S(nanoseconds=clock[0])),
        _standard_unavailable=reasons.append,_mppi_diagnostics=S(publish=lambda *a:None),
        _publish_twist=lambda *a:published.append(a),_publish_execution_status=lambda:None)
    PureWheeledControllerNode._tick_standard_path(node)
    assert reasons==['stale_feedback_after_optimization'] and not published


@pytest.mark.parametrize('reason,retryable,expected_state',[
    ('publication_certificate_expired',True,1),('map_changed_during_certification',True,1),
    ('missing_or_stale_feedback',False,0),('missing_or_stale_map',False,0),('latest_map_or_aged_motion_unsafe',False,6)])
def test_candidate_invalidation_is_not_a_permanent_tracking_failure(reason,retryable,expected_state):
    import json
    emitted=[];statuses=[];diagnostics=[];discarded=[]
    node=S(_executor=S(execution_reference=None,_stop_reason='WAITING_FOR_POSE',
        wait_for_pose=lambda:discarded.append(True),discard_prediction=lambda:discarded.append(True)),
        _mppi_diagnostics=S(publish=lambda m:diagnostics.append(json.loads(m.data))),
        _publish_twist=lambda:emitted.append(0),
        _publish_execution_status=lambda **kw:statuses.append(kw))
    node._standard_pose_wait=lambda reason='WAITING_FOR_POSE':PureWheeledControllerNode._standard_pose_wait(node,reason)
    PureWheeledControllerNode._standard_unavailable(node,reason,retryable=retryable)
    assert emitted==[0]
    assert discarded==([] if retryable else [True])
    assert statuses[0]['state_code']==expected_state
    assert diagnostics[0]['certified_stop'] is False and diagnostics[0]['measured_stopped'] is None


def test_real_executor_failure_wins_over_retryable_map_race():
    statuses=[];discarded=[]
    node=S(_executor=S(failure='TRACKING_STALLED',last_execution_state=6,
        discard_prediction=lambda:discarded.append(True)),_mppi_diagnostics=S(publish=lambda m:None),
        _publish_twist=lambda:None,_publish_execution_status=lambda **kw:statuses.append(kw))
    PureWheeledControllerNode._standard_unavailable(node,'map_changed_during_certification',retryable=True)
    assert statuses[0]['state_code']==6
    assert statuses[0]['reason']=='EXECUTION_INPUT_UNAVAILABLE:TRACKING_STALLED'
    assert discarded==[True]


def test_missing_pose_requests_zero_and_reports_unaccepted_current_pair():
    calls=[];offered=object();active=object()
    executor=S(failure=None,_stop_reason='WAITING_FOR_POSE',execution_reference=active,wait_for_pose=lambda:calls.append('retire'))
    n=S(_executor=executor,_execution_offered=offered,_active=object(),_path_received_at=1.,_last_control_ns=1.,
        _mppi_diagnostics=S(publish=lambda *a:None),_publish_twist=lambda *a:calls.append(('command',a)),
        _publish_execution_status=lambda **kw:calls.append(('status',kw)))
    n._standard_pose_wait=lambda reason='WAITING_FOR_POSE':PureWheeledControllerNode._standard_pose_wait(n,reason)
    PureWheeledControllerNode._standard_unavailable(n,'missing_or_stale_feedback')
    assert ('command',()) in calls and n._active is None and n._last_control_ns is None
    status=next(v for k,v in (v for v in calls if isinstance(v,tuple)) if k=='status')
    assert status==dict(reference=offered,state_code=0,accepted=False,reason='WAITING_FOR_POSE')


def test_pose_input_loss_cannot_recover_an_existing_terminal_failure():
    statuses=[];calls=[]
    n=S(_executor=S(failure='UNSAFE',last_execution_state=6,discard_prediction=lambda:None),
        _standard_pose_wait=lambda:calls.append('wrong_recovery'),_mppi_diagnostics=S(publish=lambda *a:None),
        _publish_twist=lambda *a:None,_publish_execution_status=lambda **kw:statuses.append(kw))
    PureWheeledControllerNode._standard_unavailable(n,'missing_or_stale_feedback')
    assert not calls and statuses[0]['state_code']==6 and 'UNSAFE' in statuses[0]['reason']


@pytest.mark.parametrize('case',['future','old_source','missing_map','valid'])
def test_waiting_for_reference_still_checks_source_time_and_map(case):
    clock=10_000_000_000;reasons=[];wait=[]
    source=clock+1 if case=='future' else clock-600_000_000 if case=='old_source' else clock
    n=S(_odometry=object(),_odometry_received_at=1.,_odometry_timeout_s=.5,_active=None,
        _executor=S(_stop_reason='WAITING_FOR_POSE'),_input_is_stale=lambda *a:False,
        _standard_state=object(),_standard_grid=lambda state:None if case=='missing_map' else object(),
        get_clock=lambda:S(now=lambda:S(nanoseconds=clock)),_standard_state_ns=source,
        _standard_unavailable=reasons.append,_standard_pose_wait=wait.append)
    PureWheeledControllerNode._tick_standard_path(n)
    expected={'future':'future_feedback','missing_map':'missing_or_stale_map'}
    assert reasons==([] if case in ('valid','old_source') else [expected[case]])
    assert wait==(['WAITING_FOR_FRESH_REFERENCE'] if case in ('valid','old_source') else [])


@pytest.mark.parametrize('crossing',['solve','certificate'])
def test_source_age_crossing_old_limit_keeps_certified_publication(crossing):
    clock=[10_000_000_000];reasons=[];published=[];queries=[]
    old=S(revision=1,epoch='1',origin=(0.,0.));new=S(revision=2,epoch='1',origin=(0.,0.))
    def update(*a,**kw):
        clock[0]+=20_000_000 if crossing=='solve' else 5_000_000
        return S(command=S(linear_x_mps=.1,angular_z_radps=0.),phase='TRACKING')
    def certify(*a):clock[0]+=20_000_000;return True
    def grid(state):queries.append(1);return old if len(queries)==1 else new
    n=S(_feedback_lock=Lock(),_execution_ingress_overflow=False,_odometry=object(),_odometry_received_at=1.,
        _odometry_timeout_s=.5,_active=object(),_path_received_at=1.,_path_timeout_s=.35,
        _standard_state=object(),_standard_state_ns=9_510_000_000,_last_control_ns=None,
        _executor=S(config=S(compute_budget_s=.2),request_stop=lambda *a:None,set_map=lambda *a:None,
            update=update,certify_publication=certify,diagnostics={},_stopped=lambda *a:False),_input_is_stale=lambda *a:False,
        _standard_grid=grid,get_clock=lambda:S(now=lambda:S(nanoseconds=clock[0])),
        _standard_unavailable=lambda reason,**kw:reasons.append(reason),
        _publish_twist=lambda *a:published.append(a),_publish_execution_status=lambda:None,
        _mppi_diagnostics=S(publish=lambda *a:None))
    PureWheeledControllerNode._tick_standard_path(n)
    assert not reasons and published==[(.1,0.)]


def test_failed_feedback_stays_failed_when_pose_returns_during_wait():
    statuses=[];wrong=[]
    executor=S(failure='INVALID_FEEDBACK',_stop_reason='WAITING_FOR_POSE',execution_reference=object(),
        last_execution_state=0,discard_prediction=lambda:None,wait_for_pose=lambda:wrong.append(True))
    n=S(_odometry=object(),_odometry_received_at=1.,_odometry_timeout_s=.5,_active=None,
        _executor=executor,_input_is_stale=lambda *a:False,_standard_state=object(),
        _standard_grid=lambda state:object(),get_clock=lambda:S(now=lambda:S(nanoseconds=10**10)),
        _standard_state_ns=10**10,_mppi_diagnostics=S(publish=lambda *a:None),_publish_twist=lambda:None,
        _publish_execution_status=lambda **kw:statuses.append(kw))
    n._standard_unavailable=lambda reason:PureWheeledControllerNode._standard_unavailable(n,reason)
    n._standard_pose_wait=lambda reason:PureWheeledControllerNode._standard_pose_wait(n,reason)
    PureWheeledControllerNode._tick_standard_path(n)
    assert not wrong and statuses[-1]['state_code']==6
    assert statuses[-1]['reason'].endswith('INVALID_FEEDBACK')


def test_delayed_source_is_forwarded_to_model_without_source_age_veto():
    age=[];emitted=[];errors=[];clock=10_000_000_000
    grid=S(revision=1,epoch='1',origin=(0.,0.))
    node=S(_feedback_lock=Lock(),_execution_ingress_overflow=False,_odometry=object(),
        _odometry_received_at=0.,_odometry_advanced_at=0.,_odometry_timeout_s=.5,
        _active=object(),_path_received_at=0.,_path_timeout_s=.35,
        _standard_state=object(),_standard_state_ns=1_000_000_000,_last_control_ns=None,
        _executor=S(config=S(compute_budget_s=.05),set_map=lambda *a:None,
            update=lambda *a,**kw:(age.append(kw['state_age_s']) or S(command=S(linear_x_mps=.1,angular_z_radps=0.),phase='TRACKING')),
            diagnostics={},_stopped=lambda *a:False,certify_publication=lambda *a:True),
        _input_is_stale=lambda *a:False,_standard_grid=lambda state:grid,
        get_clock=lambda:S(now=lambda:S(nanoseconds=clock)),
        _standard_unavailable=errors.append,_mppi_diagnostics=S(publish=lambda *a:None),
        _publish_twist=lambda *a:emitted.append(a),_publish_execution_status=lambda:None)
    PureWheeledControllerNode._tick_standard_path(node)
    assert age==[9.] and emitted==[(.1,0.)] and not errors


def test_only_new_source_stamp_renews_controller_continuity(monkeypatch):
    import nav1cm.ros_conversion as conversion
    monkeypatch.setattr(conversion,'state_from_odometry',lambda *a:S(x=0.,y=0.,yaw=0.,v=0.,w=0.))
    n=S(_executor=S(failure=None),_latest_transforms=object(),_map_frame='map',_p3_source=None)
    def feed(source,received,ros):
        message=S(header=S(stamp=S(sec=source,nanosec=0)),twist=S(twist=S(linear=S(y=0.))))
        n._standard_pending_odometry=(message,received,ros*10**9)
        PureWheeledControllerNode._accept_standard_odometry(n)
    feed(90,10.,100)
    assert n._standard_state_ns==90*10**9 and n._odometry_advanced_at==10.
    feed(90,14.,104)
    assert n._odometry_received_at==14. and n._odometry_advanced_at==10.
    feed(92,15.,105)
    assert n._odometry_advanced_at==15. and n._executor.failure is None
    feed(91,16.,106)
    assert n._executor.failure=='INVALID_FEEDBACK_TIMESTAMP' and n._odometry is None


@pytest.mark.parametrize('hard_failure',[False,True])
def test_input_fault_uses_current_offered_identity_but_execution_failure_keeps_active(hard_failure):
    statuses=[];active=object();offered=object()
    n=S(_execution_offered=offered,
        _executor=S(failure='UNSAFE' if hard_failure else None,last_execution_state=0,
                    execution_reference=active,discard_prediction=lambda:None,
                    _stop_reason='WAITING_FOR_POSE',wait_for_pose=lambda:None),
        _mppi_diagnostics=S(publish=lambda msg:None),_publish_twist=lambda:None,
        _publish_execution_status=lambda **kw:statuses.append(kw))
    n._standard_pose_wait=lambda reason='WAITING_FOR_POSE':PureWheeledControllerNode._standard_pose_wait(n,reason)
    PureWheeledControllerNode._standard_unavailable(n,'missing_or_stale_map')
    status=statuses[-1]
    if hard_failure:
        assert status['state_code']==6 and status.get('reference',active) is active and status['reason'].endswith('UNSAFE')
    else:
        assert status['state_code']==0 and status['reason']=='WAITING_FOR_MAP'
        assert status['reference'] is offered and status['accepted'] is False


@pytest.mark.parametrize('reason',['WAITING_FOR_POSE','WAITING_FOR_FRESH_REFERENCE'])
@pytest.mark.parametrize('previously_accepted',[False,True])
def test_braking_recovery_heartbeat_belongs_to_current_offered_reference(reason,previously_accepted):
    from test_terminal_execution_contract import ref
    from builtin_interfaces.msg import Time
    emitted=[];active=ref(session='a'*32);offered=ref(2,4,session='a'*32)
    executor=S(execution_reference=active,_stop_reason=reason,failure=None,
               reference_accepted=previously_accepted,last_execution_state=1)
    n=S(_execution_enabled=True,_executor=executor,_execution_offered=offered,
        _map_frame='map',get_clock=lambda:S(now=lambda:S(to_msg=lambda:Time(sec=1))),
        _tracking_status=S(publish=emitted.append))
    PureWheeledControllerNode._publish_execution_status(n)
    assert emitted[0].segment_revision==offered.revision
    assert emitted[0].state==1 and not emitted[0].reference_accepted


@pytest.mark.parametrize('case', ['window', 'aged_window', 'world', 'world_twice',
    'policy_revision', 'policy_twice', 'source_latest', 'source_final',
    'missing', 'expired', 'unsafe', 'aged_unsafe'])
def test_map_proof_uses_source_specific_world_and_fixed_snapshot(case):
    import json
    clock=[10_000_000_000];calls=[];maps=[];published=[];reasons=[];diagnostics=[]
    a=S(revision=1,epoch='1',origin=(0.,0.))
    b=S(revision=2,epoch='2' if case in ('world','world_twice','unsafe') else '1',origin=(1.,0.))
    c=S(revision=3,epoch='3' if case=='world_twice' else b.epoch,origin=(2.,0.))
    source=object();queries=[]
    def grid(state):
        queries.append(1)
        if case=='source_latest' and len(queries)==2:n._p3_source=object()
        if case=='source_final' and len(queries)==3:n._p3_source=object()
        if case=='missing' and len(queries)==3:return None
        return [a,b,c][len(queries)-1]
    def update(*args,**kwargs):
        clock[0]+=60_000_000 if case in ('aged_window','aged_unsafe','expired') else 1_000_000
        return S(command=S(linear_x_mps=.1,angular_z_radps=0.),phase='TRACKING')
    def certify(*args):
        calls.append(maps[-1])
        clock[0]+=60_000_000 if case=='expired' else 1_000_000
        return case not in ('unsafe','aged_unsafe')
    # Policy revision tests retain fixed origin: revision alone changes content.
    if case.startswith('policy'):
        b.origin=c.origin=a.origin
        if case=='policy_revision':c=b
    n=S(_p3_source=None if case.startswith('policy') else source,
        _feedback_lock=Lock(),_execution_ingress_overflow=False,_odometry=object(),
        _odometry_received_at=0.,_odometry_timeout_s=.5,_active=object(),
        _standard_state=object(),_standard_state_ns=clock[0],_last_control_ns=None,
        _executor=S(config=S(compute_budget_s=.05),set_map=maps.append,update=update,
            certify_publication=certify,diagnostics={},_stopped=lambda *a:False),
        _input_is_stale=lambda *a:False,_standard_grid=grid,
        get_clock=lambda:S(now=lambda:S(nanoseconds=clock[0])),
        _standard_unavailable=lambda reason,**kw:reasons.append(reason),
        _publish_twist=lambda *a:published.append(a),_publish_execution_status=lambda:None,
        _mppi_diagnostics=S(publish=lambda msg:diagnostics.append(json.loads(msg.data))))
    PureWheeledControllerNode._tick_standard_path(n)
    success=case in ('window','aged_window','world','policy_revision')
    assert bool(published)==success
    if case=='window':assert calls==[] and diagnostics[-1]['map_revision']==a.revision
    if case=='aged_window':assert calls==[a] and diagnostics[-1]['map_revision']==a.revision
    if case in ('world','policy_revision'):assert calls==[b] and diagnostics[-1]['map_revision']==b.revision
    if case in ('world_twice','policy_twice','source_final'):assert reasons==['map_changed_during_certification']
    if case=='source_latest':assert reasons==['map_changed_during_certification'] and not calls
    if case=='missing':assert reasons==['stale_map_after_certification']
    if case=='expired':assert reasons==['publication_certificate_expired'] and calls==[a]
    if case in ('unsafe','aged_unsafe'):assert reasons==['latest_map_or_aged_motion_unsafe']
    if case=='aged_unsafe':assert calls==[a]
