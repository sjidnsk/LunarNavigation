"""联合证据门禁不可把局部Path或传输接受当实际探索到达。"""
from importlib.util import spec_from_file_location,module_from_spec
from pathlib import Path
import pytest

FILE=Path(__file__).resolve().parents[3]/"scripts/navigation/one_cm/frontier_validation/grade.py"
def load():
    assert FILE.exists(), "前沿联合门禁尚未实现"
    spec=spec_from_file_location("frontier_grade",FILE);m=module_from_spec(spec);spec.loader.exec_module(m);return m

def evidence():
    return dict(case="frontier_progress",common={k:True for k in ("graph","formal","motion","bounds","clearance","cleanup","installed","no_exceptions","source_received","sensor_timing")},
                reached=[dict(uuid="current",xy_error=.05,yaw_error=.02,v=0.,w=0.,hold_s=.3,status=4,outcome=0)],
                distinct_goals=2,observed_growth=20,waits=0,canceled_stopped=0,cancel_while_moving=False,no_goals_after_cancel=True)

def test_actual_terminal_requires_uuid_xy_yaw_stop_and_hold():
    grade=load().grade
    e=evidence();assert grade(e)["pass_"]
    for key,bad in [("uuid",""),("xy_error",.2),("yaw_error",.1),("v",.02),("w",.02),("hold_s",.1),("status",6),("outcome",5)]:
        x=evidence();x["reached"][0][key]=bad;assert not grade(x)["pass_"],key

def test_path_or_no_map_cannot_count_as_frontier_completion():
    e=evidence();e["reached"]=[];e["nonempty_paths"]=100
    assert not load().grade(e)["pass_"]

@pytest.mark.parametrize("key",["graph","formal","motion","bounds","clearance","cleanup","installed","no_exceptions","source_received","sensor_timing"])
def test_every_common_safety_and_identity_gate_is_independent(key):
    e=evidence();e["common"][key]=False;assert not load().grade(e)["pass_"]

def test_blocked_wait_requires_real_cancel_stop_and_distinct_frontend_goal():
    e=evidence();e.update(case="blocked_wait_reselect",waits=3,canceled_stopped=1,distinct_goals=2)
    assert load().grade(e)["pass_"]
    for k,v in [("waits",0),("canceled_stopped",0),("distinct_goals",1)]:
        x={**e,k:v};assert not load().grade(x)["pass_"]

def test_external_cancel_requires_measured_motion_stop_and_no_reselection():
    e=evidence();e.update(case="task_cancel_stop",canceled_stopped=1,cancel_while_moving=True)
    assert load().grade(e)["pass_"]
    e["no_goals_after_cancel"]=False;assert not load().grade(e)["pass_"]


def test_passive_action_probe_uuid_uses_python_wire_integers():
    import sys
    import numpy as np
    path=FILE.with_name("run.py")
    spec=spec_from_file_location("frontier_runner",path)
    module=module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    assert hasattr(module,"uuid_values"), "UUID旁观请求缺少wire整数规范化"
    values=module.uuid_values(np.arange(16,dtype=np.uint8))
    assert values==list(range(16)) and all(type(v) is int for v in values)


def test_queued_feedback_cannot_turn_plant_into_unbounded_odometry_publisher():
    import sys,time
    path=FILE.with_name("run.py");spec=spec_from_file_location("frontier_pacing",path)
    module=module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    assert hasattr(module,"pace_cycle"), "plant缺少独立50Hz时钟约束"
    # Windows单调时钟可有15.625ms量化；算法门禁用确定性时钟，实际Hz由ROS raw另验。
    from types import SimpleNamespace as NS
    clock=[0.]
    def sleep(seconds):clock[0]+=seconds
    module.time=NS(monotonic=lambda:clock[0],sleep=sleep)
    for _ in range(5):
        # 队列一直有事件，spin_once可立即返回；仍须遵守外部测量时钟。
        module.pace_cycle(clock[0])
    assert clock[0]==pytest.approx(.1)


def test_plant_services_busy_callback_queue_within_each_sensor_deadline():
    import sys,time
    path=FILE.with_name("run.py");spec=spec_from_file_location("frontier_callback_cycle",path)
    module=module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    assert hasattr(module,"service_cycle"), "plant每个测量周期只能消费一个callback"
    consumed=[];started=time.monotonic()
    module.service_cycle(started,lambda timeout:consumed.append(timeout))
    assert len(consumed)>1 and time.monotonic()-started>=.019


def test_blocked_v2_only_translates_declared_unknown_feature():
    import sys,numpy as np
    path=FILE.with_name("fixtures.py");spec=spec_from_file_location("frontier_fixture_version",path)
    module=module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    assert hasattr(module,"measurement_mask"), "fixture版本平移合同尚未实现"
    c=5-(np.arange(100)+.5)*.1;x,y=np.meshgrid(c,c,indexing='ij')
    a=module.measurement_mask(x,y,"blocked_wait_reselect","v1")
    b=module.measurement_mask(x,y,"blocked_wait_reselect","v2")
    allowed=(((x>=.9)&(x<=1.3))|((x>=1.6)&(x<=2.0)))&(abs(y)<=1.3)
    assert np.array_equal(a[~allowed],b[~allowed])
    assert a.sum()==b.sum() and not np.array_equal(a,b)


def test_terminal_hold_never_uses_future_state_samples():
    import sys
    path=FILE.with_name("run.py");spec=spec_from_file_location("frontier_strict_terminal",path)
    module=module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    rows=[dict(monotonic_s=t,x=0.,y=0.,yaw=0.,v=0.,w=0.) for t in (0.,.1,.2,.3)]
    assert module.held(rows,(0.,0.,0.),.26)["hold_s"]<=.26

def test_cancel_stop_measurement_must_precede_terminal_receipt():
    import sys
    path=FILE.with_name("run.py");spec=spec_from_file_location("frontier_strict_cancel",path)
    module=module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    assert hasattr(module,"measured_stopped_before"), "取消停稳缺少严格终态前测量门禁"
    assert not module.measured_stopped_before([dict(monotonic_s=.3,v=0.,w=0.)],.26)
    assert module.measured_stopped_before([dict(monotonic_s=.2,v=0.,w=0.)],.26)


def obstacle_evidence():
    e=evidence();e['case']='blocked_obstacle_update'
    e['obstacle_sequence']={
      'event':dict(uuid='a',monotonic_s=1.,pose=[2.,0.,0.],distance_m=2.,accepted_at=.9,active=True),
      'source':dict(monotonic_s=1.1,changed=True),
      'map':dict(monotonic_s=1.2,changed=True),
      'wait':dict(uuid='a',monotonic_s=1.3,reason='WAITING_FOR_PATH'),
      'terminal':dict(uuid='a',monotonic_s=3.3,status=5,outcome=5,reason='CANCELED_STOPPED',stopped_before=True),
      'next':dict(uuid='b',monotonic_s=3.4,pose=[2.,1.,0.],logical_accepted=True)}
    return e

def test_obstacle_update_requires_real_event_source_map_current_uuid_stop_and_new_xy():
    e=obstacle_evidence();assert load().grade(e)['pass_']

@pytest.mark.parametrize('fault',['no_event','no_source_change','no_map_change','wrong_wait_uuid','wrong_terminal_uuid','no_stop','only_yaw','not_logically_accepted','wrong_order'])
def test_obstacle_update_false_positive_evidence_is_rejected(fault):
    e=obstacle_evidence();s=e['obstacle_sequence']
    if fault=='no_event':s['event']=None
    elif fault=='no_source_change':s['source']['changed']=False
    elif fault=='no_map_change':s['map']['changed']=False
    elif fault=='wrong_wait_uuid':s['wait']['uuid']='stale'
    elif fault=='wrong_terminal_uuid':s['terminal']['uuid']='stale'
    elif fault=='no_stop':s['terminal']['stopped_before']=False
    elif fault=='only_yaw':s['next']['pose']=[2.,0.,1.]
    elif fault=='not_logically_accepted':s['next']['logical_accepted']=False
    elif fault=='wrong_order':s['terminal']['monotonic_s']=1.15
    assert not load().grade(e)['pass_']


def runner_for_fix():
    import sys
    spec=spec_from_file_location("frontier_fix_runner",FILE.with_name("run.py"))
    module=module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module);return module

def test_latest_invalid_measurement_revokes_historical_goal_hold():
    m=runner_for_fix()
    rows=[dict(monotonic_s=t,x=0.,y=0.,yaw=0.,v=0.,w=0.) for t in (0.,.1,.2,.31)]
    rows.append(dict(monotonic_s=.4,x=.2,y=0.,yaw=0.,v=.1,w=0.))
    assert m.held(rows,(0.,0.,0.),.41)['hold_s']==0.
    assert not m.measured_stopped_before([dict(monotonic_s=.3,v=0.,w=0.),dict(monotonic_s=.4,v=.1,w=0.)],.41)

def test_hold_is_latest_continuous_suffix_and_freshness_is_required():
    m=runner_for_fix()
    rows=[dict(monotonic_s=t,x=0.,y=0.,yaw=0.,v=0.,w=0.) for t in (0.,.1,.2,.31)]
    assert m.held(rows,(0.,0.,0.),.6)['hold_s']==0.
    assert not m.measured_stopped_before([dict(monotonic_s=.1,v=0.,w=0.)],.3)
    rows.append(dict(monotonic_s=.4,x=.2,y=0.,yaw=0.,v=.1,w=0.))
    rows.extend(dict(monotonic_s=t,x=0.,y=0.,yaw=0.,v=0.,w=0.) for t in (.5,.6))
    assert m.held(rows,(0.,0.,0.),.61)['hold_s']==pytest.approx(.1)

def acceptance_module():
    path=FILE.with_name('acceptance.py')
    assert path.exists(), '缺少权威任务接受身份账'
    spec=spec_from_file_location('frontier_acceptance',path);m=module_from_spec(spec);spec.loader.exec_module(m);return m

def test_status_current_goal_and_feedback_order_cannot_create_pose_binding():
    m=acceptance_module();ledger=m.AcceptanceLedger()
    a,b='a'*32,'b'*32
    assert ledger.get(a) is None and ledger.current is None
    # status/feedback/current_goal无权绑定；只有exact TASK_ACCEPTED可给坐标。
    ledger.record(dict(task_id=b,accepted_generation=2,goal=[2.,3.,.4]),2.)
    ledger.record(dict(task_id=a,accepted_generation=1,goal=[-1.,0.,.1]),3.)
    assert ledger.current.task_id==b and ledger.get(a).goal==(-1.,0.,.1)
    assert ledger.get(b).goal==(2.,3.,.4)
    ledger.finish(b)
    ledger.record(dict(task_id=b,accepted_generation=2,goal=[2.,3.,.4]),4.)
    assert ledger.current is None

def test_acceptance_conflicts_and_missing_binding_fail_closed():
    m=acceptance_module();ledger=m.AcceptanceLedger();a='a'*32
    assert ledger.get(a) is None
    ledger.record(dict(task_id=a,accepted_generation=1,goal=[0.,0.,0.]),1.)
    ledger.record(dict(task_id=a,accepted_generation=1,goal=[1.,0.,0.]),2.)
    assert not ledger.ok and ledger.current is None and ledger.get(a) is None
