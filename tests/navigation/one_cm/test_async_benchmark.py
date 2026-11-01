import numpy as np
import pytest
from dataclasses import replace
from time import perf_counter, sleep
import sys
import json
import hashlib
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'scripts/navigation/one_cm'))
import benchmark_async_navigation as benchmark
from scripts.navigation.one_cm.nav1cm.verification_plant import PlantModel, TimedPlan, predict_feedback
from scripts.navigation.one_cm.nav1cm.contracts import VehicleState, WheelLimits
from scripts.navigation.one_cm.nav1cm.verification_plant import IndependentPlant, WorkerDelay
from scripts.navigation.one_cm.nav1cm.verification_plant import prepare_handover, plan_from_candidate
from scripts.navigation.one_cm.nav1cm.contracts import Trajectory
from scripts.navigation.one_cm.nav1cm.local_worker import LocalWorker


def plan(seq=1, start=.1, model=None, feedback=None, limits=None):
    if model is not None: feedback=model.feedback(); limits=model.limits
    if feedback is None: return TimedPlan(seq,0.,start,(0.,.4),((.5,0.,0.),),0,1,'forward','test')
    expected=predict_feedback(feedback,start,limits)
    return TimedPlan(seq,feedback['sample_s'],start,(0.,.4),((.5,0.,0.),),0,1,'forward','test',tuple(expected[:5]),feedback['feedback_id'],feedback['sequence'],tuple(feedback['vector']))


def test_future_handover_and_prediction():
    model=PlantModel(VehicleState(0,0,0,0,0), WheelLimits(linear_speed_mps=.5))
    assert model.accept(plan(model=model), .01)=='ACCEPTED'
    snapshot=model.feedback()
    predicted=predict_feedback(snapshot,.2,model.limits)
    for _ in range(4): model.advance(.05)
    assert np.allclose(predicted,model.vector)
    assert model.vector[0]>0
    assert snapshot['sample_s']==0


def test_late_rejected_and_exhaustion_brakes_without_freeze():
    model=PlantModel(VehicleState(0,0,0,0,0),WheelLimits(linear_speed_mps=.5))
    assert model.accept(plan(),.11)=='LATE_DELIVERY'
    assert model.accept(plan(model=model),.01)=='ACCEPTED'
    for _ in range(24): model.advance(.05)
    assert model.sample_s>1
    assert model.vector[0]>0
    assert abs(model.vector[3])<1e-9
    assert model.last_source=='brake'


def test_target_reject_clears_motion_before_future_commands():
    model=PlantModel(VehicleState(0,0,0,.1,0),WheelLimits())
    model.accept(plan(model=model),.01)
    model.reject_target()
    assert model.accept(plan(2),.02)=='TARGET_REJECTED'
    model.advance(.05)
    assert model.vector[3]<.1


def test_pending_feedback_prediction_uses_same_curved_substeps():
    model=PlantModel(VehicleState(0,0,0,.1,.05),WheelLimits())
    curved=replace(plan(model=model),controls=((.1,.02,0.),))
    assert model.accept(curved,.01)=='ACCEPTED'
    predicted=predict_feedback(model.feedback(),.35,model.limits)
    for _ in range(7): model.advance(.05)
    assert np.allclose(predicted,model.vector,atol=1e-12)


def test_handover_measured_source_and_finite_hold():
    model=PlantModel(VehicleState(0,0,0,0,0),WheelLimits())
    feedback=model.feedback()
    prepared=prepare_handover(feedback,.012,model.limits)
    assert prepared['source_feedback_s']==0.
    assert prepared['prediction_target_s']==pytest.approx(.15)
    empty=Trajectory(0,0,[0.],[[0,0,0,0,0,0]],np.zeros((0,3)),'forward','brake')
    command=plan_from_candidate(empty,1,0.,.15,feedback=feedback)
    assert command.times==(0.,.05)
    assert command.controls==((0.,0.,0.),)
    assert model.accept(command,.01)=='ACCEPTED'
    assert prepare_handover(model.feedback(),.02,model.limits) is not None
    assert model.accept(replace(command,sequence=2,source_feedback_s=.01),.02)=='STALE_SOURCE'


def echo(request): return dict(request_identity=request['identity'],candidate=None)


def test_spawn_plant_advances_while_real_worker_busy():
    plant=IndependentPlant(VehicleState(0,0,0,0,0),WheelLimits(linear_speed_mps=.5),200)
    worker=LocalWorker(runner=echo,request_wrapper=WorkerDelay(.35))
    rows=[]
    try:
        deadline=perf_counter()+8
        while not any(r['kind']=='READY' for r in rows):
            rows.extend(plant.poll()); sleep(.01)
            assert perf_counter()<deadline
        origin=next(r['origin_wall_s'] for r in rows if r['kind']=='READY')
        # 预热测试worker，防止spawn时间与注入睡眠混淆。
        worker.submit(dict(identity=0))
        while worker.poll() is None:
            rows.extend(plant.poll()); sleep(.005)
            assert perf_counter()<deadline
        start=np.ceil((perf_counter()-origin+.1)/.05)*.05
        latest=next(r for r in reversed(rows) if r['kind'] in ('READY','FEEDBACK'))
        plant.send(plan(start=start,feedback=latest,limits=WheelLimits(linear_speed_mps=.5)))
        begin=perf_counter()
        worker.submit(dict(identity=1))
        while worker.poll() is None:
            rows.extend(plant.poll()); sleep(.005)
            assert perf_counter()<deadline
        rows.extend(plant.poll())
        inside=[r for r in rows if r['kind']=='FEEDBACK' and r['published_wall_s']>=begin]
        assert len(inside)>=4
        # 前一段计划可能已耗尽，制动期间仍有实际位移；不冻结反馈时钟。
        assert inside[-1]['sample_s']>inside[0]['sample_s']
        assert inside[-1]['vector'][0]>inside[0]['vector'][0]
    finally:
        worker.close(); plant.close()


def test_pending_does_not_gate_twenty_hz_core_targets():
    from scripts.navigation.one_cm.nav1cm.async_coordinator import AsyncNavigationCoordinator
    from scripts.navigation.one_cm.nav1cm.contracts import Route,WorldSnapshot,CellRect
    class Global:
        busy=True
        def poll(self): return None
        def close(self): pass
    class Local:
        busy=False
        def __init__(self): self.requests=[]; self.pending=None
        def submit(self,request): self.requests.append(request); self.pending=request; self.busy=True
        def poll(self):
            if self.pending is None: return None
            request,self.pending=self.pending,None; self.busy=False
            return dict(request_identity=request['identity'],candidate=None)
        def close(self): pass
    local=Local()
    core=AsyncNavigationCoordinator(local_worker=local,global_runner=Global(),limits=WheelLimits())
    core._goal=(3.,0.); core.route=Route(1,1,[[0,0],[3,0]],(3,0))
    world=WorldSnapshot(1,(),(CellRect(-1000,-1000,1500,1500),))
    model=PlantModel(VehicleState(0,0,0,0,0),WheelLimits())
    last=-1.; targets=[]
    for index in range(20):
        fb=model.feedback()
        prepared=prepare_handover(fb,index*.05,model.limits,last)
        assert prepared is not None
        last=prepared['prediction_target_s']; targets.append(last)
        result=core.tick(world,prepared['state'],(3,0),last)
        assert result.status!='NOT_DUE'
        if model.pending is None:
            command=plan_from_candidate(result.candidate,index+1,fb['sample_s'],last,feedback=fb)
            assert model.accept(command,index*.05)=='ACCEPTED'
        model.advance(.05)
    core.close()
    assert len(targets)==20
    assert np.allclose(np.diff(targets),.05)
    assert len(local.requests)==10
    assert np.allclose(np.diff([r['identity']['submitted_s'] for r in local.requests]),.1)


def test_old_feedback_and_wrong_prediction_rejected():
    model=PlantModel(VehicleState(0,0,0,0,0),WheelLimits())
    model.feedback()
    model.advance(1.)
    assert model.accept(replace(plan(99,1.2),predicted_initial=(10,0,0,0,0)),1.)!='ACCEPTED'


def test_source_binding_expiry_and_handover_failure_are_explicit():
    model=PlantModel(VehicleState(0,0,0,0,0),WheelLimits())
    command=plan(model=model)
    assert model.accept(replace(command,predicted_initial=(10,0,0,0,0)),.01)=='PREDICTION_MISMATCH'
    assert model.accept(replace(command,source_vector=(10,0,0,0,0,0)),.01)=='SOURCE_IDENTITY_MISMATCH'
    assert model.accept(command,.01)=='ACCEPTED'
    model.vector[0]+=.01  # 模拟执行层扰动；协议必须记失败，不能传送回预测初态。
    model.advance(.15)
    assert any(e['reason']=='HANDOVER_MISMATCH' for e in model.protocol_failures)
    assert model.vector[0]>=.01
    other=PlantModel(VehicleState(0,0,0,0,0),WheelLimits())
    old=plan(start=.4,model=other)
    other.advance(.2)
    assert other.accept(old,.2)=='SOURCE_EXPIRED'


def test_map_event_at_last_endpoint_is_audited():
    from scripts.navigation.one_cm.nav1cm.contracts import WorldSnapshot,CellRect
    from scripts.navigation.one_cm.nav1cm.fixtures import Scenario
    world=WorldSnapshot(1,(),(CellRect(-1000,-1000,1500,1500),))
    changed=replace(world,epoch=2,blocked=(CellRect(-20,-20,20,20),))
    scenario=Scenario('endpoint',world,VehicleState(0,0,0,0,0),(2.,0.),update_step=40,updated_world=changed)
    row=dict(kind='FEEDBACK',sample_s=2.,vector=[0,0,0,0,0,0],segments=[dict(start_s=1.95,end_s=2.,before=[0,0,0,0,0,0],after=[0,0,0,0,0,0],control=[0,0,0],mode='forward',source='brake',route_version=1)])
    audit=benchmark.audit_feedback([row],scenario,WheelLimits(),None)
    assert any(r['world_epoch']==2 and not r['valid'] for r in audit)
    assert any(r.get('kind')=='TERMINAL_STATE' and not r['valid'] for r in audit)
    with pytest.raises(ValueError,match='align'):
        benchmark.audit_feedback([row],replace(scenario,update_step=40.5),WheelLimits(),None)


def test_empty_busy_coverage_is_not_continuity_pass():
    result=benchmark.continuity_assessment([],[],[],False)
    assert not result['delayed_motion_covered']
    assert not result['continuity_pass']
    stationary=[dict(feedback_count=3,displacement_m=0.)]
    assert not benchmark.continuity_assessment(stationary,[dict(valid=True)],[],True)['continuity_pass']
    moving=[dict(feedback_count=3,displacement_m=.01)]
    covered=benchmark.continuity_assessment(moving,[dict(valid=True)],[],True,execution_coverage={'complete':True})
    assert covered['continuity_pass'] and covered['tail_inflight_uncovered']
    assert not benchmark.continuity_assessment(moving,[dict(valid=True)],[dict(reason='HANDOVER_MISMATCH')],False)['continuity_pass']


def test_freshness_drop_deduplicated_does_not_hide_actual_arrival():
    failure=dict(reason='SOURCE_EXPIRED',sequence=537,sample_s=133.65)
    rows=[dict(kind='PLAN_ACK',sequence=537,reason='SOURCE_EXPIRED',received_wall_s=200.,plan=dict(source_feedback_s=133.5)),
          dict(kind='FEEDBACK',sample_s=134.,vector=[1.4,0,0,0,0,0],protocol_failures=[failure]),
          dict(kind='FEEDBACK',sample_s=134.05,vector=[1.4,0,0,0,0,0],protocol_failures=[failure])]
    result=benchmark.session_outcome(rows,[dict(valid=True)],(1.4,0),[],False)
    assert result['navigation_arrived'] and result['integrity_pass']
    assert result['motion_outcome']=='ARRIVED'
    assert len(result['freshness_drops'])==1 and not result['protocol_errors']
    assert not result['continuity_pass']
    for reason in ('SOURCE_IDENTITY_MISMATCH','HANDOVER_MISMATCH'):
        bad=[dict(rows[-1],protocol_failures=[dict(failure,reason=reason)])]
        outcome=benchmark.session_outcome(bad,[dict(valid=True)],(1.4,0),[],False)
        assert outcome['navigation_arrived'] and not outcome['integrity_pass']
        assert outcome['status']=='FAILED_PROTOCOL'
    elsewhere=[dict(rows[-1],vector=[0,0,0,0,0,0])]
    assert not benchmark.session_outcome(elsewhere,[dict(valid=True)],(1.4,0),[],False)['navigation_arrived']
    rejected=[dict(rows[-1],target_rejected=True)]
    assert benchmark.session_outcome(rejected,[dict(valid=True)],(1.4,0),[],False)['motion_outcome']=='SAFE_STOP_GOAL_REJECTED'


def test_reanalysis_preserves_execution_identity_and_original_failure():
    identity=json.dumps(dict(source_sha256={'plant.py':'old-sha'},geometry_sha256='library-sha')).encode()
    prior=json.dumps(dict(identity=json.loads(identity),cases={'straight':dict(status='FAILED_PROTOCOL',worker_busy_evidence=[dict(feedback_count=3,displacement_m=.1)],tail_inflight_uncovered=False)})).encode()
    raw=json.dumps(dict(kind='FEEDBACK',sample_s=1.,vector=[1.4,0,0,0,0,0],protocol_failures=[])).encode()+b'\n'
    audit=b'[{"valid":true}]'
    result=benchmark.reanalyze_session(identity,raw,audit,prior,'straight',goal=(1.4,0),analysis_sha256='new-analysis-sha')
    assert result['identity']==json.loads(identity)
    assert result['original_status']=='FAILED_PROTOCOL'
    assert json.loads(prior)['cases']['straight']['status']=='FAILED_PROTOCOL'
    assert result['cases']['straight']['navigation_arrived']
    assert not result['cases']['straight']['continuity_pass']
    assert result['cases']['straight']['status']=='FAILED_EXECUTION_COVERAGE'
    assert result['execution_rerun'] is False
    assert result['analysis_sha256']=='new-analysis-sha'
    assert result['input_sha256']['raw_jsonl']==hashlib.sha256(raw).hexdigest()


def test_old_arrived_status_requires_final_measured_arrival():
    result=benchmark.session_outcome([dict(kind='FEEDBACK',vector=[0,0,0,0,0,0])],
        [dict(valid=True)],(1.4,0),[],False,'ARRIVED')
    assert result['status']=='FAILED_EXECUTION_COVERAGE'
    assert result['motion_outcome']=='STOPPED_NOT_ARRIVED'
    assert not result['navigation_arrived']


def test_actual_arrival_does_not_override_execution_failure():
    result=benchmark.session_outcome([dict(kind='FEEDBACK',vector=[1.4,0,0,0,0,0])],
        [dict(valid=True)],(1.4,0),[],False,'FAILED_NO_CERTIFIED_ACTION')
    assert result['status']=='FAILED_NO_CERTIFIED_ACTION'
    assert result['navigation_arrived'] and result['motion_outcome']=='ARRIVED'


@pytest.mark.parametrize('original_status',['FAILED_NO_CERTIFIED_ACTION','WALL_TIMEOUT'])
def test_reanalysis_retains_unrelated_original_failure(original_status):
    prior=json.dumps(dict(cases={'straight':dict(status=original_status)}))
    raw=json.dumps(dict(kind='FEEDBACK',vector=[1.4,0,0,0,0,0]))
    result=benchmark.reanalyze_session('{}',raw,'[{"valid":true}]',prior,'straight',
        goal=(1.4,0),analysis_sha256='analysis')
    assert result['cases']['straight']['status']==original_status
    assert result['cases']['straight']['navigation_arrived']
    assert result['original_status']==original_status
