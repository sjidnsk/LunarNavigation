import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import Trajectory, VehicleState
from scripts.navigation.one_cm.nav1cm.seeds import rollout
from scripts.navigation.one_cm.nav1cm.trajectory_buffer import TrajectoryBuffer
from scripts.navigation.one_cm.nav1cm.async_coordinator import AsyncNavigationCoordinator


def candidate(duration=1.):
    u = np.array([[0., 0., .1]]*10)
    return Trajectory(1, 1, np.linspace(0,duration,11), rollout([0,0,0,.1,0,0],u,duration/10),u)

@pytest.mark.parametrize('age',[.1,.15,.25])
def test_late_candidate_reintegrates_feedback(age):
    b=TrajectoryBuffer()
    c=b.rebase(candidate(),VehicleState(age*.1+.001,0,0,.1,0),age,.05)
    assert c.times[-1] == pytest.approx(1-age)
    assert c.states[0,0] == pytest.approx(age*.1+.001)
    assert c.states[-1,0] == pytest.approx(.101)


def test_exhaustion_and_feedback_jump_rejected():
    b=TrajectoryBuffer()
    with pytest.raises(ValueError,match='EXHAUSTED'):
        b.rebase(candidate(),VehicleState(.1,0,0,.1,0),.99,.05)
    with pytest.raises(ValueError,match='FEEDBACK'):
        b.rebase(candidate(),VehicleState(.5,0,0,.1,0),.1,.05)
from scripts.navigation.one_cm.nav1cm.contracts import CellRect, WorldSnapshot, Route, WheelLimits
from scripts.navigation.one_cm.nav1cm.execution import execute_action

class PendingGlobal:
    busy=True
    def poll(self): return None
    def close(self): pass

class ManualLocal:
    def __init__(self): self.pending=None; self.ready=False; self.requests=[]
    @property
    def busy(self): return self.pending is not None
    def submit(self,request):
        assert not self.busy
        self.pending=request
        self.requests.append(request)
    def poll(self):
        if not self.ready or self.pending is None: return None
        r,self.pending=self.pending,None
        self.ready=False
        s=r['state']; u=np.array([[0.,0.,abs(s.v)]]*10)
        trajectory=Trajectory(r['grid'].epoch,r['reference'].route_version,np.linspace(0,1,11),
                              rollout([s.x,s.y,s.yaw,s.v,s.w,0],u,.1),u)
        return {'request_identity':r['identity'],'candidate':trajectory,'timings_ms':{}}
    def close(self): pass


def make_coordinator():
    local=ManualLocal()
    c=AsyncNavigationCoordinator(local_worker=local,global_runner=PendingGlobal(),limits=WheelLimits(linear_speed_mps=.5,angular_speed_rps=.5))
    c._goal=(3.,0.)
    c.route=Route(1,1,[[0,0],[3,0]],(3,0))
    return c,local,WorldSnapshot(1,(),(CellRect(-2000,-2000,2000,2000),))

@pytest.mark.parametrize('delay',[.1,.15,.25])
def test_worker_busy_executes_every_fifty_ms_and_accepts_late(delay):
    c,w,world=make_coordinator()
    state=VehicleState(0,0,0,.1,0)
    first=c.tick(world,state,(3,0),0)
    state=execute_action(first.action,state)
    w.ready=True
    accepted=c.tick(world,state,(3,0),.05)
    assert any(e['kind']=='LOCAL_ACCEPTED' for e in accepted.events)
    state=execute_action(accepted.action,state)
    positions=[]
    for i in range(2,int(round((.1+delay)/.05))+1):
        now=i*.05
        if now >= .1+delay-1e-9: w.ready=True
        result=c.tick(world,state,(3,0),now)
        assert result.action is not None and result.action.source!='brake'
        assert result.action_certificate.matches(result.action,result.grid,c.limits,1)
        state=execute_action(result.action,state)
        positions.append(state.x)
    assert np.all(np.diff(positions)>0)
    assert any(e['kind']=='LOCAL_ACCEPTED' for e in result.events)

@pytest.mark.parametrize('change',['map','goal','route','policy'])
def test_stale_result_cannot_replace_buffer(change):
    c,w,world=make_coordinator()
    state=VehicleState(0,0,0,.1,0)
    state=execute_action(c.tick(world,state,(3,0),0).action,state)
    goal=(3,0)
    if change=='map': world=WorldSnapshot(2,(),world.known_free)
    if change=='goal': goal=(0,3)
    if change=='route': c.route=Route(2,1,[[0,0],[3,0]],(3,0))
    if change=='policy': c.allow_unknown=True
    w.ready=True
    r=c.tick(world,state,goal,.05)
    assert not any(e['kind']=='LOCAL_ACCEPTED' for e in r.events)
    assert r.action.source=='brake'


def test_buffer_exhaustion_brakes_without_extrapolation():
    c,w,world=make_coordinator()
    state=VehicleState(0,0,0,.1,0)
    state=execute_action(c.tick(world,state,(3,0),0).action,state)
    w.ready=True
    for i in range(1,23):
        r=c.tick(world,state,(3,0),i*.05)
        state=execute_action(r.action,state)
    assert r.action.source=='brake'
    assert any(e['kind']=='BRAKE_REQUIRED' for e in r.events)


def test_new_obstacle_and_feedback_jump_cannot_execute_old_motion():
    c,w,world=make_coordinator()
    state=VehicleState(0,0,0,.1,0)
    state=execute_action(c.tick(world,state,(3,0),0).action,state)
    w.ready=True
    r=c.tick(world,state,(3,0),.05)
    state=execute_action(r.action,state)
    jumped=VehicleState(state.x+.3,0,0,state.v,0)
    r=c.tick(world,jumped,(3,0),.1)
    assert r.action.source=='brake'
    assert r.action.states[0,0]==pytest.approx(jumped.x)
    blocked=WorldSnapshot(2,(CellRect(290,-10,310,10),),world.known_free)
    r=c.tick(blocked,jumped,(3,0),.15)
    assert r.status=='GOAL_IN_OBSTACLE' and r.action.source=='brake'

def test_unsafe_late_candidate_preserves_certified_old_buffer():
    c,w,world=make_coordinator()
    state=VehicleState(0,0,0,.1,0)
    state=execute_action(c.tick(world,state,(3,0),0).action,state)
    w.ready=True
    state=execute_action(c.tick(world,state,(3,0),.05).action,state)
    state=execute_action(c.tick(world,state,(3,0),.1).action,state)
    original_poll=w.poll
    def unsafe_poll():
        result=original_poll()
        if result is not None:
            tr=result['candidate']; controls=tr.controls.copy(); controls[4,0]=100
            result['candidate']=Trajectory(tr.map_epoch,tr.route_version,tr.times,tr.states,controls)
        return result
    w.poll=unsafe_poll; w.ready=True
    r=c.tick(world,state,(3,0),.15)
    assert any(e['kind']=='LOCAL_REJECTED' for e in r.events)
    assert r.action is not None and r.action.source!='brake'
    assert r.action_certificate.matches(r.action,r.grid,c.limits,1)


def test_clock_reversal_rejected_and_not_due_has_no_action():
    c,w,world=make_coordinator(); state=VehicleState(0,0,0,0,0)
    c.tick(world,state,(3,0),1.)
    assert c.tick(world,state,(3,0),1.01).status=='NOT_DUE'
    with pytest.raises(ValueError,match='monotonic'): c.tick(world,state,(3,0),.5)

def test_map_update_recertifies_safe_remaining_old_buffer():
    c,w,world=make_coordinator(); state=VehicleState(0,0,0,.1,0)
    state=execute_action(c.tick(world,state,(3,0),0).action,state)
    w.ready=True
    state=execute_action(c.tick(world,state,(3,0),.05).action,state)
    changed=WorldSnapshot(2,(CellRect(1000,1000,1100,1100),),world.known_free)
    r=c.tick(changed,state,(3,0),.1)
    assert r.action.source!='brake' and r.action.map_epoch==2
    assert r.action_certificate.matches(r.action,r.grid,c.limits,1)

def test_worker_submit_error_still_returns_certified_braking():
    c,w,world=make_coordinator()
    def broken(request): raise RuntimeError('worker unavailable')
    w.submit=broken
    r=c.tick(world,VehicleState(0,0,0,.1,0),(3,0),0)
    assert r.action.source=='brake'
    assert any(e['kind']=='LOCAL_SUBMIT_ERROR' for e in r.events)

def test_delayed_acceleration_from_rest_is_certified_without_teleport():
    c,w,world=make_coordinator(); state=VehicleState(0,0,0,0,0)
    original_poll=w.poll
    def accelerate_poll():
        result=original_poll()
        if result is not None:
            tr=result['candidate']; controls=np.zeros((20,3)); controls[:15,0]=.5
            times=np.linspace(0,1,21)
            states=rollout(tr.states[0],controls,.05)
            result['candidate']=Trajectory(tr.map_epoch,tr.route_version,times,states,controls)
        return result
    w.poll=accelerate_poll
    c.tick(world,state,(3,0),0)
    for i in range(1,8):
        if i==7: w.ready=True
        r=c.tick(world,state,(3,0),i*.05)
        assert r.action.states[0,0]==pytest.approx(state.x)
        assert r.action.states[0,3]==pytest.approx(state.v)
        assert r.action_certificate.matches(r.action,r.grid,c.limits,1)
        assert r.certificate.matches(r.candidate,r.grid,c.limits,1)
        state=execute_action(r.action,state)
    assert state.x>0 and state.v>0
    assert any(e['kind']=='LOCAL_ACCEPTED' for e in r.events)

def test_one_context_and_no_duplicate_whole_candidate_certification(monkeypatch):
    from scripts.navigation.one_cm.nav1cm import async_coordinator as module
    c,w,world=make_coordinator(); state=VehicleState(0,0,0,.1,0)
    state=execute_action(c.tick(world,state,(3,0),0).action,state)
    original_cert,original_prepare=module.certify,module.prepare_certification
    counts={'cert':0,'context':0}
    def cert(*args,**kwargs):
        counts['cert']+=1
        return original_cert(*args,**kwargs)
    def prepare(*args,**kwargs):
        counts['context']+=1
        return original_prepare(*args,**kwargs)
    monkeypatch.setattr(module,'certify',cert)
    monkeypatch.setattr(module,'prepare_certification',prepare)
    w.ready=True
    r=c.tick(world,state,(3,0),.05)
    assert r.action.source!='brake'
    assert counts=={'cert':2,'context':1}

@pytest.mark.parametrize('clock',[(0,.051,.100,.151,.200,.251),(0,.051,.260,.300)])
def test_execution_schedule_retains_phase_and_skips_missed_slots(clock):
    c,w,world=make_coordinator(); state=VehicleState(0,0,0,.1,0)
    for index,now in enumerate(clock):
        if index==1: w.ready=True
        result=c.tick(world,state,(3,0),now)
        assert result.status!='NOT_DUE' and result.action is not None
        assert result.action_certificate.matches(result.action,result.grid,c.limits,1)
        state=execute_action(result.action,state)
    assert c._next_local==pytest.approx(.3 if clock[-1]==.251 else .35)
    assert c.tick(world,state,(3,0),clock[-1]+.001).status=='NOT_DUE'


def test_missed_execution_slots_are_reported_without_catchup_actions():
    c,w,world=make_coordinator(); state=VehicleState(0,0,0,0,0)
    c.tick(world,state,(3,0),0)
    result=c.tick(world,state,(3,0),.26)
    schedule=next(e for e in result.events if e['kind']=='EXECUTION_SCHEDULE')
    assert schedule['skipped_periods']==4
    assert schedule['scheduled_s']==pytest.approx(.25)
    assert schedule['next_due_s']==pytest.approx(.3)
    assert result.action.times[-1]==pytest.approx(.05)
    assert c.tick(world,state,(3,0),.261).action is None


def test_global_submit_error_preserves_old_buffer_and_latest_goal_until_next_period():
    c,w,world=make_coordinator(); state=VehicleState(0,0,0,.1,0)
    state=execute_action(c.tick(world,state,(3,0),0).action,state)
    w.ready=True
    state=execute_action(c.tick(world,state,(3,0),.05).action,state)
    class BrokenGlobal:
        busy=False
        def poll(self): return None
        def submit(self,request): raise RuntimeError('global unavailable')
    c.runner=BrokenGlobal()
    r=c.tick(world,state,(3,0),.1)
    assert r.action.source!='brake'
    assert r.action_certificate.matches(r.action,r.grid,c.limits,1)
    assert any(e['kind']=='GLOBAL_SUBMIT_ERROR' for e in r.events)
    assert c._pending_reason is None
    assert c._next_global == pytest.approx(.1+c.global_period_s)
    r=c.tick(world,state,(0,3),.15)
    assert r.action.source=='brake'
    assert c._goal==(0.,3.) and c._pending_reason is None
    assert c._next_global == pytest.approx(.15+c.global_period_s)


@pytest.mark.parametrize('direction,target,velocity',[('forward',0.,.02),('reverse',np.pi,-.02)])
def test_initial_direction_lives_once_per_goal_not_reference_refresh(direction,target,velocity):
    from scripts.navigation.one_cm.nav1cm.contracts import LocalReference
    c=AsyncNavigationCoordinator(local_worker=ManualLocal(),global_runner=PendingGlobal(),initial_travel_direction=direction)
    c.replace_goal((3.,0.),goal_yaw=0.,generation=1)
    reference=LocalReference(1,1,[[0.,0.],[3.,0.]],[0.,3.])
    assert c._initial_start_direction(VehicleState(0.,0.,target,0.,0.),reference)==direction
    assert c._initial_start_direction(VehicleState(.1,0.,target,velocity,0.),reference)==''
    changed=LocalReference(2,2,[[.1,0.],[4.,1.]],[0.,4.])
    assert c._initial_start_direction(VehicleState(.1,0.,target,0.,0.),changed)==''
    c.replace_goal((3.,0.),goal_yaw=0.,generation=2)
    assert c._initial_start_direction(VehicleState(.1,0.,target,0.,0.),reference)==direction
    c.close()


def test_invalid_initial_direction_fails_before_creating_resources(monkeypatch):
    import scripts.navigation.one_cm.nav1cm.async_coordinator as module
    monkeypatch.setattr(module.NavigationCoordinator,'__init__',lambda *a,**kw:pytest.fail('must validate before resource creation'))
    with pytest.raises(ValueError):AsyncNavigationCoordinator(initial_travel_direction='sideways')


def test_ordinary_alignment_request_uses_configured_stop_tolerance():
    c, worker, world = make_coordinator()
    c.stopped_angular_rps = .025
    c.tick(world, VehicleState(0,0,0,0,.02), (3,0), 0., need_action=False)
    assert worker.requests[-1]['initial_stopped_angular_rps'] == .025
    assert not worker.requests[-1].get('initial_travel_direction')
