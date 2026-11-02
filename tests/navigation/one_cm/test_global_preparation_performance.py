"""启动准备与后台平滑边界；不把单元夹具声称为现场闭环。"""
from types import SimpleNamespace
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm import global_worker as worker
from scripts.navigation.one_cm.nav1cm.contracts import WorldSnapshot,CellRect,WheelLimits,VehicleState
from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator

def test_worker_starts_before_first_real_goal(monkeypatch):
    calls=[]
    class Pool:
        def __init__(self,**kwargs): pass
        def submit(self,fn,*args): calls.append((fn,args));return SimpleNamespace(done=lambda:False)
        def shutdown(self,**kwargs): pass
    monkeypatch.setattr(worker,'ProcessPoolExecutor',Pool)
    runner=worker.GlobalWorker()
    assert len(calls)==1
    assert calls[0][0].__name__=='prepare_worker'
    assert not runner.busy
    runner.submit({'goal':'test'})
    assert runner.busy and calls[1][0] is worker.run_request
    runner.close()

def request(**kwargs):
    world=WorldSnapshot(1,(),(CellRect(-2000,-2000,2000,2000),))
    return worker.make_request(world,(0,0),(4,0),1,WheelLimits(),
        reference_smoothing=True,terminal_yaw=np.pi/2,**kwargs)

def test_worker_returns_readonly_shaped_reference_and_separate_timings():
    result=worker.restore_result(worker.run_request(request()))
    shaped=result['shaped_reference']
    assert shaped.changed and shaped.reason=='SMOOTHED'
    assert len(shaped.route.points)>len(result['route'].points)
    assert result['smoothing_ms']>0
    assert result['worker_compute_ms']>=result['smoothing_ms']
    with pytest.raises(ValueError):shaped.proof_boxes.setflags(write=True)
    with pytest.raises(ValueError):shaped.route.points.setflags(write=True)

def test_parent_adopts_worker_curve_without_running_smoother(monkeypatch):
    from scripts.navigation.one_cm.nav1cm import reference_smoothing
    class Runner:
        busy=False;payload=None;ready=False
        def submit(self,r):self.payload=worker.run_request(r);self.busy=True
        def poll(self):
            if not self.ready:return None
            self.ready=False;self.busy=False;return self.payload
    r=Runner();c=NavigationCoordinator(None,runner=r,revalidate_current_map=True,reference_smoothing=True)
    w=WorldSnapshot(1,(),(CellRect(-2000,-2000,2000,2000),));s=VehicleState(0,0,0,0,0)
    c._update_route(w,s,(4,0),0,[],goal_yaw=np.pi/2)
    assert r.payload['shaped_reference']['changed']
    monkeypatch.setattr(reference_smoothing,'shape_route',lambda *a,**k:(_ for _ in ()).throw(AssertionError('parent smoother ran')))
    r.ready=True;events=[];c._update_route(w,s,(4,0),.1,events,goal_yaw=np.pi/2)
    assert c.route is not None
    assert len(c.route.points)>len(c.raw_route.points)
    assert any(e['kind']=='GLOBAL_REFERENCE_SHAPED' and e['changed'] for e in events)
    assert not any(e['kind']=='GLOBAL_ERROR' for e in events)

@pytest.mark.parametrize('budget',[-1.,float('nan'),float('inf'),True])
def test_invalid_total_budget_rejected_before_worker_creation(budget):
    with pytest.raises(ValueError,match='global_budget_s'):
        NavigationCoordinator(None,runner=SimpleNamespace(),global_budget_s=budget)


def test_same_curve_segment_is_not_rechecked_in_one_smoothing_call(monkeypatch):
    from scripts.navigation.one_cm.nav1cm import reference_smoothing as api
    from scripts.navigation.one_cm.nav1cm.contracts import Route
    original=api._segments_clear;seen=set()
    def counted(starts,ends,blocked,corner_radius=0.):
        for a,b in zip(starts,ends):
            key=tuple(a)+tuple(b)
            assert key not in seen,'same segment rechecked against identical obstacle snapshot'
            seen.add(key)
        return original(starts,ends,blocked,corner_radius)
    monkeypatch.setattr(api,'_segments_clear',counted)
    raw=Route(1,1,[(0,0),(4,0),(4,4)],(4,4))
    boxes=np.array([[100.,100.,101.,101.]])
    result=api.shape_route(raw,boxes,budget_s=2.,raw_verified=True,corner_radius=.75)
    assert result.changed


def test_current_map_can_reject_worker_curve_while_preserving_safe_raw_route():
    from scripts.navigation.one_cm.nav1cm.contracts import Route
    payload=worker.restore_result(worker.run_request(request()))
    shaped=payload['shaped_reference'];point=shaped.route.points[np.argmin(shaped.route.points[:,1])]
    x,y=np.round((point+np.array([0.,-.70]))*100).astype(int)
    changed=WorldSnapshot(2,(CellRect(int(x-1),int(y-1),int(x+1),int(y+1)),),())
    c=NavigationCoordinator(None,runner=SimpleNamespace(),revalidate_current_map=True,reference_smoothing=True)
    c._observe_world(changed)
    raw=payload['route']
    assert c._route_clear(raw)
    events=[];c._adopt_route(raw,events,raw_verified=True,shaped_reference=shaped,smoothing_ms=payload['smoothing_ms'])
    np.testing.assert_array_equal(c.route.points,raw.points)
    assert events[-1]['reason']=='CURRENT_MAP_REJECTED'


def test_shaped_payload_cannot_change_goal_identity():
    payload=worker.run_request(request())
    payload['shaped_reference']['route']['goal']=(8.,9.)
    with pytest.raises(ValueError,match='shaped/raw route identity'):
        worker.restore_result(payload)


@pytest.mark.parametrize('search_elapsed,expected_remaining',[(.7,.3),(1.01,0.)])
def test_search_and_smoothing_share_one_deadline(monkeypatch,search_elapsed,expected_remaining):
    clock=[100.];budgets=[]
    monkeypatch.setattr(worker,'perf_counter',lambda:clock[0])
    original_plan=worker.AdaptiveGlobalPlanner.plan
    original_shape=worker.shape_route
    def plan(self,*args,**kwargs):
        result=original_plan(self,*args,**kwargs)
        clock[0]+=search_elapsed
        return result
    def shape(*args,**kwargs):
        budgets.append(kwargs['budget_s'])
        return original_shape(*args,**kwargs)
    monkeypatch.setattr(worker.AdaptiveGlobalPlanner,'plan',plan)
    monkeypatch.setattr(worker,'shape_route',shape)
    payload=worker.run_request(request())
    assert payload['total_budget_s']==1.
    assert payload['smoothing_budget_s']==pytest.approx(expected_remaining)
    assert budgets==pytest.approx([expected_remaining])
    if expected_remaining==0:
        assert payload['shaped_reference']['reason']=='BUDGET_EXHAUSTED'
        assert payload['shaped_reference']['changed'] is False
