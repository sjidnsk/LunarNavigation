"""几何整形接入：原路线诊断、实际MPCC参考与代际不能混用。"""
from dataclasses import asdict
import numpy as np
from scripts.navigation.one_cm.nav1cm.contracts import CellRect, Route, VehicleState, WheelLimits, WorldSnapshot, ProgressToken
from scripts.navigation.one_cm.nav1cm.coordinator import NavigationCoordinator
from tests.navigation.one_cm.test_coordinator import Runner


class CornerRunner(Runner):
    def poll(self):
        if not self.ready or not self.busy:return None
        r,self.pending=self.pending,None;self.ready=False
        route=Route(r['version'],r['world']['epoch'],[[0,0],[4,0],[4,4]],(4,4))
        return dict(route=asdict(route),requested_version=r['version'],requested_epoch=r['world']['epoch'],
                    requested_goal=r['goal'],planning_outcome=0,reason_code='PLAN_FOUND',has_reference=True,
                    submitted_wall_s=r['submitted_wall_s'],worker_start_wall_s=r['submitted_wall_s'],worker_finish_wall_s=r['submitted_wall_s'])


def build():
    runner=CornerRunner()
    c=NavigationCoordinator(None,runner=runner,revalidate_current_map=True,reference_smoothing=True,
        limits=WheelLimits(body_length_m=.01,body_width_m=.01,minimum_clearance_m=0),clearance_radius_m=.01)
    return c,runner,WorldSnapshot(1,(),(CellRect(-1000,-1000,1000,1000),)),VehicleState(0,0,0,0,0)


def adopt(c,runner,w,state):
    c._update_route(w,state,(4,4),0.,[])
    runner.ready=True;events=[];c._update_route(w,state,(4,4),.1,events)
    return events


def test_adopted_route_is_shaped_but_original_geometry_is_preserved():
    c,r,w,s=build();events=adopt(c,r,w,s)
    assert c.raw_route.points.tolist()==[[0.,0.],[4.,0.],[4.,4.]]
    assert len(c.route.points)>3
    assert c.route.version==c.raw_route.version and c.route.map_epoch==c.raw_route.map_epoch
    assert any(x['kind']=='GLOBAL_REFERENCE_SHAPED' and x['changed'] for x in events)
    assert c._route_clear(c.route)


def test_periodic_raw_equivalent_keeps_shaped_geometry_and_progress():
    c,r,w,s=build();adopt(c,r,w,s);shape=c.route
    c.progress=ProgressToken(shape.version,.2)
    c._update_route(w,s,(4,4),1.1,[]);r.ready=True;events=[]
    c._update_route(w,s,(4,4),1.2,events)
    assert c.route is shape and c.progress.s==.2
    assert any(x['kind']=='ROUTE_CONTINUITY_RETAINED' for x in events)


def test_new_obstacle_only_in_curve_retires_reference_and_raw_publication():
    c,r,w,s=build();adopt(c,r,w,s);raw=c.raw_route
    # 该障碍远离原x=4/y=0折线，却穿过Bezier圆角中段。
    changed=WorldSnapshot(2,(CellRect(380,17,383,20),),w.known_free)
    c._observe_world(changed)
    assert all(__import__('scripts.navigation.one_cm.nav1cm.global_planner',fromlist=['_segment_clear'])._segment_clear(a,b,c._blocked_boxes) for a,b in zip(raw.points[:-1],raw.points[1:]))
    # 让更新入口实际观察本次地图变化。
    c._observed_world=w;c._blocked=w.blocked
    events=[];c._update_route(changed,s,(4,4),.2,events)
    assert c.route is None and c.raw_route is None and c.progress is None
    assert any(x['kind']=='GLOBAL_CURRENT_MAP_REJECTED' for x in events)


def test_goal_replacement_cannot_publish_previous_raw_or_shaped_route():
    c,r,w,s=build();adopt(c,r,w,s)
    c.replace_goal((0,4),goal_yaw=None,generation=1)
    assert c.route is None and c.raw_route is None


def test_async_worker_receives_the_same_shaped_geometry_published_globally():
    from scripts.navigation.one_cm.nav1cm.async_coordinator import AsyncNavigationCoordinator
    from scripts.navigation.one_cm.nav1cm.maps import extract_window
    from scripts.navigation.one_cm.nav1cm.reference import extract_reference
    from tests.navigation.one_cm.test_async_execution import ManualLocal,PendingGlobal
    local=ManualLocal()
    c=AsyncNavigationCoordinator(local_worker=local,global_runner=PendingGlobal(),
        revalidate_current_map=True,reference_smoothing=True)
    w=WorldSnapshot(1,(),(CellRect(-2000,-2000,2000,2000),))
    state=VehicleState(3.,0.,0.,0.,0.)
    c._set_goal((4.,4.),None);c._observe_world(w)
    c._adopt_route(Route(11,1,[[0,0],[4,0],[4,4]],(4,4)),[])
    expected,_=extract_reference(c.route,state,extract_window(w,state,False))
    packet=c.tick(w,state,(4,4),0.)
    assert local.requests
    actual=local.requests[0]['reference']
    np.testing.assert_array_equal(actual.points,expected.points)
    np.testing.assert_array_equal(packet.reference.points,actual.points)
    assert actual.route_version==c.route.version==11
    assert c.raw_route.points.tolist()==[[0.,0.],[4.,0.],[4.,4.]]
    assert len(actual.points)>3


def test_curve_invalidation_clears_old_async_buffer_and_brakes():
    from scripts.navigation.one_cm.nav1cm.async_coordinator import AsyncNavigationCoordinator
    from tests.navigation.one_cm.test_async_execution import ManualLocal,PendingGlobal
    local=ManualLocal()
    c=AsyncNavigationCoordinator(local_worker=local,global_runner=PendingGlobal(),
        limits=WheelLimits(body_length_m=.01,body_width_m=.01,minimum_clearance_m=0),
        clearance_radius_m=.01,revalidate_current_map=True,reference_smoothing=True)
    w=WorldSnapshot(1,(),(CellRect(-2000,-2000,2000,2000),));state=VehicleState(3.,0.,0.,0.,0.)
    c._set_goal((4.,4.),None);c._observe_world(w)
    c._adopt_route(Route(11,1,[[0,0],[4,0],[4,4]],(4,4)),[])
    c.tick(w,state,(4,4),0.)
    changed=WorldSnapshot(2,(CellRect(380,17,383,20),),w.known_free)
    result=c.tick(changed,state,(4,4),.1)
    assert c.route is None and c.raw_route is None
    assert result.candidate is not None and result.candidate.source=='brake'
    assert result.candidate.route_version==-1
    assert c.buffer.remaining_s(.1)==0.


def test_smoothing_requires_current_map_revalidation():
    import pytest
    with pytest.raises(ValueError,match='revalidation'):
        NavigationCoordinator(None,runner=Runner(),reference_smoothing=True)


def test_direct_adoption_cannot_claim_unverified_raw_geometry():
    import pytest
    c,runner,w,state=build()
    c._observe_world(WorldSnapshot(2,(CellRect(180,-10,190,10),),w.known_free))
    with pytest.raises(ValueError,match='raw route collision'):
        c._adopt_route(Route(1,2,[[0,0],[4,0],[4,4]],(4,4)),[])
    assert c.route is None
