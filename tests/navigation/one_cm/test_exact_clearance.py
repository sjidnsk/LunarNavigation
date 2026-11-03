"""同一原始矩形的全局、局部和认证临界距离回归。"""
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import CellRect, WorldSnapshot, VehicleState, WheelLimits
from scripts.navigation.one_cm.nav1cm.maps import extract_window, build_distance_field, DistanceFieldCache
from scripts.navigation.one_cm.nav1cm.certification import _body_collision, prepare_certification
from scripts.navigation.one_cm.nav1cm.global_planner import _segment_clear


def fixture(x=.24175):
    world=WorldSnapshot(1,(CellRect(-100,100,100,120),),())
    state=VehicleState(x,.24175,0.,0.,0.)
    return world,state,extract_window(world,state,True)


def test_raw_geometry_distance_and_gradient_are_query_exact():
    _,s,g=fixture()
    d,gx,gy=build_distance_field(g).sample(s.x,s.y)
    assert d == pytest.approx(.75825,abs=1e-12)
    assert (gx,gy)==pytest.approx((0.,-1.))


def test_same_physical_obstacle_does_not_change_with_window_anchor():
    w,s,g=fixture();cache=DistanceFieldCache()
    a=cache.get(g).sample(s.x,s.y)[0]
    shifted=extract_window(w,VehicleState(s.x+.003,s.y+.004,0.,0.,0.),True)
    assert cache.get(shifted).sample(s.x,s.y)[0] == pytest.approx(a,abs=1e-12)


def test_global_local_certification_agree_at_hard_circle_boundary():
    l=WheelLimits(body_length_m=1.182,body_width_m=.818,minimum_clearance_m=.75-np.hypot(1.182,.818)/2)
    for distance in (.749,.751,.75825):
        w=WorldSnapshot(1,(CellRect(-100,100,100,120),),())
        s=VehicleState(.003,1-distance,0.,0.,0.);g=extract_window(w,s,True)
        d=build_distance_field(g).sample(s.x,s.y)[0]
        boxes=np.array([[-1-.75,1-.75,1+.75,1.2+.75]])
        global_clear=_segment_clear((s.x,s.y),(s.x,s.y),boxes,.75)
        certified=_body_collision((s.x,s.y,0.),0.,prepare_certification(g),l) is None
        assert (d>.75)==global_clear==certified


def test_continuous_tangential_travel_and_spin_do_not_inflate_circle():
    from scripts.navigation.one_cm.nav1cm.certification import _sweep
    l=WheelLimits(body_length_m=1.182,body_width_m=.818,minimum_clearance_m=.75-np.hypot(1.182,.818)/2,motion_model='differential_drive')
    w=WorldSnapshot(1,(CellRect(-100,100,100,120),),())
    s=VehicleState(.003,.249,0.,0.,0.);g=extract_window(w,s,True);ctx=prepare_certification(g)
    for v,omega in ((.1,0.),(0.,.1)):
        reason,_=_sweep(np.array([s.x,s.y,0.,v,omega,0.]),np.zeros(3),.2,ctx,l)
        assert reason is None


def test_raw_geometry_changes_identity_even_when_raster_is_equal():
    from dataclasses import replace
    _,_,g=fixture()
    moved=replace(g,obstacle_boxes_m=tuple((a+.0001,b,c+.0001,d) for a,b,c,d in g.obstacle_boxes_m))
    assert moved.digest()!=g.digest()


def test_measured_center_drift_during_rotation_is_still_collision_checked():
    from scripts.navigation.one_cm.nav1cm.certification import _sweep
    l=WheelLimits(body_length_m=1.182,body_width_m=.818,minimum_clearance_m=.75-np.hypot(1.182,.818)/2,motion_model='differential_drive')
    _,_,g=fixture();ctx=prepare_certification(g)
    # 旋转时中心向障碍偏移：偏移后的实测状态必须仍被拒绝。
    reason=_body_collision((.003,.251,.2),0.,ctx,l)
    assert reason is not None
    # 带平移的旋转轨迹越过硬边界，不能因模式名叫spin而忽略。
    reason,_=_sweep(np.array([.003,.249,np.pi/2,.1,.1,0.]),np.zeros(3),.2,ctx,l)
    assert reason is not None


def test_exact_query_matches_independent_rectangle_oracle_and_gradient():
    import math
    rng=np.random.default_rng(20261103)
    rects=tuple(CellRect(int(x),int(y),int(x+w),int(y+h)) for x,y,w,h in
        np.column_stack((rng.integers(-200,200,(20,2)),rng.integers(1,40,(20,2)))))
    world=WorldSnapshot(1,rects,());state=VehicleState(.003,.007,0.,0.,0.)
    grid=extract_window(world,state,True);field=build_distance_field(grid)
    ox,oy=grid.origin_xy
    for x,y in rng.uniform(-1.8,1.8,(100,2)):
        expected=min(x-ox,y-oy,ox+5-x,oy+5-y,*[
            math.hypot(max(b.x0*.01-x,0.,x-b.x1*.01),max(b.y0*.01-y,0.,y-b.y1*.01))
            for b in rects])
        d,gx,gy=field.sample(x,y)
        assert d==pytest.approx(expected,abs=1e-12)
        if d>1e-4:
            eps=1e-7
            assert gx==pytest.approx((field.sample(x+eps,y)[0]-field.sample(x-eps,y)[0])/(2*eps),abs=1e-7)
            assert gy==pytest.approx((field.sample(x,y+eps)[0]-field.sample(x,y-eps)[0])/(2*eps),abs=1e-7)


def test_stationary_circle_rotation_checks_geometry_once_but_keeps_dynamics(monkeypatch):
    from scripts.navigation.one_cm.nav1cm import certification as c
    from scripts.navigation.one_cm.nav1cm.contracts import Trajectory
    from scripts.navigation.one_cm.nav1cm.dynamics import step
    l=WheelLimits(body_length_m=1.182,body_width_m=.818,minimum_clearance_m=.75-np.hypot(1.182,.818)/2,motion_model='differential_drive')
    w,s,g=fixture();z=np.array([s.x,s.y,0.,0.,.1,0.]);u=np.zeros(3)
    states=np.array([z.copy()]+[step(z,u,.2*k) for k in range(1,81)])
    t=Trajectory(1,1,np.arange(81)*.2,states,np.zeros((80,3)),'spin')
    original=c._body_collision;calls=[]
    def check(*args):calls.append(1);return original(*args)
    monkeypatch.setattr(c,'_body_collision',check)
    assert c.certify(t,g,l,1,check_braking_tail=False).valid
    assert len(calls)==1
    states[40,2]+=.01
    bad=Trajectory(1,1,t.times,states,t.controls,'spin')
    assert not c.certify(bad,g,l,1,check_braking_tail=False).valid


def test_batch_exact_distance_and_gradient_equal_scalar_queries():
    w,s,g=fixture();f=build_distance_field(g)
    points=np.random.default_rng(123).uniform(-2.,2.,(200,2))
    np.testing.assert_allclose(f.sample_many(points),[f.sample(*p) for p in points],atol=1e-12)
