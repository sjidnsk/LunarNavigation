"""Task 2 的行为回归，真实求解测试仅在显式启用的 acados 环境运行。"""
import importlib
import importlib.util
import os
from pathlib import Path
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import CellRect, VehicleState, WheelLimits, WorldSnapshot, Route
from scripts.navigation.one_cm.nav1cm.maps import extract_window, build_distance_field
from scripts.navigation.one_cm.nav1cm.reference import extract_reference
from scripts.navigation.one_cm.nav1cm.certification import certify


def modules():
    prefix = 'scripts.navigation.one_cm.nav1cm.'
    assert importlib.util.find_spec(prefix + 'local_mpcc'), 'Task 2 MPCC implementation missing'
    assert importlib.util.find_spec(prefix + 'seeds'), 'Task 2 braking implementation missing'
    return importlib.import_module(prefix + 'local_mpcc'), importlib.import_module(prefix + 'seeds')


def scene(points=((0,0),(2,0)), blocked=(), free=(CellRect(-400,-400,400,400),), allow=False, state=None):
    state = state or VehicleState(0,0,0,0,0)
    grid = extract_window(WorldSnapshot(1, tuple(blocked), tuple(free)), state, allow)
    route = Route(3,1,np.asarray(points,dtype=float),tuple(points[-1]))
    ref,_ = extract_reference(route,state,grid)
    return state,grid,ref


def test_braking_preserves_measured_velocity_and_certifies():
    _,seeds = modules()
    state,grid,ref=scene(state=VehicleState(0,0,0,.2,.1))
    brake=seeds.brake_candidate(state,grid,ref,WheelLimits())
    assert brake.source == 'brake'
    assert np.array_equal(brake.states[0,:5], [0,0,0,.2,.1])
    assert np.allclose(brake.states[-1,3:5],0)
    assert certify(brake,grid,WheelLimits(),ref.route_version).valid


def test_mode_change_keeps_spin_until_angular_velocity_stops():
    mpcc,_=modules()
    state,_,ref=scene(state=VehicleState(0,0,0,0,.03))
    assert mpcc.select_mode(state,ref)=='spin'
    state,_,ref=scene(points=((0,0),(-2,0)),state=VehicleState(0,0,0,.1,0))
    assert mpcc.select_mode(state,ref)=='brake'


def test_missing_dependency_is_explicit(tmp_path):
    mpcc,_=modules()
    if importlib.util.find_spec('acados_template'):
        pytest.skip('dependency installed')
    with pytest.raises(mpcc.SolverUnavailable, match='acados'):
        mpcc.MpccPlanner(WheelLimits(),tmp_path)


@pytest.fixture(scope='module')
def planner():
    mpcc,_=modules()
    if os.environ.get('NAV1CM_REAL_SOLVER')!='1':
        pytest.skip('real acados runtime not enabled')
    return mpcc.MpccPlanner(WheelLimits(),Path(os.environ['NAV1CM_BUILD_DIR']))


@pytest.mark.parametrize('name,points,state',[('forward',((0,0),(2,0)),VehicleState(0,0,0,0,0)),('reverse',((0,0),(-2,0)),VehicleState(0,0,0,0,0)),('spin',((0,0),(0,2)),VehicleState(0,0,0,0,0))])
def test_real_progress_and_full_constraints(planner,name,points,state):
    state,grid,ref=scene(points=points,state=state)
    result=planner.plan(state,grid,ref)
    assert result.candidate is not None, result
    candidate=result.candidate
    assert candidate.mode==name and candidate.source.startswith('mpcc:')
    assert certify(candidate,grid,WheelLimits(),ref.route_version).valid
    if name=='spin':
        assert candidate.states[-1,2]>.05
        assert np.max(np.abs(candidate.states[:,3]))<1e-9
    else:
        assert candidate.states[-1,5]>.1
        assert candidate.states[-1,0]*(1 if name=='forward' else -1)>.1


def test_known_blocked_goal_rejected(planner):
    state,grid,ref=scene(blocked=(CellRect(190,-10,210,10),))
    result=planner.plan(state,grid,ref)
    assert result.candidate is None and result.reason=='KNOWN_BLOCKED_GOAL'


def test_unknown_policies_and_blocked(planner):
    results=[]
    for allow in (False,True):
        state,grid,ref=scene(free=(CellRect(-65,-50,65,50),),allow=allow)
        result=planner.plan(state,grid,ref)
        results.append(result)
        if result.candidate is not None:
            assert certify(result.candidate,grid,WheelLimits(),ref.route_version).valid
    assert results[1].candidate is not None
    assert results[0].candidate is None or results[0].candidate.states[-1,5]<results[1].candidate.states[-1,5]
    state,grid,ref=scene(blocked=(CellRect(60,-250,70,250),))
    result=planner.plan(state,grid,ref)
    assert result.candidate is None
    assert 1<=len(result.attempts)<=3
    assert len(result.attempts)==1
    assert result.attempts[0]['reason']=='FIXED_INITIAL_CIRCLE_INFEASIBLE'


def test_obstacle_candidate_requires_independent_certificate(planner):
    state,grid,ref=scene(points=((0,0),(.6,-.1),(1.4,-.4),(2,-.4)),blocked=(CellRect(110,35,111,60),))
    result=planner.plan(state,grid,ref)
    assert result.candidate is not None, result
    assert certify(result.candidate,grid,WheelLimits(),ref.route_version).valid


def test_external_distance_field_requires_exact_source_digest(planner):
    state,grid,ref=scene()
    field=build_distance_field(grid)
    with pytest.raises(ValueError,match='source grid digest'):
        planner.plan(state,grid,ref,distance_field=field)
    other=extract_window(WorldSnapshot(1,(),(CellRect(-400,-400,400,400),)),VehicleState(.01,0,0,0,0),False)
    with pytest.raises(ValueError,match='source grid digest'):
        planner.plan(state,grid,ref,distance_field=field,field_grid_digest=other.digest())


def test_real_spin_braking_preserves_measured_initial_state(planner):
    state,grid,ref=scene(state=VehicleState(0,0,0,0,.03))
    result=planner.plan(state,grid,ref)
    assert result.candidate is not None
    assert result.candidate.mode=='spin'
    assert result.candidate.states[0,4]==.03
    assert abs(result.candidate.states[-1,4])<.03
    assert certify(result.candidate,grid,WheelLimits(),ref.route_version).valid


def test_batch_geometry_matches_scalar_and_hand_truth():
    from scripts.navigation.one_cm.nav1cm import certification as c
    assert hasattr(c,'_batch_body_collision'), 'batched geometry not implemented'
    limits=WheelLimits()
    rng=np.random.default_rng(20261026)
    cases=[((CellRect(60,-100,61,100),),False),((CellRect(58,40,60,42),),False),
           ((),False),((),True)]
    for blocked,allow in cases:
        free=(CellRect(-100,-100,100,100),)
        grid=extract_window(WorldSnapshot(1,blocked,free),VehicleState(0,0,0,0,0),allow)
        context=c._CollisionContext.build(grid)
        original_context=c._CollisionContext(grid,context.hazard_prefix)
        poses=np.column_stack((rng.uniform(-2.4,2.4,300),rng.uniform(-2.4,2.4,300),rng.uniform(-np.pi,np.pi,300)))
        poses=np.vstack(([0,0,0],[.02,0,0],[0,0,np.pi/4],[2,0,0],poses))
        for chunk in np.array_split(poses,38):
            expected=(None,len(chunk))
            for i,pose in enumerate(chunk):
                reason=c._body_collision(pose,1e-6,original_context,limits)
                if reason is not None:
                    expected=(reason,i+1); break
            assert c._batch_body_collision(chunk,1e-6,context,limits)==expected
    state,grid,_=scene(blocked=(CellRect(60,-100,61,100),))
    context=c._CollisionContext.build(grid)
    assert c._batch_body_collision(np.array([[.02,0,0]]),0.,context,limits)[0]==c.CertificationReason.COLLISION
    assert c._batch_body_collision(np.array([[2.,0,0]]),0.,context,limits)[0]==c.CertificationReason.OUT_OF_BOUNDS
    state,grid,_=scene(free=(CellRect(-60,-41,60,41),))
    context=c._CollisionContext.build(grid)
    assert c._batch_body_collision(np.array([[.02,0,0]]),0.,context,limits)[0]==c.CertificationReason.UNKNOWN
    state,grid,_=scene(blocked=(CellRect(58,40,60,42),))
    assert c._batch_body_collision(np.array([[0.,0,0]]),0.,c._CollisionContext.build(grid),limits)[0]==c.CertificationReason.COLLISION


def test_vector_sweep_matches_original_midpoint_sampling():
    import math
    from scripts.navigation.one_cm.nav1cm import certification as c
    from scripts.navigation.one_cm.nav1cm.dynamics import step
    limits=WheelLimits(); rng=np.random.default_rng(77)
    _,grid,_=scene(blocked=(CellRect(60,-100,61,100),CellRect(-100,59,100,60)))
    context=c._CollisionContext.build(grid)
    original=c._CollisionContext(grid,context.hazard_prefix)
    for _ in range(100):
        state=np.array([rng.uniform(-.3,.3),rng.uniform(-.3,.3),rng.uniform(-3,3),.2,.1,0.])
        u=np.array([rng.uniform(-.1,0),rng.uniform(-.05,0),.1]);dt=.2
        vmax=max(abs(state[3]),abs(state[3]+u[0]*dt));wmax=max(abs(state[4]),abs(state[4]+u[1]*dt))
        radius=math.hypot(limits.body_length_m/2,limits.body_width_m/2)
        speed=vmax*(1+dt*wmax/2)+radius*wmax
        n=max(1,int(math.ceil(speed*dt/.005)));interval=dt/n;margin=speed*interval/2+1e-6
        expected=(None,n+1)
        for i in range(n+1):
            pose=state if i==0 else step(state,u,i*interval)
            reason=c._body_collision(pose,margin,original,limits)
            if reason is not None:
                expected=(reason,i+1);break
        assert c._sweep(state,u,dt,context,limits)==expected


def test_native_geometry_module_is_optional_and_explicit():
    prefix='scripts.navigation.one_cm.nav1cm.'
    assert importlib.util.find_spec(prefix+'geometry_native'), 'optional native geometry missing'
    module=importlib.import_module(prefix+'geometry_native')
    with pytest.raises((OSError,ValueError)):
        module.NativeGeometry('/definitely/not/a/geometry/library.so')


def test_real_native_geometry_matches_python_certification(planner):
    from scripts.navigation.one_cm.nav1cm.geometry_native import NativeGeometry
    from scripts.navigation.one_cm.nav1cm import certification as c
    if not os.environ.get('NAV1CM_GEOMETRY_LIB'): pytest.skip('native geometry not enabled')
    backend=NativeGeometry(os.environ['NAV1CM_GEOMETRY_LIB'])
    rng=np.random.default_rng(81);limits=WheelLimits()
    for blocked in ((CellRect(60,-100,61,100),),(CellRect(58,40,60,42),),()):
        for allow in (False,True):
            state,grid,ref=scene(blocked=blocked,free=(CellRect(-100,-100,100,100),),allow=allow)
            context=c._CollisionContext.build(grid)
            poses=np.column_stack((rng.uniform(-2.5,2.5,1000),rng.uniform(-2.5,2.5,1000),rng.uniform(-np.pi,np.pi,1000)))
            poses=np.vstack(([0,0,0],[.02,0,0],[0,0,np.pi/4],[2,0,0],poses))
            for pose in poses:
                expected=c._body_collision(pose,.002,context,limits)
                reason,count=backend.check_poses(pose.reshape(1,3),.002,context,limits)
                assert reason==expected and count==1
            _,seeds=modules()
            brake=seeds.brake_candidate(VehicleState(0,0,0,.2,.1),grid,ref,limits)
            assert certify(brake,grid,limits,ref.route_version,backend=backend)==certify(brake,grid,limits,ref.route_version)
    for points in (((0,0),(2,0)),((0,0),(0,2)),((0,0),(-2,0))):
        state,grid,ref=scene(points=points)
        candidate=planner.plan(state,grid,ref).candidate
        assert candidate is not None
        assert certify(candidate,grid,limits,ref.route_version,backend=backend)==certify(candidate,grid,limits,ref.route_version)


def test_native_swept_wall_rotation_braking_and_unknown_truth():
    if not os.environ.get('NAV1CM_GEOMETRY_LIB'): pytest.skip('native geometry not enabled')
    from scripts.navigation.one_cm.nav1cm.geometry_native import NativeGeometry
    from scripts.navigation.one_cm.nav1cm.contracts import Trajectory,GridSnapshot
    from scripts.navigation.one_cm.nav1cm.dynamics import step
    backend=NativeGeometry(os.environ['NAV1CM_GEOMETRY_LIB']);limits=WheelLimits()
    def check(start,u,dt,cells,mode,expected,allow=False):
        states=[np.array(start,dtype=float)]
        if u: states.append(step(states[0],u,dt))
        tr=Trajectory(1,1,[0.,dt] if u else [0.],states,np.array([u] if u else []).reshape(-1,3),mode)
        grid=GridSnapshot(1,(-2.5,-2.5),.01,cells,allow)
        result=certify(tr,grid,limits,1,backend=backend)
        assert result.reason.value==expected
        assert result==certify(tr,grid,limits,1)
    cells=np.zeros((500,500),dtype=np.int8);cells[210:290,200]=1
    check([-1.4,0,0,.2,0,0],[0,0,.1],8.,cells,'forward','COLLISION')
    cells=np.zeros((500,500),dtype=np.int8);cells[290,308]=1
    check([0,0,-.05,0,.1,0],[0,0,0],1.,cells,'spin','COLLISION')
    cells=np.zeros((500,500),dtype=np.int8);cells[215:285,312]=1
    check([0,0,0,.2,0,0],[],.2,cells,'forward','BRAKING_COLLISION')
    cells[215:285,312]=-1
    check([0,0,0,.2,0,0],[],.2,cells,'forward','BRAKING_UNKNOWN')
    check([0,0,0,.2,0,0],[],.2,cells,'forward','VALID',True)
    cells=np.zeros((500,500),dtype=np.int8)
    check([1.87,0,0,.2,0,0],[],.2,cells,'forward','BRAKING_OUT_OF_BOUNDS')


@pytest.mark.parametrize("mode", ["forward", "reverse"])
@pytest.mark.parametrize("terminal", [None, -3.0])
def test_curved_node_yaws_follow_each_tangent_and_terminal_blend(mode, terminal):
    from types import SimpleNamespace
    mpcc, _ = modules()
    planner = object.__new__(mpcc.MpccPlanner)
    planner.limits = WheelLimits()
    planner.offsets = (0.,)
    planner.radius = .6
    points = np.array([[0., 0.], [-1., .05], [-2., -.05], [-2., -1.]])
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    ref = SimpleNamespace(points=points, s=s, terminal_yaw=terminal)
    progress = [0., s[1]+.1, s[2]+.1, s[-1]]
    states = np.zeros((4, 6)); states[:, 2] = 3.12; states[:, 5] = progress
    field = SimpleNamespace(sample=lambda x,y: (2., 0., 0.))
    params = planner._parameters(states, np.zeros((3,3)), ref, field, 3.12, s[-1], mode=mode)
    for z, param in zip(states, params):
        tangent_yaw = np.arctan2(param[3], param[2]) + (np.pi if mode=='reverse' else 0.)
        expected = mpcc._approach_yaw(ref, z[5], tangent_yaw, planner.N*planner.dt*planner.limits.linear_speed_mps)
        assert abs(mpcc.angle_error(expected, param[5])) < 1e-12
        assert abs(param[5]-z[2]) <= np.pi
    spin = planner._parameters(states, np.zeros((3,3)), ref, field, -3.0, s[-1], mode='spin')
    assert all(abs(mpcc.angle_error(-3., p[5])) < 1e-12 for p in spin)


def test_spin_target_stays_fixed_during_deceleration_and_residual_handoff():
    mpcc, _ = modules()
    for yaw in (.5, .07, .02):
        state, _, ref = scene(state=VehicleState(0,0,yaw,0,.03))
        assert abs(mpcc.travel_target_yaw(state,ref,'spin',.8)) < 1e-12
        assert abs(mpcc.travel_target_yaw(state,ref,'spin',.8,initial_travel_direction='forward')) < 1e-12
    state, _, ref = scene(state=VehicleState(0,0,0,0,.005))
    assert mpcc.select_mode(state,ref) == 'forward'
    assert mpcc.select_mode(state,ref,initial_travel_direction='forward') == 'forward'


@pytest.mark.parametrize('angular_speed, expected', [(.009, 'brake'), (-.009, 'brake'), (.011, 'spin')])
def test_terminal_mode_uses_same_angular_stopped_threshold(angular_speed, expected):
    from types import SimpleNamespace
    mpcc, _ = modules()
    ref = SimpleNamespace(points=np.zeros((1,2)), terminal_yaw=0.)
    assert mpcc.select_mode(VehicleState(0,0,0,0,angular_speed), ref) == expected


def test_approach_spin_target_is_reference_blend_across_deceleration():
    from types import SimpleNamespace
    mpcc, _ = modules()
    ref = SimpleNamespace(points=np.array([[0.,0.],[.8,0.]]), s=np.array([0.,.8]),
                          progress_s=.6, terminal_yaw=1.)
    for yaw, angular_speed in ((.5,.1), (.8,.03), (.85,.015)):
        state = VehicleState(0,0,yaw,0,angular_speed)
        assert mpcc.travel_target_yaw(state,ref,'spin',.8) == pytest.approx(.84375)


@pytest.mark.parametrize('direction',[1,-1])
@pytest.mark.parametrize('angle,expected',[(20.,'travel'),(29.9,'travel'),(29.999999,'travel'),(30.1,'spin'),(-29.9,'travel'),(-30.1,'spin')])
def test_normal_stopped_spin_threshold_is_thirty_degrees(direction,angle,expected):
    mpcc,_=modules()
    state,_,ref=scene(points=((0,0),(2*direction,0)),state=VehicleState(0,0,np.deg2rad(angle),0,0))
    desired='forward' if direction>0 else 'reverse'
    assert mpcc.select_mode(state,ref)==(desired if expected=='travel' else 'spin')


def test_thirty_degree_change_preserves_initial_fifteen_degree_alignment_and_moving_mode():
    mpcc,_=modules()
    state,_,ref=scene(state=VehicleState(0,0,np.deg2rad(16),0,0))
    assert mpcc.select_mode(state,ref,initial_travel_direction='forward')=='spin'
    state,_,ref=scene(state=VehicleState(0,0,np.deg2rad(40),.1,0))
    assert mpcc.select_mode(state,ref)=='forward'


@pytest.mark.parametrize('direction',['forward','reverse'])
@pytest.mark.parametrize('angle,spin',[(10.,False),(14.9,False),(14.999999,False),(15.1,True),(-14.9,False),(-15.1,True)])
def test_initial_alignment_spin_threshold_is_fifteen_degrees(direction,angle,spin):
    mpcc,_=modules()
    sign=1 if direction=='forward' else -1
    state,_,ref=scene(points=((0,0),(sign*2,0)),state=VehicleState(0,0,np.deg2rad(angle),0,0))
    assert mpcc.select_mode(state,ref,initial_travel_direction=direction)==('spin' if spin else direction)


@pytest.mark.parametrize('moving',[False,True])
@pytest.mark.parametrize('travel_success',[False,True])
@pytest.mark.parametrize('spin_success',[False,True])
def test_travel_and_spin_fallback_are_bounded_and_certified(moving,travel_success,spin_success):
    mpcc,_=modules()
    planner=mpcc.MpccPlanner.__new__(mpcc.MpccPlanner)
    planner.N=20;planner.dt=.2
    planner.limits=WheelLimits(motion_model='differential_drive',minimum_clearance_m=.0312775222660697)
    planner.offsets=np.array([0.]);planner.radius=.75;planner._warm=None
    calls=[]
    class Solver:
        def __init__(self,mode):self.mode=mode
        def reset(self):pass
        def set(self,*args):pass
        def solve(self):calls.append(self.mode);return 0 if travel_success or (spin_success and self.mode=='spin') else 1
        def get_stats(self,*args):return 0
        def get(self,k,field):
            if self.mode=='spin':return np.array([0.,-.1 if k<2 else 0.,0.])
            return np.array([.1 if k<2 else 0.,0.,.02])
    planner._solvers={mode:Solver(mode) for mode in ('forward','reverse','spin')}
    state,grid,ref=scene(state=VehicleState(0,0,np.deg2rad(10),.1 if moving else 0.,0.))
    before=grid.digest()
    result=planner.plan(state,grid,ref)
    assert calls==(['forward'] if travel_success else ['forward']*3+['spin']*(1 if spin_success else 3))
    if not travel_success and not spin_success:
        assert result.candidate is None and result.reason=='NO_FEASIBLE_MPCC_CANDIDATE'
        assert grid.digest()==before
        return
    assert result.candidate is not None
    assert result.candidate.mode==('forward' if travel_success else 'spin')
    np.testing.assert_array_equal(result.candidate.states[0,:5],[state.x,state.y,state.yaw,state.v,state.w])
    assert certify(result.candidate,grid,planner.limits,ref.route_version).valid
    assert grid.digest()==before
    assert [row['mode'] for row in result.attempts]==calls


def test_real_solver_fallback_after_travel_status_failures(planner,monkeypatch):
    state,grid,ref=scene(state=VehicleState(0,0,np.deg2rad(10),0.,0.))
    monkeypatch.setattr(planner._solvers['forward'],'solve',lambda:1)
    result=planner.plan(state,grid,ref,initial_travel_direction='forward')
    assert result.candidate is not None,result
    assert result.candidate.mode=='spin'
    assert [row['mode'] for row in result.attempts[:3]]==['forward']*3
    assert result.attempts[3]['mode']=='spin'
    assert certify(result.candidate,grid,planner.limits,ref.route_version).valid
