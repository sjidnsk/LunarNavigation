import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm import GridSnapshot, Trajectory, WheelLimits, certify
from scripts.navigation.one_cm.nav1cm.seeds import initial_seed


def test_model_default_and_identity():
    legacy = WheelLimits()
    assert legacy.motion_model == 'curvature_limited'
    diff = WheelLimits(motion_model='differential_drive')
    assert legacy.digest() != diff.digest()
    for model in ('unknown', '', None, True):
        with pytest.raises(ValueError):
            WheelLimits(motion_model=model)


@pytest.mark.parametrize('v,mode', [(.01, 'forward'), (-.01, 'reverse'), (0., 'forward'), (0., 'spin')])
def test_independent_turning_certification(v, mode):
    grid = GridSnapshot(1, (-2.5, -2.5), .01, np.zeros((500, 500), dtype=np.int8), False)
    t = Trajectory(1, 1, [0.], [[0., 0., 0., v, .08, 0.]], np.empty((0, 3)), mode)
    diff = WheelLimits(motion_model='differential_drive')
    cert = certify(t, grid, diff, 1)
    assert cert.valid
    assert not cert.matches(t, grid, WheelLimits(), 1)
    assert certify(t, grid, WheelLimits(), 1).valid == (mode == 'spin')


@pytest.mark.parametrize('v,w,mode', [(.201, 0., 'forward'), (.01, .101, 'forward'), (-.01, .08, 'forward'), (.01, .08, 'spin')])
def test_independent_limits_and_mode_still_reject(v, w, mode):
    grid = GridSnapshot(1, (-2.5, -2.5), .01, np.zeros((500, 500), dtype=np.int8), False)
    t = Trajectory(1, 1, [0.], [[0., 0., 0., v, w, 0.]], np.empty((0, 3)), mode)
    assert not certify(t, grid, WheelLimits(motion_model='differential_drive'), 1).valid


def test_seed_does_not_couple_differential_turning():
    initial = np.zeros(6)
    diff = WheelLimits(motion_model='differential_drive', linear_speed_mps=.01)
    states, controls = initial_seed(initial, 'forward', diff, 1.)
    assert np.any(np.abs(states[:, 4]) > diff.curvature_inv_m * np.abs(states[:, 3]))
    assert np.max(np.abs(controls[:, 1])) <= diff.angular_accel_rps2
    legacy, _ = initial_seed(initial, 'forward', WheelLimits(linear_speed_mps=.01), 1.)
    assert np.all(np.abs(legacy[:, 4]) <= np.abs(legacy[:, 3]) + 1e-9)

@pytest.mark.parametrize('model,expected', [('differential_drive', 'CANDIDATE'), ('curvature_limited', 'CURVATURE')])
def test_candidate_review_uses_selected_model(model, expected):
    from scripts.navigation.one_cm.nav1cm.local_mpcc import MpccPlanner
    planner = object.__new__(MpccPlanner)
    planner.limits = WheelLimits(motion_model=model)
    planner.offsets = [0.]
    planner.radius = .5
    class Field:
        def sample(self, x, y):
            return 10., 0., 0.
    xs = np.array([[0., 0., 0., .01, .08, 0.], [.01, 0., .08, .01, .08, .01]])
    assert planner._candidate_reason(xs, np.zeros((1, 3)), 'forward', Field(), 10.) == expected

@pytest.mark.parametrize('name,value', [('linear_speed_mps', 0.), ('angular_speed_rps', float('nan')), ('minimum_clearance_m', -1.)])
def test_numeric_validation_still_applies(name, value):
    with pytest.raises(ValueError):
        WheelLimits(motion_model='differential_drive', **{name: value})


def test_differential_braking_is_independent_and_preserves_measurement():
    from scripts.navigation.one_cm.nav1cm import VehicleState
    from scripts.navigation.one_cm.nav1cm.seeds import brake_candidate
    from types import SimpleNamespace
    limits = WheelLimits(motion_model='differential_drive', angular_accel_rps2=.01)
    grid = GridSnapshot(1, (-2.5, -2.5), .01, np.zeros((500, 500), dtype=np.int8), False)
    state = VehicleState(0., 0., 0., .01, .08)
    result = brake_candidate(state, grid, SimpleNamespace(progress_s=0., route_version=1), limits)
    np.testing.assert_array_equal(result.states[0, :5], [0., 0., 0., .01, .08])
    assert result.controls[0, 0] == -limits.linear_accel_mps2
    assert np.any((np.abs(result.states[:, 3]) < 1e-9) & (result.states[:, 4] > .01))
    assert certify(result, grid, limits, 1).valid
    altered = result.controls.copy()
    altered[0, 1] = limits.angular_accel_rps2 + .01
    bad = Trajectory(1, 1, result.times, result.states, altered, result.mode)
    assert not certify(bad, grid, limits, 1).valid

@pytest.mark.skipif(__import__('os').environ.get('NAV1CM_REAL_SOLVER')!='1',reason='real acados required')
def test_real_diffdrive_solver_and_certification_accept_independent_initial_turn(tmp_path):
    from scripts.navigation.one_cm.nav1cm.contracts import VehicleState,Route
    from scripts.navigation.one_cm.nav1cm.reference import extract_reference
    from scripts.navigation.one_cm.nav1cm.maps import build_distance_field
    from scripts.navigation.one_cm.nav1cm.local_mpcc import MpccPlanner
    limits=WheelLimits(motion_model='differential_drive',angular_speed_rps=1.)
    state=VehicleState(0.,0.,0.,.01,.08)
    grid=GridSnapshot(1,(-2.5,-2.5),.01,np.zeros((500,500),np.int8))
    route=Route(1,1,[[0.,0.],[1.,0.]],(1.,0.))
    reference,_=extract_reference(route,state,grid)
    planner=MpccPlanner(limits,tmp_path)
    result=planner.plan(state,grid,reference,distance_field=build_distance_field(grid),field_grid_digest=grid.digest())
    assert result.candidate is not None,(result.reason,result.attempts)
    assert np.allclose(result.candidate.states[0,:5],[state.x,state.y,state.yaw,state.v,state.w])
    assert certify(result.candidate,grid,limits,1).valid
    assert not certify(result.candidate,grid,WheelLimits(angular_speed_rps=1.),1).valid
