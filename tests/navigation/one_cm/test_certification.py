import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm import (
    CellState,
    CertificationReason,
    GridSnapshot,
    Trajectory,
    WheelLimits,
    certify,
    step,
)


def free_grid():
    return GridSnapshot(1, (-2.5, -2.5), 0.01, np.zeros((500, 500), dtype=np.int8), False)


def trajectory(start, controls, dt=0.2, mode="forward", epoch=1, version=1):
    states = [np.array(start, dtype=float)]
    for control in controls:
        states.append(step(states[-1], control, dt))
    return Trajectory(epoch, version, np.arange(len(states)) * dt, np.stack(states), np.asarray(controls, dtype=float).reshape(-1, 3), mode)


def test_midpoint_dynamics_propagates_all_six_states():
    out = step(np.array([0.0, 0.0, 0.0, 0.1, 0.0, 2.0]), np.array([0.5, 0.0, 0.2]), 0.2)
    np.testing.assert_allclose(out, [0.03, 0.0, 0.0, 0.2, 0.0, 2.04], atol=1e-12)


def test_certifies_full_body_in_open_map_and_reports_braking_tail():
    t = trajectory([0.0, 0.0, 0.0, 0.1, 0.0, 0.0], [[0.0, 0.0, 0.1]])
    result = certify(t, free_grid(), WheelLimits(), 1)
    assert result.valid
    assert result.reason == CertificationReason.VALID
    assert result.brake_time_s == pytest.approx(0.2)


def test_certificate_carries_source_and_becomes_stale_when_map_or_route_changes():
    t = Trajectory(1, 2, [0.0], [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
                   np.empty((0, 3)), "spin", source="mpcc-run-17")
    result = certify(t, free_grid(), WheelLimits(), 2)
    assert result.valid
    assert result.trajectory_source == "mpcc-run-17"
    assert result.matches(t, free_grid(), WheelLimits(), 2)
    changed = GridSnapshot(2, (-2.5, -2.5), 0.01, np.zeros((500, 500), dtype=np.int8), False)
    assert not result.matches(t, changed, WheelLimits(), 2)
    assert not result.matches(t, free_grid(), WheelLimits(), 3)
    altered = Trajectory(1, 2, [0.0], [[0.01, 0.0, 0.0, 0.0, 0.0, 0.0]],
                         np.empty((0, 3)), "spin", source="mpcc-run-17")
    assert not result.matches(altered, free_grid(), WheelLimits(), 2)


def test_certificate_binds_full_grid_and_all_vehicle_limits():
    original = free_grid()
    t = Trajectory(1, 4, [0.0], [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
                   np.empty((0, 3)), "spin")
    result = certify(t, original, WheelLimits(), 4)
    assert result.matches(t, original, WheelLimits(), 4)
    cells = np.zeros((500, 500), dtype=np.int8)
    cells[250, 250] = CellState.BLOCKED
    variants = [
        GridSnapshot(1, (-2.5, -2.5), 0.01, np.zeros((500, 500), dtype=np.int8), True),
        GridSnapshot(1, (-2.49, -2.5), 0.01, np.zeros((500, 500), dtype=np.int8), False),
        GridSnapshot(1, (-2.5, -2.5), 0.02, np.zeros((500, 500), dtype=np.int8), False),
        GridSnapshot(1, (-2.5, -2.5), 0.01, cells, False),
    ]
    assert all(not result.matches(t, grid, WheelLimits(), 4) for grid in variants)
    assert not result.matches(t, original, WheelLimits(body_width_m=1.0), 4)
    assert not result.matches(t, original, WheelLimits(linear_accel_mps2=0.4), 4)
    assert not result.matches(t, original, WheelLimits(curvature_inv_m=0.8), 4)


def test_rejects_corner_collision_even_when_center_path_is_free():
    cells = np.zeros((500, 500), dtype=np.int8)
    cells[290, 308] = CellState.BLOCKED
    grid = GridSnapshot(1, (-2.5, -2.5), 0.01, cells, False)
    t = trajectory([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], mode="spin")
    assert certify(t, grid, WheelLimits(), 1).reason == CertificationReason.COLLISION


def test_rejects_one_centimeter_wall_between_time_nodes():
    cells = np.zeros((500, 500), dtype=np.int8)
    cells[210:290, 200] = CellState.BLOCKED
    grid = GridSnapshot(1, (-2.5, -2.5), 0.01, cells, False)
    for x in (-1.4, 0.2):
        endpoint = trajectory([x, 0.0, 0.0, 0.0, 0.0, 0.0], [], mode="spin")
        assert certify(endpoint, grid, WheelLimits(), 1).valid
    t = trajectory([-1.4, 0.0, 0.0, 0.2, 0.0, 0.0], [[0.0, 0.0, 0.1]], dt=8.0)
    assert certify(t, grid, WheelLimits(), 1).reason == CertificationReason.COLLISION


def test_sparse_single_cell_wall_and_unknown_survive_fast_candidate_filter():
    cells = np.zeros((500, 500), dtype=np.int8)
    cells[250, 250] = CellState.BLOCKED
    t = trajectory([-0.8, 0.0, 0.0, 0.2, 0.0, 0.0], [[0.0, 0.0, 0.1]], dt=8.0)
    blocked = GridSnapshot(1, (-2.5, -2.5), 0.01, cells, False)
    assert certify(t, blocked, WheelLimits(), 1).reason == CertificationReason.COLLISION
    cells[250, 250] = CellState.UNKNOWN
    strict = GridSnapshot(1, (-2.5, -2.5), 0.01, cells, False)
    permissive = GridSnapshot(1, (-2.5, -2.5), 0.01, cells, True)
    assert certify(t, strict, WheelLimits(), 1).reason == CertificationReason.UNKNOWN
    assert certify(t, permissive, WheelLimits(), 1).valid


def test_rejects_intermediate_rotation_contact_with_free_endpoint_footprints():
    cells = np.zeros((500, 500), dtype=np.int8)
    cells[290, 308] = CellState.BLOCKED
    grid = GridSnapshot(1, (-2.5, -2.5), 0.01, cells, False)
    for yaw in (-0.05, 0.05):
        endpoint = trajectory([0.0, 0.0, yaw, 0.0, 0.0, 0.0], [], mode="spin")
        assert certify(endpoint, grid, WheelLimits(), 1).valid
    t = trajectory([0.0, 0.0, -0.05, 0.0, 0.1, 0.0], [[0.0, 0.0, 0.0]], dt=1.0, mode="spin")
    assert certify(t, grid, WheelLimits(), 1).reason == CertificationReason.COLLISION


def test_rejects_collision_only_in_braking_tail():
    cells = np.zeros((500, 500), dtype=np.int8)
    cells[215:285, 312] = CellState.BLOCKED
    grid = GridSnapshot(1, (-2.5, -2.5), 0.01, cells, False)
    t = trajectory([0.0, 0.0, 0.0, 0.2, 0.0, 0.0], [], mode="forward")
    assert certify(t, grid, WheelLimits(), 1).reason == CertificationReason.BRAKING_COLLISION


def test_braking_tail_preserves_unknown_and_window_failure_reasons():
    cells = np.zeros((500, 500), dtype=np.int8)
    cells[215:285, 312] = CellState.UNKNOWN
    grid = GridSnapshot(1, (-2.5, -2.5), 0.01, cells, False)
    t = trajectory([0.0, 0.0, 0.0, 0.2, 0.0, 0.0], [], mode="forward")
    assert certify(t, grid, WheelLimits(), 1).reason == CertificationReason.BRAKING_UNKNOWN
    edge = trajectory([1.87, 0.0, 0.0, 0.2, 0.0, 0.0], [], mode="forward")
    assert certify(edge, free_grid(), WheelLimits(), 1).reason == CertificationReason.BRAKING_OUT_OF_BOUNDS


def test_braking_respects_curvature_when_angular_deceleration_is_slower():
    limits = WheelLimits(angular_accel_rps2=0.1)
    t = trajectory([0.0, 0.0, 0.0, 0.2, 0.1, 0.0], [], mode="forward")
    result = certify(t, free_grid(), limits, 1)
    assert result.valid
    assert result.brake_time_s == pytest.approx(1.0)
    cells = np.zeros((500, 500), dtype=np.int8)
    cells[215:285, 318] = CellState.BLOCKED
    grid = GridSnapshot(1, (-2.5, -2.5), 0.01, cells, False)
    assert certify(t, grid, limits, 1).reason == CertificationReason.BRAKING_COLLISION


@pytest.mark.parametrize("change,reason", [
    ("speed", CertificationReason.LIMIT),
    ("acceleration", CertificationReason.LIMIT),
    ("nan", CertificationReason.NONFINITE),
    ("dynamics", CertificationReason.DYNAMICS),
    ("epoch", CertificationReason.EPOCH_MISMATCH),
    ("route", CertificationReason.ROUTE_VERSION_MISMATCH),
])
def test_rejects_invalid_trajectory(change, reason):
    t = trajectory([0.0, 0.0, 0.0, 0.1, 0.0, 0.0], [[0.0, 0.0, 0.1]])
    states = np.array(t.states)
    controls = np.array(t.controls)
    epoch = 1
    version = 1
    if change == "speed":
        states[:, 3] = 0.3
    elif change == "acceleration":
        controls[0, 0] = 0.6
    elif change == "nan":
        states[1, 0] = np.nan
    elif change == "dynamics":
        states[1, 0] += 0.01
    elif change == "epoch":
        epoch = 2
    else:
        version = 2
    candidate = Trajectory(epoch, version, t.times, states, controls, "forward")
    assert certify(candidate, free_grid(), WheelLimits(), 1).reason == reason


def test_rejects_curvature_violation_and_spin_translation():
    t = trajectory([0.0, 0.0, 0.0, 0.01, 0.05, 0.0], [], mode="forward")
    assert certify(t, free_grid(), WheelLimits(), 1).reason == CertificationReason.LIMIT
    t = trajectory([0.0, 0.0, 0.0, 0.01, 0.0, 0.0], [], mode="spin")
    assert certify(t, free_grid(), WheelLimits(), 1).reason == CertificationReason.MODE


def test_unknown_policy_and_window_boundary_apply_to_entire_body():
    cells = np.zeros((500, 500), dtype=np.int8)
    cells[250, 308] = CellState.UNKNOWN
    strict = GridSnapshot(1, (-2.5, -2.5), 0.01, cells, False)
    allowed = GridSnapshot(1, (-2.5, -2.5), 0.01, cells, True)
    t = trajectory([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], mode="spin")
    assert certify(t, strict, WheelLimits(), 1).reason == CertificationReason.UNKNOWN
    assert certify(t, allowed, WheelLimits(), 1).valid
    edge = trajectory([2.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], mode="spin")
    assert certify(edge, free_grid(), WheelLimits(), 1).reason == CertificationReason.OUT_OF_BOUNDS


def braking_entry(mode='spin',progress=0.):
    state=np.array([0.,0.,.4,-.004,.02,0.])
    controls=np.array([[.5,0.,progress],[0.,.1,0.]])
    times=np.array([0.,.008,.208]);states=[state]
    for u,dt in zip(controls,np.diff(times)):states.append(step(states[-1],u,float(dt)))
    return Trajectory(1,1,times,np.array(states),controls,mode,'candidate-with-real-braking-entry')


def test_real_braking_entry_precedes_spin_without_zeroing_measurement():
    limits=WheelLimits(motion_model='differential_drive')
    candidate=braking_entry()
    assert candidate.states[0,3]==-.004 and candidate.states[1,3]==pytest.approx(0.)
    assert certify(candidate,free_grid(),limits,1).valid


def test_braking_entry_cannot_credit_route_progress_or_hide_bad_dynamics():
    from dataclasses import replace
    limits=WheelLimits(motion_model='differential_drive')
    assert not certify(braking_entry(progress=.1),free_grid(),limits,1).valid
    candidate=braking_entry();states=candidate.states.copy();states[1,0]+=.01
    assert not certify(replace(candidate,states=states),free_grid(),limits,1).valid


def test_braking_entry_remains_subject_to_full_body_collision():
    cells=np.zeros((500,500),dtype=np.int8);cells[250,250]=CellState.BLOCKED
    grid=GridSnapshot(1,(-2.5,-2.5),.01,cells,False)
    result=certify(braking_entry(),grid,WheelLimits(motion_model='differential_drive'),1)
    assert not result.valid and result.reason==CertificationReason.COLLISION


def test_braking_entry_must_not_accelerate_away_before_stopping():
    initial=np.array([0.,0.,0.,-.004,0.,0.]);controls=np.array([[-.1,0.,0.],[.5,0.,0.],[0.,.1,0.]])
    intervals=[.01,.01,.2];states=[initial]
    for u,dt in zip(controls,intervals):states.append(step(states[-1],u,dt))
    candidate=Trajectory(1,1,np.r_[0.,np.cumsum(intervals)],np.array(states),controls,'spin')
    assert not certify(candidate,free_grid(),WheelLimits(motion_model='differential_drive'),1).valid


@pytest.mark.parametrize('v,mode', [(.1,'spin'),(-.1,'spin'),(.1,'reverse'),(-.1,'forward')])
@pytest.mark.parametrize('duration', [.05,.1,.2])
def test_incomplete_real_braking_entry_and_zero_node(v, mode, duration):
    candidate = trajectory([0.,0.,0.,v,0.,0.], [[-np.sign(v)*.5,0.,0.]], dt=duration, mode=mode)
    result = certify(candidate, free_grid(), WheelLimits(motion_model='differential_drive'), 1)
    assert result.valid, result.reason
    assert candidate.states[0,3] == pytest.approx(v)
    assert result.brake_time_s == pytest.approx(max(0., abs(v)/.5-duration))


@pytest.mark.parametrize('v,mode',[(.1,'forward'),(-.1,'reverse')])
def test_same_direction_short_deceleration_remains_valid(v,mode):
    t=trajectory([0.,0.,0.,v,0.,0.],[[-np.sign(v)*.5,0.,0.]],dt=.05,mode=mode)
    assert certify(t,free_grid(),WheelLimits(motion_model='differential_drive'),1).valid


@pytest.mark.parametrize('control,dt,reason',[([.1,0.,0.],.05,CertificationReason.MODE),
    ([-.5,0.,.1],.05,CertificationReason.MODE), ([-.5,0.,0.],.25,CertificationReason.MODE),
    ([-.6,0.,0.],.05,CertificationReason.LIMIT)])
def test_incomplete_entry_rejects_invalid_motion(control,dt,reason):
    t=trajectory([0.,0.,0.,.1,0.,0.],[control],dt=dt,mode='spin')
    assert certify(t,free_grid(),WheelLimits(motion_model='differential_drive'),1).reason == reason


def test_incomplete_entry_still_checks_dynamics_and_both_collision_phases():
    limits=WheelLimits(motion_model='differential_drive')
    t=trajectory([0.,0.,0.,.1,0.,0.],[[-.5,0.,0.]],dt=.05,mode='spin')
    states=t.states.copy();states[1,0]+=.01
    bad=Trajectory(1,1,t.times,states,t.controls,t.mode)
    assert certify(bad,free_grid(),limits,1).reason == CertificationReason.DYNAMICS
    cells=np.zeros((500,500),dtype=np.int8);cells[250,250]=CellState.BLOCKED
    assert certify(t,GridSnapshot(1,(-2.5,-2.5),.01,cells,False),limits,1).reason == CertificationReason.COLLISION
    limits=WheelLimits(motion_model='differential_drive',body_length_m=.1,body_width_m=.1)
    cells=np.zeros((500,500),dtype=np.int8);cells[200:300,308]=CellState.BLOCKED
    assert certify(t,GridSnapshot(1,(-.25,-.25),.001,cells,False),limits,1).reason == CertificationReason.BRAKING_COLLISION


def test_incomplete_entry_cannot_remain_at_constant_wrong_mode_speed():
    t=trajectory([0.,0.,0.,.1,0.,0.],[[0.,0.,0.]],dt=.05,mode='spin')
    assert certify(t,free_grid(),WheelLimits(motion_model='differential_drive'),1).reason == CertificationReason.MODE


def test_incomplete_entry_still_obeys_curvature_limit():
    t=trajectory([0.,0.,0.,.1,.2,0.],[[-.5,0.,0.]],dt=.05,mode='spin')
    assert certify(t,free_grid(),WheelLimits(motion_model='curvature_limited'),1).reason == CertificationReason.LIMIT
