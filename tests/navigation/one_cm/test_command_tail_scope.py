"""指令证明只认证response与hold；通用轨迹保持完整制动尾默认。"""
from types import SimpleNamespace
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm import certification as module
from scripts.navigation.one_cm.nav1cm.contracts import GridSnapshot, WheelLimits, VehicleState
from scripts.navigation.one_cm.nav1cm.execution import command_hold_proof, certified_velocity_command, certified_command
from tests.navigation.one_cm.test_certification import trajectory, free_grid


def test_hold_proof_ends_at_issued_velocity_without_stop():
    limits=WheelLimits(motion_model='differential_drive')
    action=trajectory([0,0,0,.1,.04,0],[[.2,.1,0]],dt=.1)
    proof=command_hold_proof(action,free_grid(),limits,1.2)
    assert proof.times[-1]==pytest.approx(1.3)
    assert proof.states[-1,3:5]==pytest.approx(action.states[-1,3:5])
    assert np.all(proof.controls[1:]==0)


def test_no_tail_certificate_cannot_be_reused_as_full_certificate(monkeypatch):
    grid=free_grid();limits=WheelLimits();t=trajectory([0,0,0,.1,0,0],[[0,0,0]])
    original=module._brake_tail;calls=[]
    def tail(*args):calls.append(1);return original(*args)
    monkeypatch.setattr(module,'_brake_tail',tail)
    cert=module.certify(t,grid,limits,1,check_braking_tail=False)
    assert cert.valid and not cert.braking_tail_checked and cert.brake_time_s==0
    assert not calls
    assert not cert.matches(t,grid,limits,1)
    assert cert.matches(t,grid,limits,1,check_braking_tail=False)
    assert not cert.matches(t,grid,limits,2,check_braking_tail=False)
    full=module.certify(t,grid,limits,1)
    assert calls and full.braking_tail_checked
    assert full.matches(t,grid,limits,1)
    assert full.matches(t,grid,limits,1,check_braking_tail=False)


@pytest.mark.parametrize('obstacle,accepted',[(.85,True),(.5,False)])
def test_real_hold_collision_still_rejects_but_extra_stop_only_collision_does_not(obstacle,accepted):
    limits=WheelLimits(body_length_m=.2,body_width_m=.2,linear_speed_mps=1.,motion_model='differential_drive')
    cells=np.zeros((500,500),np.int8);cells[245:255,int((obstacle+2.5)/.01)]=1
    grid=GridSnapshot(1,(-2.5,-2.5),.01,cells)
    action=trajectory([0,0,0,.5,0,0],[[0,0,0]],dt=.05)
    cert=module.certify(action,grid,limits,1)
    result=SimpleNamespace(candidate=action,certificate=cert,action=action,action_certificate=cert,grid=grid,status='TRACKING',reference_phase_s=0.)
    state=VehicleState(0,0,0,.5,0)
    proof=command_hold_proof(action,grid,limits,1.2)
    assert not module.certify(proof,grid,limits,1).valid
    assert module.certify(proof,grid,limits,1,check_braking_tail=False).valid == accepted
    expected=(.5,0.) if accepted else (0.,0.)
    assert certified_command(result,state,limits,command_hold_s=1.2)==pytest.approx(expected)
    assert certified_velocity_command(result,state,limits,reference_interval_s=.05,last_command=(.5,0),elapsed_s=.05,command_hold_s=1.2)==pytest.approx(expected)


@pytest.mark.parametrize('value',[0,1,'false',None])
def test_certificate_scope_flags_require_strict_bool(value):
    from dataclasses import replace
    t=trajectory([0,0,0,.1,0,0],[[0,0,0]])
    grid=free_grid();limits=WheelLimits()
    with pytest.raises(TypeError,match='must be bool'):
        module.certify(t,grid,limits,1,check_braking_tail=value)
    full=module.certify(t,grid,limits,1)
    with pytest.raises(TypeError,match='must be bool'):
        full.matches(t,grid,limits,1,check_braking_tail=value)
    with pytest.raises(TypeError,match='must be bool'):
        replace(full,braking_tail_checked=value)


def test_velocity_command_never_calls_implicit_brake_tail(monkeypatch):
    grid=free_grid();limits=WheelLimits(motion_model='differential_drive')
    action=trajectory([0,0,0,.1,0,0],[[0,0,0]])
    result=SimpleNamespace(candidate=action,certificate=None,reference_phase_s=0.,grid=grid,status='BUFFER_EXECUTING')
    def forbidden(*args):raise AssertionError('command proof must not append implicit stop')
    monkeypatch.setattr(module,'_brake_tail',forbidden)
    assert certified_velocity_command(result,VehicleState(0,0,0,.1,0),limits,
        reference_interval_s=.2,last_command=(.1,0),elapsed_s=.05,command_hold_s=1.2)==pytest.approx((.1,0))


@pytest.mark.parametrize('change,reason', [('epoch','EPOCH_MISMATCH'),('version','ROUTE_VERSION_MISMATCH'),
    ('speed','LIMIT'),('dynamics','DYNAMICS'),('mode','MODE'),('nonfinite','NONFINITE')])
def test_no_tail_keeps_identity_dynamics_and_limit_checks(change,reason):
    from dataclasses import replace
    t=trajectory([0,0,0,.1,0,0],[[0,0,0]])
    if change=='epoch':t=replace(t,map_epoch=2)
    if change=='version':t=replace(t,route_version=2)
    if change in ('speed','dynamics','nonfinite'):
        states=t.states.copy()
        if change=='speed':states[:,3]=1.
        elif change=='dynamics':states[-1,0]+=.01
        else:states[-1,0]=float('nan')
        t=replace(t,states=states)
    if change=='mode':t=replace(t,mode='unsupported')
    result=module.certify(t,free_grid(),WheelLimits(),1,check_braking_tail=False)
    assert not result.valid and result.reason.value==reason


def test_no_tail_certificate_still_binds_map_and_vehicle_limits():
    from dataclasses import replace
    t=trajectory([0,0,0,.1,0,0],[[0,0,0]])
    grid=free_grid();limits=WheelLimits()
    cert=module.certify(t,grid,limits,1,check_braking_tail=False)
    assert not cert.matches(t,replace(grid,origin_xy=(-2.49,-2.5)),limits,1,check_braking_tail=False)
    assert not cert.matches(t,grid,replace(limits,linear_speed_mps=.3),1,check_braking_tail=False)
