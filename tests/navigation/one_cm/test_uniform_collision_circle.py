"""统一生产圆包络：原始格面积、yaw 不变性与真实扫掠余量。"""
from dataclasses import replace
import math
import os
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits, GridSnapshot, CertificationReason, Trajectory
from scripts.navigation.one_cm.nav1cm.certification import (_body_collision, _batch_body_collision, prepare_certification, certify)


@pytest.fixture(params=[False, True], ids=['python', 'native'])
def backend(request):
    if not request.param:
        return None
    path=os.environ.get('NAV1CM_GEOMETRY_LIB')
    if not path:
        pytest.skip('explicit newly built native library required')
    from scripts.navigation.one_cm.nav1cm.geometry_native import NativeGeometry
    return NativeGeometry(path)


def limits():
    return replace(WheelLimits(), minimum_clearance_m=.75-math.hypot(1.182,.818)/2)


def grid_at(x, y, value=1, allow_unknown=False):
    # 格左下角为指定坐标，不能用格中心近似圆到格面积的距离。
    cells=np.zeros((400,400),dtype=np.int8)
    cells[200,200]=value
    return GridSnapshot(1,(x-2.,y-2.),.01,cells,allow_unknown)


@pytest.mark.parametrize('distance,reason',[(.7501,None),(.7499,CertificationReason.COLLISION)])
def test_all_yaws_share_exact_75cm_circle(distance,reason,backend):
    context=prepare_certification(grid_at(0.,distance),backend=backend)
    poses=np.column_stack((np.zeros((65,2)),np.linspace(-math.pi,math.pi,65)))
    assert all(_body_collision(p,0.,context,limits())==reason for p in poses)
    assert _batch_body_collision(poses,0.,context,limits())==(reason,len(poses) if reason is None else 1)


@pytest.mark.parametrize('distance,reason',[(.7501,None),(.7499,CertificationReason.COLLISION)])
def test_corner_uses_nearest_point_on_cell_not_center(distance,reason,backend):
    context=prepare_certification(grid_at(distance/math.sqrt(2),distance/math.sqrt(2)),backend=backend)
    assert _body_collision((0.,0.,1.234),0.,context,limits())==reason


@pytest.mark.parametrize('allow_unknown,reason',[(False,CertificationReason.UNKNOWN),(True,None)])
def test_unknown_remains_policy_bound(allow_unknown,reason,backend):
    context=prepare_certification(grid_at(0.,.74,-1,allow_unknown),backend=backend)
    assert _body_collision((0.,0.,0.),0.,context,limits())==reason


def test_circle_outside_grid_for_every_yaw(backend):
    grid=GridSnapshot(1,(-2.,-.74),.01,np.zeros((400,400),np.int8))
    context=prepare_certification(grid,backend=backend)
    for yaw in (0.,math.pi/2,math.pi/4):
        assert _body_collision((0.,0.,yaw),0.,context,limits())==CertificationReason.OUT_OF_BOUNDS


def test_dynamic_sweep_margin_and_real_speed_are_retained(backend):
    context=prepare_certification(grid_at(0.,.751),backend=backend)
    assert _body_collision((0.,0.,0.),0.,context,limits()) is None
    assert _body_collision((0.,0.,0.),.002,context,limits())==CertificationReason.COLLISION
    trajectory=Trajectory(1,1,[0.],[[0.,0.,math.pi/2,.2,0.,0.]],np.empty((0,3)),'forward')
    result=certify(trajectory,context.grid,limits(),1,backend=backend)
    assert result.reason==CertificationReason.BRAKING_COLLISION
    assert trajectory.states[0,3]==.2


def test_zero_clearance_prototype_keeps_rectangle(backend):
    context=prepare_certification(grid_at(0.,.58),backend=backend)
    assert _body_collision((0.,0.,0.),0.,context,WheelLimits()) is None
    assert _body_collision((0.,0.,math.pi/2),0.,context,WheelLimits())==CertificationReason.COLLISION


@pytest.mark.parametrize('cm,accepted',[(74,False),(76,True)])
def test_pose_goal_uses_same_yaw_independent_circle(cm,accepted):
    from scripts.navigation.one_cm.nav1cm.contracts import CellRect,WorldSnapshot
    from scripts.navigation.one_cm.nav1cm.pose_goal import check_pose_goal
    world=WorldSnapshot(1,(CellRect(0,cm,1,cm+1),),())
    for yaw in np.linspace(-math.pi,math.pi,65):
        assert check_pose_goal(world,(0.,0.),yaw,limits()).accepted==accepted


@pytest.mark.parametrize('dense',[False,True])
def test_actual_native_matches_python_for_random_grid_and_swept_margins(dense):
    path=os.environ.get('NAV1CM_GEOMETRY_LIB')
    if not path:
        pytest.skip('explicit newly built native library required')
    from scripts.navigation.one_cm.nav1cm.geometry_native import NativeGeometry
    rng=np.random.default_rng(901)
    cells=rng.choice(np.array([0,1,-1],np.int8),size=(200,200),p=(.7,.2,.1) if dense else (.998,.001,.001))
    grid=GridSnapshot(1,(-1.,-1.),.01,cells)
    python=prepare_certification(grid)
    native=prepare_certification(grid,backend=NativeGeometry(path))
    for margin in (0.,.002,.01):
        poses=rng.uniform((-1.,-1.,-math.pi),(1.,1.,math.pi),(150,3))
        for pose in poses:
            assert _body_collision(pose,margin,native,limits())==_body_collision(pose,margin,python,limits())
        assert _batch_body_collision(poses,margin,native,limits())==_batch_body_collision(poses,margin,python,limits())


def test_native_loader_rejects_old_rectangle_abi(monkeypatch):
    from types import SimpleNamespace
    from scripts.navigation.one_cm.nav1cm import geometry_native
    def old_abi():
        return 2
    monkeypatch.setattr(geometry_native.ctypes,'CDLL',lambda path:SimpleNamespace(nav1cm_geometry_abi=old_abi))
    with pytest.raises(ValueError,match='ABI'):
        geometry_native.NativeGeometry('old-rectangle-library.dll')


def test_batch_reports_first_reject_after_clear_poses(backend):
    context=prepare_certification(grid_at(0.,.74),backend=backend)
    poses=np.array([[-1.,0.,.1],[-1.,.2,1.],[0.,0.,2.]])
    assert [_body_collision(p,0.,context,limits()) for p in poses]==[None,None,CertificationReason.COLLISION]
    assert _batch_body_collision(poses,0.,context,limits())==(CertificationReason.COLLISION,3)
