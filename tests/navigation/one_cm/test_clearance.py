"""所选能力净空必须约束真实车体外的空间，而非只保留 YAML。"""
from dataclasses import replace
from pathlib import Path
import inspect
import sys
import math
import numpy as np
import pytest
import yaml
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts/navigation/one_cm'))
sys.path.insert(0,str(ROOT/'scripts/navigation/one_cm/ros2/lunar_nav1cm_planner'))
from nav1cm.contracts import WheelLimits,CellRect,WorldSnapshot,GridSnapshot,Trajectory,CertificationReason
from nav1cm.certification import certify
from nav1cm.global_planner import AdaptiveGlobalPlanner
from nav1cm.follower_map import center_support
from lunar_nav1cm_planner.configuration import load_configuration


def selected(tmp_path,margin,overrides=None):
    doc=yaml.safe_load((ROOT/'config/wheel.yaml').read_text(encoding='utf-8'))
    doc['capability']['minimum_clearance_m']=margin
    path=tmp_path/('wheel-'+str(margin)+'.yaml');path.write_text(yaml.safe_dump(doc),encoding='utf-8')
    return load_configuration(path,overrides)[0]


def wall_grid(front,unknown=False):
    cells=np.zeros((500,500),np.int8)
    col=int(round((front+2.5)*100));cells[225:275,col:col+1]=-1 if unknown else 1
    return GridSnapshot(1,(-2.5,-2.5),.01,cells,False)


def stationary(v=0.):
    return Trajectory(1,1,[0.],[[0.,0.,0.,v,0.,0.]],np.empty((0,3)),'forward')


@pytest.mark.parametrize('margin',[.2,.4])
def test_selected_loader_retains_clearance_and_actual_body(tmp_path,margin):
    limits=selected(tmp_path,margin)
    assert getattr(limits,'minimum_clearance_m',None)==margin
    assert (limits.body_length_m,limits.body_width_m)==(1.182,.818)


@pytest.mark.parametrize('margin',[.2,.4])
def test_body_free_but_clearance_insufficient_is_rejected(tmp_path,margin):
    limits=selected(tmp_path,margin)
    front=round(limits.body_length_m/2+margin-.05,2)
    grid=wall_grid(front)
    assert certify(stationary(),grid,WheelLimits(),1).valid  # 真正车体无碰撞。
    assert certify(stationary(),grid,limits,1).reason==CertificationReason.COLLISION
    assert certify(stationary(),wall_grid(round(limits.body_length_m/2+margin+.05,2)),limits,1).valid


@pytest.mark.parametrize('margin',[.2,.4])
def test_required_global_radius_cannot_be_cancelled_by_request(tmp_path,margin):
    limits=selected(tmp_path,margin);required=math.hypot(1.182,.818)/2+margin
    planner=AdaptiveGlobalPlanner(limits,.9)
    assert planner.clearance_radius_m>=required
    world=WorldSnapshot(1,(CellRect(95,-10,96,10),),())
    assert not planner.check_goal(world,(0.,0.)).accepted if margin==.4 else True
    assert AdaptiveGlobalPlanner(WheelLimits(),.409).clearance_radius_m==.409


def test_explicit_clearance_floor_validation_and_certificate_identity(tmp_path):
    limits=selected(tmp_path,.2,{'minimum_clearance_m':.4})
    assert limits.minimum_clearance_m==.4
    assert selected(tmp_path,.4,{'minimum_clearance_m':.1}).minimum_clearance_m==.4
    free=GridSnapshot(1,(-2.5,-2.5),.01,np.zeros((500,500),np.int8))
    cert=certify(stationary(),free,limits,1)
    assert not cert.matches(stationary(),free,replace(limits,minimum_clearance_m=.2),1)
    for value in (-.1,float('nan'),float('inf')):
        with pytest.raises(ValueError):selected(tmp_path,value)


@pytest.mark.parametrize('margin',[.2,.4])
def test_clearance_reaches_braking_tail_and_unknown(tmp_path,margin):
    limits=selected(tmp_path,margin)
    # 停车距离 .04m；当前车体+净空无碰撞，制动尾段违反净空。
    front=round(limits.body_length_m/2+margin+.02,2)
    result=certify(stationary(.2),wall_grid(front),limits,1)
    assert result.reason==CertificationReason.BRAKING_COLLISION
    assert certify(stationary(.2),wall_grid(front),WheelLimits(),1).valid
    near=round(limits.body_length_m/2+margin-.05,2)
    assert certify(stationary(),wall_grid(near,True),limits,1).reason==CertificationReason.UNKNOWN


@pytest.mark.parametrize('margin',[.2,.4])
def test_follower_support_margin_and_halo_without_unknown_relabel(tmp_path,margin):
    assert 'minimum_clearance_m' in inspect.signature(center_support).parameters
    free=(CellRect(-500,-500,500,500),)
    x=int(round((1.182/2+margin-.05)*100))
    world=WorldSnapshot(1,(CellRect(x,-10,x+1,10),),free)
    raw=center_support(world,0.,0.,body_length_m=1.182,body_width_m=.818)
    assert raw.eligible[250,250]
    padded=center_support(world,0.,0.,body_length_m=1.182,body_width_m=.818,minimum_clearance_m=margin)
    assert not padded.eligible[250,250]
    far=int(math.ceil((math.hypot(1.182,.818)/2+margin+.05)*100))
    safe=center_support(WorldSnapshot(2,(CellRect(far,-10,far+1,10),),free),0.,0.,body_length_m=1.182,body_width_m=.818,minimum_clearance_m=margin)
    assert safe.eligible[250,250] and safe.eligible[490,250]
    unknown=center_support(WorldSnapshot(3,(),()),0.,0.,body_length_m=1.182,body_width_m=.818,minimum_clearance_m=margin,allow_unknown=True)
    assert np.all(unknown.cells==-1) and unknown.eligible.all()


@pytest.mark.parametrize('margin',[.2,.4])
def test_native_and_python_clearance_and_braking_agree(tmp_path,margin):
    import os
    from nav1cm.geometry_native import NativeGeometry
    native=NativeGeometry(os.environ['NAV1CM_GEOMETRY_LIB'])
    limits=selected(tmp_path,margin)
    for front,velocity,unknown in [(round(.591+margin-.05,2),0.,False),
                                   (round(.591+margin+.05,2),0.,False),
                                   (round(.591+margin+.02,2),.2,False),
                                   (round(.591+margin-.05,2),0.,True)]:
        grid=wall_grid(front,unknown);trajectory=stationary(velocity)
        reference=certify(trajectory,grid,limits,1)
        accelerated=certify(trajectory,grid,limits,1,backend=native)
        assert (accelerated.valid,accelerated.reason)==(reference.valid,reference.reason)
