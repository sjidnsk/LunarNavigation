"""独立解析矩形净空和相邻状态扫掠下界，不读生产认证。"""
import importlib.util
from pathlib import Path
import math
import pytest
ROOT=Path(__file__).resolve().parents[1]


def module():
    path=ROOT/'scripts/navigation/one_cm/path_follower_validation/clearance.py'
    assert path.exists(), 'independent clearance evidence missing'
    spec=importlib.util.spec_from_file_location('independent_clearance',path)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    return mod


def row(x=0.,yaw=0.,t=0.):return dict(x=x,y=0.,yaw=yaw,t=t)


def test_rectangle_distance_is_not_body_collision_only():
    m=module();box=[(1.,-.1,1.2,.1)]
    assert m.pose_distance(row(),box)==pytest.approx(.409)
    assert m.pose_distance(row(yaw=math.pi/2),box)==pytest.approx(.591)
    assert m.pose_distance(row(x=.5),box)==0.
    assert not m.measure([row(),row(t=.01)],lambda t:[(.75,-.1,.8,.1)],.2)['pass']


def test_swept_record_is_conservative_and_empty_evidence_fails():
    m=module();boxes=lambda t:[(1.,-.1,1.2,.1)]
    result=m.measure([row(),row(x=.02,t=.01)],boxes,.2)
    assert result['pass']
    assert result['minimum_pose_distance_m']==pytest.approx(.389)
    assert result['minimum_swept_lower_bound_m']==pytest.approx(.379)
    assert not m.measure([row(),row(x=.02,t=.01)],boxes,.4)['pass']
    assert not m.measure([],boxes,.2)['pass']
    assert m.measure([row(),row(t=.01)],lambda t:[],.4)['pass']
