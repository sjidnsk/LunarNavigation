"""Batched endpoint truth queries must equal full-sensor scalar projections."""
from dataclasses import replace
import numpy as np
import pytest
from lunar_drl_exploration.contracts import Pose
from lunar_drl_exploration.privileged import _RemovalPrediction
from test_frontier_privileged import fixture


@pytest.mark.parametrize('range_m,fov_deg', [(10.,60.),(24.3,90.),(100.,180.),(100.,360.)])
@pytest.mark.parametrize('invalid', [None,'nan','unknown','blocked_center','blocked_demand'])
def test_batched_projection_matches_scalar_reveal(range_m,fov_deg,invalid):
    terrain,_,_,s,sensor,before=fixture(origin=(-10.05,-4.15),resolution=.2)
    sensor=replace(sensor,range_m=range_m,fov_deg=fov_deg,offset_yaw_rad=.17)
    terrain.heights=terrain.heights.copy();terrain.intrinsic=terrain.intrinsic.copy()
    if invalid=='nan':terrain.heights[6,7]=np.nan
    if invalid=='unknown':terrain.intrinsic[6,7]=0
    if invalid=='blocked_center':terrain.intrinsic[6,6]=2
    if invalid=='blocked_demand':terrain.intrinsic[6,7]=2
    goals=np.array([(s.pose.x+dx,s.pose.y,yaw) for dx in (0.,-.2)
                    for yaw in np.linspace(-3.,3.,10)])
    prediction=_RemovalPrediction(before,terrain)
    expected=np.array([prediction.predict(terrain,Pose(*g),sensor) for g in goals])
    np.testing.assert_array_equal(prediction.predict_many(terrain,goals,sensor),expected)
    np.testing.assert_array_equal(prediction.predict_many(terrain,goals[::-1],sensor),expected[::-1])


def test_empty_projection_still_validates_sensor_and_pose():
    from lunar_drl_exploration.frontiers import build_frontiers
    from lunar_drl_exploration.task_analysis import measured_workspace
    from test_task_analysis import task
    terrain,_,_,s,sensor,_=fixture()
    t=task(-10,-10,-9,-9)
    before=build_frontiers(s,measured_workspace(s,t,sensor),t)
    assert not len(before.inside.cell_ids)
    p=_RemovalPrediction(before,terrain)
    assert not p.predict_many(terrain,np.array([[s.pose.x,s.pose.y,0.]]),sensor).any()
    with pytest.raises(ValueError):p.predict_many(terrain,np.array([[999.,999.,0.]]),sensor)
    with pytest.raises(ValueError):p.predict_many(terrain,np.array([[s.pose.x,s.pose.y,np.nan]]),sensor)
