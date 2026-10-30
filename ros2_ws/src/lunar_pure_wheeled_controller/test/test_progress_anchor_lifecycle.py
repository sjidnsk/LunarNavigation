"""滚动参考重绑只能转移已有物理进展，不能吞掉或凭空产生进展。"""
import numpy as np
import pytest
from test_standard_path_follower import follower
from lunar_pure_wheeled_controller.tracking import TrackingState


@pytest.mark.parametrize('direction',[1.,-1.])
def test_subthreshold_physical_steps_survive_repeated_rebinding(direction):
    e=follower();e.set_standard_path(((0.,0.,0.),(direction,0.,0.)))
    e.update(TrackingState(0.,0.,0.),.05);e.no_progress_s=14.7
    for i in range(1,5):
        state=TrackingState(direction*.01*i,0.,0.)
        e._orient_progress_anchor(state)
        e._observe_progress(state,1.-.01*i,0.,.05)
    assert e.no_progress_s==0.
    assert e._translation_credit_m==0.


def test_signed_backtracking_cancels_across_rebinding():
    e=follower();e.set_standard_path(((0.,0.,0.),(1.,0.,0.)))
    e.update(TrackingState(0.,0.,0.),.05);e.no_progress_s=.7
    for x in (.02,0.,.02,0.):
        state=TrackingState(x,0.,0.)
        e._orient_progress_anchor(state)
        e._observe_progress(state,1.-x,0.,.05)
    assert e.no_progress_s==pytest.approx(.9)
    assert e._translation_credit_m==pytest.approx(0.)


def test_reference_only_direction_changes_do_not_create_translation_credit():
    e=follower();state=TrackingState(0.,0.,0.)
    e.set_standard_path(((0.,0.,0.),(1.,0.,0.)));e.update(state,.05);e.no_progress_s=.7
    for end in ((0.,1.,1.57),(-1.,0.,3.14),(0.,-1.,-1.57)):
        from lunar_pure_wheeled_controller.path_geometry import Polyline
        e.path=Polyline(((0.,0.,0.),end));e._orient_progress_anchor(state)
        e._observe_progress(state,1.,end[2],.05)
    assert e.no_progress_s==pytest.approx(.85)
    assert e._translation_credit_m==0.


def test_clear_and_new_session_do_not_inherit_translation_credit():
    from test_terminal_execution_contract import ref
    e=follower();e.offer_execution_reference(((0.,0.,0.),(1.,0.,0.)),ref())
    e.update(TrackingState(0.,0.,0.),.05)
    e._orient_progress_anchor(TrackingState(.02,0.,0.))
    assert e._translation_credit_m==pytest.approx(.02)
    e.offer_execution_reference(((.02,0.,0.),(1.,0.,0.)),ref(session='new'))
    e.update(TrackingState(.02,0.,0.),.05)
    assert e._translation_credit_m==0.
    e._orient_progress_anchor(TrackingState(.04,0.,0.));assert e._translation_credit_m>0
    e.clear();assert e._translation_credit_m==0. and e._last_pose is None


def test_r16_small_actual_moves_transfer_through_rebinds_before_terminal():
    """采用实际r16锚点/位姿；累计有向位移约3.75cm，不应被切片丢失。"""
    e=follower();e.set_standard_path(((.16784148027020365,1.7813110271059407,1.695),(.13,1.9,1.9)))
    e.update(TrackingState(.16784148027020365,1.7813110271059407,1.695),.05)
    e._last_pose=(np.array([.16784148027020365,1.7813110271059407]),np.array([-.3092525195466271,.9509799572830455]),.19082703967227577)
    e.no_progress_s=14.7
    start=e._last_pose[0].copy();end=np.array([.1575565790888329,1.8173914314867397])
    for i in range(1,5):
        x,y=start+(end-start)*i/4;state=TrackingState(x,y,1.886)
        e._orient_progress_anchor(state)
        e._observe_progress(state,.0955,.154,.05)
    assert e.no_progress_s==0.
    assert e.failure is None
