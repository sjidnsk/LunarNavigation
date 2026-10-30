"""标准Path不拥有任务级无进展累计；导航统一处理。"""
import pytest
from test_standard_path_follower import follower
from lunar_pure_wheeled_controller.tracking import TrackingState


@pytest.mark.parametrize('points',[((0.,0.,0.),(0.,0.,1.)),((0.,0.,0.),(1.,0.,0.))])
def test_standard_reference_does_not_accumulate_task_stall(points):
    e=follower();state=TrackingState(0.,0.,0.);e.set_standard_path(points)
    e._accept_pending(state)
    for i in range(400):
        assert not e._observe_progress(state,0. if len(e.path.points)==1 else 1.,1.,.05)
    assert e.no_progress_s==0. and e._last_pose is None
    assert e.failure is None


def test_rolling_turn_reference_cannot_create_a_second_stall_clock():
    e=follower();state=TrackingState(0.,0.,0.)
    for i in range(80):
        e.set_standard_path(((0.,0.,0.),(0.,0.,.3+.001*i)),i)
        e._accept_pending(state)
        assert not e._observe_progress(state,0.,.3+.001*i,.2)
    assert e.no_progress_s==0. and e._last_pose is None


def test_existing_execution_failure_remains_latched():
    e=follower();e.set_standard_path(((0.,0.,0.),(0.,0.,1.)))
    e.failure='UNSAFE';r=e.update(TrackingState(0.,0.,0.),.05)
    assert e.failure=='UNSAFE' and r.phase=='FAILED'
