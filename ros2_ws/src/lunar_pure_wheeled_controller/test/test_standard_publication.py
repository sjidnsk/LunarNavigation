"""标准Path发布必须有界，真实耗时不能靠重复认证追赶。"""
from types import SimpleNamespace as S
import pytest
from lunar_pure_wheeled_controller.node import PureWheeledControllerNode

@pytest.mark.parametrize('case', ['slow_certificate', 'map_changes', 'unsafe', 'valid'])
def test_publication_certification_is_single_pass_and_checks_completion(case):
    clock=[10_000_000_000]
    grid=S(revision=1,epoch='1',origin=(0.,0.))
    newer=S(revision=2,epoch='1',origin=(0.,0.))
    calls=[];published=[];unavailable=[]
    def update(*args,**kwargs):
        clock[0]+=60_000_000
        return S(command=S(linear_x_mps=.1,angular_z_radps=0.),phase='TRACKING')
    def certify(*args):
        calls.append(args)
        # 第一轮慢，第二轮快：旧循环会追赶成功并错误发布。
        if case=='slow_certificate' and len(calls)==1:clock[0]+=60_000_000
        else:clock[0]+=1_000_000
        return case!='unsafe'
    def latest(state):
        return newer if case=='map_changes' and calls else grid
    executor=S(config=S(compute_budget_s=.05),request_stop=lambda *a:None,
        set_map=lambda *a:None,update=update,certify_publication=certify,
        diagnostics={},_stopped=lambda *a:False)
    node=S(_odometry=object(),_odometry_received_at=0.,_odometry_timeout_s=.5,
        _active=object(),_path_received_at=0.,_path_timeout_s=.35,
        _standard_state=object(),_standard_state_ns=clock[0],_last_control_ns=None,
        _executor=executor,_input_is_stale=lambda *a:False,_standard_grid=latest,
        get_clock=lambda:S(now=lambda:S(nanoseconds=clock[0])),
        _standard_unavailable=unavailable.append,_mppi_diagnostics=S(publish=lambda *a:None),
        _publish_twist=lambda *a:published.append(a),_publish_execution_status=lambda:None)
    PureWheeledControllerNode._tick_standard_path(node)
    assert len(calls)==1
    if case=='valid':
        assert published==[(.1,0.)] and not unavailable
    else:
        assert not published and unavailable


def test_new_feedback_cannot_renew_the_state_used_by_this_calculation():
    clock=[10_000_000_000];received=[0.];reasons=[];published=[]
    grid=S(revision=1,epoch='1',origin=(0.,0.))
    def update(*a,**kw):
        clock[0]+=600_000_000
        node._odometry_received_at=.6  # 模拟新消息；本轮state仍来自0秒。
        return S(command=S(linear_x_mps=.1,angular_z_radps=0.),phase='TRACKING')
    node=S(_odometry=object(),_odometry_received_at=0.,_odometry_timeout_s=.5,
        _active=object(),_path_received_at=0.,_path_timeout_s=.35,
        _standard_state=object(),_standard_state_ns=clock[0],_last_control_ns=None,
        _executor=S(config=S(compute_budget_s=.05),set_map=lambda *a:None,update=update,
            diagnostics={},_stopped=lambda *a:False,certify_publication=lambda *a:True),
        _input_is_stale=lambda stamp,limit:(clock[0]-10_000_000_000)/1e9-stamp>limit,
        _standard_grid=lambda state:grid,get_clock=lambda:S(now=lambda:S(nanoseconds=clock[0])),
        _standard_unavailable=reasons.append,_mppi_diagnostics=S(publish=lambda *a:None),
        _publish_twist=lambda *a:published.append(a),_publish_execution_status=lambda:None)
    PureWheeledControllerNode._tick_standard_path(node)
    assert reasons==['stale_feedback_after_optimization'] and not published
