"""规划质量、任务策略、输入等待与平台传输职责回归。"""
from pathlib import Path
import importlib.util
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits
from scripts.navigation.one_cm.nav1cm.local_mpcc import MpccPlanner


def disposition():
    p=Path(__file__).resolve().parents[1]/'ros2_ws/src/lunar_pure_wheeled_controller/python/lunar_pure_wheeled_controller/motion_configuration.py'
    spec=importlib.util.spec_from_file_location('input_policy',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    return m.input_disposition


@pytest.mark.parametrize('advance',[0.,.001,.0049])
def test_safe_small_progress_is_not_a_safety_rejection(advance):
    p=object.__new__(MpccPlanner);p.limits=WheelLimits(motion_model='differential_drive');p.offsets=[0.];p.radius=.5
    field=type('Field',(),{'sample':lambda self,x,y:(10.,0.,0.)})()
    xs=np.zeros((2,6));xs[-1,5]=advance
    assert p._candidate_reason(xs,np.zeros((1,3)),'forward',field,1.)=='CANDIDATE'
    xs[0,3]=100.
    assert p._candidate_reason(xs,np.zeros((1,3)),'forward',field,1.)=='LIMIT'


@pytest.mark.parametrize('reason,expected',[
 ('missing_or_stale_feedback','WAITING_FOR_POSE'),('missing_or_stale_map','WAITING_FOR_MAP'),
 ('stale_map_after_optimization','WAITING_FOR_MAP'),('publication_certificate_expired','RECOMPUTE'),
 ('map_changed_during_certification','RECOMPUTE'),('future_feedback','FAIL'),
 ('latest_map_or_aged_motion_unsafe','FAIL')])
def test_wait_recompute_and_failure_are_distinct(reason,expected):
    classify=disposition();assert classify(reason)==expected
    assert classify(reason,hard_failure=True)=='FAIL'


# 已删除的固定指令龄门禁不再建立回归；转发/断连职责统一由
# test_p4_tcp_runtime 的 TcpCommandRelay 行为测试覆盖。
