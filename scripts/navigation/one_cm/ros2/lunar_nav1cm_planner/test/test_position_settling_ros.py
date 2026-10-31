"""使用实际ROS消息验证owner位置目标沿原字段发布；配置失败须清理资源。"""
from types import SimpleNamespace as S
import math
import pytest
from test_runtime import build_runtime


def test_existing_execution_reference_carries_position_target():
    from lunar_nav1cm_planner.node import PlannerNode
    from nav1cm.execution_contract import FOLLOW
    _,session,_,_=build_runtime();c=session.coordinator
    c.goal_tolerance_m=.1;c.goal_position_target_m=.02
    captured=[];registered=[]
    owner=S(_execution_contract_enabled=True,_execution_revision=0,
        runtime=S(execution_context=('a'*32,1,FOLLOW),session=session,map_frame='map',register_execution_reference=registered.append),
        _execution_publisher=S(publish=captured.append),_global_publisher=S(publish=lambda m:None),_reference_publisher=S(publish=lambda m:None),_local_publisher=S(publish=lambda m:None))
    path=S(frame_id='map',stamp_ns=1_000_000,poses=())
    PlannerNode._publish(owner,S(local=path,global_route=path,global_reference=path))
    assert captured[0].position_tolerance_m==.02
    assert c.goal_tolerance_m==.1 and registered[0].position_tolerance_m==.02


@pytest.mark.parametrize('target',[.02,0.,-1.,float('nan'),None,'invalid'])
def test_resource_configuration_validates_position_target_and_cleans(monkeypatch,target):
    from lunar_nav1cm_planner import node as module
    _,session,_,_=build_runtime();c=session.coordinator;closed=[]
    monkeypatch.setattr(c,'close',lambda:closed.append('coordinator'))
    terrain=S(_terrain=S(close=lambda:closed.append('terrain')))
    monkeypatch.setattr(module,'P3MapStore',lambda **kwargs:terrain)
    monkeypatch.setattr(module,'NativeGeometry',lambda *args:None)
    monkeypatch.setattr(module,'NativeDistanceField',lambda *args:None)
    monkeypatch.setattr(module,'AsyncNavigationCoordinator',lambda **kwargs:c)
    settings=dict(build_dir='',allow_unknown=False,smooth_global_reference=True,planning_period_s=.1,execution_period_s=.05,
        global_budget_s=1.,clearance_radius_m=.9,horizon_steps=20,horizon_dt_s=.2,
        goal_tolerance_m=.1,goal_yaw_tolerance_rad=.05,stopped_linear_mps=.01,stopped_angular_rps=.01,goal_position_target_m=target)
    if isinstance(target,(int,float)) and target>0. and math.isfinite(target):
        actual,_=module.create_resources(c.limits,None,settings)
        assert actual.coordinator.position_target_tolerance_m==.02 and not closed
    else:
        with pytest.raises(ValueError,match='goal_position_target_m'):
            module.create_resources(c.limits,None,settings)
        assert closed==['coordinator','terrain']
