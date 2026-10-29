"""真实ROS节点从同一平台配置解析运动模型。"""
import pytest
import rclpy
from lunar_pure_wheeled_controller.node import PureWheeledControllerNode


@pytest.mark.parametrize('override,expected',[(None,'differential_drive'),('curvature_limited','curvature_limited')])
def test_standard_path_node_loads_model_and_explicit_override(override,expected):
    args=['--ros-args','-p','controller_backend:=mppi_path_follower','-p','input_mode:=incremental_path']
    if override is not None:args.extend(['-p','motion_model:='+override])
    rclpy.init(args=args);node=None
    try:
        node=PureWheeledControllerNode()
        assert node._policy.motion_model==expected
        assert node._executor.policy.motion_model==expected
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
