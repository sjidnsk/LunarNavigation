"""真实ROS消息的标准Path/执行元信息接线；不驱动车辆。"""
from types import SimpleNamespace as S
import pytest
import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from lunar_pure_wheeled_controller.node import PureWheeledControllerNode
from lunar_pure_wheeled_controller.tracking import TrackingState
from lunar_pure_wheeled_controller.mppi_map import GridMap
from nav1cm.execution_contract import ExecutionReference,path_fingerprint,FOLLOW,BRAKE
from nav1cm.execution_ros import reference_message,stamp_ns,status_from_message
import numpy as np


@pytest.mark.parametrize('order',['path','metadata'])
def test_pairing_requires_matching_geometry_and_reports_actual_acceptance(order):
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi_path_follower','-p','input_mode:=incremental_path','-p','execution_contract_enabled:=true'])
    node=None
    try:
        node=PureWheeledControllerNode();path=Path();path.header.frame_id='map'
        path.header.stamp=node.get_clock().now().to_msg()
        for x in (0.,1.):
            p=PoseStamped();p.header=path.header;p.pose.position.x=x;p.pose.orientation.w=1.;path.poses.append(p)
        ref=ExecutionReference('a'*32,1,path_fingerprint(path),FOLLOW,stamp_ns(path.header))
        meta=reference_message(ref,'map')
        if order=='path':node._on_path(path)
        else:node._on_execution_reference(meta)
        assert node._active is None
        assert node._executor.execution_reference is None
        if order=='path':node._on_execution_reference(meta)
        else:node._on_path(path)
        assert node._execution_offered==ref
        assert node._executor.execution_reference is None
        node._executor.set_map(GridMap(np.ones((100,100),np.uint8),.1,(-5.,-5.)))
        node._standard_state=TrackingState(0.,0.,0.)
        node._executor.update(node._standard_state,.05,now_s=1.)
        assert node._executor.execution_reference==ref
        assert node._executor.reference_accepted
        sent=[];node._tracking_status=S(publish=sent.append)
        node._publish_execution_status()
        status=status_from_message(sent[-1])
        assert status.accepted and status.session_id==ref.session_id
        assert status.path_digest==ref.path_digest and status.revision==1
        # 配对消息即使收到，错摘要也不能接管新的几何。
        wrong=ExecutionReference('a'*32,2,'0'*64,BRAKE,ref.stamp_ns+1)
        node._on_execution_reference(reference_message(wrong,'map'))
        assert node._executor.execution_reference==ref
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
