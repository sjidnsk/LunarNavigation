"""真实DDS+Action的跟踪故障注入；工作线程隔离，不驱动车辆。"""
import math,time
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).parent))
from test_runtime import build_runtime


def test_matched_tracking_failure_aborts_action_only_after_independent_stop():
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from nav_msgs.msg import Odometry
    from lunar_planning_msgs.msg import PathExecutionReference,TrackingStatus
    from lunar_planning_msgs.action import NavigateToPose
    from lunar_nav1cm_planner.task_node import NavigationTaskNode
    rclpy.init();executor=SingleThreadedExecutor();node=client_node=None
    prefix='/nav1cm_execution_failure_test';measured=[.02];active=[None];latest=[None]
    def factory(*args):return build_runtime()[1],None
    try:
        node=NavigationTaskNode(resource_factory=factory,parameter_overrides=[
            Parameter('build_dir',value='/tmp/nav1cm-unused-execution-test'),
            Parameter('action_name',value=prefix+'/navigate'),Parameter('odometry_topic',value=prefix+'/odom'),
            Parameter('local_path_topic',value=prefix+'/path'),Parameter('global_route_topic',value=prefix+'/route'),
            Parameter('execution_contract_enabled',value=True),
            Parameter('execution_reference_topic',value=prefix+'/execution'),Parameter('tracking_status_topic',value=prefix+'/tracking')])
        client_node=Node('nav1cm_execution_failure_client');executor.add_node(node);executor.add_node(client_node)
        odom=client_node.create_publisher(Odometry,prefix+'/odom',10)
        status=client_node.create_publisher(TrackingStatus,prefix+'/tracking',10)
        def metadata(message):latest[0]=message
        sub=client_node.create_subscription(PathExecutionReference,prefix+'/execution',metadata,10)
        def tick():
            m=Odometry();m.header.frame_id='map';m.child_frame_id='base_link'
            m.header.stamp=client_node.get_clock().now().to_msg();m.pose.pose.orientation.w=1.;m.twist.twist.linear.x=measured[0];odom.publish(m)
            ref=latest[0]
            if active[0] is not None and ref is not None and bytes(ref.session_id.uuid)==active[0]:
                s=TrackingStatus();s.header.frame_id='map';s.header.stamp=client_node.get_clock().now().to_msg()
                s.session_id=ref.session_id;s.segment_revision=ref.reference_revision;s.reference_digest=ref.path_digest
                s.reference_intent=ref.intent;s.reference_accepted=True;s.state=TrackingStatus.FAILED;s.reason='TRACKING_STALLED'
                s.linear_speed_mps=measured[0];status.publish(s)
        timer=client_node.create_timer(.03,tick)
        def until(predicate,timeout=5.):
            end=time.monotonic()+timeout
            while not predicate() and time.monotonic()<end:executor.spin_once(timeout_sec=.02)
            assert predicate()
        client=ActionClient(client_node,NavigateToPose,prefix+'/navigate')
        assert client.wait_for_server(timeout_sec=3.)
        sent=client.send_goal_async(NavigateToPose.Goal(target_x_m=0.,target_y_m=3.,has_target_yaw=True,target_yaw_rad=0.))
        until(sent.done);handle=sent.result();assert handle.accepted
        active[0]=bytes(handle.goal_id.uuid);result=handle.get_result_async()
        until(lambda:node.runtime.tasks.active is not None and node.runtime.tasks.active.phase=='ERROR_STOPPING')
        assert not result.done()
        measured[0]=0.
        until(result.done)
        assert result.result().status==6
        assert result.result().result.outcome==NavigateToPose.Result.INTERNAL_ERROR
        assert result.result().result.reason_code=='TRACKING_STALLED'
    finally:
        if node is not None:
            node.stop_runtime();executor.remove_node(node);node.destroy_node()
        if client_node is not None:executor.remove_node(client_node);client_node.destroy_node()
        executor.shutdown();rclpy.shutdown()
