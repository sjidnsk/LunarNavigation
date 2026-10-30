"""真实ROS验证raw/global reference/local三条消息独立且清理不残留。"""
import time
import pytest


def test_actual_reference_topic_is_distinct_and_carries_the_mpcc_geometry():
    rclpy=pytest.importorskip('rclpy')
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from nav_msgs.msg import Path
    from lunar_nav1cm_planner.node import PlannerNode
    from nav1cm.contracts import Route
    from nav1cm.ros_conversion import paths
    from test_runtime import build_runtime
    rclpy.init();captured={}
    def factory(limits,capability,settings):
        captured.update(settings);_,session,_,_=build_runtime();return session,None
    node=PlannerNode(resource_factory=factory,parameter_overrides=[
        Parameter('build_dir',value='/unused'),Parameter('global_route_topic',value='/smoothing_ros/raw'),
        Parameter('local_path_topic',value='/smoothing_ros/local')])
    peer=rclpy.create_node('smoothing_reference_observer');executor=SingleThreadedExecutor()
    executor.add_node(node);executor.add_node(peer);received={}
    topics=['/smoothing_ros/raw','/smoothing_ros/raw_reference','/smoothing_ros/local']
    subs=[peer.create_subscription(Path,t,lambda msg,t=t:received.__setitem__(t,msg),10) for t in topics]
    try:
        node.stop_runtime()
        assert node.get_parameter('smooth_global_reference').value is True
        assert node.describe_parameter('smooth_global_reference').read_only
        assert captured['smooth_global_reference'] is True
        assert node.get_parameter('global_reference_topic').value==topics[1]
        raw=Route(1,1,[[0,0],[2,0],[2,2]],(2,2))
        actual=Route(1,1,[[0,0],[1.8,0],[1.95,.05],[2,.2],[2,2]],(2,2))
        packet=paths(raw,None,peer.get_clock().now().nanoseconds,global_reference=actual)
        end=time.monotonic()+3.
        while len(received)<3 and time.monotonic()<end:
            node._publish(packet);executor.spin_once(timeout_sec=.02)
        assert len(received)==3
        assert len(received[topics[0]].poses)==3
        assert [(p.pose.position.x,p.pose.position.y) for p in received[topics[1]].poses]==[tuple(x) for x in actual.points]
        assert not received[topics[2]].poses
        received.clear();empty=paths(None,None,peer.get_clock().now().nanoseconds)
        end=time.monotonic()+2.
        while time.monotonic()<end:
            node._publish(empty);executor.spin_once(timeout_sec=.01)
            if len(received)==3 and all(not m.poses for m in received.values()):break
        assert len(received)==3 and all(not m.poses for m in received.values())
    finally:
        executor.shutdown();node.stop_runtime();node.destroy_node();peer.destroy_node();rclpy.shutdown()


def test_resolved_reference_topic_must_not_alias_raw_route():
    rclpy=pytest.importorskip('rclpy')
    from rclpy.parameter import Parameter
    from lunar_nav1cm_planner.node import PlannerNode
    rclpy.init()
    try:
        with pytest.raises(ValueError,match='distinct'):
            PlannerNode(resource_factory=lambda *args:pytest.fail('alias must be rejected before allocating workers'),
                parameter_overrides=[Parameter('build_dir',value='/unused'),
                    Parameter('global_route_topic',value='/smoothing_ros/raw'),
                    Parameter('global_reference_topic',value='smoothing_ros/raw')])
    finally:rclpy.shutdown()
