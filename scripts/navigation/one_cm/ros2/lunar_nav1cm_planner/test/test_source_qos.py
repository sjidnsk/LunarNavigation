"""共享地图输入可选QoS；真实ROS订阅信息验证默认兼容。"""
import pytest

@pytest.mark.parametrize("configured", [False, True])
def test_source_qos_is_configurable_without_changing_legacy_default(configured):
    rclpy=pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from rclpy.qos import ReliabilityPolicy,DurabilityPolicy
    from lunar_nav1cm_planner.node import PlannerNode
    from test_runtime import build_runtime
    rclpy.init()
    def factory(*args):
        _,session,_,_=build_runtime()
        return session,None
    params=[Parameter("build_dir",value="/tmp/task3-unused")]
    if configured:
        params += [Parameter("map_qos_reliability",value="reliable"),Parameter("map_qos_durability",value="transient_local")]
    node=PlannerNode(resource_factory=factory,parameter_overrides=params)
    try:
        subscription=next(s for s in node.subscriptions if s.topic_name=="/Car/T3/mapping/grid_map")
        assert subscription.qos_profile.reliability==(ReliabilityPolicy.RELIABLE if configured else ReliabilityPolicy.BEST_EFFORT)
        assert subscription.qos_profile.durability==(DurabilityPolicy.TRANSIENT_LOCAL if configured else DurabilityPolicy.VOLATILE)
    finally:
        node.stop_runtime();node.destroy_node();rclpy.shutdown()


def test_static_tf_late_join_keeps_earlier_retained_edges():
    import time
    rclpy=pytest.importorskip('rclpy')
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from lunar_nav1cm_planner.node import PlannerNode
    from test_runtime import build_runtime
    rclpy.init();peer=Node('retained_tf_peer');node=None
    executor=SingleThreadedExecutor();executor.add_node(peer)
    qos=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher=peer.create_publisher(TFMessage,'/p3_late_tf_test',qos)
    for index in range(12):
        transform=TransformStamped();transform.header.frame_id='map' if index==0 else 'base_link'
        transform.child_frame_id='odom' if index==0 else f'camera_{index}'
        transform.transform.rotation.w=1.
        publisher.publish(TFMessage(transforms=[transform]))
    def factory(*args):
        _,session,_,_=build_runtime();return session,None
    try:
        node=PlannerNode(resource_factory=factory,parameter_overrides=[
            Parameter('build_dir',value='/unused'),Parameter('tf_static_topic',value='/p3_late_tf_test')])
        executor.add_node(node)
        end=time.monotonic()+2.
        while time.monotonic()<end:executor.spin_once(timeout_sec=.02)
        assert node.runtime._transforms.lookup('map','odom').x==0.
        assert len(node.runtime._transforms._edges)>=12
    finally:
        executor.shutdown()
        if node is not None:node.stop_runtime();node.destroy_node()
        peer.destroy_node();rclpy.shutdown()
