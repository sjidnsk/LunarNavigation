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
