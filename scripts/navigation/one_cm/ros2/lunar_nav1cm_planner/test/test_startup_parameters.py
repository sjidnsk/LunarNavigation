"""真实rclpy参数服务；隔离worker，不以本测试证明规划/车辆闭环。"""
import time
import pytest


@pytest.fixture
def configured_node():
    rclpy=pytest.importorskip('rclpy')
    from rclpy.parameter import Parameter
    from lunar_nav1cm_planner.task_node import NavigationTaskNode
    from test_runtime import build_runtime
    observed={}
    def factory(limits,capability,settings):
        observed.update(limits=limits,settings=dict(settings))
        _,session,_,_=build_runtime()
        return session,None
    rclpy.init();node=None
    try:
        prefix='/startup_parameters_test'
        overrides={'build_dir':'/unused-startup-parameter-test','linear_speed_mps':.07,
            'planning_period_s':.27,'map_topic':prefix+'/map','goal_topic':prefix+'/goal',
            'odometry_topic':prefix+'/odom','tf_topic':prefix+'/tf','tf_static_topic':prefix+'/tf_static',
            'local_path_topic':prefix+'/local','global_route_topic':prefix+'/global',
            'action_name':prefix+'/navigate','minimum_clearance_m':.01,'clearance_radius_m':.409}
        node=NavigationTaskNode(resource_factory=factory,parameter_overrides=[
            Parameter(name,value=value) for name,value in overrides.items()])
        yield node,observed
    finally:
        if node is not None:
            node.task_api.close();node.stop_runtime();node.destroy_node()
        rclpy.shutdown()


def test_startup_overrides_reach_resources_and_endpoints(configured_node):
    node,observed=configured_node
    assert observed['limits'].linear_speed_mps==.07
    assert observed['settings']['planning_period_s']==.27
    assert '/startup_parameters_test/map' in [s.topic_name for s in node.subscriptions]
    assert node.get_parameter('action_name').value=='/startup_parameters_test/navigate'


def test_effective_clearance_is_published_before_freezing(configured_node):
    node,observed=configured_node
    assert node.get_parameter('minimum_clearance_m').value==observed['limits'].minimum_clearance_m
    assert observed['limits'].minimum_clearance_m>.01
    assert node.get_parameter('clearance_radius_m').value==observed['settings']['clearance_radius_m']
    assert observed['settings']['clearance_radius_m']>.409


def test_all_application_parameters_reject_runtime_writes(configured_node):
    node,_=configured_node
    from rclpy.parameter import Parameter
    names=[name for name in node.get_parameters_by_prefix('') if name!='use_sim_time']
    assert len(names)>30
    for name in names:
        descriptor=node.describe_parameter(name)
        assert descriptor.read_only,name
        assert 'restart' in descriptor.description.lower(),name
        original=node.get_parameter(name).value
        assert not node.set_parameters([Parameter(name,value=original)])[0].successful,name
        assert node.get_parameter(name).value==original
    assert not node.describe_parameter('use_sim_time').read_only
    for value in (True,False):
        assert node.set_parameters([Parameter('use_sim_time',value=value)])[0].successful
        assert node.get_parameter('use_sim_time').value is value


def test_atomic_mixed_update_does_not_change_any_parameter(configured_node):
    node,observed=configured_node
    from rclpy.parameter import Parameter
    result=node.set_parameters_atomically([Parameter('use_sim_time',value=True),
        Parameter('linear_speed_mps',value=.04)])
    assert not result.successful
    assert node.get_parameter('use_sim_time').value is False
    assert node.get_parameter('linear_speed_mps').value==observed['limits'].linear_speed_mps==.07


def test_parameter_service_rejects_instead_of_reporting_false_success(configured_node):
    node,observed=configured_node
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rcl_interfaces.srv import SetParameters
    from rclpy.executors import SingleThreadedExecutor
    peer=Node('startup_parameter_client');executor=SingleThreadedExecutor()
    executor.add_node(node);executor.add_node(peer)
    try:
        client=peer.create_client(SetParameters,node.get_fully_qualified_name()+'/set_parameters')
        assert client.wait_for_service(timeout_sec=3.)
        request=SetParameters.Request(parameters=[Parameter('linear_speed_mps',value=.04).to_parameter_msg()])
        future=client.call_async(request)
        deadline=time.monotonic()+3.
        while not future.done() and time.monotonic()<deadline:executor.spin_once(timeout_sec=.02)
        assert future.done()
        result=future.result().results[0]
        assert not result.successful
        assert 'read-only' in result.reason.lower()
        assert node.get_parameter('linear_speed_mps').value==observed['limits'].linear_speed_mps==.07
    finally:
        executor.shutdown();peer.destroy_node()
