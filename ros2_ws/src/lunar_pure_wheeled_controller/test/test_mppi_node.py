"""Humble ROS boundaries for the optional map-aware MPPI backend."""
import pytest
import rclpy
from lunar_pure_wheeled_controller.node import PureWheeledControllerNode
from lunar_pure_wheeled_controller.mppi import MppiExecutor


def test_mppi_backend_is_explicit_and_requires_revisioned_references():
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi','-p','input_mode:=incremental_reference'])
    node=None
    try:
        node=PureWheeledControllerNode()
        assert isinstance(node._executor,MppiExecutor)
        assert node._executor.map is None
        assert node._map_client is not None
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_mppi_cannot_silently_run_without_a_map_contract():
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi'])
    try:
        with pytest.raises(ValueError,match='incremental_reference'):PureWheeledControllerNode()
    finally:rclpy.shutdown()


@pytest.mark.parametrize("stale,clock_mode",[(False,"real"),(True,"real"),("during","real"),("unsafe_after","real"),(False,"paused"),(False,"recert_slow")])
@pytest.mark.parametrize("map_age_s",[0.,2.])
def test_performance_overrun_does_not_stop_valid_state_or_clear_prediction(monkeypatch,stale,map_age_s,clock_mode):
    import time
    from types import SimpleNamespace as S
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped,TransformStamped
    from tf2_msgs.msg import TFMessage
    from lunar_planning_msgs.msg import PathReference
    from lunar_pure_wheeled_controller.execution import ExecutionResult
    from lunar_pure_wheeled_controller.tracking import TrackingCommand
    from lunar_pure_wheeled_controller.mppi_map import GridMap
    import numpy as np
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi','-p','input_mode:=incremental_reference'])
    node=None
    try:
        node=PureWheeledControllerNode()
        ref=PathReference();ref.session_id.uuid=[11]*16;ref.segment_revision=1;ref.traversability_revision=1;ref.path.header.frame_id='map'
        for x in (0.,1.):
            p=PoseStamped();p.pose.position.x=x;p.pose.orientation.w=1.;ref.path.poses.append(p)
        node._on_path_reference(ref)
        odom=Odometry();odom.header.stamp=node.get_clock().now().to_msg()
        if stale is True:odom.header.stamp.sec-=1
        odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.pose.pose.orientation.w=1.;node._on_odometry(odom)
        t=TransformStamped();t.header.stamp=odom.header.stamp;t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.;node._on_tf(TFMessage(transforms=[t]))
        node._map_cache=S(revision=1,local_grid=lambda *a:GridMap(np.ones((40,40),np.uint8),.1,(-2.,-2.)))
        node._map_received_at=time.monotonic();node._map_processed_ns=node.get_clock().now().nanoseconds
        node._map_processed_ns-=int(map_age_s*1e9)
        clock_ns=[node.get_clock().now().nanoseconds]
        if clock_mode!="real":
            from rclpy.time import Time
            monkeypatch.setattr(node,'get_clock',lambda:S(now=lambda:Time(nanoseconds=clock_ns[0])))
        called=[]
        def slow(*a,**kw):
            called.append(True)
            time.sleep(.06)
            if clock_mode=="recert_slow":clock_ns[0]+=60_000_000
            if stale=='during':node._odometry_received_at=time.monotonic()-1.
            return ExecutionResult(TrackingCommand(.2,0.,False,None),'TRACKING',0.,0.,0.,1)
        monkeypatch.setattr(node._executor,'update',slow)
        recertified=[];certify=node._executor.certify_publication
        def recertify(state,v,w,age):
            recertified.append(age)
            safe=False if stale=="unsafe_after" else certify(state,v,w,age)
            if clock_mode=="recert_slow" and len(recertified)==1:clock_ns[0]+=60_000_000
            return safe
        monkeypatch.setattr(node._executor,'certify_publication',recertify)
        discarded=[];monkeypatch.setattr(node._executor,'discard_prediction',lambda:discarded.append(True))
        diagnostics=[];node._mppi_diagnostics=S(publish=lambda m:diagnostics.append(m.data))
        emitted=[];monkeypatch.setattr(node,'_publish_twist',lambda v=0.,w=0.:emitted.append((v,w)))
        node._tick_incremental_path()
        assert emitted==([(0.,0.)] if stale else [(.2,0.)])
        assert bool(called)==(stale is not True)
        assert bool(discarded)==(stale in ("during","unsafe_after"))
        if clock_mode=="paused":assert not recertified
        elif stale in (False,"unsafe_after"):
            assert recertified and recertified[0]>.05
        if clock_mode=="recert_slow":assert len(recertified)==2
        if not stale:
            import json
            assert json.loads(diagnostics[-1])['deadline_missed'] is True
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('unpaired,receipt_delay,source_age_s,expected',[(True,0.,.002,.1),(False,.051,.002,.2),(True,0.,.060,None),(False,.501,.002,None)])
def test_unpaired_latest_odometry_can_use_a_fresh_coherent_pair(monkeypatch,unpaired,receipt_delay,source_age_s,expected):
    """A newer odometry callback must not hide a still-valid matched pair."""
    import time
    from types import SimpleNamespace as S
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped,TransformStamped
    from tf2_msgs.msg import TFMessage
    from lunar_planning_msgs.msg import PathReference
    from lunar_pure_wheeled_controller.execution import ExecutionResult
    from lunar_pure_wheeled_controller.tracking import TrackingCommand
    from lunar_pure_wheeled_controller.mppi_map import GridMap
    import numpy as np
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi','-p','input_mode:=incremental_reference'])
    node=None
    try:
        node=PureWheeledControllerNode()
        ref=PathReference();ref.session_id.uuid=[15]*16;ref.segment_revision=1;ref.traversability_revision=1;ref.path.header.frame_id='map'
        for x in (0.,1.):
            p=PoseStamped();p.pose.position.x=x;p.pose.orientation.w=1.;ref.path.poses.append(p)
        node._on_path_reference(ref)
        now=node.get_clock().now().nanoseconds
        def odometry(stamp_ns,x):
            m=Odometry();m.header.stamp.sec=stamp_ns//1_000_000_000;m.header.stamp.nanosec=stamp_ns%1_000_000_000
            m.header.frame_id='odom';m.child_frame_id='base_link';m.pose.pose.orientation.w=1.;m.pose.pose.position.x=x
            return m
        import math
        matched=odometry(now-int(source_age_s*1e9),.1);matched.twist.twist.linear.x=.03;matched.twist.twist.linear.y=.005;matched.twist.twist.angular.z=.04;node._on_odometry(matched)
        t=TransformStamped();t.header.stamp=matched.header.stamp;t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.
        t.transform.translation.x=.3;t.transform.translation.y=.2;t.transform.rotation.z=math.sin(math.pi/4);t.transform.rotation.w=math.cos(math.pi/4)
        node._on_tf(TFMessage(transforms=[t]));latest=odometry(now-int(source_age_s*1e9)+1_000_000,.2);latest.twist.twist.linear.x=.07;latest.twist.twist.linear.y=-.005;latest.twist.twist.angular.z=-.08;node._on_odometry(latest)
        if not unpaired:
            t.header.stamp=latest.header.stamp;t.transform.translation.x=.7;t.transform.translation.y=-.2;t.transform.rotation.z=0.;t.transform.rotation.w=1.;node._on_tf(TFMessage(transforms=[t]))
            from lunar_pure_wheeled_controller import node as node_module
            received_now=time.monotonic()
            monkeypatch.setattr(node_module,"monotonic",lambda:received_now+receipt_delay)
        node._map_cache=S(revision=1,local_grid=lambda *a:GridMap(np.ones((40,40),np.uint8),.1,(-2.,-2.)))
        node._map_received_at=time.monotonic();node._map_processed_ns=node.get_clock().now().nanoseconds
        seen=[]
        def solve(state,*args,**kwargs):
            seen.append((state.x_m,state.y_m,state.yaw_rad,state.linear_mps,state.angular_radps,state.lateral_mps))
            return ExecutionResult(TrackingCommand(0.,0.,False,None),'WAITING',0.,0.,0.,1)
        monkeypatch.setattr(node._executor,'update',solve)
        node._tick_incremental_path()
        if expected is None:
            assert seen==[]
        else:
            assert len(seen)==1
            target=(.3,.3,math.pi/2,.03,.04,.005) if unpaired else (.9,-.2,0.,.07,-.08,-.005)
            assert seen[0]==pytest.approx(target)
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_invalid_twist_history_cannot_poison_same_stamp_recovery(monkeypatch):
    import copy,time
    from types import SimpleNamespace as S
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped,TransformStamped
    from tf2_msgs.msg import TFMessage
    from lunar_planning_msgs.msg import PathReference
    from lunar_pure_wheeled_controller.execution import ExecutionResult
    from lunar_pure_wheeled_controller.tracking import TrackingCommand
    from lunar_pure_wheeled_controller.mppi_map import GridMap
    import numpy as np
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi','-p','input_mode:=incremental_reference'])
    node=None
    try:
        node=PureWheeledControllerNode()
        ref=PathReference();ref.session_id.uuid=[16]*16;ref.segment_revision=1;ref.traversability_revision=1;ref.path.header.frame_id='map'
        for x in (0.,1.):
            p=PoseStamped();p.pose.position.x=x;p.pose.orientation.w=1.;ref.path.poses.append(p)
        node._on_path_reference(ref)
        good=Odometry();good.header.stamp=node.get_clock().now().to_msg();good.header.frame_id='odom';good.child_frame_id='base_link';good.pose.pose.orientation.w=1.;good.twist.twist.linear.x=.1
        bad=copy.deepcopy(good);bad.twist.twist.linear.x=float('nan');node._on_odometry(bad)
        t=TransformStamped();t.header.stamp=good.header.stamp;t.header.frame_id='map';t.child_frame_id='odom';t.transform.rotation.w=1.;node._on_tf(TFMessage(transforms=[t]))
        node._map_cache=S(revision=1,local_grid=lambda *a:GridMap(np.ones((40,40),np.uint8),.1,(-2.,-2.)))
        node._map_received_at=time.monotonic();node._map_processed_ns=node.get_clock().now().nanoseconds
        seen=[]
        def solve(state,*a,**kw):
            seen.append(state.linear_mps)
            return ExecutionResult(TrackingCommand(0.,0.,False,None),'WAITING',0.,0.,0.,1)
        monkeypatch.setattr(node._executor,'update',solve)
        node._tick_incremental_path();assert seen==[]
        node._on_odometry(good);node._tick_incremental_path()
        assert seen==[.1]
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('moving',[False,True])
def test_simulation_step_ack_follows_the_actual_published_command(monkeypatch,moving):
    import json
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi','-p','input_mode:=incremental_reference',
        '-p','use_sim_time:=true','-p','mppi_simulation_step_sync:=true'])
    node=None
    try:
        node=PureWheeledControllerNode();events=[]
        class Capture:
            def publish(self,message):events.append(json.loads(message.data))
        node._mppi_step_ack=Capture()
        if moving:monkeypatch.setattr(node,'_tick',lambda:node._publish_twist(.1,-.02))
        node._control_timer_tick()
        assert events[-1]['event']=='control_step'
        assert events[-1]['now_ns']==node.get_clock().now().nanoseconds
        assert events[-1]['command']==([.1,-.02] if moving else [0.,0.])
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_simulation_step_sync_requires_a_simulation_clock():
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi','-p','input_mode:=incremental_reference',
        '-p','mppi_simulation_step_sync:=true'])
    try:
        with pytest.raises(ValueError,match='use_sim_time'):PureWheeledControllerNode()
    finally:rclpy.shutdown()


def test_simulation_step_exception_cannot_acknowledge_a_previous_command(monkeypatch):
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi','-p','input_mode:=incremental_reference',
        '-p','use_sim_time:=true','-p','mppi_simulation_step_sync:=true'])
    node=None
    try:
        node=PureWheeledControllerNode();events=[]
        class Capture:
            def publish(self,message):events.append(message)
        node._mppi_step_ack=Capture();node._publish_twist(.1,0.)
        def fail():raise RuntimeError('tick failed')
        monkeypatch.setattr(node,'_tick',fail)
        with pytest.raises(RuntimeError,match='tick failed'):node._control_timer_tick()
        assert not events
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_simulation_step_startup_ack_survives_late_discovery_without_clock_advance():
    import json,time
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi','-p','input_mode:=incremental_reference',
        '-p','use_sim_time:=true','-p','mppi_simulation_step_sync:=true'])
    node=observer=None
    try:
        node=PureWheeledControllerNode();observer=rclpy.create_node('late_step_ack_observer');events=[]
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        observer.create_subscription(String,node.get_parameter('mppi_step_ack_topic').value,lambda m:events.append(json.loads(m.data)),qos)
        deadline=time.monotonic()+3.
        while not events and time.monotonic()<deadline:rclpy.spin_once(observer,timeout_sec=.05)
        assert events and events[0]['now_ns']==0 and events[0]['command']==[0.,0.]
    finally:
        if observer is not None:observer.destroy_node()
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_advisory_compute_budget_is_not_limited_by_model_period():
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi','-p','input_mode:=incremental_reference','-p','mppi_compute_budget_s:=0.2'])
    node=None
    try:
        node=PureWheeledControllerNode()
        assert node._mppi_budget_s==.2
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_native_map_request_uses_revision_not_a_new_sensor_deadline():
    from types import SimpleNamespace as S
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi','-p','input_mode:=incremental_reference'])
    node=None
    try:
        node=PureWheeledControllerNode();requests=[]
        future=S(add_done_callback=lambda cb:None)
        node._map_client=S(service_is_ready=lambda:True,call_async=lambda request:(requests.append(request) or future))
        node._map_cache.revision=7
        node._request_policy_map()
        assert requests[0].since_revision==7
        assert requests[0].minimum_map_stamp_ns==0
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('future_stamp',[False,True])
def test_native_ready_map_is_valid_without_inventing_observation_age_gate(future_stamp):
    from types import SimpleNamespace as S
    from lunar_planning_msgs.srv import GetPolicyMap
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi','-p','input_mode:=incremental_reference'])
    node=None
    try:
        node=PureWheeledControllerNode();response=GetPolicyMap.Response()
        response.ready=True;response.frame_id='map';response.epoch='test';response.fine_revision=1;response.full_snapshot=True;response.resolution_m=.2
        stamp=node.get_clock().now().nanoseconds+int((2. if future_stamp else -2.)*1e9)
        response.processed_stamp.sec=stamp//1_000_000_000;response.processed_stamp.nanosec=stamp%1_000_000_000
        f=S(result=lambda:response);node._map_future=f;node._on_policy_map(f)
        assert (node._map_received_at is not None)==(not future_stamp)
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_path_follower_backend_does_not_create_map_client():
    from lunar_pure_wheeled_controller.mppi_path_follower import MppiPathFollower
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi_path_follower','-p','input_mode:=incremental_reference'])
    node=None
    try:
        node=PureWheeledControllerNode()
        assert isinstance(node._executor,MppiPathFollower)
        assert node._map_client is None
        assert not node._executor.requires_map
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()
