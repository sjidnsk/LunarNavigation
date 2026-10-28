"""标准入口不沿用旧 coherent-state 门槛。"""
import numpy as np
import pytest
import rclpy
from lunar_pure_wheeled_controller.node import PureWheeledControllerNode
from lunar_pure_wheeled_controller.mppi_map import GridMap


def test_unknown_eligibility_is_independent_of_measured_free():
    grid=GridMap(np.zeros((10,10),np.uint8),.01,(0.,0.),eligible=np.ones((10,10),bool))
    assert grid.sample([[.055,.055]])[0][0]==0
    assert grid.valid_segments([[.04,.04]],[[.06,.06]],0.).all()
    assert not grid.assumed.any()


def test_standard_defaults_use_direct_p3_and_wheel_capability():
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi_path_follower','-p','input_mode:=incremental_path'])
    node=None
    try:
        node=PureWheeledControllerNode()
        assert node._map_client is None
        assert node._p3_source is not None
        assert node._executor.config.state_age_budget_s==.5
        assert node._body_dimensions==pytest.approx((1.182,.818))
        import yaml
        from pathlib import Path
        from ament_index_python.packages import get_package_share_directory
        configured=yaml.safe_load((Path(get_package_share_directory('lunar_nav1cm_planner'))/'config/wheel.yaml').read_text())['capability']['minimum_clearance_m']
        assert node.get_parameter('minimum_clearance_m').value==configured
        assert node._p3_source._minimum_clearance==configured
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def transform(parent,child,x=0.,y=0.):
    from geometry_msgs.msg import TransformStamped
    t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
    t.transform.rotation.w=1.;t.transform.translation.x=x;t.transform.translation.y=y
    return t


def elevation(resolution=.05,size=160,frame='map'):
    from grid_map_msgs.msg import GridMap as Message
    from std_msgs.msg import Float32MultiArray
    message=Message();message.header.frame_id=frame
    message.info.resolution=resolution
    message.info.length_x=message.info.length_y=size*resolution
    message.info.pose.orientation.w=1.
    message.layers=['elevation'];message.data=[Float32MultiArray(data=[0.]*(size*size))]
    return message


def test_worker_pending_tf_sameepoch_and_malformed_liveness():
    import time
    from nav1cm.ros_conversion import LatestTransforms
    from tf2_msgs.msg import TFMessage
    from lunar_pure_wheeled_controller.p3_map_source import create_p3_source
    import yaml
    from pathlib import Path
    from ament_index_python.packages import get_package_share_directory
    capability=yaml.safe_load((Path(get_package_share_directory('lunar_nav1cm_planner'))/'config/wheel.yaml').read_text())['capability']
    source=create_p3_source(capability,(1.182,.818))
    try:
        source.set_center(0.,0.)
        message=elevation(frame='sensor')
        source.submit_map(message,1.);assert source.wait_idle()
        assert source.snapshot() is None
        tf=LatestTransforms();tf.update(TFMessage(transforms=[transform('map','odom'),transform('odom','sensor')]))
        source.set_transforms(tf);assert source.wait_idle()
        first=source.snapshot();assert first.received_at==1.
        assert first.grid.states[250,250]==1 and first.grid.stationary_anchor is None
        source.submit_map(message,2.);assert source.wait_idle()
        second=source.snapshot()
        assert second.received_at==2. and second.grid.revision==first.grid.revision
        bad=elevation();bad.info.resolution=0.
        source.submit_map(bad,3.);assert source.wait_idle()
        assert source.snapshot().received_at==2.
        assert source.error
        # 各种正源分辨率、旋转、滚动原点不重置世界历史。
        for index,resolution in enumerate((.05,.02,.01,.005)):
            message=elevation(resolution=resolution,size=20)
            message.info.pose.position.x=.1*index
            message.info.pose.orientation.z=np.sin(.1*index)
            message.info.pose.orientation.w=np.cos(.1*index)
            source.submit_map(message,4.+index);assert source.wait_idle()
            assert source._store.source_resolution_m==pytest.approx(resolution)
            assert source.snapshot().received_at==4.+index
            assert source.snapshot().grid.states.shape==(500,500)
    finally:source.close()
    assert not source._worker.is_alive()


def test_latest_zero_stamp_chain_and_missing_feedback_diagnostic(monkeypatch):
    import json
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    from types import SimpleNamespace as S
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi_path_follower','-p','input_mode:=incremental_path'])
    node=None
    try:
        node=PureWheeledControllerNode()
        diagnostics=[];node._mppi_diagnostics=S(publish=lambda msg:diagnostics.append(json.loads(msg.data)))
        node._tick()
        assert diagnostics[-1]['measured_stopped'] is None
        assert not diagnostics[-1]['certified_stop']
        odom=Odometry();odom.header.frame_id='arbitrary_odom';odom.child_frame_id='wheel_body'
        odom.pose.pose.orientation.w=1.;odom.twist.twist.linear.x=-.12
        node._on_odometry(odom)
        assert node._odometry is None
        node._on_tf(TFMessage(transforms=[transform('map','middle',1.),transform('middle','arbitrary_odom',2.)]))
        assert node._standard_state.x_m==3. and node._standard_state.linear_mps==-.12
        assert node._standard_state_ns>0
        node._on_tf(TFMessage(transforms=[transform('map','middle',4.)]))
        assert node._standard_state.x_m==6.
        assert not node._mppi_tf_history
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_map_replaced_during_solve_rejects_stale_command(monkeypatch):
    import time
    from types import SimpleNamespace as S
    from nav_msgs.msg import Odometry,Path
    from geometry_msgs.msg import PoseStamped
    from lunar_pure_wheeled_controller.execution import ExecutionResult
    from lunar_pure_wheeled_controller.tracking import TrackingCommand
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi_path_follower','-p','input_mode:=incremental_path'])
    node=None
    try:
        node=PureWheeledControllerNode()
        path=Path();path.header.frame_id='map'
        for x in (0.,1.):
            p=PoseStamped();p.pose.position.x=x;p.pose.orientation.w=1.;path.poses.append(p)
        node._on_path(path)
        odom=Odometry();odom.header.frame_id='map';odom.pose.pose.orientation.w=1.;node._on_odometry(odom)
        free=GridMap(np.ones((500,500),np.uint8),.01,(-2.5,-2.5),revision=1)
        blocked=GridMap(np.full((500,500),2,np.uint8),.01,(-2.5,-2.5),revision=2)
        current=[free]
        monkeypatch.setattr(node,'_standard_grid',lambda state:current[0])
        def solve(*args,**kwargs):
            current[0]=blocked
            return ExecutionResult(TrackingCommand(.1,0.,False,None),'TRACKING',0.,0.,0.,1)
        monkeypatch.setattr(node._executor,'update',solve)
        emitted=[];monkeypatch.setattr(node,'_publish_twist',lambda v=0.,w=0.:emitted.append((v,w)))
        node._tick()
        assert emitted==[(0.,0.)]
        assert node._executor.map.revision==2
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_published_support_arrays_cannot_be_reopened_for_writing():
    grid=GridMap(np.ones((10,10),np.uint8),.01,(0.,0.))
    for array in (grid.states,grid.costs,grid.assumed,grid.eligible):
        with pytest.raises(ValueError):array.flags.writeable=True


@pytest.mark.parametrize('allow',[False,True])
def test_unknown_permission_and_full_body_spin_have_no_stationary_anchor(allow):
    from nav1cm.contracts import WorldSnapshot
    from nav1cm.follower_map import center_support
    from lunar_pure_wheeled_controller.standard_path_follower import StandardPathFollower
    from lunar_pure_wheeled_controller.tracking import TrackingPolicy,TrackingState
    support=center_support(WorldSnapshot(1,(),()),0.,0.,body_length_m=1.182,body_width_m=.818,allow_unknown=allow)
    grid=GridMap(np.where(support.cells==-1,0,support.cells+1),.01,support.origin,eligible=support.eligible)
    follower=StandardPathFollower(TrackingPolicy())
    follower.set_map(grid);follower.set_standard_path(((0.,0.,0.),(0.,0.,1.)))
    result=follower.update(TrackingState(0.,0.,0.),.05)
    assert grid.stationary_anchor is None and np.all(grid.states==0)
    assert (result.command.angular_z_radps>0.) is allow


def test_full_body_map_delayed_braking_tail_reaches_intrinsic_wall():
    from dataclasses import replace
    from nav1cm.contracts import CellRect,WorldSnapshot
    from nav1cm.follower_map import center_support
    from lunar_pure_wheeled_controller.standard_path_follower import StandardPathFollower
    from lunar_pure_wheeled_controller.mppi import MppiPolicy
    from lunar_pure_wheeled_controller.tracking import TrackingPolicy,TrackingState
    world=WorldSnapshot(1,(CellRect(79,-400,80,400),),(CellRect(-400,-400,400,400),))
    support=center_support(world,0.,0.,body_length_m=1.182,body_width_m=.818)
    grid=GridMap(support.cells+1,.01,support.origin,eligible=support.eligible)
    follower=StandardPathFollower(replace(TrackingPolicy(),execution_clearance_m=.000001),MppiPolicy(time_steps=1,nominal_response_time_s=0.))
    follower.set_standard_path(((0.,0.,0.),(1.,0.,0.)))
    follower.set_map(grid);follower.note_command(.2,0.,stamp_s=-1.)
    state=TrackingState(0.,.005,0.,.2)
    sequence=np.zeros((1,1,2))
    poses,velocity=follower._rollout(state,sequence,.05)
    assert grid.valid_motion(poses,velocity,.05,0.)[0]
    assert not follower.certify_publication(state,0.,0.,.1)


def test_slow_publication_recertification_covers_current_ros_age(monkeypatch):
    from types import SimpleNamespace as S
    from nav_msgs.msg import Odometry,Path
    from geometry_msgs.msg import PoseStamped
    from rclpy.time import Time
    from lunar_pure_wheeled_controller.execution import ExecutionResult
    from lunar_pure_wheeled_controller.tracking import TrackingCommand
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi_path_follower','-p','input_mode:=incremental_path'])
    node=None
    try:
        node=PureWheeledControllerNode();clock=[10_000_000_000]
        monkeypatch.setattr(node,'get_clock',lambda:S(now=lambda:Time(nanoseconds=clock[0])))
        odom=Odometry();odom.header.frame_id='map';odom.pose.pose.orientation.w=1.
        node._on_odometry(odom)
        path=Path();path.header.frame_id='map'
        for x in (0.,1.):
            p=PoseStamped();p.pose.position.x=x;p.pose.orientation.w=1.;path.poses.append(p)
        node._on_path(path)
        free=GridMap(np.ones((500,500),np.uint8),.01,(-2.5,-2.5))
        monkeypatch.setattr(node,'_standard_grid',lambda state:free)
        def solve(*args,**kwargs):
            clock[0]+=100_000_000
            return ExecutionResult(TrackingCommand(.1,0.,False,None),'TRACKING',0.,0.,0.,1)
        monkeypatch.setattr(node._executor,'update',solve)
        ages=[]
        def certify(state,v,w,age):
            ages.append(age)
            if len(ages)==1:clock[0]+=100_000_000
            return True
        monkeypatch.setattr(node._executor,'certify_publication',certify)
        node._tick()
        assert len(ages)==2 and ages[-1]>=.2
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_invalid_standard_startup_does_not_start_a_map_worker(monkeypatch):
    from lunar_pure_wheeled_controller import p3_map_source
    original=p3_map_source.create_p3_source
    sources=[]
    def create(*args,**kwargs):
        source=original(*args,**kwargs);sources.append(source);return source
    monkeypatch.setattr(p3_map_source,'create_p3_source',create)
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi_path_follower',
        '-p','input_mode:=incremental_path','-p','control_rate_hz:=-1.0'])
    try:
        with pytest.raises(ValueError,match='control_rate_hz'):PureWheeledControllerNode()
        assert not sources
    finally:
        for source in sources:source.close()
        rclpy.shutdown()


def test_lateral_feedback_is_preserved_in_body_frame_and_is_not_stopped(monkeypatch):
    import json
    from types import SimpleNamespace as S
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi_path_follower',
        '-p','input_mode:=incremental_path','-p','command_topic:=/lunar_test/task2_fix_cmd'])
    node=None
    try:
        node=PureWheeledControllerNode()
        tf=transform('map','sensor_odom')
        tf.transform.rotation.z=np.sin(np.pi/4)
        tf.transform.rotation.w=np.cos(np.pi/4)
        node._on_tf(TFMessage(transforms=[tf]))
        odom=Odometry();odom.header.frame_id='sensor_odom';odom.child_frame_id='wheel_body'
        odom.pose.pose.orientation.w=1.;odom.twist.twist.linear.y=.1
        node._on_odometry(odom)
        assert node._standard_state.yaw_rad==pytest.approx(np.pi/2)
        grid=GridMap(np.ones((500,500),np.uint8),.01,(-2.5,-2.5))
        monkeypatch.setattr(node,'_standard_grid',lambda state:grid)
        diagnostics=[];node._mppi_diagnostics=S(publish=lambda msg:diagnostics.append(json.loads(msg.data)))
        node._tick()
        assert (node._standard_state.linear_mps,node._standard_state.lateral_mps,
                diagnostics[-1]['measured_stopped'])==(0.,.1,False)
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_nan_lateral_feedback_is_rejected_and_cannot_renew_liveness():
    import json
    from types import SimpleNamespace as S
    from nav_msgs.msg import Odometry
    rclpy.init(args=['--ros-args','-p','controller_backend:=mppi_path_follower',
        '-p','input_mode:=incremental_path','-p','command_topic:=/lunar_test/task2_fix_cmd'])
    node=None
    try:
        node=PureWheeledControllerNode()
        good=Odometry();good.header.frame_id='map';good.pose.pose.orientation.w=1.
        node._on_odometry(good)
        assert node._odometry is good and node._odometry_received_at is not None
        bad=Odometry();bad.header.frame_id='map';bad.pose.pose.orientation.w=1.
        bad.twist.twist.linear.y=float('nan')
        node._on_odometry(bad)
        assert node._odometry is None and node._odometry_received_at is None
        assert node._standard_pending_odometry is None
        diagnostics=[];node._mppi_diagnostics=S(publish=lambda msg:diagnostics.append(json.loads(msg.data)))
        node._tick()
        assert diagnostics[-1]['event']=='certificate_unavailable'
        assert diagnostics[-1]['measured_stopped'] is None
        assert not diagnostics[-1]['certified_stop']
    finally:
        if node is not None:node.destroy_node()
        rclpy.shutdown()


def test_r7_fractional_p3_native_map_and_zero_history_stop():
    import json
    from dataclasses import replace
    from pathlib import Path
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from lunar_pure_wheeled_controller.p3_map_source import create_p3_source
    from lunar_pure_wheeled_controller.standard_path_follower import StandardPathFollower
    from lunar_pure_wheeled_controller.tracking import TrackingPolicy,TrackingState
    from nav1cm.follower_map import center_support
    from nav1cm.maps import extract_window
    from nav1cm.contracts import VehicleState
    capability=yaml.safe_load((Path(get_package_share_directory('lunar_nav1cm_planner'))/'config/wheel.yaml').read_text())['capability']
    source=create_p3_source(capability,(1.182,.818))
    x,y,yaw=-.00022610096342229037,.0023270243419165264,1.66648642142486
    try:
        message=elevation()
        coords=4-(np.arange(160)+.5)*.05
        xx,yy=np.meshgrid(coords,coords,indexing='ij');h=np.zeros((160,160),np.float32)
        h[(xx>=1.2)&(xx<1.5)&(yy>=-2)&(yy<.2)]=2.
        message.data[0].data=h.ravel(order='F').tolist()
        source.set_center(x,y);source.submit_map(message,1.);assert source.wait_idle()
        assert source.error is None
        grid=source.snapshot().grid;world=source._world
        reference=center_support(world,x,y,body_length_m=1.182,body_width_m=.818)
        np.testing.assert_array_equal(grid.states,reference.cells+1)
        np.testing.assert_array_equal(grid.eligible,reference.eligible)
        follower=StandardPathFollower(replace(TrackingPolicy(),execution_clearance_m=.01))
        follower.set_map(grid)
        # r7 最后一条 UPDATE 的实际时钟与最近 12 条零指令；不是完整历史重放。
        follower._model_time=1793180631.4765975
        for stamp in [1793180630.7439957, 1793180630.9431612, 1793180630.977659, 1793180630.9961402, 1793180631.0438285, 1793180631.0939012, 1793180631.1440206, 1793180631.1939466, 1793180631.244358, 1793180631.3865006, 1793180631.4006023, 1793180631.4462614]:
            follower.note_command(0.,0.,stamp_s=stamp)
        state=TrackingState(x,y,yaw,0.,0.)
        certified=follower.certify_publication(state,0.,0.,0.002820814)
        planner=extract_window(world,VehicleState(x,y,yaw,0.,0.),False)
        print('R7_RECONSTRUCTION',json.dumps(dict(origin=grid.origin,epoch=world.epoch,
            planner_digest=planner.digest(),free_rectangles=len(world.known_free),
            states=dict(zip(*[a.tolist() for a in np.unique(grid.states,return_counts=True)])),
            eligible=int(grid.eligible.sum()),center_state=int(grid.states[250,250]),
            certified_stop=bool(certified),terrain_sha=source._store._terrain.library_sha256,
            field_sha=source._backend.library_sha256)))
        assert grid.origin==(x-2.5,y-2.5)
        assert grid.states[250,250]==1
        assert certified
    finally:source.close()


@pytest.mark.parametrize('margin',[.2,.4])
def test_direct_p3_clearance_support_drives_continuous_command_certificate(margin):
    from types import SimpleNamespace
    from nav1cm.contracts import CellRect,WorldSnapshot
    from lunar_pure_wheeled_controller.p3_map_source import P3MapSource
    free=(CellRect(-500,-500,500,500),)
    def grid_at(front,clearance):
        world=WorldSnapshot(1,(CellRect(front,-10,front+1,10),),free)
        source=P3MapSource(SimpleNamespace(apply=lambda *args:world),None,(1.182,.818),minimum_clearance_m=clearance)
        try:
            source.set_center(0.,0.);source.submit_map(elevation());assert source.wait_idle()
            assert source.error is None
            return source.snapshot().grid
        finally:source.close()
    front=int(round((.591+margin-.05)*100))
    poses=np.array([[[0.,0.,0.],[.01,0.,0.]]]);velocity=np.array([[[.1,0.]]])
    assert grid_at(front,0.).valid_motion(poses,velocity,.1,0.).all()
    assert not grid_at(front,margin).valid_motion(poses,velocity,.1,0.).all()
    far=int(np.ceil((np.hypot(1.182,.818)/2+margin+.05)*100))
    # 支撑图已包含配置净空；command层只追加执行误差，不能再加一遍净空。
    assert grid_at(far,margin).valid_motion(poses,velocity,.1,0.).all()
