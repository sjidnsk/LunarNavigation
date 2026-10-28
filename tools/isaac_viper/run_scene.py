"""单实例CPU PhysX/RTX/ROS运行端；由宿主Humble独立发送Twist并验收。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback


class RuntimeLifecycle:
    """Stop不可逆结束本控制会话；Pause不使物理视图失效。"""
    def __init__(self, keep_open):
        self.keep_open = keep_open
        self.stop_latched = False
        self.view_access_after_stop = self.target_write_after_stop = 0

    def notify_stop(self):
        self.stop_latched = True

    def control_state(self, playing):
        if self.stop_latched:
            return 'requires_runtime_restart' if self.keep_open else 'stopped_by_user'
        return 'running' if playing else 'paused'

    def guard_views(self, *, write=False):
        if self.stop_latched:
            if write:
                self.target_write_after_stop += 1
            else:
                self.view_access_after_stop += 1
            raise RuntimeError('Stop invalidated views; restart this runtime session')


def require_stage_profile(stage_path, profile, report_path):
    """物理/ROS 启动前只读验证实际 USD；失败和读取异常都先持久化再拒绝。"""
    result = {'passed': False, 'errors': [], 'stage': str(Path(stage_path).resolve()),
              'boundary': 'READ_ONLY_PXR; physics/ROS NOT_RUN'}
    try:
        result['stage_sha256'] = hashlib.sha256(Path(stage_path).read_bytes()).hexdigest()
        result['profile_semantic_sha256'] = hashlib.sha256(json.dumps(
            profile, sort_keys=True, allow_nan=False).encode('utf-8')).hexdigest()
        from tools.isaac_viper.validate_stage import validate_stage
        result.update(validate_stage(stage_path, profile))
        sensors = profile['sensors']
        # IMU 跟随固定 .01s 物理步；RTX tick_rate 虽可设置，但不能超过15Hz渲染。
        if sensors['imu']['frequency_hz'] != 100:
            result['errors'].append('imu.frequency_hz: runtime requires 100 Hz physics sampling')
        for name in ('stereo', 'lidar'):
            if sensors[name]['frequency_hz'] > 15:
                result['errors'].append(name + '.frequency_hz: exceeds 15 Hz render scheduling')
        for name, implementation in (('imu', 'physics_sensor'), ('lidar', 'rtx_builtin_schema')):
            if sensors[name]['implementation'] != implementation:
                result['errors'].append(name + '.implementation: unsupported by runtime')
        result['passed'] = bool(result['passed']) and not result['errors']
    except Exception as exc:
        result['passed'] = False
        result['errors'].append(type(exc).__name__ + ': ' + str(exc))
    Path(report_path).write_text(json.dumps(result, ensure_ascii=False, indent=2,
                                           allow_nan=False), encoding='utf-8')
    if not result['passed']:
        raise RuntimeError('stage/profile contract rejected; see ' + str(report_path))
    return result


def prepare_runtime_stage(source_path, runtime_path):
    """无GPU创建运行覆盖层，并在交给Kit前验证不继承的物理单位元数据。"""
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
    source = Usd.Stage.Open(str(Path(source_path).resolve()))
    runtime = Usd.Stage.CreateNew(str(Path(runtime_path).resolve()))
    runtime.GetRootLayer().subLayerPaths = [str(Path(source_path).resolve())]
    # Stage级单位/upAxis/defaultPrim不从sublayer继承；必须在新root明确声明。
    UsdGeom.SetStageMetersPerUnit(runtime, UsdGeom.GetStageMetersPerUnit(source))
    UsdGeom.SetStageUpAxis(runtime, UsdGeom.GetStageUpAxis(source))
    UsdPhysics.SetStageKilogramsPerUnit(runtime, UsdPhysics.GetStageKilogramsPerUnit(source))
    copied = ['metersPerUnit', 'upAxis', 'kilogramsPerUnit']
    for key in ('timeCodesPerSecond', 'framesPerSecond', 'defaultPrim'):
        if source.HasAuthoredMetadata(key):
            runtime.SetMetadata(key, source.GetMetadata(key))
            copied.append(key)
    runtime.OverridePrim('/Render').SetActive(True)
    legacy_products = []
    for prim in list(runtime.Traverse()):
        if str(prim.GetPath()).startswith('/Render/') and prim.GetTypeName() in ('RenderProduct','RenderSettings'):
            legacy_products.append(str(prim.GetPath()))
            prim.SetActive(False)
    # 旧材质数值已在冻结组合层，但祖先over使其未定义；只恢复定义和缺失绑定。
    # 材质字段继续继承源意见，不另造/拟合摩擦、恢复或combine数值。
    UsdGeom.Scope.Define(runtime, '/World/Materials')
    material_audit = {'materials':{}, 'wheel_binding_purpose':'physics', 'wheel_bindings':{}}
    for name in ('TireMaterial','GroundContact'):
        path = '/World/Materials/'+name
        original = source.GetPrimAtPath(path)
        if not original or original.GetTypeName() != 'Material':
            raise RuntimeError('missing source physics material: '+path)
        for field in ('physics:staticFriction','physics:dynamicFriction','physics:restitution'):
            if not original.GetAttribute(field).HasAuthoredValueOpinion():
                raise RuntimeError('source physics material field is not authored: '+path+'/'+field)
        restored = UsdShade.Material.Define(runtime, path).GetPrim()
        source_fields = {a.GetName():a.Get() for a in original.GetAttributes() if a.HasAuthoredValueOpinion()}
        restored_fields = {key:restored.GetAttribute(key).Get() for key in source_fields}
        if not restored.IsDefined() or source_fields != restored_fields:
            raise RuntimeError('restored physics material differs from source: '+path)
        material_audit['materials'][path] = {'source_defined':original.IsDefined(),
            'runtime_defined':restored.IsDefined(), 'source_fields':source_fields,
            'runtime_fields':restored_fields, 'source_api_schemas':str(original.GetMetadata('apiSchemas')),
            'runtime_api_schemas':str(restored.GetMetadata('apiSchemas'))}
    tire = UsdShade.Material(runtime.GetPrimAtPath('/World/Materials/TireMaterial'))
    for corner in ('fl','fr','rl','rr'):
        path = f'/World/ProjectVIPER/{corner}_wheel/Collision'
        binding = UsdShade.MaterialBindingAPI.Apply(runtime.GetPrimAtPath(path))
        binding.Bind(tire, UsdShade.Tokens.weakerThanDescendants, 'physics')
        bound = binding.ComputeBoundMaterial('physics')[0]
        if not bound or bound.GetPath() != tire.GetPath():
            raise RuntimeError('wheel source physics binding failed: '+path)
        material_audit['wheel_bindings'][path] = str(bound.GetPath())
    ground = UsdShade.MaterialBindingAPI(runtime.GetPrimAtPath('/World/Environment')).ComputeBoundMaterial('physics')[0]
    if not ground or str(ground.GetPath()) != '/World/Materials/GroundContact':
        raise RuntimeError('source ground physics binding was lost')
    material_audit['ground_inherited_binding'] = str(ground.GetPath())
    custom_data = dict(source.GetRootLayer().customLayerData)
    custom_data['physics_material_restore_json'] = json.dumps(material_audit, ensure_ascii=False)
    runtime.GetRootLayer().customLayerData = custom_data
    keys = ('metersPerUnit','upAxis','kilogramsPerUnit','timeCodesPerSecond','framesPerSecond','defaultPrim')
    comparison = {key: {'source':source.GetMetadata(key), 'runtime':runtime.GetMetadata(key),
        'source_authored':source.HasAuthoredMetadata(key), 'copied':key in copied} for key in keys}
    if any(comparison[key]['source'] != comparison[key]['runtime'] for key in copied):
        raise RuntimeError('runtime stage metadata does not match frozen source')
    runtime.GetRootLayer().Save()
    return legacy_products, comparison


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', required=True)
    parser.add_argument('--profile', default='config/isaac_viper.yaml')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--duration', type=float, default=30., help='仿真秒；0仅允许配合--keep-open')
    parser.add_argument('--wall-timeout', type=float, default=600.)
    parser.add_argument('--keep-open', action='store_true', help='保留GUI；Pause可恢复，Stop后需重新运行原命令建立控制会话')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--lifecycle-probe', choices=('stop','stop-play'),
        help='有界SDK回归：在.5sim秒Stop，可同frame再Play；记录后退出测试')
    args = parser.parse_args()
    if args.duration < 0 or (args.duration == 0 and not args.keep_open):
        parser.error('duration must be positive, or zero with --keep-open')
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=False)
    status_path = output/'status.json'
    log = (output/'telemetry.jsonl').open('w', encoding='utf-8', buffering=1)

    def record(event):
        log.write(json.dumps(event, ensure_ascii=False, allow_nan=False)+'\n')

    status = {'state':'starting', 'pid':os.getpid(), 'argv':sys.argv, 'stage':str(Path(args.stage).resolve()),
        'stage_sha256':hashlib.sha256(Path(args.stage).read_bytes()).hexdigest(),
        'ros_domain_id':os.environ.get('ROS_DOMAIN_ID'), 'localhost_only':os.environ.get('ROS_LOCALHOST_ONLY'),
        'physical_dt':.01, 'render_hz':15, 'odometry_adapter':'simulation_truth',
        'NOT_RUN':['Orin', 'real_vehicle', 'real_calibration', 'SLAM', 'navigation']}

    def save():
        status_path.write_text(json.dumps(status, ensure_ascii=False, indent=2), encoding='utf-8')

    save()
    app = bridge = sim = None
    try:
        from tools.isaac_viper.config import load_profile
        from tools.isaac_viper.drive import DriveController
        from tools.isaac_viper.ros_bridge import RosBridge, assert_ros_environment
        assert_ros_environment()
        profile = load_profile(args.profile)
        # 所有ABI环境由启动脚本在进程创建前设置；禁止注入系统cp310 overlay。
        if any('/opt/ros/' in p for p in sys.path):
            raise RuntimeError('system ROS Python overlay cannot enter Kit cp312')
        from isaacsim import SimulationApp
        app = SimulationApp({'headless':args.headless, 'width':1280, 'height':720,
            'renderer':'RaytracedLighting', 'sync_loads':True,
            'extra_args':[f'--/log/file={output / "kit.log"}',
                          f'--/app/tokens/cache={output / "kit-cache"}']})
        # SDK完成PXR bootstrap后，只读核对冻结输入；尚未加载用户stage或启动物理/ROS。
        status['stage_profile_contract'] = require_stage_profile(
            args.stage, profile, output/'stage-profile-contract.json')
        save()
        from isaacsim.core.experimental.utils.app import enable_extension
        for ext in ('isaacsim.ros2.bridge', 'isaacsim.sensors.experimental.rtx', 'isaacsim.sensors.experimental.physics'):
            enable_extension(ext)
        app.update()
        import omni.usd
        import omni.timeline
        from pxr import UsdGeom, Gf
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.core.rendering_manager import RenderingManager
        import isaacsim.core.experimental.utils.app as app_utils
        import isaacsim.core.experimental.utils.stage as stage_utils
        import numpy as np
        from tools.isaac_viper.sensors import RealSensors, rotation_matrix, active_render_products, NativePhysicsProbe
        # 在交给Kit/viewport之前准备运行覆盖层，避免inactive /Render使
        # viewport在stage-open事件中创建/Render/OmniverseKit失败。
        runtime_path = output/'runtime.usda'
        legacy_products, metadata_gate = prepare_runtime_stage(args.stage, runtime_path)
        status.update(runtime_stage=str(runtime_path), disabled_legacy_render_leaves=legacy_products,
                      stage_metadata_gate=metadata_gate)
        save()
        omni.usd.get_context().open_stage(str(runtime_path))
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            app.update()
        stage = omni.usd.get_context().get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        metadata = json.loads(stage.GetRootLayer().customLayerData['engineering_metadata_json'])
        status['physics_material_restore'] = json.loads(stage.GetRootLayer().customLayerData['physics_material_restore_json'])
        com = metadata['base_link_origin_in_footprint_m']
        # 100Hz物理步对应100Hz真实IMU；渲染15Hz单独调度。
        sim = SimulationManager
        if sim.get_active_physics_engine() != 'physx':
            if not sim.switch_physics_engine('physx'):
                raise RuntimeError('cannot activate PhysX')
        sim.set_default_physics_scene('/World/PhysicsScene')
        sim.set_backend('numpy')
        sim.set_device('cpu')
        sim.set_physics_dt(.01)
        RenderingManager.set_dt(1/15)
        status['render_clock_override'] = {'timeCodesPerSecond':stage.GetTimeCodesPerSecond(),
            'reason':'RenderingManager.set_dt(1/15); physics_dt remains .01s'}
        sensors = RealSensors(stage, profile)
        # 前方世界+Y；查看车头与路面，不改变任何actor位姿。
        initial = metadata['initial_position_m']
        view = UsdGeom.Camera.Define(stage, '/World/EngineeringView')
        view_xf = UsdGeom.Xformable(view.GetPrim())
        matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(initial[0]+3,initial[1]+3,initial[2]+2.5),
            Gf.Vec3d(initial[0],initial[1],initial[2]+.6),Gf.Vec3d(0,0,1)).GetInverse()
        view_xf.AddTransformOp().Set(matrix)
        from omni.kit.viewport.utility import get_active_viewport
        viewport = get_active_viewport()
        if viewport is not None:
            viewport.camera_path = str(view.GetPath())
        controller = DriveController(profile)
        bridge = RosBridge(profile, controller, com, record)
        timeline = omni.timeline.get_timeline_interface()
        lifecycle = RuntimeLifecycle(args.keep_open)
        def on_timeline_event(event):
            if event.type == int(omni.timeline.TimelineEventType.STOP):
                lifecycle.notify_stop()
        stop_subscription = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), on_timeline_event)
        app_utils.play()  # commit=True：立即提交状态并触发物理/传感器callbacks。
        sim.initialize_physics()
        for _ in range(3):
            sim.step()
        import omni.physics.tensors as tensors
        lifecycle.guard_views()
        stage_id = stage_utils.get_stage_id(stage)
        sv = tensors.create_simulation_view('numpy', stage_id=stage_id, backend='physx')
        sv.set_subspace_roots('/')
        av = sv.create_articulation_view('/World/ProjectVIPER')
        names = list(av.shared_metatype.dof_names)
        corners = ('fl','fr','rl','rr')
        steer = [names.index(c+'_steering_joint') for c in corners]
        roll = [names.index(c+'_rolling_joint') for c in corners]
        if av.count != 1 or len(names) != 8:
            raise RuntimeError(f'unexpected articulation {av.count} {names}')
        indices = np.arange(av.count, dtype=np.uint32)
        actual_dt = sim.get_physics_dt('/World/PhysicsScene')
        actual_tps = stage.GetPrimAtPath('/World/PhysicsScene').GetAttribute('physxScene:timeStepsPerSecond').Get()
        if abs(actual_dt-.01) > 1e-9 or actual_tps != 100:
            raise RuntimeError(f'physics timestep mismatch: dt={actual_dt}, steps_per_second={actual_tps}')
        native_probe = NativePhysicsProbe(sv, stage, sim)
        record(native_probe.capture('after_init', include_raycast=True))
        status.update(state='running', dof_names=names, active_render_products=active_render_products(stage),
                      metadata_com=com, rclpy_file=bridge.rclpy.__file__,
                      physics_stage_id=stage_id, physics_backend='physx_numpy_cpu',
                      physical_dt=actual_dt, physics_time_steps_per_second=actual_tps,
                      node_use_sim_time=bridge.node.get_parameter('use_sim_time').value,
                      node_subscription_topics=[sub.topic_name for sub in bridge.node.subscriptions],
                      dds_service='background_executor_latest_mailbox',
                      initial_ros_nodes=bridge.node.get_node_names_and_namespaces())
        save()
        wall_start, next_render, last_status = time.monotonic(), 0., 0.
        last_playing = True
        completed = False
        saved_samples = set()
        last_control_stamp = float(sim.get_simulation_time())
        stop_handled, probe_triggered, restart_pumps = False, False, 0
        restart_window = None
        first_step_probe_done = False
        while app.is_running():
            if not args.keep_open and time.monotonic()-wall_start > args.wall_timeout:
                raise TimeoutError('bounded runtime wall timeout')
            executor_check_start_wall = time.monotonic()
            bridge.check_executor()
            executor_check_done_wall = time.monotonic()
            playing = timeline.is_playing()
            if timeline.is_stopped():
                lifecycle.notify_stop()
            control_state = lifecycle.control_state(playing)
            if lifecycle.stop_latched:
                if not stop_handled:
                    bridge.clear_commands()
                    controller.receive_twist(float('nan'),0,0,time.monotonic())
                    status.update(state=control_state, last_control_sim_time=last_control_stamp,
                                  stop_event_latched=True)
                    record({'event':'control_session_stopped','last_control_time':last_control_stamp,
                            'state':control_state})
                    if args.keep_open and not args.headless:
                        import omni.ui as ui
                        restart_window = ui.Window('VIPER 控制会话需重新启动', width=520, height=160)
                        with restart_window.frame:
                            ui.Label('Stop 已结束本控制会话。Play不会恢复旧控制。\n'
                                '如需恢复，请关闭此运行窗口，重新执行原 run_scene --keep-open 启动命令。\n'
                                'Pause/Resume在Stop之前继续可用。', word_wrap=True)
                    print('VIPER control stopped; relaunch the original run_scene command to rebuild the session.', flush=True)
                    save()
                    stop_handled = True
                if playing:
                    app_utils.pause()  # 拒绝复用旧会话；不自行重新play。
                    status['play_after_stop_blocked'] = True
                if not args.keep_open:
                    break
                app.update()  # 只服务GUI，禁止访问旧av或恢复控制。
                restart_pumps += 1
                if args.lifecycle_probe and restart_pumps >= 2:
                    break
                continue
            if not playing:
                if last_playing:
                    bridge.clear_commands()
                    controller.receive_twist(float('nan'),0,0,time.monotonic())
                    if not timeline.is_stopped():
                        lifecycle.guard_views(write=True)
                        vt = av.get_dof_velocity_targets().copy()
                        vt[:,roll] = 0
                        av.set_dof_velocity_targets(vt, indices)
                    record({'event':'user_pause_or_stop','time':sim.get_simulation_time()})
                last_playing = False
                app.update()
                continue
            if not last_playing:
                controller.receive_twist(float('nan'),0,0,time.monotonic())
                bridge.clear_commands()
            last_playing = True
            lifecycle.guard_views()
            feedback_stamp = float(sim.get_simulation_time())
            pose = av.get_root_transforms()[0].copy()
            velocity = av.get_root_velocities()[0].copy()
            positions = av.get_dof_positions()[0].copy()
            r = rotation_matrix(pose[3:])
            body_velocity, body_omega = r.T@velocity[:3], r.T@velocity[3:]
            bridge.consume_command()
            drive_step_wall = time.monotonic()
            command_age = (drive_step_wall-controller.command_time
                           if controller.command_time is not None else None)
            out = controller.step(.01, drive_step_wall, positions[steer].tolist(),
                (float(np.linalg.norm(body_velocity[:2])),float(body_omega[2])))
            lifecycle.guard_views(write=True)
            pt, vt = av.get_dof_position_targets().copy(), av.get_dof_velocity_targets().copy()
            pt[:,steer], vt[:,roll] = out['steering_rad'], out['wheel_radps']
            av.set_dof_position_targets(pt, indices)
            av.set_dof_velocity_targets(vt, indices)
            sim.step()
            physics_done_wall = time.monotonic()
            if lifecycle.stop_latched:
                continue
            lifecycle.guard_views()
            stamp = float(sim.get_simulation_time())
            last_control_stamp = stamp
            pose, velocity = av.get_root_transforms()[0].copy(), av.get_root_velocities()[0].copy()
            positions_post = av.get_dof_positions()[0].copy()
            velocities_post = av.get_dof_velocities()[0].copy()
            if not first_step_probe_done:
                record(native_probe.capture('first_physics_step'))
                first_step_probe_done = True
            bridge.state(stamp, pose, velocity)
            bridge.imu(sensors.imu_sample())
            state_publish_done_wall = time.monotonic()
            record({'event':'physics', 'time':stamp, 'root_pose_xyzw':pose.tolist(),
                'root_world_velocity':velocity.tolist(), 'dof_positions':positions_post.tolist(),
                'dof_velocities':velocities_post.tolist(), 'targets':out,
                'control_feedback_time':feedback_stamp, 'state_sample_phase':'post_physics_step',
                'executor_check_start_wall':executor_check_start_wall,
                'executor_check_done_wall':executor_check_done_wall,
                'drive_step_wall':drive_step_wall, 'rx_wall':bridge.last_rx_wall,
                'rx_sequence':bridge.rx_count, 'command_age_s':command_age,
                'consumed_sequence':bridge.consumed_sequence,
                'physics_done_wall':physics_done_wall,
                'state_publish_done_wall':state_publish_done_wall})
            if stamp >= next_render:
                render_start_wall = time.monotonic()
                render_rx_sequence_before = bridge.rx_count
                RenderingManager.render()
                render_done_wall = time.monotonic()
                if lifecycle.stop_latched:
                    continue
                status['active_render_products'] = active_render_products(stage)
                status['render_products_after_final_camera_setup'] = True
                camera_samples = sensors.camera_samples()
                lidar_sample = sensors.lidar_sample()
                for side, (capture_time, rgb, depth, timing) in camera_samples.items():
                    if side not in saved_samples:
                        np.savez_compressed(output/f'first-{side}-camera.npz',
                            rgb=rgb, depth=depth, simulation_time=capture_time,
                            camera_k=bridge.k)
                        saved_samples.add(side)
                if lidar_sample is not None and 'lidar' not in saved_samples:
                    capture_time, xyz, flags = lidar_sample
                    np.savez_compressed(output/'first-lidar.npz', xyz=xyz,
                        flags=flags, simulation_time=capture_time)
                    saved_samples.add('lidar')
                bridge.cameras(camera_samples)
                bridge.lidar(lidar_sample)
                record({'event':'render_timing', 'time':stamp,
                    'render_start_wall':render_start_wall, 'render_done_wall':render_done_wall,
                    'sensor_publish_done_wall':time.monotonic(), 'last_rx_wall':bridge.last_rx_wall,
                    'rx_sequence_before':render_rx_sequence_before, 'rx_sequence_after':bridge.rx_count})
                next_render += 1/15
            if stamp-last_status >= 1:
                status.update(sim_time=stamp, wall_elapsed_s=time.monotonic()-wall_start,
                              counts=bridge.counts, ros_nodes=bridge.node.get_node_names_and_namespaces(),
                              active_render_products=active_render_products(stage))
                save()
                last_status = stamp
            if args.lifecycle_probe and stamp >= .5 and not probe_triggered:
                probe_triggered = True
                lifecycle.guard_views()
                record(native_probe.capture('before_stop', include_raycast=True))
                app_utils.stop()
                if args.lifecycle_probe == 'stop-play':
                    app_utils.play()
                record({'event':'lifecycle_probe_action','time':stamp,'action':args.lifecycle_probe})
                continue
            if args.duration and stamp >= args.duration and not completed:
                controller.receive_twist(0,0,0,time.monotonic())
                completed = True
                if not args.keep_open:
                    break
        status.update(state=lifecycle.control_state(False) if lifecycle.stop_latched else 'finished',
                      sim_time=last_control_stamp, counts=bridge.counts,
                      wall_elapsed_s=time.monotonic()-wall_start,
                      view_access_after_stop=lifecycle.view_access_after_stop,
                      target_write_after_stop=lifecycle.target_write_after_stop,
                      active_render_products=active_render_products(stage))
        # 收尾清轮速目标；暂停/停止不会继续盲步进。
        if not lifecycle.stop_latched:
            lifecycle.guard_views(write=True)
            record(native_probe.capture('before_normal_exit', include_raycast=True))
            vt = av.get_dof_velocity_targets().copy()
            vt[:,roll] = 0
            av.set_dof_velocity_targets(vt, indices)
    except BaseException:
        status.update(state='failed', error=traceback.format_exc())
        raise
    finally:
        save()
        if bridge is not None:
            bridge.close()
        if app is not None:
            app.close(exit_code=1 if status['state'] == 'failed' else 0)
        log.close()


if __name__ == '__main__':
    main()
