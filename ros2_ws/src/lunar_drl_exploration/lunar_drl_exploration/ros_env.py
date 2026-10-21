"""Synchronous isolated native-navigation/public-controller training environment.

Task8 owns workers, asynchronous dispatch, retained completions/ACK and recovery.
This object never imports Torch and never resets inside step().
"""
from dataclasses import asdict, dataclass
import math
import json
from pathlib import Path
import os
import threading
import time
import uuid
import numpy as np
from .contracts import Pose, PrivilegedState, RewardParts, Transition
from .config import GraphConfig
from .frontiers import count_removed
from .decision import DecisionCore
from .plant import KinematicPlant, GeometryFailure
from .processes import OwnedProcesses
from .stagnation import StagnationMonitor
from .ros_messages import (InfrastructureError, MeasurementBuffer, RosNavigationAdapter,
                           NavigationResult, grid_message, odometry_message, ros_time, velocity_is_stopped)


class InputUnavailable(RuntimeError): pass
class TaskCanceled(RuntimeError): pass


@dataclass(frozen=True)
class ExecutionMetadata:
    goal: tuple
    outcome: int
    reason_code: str
    last_segment_revision: int
    goal_cell: tuple
    actual_cell: tuple
    actual_pose: tuple
    distance_m: float
    turn_rad: float
    simulation_s: float
    wall_s: float
    map_revision: int
    map_processed_stamp_ns: int
    geometry_failure: bool = False


def make_transition(observation,action,next_observation,privileged,next_privileged,*,
                    initial,final,removed_frontiers_in: int,removed_frontiers_out: int,
                    completed,budget_hit,episode_id,actor_version,completion_bonus=0.,exhausted=False,approach_reward=None,gamma=1.,exploration_reward_scale=1.):
    area,distance,turn=(float(b-a) for a,b in zip(initial,final))
    if area < -1e-8 or distance < -1e-8 or turn < -1e-8:
        raise InfrastructureError('nonmonotonic whole-action accounting')
    parts=RewardParts(area,distance,turn,removed_frontiers_in,removed_frontiers_out)
    reward=exploration_reward_scale*parts.removed_frontiers_in/50.-.02*distance/10.-.005*turn/math.pi-.001
    if completed:
        reward += completion_bonus
    terminated=bool(completed or exhausted)
    from .approach_potential import observation_shaping
    auxiliary=observation_shaping(observation,next_observation,approach_reward,terminated=terminated,gamma=gamma)
    from dataclasses import replace
    parts=replace(parts,approach_reward=auxiliary)
    reward+=auxiliary
    return Transition(observation,action,reward,next_observation,privileged,next_privileged,
        parts,terminated,bool(budget_hit and not terminated),episode_id,int(actor_version))


class RosExplorationEnv:
    def __init__(self,config,env_id,*,domain_base=210,wall_timeout_s=120.,training=False,simulation_step_sync=False,
                 controller_backend=None,mppi_nominal_response_time_s=None):
        self.config=config;self.env_id=int(env_id)
        self._simulation_step_sync=bool(simulation_step_sync)
        from .evaluation_navigation import validate_evaluation_navigation
        navigation=validate_evaluation_navigation(dict(
            controller_backend=controller_backend or ('mppi' if simulation_step_sync else 'pure_pursuit'),
            simulation_step_sync=simulation_step_sync,
            mppi_nominal_response_time_s=mppi_nominal_response_time_s))
        self._controller_backend=navigation['controller_backend']
        self._mppi_nominal_response_time_s=mppi_nominal_response_time_s
        self._mppi_compute_ms=[];self._mppi_deadline_events=[]
        if self._simulation_step_sync and abs(config.integration_step_s-.05)>1e-12:
            raise ValueError("simulation step synchronization requires the 20Hz 0.05s integration step")
        self.sensor=config.sensor
        self._fixed_sensor=self.sensor
        self._initial_reference_coverage=0.
        self._first_roi=None;self._approach_limits={};self._approach_start_distance=0.
        self._first_entry=None;self._initial_inside_task=False
        self.namespace=f'/lunar_training/env_{self.env_id}'
        self._requested_domain=None if domain_base is None else domain_base+self.env_id
        self.domain_id=self._requested_domain
        self._domain_lease=None
        if self.domain_id is not None and not 0 <= self.domain_id <= 232: raise ValueError('invalid isolated DDS domain')
        self.wall_timeout_s=float(wall_timeout_s)
        self.owned=OwnedProcesses();self.node=None;self.context=None;self.adapter=None;self.executor=None;self._spin_error=None;self._spin_thread=None
        self._cancel=threading.Event();self._closed=True;self.actor_version=0
        self.episode_budget=config.curriculum_budgets[0]
        self.last_execution=None;self.episode_metadata={};self.static_scenes={}
        self._step_active=False
        # Explicit construction mode prevents a training YAML from shortening
        # frozen evaluation or affecting the observed-only deployment runtime.
        self.stagnation=(StagnationMonitor(config.stagnation_window_steps,
            config.stagnation_max_positions,config.stagnation_position_tolerance_m)
            if training and config.stagnation_window_steps>0 else None)
        self.truncation_reason=None
        self._episode_truncated=False
        self.native_observation=None;self.fine_observation=None
        self.policy_graph_stats=None;self.policy_graph_diagnostics=None
        self._training_terrain_guard=None
        if training and config.cave_generator_version:
            from .region_relation_data_guard import TrainingTerrainGuard
            self._training_terrain_guard=TrainingTerrainGuard(config.artifact_root)

    @property
    def owned_pids(self): return self.owned.pids

    def reset(self,seed,family,extent,*,episode_budget=None,start_mode='any',curriculum_task=None,
              course_two_case=None,task_scope='',outside_distance_max_m=None,
              outside_distance_min_m=None,context_margin_m=12.,sensor=None,scene_profile='',
              cave_generator_version='',mother_graph_id=None,dataset_split='train',
              cave_split_manifest_hash=None,safe_start_radius_m=None):
        self.close()
        from .scene import validate_safe_start_radius, safe_start_record
        if safe_start_radius_m is None:safe_start_radius_m=self.config.safe_start_radius_m
        safe_start_radius_m=validate_safe_start_radius(safe_start_radius_m)
        if safe_start_radius_m!=self.config.safe_start_radius_m:
            raise ValueError('safe start radius differs from frozen configuration')
        if safe_start_radius_m:
            if course_two_case is not None:
                raise ValueError('safe start cannot modify a frozen course-two case')
            if safe_start_radius_m < self.config.platform.footprint_radius_m:
                raise ValueError('safe start radius must cover the vehicle footprint radius')
        if cave_generator_version and (family!='cave' or task_scope!='whole_cave_v1'
                                      or course_two_case is not None):
            raise ValueError('cave generator identity requires ordinary whole-cave scene')
        if self.config.cave_generator_version and family=='cave' and (
                cave_generator_version != self.config.cave_generator_version or
                cave_split_manifest_hash != self.config.cave_split_manifest_hash):
            raise ValueError('cave dispatch differs from frozen generator/split identity')
        if (outside_distance_max_m is not None or outside_distance_min_m is not None) and (
                family!='moon' or start_mode!='outside' or course_two_case is not None):
            raise ValueError('bounded distance applies only to ordinary outside moon scenes')
        if scene_profile not in ('','distance_sensor_10_30_v1'):
            raise ValueError('unknown ordinary scene profile')
        if course_two_case is not None and (sensor is not None or scene_profile or context_margin_m!=12.):
            raise ValueError('ordinary scene overrides cannot alter a frozen course-two case')
        from .contracts import SensorSpec
        from .sensor import validate_sensor
        if sensor is not None:
            sensor=SensorSpec(**sensor) if isinstance(sensor,dict) else sensor
            validate_sensor(sensor)
        extra={}
        if sensor is not None:extra['sensor']=asdict(sensor)
        if scene_profile:extra['scene_profile']=scene_profile
        from types import MappingProxyType
        if course_two_case is not None:
            rebalanced=course_two_case.get('version') in ('region_sensor_rebalanced_scene_v2','region_sensor_rebalanced_scene_v3')
            if rebalanced:
                from .course_two_rebalanced_scene import build_case
            else:from .course_two_scene import build_case
            if (seed != course_two_case['seed'] or family != course_two_case['family']
                    or curriculum_task != course_two_case['curriculum_task']):
                raise ValueError('course-two dispatch identity differs from frozen scene')
            if rebalanced and course_two_case.get('qualification_request'):
                from .course_two_rebalanced_scene import validate_case
                validate_case(course_two_case)
                from .course_two_prelude import resolve_generated
                return resolve_generated(self,course_two_case)
            terrain,pose,task,sensor,descriptor=build_case(course_two_case,self.config.platform)
            self.episode_budget=int(episode_budget or course_two_case['episode_budget'])
            if not 0 < self.episode_budget <= course_two_case['episode_budget']:
                raise ValueError('invalid course-two episode budget')
            self.reset_scene(terrain,pose,task,sensor=sensor,descriptor=descriptor)
            if rebalanced:
                from .course_two_prelude import prepare_start
                try:prepare_start(self,course_two_case)
                except BaseException:
                    self.close();raise
            return self.observation,self.privileged
        from .scene import Scene,TerrainGrid
        if start_mode not in ('any', 'inside', 'outside'):
            raise ValueError('start_mode must be any, inside or outside')
        if curriculum_task is not None and (
                start_mode == 'any' or curriculum_task != f'{family}_{start_mode}'):
            raise ValueError('curriculum_task must match family and start_mode')
        if task_scope:
            if family!='cave' or task_scope!='whole_cave_v1':raise ValueError('invalid rehearsal task scope')
            from .course_two_task_scope import build_rehearsal
            terrain,pose,task,descriptor=build_rehearsal(seed,extent,start_mode,self.config.platform,
                cave_generator_version=cave_generator_version,mother_graph_id=mother_graph_id,
                dataset_split=dataset_split,cave_split_manifest_hash=cave_split_manifest_hash,
                safe_start_radius_m=safe_start_radius_m)
            descriptor=dict(descriptor,**extra)
            terrain.generator_descriptor=MappingProxyType(dict(terrain.generator_descriptor,**extra))
            self.episode_budget=int(episode_budget or self.config.curriculum_budgets[0])
            if self.episode_budget<=0:raise ValueError('positive episode budget required')
            return self.reset_scene(terrain,pose,task,descriptor=descriptor,sensor=sensor)
        scene=Scene(seed,family,extent,platform=self.config.platform,context_margin_m=context_margin_m,
            safe_start_radius_m=safe_start_radius_m)
        terrain=TerrainGrid.from_scene(scene)
        self.episode_budget=int(episode_budget) if episode_budget is not None else self.config.curriculum_budgets[0 if extent<=80 else 1 if extent<=150 else 2]
        if self.episode_budget<=0:raise ValueError("positive episode budget required")
        start_descriptor=dict(start_mode=start_mode,**extra)
        if outside_distance_max_m is not None:
            start_descriptor['outside_distance_max_m']=outside_distance_max_m
        if outside_distance_min_m is not None:
            start_descriptor['outside_distance_min_m']=outside_distance_min_m
        if context_margin_m!=12.:start_descriptor['context_margin_m']=context_margin_m
        if curriculum_task is not None:
            start_descriptor['curriculum_task']=curriculum_task
        pose=scene.initial_pose(terrain,start_mode=start_mode,outside_distance_max_m=outside_distance_max_m,
            outside_distance_min_m=outside_distance_min_m,safe_start_radius_m=safe_start_radius_m)
        start_descriptor.update(safe_start_record(terrain,pose,safe_start_radius_m))
        if outside_distance_min_m is not None:
            from .scene import polygon_boundary_distance
            start_descriptor['start_boundary_distance_m']=float(polygon_boundary_distance(pose.x,pose.y,scene.task.polygon))
        terrain.generator_descriptor=MappingProxyType(dict(terrain.generator_descriptor,
            **start_descriptor))
        return self.reset_scene(terrain,pose,scene.task,sensor=sensor,
            descriptor={'seed':seed,'family':family,'extent_m':extent,
                        **start_descriptor})

    def reset_scene(self,terrain,pose,task,*,descriptor=None,sensor=None):
        """Explicit deterministic fixture entry; identical sensor/navigation path."""
        self.close()
        from .scene import safe_start_record, validate_safe_start_radius
        assumption=(descriptor or {}).get('safe_start_region',{})
        self._safe_start_radius_m=validate_safe_start_radius(assumption.get('navigation_radius_m',0.))
        if self._safe_start_radius_m!=self.config.safe_start_radius_m:
            raise ValueError('scene lacks the configured safe start region')
        if self._safe_start_radius_m:
            if self._safe_start_radius_m < self.config.platform.footprint_radius_m:
                raise ValueError('safe start radius must cover the vehicle footprint radius')
            canonical=safe_start_record(terrain,pose,self._safe_start_radius_m)
            if assumption!=canonical['safe_start_region']:
                raise ValueError('safe start certificate differs from actual geometry')
        from .reference import CoverageReference
        from .privileged import build_truth, PrivilegedBuilder
        from .sensor import validate_sensor
        self.sensor=self.config.sensor if sensor is None else sensor
        validate_sensor(self.sensor)
        self._fixed_sensor=self.sensor
        self.terrain=terrain;self.task=task
        self.episode_id=uuid.uuid4().hex
        guard=getattr(self,'_training_terrain_guard',None)
        terrain_sha=(guard.check(terrain,descriptor or {},self.episode_id,self.env_id)
                     if guard is not None else None)
        self.plant=KinematicPlant(terrain,pose,self.config.platform)
        from .approach_potential import distance_to_region
        self._initial_inside_task=distance_to_region((pose.x,pose.y),task.polygon)==0.
        self._first_entry=None
        self.reference=CoverageReference.build(terrain,pose,task,self.sensor)
        static=build_truth(terrain,self.reference,GraphConfig(platform=self.config.platform))
        self.privileged_builder=PrivilegedBuilder(terrain,static,task,self.sensor,
            GraphConfig(platform=self.config.platform))
        self.static_scenes={static.scene_id:static}
        self.core=DecisionCore(task,self.sensor,GraphConfig(platform=self.config.platform))
        self.core.history.tolerance_m=self.config.goal_position_tolerance_m
        self.buffer=MeasurementBuffer(terrain.origin,terrain.resolution_m)
        self._cancel.clear();self._closed=False;self.steps=0;self.last_execution=None
        self.truncation_reason=None;self._episode_truncated=False
        self._first_roi=None
        self._approach_limits=(dict(descriptor) if descriptor and descriptor.get('scenario_id') else {})
        if self.stagnation is not None:self.stagnation.reset()
        self.report=None
        self._sim_ns=1_000_000_000;self._last_sensor_ns=0;self._latest_sensor_ns=0
        self._command=(0.,0.);self._next_tick=time.monotonic()
        self._start_wall=time.monotonic();self._max_command_speed=0.;self._sensor_count=0
        try:
            self._spin_error=None
            self._start_ros()
            self._wait(lambda:self.adapter.ready,'native/controller discovery')
            # Wait for actual observed measurements, with continuing physical ticks.
            snap=self._snapshot()
            initial_turn=self.plant.turn_rad;initial_time=self.plant.simulation_s
            # A finite initialization scan uses exactly the same native action,
            # public controller and real sensor path as a policy action.
            self.observation,self.report=self.core.observe(snap,self.adapter.velocity)
            for quarter in range(4):
                if len(snap.start_connections) and self.report.available: break
                goal=(self.plant.pose.x,self.plant.pose.y,pose.yaw+(quarter+1)*math.pi/2)
                result=self._execute_goal(goal)
                snap=self._snapshot()
                if result.outcome!=0:
                    raise InputUnavailable(f'initialization {result.reason_code}')
                self.observation,self.report=self.core.observe(snap,self.adapter.velocity)
            if not len(snap.start_connections):
                raise InputUnavailable('bounded initialization produced no native start connection')
            if not self.report.available: raise InputUnavailable(self.report.reason_code)
            native=self.observation
            policy,fine,diagnostics,stats=self._prepare_policy_state(native,snapshot=snap)
            self.privileged=self._privileged(fine)
            self.observation=policy;self.native_observation=native;self.fine_observation=fine
            self.policy_graph_diagnostics=diagnostics;self.policy_graph_stats=stats
            self._initial_reference_coverage=self.reference.coverage_ratio(self.privileged.observed)
            self._approach_start_distance=self.plant.distance_m
            self.episode_metadata=dict(descriptor or {},episode_id=self.episode_id,
                scene_id=static.scene_id,reference_area_m2=self.reference.area_m2,
                initial_known_area_m2=self._known_area,
                initialization_turn_rad=self.plant.turn_rad-initial_turn,
                initialization_simulation_s=self.plant.simulation_s-initial_time,
                initialization_wall_s=time.monotonic()-self._start_wall,
                budget=self.episode_budget,domain_id=self.domain_id,namespace=self.namespace,
                sensor=asdict(self.sensor),initial_reference_coverage=self._initial_reference_coverage,
                initial_pose=[self.plant.pose.x,self.plant.pose.y,self.plant.pose.yaw],
                initial_roi_visible=self._initial_reference_coverage>0)
            if terrain_sha is not None:self.episode_metadata['terrain_sha256']=terrain_sha
            return self.observation,self.privileged
        except BaseException:
            self.close();raise

    def prelude_goal(self,goal):
        """Execute a real navigation prefix without generating a learning sample."""
        if time.monotonic()>getattr(self,'_preparation_deadline',float('inf')):
            raise TimeoutError('native preparation hard deadline exceeded')
        if self.sensor!=self._fixed_sensor:raise InfrastructureError('prelude sensor changed')
        if self.plant.collision:raise GeometryFailure('prelude collision')
        before=(self.plant.distance_m,self.plant.turn_rad,self.plant.simulation_s,time.monotonic())
        result=self._execute_goal(goal)
        snapshot=self._snapshot()
        native,report=self.core.observe(snapshot,self.adapter.velocity)
        if result.outcome!=0 or not report.available or self.plant.collision:
            raise InputUnavailable('prelude navigation failed: '+str(result.reason_code))
        policy,fine,diagnostics,stats=self._prepare_policy_state(native,snapshot=snapshot)
        self.observation,self.native_observation,self.fine_observation=policy,native,fine
        self.privileged=self._privileged(fine);self.report=report
        self.policy_graph_diagnostics=diagnostics;self.policy_graph_stats=stats
        p=self.plant.pose
        return dict(goal=list(goal),outcome=int(result.outcome),reason_code=result.reason_code,
            actual_pose=[p.x,p.y,p.yaw],distance_m=self.plant.distance_m-before[0],
            turn_rad=self.plant.turn_rad-before[1],simulation_s=self.plant.simulation_s-before[2],
            wall_s=time.monotonic()-before[3],map_revision=snapshot.revision,
            coverage=self.reference.coverage_ratio(self.privileged.observed))

    def set_policy_start(self,prelude):
        self._initial_reference_coverage=self.reference.coverage_ratio(self.privileged.observed)
        self._approach_start_distance=self.plant.distance_m
        self._first_roi=None;self.steps=0;self._episode_truncated=False;self.truncation_reason=None
        if self.stagnation is not None:self.stagnation.reset()
        p=self.plant.pose
        from .approach_potential import distance_to_region
        self._initial_inside_task=distance_to_region((p.x,p.y),self.task.polygon)==0.
        self._first_entry=None
        self.episode_metadata.update(prelude=prelude,initial_reference_coverage=self._initial_reference_coverage,
            initial_pose=[p.x,p.y,p.yaw],initial_roi_visible=self._initial_reference_coverage>0,
            initial_known_area_m2=self._known_area)

    def _start_ros(self):
        from .domain_lease import DomainLease
        self._domain_lease=DomainLease.acquire(self._requested_domain)
        self.domain_id=self._domain_lease.domain_id
        import rclpy
        from rclpy.context import Context
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage
        from grid_map_msgs.msg import GridMap
        from rosgraph_msgs.msg import Clock
        self.context=Context();rclpy.init(context=self.context,domain_id=self.domain_id,
            args=['--ros-args','--disable-external-lib-logs'])
        self.node=rclpy.create_node('drl_environment',namespace=self.namespace,context=self.context,
            enable_rosout=False,cli_args=['--ros-args','--disable-external-lib-logs'],
            parameter_overrides=[Parameter('use_sim_time',value=True)])
        from rclpy.executors import SingleThreadedExecutor
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        ns=self.namespace
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._clock_pub=self.node.create_publisher(Clock,'/clock',10)
        self._odom_pub=self.node.create_publisher(Odometry,ns+'/odom',10)
        self._tf_pub=self.node.create_publisher(TFMessage,ns+'/tf',10)
        self._map_pub=self.node.create_publisher(GridMap,ns+'/grid',qos)
        self._cmd_sub=self.node.create_subscription(Twist,ns+'/cmd_vel',self._on_command,10)
        if self._simulation_step_sync:
            from std_msgs.msg import String
            self._mppi_step_condition=threading.Condition()
            self._mppi_step_ns=None;self._mppi_step_command=None
            ack_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._mppi_step_sub=self.node.create_subscription(String,ns+'/mppi_step_ack',self._on_mppi_step,ack_qos)
        if self._controller_backend!='pure_pursuit':
            from std_msgs.msg import String
            self._mppi_diagnostics_sub=self.node.create_subscription(
                String,ns+'/mppi_diagnostics',self._on_mppi_diagnostics,100)
        self.adapter=RosNavigationAdapter(self.node,action_name=ns+'/navigate',policy_map_service=ns+'/get_map',
            odometry_topic=ns+'/odom',diagnostics_topic=ns+'/diagnostics',path_reference_topic=ns+'/reference',tf_topic=ns+'/tf')
        def spin():
            try:self.executor.spin()
            except Exception as exc:self._spin_error=exc
        self._spin_thread=threading.Thread(target=spin,daemon=True);self._spin_thread.start()
        env=dict(os.environ,ROS_DOMAIN_ID=str(self.domain_id),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        common={'platform_config':self._runtime_platform_config(),'use_sim_time':True,
                'goal_position_tolerance_m':self.config.goal_position_tolerance_m,
                'goal_yaw_tolerance_rad':self.config.goal_yaw_tolerance_rad,
                'odometry_topic':ns+'/odom','tf_topic':ns+'/tf'}
        self.owned.start('lunar_incremental_navigation_ros','lunar_incremental_navigation_node',ns,
            dict(common,local_map_topic=ns+'/grid',action_name=ns+'/navigate',path_reference_topic=ns+'/reference',
                local_path_topic=ns+'/path',global_route_topic=ns+'/route',diagnostics_topic=ns+'/diagnostics',
                exploration_map_topic=ns+'/map',policy_map_service=ns+'/get_map',tracking_status_topic=ns+'/tracking',
                enable_tracking_feedback=True),env)
        self.owned.start('lunar_pure_wheeled_controller','lunar_pure_wheeled_controller_node.py',ns,
            dict(common,input_mode='incremental_reference',path_reference_topic=ns+'/reference',
                tracking_status_topic=ns+'/tracking',reference_topic=ns+'/wheeled_reference',path_topic=ns+'/path',
                command_topic=ns+'/cmd_vel',execution_cancel_topic=ns+'/cancel',
                max_linear_mps=.2,max_angular_radps=1.,control_rate_hz=20.,**self._controller_parameters()),env)

    def _runtime_platform_config(self):
        """Freeze unchanged native settings and check the certified disk covers them."""
        radius=getattr(self,'_safe_start_radius_m',0.)
        if not radius:return self.config.platform.capability_path
        import hashlib
        from pathlib import Path
        import yaml
        source=Path(self.config.platform.capability_path).read_bytes()
        profile=yaml.safe_load(source)
        if profile.get('capability')!=dict(self.config.platform.capability):
            raise ValueError('runtime capability differs from frozen configuration')
        if hashlib.sha256(source).hexdigest()!=self.config.safe_start_profile_sha256:
            raise ValueError('runtime profile differs from frozen configuration')
        # LoadPlatformConfig's native default is 0.2m when the field is absent.
        patch_radius=self.config.platform.footprint_radius_m+float(profile.get('start_blind_zone_margin_m',.2))
        if radius < patch_radius:
            raise ValueError('safe start radius must cover the configured native start patch')
        content=yaml.safe_dump(profile,sort_keys=True).encode('utf-8')
        digest=hashlib.sha256(content).hexdigest()
        folder=Path(self.config.artifact_root or self.config.output_dir)/'safe-start-profiles'
        folder.mkdir(parents=True,exist_ok=True)
        path=folder/(digest+'.yaml')
        # Identical workers may create the same immutable content-addressed file.
        # Publish atomically so another worker never observes a partial YAML.
        import tempfile,os
        with tempfile.NamedTemporaryFile(dir=folder,delete=False) as stream:
            temporary=Path(stream.name);stream.write(content)
        try:
            os.link(temporary,path)
        except FileExistsError:
            if path.read_bytes()!=content:raise ValueError('safe start profile identity changed')
        finally:temporary.unlink()
        return str(path.resolve())

    def _controller_parameters(self):
        if self._controller_backend=='pure_pursuit':return {}
        ns=self.namespace
        parameters=dict(controller_backend=self._controller_backend,policy_map_service=ns+'/get_map',
                        mppi_diagnostics_topic=ns+'/mppi_diagnostics')
        if self._mppi_nominal_response_time_s is not None:
            parameters['mppi_nominal_response_time_s']=self._mppi_nominal_response_time_s
        if self._simulation_step_sync:
            parameters.update(mppi_simulation_step_sync=True,mppi_step_ack_topic=ns+'/mppi_step_ack')
        return parameters

    def _on_mppi_diagnostics(self,message):
        try:event=json.loads(message.data)
        except (ValueError,TypeError):return
        if not isinstance(event,dict) or event.get('event')!='solve_result':return
        value=event.get('compute_ms')
        if type(value) in (int,float) and math.isfinite(value):self._mppi_compute_ms.append(float(value))
        if event.get('deadline_missed'):self._mppi_deadline_events.append(event)

    def _on_mppi_step(self,msg):
        import json
        try:
            event=json.loads(msg.data);stamp=event.get('now_ns');command=event.get('command')
            if (event.get('event')!='control_step' or type(stamp) is not int or stamp!=self._sim_ns
                    or not isinstance(command,list) or len(command)!=2
                    or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in command)):
                return
        except (ValueError,TypeError,AttributeError):return
        with self._mppi_step_condition:
            self._mppi_step_ns=stamp;self._mppi_step_command=tuple(command)
            self._mppi_step_condition.notify_all()

    def _wait_mppi_step(self,republish):
        deadline=time.monotonic()+min(self.wall_timeout_s,5.)
        while True:
            if self._spin_error is not None:raise InfrastructureError(str(self._spin_error))
            try:self.owned.check()
            except RuntimeError as exc:raise InfrastructureError(str(exc)) from exc
            with self._mppi_step_condition:
                if self._mppi_step_ns==self._sim_ns:
                    self._command=self._mppi_step_command
                    self._max_command_speed=max(self._max_command_speed,abs(self._command[0]))
                    return
                remaining=deadline-time.monotonic()
                if remaining<=0:raise InfrastructureError('simulation controller step synchronization timeout')
                self._mppi_step_condition.wait(timeout=min(.02,remaining))
            # Discovery may miss the first clock/odom/TF. Repeat this frame,
            # never a physical integration or a sensor observation.
            republish()

    def _on_command(self,msg):
        if getattr(self,'_simulation_step_sync',False):return
        self._command=(msg.linear.x,msg.angular.z)
        self._max_command_speed=max(self._max_command_speed,abs(msg.linear.x))
        if abs(msg.linear.y)>1e-12: raise InfrastructureError('public controller issued lateral motion')

    def _pump(self):
        import rclpy
        from rosgraph_msgs.msg import Clock
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
        from .sensor import SensorModel
        try:self.owned.check()
        except RuntimeError as exc:raise InfrastructureError(str(exc)) from exc
        # Drain incoming commands before each physical integration. No simulated
        # time catch-up or omitted integration/sensor ticks when computation lags.
        if self._spin_error is not None:raise InfrastructureError(str(self._spin_error))
        if not self.plant.collision:
            try:self.plant.advance(*self._command,self.config.integration_step_s)
            except GeometryFailure:pass
        self._sim_ns+=round(self.config.integration_step_s*1e9)
        stamp=ros_time(self._sim_ns)
        self._clock_pub.publish(Clock(clock=stamp))
        p=self.plant.pose
        self._record_task_entry()
        x,y=self.terrain.world_to_cell(p.x,p.y)
        odometry=odometry_message(p,self.plant.linear_mps,self.plant.angular_radps,
            self._sim_ns,self.terrain.heights[y,x])
        self._odom_pub.publish(odometry)
        tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='map';tf.child_frame_id='odom';tf.transform.rotation.w=1.
        self._tf_pub.publish(TFMessage(transforms=[tf]))
        if self._sim_ns-self._last_sensor_ns>=round(1e9/self.config.observation_hz):
            if self.sensor != self._fixed_sensor:raise InfrastructureError('sensor changed within scene')
            hits=SensorModel.observe(self.terrain,p,self.sensor)
            self.buffer.add(hits,self._sim_ns)
            self.core.record_observation(p,hits.indices)
            payload=self.buffer.payload()
            if payload is not None:
                origin,planes=payload
                self._map_pub.publish(grid_message(planes,origin,self.terrain.resolution_m,self._sim_ns))
            self._last_sensor_ns=self._latest_sensor_ns=self._sim_ns;self._sensor_count+=1
        snap=self.adapter.poll_snapshot(self._latest_sensor_ns,refresh=False)
        if snap is not None:
            self.buffer.acknowledge(self.adapter.processed_stamp_ns)
            self._apply_known(snap)
        if self._simulation_step_sync:
            def republish():
                self._clock_pub.publish(Clock(clock=stamp))
                self._odom_pub.publish(odometry)
                self._tf_pub.publish(TFMessage(transforms=[tf]))
            self._wait_mppi_step(republish)
        self._pace()

    def _record_task_entry(self):
        """Actual robot entry at physical integration ticks, separate from visibility."""
        if self._initial_inside_task or self._first_entry is not None:
            return
        from .approach_potential import distance_to_region
        pose=self.plant.pose
        if distance_to_region((pose.x,pose.y),self.task.polygon)==0.:
            self._first_entry=dict(decisions=self.steps+int(self._step_active),
                distance_m=self.plant.distance_m-self._approach_start_distance,
                simulation_s=self.plant.simulation_s,x_m=pose.x,y_m=pose.y)

    def _pace(self):
        # Total wall-step budget includes computation. Overload resets the
        # deadline without omitting physical steps or accumulating catch-up.
        self._next_tick+=self.config.integration_step_s/self.config.target_rtf
        now=time.monotonic()
        remaining=self._next_tick-now
        if remaining>0:time.sleep(remaining)
        else:self._next_tick=now

    @property
    def _known_area(self):return self.core.coverage.known_area_m2

    def _apply_known(self,snap):
        self.core.consume(snap)

    def _wait(self,predicate,reason,*,timeout=None):
        deadline=time.monotonic()+(self.wall_timeout_s if timeout is None else timeout)
        while True:
            result=predicate()
            if result:return result
            if time.monotonic()>deadline:raise InfrastructureError(f'wall watchdog: {reason}; map={self.adapter.map_reason}; '+self.owned.error_tail())
            self._pump()

    def _snapshot(self):
        minimum=self._latest_sensor_ns
        # A cached moving anchor cannot certify an action's final stopped pose.
        self.adapter.require_fresh_snapshot()
        snap=self._wait(lambda:self.adapter.poll_snapshot(minimum,refresh=False),'processed final policy map')
        self.buffer.acknowledge(self.adapter.processed_stamp_ns);self._apply_known(snap)
        return snap

    def _execute_goal(self,goal):
        self._navigation_attempt=None
        self._navigation_attempt=self.adapter.begin_goal(goal)
        from .navigation_budget import NavigationBudget, BudgetPolicy
        p=self.plant.pose
        budget=NavigationBudget(start_wall_s=time.monotonic(),
            start_simulation_s=self.plant.simulation_s,pose=(p.x,p.y,p.yaw),
            route_length_m=math.hypot(goal[0]-p.x,goal[1]-p.y),
            target_rtf=self.config.target_rtf,
            policy=BudgetPolicy(minimum_journey_s=self.wall_timeout_s))
        result=None
        while result is None:
            if self._cancel.is_set() or self.plant.collision:self.adapter.cancel_goal()
            result=self.adapter.poll_goal()
            if result is not None:break
            state=self.adapter.execution_state()
            p=self.plant.pose
            reason=budget.observe(wall_s=time.monotonic(),simulation_s=self.plant.simulation_s,
                pose=(p.x,p.y,p.yaw),accepted=state['accepted'],
                feedback_wall_s=state['feedback_wall_s'],route_length_m=state['route_length_m'],
                planning_reason=state['planning_reason'])
            self.last_navigation_budget=budget.record()
            if reason:
                self.adapter.cancel_goal()
                evidence=dict(reason=reason,goal=list(goal),actual_pose=[p.x,p.y,p.yaw],
                    simulation_s=self.plant.simulation_s,budget=self.last_navigation_budget,
                    navigation=state,owned_pids=list(self.owned.pids),stderr_tail=self.owned.error_tail())
                snapshot='unavailable'
                try:
                    folder=Path(self.config.artifact_root)/'navigation-watchdogs'
                    folder.mkdir(parents=True,exist_ok=True)
                    path=folder/f'env-{self.env_id}-{time.time_ns()}-{uuid.uuid4().hex[:8]}.json'
                    # ROS diagnostics may contain inf/nan; encode these as null.
                    def finite(value):
                        if isinstance(value,float) and not math.isfinite(value):return None
                        if isinstance(value,dict):return {k:finite(v) for k,v in value.items()}
                        if isinstance(value,(list,tuple)):return [finite(v) for v in value]
                        return value
                    path.write_text(json.dumps(finite(evidence),ensure_ascii=False,indent=2,allow_nan=False)+'\n')
                    snapshot=str(path)
                except (OSError,TypeError,ValueError) as exc:
                    snapshot=f'write failed: {exc}'
                raise InfrastructureError(f'navigation {reason}; elapsed={budget.record()["elapsed_wall_s"]:.1f}s; '
                    f'budget={budget.budget_s:.1f}s; route={budget.route_length_m:.1f}m; '
                    f'actual_rtf={budget.effective_rtf:.2f}; snapshot={snapshot}; '+self.owned.error_tail())
            self._pump()
        self.last_navigation_budget=budget.record()
        # Result completion can precede the controller's last braking command.
        self._wait(lambda:velocity_is_stopped((self.plant.linear_mps,self.plant.angular_radps)),
                   'cancel/goal stop',timeout=10.)
        if self._cancel.is_set():raise TaskCanceled('task canceled after native cancel-and-stop')
        if self.plant.collision:return NavigationResult(6,'COLLISION',result.last_segment_revision)
        return result

    def _privileged(self, observation=None):
        identity=self.core.coverage.identity
        if identity is None:raise InfrastructureError('coverage is unavailable')
        resolution,origin=identity[1:]
        if abs(resolution-self.terrain.resolution_m)>1e-12:
            raise InfrastructureError('native/reference resolution mismatch')
        offset=(np.asarray(self.terrain.origin[:2])-origin[:2])/resolution
        rounded=np.rint(offset).astype(int)
        if np.any(np.abs(offset-rounded)>1e-6):raise InfrastructureError('native/reference lattice mismatch')
        x,y=map(int,rounded);h,w=self.terrain.shape
        mask=self.core.coverage.mask((x,y,x+w,y+h))
        return self.privileged_builder.build(self.observation if observation is None else observation,
                                             self.reference.pack(mask), frontiers=self.core.frontiers)

    def _prepare_policy_state(self, native, *, snapshot=None):
        """Adapt the Actor graph before computing real-goal Critic descriptors."""
        started=time.perf_counter()
        mode=self.config.policy_graph_mode
        cap=self.config.remote_frontier_positions
        if self.config.model.architecture_version == 'candidate_geometry_v3':
            from .candidate_observation import prepare_candidate_observation
            if snapshot is None:
                raise ValueError('v5 input requires the frozen build snapshot')
            result=prepare_candidate_observation(native,snapshot.pose,contract=mode=='a2',
                max_remote_positions=cap if mode=='a2' else 0,
                metadata=dict(platform_profile_hash=snapshot.profile_hash,
                    map_resolution_m=float(snapshot.resolution_m),task_id=self.task.task_id,
                    sensor_mount=dict(offset_x_m=self.sensor.offset_x_m,
                        offset_y_m=self.sensor.offset_y_m,offset_yaw_rad=self.sensor.offset_yaw_rad)))
            policy,fine=result.observation,result.fine_observation
            diagnostics=result.diagnostics
            virtual_nodes=int(result.virtual_mask.sum())
        elif mode=='a2':
            from .policy_graph import prepare_policy_graph
            result=prepare_policy_graph(native,max_remote_positions=cap)
            policy,fine=result.observation,result.fine_observation
            diagnostics=result.diagnostics
            virtual_nodes=int(result.virtual_mask.sum())
        elif mode=='fine':
            policy=fine=native;diagnostics={};virtual_nodes=0
        else:
            raise ValueError('unknown policy graph mode')
        stats=dict(mode=mode,max_remote_positions=cap,
            native_nodes=len(native.node_ids),fine_nodes=len(fine.node_ids),
            coarse_nodes=len(policy.node_ids),native_edges=len(native.edges),
            fine_edges=len(fine.edges),coarse_edges=len(policy.edges),
            native_actions=len(native.goals),actions=len(policy.goals),virtual_nodes=virtual_nodes,
            preparation_wall_s=time.perf_counter()-started)
        return policy,fine,diagnostics,stats

    @property
    def completed(self):
        """Training/evaluation success; never enters observed Actor inputs."""
        state=getattr(self,'privileged',None)
        return bool(state is not None and not self.plant.collision and
            self.reference.coverage_ratio(state.observed)>=self.config.success_coverage)

    @property
    def terminated(self):
        report=getattr(self,'report',None)
        return self.completed or bool(report is not None and report.completed)

    def step(self,action_index,*,actor_version=None):
        if self._closed or self._step_active:raise RuntimeError('environment is closed or already stepping')
        if self.sensor != self._fixed_sensor:raise InfrastructureError('sensor changed within scene')
        if self.terminated or self._episode_truncated or self.steps>=self.episode_budget:raise RuntimeError('episode requires reset')
        if self.plant.collision:raise GeometryFailure('collision episode requires reset')
        if not 0<=int(action_index)<len(self.observation.goals):raise ValueError('action outside frozen observation')
        action_index=int(action_index);goal=tuple(self.observation.goals[action_index])
        before=(self._known_area,self.plant.distance_m,self.plant.turn_rad)
        before_frontiers=self.core.frontiers
        sim=self.plant.simulation_s;wall=time.monotonic();version=self.actor_version if actor_version is None else int(actor_version)
        if version<0:raise ValueError("actor_version must be nonnegative")
        observation,privileged=self.observation,self.privileged
        self._step_active=True
        self._navigation_attempt=None
        result=snap=report=None
        try:
            result=self._execute_goal(goal)
            snap=self._snapshot()
            next_native,report=self.core.observe(snap,self.adapter.velocity)
            if not report.available:raise InfrastructureError('unconstructable final state: '+report.reason_code)
            next_observation,next_fine,diagnostics,stats=self._prepare_policy_state(next_native,snapshot=snap)
            next_privileged=self._privileged(next_fine);self.steps+=1
            success=(not self.plant.collision and
                self.reference.coverage_ratio(next_privileged.observed)>=self.config.success_coverage)
            final=(self._known_area,self.plant.distance_m,self.plant.turn_rad)
            p=self.plant.pose
            stagnated=False
            if self.stagnation is not None:
                stagnated=self.stagnation.observe((p.x,p.y),final[0]-before[0])
            budget_hit=self.steps>=self.episode_budget
            after_coverage=self.reference.coverage_ratio(next_privileged.observed)
            if self._initial_reference_coverage == 0 and self._first_roi is None and after_coverage>0:
                self._first_roi=dict(decisions=self.steps,distance_m=self.plant.distance_m-self._approach_start_distance,
                    simulation_s=self.plant.simulation_s,coverage=after_coverage)
            approach_hit=bool(self._approach_limits and self._initial_reference_coverage==0
                and self._first_roi is None and (self.steps>=self._approach_limits['approach_decision_budget']
                    or self.plant.distance_m-self._approach_start_distance>=self._approach_limits['approach_distance_budget_m']))
            transition=make_transition(observation,action_index,next_observation,privileged,next_privileged,
                initial=before,final=final,
                removed_frontiers_in=count_removed(before_frontiers,self.core.frontiers,domain="inside"),
                removed_frontiers_out=count_removed(before_frontiers,self.core.frontiers,domain="outside"),
                completed=success,exhausted=report.completed,
                budget_hit=budget_hit or stagnated or approach_hit,
                episode_id=self.episode_id,actor_version=version,completion_bonus=self.config.completion_bonus,
                approach_reward=self.config.approach_reward,gamma=self.config.learning.gamma,
                exploration_reward_scale=self.config.exploration_reward_scale)
            self._episode_truncated=transition.truncated
            self.truncation_reason=('DECISION_BUDGET' if budget_hit else 'APPROACH_BUDGET' if approach_hit else 'STAGNATION') if transition.truncated else None
            self.last_execution=ExecutionMetadata(goal,result.outcome,result.reason_code,result.last_segment_revision,
                self.terrain.world_to_cell(*goal[:2]),self.terrain.world_to_cell(p.x,p.y),(p.x,p.y,p.yaw),
                final[1]-before[1],final[2]-before[2],self.plant.simulation_s-sim,time.monotonic()-wall,
                snap.revision,self.adapter.processed_stamp_ns,self.plant.collision)
            self.observation,self.report,self.privileged=next_observation,report,next_privileged
            self.native_observation=next_native;self.fine_observation=next_fine
            self.policy_graph_diagnostics=diagnostics;self.policy_graph_stats=stats
            return transition
        except InfrastructureError as exc:
            # Evidence collection is failure-only and must never hide the real
            # navigation error or fabricate an observation/transition.
            evidence='unavailable'
            try:
                from .navigation_failure import write_navigation_failure
                p=self.plant.pose
                attempt=self._navigation_attempt
                current=(attempt is not None and
                    getattr(self.adapter,'last_goal_execution_serial',None)==attempt)
                if result is None and current:
                    result=getattr(self.adapter,'last_goal_result',None)
                navigation=getattr(self.adapter,'last_goal_execution',None) if current else None
                if navigation is None and hasattr(self.adapter,'execution_state'):
                    navigation=self.adapter.execution_state()
                context=dict(error=str(exc),episode_id=self.episode_id,
                    episode_metadata=self.episode_metadata,decisions=self.steps,
                    action_index=action_index,actor_version=version,goal=list(goal),
                    navigation_attempt=attempt,
                    actual_pose=[p.x,p.y,p.yaw],simulation_s=self.plant.simulation_s,
                    distance_m=self.plant.distance_m,turn_rad=self.plant.turn_rad,
                    result=asdict(result) if result is not None else None,
                    report_reason=report.reason_code if report is not None else None,
                    processed_stamp_ns=self.adapter.processed_stamp_ns,
                    latest_sensor_stamp_ns=getattr(self,'_latest_sensor_ns',None),
                    map_reason=getattr(self.adapter,'map_reason',None),
                    snapshot_origin='stopped_current_response' if snap is not None else
                        'retained_cache_not_current',
                    navigation=navigation,
                    diagnostics=list(getattr(self.adapter,'diagnostics',())),
                    references=list(getattr(self.adapter,'references',())),
                    reference_history=list(getattr(self.adapter,'reference_history',())),
                    tracking_history=list(getattr(self.adapter,'tracking_history',())),
                    actual_pose_history=list(getattr(self.adapter,'pose_history',())),
                    budget=getattr(self,'last_navigation_budget',None),
                    stderr_tail=self.owned.error_tail())
                evidence=str(write_navigation_failure(self.config.artifact_root,
                    self.env_id,context,snapshot=snap if snap is not None else
                    getattr(self.adapter,'snapshot',None)))
            except Exception as evidence_error:
                evidence=f'evidence write failed: {evidence_error}'
            raise InfrastructureError(f'{exc}; failure_snapshot={evidence}') from exc
        finally:self._step_active=False

    def progress(self):
        """Cheap snapshot only: never advances TaskAnalyzer's reward baseline."""
        if self._closed:return {'state':'CLOSED','owned_pids':self.owned_pids}
        report=getattr(self,'report',None)
        return {'state':'EXECUTING' if self._step_active else 'READY','episode_id':self.episode_id,
            'steps':self.steps,'known_area_m2':self._known_area,'distance_m':self.plant.distance_m,
            'turn_rad':self.plant.turn_rad,'simulation_s':self.plant.simulation_s,
            'wall_s':time.monotonic()-self._start_wall,'sensor_frames':self._sensor_count,
            'max_command_speed_mps':self._max_command_speed,'owned_pids':self.owned_pids,
            'completed':None if report is None else self.completed,
            'terminated':None if report is None else self.terminated,
            'exhausted':None if report is None else report.exhausted,
            'truncation_reason':self.truncation_reason,
            'stagnation_enabled':self.stagnation is not None,
            'stagnation_zero_gain_steps':self.stagnation.zero_gain_steps if self.stagnation else 0,
            'stagnation_position_count':self.stagnation.position_count if self.stagnation else 0,
            'stagnation_triggered':self.stagnation.triggered if self.stagnation else False,
            'sensor':asdict(self.sensor),'scenario_id':self.episode_metadata.get('scenario_id'),
            'first_roi':self._first_roi,'initial_roi_visible':self._initial_reference_coverage>0,
            'first_entry':self._first_entry,'initial_inside_task':self._initial_inside_task,
            'policy_graph':dict(self.policy_graph_stats or {})}

    def cancel(self):
        """Thread-safe request; active step owns ROS cancel-and-stop pumping."""
        self._cancel.set()

    def _release_scene(self):
        # Replay/collector own their explicit immutable static-scene references.
        # A closed worker must not retain terrain or an old native/map graph.
        for name in ('terrain','plant','reference','core','buffer','observation','privileged','privileged_builder',
                     'native_observation','fine_observation','policy_graph_stats','policy_graph_diagnostics'):
            if hasattr(self,name):setattr(self,name,None)
        self.static_scenes={}

    def close(self):
        if self._closed:
            self._release_scene();return
        self._cancel.set()
        if self._step_active:raise RuntimeError('join active synchronous step after cancel before close')
        try:
            if self.adapter is not None and self.adapter.inflight:
                self.adapter.cancel_goal()
                try:self._wait(lambda:self.adapter.poll_goal(),'close cancellation',timeout=5.)
                except Exception:pass
        finally:
            self.owned.close()
            if self.executor is not None:self.executor.shutdown();self.executor=None
            if self._spin_thread is not None:self._spin_thread.join(timeout=3);self._spin_thread=None
            if self.adapter is not None:self.adapter.close();self.adapter=None
            if self.node is not None:self.node.destroy_node();self.node=None
            if self.context is not None and self.context.ok():self.context.shutdown()
            self.context=None;self._closed=True
            self._release_scene()
            if self._domain_lease is not None:
                self._domain_lease.close();self._domain_lease=None
