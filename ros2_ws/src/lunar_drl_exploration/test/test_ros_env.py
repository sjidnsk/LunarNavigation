import math
import numpy as np
import pytest
from lunar_drl_exploration.contracts import Pose, SensorSpec
from lunar_drl_exploration.config import load_platform_config
from lunar_drl_exploration.scene import TerrainGrid
from lunar_drl_exploration.sensor import SensorModel
from lunar_drl_exploration.ros_messages import MeasurementBuffer, grid_message


def test_physical_entry_is_distinct_from_visibility_and_kept_after_exit():
    from types import SimpleNamespace
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    env=object.__new__(RosExplorationEnv)
    env._initial_inside_task=False;env._first_entry=None
    env._first_roi={'decisions':0}  # Seeing task cells is not physical entry.
    env._approach_start_distance=2.;env.steps=3;env._step_active=True
    env.task=SimpleNamespace(polygon=np.array([[0,0],[2,0],[2,2],[0,2]]))
    env.plant=SimpleNamespace(pose=Pose(-1,1,0),distance_m=5.,simulation_s=8.)
    env._record_task_entry()
    assert env._first_entry is None
    env.plant.pose=Pose(0,1,0)
    env._record_task_entry()
    first=dict(env._first_entry)
    assert first['decisions']==4 and first['distance_m']==3.
    env.plant.pose=Pose(-1,1,0);env._record_task_entry()
    assert env._first_entry==first


def test_coalesced_frames_retain_only_actual_hits_until_native_ack():
    t=TerrainGrid.from_heights(np.zeros((100,100),np.float32),.2,(-10.,-10.),load_platform_config())
    a=SensorModel.observe(t,Pose(.1,.1,0),SensorSpec(range_m=2))
    b=SensorModel.observe(t,Pose(.1,.1,math.pi),SensorSpec(range_m=2))
    buf=MeasurementBuffer(t.origin,t.resolution_m)
    buf.add(a,10);buf.add(b,20)
    origin,planes=buf.payload()
    assert np.isfinite(planes[0]).sum()==len(np.union1d(a.indices,b.indices))
    assert planes.shape[1]<100 and planes.shape[2]<100
    # Ack for older publication cannot erase newly included measurements.
    buf.acknowledge(10)
    _,remaining=buf.payload()
    assert np.isfinite(remaining[0]).sum()==len(b.indices)
    buf.acknowledge(20)
    assert buf.payload() is None


def test_grid_wire_preserves_center_lattice_and_nan_holes():
    import pytest
    pytest.importorskip('grid_map_msgs')
    planes=np.full((5,3,4),np.nan,np.float32);planes[:,1,2]=[2,.1,.2,.3,1]
    msg=grid_message(planes,(-.4,.8),.2,1234)
    assert msg.info.pose.position.x==0
    assert msg.info.pose.position.y==1.1
    assert np.array_equal(np.asarray(msg.data[0].data).reshape(3,4)[::-1,::-1],planes[0],equal_nan=True)
    assert msg.header.stamp.nanosec==1234
    assert [d.size for d in msg.data[0].layout.dim]==[3,4]
    assert [d.stride for d in msg.data[0].layout.dim]==[12,4]


def test_transition_whole_action_baseline_and_final_budget_state():
    from lunar_drl_exploration.ros_env import make_transition
    from lunar_drl_exploration.contracts import PrivilegedState
    from test_task_analysis import snapshot
    from lunar_drl_exploration.contracts import TaskSpec
    from lunar_drl_exploration.decision import DecisionCore
    # Reuse actual graph contracts; progress polling is irrelevant to reward.
    snap=snapshot(np.ones((9,9),np.uint8))
    task=TaskSpec('t','map',np.array([[0,0],[2,0],[2,2],[0,2]]))
    obs,report=DecisionCore(task,SensorSpec()).observe(snap)
    priv=PrivilegedState('scene',np.array([3],np.uint8))
    t=make_transition(obs,0,obs,priv,priv,initial=(1.,2.,3.),final=(5.,12.,3.+math.pi),
        removed_frontiers_in=2,removed_frontiers_out=0,
        completed=False,budget_hit=True,episode_id='e',actor_version=7)
    assert t.reward == .04-.02-.005-.001
    assert t.parts.distance_m==10
    assert t.truncated and not t.terminated
    assert t.next_observation is obs and t.actor_version==7


def test_native_no_path_and_timeout_with_final_state_remain_transitions(monkeypatch):
    from lunar_drl_exploration.ros_env import RosExplorationEnv,InfrastructureError
    from lunar_drl_exploration.ros_messages import NavigationResult
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.reference import CoverageReference
    from lunar_drl_exploration.decision import DecisionCore
    from lunar_drl_exploration.plant import KinematicPlant
    from test_task_analysis import snapshot,task
    from types import SimpleNamespace
    cfg=TrainingConfig();env=RosExplorationEnv(cfg,0)
    t=TerrainGrid.from_heights(np.zeros((12,12),np.float32),1.,(0.,0.),cfg.platform)
    pose=Pose(3.5,3.5,0);s=snapshot(np.pad(np.ones((6,6),np.uint8),3),pose=pose)
    env.terrain=t;env.task=task(0,0,12,12);env.core=DecisionCore(env.task,SensorSpec(range_m=3))
    env.reference=CoverageReference.build(t,pose,env.task,SensorSpec(range_m=3))
    from lunar_drl_exploration.privileged import build_truth, PrivilegedBuilder
    env.privileged_builder=PrivilegedBuilder(t,build_truth(t,env.reference),env.task,env.config.sensor)
    env.plant=KinematicPlant(t,pose,cfg.platform);env._apply_known(s)
    env.observation,env.report=env.core.observe(s);env.privileged=env._privileged()
    env._closed=False;env.steps=0;env.episode_id='failure-fixture'
    env.adapter=SimpleNamespace(velocity=(0.,0.),processed_stamp_ns=123)
    monkeypatch.setattr(env,'_snapshot',lambda:s)
    for outcome,reason in ((1,'NO_PATH'),(4,'TIMEOUT')):
        monkeypatch.setattr(env,'_execute_goal',lambda goal,o=outcome,r=reason:NavigationResult(o,r,0))
        before=env.observation
        result=env.step(0,actor_version=12)
        assert result.observation is before and result.next_observation.revision==s.revision
        assert result.reward==-.001 and not result.terminated and not result.truncated
        assert env.last_execution.reason_code==reason
    monkeypatch.setattr(env,'_snapshot',lambda:(_ for _ in ()).throw(InfrastructureError('lost map transport')))
    import pytest
    with pytest.raises(InfrastructureError,match='lost map'):env.step(0)
    # Completion is awarded by the same environment step, once only; native
    # goal failures above did not earn a bonus merely for ending an action.
    from dataclasses import replace
    env.config=replace(cfg,completion_bonus=2.)
    mostly_known=np.ones((12,12),np.uint8);mostly_known[5,9]=0
    full=replace(snapshot(np.pad(np.ones((6,6),np.uint8),3),
                          b=mostly_known,pose=pose),revision=2)
    monkeypatch.setattr(env,'_snapshot',lambda:full)
    monkeypatch.setattr(env,'_execute_goal',lambda goal:NavigationResult(0,'GOAL_REACHED',0))
    result=env.step(0)
    # Keep actual 107 m² diagnostic; exploration reward comes from removed fronts.
    assert result.parts.new_area_m2 == 107.
    assert result.terminated and result.reward == pytest.approx(result.parts.removed_frontiers_in/50 + 2. - .001)
    assert env.completed and not env.report.exhausted
    with pytest.raises(RuntimeError,match='requires reset'):env.step(0)


@pytest.mark.parametrize('completed,budget_hit,bonus,want,terminated,truncated', [
    (False, False, 5., .974, False, False),
    (False, True, 5., .974, False, True),
    (True, False, 5., 5.974, True, False),
    (True, True, 5., 5.974, True, False),
    (True, False, 2., 2.974, True, False),
    (True, False, 0., .974, True, False),
])
def test_completion_bonus_only_on_success_with_whole_action_costs(
        completed,budget_hit,bonus,want,terminated,truncated):
    from lunar_drl_exploration.ros_env import make_transition
    from test_model import observation,state
    obs=observation();priv=state(obs=obs)
    result=make_transition(obs,0,obs,priv,priv,initial=(0.,0.,0.),
        final=(100.,10.,math.pi),completed=completed,budget_hit=budget_hit,
        removed_frontiers_in=50,removed_frontiers_out=0,
        episode_id='completion',actor_version=0,completion_bonus=bonus)
    assert result.reward == pytest.approx(want)
    assert result.terminated == terminated and result.truncated == truncated
    assert result.parts.new_area_m2 == 100.


@pytest.mark.parametrize('count,exhausted,collision,success,terminal', [
    (98,False,False,False,False), (99,False,False,True,True),
    (100,False,False,True,True), (98,True,False,False,True),
    (99,True,False,True,True), (99,False,True,False,False),
])
def test_training_success_uses_reference_coverage_not_frontier_exhaustion(
        count,exhausted,collision,success,terminal):
    from types import SimpleNamespace
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    from lunar_drl_exploration.reference import CoverageReference
    from lunar_drl_exploration.config import TrainingConfig
    bits=np.packbits(np.arange(104)<count,bitorder='little')
    reference=np.packbits(np.arange(104)<100,bitorder='little')
    env=RosExplorationEnv(TrainingConfig(),0)
    env.reference=CoverageReference((1,104),reference,1.,100.,'reference',reference)
    env.privileged=SimpleNamespace(observed=bits)
    env.plant=SimpleNamespace(collision=collision)
    env.report=SimpleNamespace(completed=exhausted,exhausted=exhausted)
    assert env.completed is success
    assert env.terminated is terminal


def test_exhaustion_below_target_terminates_without_success_bonus():
    from lunar_drl_exploration.ros_env import make_transition
    from test_model import observation,state
    obs=observation();priv=state(obs=obs)
    result=make_transition(obs,0,obs,priv,priv,initial=(0.,0.,0.),final=(0.,10.,0.),
        removed_frontiers_in=0,removed_frontiers_out=0,
        completed=False,exhausted=True,budget_hit=True,episode_id='below-target',
        actor_version=0,completion_bonus=5.)
    assert result.terminated and not result.truncated
    assert result.reward==pytest.approx(-.021)


def test_exact_99_percent_at_point_two_meter_resolution_is_success():
    from types import SimpleNamespace
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.reference import CoverageReference
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    ref=np.packbits(np.ones(1700,np.uint8),bitorder='little')
    observed=np.packbits(np.arange(1700)<1683,bitorder='little')
    env=RosExplorationEnv(TrainingConfig(),0)
    env.reference=CoverageReference((1,1700),ref,.2**2,1700*.2**2,'exact99',ref)
    env.privileged=SimpleNamespace(observed=observed)
    env.plant=SimpleNamespace(collision=False)
    env.report=SimpleNamespace(completed=False,exhausted=False)
    assert env.reference.coverage_ratio(observed)==.99
    assert env.completed and env.terminated


def test_deployment_history_pose_uses_actual_map_from_odom_transform():
    import pytest
    pytest.importorskip('geometry_msgs')
    from geometry_msgs.msg import TransformStamped
    from lunar_drl_exploration.ros_messages import odometry_message,pose_in_map
    msg=odometry_message(Pose(1.,2.,.25),.1,.2,100)
    tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom'
    tf.transform.translation.x=10.;tf.transform.translation.y=-3.
    tf.transform.rotation.w=math.cos(math.pi/4);tf.transform.rotation.z=math.sin(math.pi/4)
    pose=pose_in_map(msg,tf)
    assert pose.x==pytest.approx(8.) and pose.y==pytest.approx(-2.)
    assert pose.yaw==pytest.approx(.25+math.pi/2)


def test_pacing_counts_computation_and_never_catches_up_overdue_steps(monkeypatch):
    from types import SimpleNamespace
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    import lunar_drl_exploration.ros_env as module
    clock=SimpleNamespace(now=0.,sleeps=[])
    monkeypatch.setattr(module.time,'monotonic',lambda:clock.now)
    def sleep(duration):clock.sleeps.append(duration);clock.now+=duration
    monkeypatch.setattr(module.time,'sleep',sleep)
    env=RosExplorationEnv.__new__(RosExplorationEnv)
    env.config=SimpleNamespace(integration_step_s=.05,target_rtf=30.)
    env._next_tick=0.
    clock.now=.0007;env._pace()
    assert clock.now==pytest.approx(.05/30)
    assert clock.sleeps==pytest.approx([.05/30-.0007])
    clock.now+=.003;env._pace()
    assert len(clock.sleeps)==1  # Overloaded computation gets no added sleep.
    assert env._next_tick==clock.now
    clock.now+=.0007;env._pace()
    assert clock.sleeps[-1]==pytest.approx(.05/30-.0007)


@pytest.mark.parametrize('ready_after,exhausted', [(2, False), (0, True), (5, False)])
def test_initialization_requires_measured_report_within_existing_four_scans(monkeypatch, ready_after, exhausted):
    from dataclasses import replace
    from types import SimpleNamespace
    from lunar_drl_exploration.ros_env import RosExplorationEnv, InputUnavailable
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.decision import DecisionCore
    from lunar_drl_exploration.contracts import Pose
    from test_task_analysis import snapshot, task
    config=TrainingConfig(); env=RosExplorationEnv(config,0)
    terrain=TerrainGrid.from_heights(np.zeros((12,12),np.float32),1.,(0.,0.),config.platform)
    pose=Pose(3.5,3.5,0.); snap=snapshot(terrain.navigation,pose=pose)
    assert len(snap.start_connections)>0
    goals=[]; real_observe=DecisionCore.observe
    def observe(core,*args):
        obs,report=real_observe(core,*args)
        return obs,replace(report,available=len(goals)>=ready_after,exhausted=exhausted,
            reason_code='READY' if len(goals)>=ready_after else 'INPUT_UNAVAILABLE')
    monkeypatch.setattr(DecisionCore,'observe',observe)
    monkeypatch.setattr(env,'close',lambda:None)
    monkeypatch.setattr(env,'_start_ros',lambda:setattr(env,'adapter',SimpleNamespace(ready=True,velocity=(0.,0.))))
    monkeypatch.setattr(env,'_wait',lambda *a,**k:None)
    monkeypatch.setattr(env,'_snapshot',lambda:snap)
    def execute(goal):
        goals.append(goal); env.plant.turn_rad+=math.pi/2; env.plant.simulation_s+=1
        return SimpleNamespace(outcome=0,reason_code='GOAL_REACHED')
    monkeypatch.setattr(env,'_execute_goal',execute)
    if ready_after>4:
        with pytest.raises(InputUnavailable,match='INPUT_UNAVAILABLE'):
            env.reset_scene(terrain,pose,task(0,0,12,12))
        assert len(goals)==4
    else:
        env.reset_scene(terrain,pose,task(0,0,12,12))
        assert len(goals)==ready_after and env.report.available and env.report.exhausted==exhausted
        assert env.episode_metadata['initialization_turn_rad']==pytest.approx(ready_after*math.pi/2)
    assert all(g[:2]==(pose.x,pose.y) for g in goals)
    assert [g[2] for g in goals]==[pose.yaw+(i+1)*math.pi/2 for i in range(len(goals))]


def stagnation_env(monkeypatch,*,training=True,window=4,positions=2,budget=512):
    """Exercise real reset, graph, coverage and step; replace only native I/O."""
    from dataclasses import replace
    from types import SimpleNamespace
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    from lunar_drl_exploration.ros_messages import NavigationResult
    from test_task_analysis import snapshot,task
    config=replace(TrainingConfig(),stagnation_window_steps=window,
        stagnation_max_positions=positions,stagnation_position_tolerance_m=.25)
    env=RosExplorationEnv(config,0,training=training)
    terrain=TerrainGrid.from_heights(np.zeros((12,12),np.float32),1.,(0.,0.),config.platform)
    pose=Pose(3.5,3.5,0.)
    state={'snapshot':snapshot(np.pad(np.ones((6,6),np.uint8),3),pose=pose)}
    monkeypatch.setattr(env,'close',lambda:None)
    monkeypatch.setattr(env,'_start_ros',lambda:setattr(env,'adapter',
        SimpleNamespace(ready=True,velocity=(0.,0.),processed_stamp_ns=123)))
    monkeypatch.setattr(env,'_wait',lambda predicate,*a,**k:predicate())
    monkeypatch.setattr(env,'_snapshot',lambda:state['snapshot'])
    monkeypatch.setattr(env,'_execute_goal',lambda goal:NavigationResult(0,'GOAL_REACHED',0))
    env.episode_budget=budget
    env.reset_scene(terrain,pose,task(0,0,12,12))
    return env,state


def test_training_stagnation_preserves_real_transition_reward_and_stops_step(monkeypatch):
    env,_=stagnation_env(monkeypatch)
    for _ in range(3): assert not env.step(0).truncated
    last=env.step(0)
    assert last.truncated and not last.terminated and last.reward==-.001
    assert last.next_observation is env.observation
    assert env.last_execution.reason_code=='GOAL_REACHED'
    progress=env.progress()
    assert progress['truncation_reason']=='STAGNATION'
    assert progress['stagnation_zero_gain_steps']==4 and progress['stagnation_triggered']
    assert not env.completed and not env.terminated
    with pytest.raises(RuntimeError,match='requires reset'): env.step(0)


@pytest.mark.parametrize('training,window', [(False,4),(True,0)])
def test_evaluation_and_disabled_training_keep_the_full_decision_budget(monkeypatch,training,window):
    env,_=stagnation_env(monkeypatch,training=training,window=window,budget=6)
    for _ in range(5): assert not env.step(0).truncated
    assert env.step(0).truncated
    assert env.progress()['truncation_reason']=='DECISION_BUDGET'
    assert not env.progress()['stagnation_triggered']


def test_environment_gain_and_scene_reset_clear_stagnation(monkeypatch):
    from dataclasses import replace
    from test_task_analysis import snapshot
    env,state=stagnation_env(monkeypatch)
    for _ in range(3): assert not env.step(0).truncated
    known=np.pad(np.ones((6,6),np.uint8),3);known[2,3]=1
    state['snapshot']=replace(snapshot(np.pad(np.ones((6,6),np.uint8),3),
        b=known,pose=env.plant.pose),revision=2)
    gained=env.step(0)
    assert gained.parts.new_area_m2==1. and not gained.truncated
    assert gained.parts.removed_frontiers_in==0 and gained.reward==-.001
    assert env.progress()['stagnation_zero_gain_steps']==0
    for _ in range(3): assert not env.step(0).truncated
    assert env.step(0).truncated
    env.reset_scene(env.terrain,env.plant.pose,env.task)
    assert env.progress()['truncation_reason'] is None
    assert env.progress()['stagnation_zero_gain_steps']==0
    assert not env.step(0).truncated


@pytest.mark.parametrize('terminal', ['success','exhausted'])
def test_real_terminal_precedes_stagnation_and_retains_completion_bonus(monkeypatch,terminal):
    from dataclasses import replace
    from test_task_analysis import snapshot
    env,state=stagnation_env(monkeypatch,window=4)
    for _ in range(3): env.step(0)
    if terminal=='success':
        # The fourth action reaches true 99%; the terminal transition keeps its
        # ordinary frontier reward and receives the existing one-time bonus.
        known=np.ones((12,12),np.uint8);known[5,9]=0
        state['snapshot']=replace(snapshot(np.pad(np.ones((6,6),np.uint8),3),
            b=known,pose=env.plant.pose),revision=2)
    else:
        observe=env.core.observe
        def exhausted(*args):
            obs,report=observe(*args)
            return obs,replace(report,exhausted=True)
        monkeypatch.setattr(env.core,'observe',exhausted)
    result=env.step(0)
    assert result.terminated and not result.truncated
    assert env.progress()['truncation_reason'] is None
    assert result.reward==pytest.approx(result.parts.removed_frontiers_in/50 +
        (5. if terminal=='success' else 0.) - .001)
    if terminal=='success': assert env.progress()['stagnation_zero_gain_steps']==0
    with pytest.raises(RuntimeError,match='requires reset'): env.step(0)


def test_stagnation_uses_executed_xy_even_when_requested_targets_keep_changing(monkeypatch):
    env,_=stagnation_env(monkeypatch,positions=1)
    _,actions=np.unique(env.observation.goals[:,:2],axis=0,return_index=True)
    assert len(actions)>=2
    for action in [actions[0],actions[1],actions[0]]: assert not env.step(action).truncated
    assert env.step(actions[1]).truncated
    assert env.progress()['stagnation_position_count']==1
