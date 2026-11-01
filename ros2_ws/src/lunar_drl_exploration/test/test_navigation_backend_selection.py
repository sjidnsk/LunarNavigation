"""后端选择与默认配置身份；不启动 ROS 或训练。"""
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from lunar_drl_exploration.config import TrainingConfig, config_record


def test_default_serialization_omits_optional_backend_and_explicit_roundtrips():
    from lunar_drl_exploration.config import training_config_from_record
    default = TrainingConfig()
    assert getattr(default, 'navigation_backend', None) == 'incremental_v2'
    assert 'navigation_backend' not in config_record(default)
    explicit = replace(default, navigation_backend='nav1cm')
    assert config_record(explicit)['navigation_backend'] == 'nav1cm'
    assert training_config_from_record(config_record(explicit)).navigation_backend == 'nav1cm'
    with pytest.raises(ValueError, match='navigation_backend'):
        replace(default, navigation_backend='typo')


def test_factory_defaults_legacy_and_rejects_unknown_before_ros():
    from lunar_drl_exploration.navigation_backend import adapter_class
    from lunar_drl_exploration.ros_messages import RosNavigationAdapter
    from lunar_drl_exploration.nav1cm_navigation import Nav1cmNavigationAdapter
    assert adapter_class() is RosNavigationAdapter
    assert adapter_class('nav1cm') is Nav1cmNavigationAdapter
    with pytest.raises(ValueError, match='navigation_backend'):
        adapter_class('typo')


def test_canonical_preparation_preserves_default_identity_and_v5_goal_order():
    from lunar_drl_exploration.policy_preparation import prepare_policy_state
    from lunar_drl_exploration.contracts import Pose, SensorSpec
    from test_candidate_observation import fine_observation
    native = fine_observation()
    snapshot = SimpleNamespace(pose=Pose(0., 0., 0.), profile_hash='actual-profile', resolution_m=.2)
    assert prepare_policy_state(native)[0] is native
    policy, fine, diagnostics, stats = prepare_policy_state(native, snapshot=snapshot,
        architecture_version='candidate_geometry_v3', task_id='synthetic', sensor=SensorSpec())
    assert policy.schema == 'task_graph_v5'
    np.testing.assert_array_equal(policy.goals, native.goals)
    np.testing.assert_array_equal(fine.features, native.features)
    assert policy.build_pose == snapshot.pose
    assert stats['native_actions'] == stats['actions'] == 1

def test_runtime_prepares_v5_before_calling_actor(monkeypatch):
    from lunar_drl_exploration import runtime as module
    from lunar_drl_exploration.contracts import Pose, SensorSpec
    from test_candidate_observation import fine_observation
    native=fine_observation()
    snapshot=SimpleNamespace(pose=Pose(0.,0.,0.),profile_hash='actual',resolution_m=.2,tiles={})
    class Core:
        def __init__(self,*args):self.task=args[0]
        def consume(self,snapshot):pass
        def observe(self,*args):return native,SimpleNamespace(available=True,completed=False)
    monkeypatch.setattr(module,'DecisionCore',Core)
    class Policy:
        actor=SimpleNamespace(config=SimpleNamespace(architecture_version='candidate_geometry_v3'))
        def __call__(self,obs):
            assert obs.schema=='task_graph_v5'
            np.testing.assert_array_equal(obs.goals,native.goals)
            return 0
    class Adapter:
        inflight=False;velocity=(0.,0.);pose=None
        def require_fresh_snapshot(self):pass
        def poll_snapshot(self,minimum_map_stamp_ns=0):
            assert minimum_map_stamp_ns==0
            return snapshot
        def begin_goal(self,goal):self.goal=goal;self.inflight=True
    a=Adapter();runtime=module.InferenceRuntime(a,Policy(),SensorSpec())
    runtime.start(SimpleNamespace(task_id='synthetic'));runtime.tick()
    assert a.inflight
    np.testing.assert_array_equal(a.goal,native.goals[0])


def test_nav1cm_owned_process_spec_uses_real_map_and_standard_path():
    from lunar_drl_exploration.navigation_backend import owned_nav1cm_specs
    specs=owned_nav1cm_specs('/lunar_training/task4',platform_config='/qualified/wheel.yaml',
        build_dir='/task/artifacts/cache',use_sim_time=True)
    assert [(p,e) for p,e,_ in specs]==[
        ('lunar_incremental_navigation_ros','lunar_exploration_map_node'),
        ('lunar_nav1cm_planner','navigation_task_node'),
        ('lunar_pure_wheeled_controller','lunar_pure_wheeled_controller_node.py')]
    mapper,planner,follower=[v for _,_,v in specs]
    assert mapper['platform_config']==planner['wheel_config']==follower['platform_config']=='/qualified/wheel.yaml'
    assert follower['input_mode']=='incremental_path'
    assert follower['controller_backend']=='mppi_path_follower'
    assert follower['standard_map_source']=='direct_p3'
    assert follower['mppi_nominal_response_time_s']==.15
    assert not follower['allow_unknown'] and not planner['allow_unknown']
    assert mapper['map_topic']==follower['p3_map_topic']==planner['map_topic']
    assert planner['local_path_topic']==follower['path_topic']


@pytest.mark.parametrize('backend',['incremental_v2','nav1cm'])
def test_attach_and_rosenv_share_factory_with_real_ros_clients(backend,monkeypatch,tmp_path):
    rclpy=pytest.importorskip('rclpy')
    from rclpy.context import Context
    from lunar_drl_exploration.runtime import InferenceRuntime
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    from lunar_drl_exploration import navigation_backend as module
    calls=[];original=module.create_navigation_adapter
    def factory(*args,**kwargs):
        calls.append(kwargs['navigation_backend'])
        return original(*args,**kwargs)
    monkeypatch.setattr(module,'create_navigation_adapter',factory)
    context=Context();rclpy.init(context=context,domain_id=92)
    node=rclpy.create_node('factory_attach_'+backend,context=context)
    runtime=InferenceRuntime.attach(node,lambda obs:0,navigation_backend=backend,
        task_topic='/lunar_training/factory/task',status_topic='/lunar_training/factory/status',
        action_name='/lunar_training/factory/navigate',policy_map_service='/lunar_training/factory/map',
        odometry_topic='/lunar_training/factory/odom',tf_topic='/lunar_training/factory/tf',
        tf_static_topic='/lunar_training/factory/tf_static')
    try:assert isinstance(runtime.adapter,module.adapter_class(backend))
    finally:runtime.adapter.close();node.destroy_node();context.shutdown()
    platform=TrainingConfig().platform
    if backend=='nav1cm':platform=replace(platform,capability=dict(platform.capability,minimum_clearance_m=.2))
    config=replace(TrainingConfig(),navigation_backend=backend,platform=platform,artifact_root=str(tmp_path))
    env=RosExplorationEnv(config,0,domain_base=92)
    starts=[]
    monkeypatch.setattr(env.owned,'start',lambda p,e,n,v,en:starts.append((p,e)))
    try:
        env._closed=False;env._start_ros()
        assert isinstance(env.adapter,module.adapter_class(backend))
        assert calls==[backend,backend]
        if backend=='nav1cm':
            assert starts[0][1]=='lunar_exploration_map_node'
            assert starts[1][1]=='navigation_task_node'
        else:assert starts[0][1]=='lunar_incremental_navigation_node'
    finally:env.close()


def test_inference_configuration_requires_readonly_matching_actor_metadata(tmp_path):
    from lunar_drl_exploration.inference_node import read_policy_settings
    from lunar_drl_exploration.config import ModelConfig, config_record
    import json
    model=ModelConfig(architecture_version='candidate_geometry_v3')
    settings=dict(model=config_record(model),sensor=config_record(TrainingConfig().sensor),
                  policy_graph_mode='a2',remote_frontier_positions=32)
    path=tmp_path/'config.json';path.write_text(json.dumps(settings))
    result=read_policy_settings(path,model)
    assert result['policy_graph_mode']=='a2' and result['remote_frontier_positions']==32
    assert path.read_text()==json.dumps(settings)
    settings['model']['actor_score_bound']=1.;path.write_text(json.dumps(settings))
    with pytest.raises(ValueError,match='model'):read_policy_settings(path,model)


def test_verifier_help_does_not_import_training_or_ros():
    import os, subprocess, sys
    from pathlib import Path
    script=Path(__file__).resolve().parents[4]/'scripts/drl/verify_nav1cm_backend.py'
    assert script.is_file()
    code='''import runpy,sys,importlib.abc
class Guard(importlib.abc.MetaPathFinder):
 def find_spec(self,name,path=None,target=None):
  if name.split('.')[0] in {'torch','rclpy'} or name.startswith('lunar_drl_exploration'):raise AssertionError(name)
sys.meta_path.insert(0,Guard());sys.argv=[sys.argv[1],'--help'];runpy.run_path(sys.argv[0],run_name='__main__')'''
    result=subprocess.run([sys.executable,'-c',code,str(script)],capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    assert '--declare-only' in result.stdout


def test_verifier_records_frozen_candidate_arrays_without_mutating_them(tmp_path):
    import importlib.util,json
    from pathlib import Path
    from test_candidate_observation import candidate_observation
    path=Path(__file__).resolve().parents[4]/'scripts/drl/verify_nav1cm_backend.py'
    spec=importlib.util.spec_from_file_location('verification',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    obs=candidate_observation(metadata={'task_id':'synthetic'})
    output=tmp_path/'observation.json';module.write_json(output,vars(obs))
    record=json.loads(output.read_text())
    assert record['schema']=='task_graph_v5' and record['goals']==[[1.,1.,0.]]
    assert not obs.goals.flags.writeable


def test_new_backend_rejects_legacy_clearance_profile_before_start():
    from lunar_drl_exploration.navigation_backend import validate_nav1cm_platform
    platform=TrainingConfig().platform
    wrong=replace(platform,capability=dict(platform.capability,minimum_clearance_m=.1))
    with pytest.raises(ValueError,match='0.2'):validate_nav1cm_platform(wrong)
    correct=replace(platform,capability=dict(platform.capability,minimum_clearance_m=.2))
    validate_nav1cm_platform(correct)


def test_optional_backend_cannot_resume_as_legacy_experience():
    from lunar_drl_exploration.config import resume_semantics
    default=TrainingConfig()
    assert 'navigation_backend' not in resume_semantics(default)
    explicit=replace(default,navigation_backend='nav1cm')
    assert resume_semantics(explicit)['navigation_backend']=='nav1cm'


@pytest.mark.parametrize('frame',['map','world'])
def test_task_callback_uses_configured_frame_and_rejects_wrong_start_without_mutation(frame):
    from lunar_drl_exploration.runtime import InferenceRuntime
    warnings=[]
    adapter=SimpleNamespace(map_frame=frame,inflight=False,
        require_fresh_snapshot=lambda:None,
        node=SimpleNamespace(get_logger=lambda:SimpleNamespace(warning=warnings.append)))
    runtime=InferenceRuntime(adapter,lambda observation:0)
    def message(task_id,task_frame):
        return SimpleNamespace(command=1,START=1,PAUSE=2,RESUME=3,CANCEL=4,
            task_id=task_id,header=SimpleNamespace(frame_id=task_frame),
            boundary=SimpleNamespace(points=[SimpleNamespace(x=x,y=y) for x,y in [(0,0),(2,0),(2,2),(0,2)]]))
    runtime.handle_task(message('valid',frame))
    original=runtime.core
    assert original.task.frame_id==frame and runtime.state=='RUNNING'
    adapter.inflight=True
    assert runtime.handle_task(message('bad','unexpected')) is False
    assert runtime.core is original and runtime.state=='RUNNING'
    assert runtime._pending_task is None and warnings
    assert original.task.task_id=='valid'
