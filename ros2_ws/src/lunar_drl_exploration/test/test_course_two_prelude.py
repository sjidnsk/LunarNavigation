from types import SimpleNamespace
import numpy as np
import pytest


def test_measured_prefix_is_executed_and_policy_boundary_uses_actual_state():
    from lunar_drl_exploration.course_two_prelude import prepare_start
    from lunar_drl_exploration.contracts import Pose
    class Env:
        def __init__(self):
            self.plant=SimpleNamespace(pose=Pose(0.,0.,0.),distance_m=0.,turn_rad=0.,simulation_s=0.,collision=False)
            self.episode_metadata={};self.steps=0;self.calls=[];self.coverage=.94
            self.reference=SimpleNamespace(coverage_ratio=lambda _:self.coverage)
            self.privileged=SimpleNamespace(observed=b'actual')
            from lunar_drl_exploration.contracts import SensorSpec
            from worker_fixtures import observation
            self.sensor=SensorSpec();self.task=SimpleNamespace(polygon=np.zeros((4,2)))
            self.observation=observation();self.fine_observation=self.observation
        def prelude_goal(self,goal):
            self.calls.append(goal);self.plant.distance_m+=2;self.coverage=.96
            return dict(outcome=0,actual_pose=[1.,0.,0.])
        def set_policy_start(self,record):self.episode_metadata['prelude']=record
    env=Env();case={'role':'finish','prelude_goals':[[1.,0.,0.]],'max_prelude_decisions':512}
    record=prepare_start(env,case,inside=lambda:True)
    assert len(env.calls)==1 and record['decisions']==1
    assert record['coverage']==.96 and record['distance_m']==2
    assert env.privileged.observed==b'actual'
    assert env.episode_metadata['prelude']==record
    assert len(record['handover_hash'])==64 and record['executions'][0]['outcome']==0
    env.coverage=1.
    env.prelude_goal=lambda goal:None
    with pytest.raises(ValueError,match='policy start mismatch'):prepare_start(env,case,inside=lambda:True)


def test_prefix_caps_are_checked_before_execution():
    from lunar_drl_exploration.course_two_prelude import prepare_start
    with pytest.raises(ValueError,match='prelude limit'):
        prepare_start(None,{'role':'finish','max_prelude_decisions':1,'prelude_goals':[[0,0,0],[1,1,1]]})


def test_long_prefix_has_explicit_reset_deadline_without_weakening_action_watchdog():
    from lunar_drl_exploration.collector import worker_timeout_for
    spec=dict(course_two_case=dict(version='region_sensor_rebalanced_scene_v2',
        prelude_goals=[[1,2,3]],qualification_request={'seed':1}))
    assert worker_timeout_for(dict(mode='RESETTING',reset_spec=spec),180)==2460
    assert worker_timeout_for(dict(mode='ACTIVE',reset_spec=spec),180)==180
    del spec['course_two_case']['qualification_request']
    assert worker_timeout_for(dict(mode='RESETTING',reset_spec=spec),180)==1260
    assert worker_timeout_for(dict(mode='RESETTING',reset_spec={}),180)==180


def test_handover_hash_changes_with_observations_pose_and_history():
    from lunar_drl_exploration.course_two_prelude import handover_hash
    from lunar_drl_exploration.contracts import Pose,SensorSpec
    from worker_fixtures import observation
    env=SimpleNamespace(plant=SimpleNamespace(pose=Pose(0,0,0)),sensor=SensorSpec(),
        privileged=SimpleNamespace(observed=np.array([1],np.uint8)),task=SimpleNamespace(polygon=np.zeros((4,2))),
        observation=observation(),fine_observation=observation())
    first=handover_hash(env);assert first==handover_hash(env)
    env.privileged.observed=np.array([3],np.uint8);assert first!=handover_hash(env)
    second=handover_hash(env);env.plant.pose=Pose(0,.1,0);assert second!=handover_hash(env)
