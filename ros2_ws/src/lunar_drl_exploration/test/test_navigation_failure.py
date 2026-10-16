import json
from types import SimpleNamespace as NS

import numpy as np
import pytest


def test_failure_snapshot_keeps_policy_map_without_promoting_unknown(tmp_path):
    from lunar_drl_exploration.navigation_failure import write_navigation_failure
    from test_task_analysis import snapshot
    snap = snapshot(np.pad(np.ones((4, 4), np.uint8), 2))
    path = write_navigation_failure(tmp_path, 3,
        dict(goal=[1., 2., 0.], actual_pose=[.1, .2, .3], invalid=float('nan')),
        snapshot=snap)
    record = json.loads(path.read_text())
    assert record['context']['invalid'] is None
    assert record['snapshot']['start_connection_status'] == snap.start_connection_status
    assert record['snapshot']['revision'] == snap.revision
    with np.load(path.with_suffix('.npz'), allow_pickle=False) as arrays:
        assert np.array_equal(arrays['start_connections'], snap.start_connections)
        for i, tile in enumerate(snap.tiles.values()):
            for field in ('states', 'observed', 'intrinsic_states', 'costs', 'elevation_m'):
                assert np.array_equal(arrays[f'tile_{i}_{field}'], getattr(tile, field), equal_nan=True)


def unavailable_env(monkeypatch, tmp_path):
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.contracts import Pose
    from lunar_drl_exploration.ros_messages import NavigationResult
    from test_task_analysis import snapshot
    env = RosExplorationEnv(TrainingConfig(artifact_root=str(tmp_path)), 3)
    env._closed = False
    env.steps = 0
    env.episode_id = 'failed-stopped-pose'
    env.report = NS(completed=False)
    env.privileged = NS(observed=None)
    env.reference = NS(coverage_ratio=lambda _: 0.)
    env.observation = NS(goals=np.array([[1., 2., 0.]]))
    env.plant = NS(pose=Pose(.1, .2, .3), collision=False,
                   distance_m=.7, turn_rad=.1, simulation_s=7.)
    snap = snapshot(np.pad(np.ones((4, 4), np.uint8), 2))
    env.core = NS(coverage=NS(known_area_m2=4.), frontiers=None,
                  observe=lambda *_: (None, NS(available=False, reason_code='INPUT_UNAVAILABLE')))
    env.adapter = NS(velocity=(0., 0.), processed_stamp_ns=10, snapshot=snap,
                     pose_history=[(10, Pose(.1, .2, .3))],
                     last_goal_execution={'tracking': {'reason': 'CURVATURE_INFEASIBLE'}})
    monkeypatch.setattr(env, '_execute_goal', lambda _: NavigationResult(1, 'START_BLIND_ZONE_UNRESOLVED', 2))
    monkeypatch.setattr(env, '_snapshot', lambda: snap)
    return env


def test_unavailable_state_records_executed_goal_and_does_not_fake_transition(monkeypatch, tmp_path):
    from lunar_drl_exploration.ros_env import InfrastructureError
    env = unavailable_env(monkeypatch, tmp_path)
    old = env.observation
    with pytest.raises(InfrastructureError, match='unconstructable final state: INPUT_UNAVAILABLE') as caught:
        env.step(0)
    assert 'failure_snapshot=' in str(caught.value)
    records = list((tmp_path / 'navigation-failures').glob('*.json'))
    assert len(records) == 1
    d = json.loads(records[0].read_text())
    assert d['context']['result']['reason_code'] == 'START_BLIND_ZONE_UNRESOLVED'
    assert d['context']['actual_pose'] == [.1, .2, .3]
    assert d['context']['goal'] == [1., 2., 0.]
    assert d['context']['actual_pose_history'][0][1]['x'] == .1
    assert d['context']['navigation']['tracking']['reason'] == 'CURVATURE_INFEASIBLE'
    assert env.steps == 0 and env.observation is old and not env._step_active


def test_failure_evidence_error_cannot_hide_original_navigation_error(monkeypatch, tmp_path):
    from lunar_drl_exploration.ros_env import InfrastructureError
    from lunar_drl_exploration import navigation_failure
    env = unavailable_env(monkeypatch, tmp_path)
    def broken(*args, **kwargs):
        raise OSError('disk error')
    monkeypatch.setattr(navigation_failure, 'write_navigation_failure', broken)
    with pytest.raises(InfrastructureError, match='INPUT_UNAVAILABLE') as caught:
        env.step(0)
    assert 'evidence write failed' in str(caught.value)
    assert env.steps == 0


def test_adapter_retains_execution_context_after_result(monkeypatch):
    from test_navigation_budget import adapter_fixture
    a = adapter_fixture(monkeypatch)
    a._cancel = False
    a._execution_tracking = dict(session_id='01' * 16, reason='PATH_DEVIATED', received_wall_s=4.)
    a._result_future = NS(done=lambda: True, result=lambda: NS(
        result=NS(outcome=1, reason_code='NO_PATH', last_segment_revision=2)))
    result = a.poll_goal()
    assert result.reason_code == 'NO_PATH' and a._handle is None
    assert a.last_goal_execution['session_id'] == '01' * 16
    assert a.last_goal_execution['tracking']['reason'] == 'PATH_DEVIATED'


def test_failed_begin_does_not_reuse_previous_goal_execution(monkeypatch, tmp_path):
    from lunar_drl_exploration.ros_env import InfrastructureError
    env = unavailable_env(monkeypatch, tmp_path)
    env.adapter.execution_state = lambda: {'session_id': None, 'tracking': None}
    def unavailable(_):
        raise InfrastructureError('navigation action unavailable')
    monkeypatch.setattr(env, '_execute_goal', unavailable)
    with pytest.raises(InfrastructureError, match='navigation action unavailable'):
        env.step(0)
    record = json.loads(next((tmp_path / 'navigation-failures').glob('*.json')).read_text())
    assert record['context']['result'] is None
    assert record['context']['navigation']['tracking'] is None
    assert record['context']['snapshot_origin'] == 'retained_cache_not_current'
