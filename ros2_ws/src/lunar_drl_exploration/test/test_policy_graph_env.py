"""A2 collector alignment without running native graph or ROS navigation."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from lunar_drl_exploration.contracts import (
    Pose, PrivilegedActionContext, PrivilegedState, TaskSpec)
from lunar_drl_exploration.config import TrainingConfig
from lunar_drl_exploration.ros_env import RosExplorationEnv
from test_policy_graph import observation


class CandidateBuilder:
    """Boundary double: expose each exact real goal to the resulting Critic state."""
    def __init__(self, *args):
        self.observations = []

    def build(self, obs, bits, *, frontiers):
        self.observations.append(obs)
        positions, action_positions = np.unique(obs.goals[:, :2], axis=0, return_inverse=True)
        n = len(positions)
        actions = PrivilegedActionContext(positions, action_positions, obs.goals[:, 2],
            np.arange(n + 1), np.zeros(n, np.int64), np.zeros(n),
            np.arange(len(obs.goals), dtype=np.int64))
        return PrivilegedState('dummy-scene', bits, actions)


def fixture(monkeypatch, mode='a2', cap=1, scans=1):
    # Only the external/native boundaries are replaced. reset_scene, step,
    # _privileged and the complete A2 adapter run their production code.
    import lunar_drl_exploration.ros_env as module
    import lunar_drl_exploration.privileged as privileged_module
    from lunar_drl_exploration.reference import CoverageReference
    cfg = SimpleNamespace(**TrainingConfig().__dict__)
    cfg.policy_graph_mode = mode
    cfg.remote_frontier_positions = cap
    env = RosExplorationEnv(cfg, 0)
    terrain = SimpleNamespace(origin=(0., 0.), resolution_m=1., shape=(1, 8),
        world_to_cell=lambda x, y: (int(x), int(y)))
    task = TaskSpec('dummy-task', 'map', [[0, 0], [8, 0], [8, 2], [0, 2]])
    first = observation([(i, 0) for i in range(7)],
        [(i, i + 1) for i in range(6)], gains=[0, 6], actions=[0], yaws=[.25])
    second = replace(observation([(i, 0) for i in range(8)],
        [(i, i + 1) for i in range(7)], gains=[0, 7], actions=[0], yaws=[.5]), revision=8)
    bits = np.asarray([1], np.uint8)
    reference = SimpleNamespace(area_m2=8., pack=lambda mask: bits,
        coverage_ratio=lambda observed: .125)
    monkeypatch.setattr(CoverageReference, 'build', lambda *args: reference)
    static = SimpleNamespace(scene_id='dummy-scene')
    monkeypatch.setattr(privileged_module, 'build_truth', lambda *args: static)
    monkeypatch.setattr(privileged_module, 'PrivilegedBuilder', CandidateBuilder)
    plant = SimpleNamespace(pose=Pose(0., 0., 0.), distance_m=0., turn_rad=0.,
        simulation_s=0., collision=False)
    monkeypatch.setattr(module, 'KinematicPlant', lambda *args: plant)
    state = SimpleNamespace(calls=0, stepping=False, prepare_calls=[])
    coverage = SimpleNamespace(identity=('dummy', 1., np.array([0., 0.])), known_area_m2=1.,
        mask=lambda bounds: np.array([[True] + [False] * 7]))
    class Core:
        def __init__(self, *args):
            self.coverage = coverage
            self.frontiers = None
            self.history = SimpleNamespace(tolerance_m=0.)

        def observe(self, *args):
            state.calls += 1
            ready = state.stepping or state.calls > scans
            report = SimpleNamespace(available=ready, completed=False, exhausted=False,
                reason_code='READY' if ready else 'WAITING')
            return (second if state.stepping else first), report
    monkeypatch.setattr(module, 'DecisionCore', Core)
    monkeypatch.setattr(module, 'count_removed', lambda *args, **kwargs: 0)
    # Keep lifecycle fields complete so production close releases all views.
    def start():
        env.adapter = SimpleNamespace(ready=True, velocity=(0., 0.),
            processed_stamp_ns=123, inflight=False, close=lambda: None)
    monkeypatch.setattr(env, '_start_ros', start)
    monkeypatch.setattr(env, '_wait', lambda predicate, *args, **kwargs: predicate())
    def snap():
        return SimpleNamespace(start_connections=(1,), revision=8 if state.stepping else 7)
    monkeypatch.setattr(env, '_snapshot', snap)
    executed = []
    def execute(goal):
        executed.append(goal)
        plant.turn_rad += .1
        return SimpleNamespace(outcome=0, reason_code='GOAL_REACHED', last_segment_revision=1)
    monkeypatch.setattr(env, '_execute_goal', execute)
    import lunar_drl_exploration.policy_graph as graph
    prepare = graph.prepare_policy_graph
    def traced_prepare(obs, max_remote_positions=32):
        state.prepare_calls.append(obs)
        return prepare(obs, max_remote_positions)
    monkeypatch.setattr(graph, 'prepare_policy_graph', traced_prepare)
    return env, terrain, task, first, second, state, executed


def assert_candidate_alignment(obs, privileged):
    actions = privileged.actions
    np.testing.assert_array_equal(actions.positions[actions.action_positions], obs.goals[:, :2])
    np.testing.assert_array_equal(actions.action_yaws, obs.goals[:, 2])
    assert len(actions.removed_frontiers) == len(obs.goals)


def test_reset_scans_fine_then_prepares_once_before_one_privileged_build(monkeypatch):
    # Preparing before the initialization scan, or privileged from coarse,
    # loses original fine action identity captured at the decision boundary.
    env, terrain, task, native, _, state, executed = fixture(monkeypatch, scans=2)
    obs, privileged = env.reset_scene(terrain, Pose(0., 0., 0.), task)
    assert state.calls == 3 and len(executed) == 2
    assert state.prepare_calls == [native]
    assert env.native_observation is native
    assert env.fine_observation is env.privileged_builder.observations[0]
    assert len(env.privileged_builder.observations) == 1
    assert len(obs.node_ids) == 3 and len(env.fine_observation.node_ids) == 7
    np.testing.assert_array_equal(obs.goals, env.fine_observation.goals)
    assert_candidate_alignment(obs, privileged)
    stats = env.progress()['policy_graph']
    assert stats['mode'] == 'a2' and stats['native_nodes'] == 7
    assert stats['coarse_nodes'] == 3 and stats['virtual_nodes'] == 1
    assert stats['native_actions'] == 1 and stats['actions'] == 9
    stats['coarse_nodes'] = 999
    assert env.progress()['policy_graph']['coarse_nodes'] == 3


def test_step_stores_coarse_current_and_successor_with_one_expanded_privileged_build(monkeypatch):
    # Transition action indices must refer to the expanded real goals of both
    # states; using the native prefix state for either Critic is a bug.
    env, terrain, task, native, successor, state, executed = fixture(monkeypatch)
    before, before_privileged = env.reset_scene(terrain, Pose(0., 0., 0.), task)
    expected_goal = tuple(before.goals[-1])
    state.stepping = True
    result = env.step(len(before.goals) - 1, actor_version=5)
    assert executed[-1] == expected_goal
    assert result.observation is before and result.privileged is before_privileged
    assert result.next_observation is env.observation
    assert result.next_privileged is env.privileged
    assert env.native_observation is successor
    assert state.prepare_calls == [native, successor]
    assert len(env.privileged_builder.observations) == 2
    assert env.privileged_builder.observations[-1] is env.fine_observation
    assert len(env.observation.node_ids) == 3 and len(env.fine_observation.node_ids) == 8
    assert_candidate_alignment(result.observation, result.privileged)
    assert_candidate_alignment(result.next_observation, result.next_privileged)
    assert result.reward == pytest.approx(-.001 - .005 * .1 / np.pi)
    assert not result.terminated and not result.truncated and result.actor_version == 5
    env.close()
    for name in ('observation', 'native_observation', 'fine_observation',
                 'policy_graph_stats', 'policy_graph_diagnostics'):
        assert getattr(env, name) is None


def test_fine_mode_retains_original_actions_without_running_a2(monkeypatch):
    env, terrain, task, native, successor, state, _ = fixture(monkeypatch, mode='fine')
    obs, privileged = env.reset_scene(terrain, Pose(0., 0., 0.), task)
    assert obs is native and env.fine_observation is native
    assert env.privileged_builder.observations == [native]
    assert_candidate_alignment(obs, privileged)
    state.stepping = True
    result = env.step(0)
    assert result.next_observation is successor and env.fine_observation is successor
    assert env.privileged_builder.observations == [native, successor]
    assert state.prepare_calls == []
    assert env.progress()['policy_graph']['virtual_nodes'] == 0
