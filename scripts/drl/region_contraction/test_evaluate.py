"""Adapter tests use real immutable observations and a transport-free native stand-in."""
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "ros2_ws/src/lunar_drl_exploration"))
from lunar_drl_exploration.contracts import DecisionObservation, PrivilegedState, RewardParts, Transition


def evaluator():
    path = Path(__file__).with_name("evaluate.py")
    assert path.is_file(), "frozen evaluation adapter is not implemented"
    name = "region_contraction_evaluate_under_test"
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


def observation():
    positions = np.array([[0, 0], [1, 0], [2, 0], [3, 0], [4, 0]], np.float32)
    features = np.zeros((5, 27), np.float32)
    features[:, 2] = 1
    features[4, 3] = 1
    return DecisionObservation(np.arange(10, 15), positions, features,
        np.array([[0, 1], [1, 2], [2, 3], [3, 4]]), np.ones(4), 0,
        np.array([[-1, -1], [5, -1], [5, 1], [-1, 1]]), np.zeros(8),
        np.array([0, 1]), np.array([0., .785]),
        np.array([[0., 0., 0.], [1., 0., .785]]), "fixture", 17)


def contraction(obs):
    rows = np.array([0, 1, 2, 4])
    mapping = np.array([0, 1, 2, 2, 3])
    reduced = replace(obs, node_ids=obs.node_ids[rows], positions=obs.positions[rows],
        features=obs.features[rows], edges=np.array([[0, 1], [1, 2], [2, 3]]),
        edge_lengths=np.array([1., 1., 2.]), current_index=int(mapping[obs.current_index]),
        action_nodes=mapping[obs.action_nodes])
    return SimpleNamespace(observation=reduced, fine_to_coarse=mapping,
        virtual_mask=np.array([False, False, True, False]), diagnostics={"fixture": True})


def expansion(obs, max_positions=32):
    if not max_positions:
        return obs
    return replace(obs, action_nodes=np.r_[obs.action_nodes, 4],
        action_yaws=np.r_[obs.action_yaws, 0.], goals=np.vstack([obs.goals, [4., 0., 0.]]))


class DummyEnv:
    """Only replaces external native transport; step consumes the actual env goals."""
    def __init__(self, *, error=None, coverage=99, collision=False):
        self.initial = observation()
        self.observation = self.initial
        self.privileged = PrivilegedState("truth", np.array([20], np.uint8))
        self.error, self.coverage, self.collision = error, coverage, collision
        self.core = object()
        self.config = SimpleNamespace(success_coverage=.99, goal_position_tolerance_m=.05)
        self.reference = SimpleNamespace(reference_id="reference-identity", area_m2=100.,
            coverage_ratio=lambda observed: float(observed[0]) / 100,
            covered_area=lambda observed: float(observed[0]))
        self.terrain = SimpleNamespace(terrain_id="terrain-identity")
        self.task = SimpleNamespace(polygon=self.initial.polygon)
        self.report = SimpleNamespace(exhausted=False)
        self.episode_metadata = {"domain_id": 37, "episode_id": "fixture-episode"}
        self.plant = SimpleNamespace(pose=SimpleNamespace(x=0., y=0., yaw=0.))
        self.steps = 0
        self.terminated = False
        self.closed = False
        self.executed_goals = []
        self.privileged_observation = None

    def reset(self, seed, family, extent, *, episode_budget):
        if self.error == "reset":
            raise BrokenPipeError("native worker disconnected")
        self.episode_budget = episode_budget
        return self.observation, self.privileged

    def _privileged(self, obs):
        self.privileged_observation = obs
        return PrivilegedState("truth", self.privileged.observed)

    def step(self, index):
        if self.error == "step":
            raise BrokenPipeError("native worker disconnected")
        goal = tuple(self.observation.goals[index])
        self.executed_goals.append(goal)
        before, truth = self.observation, self.privileged
        self.steps += 1
        self.plant.pose = SimpleNamespace(x=goal[0], y=goal[1], yaw=goal[2])
        self.observation = replace(self.initial, revision=18, current_index=4)
        self.privileged = PrivilegedState("truth", np.array([self.coverage], np.uint8))
        self.terminated = self.coverage >= 99
        reason = "COLLISION" if self.collision else "GOAL_REACHED"
        self.last_execution = SimpleNamespace(goal=goal, outcome=6 if self.collision else 0,
            reason_code=reason, actual_pose=goal, distance_m=4., turn_rad=0.,
            simulation_s=20., wall_s=.1, geometry_failure=self.collision,
            map_revision=18, map_processed_stamp_ns=42, last_segment_revision=1,
            goal_cell=(4, 0), actual_cell=(4, 0))
        return Transition(before, index, 1.23, self.observation, truth, self.privileged,
            RewardParts(10., 4., 0., 3, 0), self.terminated, not self.terminated,
            "fixture-episode", 0)

    def progress(self):
        return {"distance_m": 4. * self.steps, "simulation_s": 20. * self.steps,
                "sensor_frames": 2 + 200 * self.steps, "steps": self.steps}

    def close(self):
        self.closed = True


def test_contracted_actor_actions_execute_original_env_index():
    module = evaluator()
    env = DummyEnv()
    core, truth = env.core, env.privileged
    view = module.prepare_policy_view(env, env.initial, "contracted", contract_fn=contraction)
    assert view.policy_observation.goals.tobytes() == env.initial.goals.tobytes()
    assert view.policy_observation.action_nodes.tolist() == [0, 1]
    assert env.observation is env.initial and env.privileged is truth
    env.step(1)
    assert env.executed_goals == [(1., 0., .785)]
    assert env.core is core


def test_remote_substitution_uses_expanded_fine_observation_before_contracting():
    module = evaluator()
    env = DummyEnv()
    core = env.core
    view = module.prepare_policy_view(env, env.initial, "remote",
        contract_fn=contraction, expand_fn=expansion)
    assert env.observation is view.fine_observation
    assert env.privileged_observation is env.observation
    assert env.observation.goals[:2].tobytes() == env.initial.goals.tobytes()
    assert view.policy_observation.action_nodes.tolist() == [0, 1, 3]
    env.step(2)
    assert env.executed_goals == [(4., 0., 0.)]
    assert len(env.observation.goals) == 2  # Native core builds the next original fine graph.
    assert env.core is core


def test_virtual_target_or_changed_original_goal_is_rejected():
    module = evaluator()
    def bad_virtual(obs):
        result = contraction(obs)
        result.virtual_mask[0] = True
        return result
    with pytest.raises(ValueError, match="virtual"):
        module.prepare_policy_view(DummyEnv(), observation(), "contracted", contract_fn=bad_virtual)
    def bad_expansion(obs, max_positions):
        goals = obs.goals.copy()
        goals[0, 0] = 9
        return replace(obs, goals=goals)
    with pytest.raises(ValueError, match="prefix"):
        module.prepare_policy_view(DummyEnv(), observation(), "remote",
            contract_fn=contraction, expand_fn=bad_expansion)


def test_finished_episode_saves_full_original_observations_and_real_outcomes(tmp_path):
    module = evaluator()
    env = DummyEnv()
    result = module.run_episode(env, lambda obs: len(obs.goals) - 1, "remote", tmp_path,
        family="moon", extent=50., seed=123, budget=256,
        provenance={"actor_sha": "actor-hash", "config_sha": "config-hash"},
        contract_fn=contraction, expand_fn=expansion)
    assert result["status"] == "finished"
    assert result["successes"] == result["ended"] == 1 and result["success_rate"] == 1
    assert result["final_coverage"] == .99 and result["reward_total"] == 1.23
    assert result["provenance"]["actor_sha"] == "actor-hash"
    assert result["provenance"]["terrain_id"] == "terrain-identity"
    assert result["provenance"]["reference_id"] == "reference-identity"
    steps = [json.loads(line) for line in (tmp_path / "steps.jsonl").read_text().splitlines()]
    assert steps[0]["selected_goal"] == [4., 0., 0.]
    assert steps[0]["execution"]["reason_code"] == "GOAL_REACHED"
    assert steps[0]["fine_nodes"] == 5 and steps[0]["policy_nodes"] == 4
    assert steps[0]["new_area_m2"] == 10. and steps[0]["distance_m"] == 4.
    index = json.loads((tmp_path / "observations/index.json").read_text())
    original = module.load_observation(tmp_path / index["observations"][0]["path"])
    assert original.goals.tobytes() == env.initial.goals.tobytes()
    assert len(original.node_ids) == 5
    assert len(index["observations"]) == 2
    assert json.loads((tmp_path / "status.json").read_text())["status"] == "finished"
    assert env.closed


@pytest.mark.parametrize("error", ["reset", "step"])
def test_infrastructure_failure_is_terminal_error_excluded_from_success_denominator(tmp_path, error):
    module = evaluator()
    env = DummyEnv(error=error)
    result = module.run_episode(env, lambda obs: 0, "baseline", tmp_path,
        family="cave", extent=50., seed=123, budget=256)
    assert result["status"] == "error" and result["ended"] == 0
    assert result["successes"] == 0 and result["success_rate"] is None
    assert result["infrastructure_failures"] == 1
    assert result["error_category"] == "infrastructure"
    assert json.loads((tmp_path / "summary.json").read_text())["status"] == "error"
    assert env.closed


def test_budget_failure_is_valid_finished_policy_episode(tmp_path):
    module = evaluator()
    result = module.run_episode(DummyEnv(coverage=70), lambda obs: 0, "baseline", tmp_path,
        family="cave", extent=50., seed=123, budget=256)
    assert result["status"] == "finished" and result["ended"] == 1
    assert result["successes"] == 0 and result["success_rate"] == 0
    assert result["truncated"] and result["infrastructure_failures"] == 0


def test_collision_cannot_count_as_completion_at_99_percent(tmp_path):
    module = evaluator()
    result = module.run_episode(DummyEnv(collision=True), lambda obs: 0, "baseline", tmp_path,
        family="moon", extent=50., seed=123, budget=256)
    assert result["status"] == "finished" and result["ended"] == 1
    assert result["successes"] == 0 and result["collisions"] == 1


def test_observe_timer_preserves_return_value_and_restores_class_method():
    module = evaluator()
    class Core:
        def observe(self, arg):
            return arg
    original = Core.observe
    with module.ObserveTimer(Core) as timer:
        value = object()
        assert Core().observe(value) is value
        assert timer.calls == 1 and timer.seconds >= 0
    assert Core.observe is original


def test_static_raster_uses_real_reference_mask_and_frame_marks_virtual_nodes(tmp_path):
    module = evaluator()
    env = DummyEnv()
    env.terrain.navigation = np.array([[1, 2], [1, 1]], np.uint8)
    env.terrain.shape, env.terrain.origin, env.terrain.resolution_m = (2, 2), (-1., -1.), .2
    env.reference.mask = lambda: np.array([[True, False], [False, True]])
    result = module.run_episode(env, lambda obs: 1, "contracted", tmp_path,
        family="moon", extent=50., seed=123, budget=256, contract_fn=contraction)
    assert result["status"] == "finished"
    static = json.loads((tmp_path / "static.json").read_text())
    assert static["terrain"]["navigation"] == [[1, 2], [1, 1]]
    assert static["terrain"]["target"] == [[1, 0], [0, 1]]
    assert static["task_polygon"] == [[-1., -1.], [5., -1.], [5., 1.], [-1., 1.]]
    import gzip
    frame = json.loads(gzip.decompress((tmp_path / "frames/step-000000.json.gz").read_bytes()))
    assert frame["virtual_mask"] == [False, False, True, False]
    assert frame["fine_nodes"] == [[0., 0.], [1., 0.], [2., 0.], [3., 0.], [4., 0.]]
    assert frame["selected_goal"] == [1., 0., .785]


def test_pre_scan_scene_pose_pairs_even_if_initialization_yaw_or_transport_differs(tmp_path):
    module = evaluator()
    env = DummyEnv()
    env.probe_initial_pose = [0., 0., -1.]
    env.plant.pose.yaw = 2.
    result = module.run_episode(env, lambda obs: 0, "baseline", tmp_path,
        family="moon", extent=50., seed=123, budget=256)
    assert result["provenance"]["initial_pose"] == [0., 0., -1.]
    assert result["provenance"]["initial_decision_pose"] == [0., 0., 2.]
    left = observation()
    right = replace(left, epoch="another-transport-session", revision=700)
    assert module.observation_hash(left) != module.observation_hash(right)
    assert module.observation_hash(left, include_transport=False) == module.observation_hash(right, include_transport=False)


def test_sampled_raster_preserves_world_cell_center_and_polygon_identity(tmp_path):
    module = evaluator()
    env = DummyEnv()
    env.terrain.navigation = np.ones((181, 2), np.uint8)
    env.terrain.shape, env.terrain.origin, env.terrain.resolution_m = (181, 2), (-1., -1.), .2
    env.reference.mask = lambda: np.zeros((181, 2), bool)
    env.task.polygon = np.array([[10., 10.], [20., 10.], [20., 20.], [10., 20.]])
    result = module.run_episode(env, lambda obs: 0, "baseline", tmp_path,
        family="moon", extent=50., seed=123, budget=256)
    static = json.loads((tmp_path / "static.json").read_text())
    raster = static["terrain"]
    assert raster["shape"] == [91, 1] and raster["resolution_m"] == .4
    assert raster["native_resolution_m"] == .2 and raster["native_shape"] == [181, 2]
    assert np.allclose(np.asarray(raster["origin"]) + raster["resolution_m"] / 2, [-.9, -.9])
    assert result["provenance"]["polygon_digest"] == module.digest(env.task.polygon)
