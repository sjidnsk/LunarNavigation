"""One frozen native case; run through the original task-frontier run.sh wrapper.

Only policy observations are contracted. Remote mode substitutes a true expanded
fine action list in the environment, then the unchanged native step rebuilds the
next fine observation. No reward, navigation, sensor or simulated tick is changed.
"""
import argparse
from collections.abc import Mapping
from contextlib import AbstractContextManager
from dataclasses import dataclass, fields, is_dataclass
import gzip
import hashlib
import importlib
import json
import os
from pathlib import Path
import platform
import subprocess
import tempfile
import threading
import time
import traceback

import numpy as np

ORIGINAL_SOURCE = Path("/home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/drl-task-frontier")
ORIGINAL_WRAPPER = ORIGINAL_SOURCE / "scripts/drl/task_frontier/run.sh"
ORIGINAL_CACHE = Path("/home/kai/.cache/lunar-drl-task-frontier/jazzy-task8-20261004")
ARRAY_FIELDS = ("node_ids", "positions", "features", "edges", "edge_lengths", "polygon",
                "context", "action_nodes", "action_yaws", "goals")
SCALAR_FIELDS = ("current_index", "epoch", "revision", "schema")


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return {field.name: plain(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, Mapping):
        return {str(key): plain(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(item) for item in value]
    if hasattr(value, "__dict__"):
        return plain(vars(value))
    return value


def digest(value):
    return hashlib.sha256(json.dumps(plain(value), sort_keys=True, separators=(",", ":"),
        ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def sha(path):
    hasher = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(block)
    return hasher.hexdigest()


def observation_hash(observation, *, include_transport=True):
    """Hash all typed fields, including action goals, without rounding floats."""
    hasher = hashlib.sha256(b"region_contraction_observation_v1")
    for name in ARRAY_FIELDS:
        array = np.asarray(getattr(observation, name))
        hasher.update(name.encode())
        hasher.update(array.dtype.str.encode())
        hasher.update(str(array.shape).encode())
        hasher.update(array.tobytes(order="C"))
    names = SCALAR_FIELDS if include_transport else ("current_index", "schema")
    hasher.update(json.dumps({name: plain(getattr(observation, name)) for name in names},
                            sort_keys=True, allow_nan=False).encode())
    return hasher.hexdigest()


def atomic_file(path, writer):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix="." + path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            writer(stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def write_json(path, value):
    payload = (json.dumps(plain(value), ensure_ascii=False, allow_nan=False, indent=2) + "\n").encode()
    atomic_file(path, lambda stream: stream.write(payload))


def _same_array(left, right):
    return left.dtype == right.dtype and left.shape == right.shape and left.tobytes() == right.tobytes()


def world_polygon(env, observation):
    task = getattr(env, "task", None)
    if task is not None:
        return task.polygon
    return observation.polygon.astype(np.float64) * 10. + observation.positions[observation.current_index]


@dataclass(frozen=True)
class PolicyView:
    native_observation: object
    fine_observation: object
    policy_observation: object
    fine_to_coarse: np.ndarray
    virtual_mask: np.ndarray
    diagnostics: dict
    timing: dict


def prepare_policy_view(env, observation, mode, max_remote_positions=32, *,
                        contract_fn=None, expand_fn=None, substitute=True):
    """Validate action identity before any true-goal substitution in A2."""
    if mode not in ("baseline", "contracted", "remote"):
        raise ValueError("unknown evaluation mode")
    if max_remote_positions < 0:
        raise ValueError("max remote positions must be nonnegative")
    if mode != "baseline" and (contract_fn is None or (mode == "remote" and expand_fn is None)):
        # This local standalone module imports the installed ORIGINAL contracts.
        try:
            module = importlib.import_module("contraction")
        except ModuleNotFoundError as exc:
            if exc.name != "contraction":
                raise
            module = importlib.import_module("scripts.drl.region_contraction.contraction")
        contract_fn = contract_fn or module.contract_observation
        expand_fn = expand_fn or module.expand_frontier_actions
    timing = dict(contraction_seconds=0., remote_expansion_seconds=0., privileged_substitution_seconds=0.)
    fine = observation
    if mode == "remote":
        started = time.perf_counter()
        fine = expand_fn(observation, max_positions=max_remote_positions)
        timing["remote_expansion_seconds"] = time.perf_counter() - started
        count = len(observation.goals)
        if (len(fine.goals) < count or not _same_array(fine.goals[:count], observation.goals)
                or not _same_array(fine.action_nodes[:count], observation.action_nodes)
                or not _same_array(fine.action_yaws[:count], observation.action_yaws)):
            raise ValueError("remote original action prefix changed")
        for name in ("node_ids", "positions", "features", "edges", "edge_lengths", "polygon", "context"):
            if not _same_array(getattr(fine, name), getattr(observation, name)):
                raise ValueError("remote expansion changed native fine graph: " + name)
        if any(getattr(fine, name) != getattr(observation, name) for name in SCALAR_FIELDS):
            raise ValueError("remote expansion changed native fine graph identity")
    if mode == "baseline":
        policy = fine
        mapping = np.arange(len(fine.node_ids), dtype=np.int64)
        virtual = np.zeros(len(fine.node_ids), dtype=bool)
        diagnostics = dict(fine_nodes=len(fine.node_ids), coarse_nodes=len(fine.node_ids), virtual_nodes=0)
    else:
        started = time.perf_counter()
        result = contract_fn(fine)
        timing["contraction_seconds"] = time.perf_counter() - started
        policy, mapping, virtual = result.observation, result.fine_to_coarse, result.virtual_mask
        diagnostics = plain(result.diagnostics)
    if (mapping.shape != (len(fine.node_ids),) or virtual.shape != (len(policy.node_ids),)
            or np.any(mapping < 0) or np.any(mapping >= len(policy.node_ids))):
        raise ValueError("invalid contraction mapping")
    if not _same_array(policy.goals, fine.goals) or not _same_array(policy.action_yaws, fine.action_yaws):
        raise ValueError("contracted action goals or headings changed")
    if not np.array_equal(policy.action_nodes, mapping[fine.action_nodes]):
        raise ValueError("contracted action indices lost fine target correspondence")
    if np.any(virtual[policy.action_nodes]):
        raise ValueError("virtual region cannot be a navigation goal")
    if mode == "remote" and substitute:
        started = time.perf_counter()
        privileged = env._privileged(fine)
        # Deliberately substitute the expanded FINE graph, never the coarse graph.
        env.observation, env.privileged = fine, privileged
        timing["privileged_substitution_seconds"] = time.perf_counter() - started
    return PolicyView(observation, fine, policy, mapping, virtual, diagnostics, timing)


class ObserveTimer(AbstractContextManager):
    """Time existing observe calls once; never call observe or a physical tick extra."""
    def __init__(self, core_class):
        self.core_class, self.seconds, self.calls = core_class, 0., 0

    def __enter__(self):
        self.original = self.core_class.observe
        timer = self
        def observed(core, *args, **kwargs):
            started = time.perf_counter()
            try:
                return timer.original(core, *args, **kwargs)
            finally:
                timer.seconds += time.perf_counter() - started
                timer.calls += 1
        self.core_class.observe = observed
        return self

    def __exit__(self, *args):
        self.core_class.observe = self.original


class StatusWriter:
    def __init__(self, root, mode):
        self.path = Path(root) / "status.json"
        self.started = self.phase_started = time.monotonic()
        self.state = dict(status="running", mode=mode, pid=os.getpid(), phase="initializing", step=0)
        self.lock, self.stopped = threading.Lock(), threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while not self.stopped.wait(2.):
            self.emit()

    def update(self, **values):
        with self.lock:
            self.state.update(values)
            self.phase_started = time.monotonic()
        self.emit()

    def emit(self):
        with self.lock:
            write_json(self.path, dict(self.state, updated_unix=time.time(),
                episode_wall_s=time.monotonic() - self.started,
                phase_wall_s=time.monotonic() - self.phase_started))

    def close(self):
        self.stopped.set()
        self.thread.join()


def save_observation(path, observation, view, metadata):
    arrays = {name: getattr(observation, name) for name in (*ARRAY_FIELDS, *SCALAR_FIELDS)}
    arrays["_metadata_json"] = np.asarray(json.dumps(plain(metadata), sort_keys=True, allow_nan=False))
    arrays["_fine_to_coarse"], arrays["_virtual_mask"] = view.fine_to_coarse, view.virtual_mask
    for name in (*ARRAY_FIELDS, *SCALAR_FIELDS):
        arrays["_policy_" + name] = getattr(view.policy_observation, name)
    atomic_file(path, lambda stream: np.savez_compressed(stream, **arrays))


def load_observation(path):
    """Read the original native fine observation; allow_pickle is always disabled."""
    from lunar_drl_exploration.contracts import DecisionObservation
    with np.load(path, allow_pickle=False) as archive:
        values = {name: archive[name] for name in ARRAY_FIELDS}
        values.update({name: archive[name].item() for name in SCALAR_FIELDS})
    return DecisionObservation(**values)


class Captures:
    thresholds = (.25, .5, .75, .8, .9, .95, .99)

    def __init__(self, root, mode, proof):
        self.root, self.mode, self.proof = Path(root), mode, proof
        self.snapshots, self.frames, self.crossed = [], [], set()
        self.saved_steps = set()

    def static(self, env, observation):
        terrain = env.terrain
        raster = None
        if hasattr(terrain, "navigation"):
            stride = max(1, int(np.ceil(max(terrain.shape) / 180)))
            target = getattr(env.reference, "mask", None)
            if target is None:
                target = getattr(env.reference, "reference", None)
            if callable(target):
                target = target()
            if target is not None:
                target = np.asarray(target)
                if target.shape != terrain.navigation.shape:
                    target = None
            sampled = terrain.navigation[::stride, ::stride]
            # Preserve each sampled native cell's world center. Sampling is for
            # display, not a conservative traversability resampling operation.
            display_origin = np.asarray(terrain.origin[:2]) + terrain.resolution_m * (1 - stride) / 2
            raster = dict(origin=display_origin, resolution_m=terrain.resolution_m * stride,
                shape=sampled.shape, stride=stride, native_resolution_m=terrain.resolution_m,
                native_origin=terrain.origin, native_shape=terrain.shape,
                navigation=sampled,
                target=None if target is None else target[::stride, ::stride].astype(np.uint8))
        write_json(self.root / "static.json", dict(schema="region_contraction_static_v1",
            mode=self.mode, task_polygon=world_polygon(env, observation), initial_pose=self.proof["initial_pose"],
            terrain_id=terrain.terrain_id, reference_id=env.reference.reference_id, terrain=raster,
            raster_note="Sampled truth for offline visualization only; never supplied to the Actor.",
            provenance=self.proof))

    def observation(self, observation, view, step, coverage, *, final=False):
        newly_crossed = {threshold for threshold in self.thresholds if coverage >= threshold} - self.crossed
        self.crossed.update(newly_crossed)
        if step in self.saved_steps or not (step == 0 or final or newly_crossed or step % 32 == 0):
            return
        relative = f"observations/step-{step:06d}.npz"
        metadata = dict(step=step, coverage=coverage, final=final,
            coverage_thresholds=sorted(newly_crossed), mode=self.mode,
            observation_hash=observation_hash(observation), provenance=self.proof)
        save_observation(self.root / relative, observation, view, metadata)
        self.snapshots.append(dict(path=relative, sha=sha(self.root / relative), **metadata))
        self.saved_steps.add(step)
        write_json(self.root / "observations/index.json", dict(schema="region_contraction_observations_v1",
            observations=self.snapshots, note="Unprefixed fields are the original native fine observation."))

    def frame(self, observation, view, step, coverage, trajectory, selected_goal=None):
        policy = view.policy_observation
        value = dict(schema="region_contraction_frame_v1", mode=self.mode, step=step, coverage=coverage,
            fine_nodes=observation.positions, coarse_nodes=policy.positions, virtual_mask=view.virtual_mask,
            fine_node_ids=observation.node_ids, coarse_node_ids=policy.node_ids,
            fine_edges=observation.edges, coarse_edges=policy.edges,
            fine_edge_lengths=observation.edge_lengths, coarse_edge_lengths=policy.edge_lengths,
            current_position=observation.positions[observation.current_index], selected_goal=selected_goal,
            inside_gain=observation.features[:, 3:11].max(axis=1),
            outside_gain=observation.features[:, 11:19].max(axis=1),
            coarse_inside_gain=policy.features[:, 3:11].max(axis=1),
            coarse_outside_gain=policy.features[:, 11:19].max(axis=1),
            action_goals=view.fine_observation.goals, action_nodes=policy.action_nodes,
            fine_to_coarse=view.fine_to_coarse, trajectory=trajectory, diagnostics=view.diagnostics,
            native_observation_hash=observation_hash(observation))
        payload = (json.dumps(plain(value), ensure_ascii=False, allow_nan=False) + "\n").encode()
        relative = f"frames/step-{step:06d}.json.gz"
        atomic_file(self.root / relative, lambda stream: stream.write(gzip.compress(payload, mtime=0)))
        self.frames.append(dict(path=relative, step=step, coverage=coverage, sha=sha(self.root / relative)))
        write_json(self.root / "frames/index.json", dict(schema="region_contraction_frames_v1", frames=self.frames))


def classify_error(error):
    if type(error).__name__ in ("InfrastructureError", "InputUnavailable", "TaskCanceled") or isinstance(
            error, (BrokenPipeError, ConnectionError, TimeoutError, OSError)):
        return "infrastructure"
    if isinstance(error, KeyboardInterrupt):
        return "interrupted"
    return "adapter_or_identity"


def _pairing(proof, family, extent, seed, budget):
    keys = ("actor_sha", "pairing_config_digest", "terrain_id", "reference_id", "semantic_id",
            "polygon_digest", "initial_pose")
    return digest(dict(case=dict(family=family, extent_m=extent, seed=seed, budget=budget),
                       **{key: proof.get(key) for key in keys}))


def run_episode(env, policy, mode, output, *, family, extent, seed, budget, provenance=None,
                max_remote_positions=32, contract_fn=None, expand_fn=None, observe_timer=None,
                verify_frozen=None):
    """Run one complete case; exceptions yield a terminal error, never a policy failure."""
    from lunar_drl_exploration.evaluation import EpisodeMetrics, GeometryMetrics
    root = Path(output)
    root.mkdir(parents=True, exist_ok=True)
    if (root / "summary.json").exists() or (root / "steps.jsonl").exists():
        raise ValueError("evaluation output already contains a case; choose a fresh directory")
    proof = dict(provenance or {})
    started = time.perf_counter()
    status = StatusWriter(root, mode)
    metric = EpisodeMetrics(family, extent, seed, env.config.success_coverage)
    geometry, error, captures, observation, view = None, None, None, None, None
    trajectory, graph_rows, reward_total = [], [], 0.
    timing = {key: 0. for key in ("actor_seconds", "contraction_seconds", "remote_expansion_seconds",
        "privileged_substitution_seconds", "fine_observe_seconds", "env_step_seconds",
        "navigation_step_seconds", "native_execution_wall_seconds", "artifact_seconds", "reset_wall_seconds")}
    observe_before = observe_timer.seconds if observe_timer else 0.
    try:
        status.update(phase="reset")
        reset_started = time.perf_counter()
        observation, state = env.reset(seed, family, extent, episode_budget=budget)
        timing["reset_wall_seconds"] = time.perf_counter() - reset_started
        if observe_timer:
            timing["fine_observe_seconds"] += observe_timer.seconds - observe_before
        pose = env.plant.pose
        decision_pose = [float(pose.x), float(pose.y), float(pose.yaw)]
        proof.update(terrain_id=env.terrain.terrain_id, reference_id=env.reference.reference_id,
            initial_pose=list(getattr(env, "probe_initial_pose", decision_pose)), initial_decision_pose=decision_pose,
            polygon_digest=digest(world_polygon(env, observation)),
            initial_observation_hash=observation_hash(observation, include_transport=False),
            exact_initial_observation_hash=observation_hash(observation),
            actual_schema=observation.schema, episode_metadata=plain(env.episode_metadata))
        proof["pairing_digest"] = _pairing(proof, family, extent, seed, budget)
        write_json(root / "provenance.json", proof)
        geometry = GeometryMetrics(observation.positions[observation.current_index],
                                   env.config.goal_position_tolerance_m)
        trajectory.append([float(pose.x), float(pose.y), float(pose.yaw)])
        metric.observe(env.reference.coverage_ratio(state.observed), env.progress()["distance_m"],
            0., "INITIAL", env.terminated, False, exhausted=env.report.exhausted,
            covered_area_m2=env.reference.covered_area(state.observed), coverable_area_m2=env.reference.area_m2)
        captures = Captures(root, mode, proof)
        artifact_started = time.perf_counter()
        captures.static(env, observation)
        timing["artifact_seconds"] += time.perf_counter() - artifact_started
        with (root / "steps.jsonl").open("x", encoding="utf-8") as stream:
            while not metric.terminated and not metric.truncated and not metric.collisions:
                status.update(phase="policy", step=metric.steps, coverage=metric.coverage)
                view = prepare_policy_view(env, observation, mode, max_remote_positions,
                                           contract_fn=contract_fn, expand_fn=expand_fn)
                for key, value in view.timing.items():
                    timing[key] += value
                actor_started = time.perf_counter()
                index = int(policy(view.policy_observation))
                actor_seconds = time.perf_counter() - actor_started
                timing["actor_seconds"] += actor_seconds
                if not 0 <= index < len(view.fine_observation.goals):
                    raise ValueError("Actor action is outside the true fine action list")
                selected_goal = view.fine_observation.goals[index].tolist()
                artifact_started = time.perf_counter()
                captures.observation(observation, view, metric.steps, metric.coverage)
                captures.frame(observation, view, metric.steps, metric.coverage, trajectory, selected_goal)
                timing["artifact_seconds"] += time.perf_counter() - artifact_started
                status.update(phase="native_step", selected_goal=selected_goal)
                observed_before = observe_timer.seconds if observe_timer else 0.
                step_started = time.perf_counter()
                transition = env.step(index)
                env_step_seconds = time.perf_counter() - step_started
                fine_seconds = (observe_timer.seconds - observed_before) if observe_timer else 0.
                execution = plain(env.last_execution)
                timing["fine_observe_seconds"] += fine_seconds
                timing["env_step_seconds"] += env_step_seconds
                timing["navigation_step_seconds"] += max(0., env_step_seconds - fine_seconds)
                timing["native_execution_wall_seconds"] += float(execution.get("wall_s", 0.))
                next_observation, state = transition.next_observation, transition.next_privileged
                geometry.observe(next_observation.positions[next_observation.current_index], transition.parts.new_area_m2)
                progress = env.progress()
                metric.observe(env.reference.coverage_ratio(state.observed), progress["distance_m"],
                    transition.parts.new_area_m2, execution["reason_code"], transition.terminated, transition.truncated,
                    exhausted=env.report.exhausted, covered_area_m2=env.reference.covered_area(state.observed),
                    coverable_area_m2=env.reference.area_m2)
                reward_total += float(transition.reward)
                trajectory.append(execution.get("actual_pose", next_observation.positions[next_observation.current_index].tolist()))
                graph = dict(fine_nodes=len(observation.node_ids), policy_nodes=len(view.policy_observation.node_ids),
                    fine_edges=len(observation.edges), policy_edges=len(view.policy_observation.edges),
                    native_actions=len(observation.goals), policy_actions=len(view.policy_observation.goals),
                    remote_positions=len(set(view.fine_observation.action_nodes.tolist()) - set(observation.action_nodes.tolist())))
                graph_rows.append(graph)
                row = dict(step=metric.steps, mode=mode, coverage=metric.coverage, reward=float(transition.reward),
                    reward_total=reward_total, new_area_m2=float(transition.parts.new_area_m2),
                    distance_m=metric.distance, path_length_m=metric.distance, reward_parts=plain(transition.parts),
                    selected_action=index, selected_goal=selected_goal,
                    selected_native_node_id=int(view.fine_observation.node_ids[view.fine_observation.action_nodes[index]]),
                    selected_policy_node=int(view.policy_observation.action_nodes[index]),
                    selected_virtual=bool(view.virtual_mask[view.policy_observation.action_nodes[index]]),
                    execution=execution, terminated=transition.terminated, truncated=transition.truncated,
                    native_observation_hash=observation_hash(observation),
                    fine_observation_hash=observation_hash(view.fine_observation),
                    policy_observation_hash=observation_hash(view.policy_observation),
                    next_native_observation_hash=observation_hash(next_observation),
                    epoch=observation.epoch, revision=observation.revision,
                    actor_sha=proof.get("actor_sha"), config_sha=proof.get("config_sha"),
                    terrain_id=proof["terrain_id"], reference_id=proof["reference_id"],
                    pairing_digest=proof["pairing_digest"], progress=progress, contraction=view.diagnostics,
                    loop_metrics=geometry.record(), **graph,
                    timing=dict(view.timing, actor_seconds=actor_seconds, fine_observe_seconds=fine_seconds,
                        env_step_seconds=env_step_seconds, navigation_step_seconds=max(0., env_step_seconds-fine_seconds),
                        native_execution_wall_seconds=float(execution.get("wall_s", 0.))))
                stream.write(json.dumps(plain(row), ensure_ascii=False, allow_nan=False) + "\n")
                stream.flush()
                print(json.dumps(dict(mode=mode, step=metric.steps, coverage=metric.coverage,
                    new_area_m2=transition.parts.new_area_m2, reason=execution["reason_code"],
                    nodes=[graph["fine_nodes"], graph["policy_nodes"]]), allow_nan=False), flush=True)
                observation = next_observation
            # Terminal snapshots retain the native fine observation, even in A2.
            view = prepare_policy_view(env, observation, mode, max_remote_positions,
                contract_fn=contract_fn, expand_fn=expand_fn, substitute=False)
            artifact_started = time.perf_counter()
            captures.observation(observation, view, metric.steps, metric.coverage, final=True)
            captures.frame(observation, view, metric.steps, metric.coverage, trajectory)
            timing["artifact_seconds"] += time.perf_counter() - artifact_started
        if verify_frozen:
            verify_frozen()
    except BaseException as exc:
        error = exc
        metric.error = f"{type(exc).__name__}: {exc}"
        write_json(root / "error.json", dict(error=metric.error, error_category=classify_error(exc),
            traceback=traceback.format_exc(), step=metric.steps, coverage=metric.coverage))
    finally:
        try:
            env.close()
        except BaseException as exc:
            if error is None:
                error = exc
                metric.error = f"{type(exc).__name__}: {exc}"
        status.close()
    timing["episode_wall_seconds"] = time.perf_counter() - started
    row = dict(metric.record(), **(geometry.record() if geometry else {}), reward_total=reward_total)
    if error is not None:
        row["completed"] = False
    graph = {}
    if graph_rows:
        for key in ("fine_nodes", "policy_nodes", "fine_edges", "policy_edges", "remote_positions"):
            graph["mean_" + key] = float(np.mean([item[key] for item in graph_rows]))
        graph["node_ratio"] = graph["mean_policy_nodes"] / graph["mean_fine_nodes"]
        graph["edge_ratio"] = graph["mean_policy_edges"] / graph["mean_fine_edges"] if graph["mean_fine_edges"] else 1.
    finished = error is None
    result = dict(row, schema="region_contraction_evaluation_v1", status="finished" if finished else "error",
        mode=mode, budget=budget, max_remote_positions=max_remote_positions, policy="frozen_joint_argmax",
        success_coverage=env.config.success_coverage, ended=int(finished), successes=int(finished and row["completed"]),
        success_rate=float(row["completed"]) if finished else None,
        infrastructure_failures=int(error is not None and classify_error(error) == "infrastructure"),
        error_category=classify_error(error) if error is not None else None,
        exit_code=0 if finished else 1, provenance=proof, cases=[row], timing=timing, graph=graph,
        timing_note="navigation_step_seconds is env.step minus core.observe; it includes snapshot and bookkeeping. "
                    "fine_observe_seconds includes initialization; compression does not reduce fine graph construction.",
        observe_timing_available=observe_timer is not None, pid=os.getpid())
    write_json(root / "summary.json", result)
    status.update(status=result["status"], phase="finished" if finished else "error", step=metric.steps,
                  coverage=metric.coverage, error=metric.error, exit_code=result["exit_code"])
    return result


def runtime_identity():
    modules = {}
    for name in ("lunar_drl_exploration.evaluation", "lunar_drl_exploration.ros_env",
                 "lunar_drl_exploration.reference", "lunar_drl_exploration.runtime", "lunar_drl_terrain_native"):
        path = Path(importlib.import_module(name).__file__).resolve()
        modules[name] = dict(path=str(path), sha=sha(path))
    package = Path(importlib.import_module("lunar_drl_exploration.ros_env").__file__).resolve().parent
    from ament_index_python.packages import get_package_prefix
    native = {}
    for package_name, executable in (("lunar_incremental_navigation_ros", "lunar_incremental_navigation_node"),
                                    ("lunar_pure_wheeled_controller", "lunar_pure_wheeled_controller_node.py")):
        path = Path(get_package_prefix(package_name)) / "lib" / package_name / executable
        native[package_name] = dict(path=str(path.resolve()), sha=sha(path))
    import scipy
    return dict(modules=modules, navigation=native,
        package_sha=digest([(path.name, sha(path)) for path in sorted(package.glob("*.py"))]),
        python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__)


def native_provenance(args, config):
    from lunar_drl_exploration.config import config_record, resume_semantics
    if os.environ.get("ROS_DISTRO") != "jazzy" or Path(os.environ.get("DRL_CACHE", "")).resolve() != ORIGINAL_CACHE:
        raise ValueError("use the original run.sh with DRL_CACHE=" + str(ORIGINAL_CACHE))
    if any(path.is_symlink() for path in (ORIGINAL_CACHE, *ORIGINAL_CACHE.parents)):
        raise ValueError("unsafe cache symlink")
    marker = ORIGINAL_CACHE / ".drl-source.json"
    identity = json.loads(marker.read_text(encoding="utf-8"))
    if marker.is_symlink() or identity.get("source_root") != str(ORIGINAL_SOURCE) or identity.get("ros_distro") != "jazzy":
        raise ValueError("foreign native cache identity")
    native = ORIGINAL_CACHE / "build/lunar_drl_exploration/native"
    if not native.is_symlink() or native.resolve() != ORIGINAL_SOURCE / "ros2_ws/src/lunar_drl_exploration/native":
        raise ValueError("foreign native source")
    runtime = runtime_identity()
    for name, item in runtime["modules"].items():
        if name != "lunar_drl_terrain_native" and not Path(item["path"]).is_relative_to(ORIGINAL_SOURCE):
            raise ValueError("evaluation imported a foreign Python overlay: " + name)
    for item in runtime["navigation"].values():
        if not Path(item["path"]).is_relative_to(ORIGINAL_CACHE) and not Path(item["path"]).is_relative_to(ORIGINAL_SOURCE):
            raise ValueError("evaluation imported a foreign navigation overlay")
    record = config_record(config)
    platform_record = dict(record["platform"])
    platform_record.pop("capability_path", None)
    pair = dict(sensor=record["sensor"], platform=platform_record,
        integration_step_s=config.integration_step_s, observation_hz=config.observation_hz,
        goal_position_tolerance_m=config.goal_position_tolerance_m, success_coverage=config.success_coverage,
        generator_version=5)
    experiment = Path(__file__).resolve().parents[3]
    def commit(source):
        return subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
    sources = {path.name: sha(path) for path in (Path(__file__).resolve(), Path(__file__).with_name("contraction.py"))}
    return dict(actor=str(args.actor.resolve()), actor_sha=sha(args.actor), config=str(args.config.resolve()),
        config_sha=sha(args.config), source_root=str(ORIGINAL_SOURCE), source_commit=commit(ORIGINAL_SOURCE),
        experiment_root=str(experiment), experiment_commit=commit(experiment), experiment_sources=sources,
        wrapper=str(ORIGINAL_WRAPPER), wrapper_sha=sha(ORIGINAL_WRAPPER), cache=str(ORIGINAL_CACHE),
        cache_identity=identity, runtime=runtime, effective_config=record, config_digest=digest(record),
        semantic_id=digest(resume_semantics(config)), pairing_config_digest=digest(pair),
        torch_threads=2, run_id=f"{args.family}-{args.extent:g}-{args.seed}-{args.mode}")


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--mode", required=True, choices=("baseline", "contracted", "remote"))
    for name in ("config", "actor", "output"):
        result.add_argument("--" + name, required=True, type=Path)
    result.add_argument("--family", required=True, choices=("moon", "cave"))
    result.add_argument("--extent", required=True, type=float)
    result.add_argument("--seed", required=True, type=int)
    result.add_argument("--budget", required=True, type=int)
    result.add_argument("--max-remote-positions", type=int, default=32)
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    if args.budget <= 0 or not 20 <= args.extent <= 1000 or not 0 <= args.seed < 2**32 or args.max_remote_positions < 0:
        raise ValueError("invalid case extent, seed, budget or remote-position limit")
    if (args.output.resolve().is_relative_to(Path(__file__).resolve().parents[3])
            or args.output.resolve().is_relative_to(ORIGINAL_SOURCE.parents[1])):
        raise ValueError("evaluation artifacts must be outside the experiment repository")
    args.output.mkdir(parents=True, exist_ok=True)
    try:
        # Installed original package is loaded before the standalone contraction module.
        from lunar_drl_exploration.cli import load_config, parser as original_parser
        from lunar_drl_exploration.runtime import ActorPolicy
        from lunar_drl_exploration.ros_env import RosExplorationEnv
        from lunar_drl_exploration.decision import DecisionCore
        config = load_config(original_parser().parse_args(["evaluate", "--config", str(args.config), "--actor", str(args.actor)]))
        if config.success_coverage != .99:
            raise ValueError("this frozen experiment requires configured success_coverage=0.99")
        import torch
        torch.set_num_threads(2)
        proof = native_provenance(args, config)
        policy = ActorPolicy.load(args.actor)
        class EvaluationEnv(RosExplorationEnv):
            def reset(self, seed, family, extent, *, episode_budget=None):
                return super().reset(seed, family, extent, episode_budget=episode_budget,
                                     start_mode="inside", curriculum_task=f"{family}_inside")
            def reset_scene(self, terrain, pose, task, *, descriptor=None):
                self.probe_initial_pose = [float(pose.x), float(pose.y), float(pose.yaw)]
                return super().reset_scene(terrain, pose, task, descriptor=descriptor)
        env = EvaluationEnv(config, 0, domain_base=None)
        def verify():
            if native_provenance(args, config) != proof:
                raise ValueError("actor, configuration, source or runtime identity changed during evaluation")
        with ObserveTimer(DecisionCore) as timer:
            result = run_episode(env, policy, args.mode, args.output, family=args.family, extent=args.extent,
                seed=args.seed, budget=args.budget, provenance=proof, max_remote_positions=args.max_remote_positions,
                observe_timer=timer, verify_frozen=verify)
        print(json.dumps(dict(status=result["status"], mode=args.mode, successes=result["successes"],
            ended=result["ended"], success_rate=result["success_rate"], coverage=result["final_coverage"]), allow_nan=False), flush=True)
        return result["exit_code"]
    except BaseException as exc:
        # Bootstrap failures receive the same terminal classification as reset failures.
        if not (args.output / "summary.json").exists():
            result = dict(schema="region_contraction_evaluation_v1", status="error", mode=args.mode,
                family=args.family, extent_m=args.extent, seed=args.seed, budget=args.budget,
                error=f"{type(exc).__name__}: {exc}", error_category=classify_error(exc),
                infrastructure_failures=int(classify_error(exc) == "infrastructure"),
                successes=0, ended=0, success_rate=None, exit_code=1, completed=False)
            write_json(args.output / "summary.json", result)
            write_json(args.output / "status.json", result)
            write_json(args.output / "error.json", dict(result, traceback=traceback.format_exc()))
        raise


if __name__ == "__main__":
    raise SystemExit(main())
