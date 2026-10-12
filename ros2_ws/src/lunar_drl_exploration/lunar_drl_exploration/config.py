"""Training-side view of the canonical native platform capability file."""

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from types import MappingProxyType
from typing import Mapping

from .contracts import Pose, SensorSpec
from .feature_schema import (OBSERVATION_SCHEMA, PRIVILEGED_SCHEMA, LEARNER_SCHEMA,
                             frontier_semantics, FRONTIER_SEMANTICS)


@dataclass(frozen=True)
class GraphConfig:
    """Graph geometry constants are independent of sensor range."""
    position_unit_m: float = 10.0
    frontier_unit_cells: float = 100.0
    history_tolerance_m: float = 0.1
    coverage_radius_m: float = 2.0
    connection_limit_m: float = 8.0
    stretch: float = 1.2
    platform: "PlatformConfig | None" = None

    def __post_init__(self):
        if not (0 < self.coverage_radius_m < self.connection_limit_m and self.stretch >= 1.0):
            raise ValueError("positive graph radius/connection length and stretch >= 1 required")
        if self.position_unit_m != 10.0 or self.frontier_unit_cells != 100.0:
            raise ValueError("task_graph_v4 fixes position and frontier normalization")
        if not np.isfinite(self.history_tolerance_m) or self.history_tolerance_m <= 0:
            raise ValueError("positive history tolerance required")


@dataclass(frozen=True)
class PlatformConfig:
    maximum_forward_speed_mps: float
    maximum_reverse_speed_mps: float
    maximum_spin_rate_radps: float
    footprint_radius_m: float
    capability: Mapping = field(default_factory=dict)
    capability_path: str = ""

    def actor_context(
        self,
        pose: Pose,
        *,
        linear_speed_mps: float,
        angular_speed_radps: float,
        sensor_range_m: float,
        sensor_fov_rad: float
    ) -> np.ndarray:
        return np.asarray(
            [
                np.cos(pose.yaw),
                np.sin(pose.yaw),
                linear_speed_mps / 0.2,
                angular_speed_radps / self.maximum_spin_rate_radps,
                sensor_range_m / 10.0,
                sensor_fov_rad / np.pi,
                self.footprint_radius_m,
                self.maximum_forward_speed_mps / 0.2,
            ],
            dtype=np.float32,
        )


def load_platform_config(path: Path | None = None) -> PlatformConfig:
    """Read wheel capability values from the native source of truth."""
    if path is None:
        try:
            from ament_index_python.packages import (
                get_package_share_directory,
                PackageNotFoundError,
            )
        except ImportError:
            path = Path(__file__).resolve().parents[4] / "config" / "wheel.yaml"
        else:
            try:
                path = (
                    Path(
                        get_package_share_directory("lunar_incremental_navigation_ros")
                    )
                    / "config"
                    / "wheel.yaml"
                )
            except PackageNotFoundError:
                path = Path(__file__).resolve().parents[4] / "config" / "wheel.yaml"
    with Path(path).open(encoding="utf-8") as stream:
        capability = yaml.safe_load(stream)["capability"]
    footprint = np.asarray(capability["footprint_xy_m"], dtype=np.float64)
    radius = float(np.max(np.linalg.norm(footprint, axis=1)))
    return PlatformConfig(
        maximum_forward_speed_mps=float(capability["maximum_forward_speed_mps"]),
        maximum_reverse_speed_mps=float(capability["maximum_reverse_speed_mps"]),
        maximum_spin_rate_radps=float(capability["maximum_spin_rate_radps"]),
        footprint_radius_m=radius,
        capability=MappingProxyType(capability),
        capability_path=str(Path(path).resolve()),
    )


@dataclass(frozen=True)
class ModelConfig:
    """Primitive network settings; importing observed workers never imports Torch."""
    width: int = 128
    heads: int = 8
    layers: int = 6
    actor_score_bound: float = 0.0

    def __post_init__(self):
        if self.width <= 0 or self.heads <= 0 or self.width % self.heads or self.layers <= 0:
            raise ValueError("positive width divisible by heads and positive layers required")
        if not np.isfinite(self.actor_score_bound) or self.actor_score_bound < 0:
            raise ValueError("actor_score_bound must be finite and nonnegative")


def canonical_model_config(record):
    """Only legacy v3's missing score parameterization defaults to unbounded."""
    result = dict(record)
    result.setdefault('actor_score_bound', 0.0)
    return result


@dataclass(frozen=True)
class LearningConfig:
    batch_size: int = 64
    microbatch_size: int = 16
    learning_rate: float = 1e-5
    gamma: float = 1.0
    polyak: float = 0.005
    initial_alpha: float = 5e-5
    maximum_alpha: float = 1e-4
    target_entropy_factor: float = 0.10
    return_steps: int = 1
    entropy_mode: str = 'joint'
    position_target_entropy_factor: float = 0.10
    heading_target_entropy_factor: float = 0.10
    temperature_learning_rate: float | None = None
    position_aux_weight: float = 0.0

    def __post_init__(self):
        if (type(self.position_aux_weight) not in (int, float) or
                self.position_aux_weight != 0.0):
            raise ValueError('position_aux_weight must be 0.0 for task-frontier retraining')
        if self.entropy_mode not in ('joint', 'position_heading'):
            raise ValueError('entropy_mode must be joint or position_heading')
        optimization = (self.learning_rate, self.gamma, self.polyak, self.initial_alpha,
            self.maximum_alpha, self.target_entropy_factor, self.position_target_entropy_factor,
            self.heading_target_entropy_factor)
        if not all(np.isfinite(value) for value in optimization):
            raise ValueError('SAC optimization settings must be finite')
        if self.temperature_learning_rate is not None and not (
                np.isfinite(self.temperature_learning_rate) and self.temperature_learning_rate > 0):
            raise ValueError('temperature_learning_rate must be finite and positive or None')
        if type(self.return_steps) is not int or not 1 <= self.return_steps <= 8:
            raise ValueError('return_steps must be an integer in [1,8]')
        if self.batch_size != 64 or not 0 < self.microbatch_size <= self.batch_size:
            raise ValueError("effective batch is 64; microbatch must be in [1,64]")
        if not 0 < self.gamma <= 1:
            raise ValueError("task reward requires 0 < gamma <= 1")
        if not (0 < self.initial_alpha <= self.maximum_alpha and
                0 < self.polyak <= 1 and self.learning_rate > 0 and
                self.target_entropy_factor >= 0 and self.position_target_entropy_factor >= 0 and
                self.heading_target_entropy_factor >= 0):
            raise ValueError("invalid SAC optimization settings")


def canonical_learning_config(record):
    """Legacy learner/replay records use one-step targets and joint entropy."""
    result = dict(record)
    result.setdefault('return_steps', 1)
    result.setdefault('entropy_mode', 'joint')
    result.setdefault('position_target_entropy_factor', 0.10)
    result.setdefault('heading_target_entropy_factor', 0.10)
    result.setdefault('temperature_learning_rate', None)
    result.setdefault('position_aux_weight', 0.0)
    # Replay semantics omit optimizer controls. If initial_alpha is absent, its
    # default must not constrain the recorded cap. Use the cap as a validation-
    # only witness (0 < cap <= cap); never insert this control into the record.
    validation = dict(result)
    if 'initial_alpha' not in validation and 'maximum_alpha' in validation:
        validation['initial_alpha'] = validation['maximum_alpha']
    try:
        LearningConfig(**validation)
    except TypeError as error:
        raise ValueError('invalid learning config fields') from error
    return result


@dataclass(frozen=True)
class TrainingConfig:
    """Operational defaults and explicit experience semantics; no Torch/native import."""
    model: ModelConfig = field(default_factory=ModelConfig)
    learning: LearningConfig = field(default_factory=LearningConfig)
    completion_bonus: float = 5.0
    success_coverage: float = 0.99
    sensor: 'SensorSpec' = field(default_factory=lambda: SensorSpec())
    platform: PlatformConfig = field(default_factory=load_platform_config)
    seed: int = 20260915
    paired_scene_sequence: bool = False
    ordered_curriculum: bool = False
    course_two: bool = False
    course_two_variant: str = ''
    course_two_catalog: str = ''
    course_two_catalog_sha: str = ''
    course_two_task_scope: str = ''
    course_two_sampling: str = ''
    course_two_scene_profile: str = ''
    ordered_rehearsal_fraction: float = .2
    ordered_rehearsal_by_phase: tuple | None = None
    ordered_budget_rule: dict | None = None
    policy_graph_mode: str = 'fine'
    remote_frontier_positions: int = 32
    environments: int = 8
    target_rtf: float = 30.0
    integration_step_s: float = .05
    observation_hz: float = 2.0
    goal_position_tolerance_m: float = .05
    stagnation_window_steps: int = 0
    stagnation_max_positions: int = 8
    stagnation_position_tolerance_m: float = .25
    warmup: int = 1024
    update_ratio: float = .25
    max_update_credit: int = 32
    actor_publish_updates: int = 16
    worker_threads: int = 1
    collector_threads: int = 2
    learner_threads: int = 4
    dataloader_workers: int = 0
    curriculum_transition_boundaries: tuple = (20000, 60000)
    curriculum_mixtures: tuple = ((1., 0., 0.), (.25, .75, 0.), (.15, .25, .60))
    curriculum_budgets: tuple = (512, 2048, 8192)
    curriculum_extents_m: tuple = ((40, 80), (80, 150), (100, 300))
    replay_max_bytes: int = 8 * 1024**3
    snapshot_max_bytes: int = 2 * 1024**3
    total_output_max_bytes: int = 20 * 1024**3
    metrics_max_bytes: int = 8 * 1024**2
    artifact_root: str = ''
    reward_diagnostics: bool = False
    save_interval_s: float = 1800.0
    output_dir: str = 'training-output/drl-metric-critic'
    system_reserve_bytes: int = 2 * 1024**3
    owned_pss_limit_bytes: int | None = None

    def ordered_rehearsal_for_phase(self, phase):
        if type(phase) is not int or not 0 <= phase < 4:
            raise ValueError('invalid ordered course phase')
        return (self.ordered_rehearsal_fraction if self.ordered_rehearsal_by_phase is None
                else self.ordered_rehearsal_by_phase[phase])

    def __post_init__(self):
        boundaries = self.curriculum_transition_boundaries
        if self.policy_graph_mode not in ('fine', 'a2'):
            raise ValueError('policy_graph_mode must be fine or a2')
        if type(self.remote_frontier_positions) is not int or not 0 <= self.remote_frontier_positions <= 32:
            raise ValueError('remote_frontier_positions must be an integer in [0,32]')
        if type(self.ordered_curriculum) is not bool:
            raise ValueError('ordered_curriculum must be boolean')
        if type(self.course_two) is not bool or (self.course_two and self.ordered_curriculum):
            raise ValueError('course_two must be boolean and exclusive with ordered_curriculum')
        if self.course_two_variant:
            if (not self.course_two or self.course_two_variant not in ('C0','C1','C2','C3') or
                    (self.course_two_sampling!='scene_episode_v1' and
                        (not self.course_two_catalog or len(self.course_two_catalog_sha)!=64)) or
                    self.learning.batch_size!=64 or self.learning.return_steps!=1):
                raise ValueError('rebalance variant requires qualified catalog and one-step batch 64')
        elif self.course_two_catalog or self.course_two_catalog_sha:
            raise ValueError('catalog requires explicit rebalance variant')
        if self.course_two_task_scope not in ('','whole_cave_v1') or (
                self.course_two_task_scope and self.course_two_variant!='C3'):
            raise ValueError('whole-cave task scope requires C3')
        if self.course_two_sampling not in ('','scene_episode_v1') or (self.course_two_sampling and
                (self.course_two_variant!='C3' or self.course_two_task_scope!='whole_cave_v1')):
            raise ValueError('scene episode sampling requires whole-cave C3')
        if self.course_two_scene_profile not in ('','distance_sensor_10_30_v1') or (
                self.course_two_scene_profile and self.course_two_sampling!='scene_episode_v1'):
            raise ValueError('distance/sensor scene profile requires three-scene sampling')
        if (type(self.ordered_rehearsal_fraction) not in (int, float) or
                not np.isfinite(self.ordered_rehearsal_fraction) or
                not 0 <= self.ordered_rehearsal_fraction < 1):
            raise ValueError('ordered_rehearsal_fraction must be in [0,1)')
        if self.ordered_budget_rule is not None and not isinstance(self.ordered_budget_rule, dict):
            raise ValueError('ordered_budget_rule must be a dictionary or None')
        if self.ordered_rehearsal_by_phase is not None and (
                not isinstance(self.ordered_rehearsal_by_phase, (tuple, list)) or
                len(self.ordered_rehearsal_by_phase) != 4 or any(
                    type(value) not in (int, float) or not np.isfinite(value) or not 0 < value < 1
                    for value in self.ordered_rehearsal_by_phase)):
            raise ValueError('ordered_rehearsal_by_phase requires four fractions in (0,1)')
        if self.ordered_budget_rule is not None:
            import json
            try:
                json.dumps(self.ordered_budget_rule, allow_nan=False)
            except (TypeError, ValueError) as error:
                raise ValueError('ordered_budget_rule must contain finite JSON values') from error
        if not isinstance(self.artifact_root, str):
            raise ValueError('artifact_root must be a path string')
        if type(self.reward_diagnostics) is not bool:
            raise ValueError('reward_diagnostics must be boolean')
        if type(self.stagnation_window_steps) is not int or self.stagnation_window_steps < 0:
            raise ValueError('stagnation window must be a nonnegative integer')
        if type(self.stagnation_max_positions) is not int or self.stagnation_max_positions <= 0:
            raise ValueError('stagnation max positions must be a positive integer')
        if (not np.isfinite(self.stagnation_position_tolerance_m) or
                self.stagnation_position_tolerance_m <= 0):
            raise ValueError('stagnation position tolerance must be positive and finite')
        if self.stagnation_window_steps and not 0 < self.stagnation_max_positions < self.stagnation_window_steps:
            raise ValueError('enabled stagnation window must exceed max positions')
        if not np.isfinite(self.completion_bonus) or self.completion_bonus < 0:
            raise ValueError('completion bonus must be finite and nonnegative')
        if not 0 < self.success_coverage <= 1:
            raise ValueError('success coverage must be in (0, 1]')
        if (not isinstance(boundaries, (tuple, list)) or len(boundaries) != 2 or
                any(type(value) is not int or value < 0 for value in boundaries) or
                boundaries[0] >= boundaries[1]):
            raise ValueError('curriculum boundaries require two increasing nonnegative integers')
        mixtures = np.asarray(self.curriculum_mixtures, dtype=float)
        if (mixtures.shape != (3, 3) or not np.all(np.isfinite(mixtures)) or
                np.any(mixtures < 0) or not np.allclose(mixtures.sum(axis=1), 1., rtol=0., atol=1e-12)):
            raise ValueError('curriculum mixtures require three normalized nonnegative probability triples')
        for name in ('environments', 'max_update_credit', 'actor_publish_updates',
                     'worker_threads', 'collector_threads', 'learner_threads',
                     'replay_max_bytes', 'snapshot_max_bytes', 'total_output_max_bytes',
                     'metrics_max_bytes'):
            value = getattr(self, name)
            if not isinstance(value, int) or value <= 0:
                raise ValueError(f'positive integer required: {name}')
        for name in ('target_rtf', 'integration_step_s', 'observation_hz',
                     'goal_position_tolerance_m', 'update_ratio', 'save_interval_s'):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'positive finite value required: {name}')
        if self.warmup < 0 or self.dataloader_workers != 0 or self.system_reserve_bytes < 0:
            raise ValueError('invalid warmup, DataLoader or reserve settings')
        if self.owned_pss_limit_bytes is not None and self.owned_pss_limit_bytes <= 0:
            raise ValueError('positive owned PSS limit required')


def config_record(value):
    """Explicit canonical JSON/spawn record; asdict cannot deepcopy mappingproxy."""
    from dataclasses import fields, is_dataclass
    if is_dataclass(value):
        return {f.name: config_record(getattr(value, f.name)) for f in fields(value)}
    if isinstance(value, Mapping):
        return {str(key): config_record(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [config_record(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    if value is None or type(value) in (str, bool, int, float):
        return value
    raise TypeError(f'nonprimitive config value: {type(value).__name__}')


def resume_semantics(config):
    """Full replay continuation, including count units and experience meanings."""
    record = config_record(config)
    learning = {key: value for key, value in record['learning'].items()
                if key not in ('microbatch_size','learning_rate','polyak','initial_alpha',
                               'temperature_learning_rate')}
    platform = {key: value for key, value in record['platform'].items()
                if key != 'capability_path'}
    result = dict(model=record['model'], learning=learning,
        observation_schema=OBSERVATION_SCHEMA, action_schema='local_metric_pose_v3',
        privileged_schema=PRIVILEGED_SCHEMA, learner_schema=LEARNER_SCHEMA,
        frontier_semantics=frontier_semantics(),
        graph_geometry=dict(coverage_radius_m=2.,connection_limit_m=8.,stretch=1.2,
            base_order='clearance_desc_yx_v1'),
        observation_model='finite_center_tip_prefix_v1', generator_version=5,
        observation_origin='actual_pose_optical_offset',
        effective_measurement='classified_center_native_3x3_no_hidden_neighbors_v1',
        reward=FRONTIER_SEMANTICS['reward_formula'],
        completion_bonus=config.completion_bonus,
        success_coverage=config.success_coverage,
        termination='truth_coverage_or_current_opportunity_exhaustion_v1',
        graph_normalization=dict(position_m=10., frontier_cells=100.),
        sensor=record['sensor'], platform=platform,
        integration_step_s=config.integration_step_s, observation_hz=config.observation_hz,
        goal_position_tolerance_m=config.goal_position_tolerance_m,
        schedule=dict(warmup=config.warmup, ratio=config.update_ratio),
        curriculum_budgets=record['curriculum_budgets'],
        curriculum_extents_m=record['curriculum_extents_m'])
    if config.ordered_curriculum:
        result['ordered_curriculum'] = dict(version='ordered_task_transfer_v1',
            tasks=['moon_inside', 'cave_inside', 'moon_outside', 'cave_outside'],
            start_sampling='seeded_main_component_task_mask_v1',
            rehearsal_fraction=config.ordered_rehearsal_fraction,
            budget_rule=record['ordered_budget_rule'])
        if config.ordered_rehearsal_by_phase is not None:
            result['ordered_curriculum']['rehearsal_by_phase'] = record['ordered_rehearsal_by_phase']
    # The default fine experience predates these controls; omitting its entry
    # retains exact resume compatibility with the existing checkpoint contract.
    if config.policy_graph_mode == 'a2':
        result['policy_graph'] = dict(version='a2_region_remote_frontier_v1',
            remote_frontier_positions=config.remote_frontier_positions)
    if config.course_two:
        result['course_two']=dict(version='region_sensor_course_two_v1',
            scene='region_sensor_course_two_scene_v1',sensor_scope='immutable_scenario',
            range_m=[10,100],fov_deg=[60,90,120,180,360],max_blind_gap_m=20,
            stages=['A1','A2','B','C','D'],old_rehearsal=.2)
        if config.course_two_variant:
            result['course_two'].update(version='region_sensor_rebalance_v2',
                variant=config.course_two_variant,catalog_sha=config.course_two_catalog_sha,
                scene='region_sensor_rebalanced_scene_v2' if config.course_two_variant in ('C2','C3')
                    else 'region_sensor_course_two_scene_v1',old_rehearsal=.5,
                stages=['A1'],learning_protocol='phase_episode_v2' if config.course_two_variant=='C3'
                    else 'task_weighted_v2')
            if config.course_two_task_scope:
                result['course_two'].update(task_scope=config.course_two_task_scope,
                    scene='region_sensor_rebalanced_scene_v3',replay_scope='exclude_legacy_cave_v1')
            if config.course_two_sampling:
                result['course_two'].update(learning_protocol=config.course_two_sampling,
                    scene='three_ordinary_scenes_v1',old_rehearsal=0.,tasks=['moon_outside','moon_inside','cave_inside'])
            if config.course_two_scene_profile:
                from .course_two_sensors import semantics
                result['course_two'].update(semantics())
    if config.paired_scene_sequence:
        result['scene_sequence'] = dict(mode='paired_slot_episode_v1',
            seed=config.seed, environments=config.environments)
    # Disabled collection limits add no collection constraint.
    if config.stagnation_window_steps:
        result['training_stagnation'] = dict(mode='observed_no_gain_pose_window_v1',
            window_steps=config.stagnation_window_steps,
            max_positions=config.stagnation_max_positions,
            position_tolerance_m=config.stagnation_position_tolerance_m)
    return result


def training_config_from_record(record):
    """Reconstruct a JSON/spawn configuration without importing models or terrain."""
    values = dict(record)
    values['model'] = ModelConfig(**values['model'])
    values['learning'] = LearningConfig(**values['learning'])
    values['sensor'] = SensorSpec(**values['sensor'])
    platform = dict(values['platform'])
    platform['capability'] = MappingProxyType(dict(platform['capability']))
    values['platform'] = PlatformConfig(**platform)
    if 'curriculum_transition_boundaries' in values:
        values['curriculum_transition_boundaries'] = tuple(values['curriculum_transition_boundaries'])
    if 'curriculum_mixtures' in values:
        values['curriculum_mixtures'] = tuple(tuple(row) for row in values['curriculum_mixtures'])
    values['curriculum_budgets'] = tuple(values['curriculum_budgets'])
    if values.get('ordered_rehearsal_by_phase') is not None:
        values['ordered_rehearsal_by_phase'] = tuple(values['ordered_rehearsal_by_phase'])
    values['curriculum_extents_m'] = tuple(tuple(pair) for pair in values['curriculum_extents_m'])
    return TrainingConfig(**values)
