"""Fixed-capacity, admission-only reward summaries and selected scalar cases.

No model inference, replay copies, maps, or tensors are retained. The caller owns
transaction deduplication and calls observe only after a new replay admission.
Episode suffixes describe the actors actually used, not a frozen-policy return.
"""
from copy import deepcopy
import json
import math


_COVERAGE = ('below_80', '80_to_95', '95_to_99', 'at_least_99')
_FAMILIES = ('moon', 'cave', 'other')
_SIZES = ('small', 'medium', 'large')
_CLOSE_REASONS = ('RESET', 'RECOVERY', 'RESUME', 'EPISODE_CHANGED', 'OTHER')
_TRUNCATION_REASONS = ('STAGNATION', 'DECISION_BUDGET', 'OTHER')
_OUTCOMES = ('success', 'truncated', 'terminated_other', 'interrupted')
_WINDOW = 128
_CASE_SLOTS = 8  # Eight success plus eight stagnation cases; no shared eviction.
_SCHEMA = 'bounded_frontier_reward_diagnostics_v2'
_STEP_FIELDS = frozenset(('actor_version', 'before_coverage', 'after_coverage',
    'new_area_m2', 'distance_m', 'turn_rad', 'reward', 'exploration_reward', 'motion_reward',
    'removed_frontiers_in', 'removed_frontiers_out', 'step_reward', 'completion_reward', 'wall_s', 'simulation_s',
    'terminated', 'truncated', 'success', 'truncation_reason', 'collision',
    'actual_x', 'actual_y', 'goal_x', 'goal_y', 'action'))
_EPISODE_FIELDS = frozenset(('episode_id', 'env_id', 'family', 'extent_m', 'seed',
    'segment_steps', 'reward_sum', 'actor_version_min', 'actor_version_max',
    'start_coverage', 'end_coverage', 'late', 'steps'))
_CASE_FIELDS = _EPISODE_FIELDS | {'outcome', 'truncation_reason', 'window_is_complete_episode'}


def _finite(value, name, *, low=None, high=None):
    number = float(value)
    if not math.isfinite(number) or (low is not None and number < low) or (
            high is not None and number > high):
        raise ValueError(f'{name} must be finite and within bounds')
    return number


def _integer(value, name, *, low=0, high=2**63 - 1):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ValueError(f'{name} must be an integer within bounds')
    return value


def _versions(record, version):
    for key, select in (('actor_version_min', min), ('actor_version_max', max)):
        record[key] = version if record[key] is None else select(record[key], version)


def _stats():
    return dict(admitted=0, positive_gain=0, zero_gain=0,
        positive_gain_negative_reward=0, reward_sum=0., new_area_m2=0.,
        removed_frontiers_in=0, removed_frontiers_out=0, distance_m=0., turn_rad=0., wall_s=0., simulation_s=0., zero_gain_wall_s=0.,
        success=0, terminated_other=0, truncated=0, collisions=0,
        truncation_reasons={reason: 0 for reason in _TRUNCATION_REASONS},
        reward_components=dict(exploration=0., motion=0., step=0., completion=0.),
        reward_residual_sum=0., reward_mismatch_count=0,
        actor_version_min=None, actor_version_max=None)


def _tail():
    return dict(segments=0, steps=0, reward_sum=0., actor_version_min=None,
                actor_version_max=None)


def _merge_tail(target, source):
    if not source['steps']:
        return
    target['segments'] += 1
    target['steps'] += source['steps']
    target['reward_sum'] += source['reward_sum']
    _versions(target, source['actor_version_min'])
    _versions(target, source['actor_version_max'])


class RewardDiagnostics:
    """Bounded summaries of newly admitted transitions, persisted in checkpoints.

    Restore closes in-flight episode segments as interrupted by RESUME. A new
    simulator episode cannot be appended to an old episode's observed return.
    At most 233 environments are accepted (the available DDS domain ID count).
    """

    def __init__(self, environments, completion_bonus=5., success_coverage=.99, state=None):
        self.environments = _integer(environments, 'environments', low=1, high=233)
        self.completion_bonus = _finite(completion_bonus, 'completion_bonus', low=0.)
        self.success_coverage = _finite(success_coverage, 'success_coverage', low=0., high=1.)
        self._state = dict(schema=_SCHEMA, environments=self.environments,
            completion_bonus=self.completion_bonus, success_coverage=self.success_coverage,
            totals=_stats(), by_coverage={key: _stats() for key in _COVERAGE},
            by_family={key: _stats() for key in _FAMILIES},
            by_size={key: _stats() for key in _SIZES},
            late_episode_segments={key: _tail() for key in _OUTCOMES},
            segments_closed_by_reason={key: 0 for key in _CLOSE_REASONS},
            active=[None] * self.environments, success_cases=[], stagnation_cases=[])
        if state is not None:
            self._restore(state)
            for env_id in range(self.environments):
                self.reset(env_id, 'RESUME')

    def _env(self, env_id):
        return _integer(env_id, 'env_id', high=self.environments - 1)

    def observe(self, env_id, transition, *, before_coverage, after_coverage,
                family, extent_m, execution=None, truncation_reason=None, seed=None):
        """Account one admission using actual reward parts and reference coverage."""
        env_id = self._env(env_id)
        before = _finite(before_coverage, 'before_coverage', low=0., high=1.)
        after = _finite(after_coverage, 'after_coverage', low=0., high=1.)
        extent = _finite(extent_m, 'extent_m', low=0.)
        if extent == 0.:
            raise ValueError('extent_m must be positive')
        family = family if family in _FAMILIES else 'other'
        size = 'small' if extent <= 80. else 'medium' if extent <= 150. else 'large'
        coverage = _COVERAGE[0 if before < .8 else 1 if before < .95 else 2 if before < .99 else 3]
        episode_id = transition.episode_id
        if not isinstance(episode_id, str) or len(episode_id) > 128:
            raise ValueError('episode_id must be a bounded string')
        version = _integer(transition.actor_version, 'actor_version')
        seed = _integer(seed, 'seed', low=-(2**63)) if seed is not None else None
        area = _finite(transition.parts.new_area_m2, 'new_area_m2', low=-1e-8)
        distance = _finite(transition.parts.distance_m, 'distance_m', low=-1e-8)
        turn = _finite(transition.parts.turn_rad, 'turn_rad', low=-1e-8)
        reward = _finite(transition.reward, 'reward')
        execution = execution or {}
        wall = _finite(execution.get('wall_s', 0.), 'wall_s', low=0.)
        simulation = _finite(execution.get('simulation_s', 0.), 'simulation_s', low=0.)
        terminated = bool(transition.terminated)
        truncated = bool(transition.truncated and not terminated)
        collision = execution.get('reason_code') == 'COLLISION' or bool(execution.get('geometry_failure', False))
        success = terminated and not collision and after >= self.success_coverage
        reason = (truncation_reason if truncation_reason in _TRUNCATION_REASONS else 'OTHER') if truncated else None
        removed_in = _integer(transition.parts.removed_frontiers_in, 'removed_frontiers_in')
        removed_out = _integer(transition.parts.removed_frontiers_out, 'removed_frontiers_out')
        components = dict(exploration=removed_in / 50.,
            motion=-.002 * distance - .005 * turn / math.pi, step=-.001,
            completion=self.completion_bonus if success else 0.)
        residual = reward - sum(components.values())
        row = dict(actor_version=version, before_coverage=before, after_coverage=after,
            new_area_m2=area, distance_m=distance, turn_rad=turn, reward=reward,
            exploration_reward=components['exploration'], motion_reward=components['motion'],
            removed_frontiers_in=removed_in, removed_frontiers_out=removed_out, step_reward=components['step'],
            completion_reward=components['completion'], wall_s=wall, simulation_s=simulation,
            terminated=terminated, truncated=truncated, success=success,
            truncation_reason=reason, collision=collision)
        # Keep scalar endpoints only, never graph arrays or full execution records.
        for prefix, field in (('actual', 'actual_pose'), ('goal', 'goal')):
            position = execution.get(field)
            for axis, index in (('x', 0), ('y', 1)):
                row[f'{prefix}_{axis}'] = (_finite(position[index], field)
                    if position is not None and len(position) > index else None)
        action = getattr(transition, 'action', None)
        row['action'] = _integer(action, 'action') if action is not None else None

        # Validation above is atomic with respect to accumulated statistics.
        active = self._state['active'][env_id]
        if active is not None and active['episode_id'] != episode_id:
            self.reset(env_id, 'EPISODE_CHANGED')
            active = None
        if active is None:
            active = dict(episode_id=episode_id, env_id=env_id, family=family,
                extent_m=extent, seed=seed, segment_steps=0, reward_sum=0.,
                actor_version_min=None, actor_version_max=None, start_coverage=before,
                end_coverage=before, late=_tail(), steps=[])
            self._state['active'][env_id] = active
        for stats in (self._state['totals'], self._state['by_coverage'][coverage],
                      self._state['by_family'][family], self._state['by_size'][size]):
            stats['admitted'] += 1
            stats['positive_gain'] += int(area > 0.)
            stats['zero_gain'] += int(area <= 0.)
            stats['positive_gain_negative_reward'] += int(area > 0. and reward < 0.)
            for key, value in (('reward_sum', reward), ('new_area_m2', area),
                    ('removed_frontiers_in', removed_in), ('removed_frontiers_out', removed_out),
                    ('distance_m', distance), ('turn_rad', turn), ('wall_s', wall),
                    ('simulation_s', simulation), ('zero_gain_wall_s', wall if area <= 0. else 0.)):
                stats[key] += value
            stats['success'] += int(success)
            stats['terminated_other'] += int(terminated and not success)
            stats['truncated'] += int(truncated)
            stats['collisions'] += int(collision)
            if truncated:
                stats['truncation_reasons'][reason] += 1
            for key, value in components.items():
                stats['reward_components'][key] += value
            stats['reward_residual_sum'] += residual
            stats['reward_mismatch_count'] += int(abs(residual) > 1e-8)
            _versions(stats, version)
        active['segment_steps'] += 1
        active['reward_sum'] += reward
        active['end_coverage'] = after
        _versions(active, version)
        active['steps'].append(row)
        del active['steps'][:-_WINDOW]
        if before >= .95:
            active['late']['steps'] += 1
            active['late']['reward_sum'] += reward
            _versions(active['late'], version)
        if terminated or truncated:
            outcome = 'success' if success else 'truncated' if truncated else 'terminated_other'
            _merge_tail(self._state['late_episode_segments'][outcome], active['late'])
            if success or reason == 'STAGNATION':
                case = deepcopy(active)
                case.update(outcome=outcome, truncation_reason=reason,
                    window_is_complete_episode=active['segment_steps'] <= _WINDOW)
                cases = self._state['success_cases' if success else 'stagnation_cases']
                cases.append(case)
                del cases[:-_CASE_SLOTS]
            self._state['active'][env_id] = None

    def reset(self, env_id, reason='RESET'):
        """Close an unfinished observed segment without inventing an RL terminal."""
        env_id = self._env(env_id)
        active = self._state['active'][env_id]
        if active is None:
            return
        reason = reason if reason in _CLOSE_REASONS else 'OTHER'
        _merge_tail(self._state['late_episode_segments']['interrupted'], active['late'])
        self._state['segments_closed_by_reason'][reason] += 1
        self._state['active'][env_id] = None

    def summary(self):
        """Cumulative admissions only; selected cases never supply rate estimates."""
        result = deepcopy({key: value for key, value in self._state.items()
                           if key not in ('active', 'success_cases', 'stagnation_cases')})
        result['scope'] = 'new admissions since diagnostics initialization; restored cumulatively; excludes source replay'
        result['coverage_bucket_basis'] = 'reference coverage before the admitted action'
        result['size_bucket_extent_m'] = dict(small='<=80', medium='>80 and <=150', large='>150')
        result['late_return_scope'] = ('observed reward sums for actions starting at >=95% coverage; '
            'not a frozen-policy return; success, truncation and interrupted segments stay separate')
        result['zero_gain_scope'] = 'zero gain means actual new_area_m2 <= 0; includes useful transit; not frontier reward or a loop classification'
        active_tail = _tail()
        for active in self._state['active']:
            if active is not None:
                _merge_tail(active_tail, active['late'])
        result['late_episode_segments']['active_incomplete'] = active_tail
        self._bounded_json(result, 1024**2)
        return result

    def cases(self):
        """Latest eight examples per outcome, each with at most 128 scalar steps."""
        result = dict(schema=_SCHEMA, scope='selected examples; cannot estimate population rates',
            max_steps_per_case=_WINDOW, slots_per_outcome=_CASE_SLOTS,
            success=deepcopy(self._state['success_cases']),
            stagnation=deepcopy(self._state['stagnation_cases']))
        self._bounded_json(result, 16 * 1024**2)
        return result

    def state_dict(self):
        """Detach bounded scalar state for the learner's checkpoint barrier."""
        self._bounded_json(self._state, 32 * 1024**2)
        return deepcopy(self._state)

    @staticmethod
    def _bounded_json(value, limit):
        try:
            encoded = json.dumps(value, allow_nan=False, separators=(',', ':')).encode('utf-8')
        except (TypeError, ValueError) as error:
            raise ValueError('diagnostic state must contain finite JSON primitives') from error
        if len(encoded) > limit:
            raise ValueError('diagnostic state exceeds bounded byte limit')

    def _restore(self, state):
        self._bounded_json(state, 32 * 1024**2)
        if not isinstance(state, dict) or set(state) != set(self._state) or state.get('schema') != _SCHEMA:
            raise ValueError('diagnostic state schema mismatch')
        if state['environments'] != self.environments:
            raise ValueError('diagnostic state environments mismatch')
        if state['completion_bonus'] != self.completion_bonus or state['success_coverage'] != self.success_coverage:
            raise ValueError('diagnostic reward semantics mismatch')
        for key, allowed in (('by_coverage', _COVERAGE), ('by_family', _FAMILIES),
                ('by_size', _SIZES), ('segments_closed_by_reason', _CLOSE_REASONS),
                ('late_episode_segments', _OUTCOMES)):
            if not isinstance(state[key], dict) or set(state[key]) != set(allowed):
                raise ValueError('diagnostic state bounded buckets mismatch')
        for key in ('totals', 'by_coverage', 'by_family', 'by_size',
                    'late_episode_segments', 'segments_closed_by_reason'):
            self._validate_shape(state[key], self._state[key])
        if not isinstance(state['active'], list) or len(state['active']) != self.environments:
            raise ValueError('diagnostic state bounded environment count mismatch')
        for key in ('success_cases', 'stagnation_cases'):
            if not isinstance(state[key], list) or len(state[key]) > _CASE_SLOTS:
                raise ValueError('diagnostic state exceeds bounded case slots')
        for key in ('active', 'success_cases', 'stagnation_cases'):
            for active in state[key]:
                if active is None and key == 'active':
                    continue
                fields = _EPISODE_FIELDS if key == 'active' else _CASE_FIELDS
                if not isinstance(active, dict) or set(active) != fields:
                    raise ValueError('diagnostic state bounded episode schema mismatch')
                if not isinstance(active['steps'], list) or len(active['steps']) > _WINDOW:
                    raise ValueError('diagnostic state exceeds bounded step window')
                self._validate_scalars({k: v for k, v in active.items() if k not in ('late', 'steps')})
                self._validate_shape(active['late'], _tail())
                for step in active['steps']:
                    if not isinstance(step, dict) or set(step) != _STEP_FIELDS:
                        raise ValueError('diagnostic state bounded step schema mismatch')
                    self._validate_scalars(step)
        self._state = deepcopy(state)

    @staticmethod
    def _validate_scalars(record):
        for value in record.values():
            if value is not None and type(value) not in (str, bool, int, float):
                raise ValueError('diagnostic state must contain bounded scalars only')
            if isinstance(value, str) and len(value) > 128:
                raise ValueError('diagnostic state string exceeds bounded length')

    @classmethod
    def _validate_shape(cls, value, template):
        if isinstance(template, dict):
            if not isinstance(value, dict) or set(value) != set(template):
                raise ValueError('diagnostic state bounded counter schema mismatch')
            for key in template:
                cls._validate_shape(value[key], template[key])
        elif type(template) is int:
            _integer(value, 'bounded diagnostic counter')
        elif type(template) is float:
            if type(value) not in (float, int):
                raise ValueError('diagnostic state bounded numeric schema mismatch')
        elif value is not None:
            _integer(value, 'bounded actor version')
