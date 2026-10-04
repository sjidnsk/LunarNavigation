"""Ordered task curriculum with learner-owned admission and sample counters."""
from collections import Counter
from copy import deepcopy
import math

TASKS = ('moon_inside', 'cave_inside', 'moon_outside', 'cave_outside')
SCHEMA = 'ordered_course_v1'
DEFAULT_BUDGET_RULE = dict(mode='linear_area', min_extent_m=50., max_extent_m=100.,
    round_to=32, min_decisions=256, max_decisions=512)
_COUNTERS = ('transitions', 'samples', 'ended', 'successes', 'episode_steps')


def task_weights(phase, rehearsal=.2):
    if type(phase) is not int or not 0 <= phase < len(TASKS):
        raise ValueError('course phase must be an integer from 0 through 3')
    if type(rehearsal) not in (float, int) or not math.isfinite(rehearsal) or not 0 < rehearsal < 1:
        raise ValueError('course rehearsal fraction must be between zero and one')
    return ({TASKS[0]: 1.} if phase == 0 else
        dict([(task, float(rehearsal) / phase) for task in TASKS[:phase]] +
             [(TASKS[phase], 1. - float(rehearsal))]))


def area_budget(extent, rule=None):
    """Same bounded area interpolation and half-up quantization as eval budgets."""
    rule = DEFAULT_BUDGET_RULE if rule is None else rule
    if type(extent) not in (float, int) or not math.isfinite(extent) or extent <= 0:
        raise ValueError('positive finite course extent required')
    if (not isinstance(rule, dict) or set(rule) != set(DEFAULT_BUDGET_RULE) or
            rule['mode'] != 'linear_area'):
        raise ValueError('course requires a linear_area budget rule')
    low, high = rule['min_extent_m'], rule['max_extent_m']
    quantum, minimum, maximum = (rule[key] for key in ('round_to', 'min_decisions', 'max_decisions'))
    if (any(type(v) not in (float, int) or not math.isfinite(v) or v <= 0 for v in (low, high))
            or low >= high or any(type(v) is not int or v <= 0 for v in (quantum, minimum, maximum))
            or minimum > maximum or minimum % quantum or maximum % quantum):
        raise ValueError('invalid course budget bounds')
    bounded = min(high, max(low, extent))
    raw = minimum + (bounded**2-low**2)/(high**2-low**2) * (maximum-minimum)
    return min(maximum, max(minimum, math.floor(raw/quantum+.5)*quantum))


class OrderedCourse:
    """One label per episode; quotas count successful replay admission only.

    Active episode ownership is deliberately not restored: continuation restarts
    workers, so unfinished episodes never become ended episodes or join traces.
    Per-phase admission counts and cumulative statistics do survive every block.
    """
    def __init__(self, phase, rehearsal=.2, state=None, success_coverage=.99):
        self.phase, self.weights = phase, task_weights(phase, rehearsal)
        self.success_coverage = success_coverage
        self.active, self.episodes = {}, {}
        self.by_task = {task: dict.fromkeys(_COUNTERS, 0) for task in TASKS}
        self.phase_transitions = dict.fromkeys(self.weights, 0)
        self.phase_samples = dict.fromkeys(self.weights, 0)
        self.bootstrap_updates = 0
        self.missing_replay_tasks = []
        self.last_sampling_weights = {}
        if state is not None:
            if state.get('schema') != SCHEMA:
                raise ValueError('saved curriculum is not an ordered course')
            previous = state.get('phase')
            if type(previous) is not int or phase not in (previous, previous + 1):
                raise ValueError('course resume requires same phase or next phase')
            if state.get('rehearsal') != rehearsal:
                raise ValueError('course rehearsal fraction changed on resume')
            self.by_task = deepcopy(state['by_task'])
            if set(self.by_task) != set(TASKS) or any(
                    set(stats) != set(_COUNTERS) or any(type(v) is not int or v < 0 for v in stats.values())
                    for stats in self.by_task.values()):
                raise ValueError('invalid saved course counters')
            self.bootstrap_updates = state['bootstrap_updates']
            if phase == previous:
                self.phase_transitions = deepcopy(state['phase_transitions'])
                self.phase_samples = deepcopy(state['phase_samples'])
                for counts in (self.phase_transitions, self.phase_samples):
                    if set(counts) != set(self.weights) or any(type(v) is not int or v < 0 for v in counts.values()):
                        raise ValueError('invalid saved course phase counters')
                self.last_sampling_weights = deepcopy(state.get('sampling_weights', {}))
                self.missing_replay_tasks = list(state.get('missing_replay_tasks', []))
        self.rehearsal = float(rehearsal)

    def next_task(self):
        # Guarantee at least one live slot for each positive-weight class. Then
        # distribute remaining slots by weight; GRANT controls actual admissions.
        counts = Counter(self.active.values())
        absent = [task for task in self.weights if not counts[task]]
        if absent:
            return max(absent, key=lambda task: self.weights[task])
        total = sum(counts.values()) + 1
        return max(self.weights, key=lambda task: self.weights[task]*total-counts[task])

    def activate(self, env, task):
        if task not in self.weights:
            raise ValueError('episode task outside active course phase')
        self.active[env] = task
        self.episodes[env] = dict(steps=0, collision=False)

    def reset_episode(self, env):
        # Recovery keeps the task but starts a new physical episode.
        if env in self.active:
            self.episodes[env] = dict(steps=0, collision=False)

    def release(self, env):
        self.active.pop(env, None)
        self.episodes.pop(env, None)

    def select_pending(self, pending, reservations):
        """Backpressure fast classes while their most-owed peer is still active.

        Including reserved actions bounds asynchronous admitted-share error by
        the number of environment slots, independent of episode length.
        """
        projected = Counter(self.phase_transitions)
        projected.update(self.active[env] for env in reservations if env in self.active)
        total = sum(projected.values()) + 1
        desired = max(self.weights, key=lambda task: self.weights[task]*total-projected[task])
        candidates = [env for env in pending if self.active.get(env) == desired and env not in reservations]
        return min(candidates) if candidates else None

    def admitted(self, env):
        task = self.active[env]
        self.by_task[task]['transitions'] += 1
        self.phase_transitions[task] += 1
        self.episodes[env]['steps'] += 1

    def observe(self, env, *, terminated, truncated, coverage, collision):
        task = self.active[env]
        episode = self.episodes[env]
        episode['collision'] |= bool(collision)
        if terminated or truncated:
            stats = self.by_task[task]
            stats['ended'] += 1
            stats['successes'] += int(terminated and not episode['collision'] and coverage >= self.success_coverage)
            stats['episode_steps'] += episode['steps']
            self.release(env)

    def sampling_weights(self, counts):
        missing_retained = [task for task in TASKS[:self.phase] if counts.get(task, 0) <= 0]
        if missing_retained:
            raise ValueError('ordered replay lost retained tasks: '+', '.join(missing_retained))
        current = TASKS[self.phase]
        if self.by_task[current]['transitions'] and counts.get(current, 0) <= 0:
            raise ValueError('ordered replay lost previously admitted current task: '+current)
        available = {task: weight for task, weight in self.weights.items() if counts.get(task, 0) > 0}
        if not available:
            raise ValueError('ordered replay has no samples for this phase')
        self.missing_replay_tasks = [task for task in self.weights if task not in available]
        total = sum(available.values())
        self.last_sampling_weights = {task: weight / total for task, weight in available.items()}
        return dict(self.last_sampling_weights)

    def sampled(self, tasks):
        for task, count in Counter(tasks).items():
            if task not in self.weights:
                raise ValueError('sample task outside active course phase')
            self.by_task[task]['samples'] += count
            self.phase_samples[task] += count
        self.bootstrap_updates += int(bool(self.missing_replay_tasks))

    def summary(self):
        return dict(schema=SCHEMA, phase=self.phase, task=TASKS[self.phase],
            rehearsal=self.rehearsal, weights=dict(self.weights), by_task=deepcopy(self.by_task),
            phase_transitions=dict(self.phase_transitions), phase_samples=dict(self.phase_samples),
            bootstrap_updates=self.bootstrap_updates,
            missing_replay_tasks=list(self.missing_replay_tasks), sampling_weights=dict(self.last_sampling_weights))

    def state_dict(self):
        return self.summary()
