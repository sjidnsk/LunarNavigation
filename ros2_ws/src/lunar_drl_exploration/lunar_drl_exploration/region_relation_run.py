"""One fresh v2 run: qualified maps, exact budgets, one durable ClearML task."""
from dataclasses import replace
from fractions import Fraction
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from .course_two_control import read, write, sha, runtime_digest, root_lock, process_ticks

SEED = 20261014
TASKS = ('moon_inside', 'moon_outside', 'cave_inside')
BLOCK_ADMISSIONS = 8192
BLOCK_UPDATES = 2048
BLOCKS = 24


def make_config(root, cave_manifest_sha):
    from .config import TrainingConfig, ModelConfig, LearningConfig
    root = Path(root).resolve()
    return TrainingConfig(seed=SEED,
        model=ModelConfig(architecture_version='region_relation_v2', actor_score_bound=10.),
        learning=LearningConfig(batch_size=64, microbatch_size=8, learning_rate=1e-5,
            gamma=1., polyak=.005, initial_alpha=5e-5, maximum_alpha=1e-4,
            maximum_position_alpha=1e-3, entropy_mode='position_heading',
            temperature_learning_rate=.003, position_target_entropy_factor=.1,
            heading_target_entropy_factor=.1, return_steps=1, position_aux_weight=0.),
        course_two=True, ordered_curriculum=False, course_two_variant='C3',
        course_two_task_scope='whole_cave_v1', course_two_sampling='scene_episode_v1',
        course_two_scene_profile='distance_sensor_10_30_v1',
        course_two_scene_tasks=('moon_outside','cave_inside'),
        moon_entry_curriculum='near_to_far_v1',
        cave_generator_version='cave_topology_v2', cave_split_manifest_hash=cave_manifest_sha,
        policy_graph_mode='a2', remote_frontier_positions=32, environments=8,
        warmup=1024, update_ratio=.25, max_update_credit=32, actor_publish_updates=16,
        completion_bonus=20., success_coverage=.99,
        exploration_reward_scale=.5,
        approach_reward=dict(mode='bounded_exponential_v1', bound=2., scale_m=50.),
        stagnation_window_steps=128, stagnation_max_positions=8, stagnation_position_tolerance_m=.25,
        curriculum_extents_m=((30, 100), (80, 150), (100, 300)),
        ordered_budget_rule=dict(mode='linear_area', min_extent_m=50., max_extent_m=100.,
            round_to=32, min_decisions=256, max_decisions=512),
        replay_max_bytes=8*1024**3, snapshot_max_bytes=8*1024**3,
        total_output_max_bytes=32*1024**3, system_reserve_bytes=8*1024**3,
        artifact_root=str(root), output_dir=str(root/'run'), reward_diagnostics=True)


def stage_budget(schedule, block):
    if type(block) is not int or not 0 <= block <= BLOCKS:
        raise ValueError('invalid learning block')
    target = schedule.get('prefill_transitions', 0) + 1024 + block*BLOCK_ADMISSIONS
    remaining = target - schedule['transitions']
    if remaining < 0:
        raise ValueError('checkpoint exceeds frozen block boundary')
    return remaining


def qualification_eligible(evidence, task):
    return (task in TASKS and not evidence['initial_completed'] and not evidence['collision']
        and math.isfinite(evidence['initial_coverage']) and 0 <= evidence['initial_coverage'] < .99
        and evidence['actions'] > 0 and evidence['reference_area_m2'] > 0
        and evidence['inside'] == (task != 'moon_outside'))


def physical_terrain_sha256(terrain):
    """Physical grid identity; excludes seeds, split names and scenario IDs."""
    import hashlib
    import numpy as np
    digest = hashlib.sha256()
    digest.update(json.dumps(dict(shape=list(terrain.shape), origin=list(terrain.origin),
        resolution_m=float(terrain.resolution_m)), sort_keys=True).encode())
    heights = np.asarray(terrain.heights, dtype='<f4').copy()
    heights[heights == 0] = 0  # Canonicalize signed zero.
    heights[np.isnan(heights)] = np.float32(np.nan)
    digest.update(heights.tobytes(order='C'))
    return digest.hexdigest()


def pilot_gate(progress, readiness):
    learning = progress.get('learning', {})
    required = ('critic1_loss', 'critic2_loss', 'actor_loss', 'alpha_loss',
                'actor_grad_norm', 'q1_grad_norm', 'q2_grad_norm', 'temperature_grad_norm',
                'alpha_position', 'alpha_heading', 'log_alpha_position', 'log_alpha_heading')
    return (readiness.get('passed') is True
        and all(readiness.get('checks', {}).get(k) is True
                for k in ('gradient_isolation', 'checkpoint_restore', 'cuda_update', 'ros_closed_loop'))
        and progress.get('updates') == 4096 and progress.get('balanced_replay', {}).get('ready') is True
        and learning.get('numeric_guard_version') == 1
        and learning.get('numeric_checked_updates') == 4096
        and all(isinstance(learning.get(k), (int, float)) and math.isfinite(learning[k]) for k in required)
        and learning['alpha_position'] > 0 and learning['alpha_heading'] > 0)


def validate_run_identity(frozen, actual):
    for key in ('runtime', 'config_sha', 'manifests'):
        if frozen.get(key) != actual.get(key):
            raise ValueError('frozen run identity changed: '+key)


def validate_readiness(readiness, identity):
    for key in ('run_id', 'runtime', 'config_sha'):
        if not identity.get(key) or readiness.get(key) != identity[key]:
            raise ValueError('runtime readiness identity changed: '+key)
    if (readiness.get('passed') is not True or not all(
            readiness.get('checks', {}).get(k) is True for k in
            ('gradient_isolation','checkpoint_restore','cuda_update','ros_closed_loop'))):
        raise ValueError('runtime readiness checks are incomplete')
    evidence = readiness.get('evidence', {})
    if not evidence or any(not Path(p).is_file() or sha(p) != digest for p, digest in evidence.items()):
        raise ValueError('runtime readiness evidence is missing or changed')


def checkpoint_schedule(root, *, replay_ready=False):
    import torch
    from .checkpoint import TrainingState
    from .config import training_config_from_record
    record = torch.load(Path(root)/'run/resume.pt', map_location='cpu', weights_only=False)
    config = training_config_from_record(read(Path(root)/'config.json'))
    replay = TrainingState(record).validate(config, materialize_replay=replay_ready)
    result = dict(record['schedule'])
    if replay_ready:
        index = replay.enable_course_two_sampling(completion_bonus=config.completion_bonus,
            task_scope=config.course_two_task_scope, sampling=config.course_two_sampling,
            scene_profile=config.course_two_scene_profile, exploration_reward_scale=config.exploration_reward_scale,
            scene_tasks=config.course_two_scene_tasks)
        result['prefill_ready'] = index.status()['ready']
    return result


def qualify_case(root, request_path, output, slot):
    """Geometry-only eligibility, before policy evaluation; fixed backup order."""
    from .config import training_config_from_record
    from .ros_env import RosExplorationEnv
    from .course_two_prelude import inside_task
    root, output = Path(root), Path(output)
    request = read(request_path)
    proof = dict(request=request, config_sha=sha(root/'config.json'), runtime=runtime_digest())
    if output.exists():
        cached = read(output)
        if cached.get('proof') != proof or cached.get('status') != 'passed':
            raise ValueError('qualification cache is failed or changed')
        return cached
    config = replace(training_config_from_record(read(root/'config.json')),
                     environments=1, stagnation_window_steps=0)
    env = RosExplorationEnv(config, slot, domain_base=200)
    rejected = []
    try:
        for attempt, spec in enumerate(request['backup_specs']):
            try:
                env.reset(**spec)
                evidence = dict(initial_coverage=env.reference.coverage_ratio(env.privileged.observed),
                    initial_completed=bool(env.completed), collision=bool(env.plant.collision),
                    actions=len(env.observation.action_nodes), reference_area_m2=env.reference.area_m2,
                    inside=bool(inside_task(env)), terrain_id=env.terrain.terrain_id,
                    terrain_sha256=physical_terrain_sha256(env.terrain),
                    reference_id=env.reference.reference_id, metadata=env.episode_metadata)
                if not qualification_eligible(evidence, request['case']['capability']):
                    raise ValueError('initial scene has no nontrivial eligible policy decision: '+str(evidence))
                case = dict(request['case'], spec=spec)
                result = dict(status='passed', proof=proof, case=case, evidence=evidence,
                              rejected=rejected, accepted_attempt=attempt)
                write(output, result)
                return result
            except (ValueError, RuntimeError) as exc:
                rejected.append(dict(attempt=attempt, seed=spec['seed'], reason=str(exc)[:2000]))
                write(str(output)+'.attempts.json', dict(proof=proof, rejected=rejected))
            finally:
                env.close()
        write(output, dict(status='failed', proof=proof, rejected=rejected))
        raise RuntimeError('all predefined geometry backups failed for '+request['case']['case_id'])
    finally:
        env.close()


class Run:
    def __init__(self, root, *, clearml=False):
        self.root = Path(root).resolve()
        self.state = read(self.root/'state.json')
        self.identity = read(self.root/'identity.json')
        self.clearml = clearml
        self.stopping = False
        self.child = None
        self.evaluation_children = {}
        self.evaluation_signalled = set()
        self.tracking = None

    def save(self, **fields):
        self.state.update(fields, updated_unix=time.time())
        write(self.root/'state.json', self.state)

    def interrupt(self, *_):
        self.stopping = True
        if self.child is not None and self.child.poll() is None:
            self.child.send_signal(signal.SIGINT)
        from .course_two_parallel import interrupt_children
        interrupt_children(self)

    def _report_progress(self):
        path = self.root/'run/progress.json'
        if self.tracking is None or not path.exists():
            return
        data = read(path)
        iteration = data['transitions']
        for title, values in (('training', data), ('learning', data.get('learning', {})),
                              ('resources', data.get('resources', {})),
                              ('gpu_memory', data.get('gpu_memory') or {})):
            self.tracking.report_metrics(values, iteration=iteration, title=title)
        for task, values in data.get('course', {}).get('by_task', {}).items():
            self.tracking.report_metrics(values, iteration=iteration, title=task)

    def train_to(self, block=None, *, prefill=False):
        from .processes import _LAUNCH
        existing = (self.root/'run/resume.pt').exists()
        current = checkpoint_schedule(self.root, replay_ready=prefill) if existing else dict(transitions=0, updates=0, prefill_transitions=0)
        remaining = 4096-current['transitions'] if prefill else stage_budget(current, block)
        if prefill and (current['updates'] or current['transitions'] != current.get('prefill_transitions', 0)):
            raise ValueError('prefill cannot reuse any learned/foreign checkpoint')
        if prefill and current.get('prefill_ready'):
            self.save(prefill_admissions=current['transitions'], prefill_done=True)
            return current
        if not prefill and not remaining and Fraction(current['credit']) < 1:
            return current
        if prefill and remaining <= 0:
            raise RuntimeError('prefill limit reached without recorded readiness')
        args = [sys.executable, '-m', 'lunar_drl_exploration.cli', 'train', '--config',
                str(self.root/'config.json'), '--max-transitions', str(remaining), '--domain-base', '180']
        if existing:
            args.append('--resume')
        if prefill:
            args.append('--collect-only')
        label = 'prefill' if prefill else f'block-{block:02}'
        self.save(phase=label)
        with (self.root/'logs'/f'{label}.log').open('ab') as stream:
            self.child = subprocess.Popen([sys.executable, '-c', _LAUNCH, str(os.getpid()), *args],
                stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                self.save(child_pid=self.child.pid)
                while self.child.poll() is None:
                    time.sleep(2)
                    self._report_progress()
                code = self.child.returncode
            finally:
                if self.child.poll() is None:
                    self.child.send_signal(signal.SIGINT)
                    try:
                        self.child.wait(timeout=30)
                    except subprocess.TimeoutExpired:
                        self.child.kill()
                        self.child.wait(timeout=5)
                self.child = None
        if code:
            raise RuntimeError(f'{label} exited {code}; see logs/{label}.log')
        result = checkpoint_schedule(self.root)
        if self.stopping:
            return result
        if prefill:
            run = read(self.root/'run/run.json')['observations']['final']
            if result['updates'] or not run['balanced_replay']['ready']:
                raise RuntimeError('prefill did not preserve ready three-scene replay')
            self.save(prefill_admissions=result['transitions'], prefill_done=True)
        elif stage_budget(result, block) or result['updates'] != block*BLOCK_UPDATES or Fraction(result['credit']) >= 1:
            raise RuntimeError('training did not reach exact admission/update boundary')
        self._report_progress()
        return result

    def screen(self, actor, split, label):
        from .course_two_parallel import screen
        from .region_relation_evaluation import summarize
        manifest = read(self.root/f'{split}.json')
        rows = screen(self, actor, manifest['cases'], label)
        if rows is None:
            return None
        cases = {c['case_id']: c for c in manifest['cases']}
        for row in rows:
            case = cases[row['case_id']]
            if 'mother_graph_id' in case['spec']:
                row['mother_graph_id'] = case['spec']['mother_graph_id']
            row['dataset_split'] = split
        result = summarize(manifest, rows)
        write(self.root/'evaluation'/label/'gate.json', result)
        if self.tracking:
            for task, values in result['by_task'].items():
                self.tracking.report_metrics(values, iteration=self.state.get('block', 0),
                                             title=split+'/'+task)
        return dict(actor=str(actor), block=self.state.get('block', 0), rows=rows, gate=result)

    def run(self):
        from .clearml_tracking import ClearMLTracking
        from .evaluation import export_actor
        from .region_relation_evaluation import select_validation_candidate
        actual = dict(runtime=runtime_digest(), config_sha=sha(self.root/'config.json'),
            manifests={split:sha(self.root/f'{split}.json') for split in ('validation', 'test', 'quick')})
        validate_run_identity(self.identity, actual)
        if not self.identity.get('qualified'):
            raise ValueError('qualified data and reviewed runtime readiness are required')
        validate_readiness(read(self.root/'readiness.json'), self.identity)
        with root_lock(self.root):
            old_handlers = {s:signal.signal(s, self.interrupt) for s in (signal.SIGINT, signal.SIGTERM)}
            write(self.root/'owner.json', dict(pid=os.getpid(), start_ticks=process_ticks(os.getpid()), root=str(self.root)))
            self.save(status='running', controller_pid=os.getpid())
            try:
                self.tracking = ClearMLTracking.open(self.root, enabled=self.clearml,
                    config=read(self.root/'config.json'), protocol=read(self.root/'protocol.json'), identity=self.identity)
                if not self.state.get('prefill_done'):
                    self.train_to(prefill=True)
                if self.stopping:
                    return
                if not self.state.get('warmup_done'):
                    self.train_to(0)
                    if self.stopping:
                        return
                    self.save(warmup_done=True)
                for block in range(self.state.get('completed_blocks', 0)+1, BLOCKS+1):
                    self.save(block=block)
                    self.train_to(block)
                    if self.stopping:
                        return
                    actor = self.root/'actors'/f'block-{block:02}.pt'
                    if not actor.exists():
                        export_actor(self.root/'run/resume.pt', actor)
                    quick = self.screen(actor, 'quick', f'block-{block:02}-quick')
                    if quick is None:
                        return
                    if block % 4 == 0:
                        full = self.screen(actor, 'validation', f'block-{block:02}-validation')
                        if full is None:
                            return
                        history = self.state.get('validation_candidates', [])
                        history = [c for c in history if c['block'] != block] + [full]
                        self.save(validation_candidates=history)
                    if block == 2:
                        # Startup qualification is numeric/operational; success rate is not a pilot gate.
                        progress = read(self.root/'run/run.json')['observations']['final']
                        readiness = read(self.root/'readiness.json')
                        if not pilot_gate(progress, readiness):
                            raise RuntimeError('two-block infrastructure gate failed')
                        write(self.root/'pilot-evidence.json', dict(passed=True,
                            run_id=self.identity['run_id'], runtime=self.identity['runtime'],
                            checkpoint_sha=sha(self.root/'run/resume.pt'),
                            readiness_sha=sha(self.root/'readiness.json'), progress=progress))
                        self.save(pilot_passed=True)
                    self.save(completed_blocks=block)
                run_id = self.identity['run_id']
                candidates = [dict(checkpoint=c['actor'], run_id=run_id, results=c['rows'])
                              for c in self.state['validation_candidates']]
                selected = select_validation_candidate(read(self.root/'validation.json'), candidates, run_id=run_id)
                self.save(selected_actor=selected['checkpoint'], phase='sealed_test')
                final = self.screen(Path(selected['checkpoint']), 'test', 'final-selected-test')
                if final is None:
                    return
                self.tracking.upload_final_actor(selected['checkpoint'])
                self.save(status='complete', phase='complete', final_test=final['gate'])
            except BaseException as exc:
                self.save(status='needs_attention', error=f'{type(exc).__name__}: {exc}')
                raise
            finally:
                if self.stopping:
                    self.save(status='paused')
                if self.tracking is not None:
                    outcome=('completed' if self.state.get('status') == 'complete' else
                             'stopped' if self.stopping else 'failed')
                    self.tracking.close(outcome=outcome, message=self.state.get('error'))
                for sig, handler in old_handlers.items():
                    signal.signal(sig, handler)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('command', choices=('run', 'qualify-one'))
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--clearml', action='store_true')
    p.add_argument('--request', type=Path)
    p.add_argument('--output', type=Path)
    p.add_argument('--slot', type=int, default=0)
    args = p.parse_args()
    if args.command == 'run':
        Run(args.root, clearml=args.clearml).run()
    else:
        qualify_case(args.root, args.request, args.output, args.slot)


if __name__ == '__main__':
    main()
