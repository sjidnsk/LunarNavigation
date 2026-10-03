"""Single learner owner of replay, exact update credit and continuation state."""


FINAL_CONFIRM_TIMEOUT_S = 12.


def within_admission_budget(admitted, inflight, *, baseline, maximum):
    """Reserve room for already granted actions before starting another one."""
    return maximum is None or admitted - baseline + inflight < maximum


def _merge_progress(status, progress):
    """Never attach an earlier episode's completion fields to reset progress."""
    if status.get('state') == 'RESETTING':
        return dict(status)
    if (progress.get('episode_id') is not None and status.get('episode_id') is not None
            and progress['episode_id'] != status['episode_id']):
        return dict(progress)
    return dict(status, **progress)


class AdmissionLedger:
    def __init__(self, replay, schedule):
        self.replay, self.schedule = replay, schedule
        self.active_scenes = {}
        self.last_admitted = {}

    def scene(self, env_id, scenes):
        if len(scenes) != 1:
            raise ValueError('exactly one active scene per environment required')
        self.active_scenes[env_id] = scenes

    def admit(self, env_id, token, transition):
        previous = self.last_admitted.get(env_id, -1)
        if token <= previous:
            return False
        self.replay.add(transition, self.active_scenes[env_id])
        self.schedule.collected(reserved=True)
        self.last_admitted[env_id] = token
        return True


class Curriculum:
    """Admission-count schedule: early small; middle 25/75; late 15/25/60.

    Configurable initial stage boundaries are 20k and 60k valid transitions.
    Distribution changes only when an environment requests a new episode.
    """
    def __init__(self, config, rng, state=None, *, probe_extent=None, probe_budget=None):
        self.config, self.rng = config, rng
        import copy
        self.state = copy.deepcopy(state or {'episodes_issued': 0})
        self.probe_extent, self.probe_budget = probe_extent, probe_budget

    def next(self, env_id, transitions):
        if self.config.paired_scene_sequence:
            import numpy as np
            if not 0 <= env_id < self.config.environments:
                raise ValueError('paired curriculum requires a valid environment slot')
            counts = self.state.setdefault('slot_episodes', {})
            episode = counts.get(str(env_id), 0)
            # 24 bits per episode and 8 bits per slot keep training namespaces
            # disjoint from evaluation and independent of reset arrival order.
            if episode >= 2**24 or env_id >= 256:
                raise ValueError('paired scene identity exhausted')
            seed = ((int(self.config.seed) + 1) << 32) + (env_id << 24) + episode
            rng = np.random.default_rng(np.random.SeedSequence([self.config.seed, env_id, episode]))
            low, high = self.config.curriculum_extents_m[0]
            extent = self.probe_extent if self.probe_extent is not None else float(rng.uniform(low, high))
            counts[str(env_id)] = episode + 1
            self.state['episodes_issued'] += 1
            return dict(seed=seed, family='moon' if env_id % 2 == 0 else 'cave', extent=extent,
                episode_budget=self.probe_budget or self.config.curriculum_budgets[0])
        stage = sum(transitions >= boundary for boundary in self.config.curriculum_transition_boundaries)
        probabilities = self.config.curriculum_mixtures[stage]
        size = int(self.rng.choice(3, p=probabilities))
        low, high = self.config.curriculum_extents_m[size]
        extent = self.probe_extent if self.probe_extent is not None else float(self.rng.uniform(low, high))
        # Separate disjoint evaluation seed namespace; a monotonic issued counter
        # avoids accidental reuse on resume while RNG restores terrain selection.
        serial = self.state['episodes_issued']
        self.state['episodes_issued'] = serial + 1
        seed = ((int(self.config.seed) + 1) << 32) + serial
        return dict(seed=seed, family='moon' if env_id % 2 == 0 else 'cave', extent=extent,
            episode_budget=self.probe_budget or self.config.curriculum_budgets[size])


def initial_model_checksum(learner):
    """Compare identical Actor/twin-Q initial tensor values across paired arms."""
    import hashlib
    digest = hashlib.sha256()
    for group in ('actor', 'q1', 'q2'):
        for name, value in sorted(getattr(learner, group).state_dict().items()):
            value = value.detach().cpu().contiguous()
            digest.update(f'{group}.{name}:{value.dtype}:{tuple(value.shape)}:'.encode())
            digest.update(value.numpy().tobytes())
    return digest.hexdigest()


def state_checksum(record):
    """Content identity for nested training state, independent of storage IDs."""
    import hashlib
    import numpy as np
    import torch
    digest = hashlib.sha256()

    def token(value):
        value = value if isinstance(value, bytes) else str(value).encode('utf-8')
        digest.update(str(len(value)).encode('ascii') + b':' + value)

    def visit(value):
        token(type(value).__name__)
        if isinstance(value, dict):
            token(len(value))
            for key in sorted(value, key=lambda item: (type(item).__name__, repr(item))):
                visit(key)
                visit(value[key])
        elif isinstance(value, (list, tuple)):
            token(len(value))
            for item in value:
                visit(item)
        elif isinstance(value, torch.Tensor):
            tensor = value.detach().cpu().contiguous()
            token(tensor.dtype)
            token(tuple(tensor.shape))
            token(tensor.numpy().tobytes())
        elif isinstance(value, np.ndarray):
            token(value.dtype.str)
            token(value.shape)
            token(value.tobytes())
        elif isinstance(value, bytes):
            token(value)
        elif value is None or isinstance(value, (str, int, float, bool, np.generic)):
            token(repr(value))
        else:
            raise TypeError(f'unsupported training identity value: {type(value).__name__}')

    visit(record)
    return digest.hexdigest()


def initial_training_identity(learner, *, replay, schedule, replay_rng, curriculum_rng,
                              curriculum_state, collector_state, counters, source_replay_sha256,
                              excluded_learning_fields=('polyak',)):
    """Read-only identity except the explicitly tested learning variable."""
    import random
    import numpy as np
    import torch
    state = learner.state_dict()
    state['learning_config'] = dict(state['learning_config'])
    if tuple(excluded_learning_fields) not in ((), ('polyak',), ('return_steps',), ('entropy_mode',)):
        raise ValueError('paired identity may exclude only its one tested learning variable')
    if tuple(excluded_learning_fields) == ('entropy_mode',):
        if state['learning_config']['entropy_mode'] == 'position_heading':
            for duplicate, source in (('log_alpha_heading', 'log_alpha'),
                    ('heading_alpha_optimizer', 'alpha_optimizer')):
                if state_checksum(state[duplicate]) != state_checksum(state[source]):
                    raise ValueError('paired initial entropy temperature histories must be identical')
                del state[duplicate]
    for field in excluded_learning_fields:
        del state['learning_config'][field]
    rng = dict(python=random.getstate(), numpy=np.random.get_state(),
               torch_cpu=torch.get_rng_state(),
               torch_cuda=torch.cuda.get_rng_state_all() if learner.device.type == 'cuda' else [],
               replay=replay_rng.bit_generator.state, curriculum=curriculum_rng.bit_generator.state)
    return dict(schema='paired_initial_state_v1', learner_sha256=state_checksum(state),
        rng_sha256=state_checksum(rng), collector_sha256=state_checksum(collector_state),
        curriculum_sha256=state_checksum(curriculum_state),
        counters_sha256=state_checksum(counters),
        source_replay_sha256=source_replay_sha256, replay_count=len(replay),
        schedule=schedule.state_dict(), excluded_learning_fields=list(excluded_learning_fields))


def run_training(config, *, resume=False, fork_from=None, fork_experiment=None, device='cuda', max_transitions=None,
                 domain_base=210, probe_extent=None, probe_budget=None,
                 env_factory='lunar_drl_exploration.ros_env:RosExplorationEnv'):
    """Run finite or continuous training; max_transitions counts this-run admissions.

    The only replay/optimizer owner is this process. Periodic saves ACK-drain at
    a completed update boundary; unfinished goals are retained live until resume
    of dispatch. A restored run repays saved integral credit before dispatching
    new work; resume with a zero budget only drains that credit. Fractional
    credit is retained. Finite runs confirm the final Actor before stopping.
    """
    import multiprocessing as mp
    import os
    import pickle
    from pathlib import Path
    import random
    import signal
    import subprocess
    import sys
    import time
    from .worker import numerical_threads
    numerical_threads(config.learner_threads)
    import numpy as np
    import torch
    from .checkpoint import CheckpointManager, TrainingState, UpdateBoundary
    from .collector import collector_main
    from .ipc import Duplex
    from .config import config_record
    from .metrics import MetricsWriter, ResourceMonitor, ResourceLimitError
    from .replay import ReplayBuffer, loads_transport
    from .sac import SACLearner
    from .schedule import UpdateSchedule
    if resume and fork_from is not None:
        raise ValueError('resume and fork_from are mutually exclusive')
    if fork_experiment is not None and (fork_from is None or resume):
        raise ValueError('fork experiment requires an explicit new fork')
    if config.seed < 0: raise ValueError('training seed must be nonnegative')
    if max_transitions is not None and (max_transitions < 0 or (max_transitions == 0 and not resume)):
        raise ValueError('max_transitions must be positive, or zero with resume for credit drain')
    if not 0 <= domain_base <= 233 - config.environments:
        raise ValueError('isolated worker DDS domains must be within 0..232')
    if config.model.width != 128 or config.model.heads != 8 or config.model.layers != 6:
        raise ValueError('training requires full 128/8/6 graph model')
    # Validate the source and empty destination before creating any run artifacts.
    fork_state = (TrainingState.fork_from(fork_from, config, experiment=fork_experiment)
                  if fork_from is not None else None)
    torch.set_num_threads(config.learner_threads)
    random.seed(config.seed); np.random.seed(config.seed % 2**32); torch.manual_seed(config.seed)
    manager = CheckpointManager(config.output_dir, config.save_interval_s,
        total_max_bytes=config.total_output_max_bytes, snapshot_max_bytes=config.snapshot_max_bytes,
        artifact_root=config.artifact_root)
    if not resume and (Path(config.output_dir) / 'resume.pt').exists():
        raise ValueError('resume.pt already exists; use --resume or a new output directory')
    learner = SACLearner(config.model, config.learning, device=device)
    # The legacy multi-stage runner compares this random-initialization lineage.
    initial_checksum = initial_model_checksum(learner) if config.paired_scene_sequence else None
    replay = ReplayBuffer(config.replay_max_bytes)
    schedule = UpdateSchedule(config.warmup, config.update_ratio, config.max_update_credit)
    replay_rng, curriculum_rng = np.random.default_rng(config.seed), np.random.default_rng(config.seed + 1)
    curriculum_state = None
    collector_state = dict(actor_record=learner.actor_state(), actor_version=0,
        policy_rng=torch.Generator().manual_seed(config.seed + 2).get_state(), sequences={})
    counters = dict(transitions=0, aborted=0, recoveries=0)
    fork_provenance = None
    source_replay_sha256 = None
    if resume or fork_state is not None:
        state = manager.load(config) if resume else fork_state
        fork_provenance = state.record.get('fork')
        if config.paired_scene_sequence:
            import hashlib
            source_replay_sha256 = hashlib.sha256(state.record['replay']).hexdigest()
        restored = state.restore(learner, config)
        replay, schedule = restored.replay, restored.schedule
        replay_rng, curriculum_rng = restored.replay_rng, restored.curriculum_rng
        curriculum_state, collector_state, counters = restored.curriculum, restored.collector_state, restored.counters
        # Release the loaded replay snapshot before the new capture; learned
        # tensors/replay are now owned by the learner and restored buffer.
        del state, restored
    fork_state = None
    restored_checksum = initial_model_checksum(learner) if config.paired_scene_sequence else None
    initial_identity = (initial_training_identity(learner, replay=replay, schedule=schedule,
        replay_rng=replay_rng, curriculum_rng=curriculum_rng, curriculum_state=curriculum_state,
        collector_state=collector_state, counters=counters, source_replay_sha256=source_replay_sha256,
        # Paired normalization applies only to a fresh fork. After learning,
        # independent temperatures legitimately diverge and must both be hashed.
        excluded_learning_fields=() if resume else {'paired-multistep': ('return_steps',),
            'paired-entropy': ('entropy_mode',)}.get(
                (fork_provenance or {}).get('experiment'), ('polyak',)))
        if config.paired_scene_sequence else None)
    trace_keys = ('trace_samples', 'trace_length_sum', 'trace_multi_count',
                  'trace_legacy_count', 'trace_coefficient_mass_sum')
    trace_totals = dict(counters.get('trace_totals', {key: 0 for key in trace_keys}))
    if collector_state['actor_record']['version'] != collector_state['actor_version']:
        raise ValueError('saved collector weights/publication identity disagree')
    curriculum = Curriculum(config, curriculum_rng, curriculum_state,
        probe_extent=probe_extent, probe_budget=probe_budget)
    ledger = AdmissionLedger(replay, schedule)
    ledger.last_admitted = dict(counters.get('last_admitted', {}))
    diagnostics = None
    if config.reward_diagnostics:
        from .reward_diagnostics import RewardDiagnostics
        diagnostics = RewardDiagnostics(config.environments, completion_bonus=config.completion_bonus,
            success_coverage=config.success_coverage, state=counters.get('reward_diagnostics'))
    boundary = UpdateBoundary()
    metrics = MetricsWriter(Path(config.output_dir) / 'metrics.jsonl', config.metrics_max_bytes,
        total_max_bytes=config.total_output_max_bytes)
    context = mp.get_context('spawn')
    connection, child = context.Pipe()
    process = context.Process(target=collector_main,
        args=(child, config_record(config), pickle.dumps(collector_state, protocol=5)),
        kwargs=dict(domain_base=domain_base, env_factory=env_factory), name='drl-cpu-collector')
    previous_signals = {sig: signal.signal(sig, manager.request_sigint) for sig in (signal.SIGINT, signal.SIGTERM)}
    baseline = schedule.transitions
    started = time.monotonic()
    pending_requests = {}
    reservations = {}
    env_status = {}
    env_owners = {}
    observations = dict(learner=dict(pid=os.getpid(), device=str(learner.device),
        numeric_threads=torch.get_num_threads(), package_path=__file__,
        torch_path=torch.__file__, cuda_initialized=torch.cuda.is_initialized()), workers={},
        probe=dict(extent_m=probe_extent, budget=probe_budget, max_new_transitions=max_transitions),
        resume=resume, initial_model_checksum=initial_checksum,
        restored_model_checksum=restored_checksum, initial_state_identity=initial_identity)
    if fork_provenance is not None:
        observations['fork'] = fork_provenance
    # Bounded recent episode identities suffice for run auditing; counters remain cumulative.
    episode_ids = []
    barrier_pending = False
    stopping = False
    normal_draining = max_transitions == 0
    repay_before_dispatch = schedule.can_update
    final_phase = None
    final_version = None
    final_deadline = None
    terminal_saved = False
    stopped = False
    exit_drain_deadline = None
    stop_reason = None
    last_metrics = started
    last_summary = started
    last_checkpoint_transitions = schedule.transitions if resume else None
    last_update = {}
    observed_publication = collector_state['actor_version']
    try:
        code_revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True,
            stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError): code_revision = 'unavailable'

    def capture():
        counters['last_admitted'] = dict(ledger.last_admitted)
        counters['trace_totals'] = dict(trace_totals)
        if diagnostics is not None:
            counters['reward_diagnostics'] = diagnostics.state_dict()
        state = TrainingState.capture(learner=learner, config=config, replay=replay,
            schedule=schedule, boundary=boundary, replay_rng=replay_rng, curriculum_rng=curriculum_rng,
            curriculum=curriculum.state, collector_state=collector_state, counters=counters)
        if fork_provenance is not None:
            state.record['fork'] = fork_provenance
        return state

    def persist_diagnostics():
        nonlocal last_summary
        if diagnostics is None:
            return
        from .artifacts import atomic_json
        boundary_record = dict(transitions=schedule.transitions, updates=schedule.updates,
            last_checkpoint_transitions=last_checkpoint_transitions)
        atomic_json(Path(config.output_dir)/'reward-summary.json',
            dict(diagnostics.summary(), training_boundary=boundary_record), max_bytes=1024**2)
        atomic_json(Path(config.output_dir)/'reward-cases.json',
            dict(diagnostics.cases(), training_boundary=boundary_record), max_bytes=16*1024**2)
        last_summary = time.monotonic()

    def save():
        nonlocal last_checkpoint_transitions
        path = manager.save(capture())
        last_checkpoint_transitions = schedule.transitions
        persist_diagnostics()
        metrics.append(dict(event='checkpoint', transitions=schedule.transitions,
            updates=schedule.updates, actor_version=collector_state['actor_version'],
            unfinished_reservations=schedule.inflight, path=str(path)))

    def final_command(kind):
        nonlocal final_phase, final_deadline, final_version
        final_phase = {'PUBLISH': 'PUBLISHED', 'BARRIER': 'BARRIER', 'STOP': 'STOPPED'}[kind]
        final_deadline = time.monotonic() + FINAL_CONFIRM_TIMEOUT_S
        message = dict(kind=kind)
        if kind == 'PUBLISH':
            final_version = learner.updates
            message['payload'] = pickle.dumps(learner.actor_state(), protocol=5)
        connection.send(message)

    def stop(reason):
        nonlocal stopping, stop_reason
        if stop_reason is None or normal_draining: stop_reason = reason
        manager.request_save(reason, stop=True)
        stopping = True
        pending_requests.clear()
        # Complete updates are atomic; preserve all remaining earned credit.
        # Keep the collector alive until it confirms this final Actor.
        if final_phase is None:
            final_command('PUBLISH')

    def record_run():
        owned = [os.getpid(), process.pid] + [pid for pids in env_owners.values() for pid in pids]
        manager.write_run(config, owned_pids=[pid for pid in owned if pid is not None],
            code_revision=code_revision, extra=observations)

    def status_record():
        elapsed = max(time.monotonic() - started, 1e-9)
        return dict(event='progress', wall_s=elapsed, transitions=schedule.transitions,
            new_transitions=schedule.transitions-baseline, updates=schedule.updates,
            transitions_per_s=(schedule.transitions-baseline)/elapsed,
            updates_per_s=(schedule.updates-start_updates)/elapsed, credit=float(schedule.credit),
            inflight=schedule.inflight, actor_version=observed_publication,
            replay_count=len(replay), replay_bytes=replay.bytes_used,
            save_in_s=max(0., config.save_interval_s-(time.monotonic()-manager.last_save)),
            environments=env_status, learning=last_update, trace_totals=dict(trace_totals),
            gpu_memory=(dict(allocated_bytes=torch.cuda.memory_allocated(learner.device),
                reserved_bytes=torch.cuda.memory_reserved(learner.device),
                peak_allocated_bytes=torch.cuda.max_memory_allocated(learner.device))
                if learner.device.type == 'cuda' else None))

    from .terminal import TrainingTerminal
    terminal = TrainingTerminal(environments=config.environments, warmup=config.warmup, target=max_transitions)
    start_updates = schedule.updates
    try:
        # An initial good checkpoint also protects a first-update infrastructure failure.
        if not resume: save()
        if fork_from is not None: record_run()
        process.start(); child.close()
        connection = Duplex(connection, config.environments,
            publication_window=(config.max_update_credit + config.actor_publish_updates - 1) //
                config.actor_publish_updates)
        record_run()
        while not stopped:
            if manager.stop_requested and not stopping:
                stop(manager.requested_reason or 'stop requested')
            if final_deadline is not None and time.monotonic() >= final_deadline:
                raise RuntimeError(f'collector final {final_phase} confirmation timed out')
            if connection.poll(.01):
                try: message = connection.recv()
                except EOFError as exc:
                    raise RuntimeError('collector transport ended before acknowledged stop') from exc
                kind = message['kind']; env_id = message.get('env')
                if kind == 'NEED_RESET':
                    if not stopping and not normal_draining:
                        spec = curriculum.next(env_id, schedule.transitions)
                        env_status[env_id] = dict(state='RESETTING', seed=spec['seed'],
                            family=spec['family'], extent_m=spec['extent'], budget=spec['episode_budget'])
                        connection.send(dict(kind='RESET', env=env_id, spec=spec))
                elif kind == 'SCENE':
                    scenes = loads_transport(message['payload'])
                    ledger.scene(env_id, scenes)
                    metadata = message['metadata']
                    if diagnostics is not None:
                        diagnostics.reset(env_id, 'RESET')
                    env_status[env_id] = dict(metadata, state='READY')
                    episode_ids.append(metadata['episode_id'])
                    del episode_ids[:-config.environments * 2]
                    observations['workers'].setdefault(str(env_id), {})['scene'] = dict(metadata,
                        truth_nodes=sum(len(scene.positions) for scene in scenes.values()),
                        truth_edges=sum(len(scene.edges) for scene in scenes.values()),
                        reference_cells=sum(len(scene.reference_indices) for scene in scenes.values()))
                elif kind == 'RESERVE':
                    if not stopping and not normal_draining: pending_requests[env_id] = message['token']
                elif kind == 'DONE':
                    transition = loads_transport(message['payload'])
                    is_new = message['token'] > ledger.last_admitted.get(env_id, -1)
                    if is_new and reservations.get(env_id) != message['token']:
                        raise RuntimeError('completion lacks its learner-owned dispatch reservation')
                    admitted = ledger.admit(env_id, message['token'], transition)
                    if admitted:
                        reservations.pop(env_id)
                        counters['transitions'] = schedule.transitions
                    # Order is transactional replay admission -> collected -> ACK.
                    connection.send(dict(kind='ACK', env=env_id, token=message['token']))
                    status = env_status.setdefault(env_id, {})
                    status.update(message.get('progress', {}))
                    execution = message.get('execution', {})
                    status.update({key: value for key, value in execution.items()
                        if key not in ('distance_m', 'turn_rad', 'simulation_s', 'wall_s')})
                    status['last_execution'] = execution
                    scene = ledger.active_scenes[env_id][transition.next_privileged.scene_id]
                    visible_count = int(np.unpackbits(transition.next_privileged.observed & scene.packed_reference).sum())
                    status['reference_coverage'] = visible_count / len(scene.reference_indices) if len(scene.reference_indices) else 0.
                    status.update(new_area_m2=transition.parts.new_area_m2,
                        reward=transition.reward, terminated=transition.terminated, truncated=transition.truncated,
                        actor_version=transition.actor_version, observed_nodes=len(transition.observation.node_ids),
                        observed_edges=len(transition.observation.edges))
                    wall = message.get('execution', {}).get('wall_s', 0)
                    status['actual_rtf'] = message.get('execution', {}).get('simulation_s', 0) / wall if wall else None
                    if admitted:
                        if diagnostics is not None:
                            before_count = int(np.unpackbits(
                                transition.privileged.observed & scene.packed_reference).sum())
                            diagnostics.observe(env_id, transition,
                                before_coverage=before_count/len(scene.reference_indices) if len(scene.reference_indices) else 0.,
                                after_coverage=status['reference_coverage'],
                                family=status.get('family', 'other'), extent_m=status.get('extent_m', 0.),
                                execution=execution, truncation_reason=status.get('truncation_reason'),
                                seed=status.get('seed'))
                        metrics.append(dict(status, event='transition', env=env_id, token=message['token'],
                            transitions=schedule.transitions, episode_id=transition.episode_id))
                    if max_transitions is not None and schedule.transitions-baseline >= max_transitions:
                        normal_draining = True
                        pending_requests.clear()
                        if stop_reason is None: stop_reason = 'max new transitions reached; credit drained'
                elif kind == 'ABORTED':
                    if reservations.get(env_id) == message['token']:
                        schedule.cancel_dispatch(); reservations.pop(env_id)
                        counters['aborted'] = counters.get('aborted', 0) + 1
                    metrics.append(dict(message, event='unfinished_excluded'))
                elif kind in ('BARRIER', 'STOPPED'):
                    collector_state = pickle.loads(message['payload'])
                    if counters['transitions'] != schedule.transitions:
                        raise RuntimeError('admission counter differs at barrier')
                    if kind == 'STOPPED':
                        if final_phase != 'STOPPED':
                            raise RuntimeError('collector stopped before final Actor confirmation')
                        if schedule.inflight or reservations: raise RuntimeError('stop left unaccounted reservations')
                        if collector_state['actor_version'] != final_version:
                            raise RuntimeError('stopped collector Actor differs from final learner')
                        if not terminal_saved: save()
                        stopped = True
                        final_deadline = None
                    elif final_phase == 'BARRIER':
                        if schedule.inflight or reservations:
                            raise RuntimeError('terminal barrier left unaccounted reservations')
                        if (collector_state['actor_version'] != final_version or
                                collector_state['actor_record']['version'] != final_version):
                            raise RuntimeError('terminal collector Actor differs from final learner')
                        save()
                        terminal_saved = True
                        stopping = True
                        final_command('STOP')
                    else:
                        # A periodic barrier may already be queued when stop is
                        # requested; it cannot stand in for the final snapshot.
                        if not stopping: save()
                        if not stopping: connection.send(dict(kind='RESUME'))
                    barrier_pending = False
                elif kind == 'PUBLISHED':
                    observed_publication = message['version']
                    if final_phase == 'PUBLISHED' and observed_publication == final_version:
                        final_command('STOP' if stopping else 'BARRIER')
                elif kind == 'FATAL':
                    raise RuntimeError('Collector fatal: ' + message['reason'])
                elif kind == 'RECOVERY':
                    if diagnostics is not None:
                        diagnostics.reset(env_id, 'RECOVERY')
                    counters['recoveries'] = counters.get('recoveries', 0) + 1
                    terminal.event(f"E{env_id} 基础设施恢复，第 {message['attempt']} 次：{message['reason']}")
                    metrics.append(dict(message, event='recovery'))
                elif kind == 'OWNERSHIP':
                    env_owners[env_id] = ([message['worker_pid']] if message['worker_pid'] else []) + message['child_pids']
                elif kind == 'WORKER_HELLO':
                    observations['workers'].setdefault(str(env_id), {}).update(message)
                    if message['torch_loaded']: raise RuntimeError('Torch imported in ROS worker')
                elif kind == 'COLLECTOR_HELLO': observations['collector'] = message
                elif kind == 'STATUS':
                    progress = message['progress']
                    env_status[env_id] = _merge_progress(env_status.get(env_id, {}), progress)
                    worker_pid = observations['workers'].get(str(env_id), {}).get('pid')
                    env_owners[env_id] = ([worker_pid] if worker_pid else []) + list(progress.get('owned_pids', []))
                elif kind == 'EPISODE_END': metrics.append(dict(message, event='episode_end'))
                else: raise RuntimeError('unexpected learner message ' + kind)
            if not process.is_alive() and not stopped:
                # Process exit can precede the reader's final large-frame
                # handoff. Keep consuming ordered inbox/EOF, without dispatch
                # or updates. The deadline bounds broken transport, not normal
                # delivery latency; there is no sleep-based completion guess.
                if exit_drain_deadline is None: exit_drain_deadline = time.monotonic() + 12
                if time.monotonic() >= exit_drain_deadline:
                    raise RuntimeError('collector transport drain timed out before acknowledged stop')
                continue
            if manager.stop_requested and not stopping:
                stop(manager.requested_reason or 'stop requested')
            if repay_before_dispatch and not schedule.can_update:
                repay_before_dispatch = False
            if not stopping and not normal_draining and not barrier_pending and not repay_before_dispatch:
                for env_id, token in list(pending_requests.items()):
                    if manager.stop_requested:
                        stop(manager.requested_reason or 'stop requested')
                        break
                    if not within_admission_budget(schedule.transitions, schedule.inflight,
                            baseline=baseline, maximum=max_transitions):
                        break
                    if schedule.can_dispatch():
                        if env_id in reservations: raise RuntimeError('second reservation for environment')
                        schedule.reserve_dispatch(); reservations[env_id] = token
                        connection.send(dict(kind='GRANT', env=env_id, token=token))
                        del pending_requests[env_id]
            # Exactly one complete update; poll completions again before the next.
            if not stopping and not manager.stop_requested and not barrier_pending and final_phase is None and schedule.can_update:
                with boundary.update():
                    before_update = time.monotonic()
                    if config.learning.return_steps == 1:
                        batch = replay.sample(config.learning.batch_size, replay_rng)
                    else:
                        batch = replay.sample_sequences(config.learning.batch_size, replay_rng,
                            max_steps=config.learning.return_steps)
                    last_update = learner.update(batch, replay.scenes)
                    schedule.updated()
                    for key in trace_keys:
                        trace_totals[key] += last_update[key]
                    last_update['wall_s'] = time.monotonic() - before_update
                if schedule.updates % config.actor_publish_updates == 0:
                    connection.send(dict(kind='PUBLISH', payload=pickle.dumps(learner.actor_state(), protocol=5)))
            if manager.stop_requested and not stopping: stop(manager.requested_reason or 'stop requested')
            if (normal_draining and not stopping and final_phase is None and
                    not barrier_pending and not schedule.inflight and not schedule.can_update):
                if reservations: raise RuntimeError('drain left unmatched reservations')
                if stop_reason is None: stop_reason = 'zero-admission resume; credit drained'
                final_command('PUBLISH')
            if manager.save_requested and not stopping and not normal_draining and not barrier_pending and final_phase is None:
                connection.send(dict(kind='BARRIER')); barrier_pending = True
            if not stopped and time.monotonic() - last_metrics >= 2:
                record = status_record()
                owned = [os.getpid(), process.pid] + [pid for pids in env_owners.values() for pid in pids]
                monitor = ResourceMonitor(owned, system_reserve_bytes=config.system_reserve_bytes,
                    owned_pss_limit_bytes=config.owned_pss_limit_bytes)
                try:
                    resource = monitor.observe(config.output_dir); record['resources'] = resource
                    monitor.check(resource)
                except (FileNotFoundError, ProcessLookupError) as exc:
                    # Lifecycle messages may be queued behind a completed update.
                    # Report unknown PID data; never substitute zero PSS.
                    record['resource_unavailable'] = str(exc)
                except ResourceLimitError as exc: stop(str(exc))
                metrics.append(record)
                if time.monotonic() - last_summary >= 30:
                    persist_diagnostics()
                terminal.render(record, phase='保存并停止' if stopping else ('补完更新' if normal_draining else ('保存检查点' if barrier_pending else None)))
                last_metrics = time.monotonic()
        process.join(10)
        observations['final'] = status_record()
        observations['final']['overshoot'] = max(0, schedule.transitions-baseline-(max_transitions if max_transitions is not None else schedule.transitions-baseline))
        observations['final']['stop_reason'] = stop_reason or 'orderly stop'
        observations['final']['owned_children_closed'] = not process.is_alive()
        record_run()
        persist_diagnostics()
        terminal.render(status_record(), phase='已保存并停止', force=True)
        return dict(transitions=schedule.transitions, new_transitions=schedule.transitions-baseline,
            updates=schedule.updates, episode_ids=episode_ids, episodes_issued=curriculum.state['episodes_issued'],
            stop_reason=stop_reason)
    except BaseException as exc:
        # Never overwrite the last good checkpoint after a partly applied update
        # or an unadmitted resource-damaged completion. The reason is explicit.
        terminal.clear()
        print(f'DRL stopped with error; last completed checkpoint preserved: {type(exc).__name__}: {exc}', file=sys.stderr, flush=True)
        # A poisoned optimizer boundary forbids saving, but not receiving/ACKing
        # already finished valid work. Drain before joining the collector so it
        # can cancel active goals and close workers rather than wait on our ACK.
        if process.pid is not None:
            try:
                if process.is_alive(): connection.send(dict(kind='STOP'))
                deadline = time.monotonic() + 12
                while time.monotonic() < deadline:
                    if not connection.poll(.05):
                        continue
                    message = connection.recv(); kind = message['kind']; env_id = message.get('env')
                    if kind == 'SCENE': ledger.scene(env_id, loads_transport(message['payload']))
                    elif kind == 'DONE':
                        ledger.admit(env_id, message['token'], loads_transport(message['payload']))
                        reservations.pop(env_id, None)
                        connection.send(dict(kind='ACK', env=env_id, token=message['token']))
                    elif kind == 'ABORTED' and reservations.get(env_id) == message['token']:
                        schedule.cancel_dispatch(); reservations.pop(env_id)
                    elif kind == 'STOPPED': break
                process.join(2)
            except (EOFError, OSError, ResourceLimitError, ValueError) as drain_error:
                print(f'Fatal drain incomplete; unadmitted completion/owned reservations retained until shutdown: {drain_error}',
                    file=sys.stderr, flush=True)
        print(f'Fatal recovery uses last good resume; current admitted={schedule.transitions}, '
            f'updates={schedule.updates}, remaining reservations={schedule.inflight}', file=sys.stderr, flush=True)
        raise
    finally:
        for sig, handler in previous_signals.items(): signal.signal(sig, handler)
        if process.pid is not None:
            if process.is_alive():
                try: connection.send(dict(kind='STOP'))
                except (EOFError, BrokenPipeError): pass
                process.join(12)
            if process.is_alive(): process.terminate(); process.join(5)
            if process.is_alive(): process.kill(); process.join(5)
        child.close()
        if isinstance(connection, Duplex): connection.close(flush=False)
        else: connection.close()
