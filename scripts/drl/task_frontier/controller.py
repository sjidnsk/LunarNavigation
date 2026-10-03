"""Owned, resumable stage controller and shared strict evaluation contracts.

The historical case JSON is an identity source only, never a reusable result.
All schedule counters are absolute counters of this new experiment.
"""
import argparse
from contextlib import contextmanager
from fractions import Fraction
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import select
import stat
import subprocess
import time
import uuid

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
DEFAULT_ROOT = Path('/home/kai/CodexDownloads/lunar_navigation/drl-task-frontier-20261003/managed')
LEGACY = Path('/home/kai/CodexDownloads/lunar_navigation/drl-26856-continuation-20261001')
OLD_WRAPPER = Path('/home/kai/CodexDownloads/lunar_navigation/drl-small-recovery-20260926/evaluation/phase12_python')
OLD_CHECKPOINT = Path('/home/kai/CodexDownloads/lunar_navigation/drl-small-recovery-20260926/entropy-extended-split/resume.pt')
OLD_ACTOR_SHA = 'ab2f144f1f201b2abe189e06bcca8ca3c644d936d1d22668c4b8623b2bfc2278'
STARTUP_TIMEOUT = 30.0
CASE_KEYS = ('case_index', 'family', 'seed', 'extent_m', 'budget')
PAIR_KEYS = ('terrain_id', 'reference_id', 'pairing_digest')


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                     ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024**2), b''): h.update(chunk)
    return h.hexdigest()


def write(path, value):
    from lunar_drl_exploration.artifacts import atomic_json
    atomic_json(path, value)


def stage_target(stage: int) -> tuple[int, int]:
    if type(stage) is not int or not 0 <= stage <= 10:
        raise ValueError('stage must be 0..10')
    return 1024 + 8192 * stage, 2048 * stage


def next_training(stage, schedule):
    target_t, target_u = stage_target(stage)
    t, u = schedule['transitions'], schedule['updates']
    if type(t) is not int or type(u) is not int or min(t, u) < 0 or t > target_t or u > target_u:
        raise ValueError('schedule exceeds this new stage; no legacy counter subtraction')
    credit = Fraction(str(schedule['credit']))
    if credit < 0: raise ValueError('negative credit')
    if t < target_t: return target_t - t
    if u < target_u:
        if credit < target_u-u: raise ValueError('missing earned stage credit')
        return 0
    if credit or schedule.get('unfinished_reservations', 0):
        raise ValueError('stage not quiescent')
    return None


def freeze_cases(source, output):
    rows = read(source)['cases']
    if len(rows) != 20: raise ValueError('historical identity requires twenty ordered cases')
    cases = []
    for ordinal, row in enumerate(rows):
        index = ordinal // 2
        if (row['family'], row['seed'], row['extent_m']) != (
                ('moon', 'cave')[ordinal % 2], 2026100100+index, 50.+index*50./9.):
            raise ValueError('historical case differs from original continue.py expression/order')
        cases.append(dict(case_index=ordinal, family=row['family'], seed=row['seed'],
                          extent_m=row['extent_m'], budget=512))
    record = dict(schema='paired_cases_v1', source_sha=sha(source), cases=cases, cases_digest=digest(cases))
    output = Path(output)
    if output.exists():
        if read(output) != record: raise ValueError('frozen cases changed')
    else: write(output, record)
    return cases


def validate_evaluation(record: dict, *, actor_sha: str, semantic_id: str, cases: list[dict]) -> None:
    """Require ordered complete terminal results and expected runtime identities."""
    def require(test, message):
        if not test: raise ValueError('invalid evaluation: ' + message)
    require(record.get('status') == 'finished' and record.get('exit_code') == 0, 'unfinished run')
    require(record.get('actor_sha') == actor_sha and record.get('semantic_id') == semantic_id, 'model identity')
    rows = record.get('cases', [])
    require(len(rows) == len(cases) and bool(cases), 'case count')
    for row, expected in zip(rows, cases):
        for key in (*CASE_KEYS, *PAIR_KEYS, 'config_digest'):
            require(key in expected and bool(expected[key]) if key in (*PAIR_KEYS, 'config_digest') else key in expected,
                    'expected ' + key)
            require(row.get(key) == expected[key], key)
        require(row.get('actor_sha') == actor_sha and row.get('semantic_id') == semantic_id, 'case model identity')
        require('error' in row and row['error'] is None and row.get('exit_code') == 0, 'case error/exit')
        require(bool(row.get('run_id')) and type(row.get('pid')) is int and row['pid'] > 0
                and bool(row.get('provenance_digest')), 'case provenance')
        require(type(row.get('terminated')) is bool and type(row.get('truncated')) is bool
                and type(row.get('completed')) is bool, 'terminal flags')
        require(type(row.get('collisions')) is int and row['collisions'] >= 0, 'collision count')
        require(row['terminated'] or row['truncated'] or row['collisions'] > 0, 'nonterminal')
        require(type(row.get('decisions')) is int and 0 <= row['decisions'] <= expected['budget'], 'budget')
        for key in ('final_coverage', 'covered_area_m2', 'coverable_area_m2'):
            require(type(row.get(key)) in (float, int) and math.isfinite(row[key]) and row[key] >= 0, key)
        require(row['final_coverage'] <= 1 and row['covered_area_m2'] <= row['coverable_area_m2']+1e-8, 'coverage')
        require(row['completed'] == bool(row['terminated'] and row['final_coverage'] >= .99 and not row['collisions']), 'success')


def identity(root):
    value = read(Path(root)/'identity.json')
    if value.get('schema') != 'task_frontier_run_v1' or value.get('root') != str(Path(root).resolve()):
        raise ValueError('unowned artifact directory')
    if digest(read(Path(root)/'config.json')) != value.get('config_digest'):
        raise ValueError('owned effective config changed')
    if value.get('runtime_cache') != runtime_cache():
        raise ValueError('owned runtime cache changed or missing')
    return value


def runtime_cache():
    return str(Path(os.environ.get('DRL_CACHE','/home/kai/.cache/lunar-drl-task-frontier/jazzy')).resolve())


@contextmanager
def root_lock(root, inherited_fd=None):
    """Close our descriptor only: an inherited child retains the same flock.

    LOCK_UN here would unlock the shared open-file-description in BOTH processes.
    """
    identity(root)
    path=Path(root)/'.controller.lock'
    descriptor=(os.open(path,os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW,0o600)
                if inherited_fd is None else inherited_fd)
    try:
        actual=os.fstat(descriptor);expected=path.stat(follow_symlinks=False)
        if not stat.S_ISREG(actual.st_mode) or (actual.st_dev,actual.st_ino)!=(expected.st_dev,expected.st_ino):
            raise RuntimeError('inherited controller lock identity mismatch')
        fcntl.flock(descriptor,fcntl.LOCK_EX|fcntl.LOCK_NB)
        yield descriptor
    finally:os.close(descriptor)


def process_matches(manifest):
    pid = manifest.get('pid')
    if type(pid) is not int or pid <= 0: return False
    try:
        args = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        started = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError): return False
    return (str(HERE/'controller.py').encode() in args and
            str(manifest.get('root', '')).encode() in args and
            str(manifest.get('run_id', '')).encode() in args and
            started == manifest.get('process_start'))


def startup_identity_matches(ready,process,root,owned,token):
    expected=dict(pid=process.pid,root=str(root),run_id=owned['run_id'],startup_token=token,
                  ready=True,lock_owned=True)
    if any(ready.get(key)!=value for key,value in expected.items()) or not process_matches(ready):return False
    try:
        lock=Path(root)/'.controller.lock'
        held=os.stat(f"/proc/{process.pid}/fd/{ready['lock_fd']}")
        target=lock.stat(follow_symlinks=False)
        manifest=read(Path(root)/'manifest.json')
    except (OSError,ValueError,KeyError):return False
    fields=(*expected,'process_start','lock_fd','lock_device','lock_inode')
    return (stat.S_ISREG(held.st_mode) and
            (held.st_dev,held.st_ino)==(target.st_dev,target.st_ino)==(ready.get('lock_device'),ready.get('lock_inode'))
            and all(manifest.get(key)==ready.get(key) for key in fields))


def wait_startup(process,reader,root,owned,token):
    deadline=time.monotonic()+STARTUP_TIMEOUT;payload=bytearray()
    while True:
        code=process.poll()
        if code is not None:raise RuntimeError(f'controller exited {code} before startup confirmation; inspect controller.log')
        remaining=deadline-time.monotonic()
        if remaining<=0:raise RuntimeError('controller startup timed out; inspect controller.log')
        if not select.select([reader],[],[],min(.1,remaining))[0]:continue
        chunk=os.read(reader,16384)
        if not chunk:
            try:code=process.wait(timeout=min(.1,max(.001,deadline-time.monotonic())))
            except subprocess.TimeoutExpired:code=process.poll()
            raise RuntimeError(f'controller exited {code} / closed readiness pipe before startup confirmation')
        payload.extend(chunk)
        if len(payload)>16384:raise RuntimeError('controller startup identity message exceeds bound')
        if b'\n' not in payload:continue
        try:ready=json.loads(payload)
        except ValueError as error:raise RuntimeError('controller startup identity is invalid JSON') from error
        if process.poll() is not None or not startup_identity_matches(ready,process,root,owned,token):
            raise RuntimeError('controller startup identity/lock confirmation mismatch')
        return ready


def start_background(root):
    owned=identity(root);verify_gate(root)
    token=uuid.uuid4().hex;reader,writer=os.pipe();process=None
    try:
        with root_lock(root) as lock_fd:
            with (Path(root)/'controller.log').open('ab') as log:
                process=subprocess.Popen(['bash',str(HERE/'run.sh'),'run','--root',str(root),
                    '--run-id',owned['run_id'],'--lock-fd',str(lock_fd),'--ready-fd',str(writer),
                    '--startup-token',token],pass_fds=(lock_fd,writer),
                    start_new_session=True,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT)
            os.close(writer);writer=None
            ready=wait_startup(process,reader,root,owned,token)
        # Parent closed its duplicate; confirm the matching live child retains the lock fd.
        if process.poll() is not None or not startup_identity_matches(ready,process,root,owned,token):
            raise RuntimeError('controller exited or lost startup identity during lock handoff')
        return dict(ready,monitor=f'bash {HERE}/run.sh monitor --root {root}')
    except BaseException:
        if process is not None and process.poll() is None:
            try:os.killpg(process.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:process.wait(timeout=30)
            except subprocess.TimeoutExpired as error:
                raise RuntimeError(f'failed startup; controller {process.pid} is still saving/stopping') from error
        raise
    finally:
        os.close(reader)
        if writer is not None:os.close(writer)


def request_stop(root):
    owned = identity(root)
    manifest = read(Path(root)/'manifest.json')
    if manifest.get('run_id') != owned['run_id'] or not process_matches(manifest):
        raise ValueError('no matching owned controller process')
    os.kill(manifest['pid'], signal.SIGINT)


def source_commit():
    return subprocess.check_output(['git', '-C', str(REPO), 'rev-parse', 'HEAD'], text=True).strip()


def verify_gate(root):
    gate = read(Path(root)/'review-gate.json')
    commit = source_commit()
    if (gate.get('source_commit') != commit or gate.get('config_sha') != sha(Path(root)/'config.json')
            or any(gate.get(key, {}).get('status') != 'passed' or
                   gate[key].get('source_commit') != commit for key in ('native_smoke', 'independent_audit'))):
        raise ValueError('native smoke and independent audit gate must pass for current source/config')
    if subprocess.check_output(['git', '-C', str(REPO), 'status', '--porcelain', '--untracked-files=no'], text=True):
        raise ValueError('review gate requires clean committed source')
    return gate


def prepare(root, config_path):
    from lunar_drl_exploration.artifacts import initialize_artifact_root
    from lunar_drl_exploration.cli import load_config, parser
    from lunar_drl_exploration.config import config_record, resume_semantics
    root = Path(root).resolve()
    if root.exists() and (root/'identity.json').exists():
        return identity(root)
    if root.exists() and any(root.iterdir()): raise ValueError('refusing to adopt nonempty directory')
    config = load_config(parser().parse_args(['train', '--config', str(config_path)]))
    initialize_artifact_root(root, config.total_output_max_bytes)
    record = config_record(config)
    record.update(output_dir=str(root/'run'), artifact_root=str(root))
    from lunar_drl_exploration.config import training_config_from_record
    effective = training_config_from_record(record)
    value = dict(schema='task_frontier_run_v1', root=str(root), run_id=uuid.uuid4().hex,
                 semantic_id=digest(resume_semantics(effective)), config_digest=digest(record),runtime_cache=runtime_cache())
    write(root/'identity.json', value)
    write(root/'config.json', record)
    freeze_cases(LEGACY/'artifacts/baseline-summary.json', root/'cases.json')
    return value


class Controller:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.identity = identity(self.root)
        self.child = None
        self.stop = False
        self.manifest = dict(self.identity, pid=os.getpid(), process_start=Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()[19],
            stages=10, stage=0, phase='initializing', status='running', targets=dict(transitions=1024, updates=0),
            actuals=dict(transitions=0, updates=0), credit='0', checkpoint=str(self.root/'run/resume.pt'),
            actor_sha=None, eval_completed=0, eval_groups=0, exit_code=None,
            curves={name:str(self.root/'curves'/f'{name}.csv') for name in ('learning','fixed','evaluation','reward')})

    def record(self, **values):
        self.manifest.update(values, updated_unix=time.time())
        write(self.root/'manifest.json', self.manifest)

    def announce_ready(self,ready_fd,startup_token,lock_fd):
        if ready_fd is None:return
        held=os.fstat(lock_fd)
        self.record(ready=True,startup_token=startup_token,lock_owned=True,lock_fd=lock_fd,
                    lock_device=held.st_dev,lock_inode=held.st_ino)
        payload=(json.dumps(self.manifest,ensure_ascii=False,allow_nan=False)+'\n').encode('utf-8')
        if len(payload)>16384:raise RuntimeError('controller startup identity message exceeds bound')
        offset=0
        while offset<len(payload):offset+=os.write(ready_fd,payload[offset:])

    def interrupt(self, *_):
        self.stop = True
        # Signal handlers must not re-enter artifact locks held by this thread.
        if self.child is not None and self.child.poll() is None:
            try:os.killpg(self.child.pid, signal.SIGINT)
            except ProcessLookupError:pass

    def spawn(self, command, label, **environment):
        from lunar_drl_exploration.artifacts import append_bounded
        if self.stop: return False
        self.child = subprocess.Popen(command, start_new_session=True, stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=dict(os.environ, PYTHONUNBUFFERED='1', DRL_RUN_ID=self.identity['run_id'],
                     DRL_ROOT=str(self.root), ROS_LOG_DIR=str(self.root/'ros'), **environment))
        self.record(child_pid=self.child.pid, eval_pid=self.child.pid if self.manifest['phase']=='evaluation' else None,
                    detail_log=str(self.root/f'{label}.log'))
        if self.stop:self.interrupt()
        try:
            for line in self.child.stdout:
                append_bounded(self.root/f'{label}.log', line, 1024**2, whole_lines=True)
                if line.startswith(b'TASK_FRONTIER_PROGRESS '):
                    progress=json.loads(line.partition(b' ')[2])
                    self.record(actuals={key:progress[key] for key in ('transitions','updates')},credit=progress['credit'])
            code = self.child.wait()
        except BaseException:
            if self.child.poll() is None:
                try:os.killpg(self.child.pid,signal.SIGINT)
                except ProcessLookupError:pass
                # Drain its pipe while it saves; a full pipe must not block shutdown.
                self.child.communicate(timeout=120)
            raise
        self.child = None
        self.record(child_pid=None, child_exit_code=code)
        if self.stop: return False
        if code: raise RuntimeError(f'{label} exited {code}; inspect {label}.log')
        return True

    def schedule(self):
        checkpoint = self.root/'run/resume.pt'
        if not checkpoint.exists(): return dict(transitions=0, updates=0, credit='0')
        import torch
        from lunar_drl_exploration.checkpoint import TrainingState
        from lunar_drl_exploration.config import training_config_from_record
        state = TrainingState(torch.load(checkpoint, map_location='cpu', weights_only=False))
        state.validate(training_config_from_record(read(self.root/'config.json')))
        schedule = state.record['schedule']
        self.record(actuals={key:schedule[key] for key in ('transitions', 'updates')}, credit=schedule['credit'], checkpoint_sha=sha(checkpoint))
        return schedule

    def screen(self, label, actor, baseline=None):
        legacy = label == 'baseline'
        config = LEGACY/'artifacts/config.json' if legacy else self.root/'config.json'
        wrapper = ['bash', str(OLD_WRAPPER)] if legacy else ['bash', str(HERE/'run.sh'), 'python']
        model_sha = sha(actor)
        if legacy and model_sha != OLD_ACTOR_SHA: raise ValueError('preserved baseline Actor hash changed')
        source_checkpoint=OLD_CHECKPOINT if legacy else self.root/'run/resume.pt'
        source_checkpoint_sha=sha(source_checkpoint) if legacy else read(actor.with_suffix('.json'))['checkpoint_sha']
        if not legacy and read(actor.with_suffix('.json'))['actor_sha'] != model_sha:
            raise ValueError('stage Actor export provenance changed')
        cases = read(self.root/'cases.json')['cases']
        rows = []
        semantic = None
        for case in cases:
            index = case['case_index']
            output = self.root/f'{label}-case-{index:02}.json'
            request = dict(case, actor=str(actor), actor_sha=model_sha, config=str(config), config_sha=sha(config),
                           run_id=self.identity['run_id'], label=label, legacy=legacy, wrapper=wrapper[1],
                           source_commit=source_commit(), root=str(self.root),source_checkpoint=str(source_checkpoint),source_checkpoint_sha=source_checkpoint_sha)
            if baseline: request['paired_with']={key:baseline['cases'][index][key] for key in PAIR_KEYS}
            request_path = self.root/'evaluation-request.json'
            self.record(phase='evaluation', label=label, actor_sha=model_sha, case_index=index,
                        **{key:case[key] for key in ('family','extent_m','seed')}, eval_completed=len(rows),
                        eval_groups=len(rows)//2, eval_pid=None)
            # Validate against current effective configs in the child even for a cache hit.
            write(request_path, request)
            if not self.spawn([*wrapper, str(HERE/'eval_case.py'), '--request', str(request_path),
                               '--output', str(output)], label+f'-{index:02}'):
                return None
            result = read(output)
            row = result['cases'][0]
            expected = dict(case, config_digest=row['config_digest'], **{key:row[key] for key in PAIR_KEYS})
            if baseline:
                expected.update({key:baseline['cases'][index][key] for key in PAIR_KEYS})
            validate_evaluation(result, actor_sha=model_sha, semantic_id=row['semantic_id'], cases=[expected])
            if semantic is not None and semantic != row['semantic_id']: raise ValueError('evaluation semantics changed between cases')
            semantic = row['semantic_id']
            rows.append(row)
        result = dict(status='finished', exit_code=0, label=label, stage=0 if legacy else self.manifest['stage'],
                      actor_sha=model_sha, semantic_id=semantic, cases=rows)
        expected = [dict(case, config_digest=row['config_digest'], **{key:row[key] for key in PAIR_KEYS}) for case,row in zip(cases, rows)]
        validate_evaluation(result, actor_sha=model_sha, semantic_id=semantic, cases=expected)
        write(self.root/f'{label}-summary.json', result)
        self.record(eval_completed=20, eval_groups=10)
        return result

    def run(self,*,ready_fd=None,startup_token=None,lock_fd=None):
        verify_gate(self.root)
        signal.signal(signal.SIGINT, self.interrupt)
        signal.signal(signal.SIGTERM, self.interrupt)
        from curves import CurveRecorder
        recorder = CurveRecorder(self.root)
        recorder.start()
        try:
            self.record()
            self.schedule()
            freeze_cases(LEGACY/'artifacts/baseline-summary.json',self.root/'cases.json')
            self.announce_ready(ready_fd,startup_token,lock_fd)
            baseline = self.screen('baseline', LEGACY/'artifacts/actor-26856.pt')
            if baseline is None: return
            for stage in range(11):
                target_t, target_u = stage_target(stage)
                self.record(stage=stage, phase='warmup' if stage==0 else 'training',
                            targets=dict(transitions=target_t, updates=target_u))
                # Stages already evaluated may be behind the current checkpoint on resume.
                schedule = self.schedule()
                if schedule['transitions'] > target_t:
                    if stage:
                        actor=self.root/f'stage{stage}.pt'
                        if not actor.exists():raise ValueError('advanced checkpoint without preserved stage Actor')
                        if self.screen(f'stage{stage}',actor,baseline) is None:return
                    continue
                remaining = next_training(stage, schedule)
                if remaining is not None:
                    args = ['bash', str(HERE/'run.sh'), 'python', str(HERE/'train_live.py'), 'train',
                            '--config', str(self.root/'config.json'), '--max-transitions', str(remaining), '--domain-base', '190']
                    if (self.root/'run/resume.pt').exists(): args.append('--resume')
                    if not self.spawn(args, f'train-stage{stage}', DRL_STAGE=str(stage)): return
                    if next_training(stage, self.schedule()) is not None: raise ValueError('trainer did not reach exact stage target')
                if stage == 0:
                    from fixed_probes import freeze_index
                    freeze_index(self.root/'fixed-probes')
                    continue
                from lunar_drl_exploration.evaluation import export_actor
                actor = self.root/f'stage{stage}.pt'
                if not actor.exists():
                    export_actor(self.root/'run/resume.pt', actor)
                    write(actor.with_suffix('.json'),dict(actor_sha=sha(actor),checkpoint_sha=sha(self.root/'run/resume.pt'),updates=target_u))
                else:
                    import torch
                    checkpoint_actor=torch.load(self.root/'run/resume.pt',map_location='cpu',weights_only=False)['learner']['actor']
                    exported=torch.load(actor,map_location='cpu',weights_only=True)
                    if exported['version']!=target_u or any(not torch.equal(v,exported['state_dict'][key]) for key,v in checkpoint_actor.items()):
                        raise ValueError('saved stage Actor differs from checkpoint')
                if self.screen(f'stage{stage}', actor, baseline) is None: return
                recorder.fixed_checkpoint(self.root/'run/resume.pt', actor)
            summaries=[read(self.root/f'{label}-summary.json') for label in ['baseline',*[f'stage{i}' for i in range(1,11)]]]
            def score(result):
                rows=result['cases']
                return (sum(row['completed'] for row in rows),
                        -sum(not row['completed'] and row['max_zero_gain_run']>=128 and row['zero_gain_two_point_loop_steps']>0 for row in rows),
                        sum(row['final_coverage'] for row in rows))
            best=max(summaries,key=score)
            write(self.root/'comparison.json',dict(status='finished',summaries=summaries,best_label=best['label'],
                tie_rule='retain earlier',scope='fixed development cases; no independent holdout or vehicle claim'))
            write(self.root/'selection.json',dict(best_label=best['label'],actor_sha=best['actor_sha'],score=score(best)))
            self.record(status='finished', phase='complete', exit_code=0)
        except Exception as exc:
            self.record(status='failed', phase='error', error=f'{type(exc).__name__}: {exc}', exit_code=1)
            raise
        finally:
            try:recorder.close()
            except Exception as exc:
                self.record(status='failed',phase='error',error=f'curves: {exc}',exit_code=1)
                raise
            finally:
                if self.stop:
                    self.schedule()
                    self.record(status='paused', phase='saved_exit', exit_code=130)
                write(self.root/'exit.json',dict(run_id=self.identity['run_id'],pid=os.getpid(),status=self.manifest['status'],
                    exit_code=self.manifest['exit_code'],actuals=self.manifest['actuals'],updated_unix=time.time()))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('prepare','start','run','resume','monitor','stop','status'))
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    parser.add_argument('--config', type=Path, default=REPO/'config/drl_task_frontier.yaml')
    parser.add_argument('--run-id')
    parser.add_argument('--lock-fd',type=int,help=argparse.SUPPRESS)
    parser.add_argument('--ready-fd',type=int,help=argparse.SUPPRESS)
    parser.add_argument('--startup-token',help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    root = args.root.resolve()
    if args.command == 'prepare': print(json.dumps(prepare(root,args.config))); return
    owned = identity(root)
    if args.command == 'monitor':
        import curses
        from dashboard import display
        return curses.wrapper(lambda screen: display(screen, root))
    if args.command == 'stop': return request_stop(root)
    if args.command == 'status': print(json.dumps(read(root/'manifest.json'),ensure_ascii=False,indent=2)); return
    if args.command in ('start','resume'):
        print(json.dumps(start_background(root),ensure_ascii=False))
        return
    if args.run_id != owned['run_id']: raise ValueError('controller run identity mismatch')
    inherited=(args.lock_fd,args.ready_fd,args.startup_token)
    if any(value is not None for value in inherited) and any(value is None for value in inherited):
        raise ValueError('complete inherited startup lock/readiness identity required')
    try:
        with root_lock(root,args.lock_fd) as lock_fd:
            Controller(root).run(ready_fd=args.ready_fd,startup_token=args.startup_token,lock_fd=lock_fd)
    finally:
        if args.ready_fd is not None:os.close(args.ready_fd)



if __name__ == '__main__': main()
