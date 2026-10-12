"""Frozen A0/A2 pressure matrix, durable dispatch and identity-checked resume.

Run with system Python; children use the original runtime's owned Jazzy wrapper.
No source, reward, policy, navigation or scene replacement occurs here.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
import traceback

REPO = Path(__file__).resolve().parents[3]
SOURCE = REPO.parent / 'drl-task-frontier'
WRAPPER = SOURCE / 'scripts/drl/task_frontier/run.sh'
CACHE = '/home/kai/.cache/lunar-drl-task-frontier/jazzy-task8-20261004'
PREVIOUS = Path('/home/kai/CodexDownloads/lunar_navigation/drl-region-contraction-20261006')
ACTOR_SHA = '425209da25ff20b5d1ac833ec74074d21b45279c56cc0052ae93abaa0272358a'
CORE_SHA = {'evaluate.py': '9272b0026e4d4d469804c8b5ddd46cb994b73f771e333402693ca92e9331aac6',
            'contraction.py': 'e463b1cf220dca8c87579560f725377d74a7c11ec2cdb2ec3242fc363824fe85'}
IDENTITY = ('actor_sha', 'config_sha', 'source_commit', 'experiment_commit',
            'wrapper_sha', 'experiment_sources', 'runtime')
DEATH_BOUND = '''import ctypes, os, signal, sys
parent = int(sys.argv[1])
libc = ctypes.CDLL(None, use_errno=True)
if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
    raise OSError(ctypes.get_errno(), "prctl")
if os.getppid() != parent:
    raise SystemExit("pressure parent exited before child binding")
os.execvp(sys.argv[2], sys.argv[2:])
'''


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + '\n', encoding='utf-8')
    os.replace(temporary, path)


def budget_for_extent(extent):
    if not math.isfinite(extent) or not 50 <= extent <= 300:
        raise ValueError('pressure extent must be finite and in 50–300m')
    return int(math.ceil((256 + (extent * extent - 2500) * 256 / 7500) / 32 - 1e-12) * 32)


def build_cases():
    cases = []
    for size_index, extent in enumerate((50, 100, 150, 200, 300)):
        for repeat in range(2):
            for family_index, family in enumerate(('moon', 'cave')):
                cases.append(dict(name=f'{family}{extent:03d}-r{repeat+1}', family=family, extent=extent,
                    seed=2026100600 + size_index * 10 + repeat * 2 + family_index,
                    budget=budget_for_extent(extent), repeat=repeat,
                    mode_order=['baseline', 'remote'] if repeat == 0 else ['remote', 'baseline']))
    return cases


def validate_terminal(output, case, mode, proof):
    """Errors are retained as errors; only complete verified terminals are reusable."""
    output = Path(output)
    path = output / 'summary.json'
    if not path.exists():
        return None
    dispatch = json.loads((output / 'dispatch.json').read_text()) if (output / 'dispatch.json').exists() else {}
    if dispatch.get('case') != case or dispatch.get('mode') != mode or dispatch.get('frozen') != proof:
        raise ValueError(f'{output}: dispatch identity missing or changed')
    row = json.loads(path.read_text())
    expected = dict(family=case['family'], extent_m=case['extent'], seed=case['seed'], budget=case['budget'], mode=mode)
    if any(row.get(key) != value for key, value in expected.items()):
        raise ValueError(f'{output}: terminal case identity changed')
    if row.get('status') == 'finished':
        if row.get('error') or row.get('ended') != 1:
            raise ValueError(f'{output}: incomplete finished record')
        for key in IDENTITY:
            if row.get('provenance', {}).get(key) != proof.get(key):
                raise ValueError(f'{output}: terminal identity changed: {key}')
    elif row.get('status') == 'error':
        if not row.get('error') or row.get('ended') != 0:
            raise ValueError(f'{output}: invalid error record')
    else:
        raise ValueError(f'{output}: not a terminal record')
    return row


def commit(repo):
    return subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()


def runtime_guard():
    old = json.loads((PREVIOUS / 'moon50/baseline/summary.json').read_text())['provenance']
    runtime, files = old['runtime'], {}
    for entry in (*runtime['modules'].values(), *runtime['navigation'].values()):
        path, expected = entry['path'], entry['sha']
        if sha(path) != expected:
            raise ValueError('previous native runtime file changed: ' + path)
        files[path] = expected
    package = Path(runtime['modules']['lunar_drl_exploration.ros_env']['path']).parent
    members = [(p.name, sha(p)) for p in sorted(package.glob('*.py'))]
    packed = json.dumps(members, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode()
    if hashlib.sha256(packed).hexdigest() != runtime['package_sha']:
        raise ValueError('previous native Python package changed')
    for p in package.glob('*.py'):
        files[str(p)] = sha(p)
    for p in (Path(CACHE) / '.drl-source.json', Path(old['effective_config']['platform']['capability_path'])):
        files[str(p)] = sha(p)
    return runtime, files


def frozen_proof(root):
    core = {name: sha(Path(__file__).with_name(name)) for name in CORE_SHA}
    if core != CORE_SHA or sha(root / 'actor.pt') != ACTOR_SHA:
        raise ValueError('frozen previous Actor or A0/A2 adapter changed')
    runtime, files = runtime_guard()
    return dict(actor_sha=sha(root / 'actor.pt'), config_sha=sha(root / 'config.json'),
        source_commit=commit(SOURCE), experiment_commit=commit(REPO), wrapper_sha=sha(WRAPPER),
        experiment_sources=core, runtime=runtime, runtime_files=files)


def initialize(root):
    root = Path(root).resolve()
    if root.is_relative_to(REPO) or root.is_relative_to(SOURCE):
        raise ValueError('pressure artifacts must be outside source worktrees')
    root.mkdir(parents=True, exist_ok=True)
    protocol_path = root / 'protocol.json'
    if not protocol_path.exists():
        if any(root.iterdir()):
            raise ValueError('new pressure root is not empty')
        for name in ('actor.pt', 'config.json'):
            shutil.copyfile(PREVIOUS / name, root / name)
        protocol = dict(schema='region_pressure_protocol_v1', cases=build_cases(), modes=['baseline', 'remote'],
            frozen=frozen_proof(root), helper_sources={name: sha(Path(__file__).with_name(name))
                for name in ('pressure.py', 'compare.py')},
            max_small_parallel=2, large_serial_from_m=150, start_mode='inside', max_remote_positions=32,
            minimum_available_memory_kb=3 * 1024 * 1024,
            budget_rule='ceil32(256+(extent_m**2-2500)*256/7500), pressure extrapolation',
            captures='unchanged: every decision frame; NPZ initial/final/threshold/every32',
            frozen_note='same terrain/reference/original spawn; native initial scan timing may differ',
            runtime_wrapper=str(WRAPPER), cache=CACHE, created_unix=time.time())
        write_json(protocol_path, protocol)
        write_json(root / 'provenance.json', protocol['frozen'])
    protocol = json.loads(protocol_path.read_text())
    if protocol.get('cases') != build_cases() or protocol.get('modes') != ['baseline', 'remote']:
        raise ValueError('pressure matrix changed; use a different artifact root')
    verify_frozen(root, protocol)
    return protocol


def verify_frozen(root, protocol):
    if frozen_proof(root) != protocol['frozen']:
        raise ValueError('pressure frozen identity changed; dispatch stopped')
    for name, expected in protocol['helper_sources'].items():
        if sha(Path(__file__).with_name(name)) != expected:
            raise ValueError('pressure helper source changed: ' + name)


def host_sample():
    memory = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        key, value = line.split(':', 1)
        if key in ('MemTotal', 'MemAvailable', 'SwapFree'):
            memory[key + '_kb'] = int(value.split()[0])
    return dict(time_unix=time.time(), loadavg=list(os.getloadavg()), **memory)


def close_owned(process):
    """Bounded cleanup of this exact subprocess; no unrelated process groups."""
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


class Dispatcher:
    def __init__(self, root, protocol):
        self.root, self.protocol = root, protocol
        self.lock, self.done, self.children = threading.Lock(), {}, {}
        self.phase = 'starting'
        self.last_sample = 0.
        self.stop = threading.Event()

    def state(self, **extra):
        with self.lock:
            host = host_sample()
            if time.monotonic() - self.last_sample >= 5:
                with (self.root / 'host.jsonl').open('a') as stream:
                    stream.write(json.dumps(host, allow_nan=False) + '\n')
                self.last_sample = time.monotonic()
            write_json(self.root / 'run-state.json', dict(status='running', pid=os.getpid(), phase=self.phase,
                requested_episodes=len(self.protocol['cases']) * 2, terminal_episodes=len(self.done),
                active=list(self.children.values()), terminal=list(self.done.values()),
                updated_unix=time.time(), host=host, **extra))

    def one_case(self, case):
        for mode in case['mode_order']:
            if self.stop.is_set():
                raise InterruptedError('pressure dispatch stopped')
            verify_frozen(self.root, self.protocol)
            output = self.root / case['name'] / mode
            terminal = validate_terminal(output, case, mode, self.protocol['frozen'])
            if terminal is None:
                while host_sample()['MemAvailable_kb'] < self.protocol['minimum_available_memory_kb']:
                    write_json(self.root / (case['name'] + '-resource-wait.json'),
                        dict(case=case['name'], mode=mode, reason='available memory below3GiB; waiting', **host_sample()))
                    self.state()
                    if self.stop.wait(5):
                        raise InterruptedError('pressure dispatch stopped during memory admission')
                if self.stop.is_set():
                    raise InterruptedError('pressure dispatch stopped before launch')
                verify_frozen(self.root, self.protocol)
                if output.exists():
                    raise ValueError(f'incomplete output must be inspected, not overwritten: {output}')
                output.mkdir(parents=True)
                write_json(output / 'dispatch.json', dict(case=case, mode=mode, frozen=self.protocol['frozen']))
                command = ['bash', str(WRAPPER), 'python', str(Path(__file__).with_name('evaluate.py')),
                    '--mode', mode, '--config', str(self.root / 'config.json'), '--actor', str(self.root / 'actor.pt'),
                    '--output', str(output), '--family', case['family'], '--extent', str(case['extent']),
                    '--seed', str(case['seed']), '--budget', str(case['budget'])]
                with (output.parent / (mode + '.log')).open('x') as log:
                    process = subprocess.Popen(['python3', '-c', DEATH_BOUND, str(os.getpid()), *command],
                        cwd=REPO, env=dict(os.environ, DRL_CACHE=CACHE),
                        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                    key = case['name'] + '/' + mode
                    with self.lock:
                        self.children[key] = dict(case=case['name'], mode=mode, pid=process.pid, command=command,
                                                  launched_unix=time.time())
                    try:
                        self.state()
                        terminal_since = None
                        while process.poll() is None:
                            if self.stop.is_set():
                                raise InterruptedError('pressure dispatch stopped')
                            self.state()
                            if (output / 'summary.json').exists():
                                terminal_since = terminal_since or time.monotonic()
                                if time.monotonic() - terminal_since > 15:
                                    # This record is written only after native close; retire exact owned parent.
                                    saved = json.loads((output / 'summary.json').read_text())
                                    if saved.get('pid') != process.pid:
                                        raise ValueError('post-terminal process identity mismatch')
                                    write_json(output / 'post-terminal.json', dict(pid=process.pid,
                                        reason='native close and terminal saved; parent alive over15s', action='SIGTERM'))
                                    process.terminate()
                                    break
                            time.sleep(3)
                        code = process.wait(timeout=20)
                    finally:
                        close_owned(process)
                        with self.lock:
                            self.children.pop(key, None)
                verify_frozen(self.root, self.protocol)
                terminal = validate_terminal(output, case, mode, self.protocol['frozen'])
                if terminal is None:
                    # External process failures are infrastructure records, never coverage=0 policy samples.
                    terminal = dict(status='error', mode=mode, family=case['family'], extent_m=case['extent'],
                        seed=case['seed'], budget=case['budget'], ended=0, successes=0, completed=False,
                        infrastructure_failures=1, error_category='infrastructure', exit_code=code,
                        error=f'owned native worker exited {code} without terminal artifact',
                        provenance=self.protocol['frozen'])
                    write_json(output / 'summary.json', terminal)
                    write_json(output / 'status.json', terminal)
                write_json(output / 'process.json', dict(exit_code=code, command=command, pid=process.pid))
            with self.lock:
                key = case['name'] + '/' + mode
                self.children.pop(key, None)
                self.done[key] = dict(case=case['name'], mode=mode, status=terminal['status'],
                    completed=terminal.get('completed', False), coverage=terminal.get('final_coverage'),
                    decisions=terminal.get('decisions'), error=terminal.get('error'))
            self.state()

    def run(self):
        self.state()
        small = [r for r in self.protocol['cases'] if r['extent'] < self.protocol['large_serial_from_m']]
        self.phase = 'small_parallel'
        with ThreadPoolExecutor(max_workers=2) as pool:
            futures = [pool.submit(self.one_case, case) for case in small]
            try:
                for future in as_completed(futures):
                    future.result()
            except BaseException:
                self.stop.set()
                for future in futures:
                    future.cancel()
                raise
        self.phase = 'large_serial'
        for case in self.protocol['cases']:
            if case['extent'] >= self.protocol['large_serial_from_m']:
                self.one_case(case)
        self.phase = 'aggregate'
        self.state()
        command = ['bash', str(WRAPPER), 'python', '-m', 'scripts.drl.region_contraction.compare',
                   '--root', str(self.root), '--modes', 'baseline', 'remote', '--cases']
        command.extend(r['name'] for r in self.protocol['cases'])
        with (self.root / 'aggregate.log').open('a') as log:
            subprocess.run(command, cwd=REPO, env=dict(os.environ, DRL_CACHE=CACHE), stdout=log,
                           stderr=subprocess.STDOUT, check=True)
        comparison = json.loads((self.root / 'comparison.json').read_text())
        self.state(finished_unix=time.time())
        state = json.loads((self.root / 'run-state.json').read_text())
        state.update(status='finished' if comparison['complete_pairing'] else 'finished_with_errors',
                     phase='finished', paired_cases=len(comparison['paired_cases']))
        write_json(self.root / 'run-state.json', state)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--initialize-only', action='store_true')
    args = parser.parse_args()
    root = args.root.resolve()
    protocol = initialize(root)
    if args.initialize_only:
        print(json.dumps(dict(cases=len(protocol['cases']), episodes=40, frozen=protocol['frozen'])))
        return
    with (root / '.dispatch.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        dispatcher = Dispatcher(root, protocol)
        def interrupted(signum, frame):
            dispatcher.stop.set()
            raise KeyboardInterrupt(f'pressure dispatcher signal {signum}')
        signal.signal(signal.SIGTERM, interrupted)
        signal.signal(signal.SIGINT, interrupted)
        try:
            dispatcher.run()
        except BaseException as exc:
            write_json(root / 'dispatcher-error.json', dict(error=repr(exc), traceback=traceback.format_exc()))
            state = json.loads((root / 'run-state.json').read_text()) if (root / 'run-state.json').exists() else {}
            state.update(status='error', error=repr(exc), updated_unix=time.time())
            write_json(root / 'run-state.json', state)
            raise


if __name__ == '__main__':
    main()
