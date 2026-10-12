"""Scheduling-only parallel continuation of an immutable A0/A2 pressure matrix.

The old dispatcher is paused while its existing native arm finishes normally.
Only after a verified terminal and native cleanup is the old owner retired and
its dispatch lock transferred. A separate guardian resumes a paused old owner
if this helper dies. No incomplete evaluation is overwritten or retried.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import copy
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import traceback

REPO = Path(__file__).resolve().parents[3]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
from scripts.drl.region_contraction import pressure
from scripts.drl.region_contraction.pressure_resources import ProcReader, descendant, option, same_process


def identity(reader, pid):
    row = reader.stat(pid)
    return {key: row[key] for key in ('pid', 'ppid', 'start_ticks')}


def live_exact(reader, expected):
    try:
        return same_process(reader.stat(expected['pid']), expected)
    except (OSError, ValueError):
        return False


def original_process_alive(reader, expected):
    """A reparented native child is still the same live cleanup obligation."""
    try:
        row = reader.stat(expected['pid'])
        return row['start_ticks'] == expected['start_ticks']
    except (OSError, ValueError):
        return False


def signal_exact(reader, expected, signum):
    """pidfd protects the final signal from a PID reuse race."""
    if not live_exact(reader, expected):
        raise ValueError('process identity changed before signal')
    if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
        raise RuntimeError('safe handoff requires Linux pidfd signaling')
    fd = os.pidfd_open(expected['pid'])
    try:
        if not live_exact(reader, expected):
            raise ValueError('process identity changed while opening pidfd')
        signal.pidfd_send_signal(fd, signum)
    finally:
        os.close(fd)


def audit_outputs(root, protocol, active):
    allowed = {(row['case'], row['mode']) for row in active}
    terminals = []
    for case in protocol['cases']:
        for mode in case['mode_order']:
            output = root / case['name'] / mode
            terminal = pressure.validate_terminal(output, case, mode, protocol['frozen'])
            if terminal is not None:
                terminals.append(dict(case=case['name'], mode=mode, status=terminal['status']))
            elif output.exists() and (case['name'], mode) not in allowed:
                raise ValueError(f'unregistered partial output cannot be overwritten: {output}')
            elif not output.exists() and (output.parent / (mode + '.log')).exists():
                raise ValueError(f'orphan mode log cannot be overwritten: {output.parent / (mode + ".log")}')
    return terminals


def terminal_closed(root, protocol, active, worker_identity, reader, known_descendants=()):
    case = next((row for row in protocol['cases'] if row['name'] == active['case']), None)
    if case is None or active['mode'] not in case['mode_order']:
        raise ValueError('active case/mode is outside frozen matrix')
    terminal = pressure.validate_terminal(root / case['name'] / active['mode'], case,
                                          active['mode'], protocol['frozen'])
    if terminal is None:
        return False
    if terminal.get('pid') != active['pid']:
        raise ValueError('terminal PID does not match the active worker')
    table = reader.table()
    if active['pid'] in table and not same_process(table[active['pid']], worker_identity):
        raise ValueError('worker identity changed at terminal')
    if any(pid != active['pid'] and descendant(table, pid, active['pid']) for pid in table):
        return False
    if any(original_process_alive(reader, row) for row in known_descendants if row['pid'] != active['pid']):
        return False
    return True


def guardian(root, helper_owner, old_owner, *, interval=1., timeout=6*3600):
    """Independent process; never binds PDEATHSIG to the helper it protects."""
    reader = ProcReader()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            receipt = json.loads((root / 'handoff.json').read_text())
            if receipt.get('helper_owner') != helper_owner or receipt.get('old_owner') != old_owner:
                raise ValueError('guardian owner registration changed')
            if receipt.get('phase') in ('old_exited', 'dispatch_lock_acquired', 'parallel_running',
                                         'parallel_finished', 'restored_original'):
                return
            if not original_process_alive(reader, helper_owner):
                restored = False
                if live_exact(reader, old_owner) and reader.stat(old_owner['pid'])['state'] in ('T', 't'):
                    signal_exact(reader, old_owner, signal.SIGCONT)
                    restored = True
                pressure.write_json(root / 'guardian-recovery.json', dict(time_unix=time.time(),
                    helper_owner=helper_owner, old_owner=old_owner, restored=restored,
                    reason='parallel helper exited before dispatcher transfer'))
                return
        except (OSError, ValueError, KeyError):
            # Losing registration is also a recovery condition, never permission
            # to kill or resume an unknown process.
            if live_exact(reader, old_owner) and reader.stat(old_owner['pid'])['state'] in ('T', 't'):
                signal_exact(reader, old_owner, signal.SIGCONT)
            return
        time.sleep(interval)
    if live_exact(reader, old_owner) and reader.stat(old_owner['pid'])['state'] in ('T', 't'):
        signal_exact(reader, old_owner, signal.SIGCONT)
        pressure.write_json(root / 'guardian-recovery.json', dict(time_unix=time.time(),
            helper_owner=helper_owner, old_owner=old_owner, restored=True, reason='guardian deadline'))


class Transition:
    def __init__(self, root, protocol, old_pid, *, timeout=6*3600, interval=1.):
        self.root, self.protocol, self.old_pid = Path(root).resolve(), protocol, old_pid
        self.timeout, self.interval, self.reader = timeout, interval, ProcReader()
        self.owner = identity(self.reader, os.getpid())
        self.old = None
        self.known = {}
        self.receipt = {}

    def state(self, phase, **extra):
        self.receipt.update(phase=phase, updated_unix=time.time(), **extra)
        pressure.write_json(self.root / 'handoff.json', self.receipt)

    def inspect(self):
        state = json.loads((self.root / 'run-state.json').read_text())
        if state.get('pid') != self.old_pid or state.get('status') != 'running':
            raise ValueError('old dispatcher is not the registered running owner')
        old = identity(self.reader, self.old_pid)
        command = self.reader.command(self.old_pid)
        if (str(Path(__file__).with_name('pressure.py')) not in command or
                Path(option(command, '--root')).resolve() != self.root):
            raise ValueError('old dispatcher command/root identity mismatch')
        if self.old is not None and old != self.old:
            raise ValueError('old dispatcher identity changed')
        self.old = old
        active = state.get('active', [])
        if len(active) > 1:
            raise ValueError('boundary handoff accepts at most one active native arm')
        table = self.reader.table()
        attributed = set()
        for row in active:
            pid = row['pid']
            expected = self.root / row['case'] / row['mode']
            for command in (row['command'], self.reader.command(pid) if pid in table else row['command']):
                if (str(Path(__file__).with_name('evaluate.py')) not in command or
                        option(command, '--mode') != row['mode'] or
                        Path(option(command, '--output')).resolve() != expected):
                    raise ValueError('registered native worker command/output mismatch')
            if pid in table:
                if not descendant(table, pid, self.old_pid) or pid == self.old_pid:
                    raise ValueError('registered worker is not an old dispatcher descendant')
                self.known[pid] = identity(self.reader, pid)
                for child in table:
                    if descendant(table, child, pid):
                        attributed.add(child)
                        self.known[child] = identity(self.reader, child)
            elif not (expected / 'summary.json').exists():
                raise ValueError('active worker disappeared without a terminal')
        unknown = [pid for pid in table if pid != self.old_pid and
                   descendant(table, pid, self.old_pid) and pid not in attributed]
        if unknown:
            raise ValueError(f'unregistered dispatcher descendants: {unknown}')
        terminals = audit_outputs(self.root, self.protocol, active)
        return dict(old_owner=self.old, active=active, terminal_episodes=len(terminals),
                    protocol_sha=pressure.sha(self.root / 'protocol.json'))

    def start_guardian(self):
        with (self.root / 'parallel-guardian.log').open('a') as log:
            process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--root', str(self.root),
                '--guard', '--guard-helper', json.dumps(self.owner), '--guard-old', json.dumps(self.old),
                '--handoff-timeout', str(self.timeout + 60)], stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            path = self.root / 'parallel-guardian.json'
            if path.exists():
                receipt = json.loads(path.read_text())
                if (receipt.get('helper_owner') == self.owner and receipt.get('old_owner') == self.old
                        and receipt.get('guardian_owner', {}).get('pid') == process.pid):
                    self.state('guardian_ready', guardian_owner=receipt['guardian_owner'])
                    return process
            if process.poll() is not None:
                raise RuntimeError('guardian failed before handoff')
            time.sleep(.1)
        raise TimeoutError('guardian readiness timeout; old dispatcher remains running')

    def restore(self):
        if self.old and live_exact(self.reader, self.old):
            if self.reader.stat(self.old_pid)['state'] in ('T', 't'):
                signal_exact(self.reader, self.old, signal.SIGCONT)
            self.state('restored_original')

    def check_guard(self, guardian_process):
        if guardian_process is None or guardian_process.poll() is not None:
            raise RuntimeError('independent guardian disappeared before dispatcher transfer')
        registered = self.receipt.get('guardian_owner')
        if registered and not live_exact(self.reader, registered):
            raise RuntimeError('independent guardian identity changed before transfer')

    def check_paused_boundary(self, guardian_process):
        self.check_guard(guardian_process)
        if (not live_exact(self.reader, self.old) or
                self.reader.stat(self.old_pid)['state'] not in ('T', 't')):
            raise RuntimeError('old dispatcher is no longer paused at the verified boundary')

    def handoff(self):
        snapshot = self.inspect()
        self.receipt = dict(schema='region_pressure_handoff_v1', helper_owner=self.owner,
                            old_owner=self.old, active=snapshot['active'], started_unix=time.time())
        self.state('validated')
        guardian_process = self.start_guardian()
        try:
            self.check_guard(guardian_process)
            signal_exact(self.reader, self.old, signal.SIGSTOP)
            deadline = time.monotonic() + 5
            while self.reader.stat(self.old_pid)['state'] not in ('T', 't'):
                if time.monotonic() > deadline:
                    raise TimeoutError('old dispatcher did not pause')
                time.sleep(.05)
            snapshot = self.inspect()  # Catch Popen/registration races after STOP.
            active = snapshot['active']
            self.state('terminal_wait', active=active)
            deadline, terminal_since = time.monotonic() + self.timeout, None
            while active:
                self.check_paused_boundary(guardian_process)
                row = active[0]
                table = self.reader.table()
                if not live_exact(self.reader, self.old):
                    raise RuntimeError('old dispatcher disappeared during native completion')
                for pid in table:
                    if descendant(table, pid, row['pid']):
                        self.known[pid] = identity(self.reader, pid)
                worker = self.known.get(row['pid'])
                if worker is None:
                    # Worker completed just before STOP; its terminal still must
                    # identify the registered PID and no owned descendants live.
                    worker = dict(pid=row['pid'], ppid=self.old_pid, start_ticks=-1)
                if terminal_closed(self.root, self.protocol, row, worker, self.reader, self.known.values()):
                    if live_exact(self.reader, worker):
                        terminal_since = terminal_since or time.monotonic()
                        if time.monotonic() - terminal_since >= 15:
                            signal_exact(self.reader, worker, signal.SIGTERM)
                    else:
                        break
                elif (self.root / row['case'] / row['mode'] / 'summary.json').exists():
                    terminal_since = terminal_since or time.monotonic()
                    if time.monotonic() - terminal_since >= 30:
                        raise RuntimeError('terminal exists but native descendants have not closed')
                if time.monotonic() > deadline:
                    raise TimeoutError('native completion boundary deadline; original dispatcher restored')
                time.sleep(self.interval)
            audit_outputs(self.root, self.protocol, [])
            self.check_paused_boundary(guardian_process)
            current = self.inspect()
            if current['active'] != active:
                raise ValueError('old dispatcher active registration changed at terminal boundary')
            self.state('old_interrupt_sent')
            signal_exact(self.reader, self.old, signal.SIGINT)
            signal_exact(self.reader, self.old, signal.SIGCONT)
            deadline = time.monotonic() + 30
            while live_exact(self.reader, self.old):
                if time.monotonic() > deadline:
                    raise TimeoutError('old dispatcher did not exit after completed native arm')
                time.sleep(self.interval)
            self.state('old_exited')
            if guardian_process is not None:
                guardian_process.wait(timeout=5)
            return self.receipt
        except BaseException as exc:
            self.state('handoff_error', error=repr(exc))
            self.restore()
            raise


def run_case_pool(dispatcher, cases, workers):
    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures = [pool.submit(dispatcher.one_case, case) for case in cases]
        try:
            for future in as_completed(futures):
                future.result()
        except BaseException:
            dispatcher.stop.set()
            for future in futures:
                future.cancel()
            raise


class MemoryAdmission:
    """Separate fixed reservation capacity from current host memory headroom."""
    def __init__(self, minimum_kb, stop, *, host=pressure.host_sample, interval=5.):
        self.minimum_kb, self.stop, self.host, self.interval = minimum_kb, stop, host, interval
        self.condition, self.reserved_kb = threading.Condition(), 0
        self.initial_available_kb = self.host()['MemAvailable_kb']
        self.capacity_kb = max(0, self.initial_available_kb - minimum_kb)

    @contextmanager
    def reserve(self, case):
        amount = (3 if case['extent'] >= 300 else 2)*1024*1024
        with self.condition:
            while (self.reserved_kb + amount > self.capacity_kb or
                   self.host()['MemAvailable_kb'] < self.minimum_kb + amount):
                if self.stop.is_set():
                    raise InterruptedError('stopped during conservative memory reservation')
                self.condition.wait(self.interval)
            if self.stop.is_set():
                raise InterruptedError('stopped before memory reservation')
            self.reserved_kb += amount
        try:
            yield
        finally:
            with self.condition:
                self.reserved_kb -= amount
                self.condition.notify_all()


class ParallelDispatcher(pressure.Dispatcher):
    def __init__(self, root, protocol, workers, minimum_memory_kb, override_sha):
        effective = copy.deepcopy(protocol)
        effective['minimum_available_memory_kb'] = minimum_memory_kb
        super().__init__(root, effective)
        self.workers, self.override_sha = workers, override_sha
        self.admission = MemoryAdmission(minimum_memory_kb, self.stop)
        self.event_lock, self.event_children = threading.Lock(), None
        # Carry the complete count into the first published new-owner snapshot;
        # completed arms do not briefly appear to reset while pool tasks resume.
        for case in self.protocol['cases']:
            for mode in case['mode_order']:
                terminal = pressure.validate_terminal(self.root/case['name']/mode, case, mode,
                                                       self.protocol['frozen'])
                if terminal is not None:
                    self.done[case['name']+'/'+mode] = dict(case=case['name'], mode=mode,
                        status=terminal['status'], completed=terminal.get('completed', False),
                        coverage=terminal.get('final_coverage'), decisions=terminal.get('decisions'),
                        error=terminal.get('error'))

    def event(self, kind, **extra):
        with self.event_lock:
            with (self.root / 'parallel-events.jsonl').open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(dict(time_unix=time.time(), event=kind, owner_pid=os.getpid(),
                    execution_override_sha=self.override_sha, **extra), ensure_ascii=False, allow_nan=False)+'\n')

    def state(self, **extra):
        super().state(execution_override_sha=self.override_sha, parallel_workers=self.workers,
                      memory_reserved_kb=self.admission.reserved_kb, **extra)
        with self.lock:
            active = list(self.children.values())
        members = sorted((row['pid'], row['case'], row['mode']) for row in active)
        with self.event_lock:
            changed = members != self.event_children
            self.event_children = members
        if changed:
            self.event('native_activity', active=active, active_native_count=len(active))

    def one_case(self, case):
        if all(pressure.validate_terminal(self.root/case['name']/mode, case, mode, self.protocol['frozen'])
               is not None for mode in case['mode_order']):
            return super().one_case(case)
        with self.admission.reserve(case):
            self.event('case_admitted', case=case['name'], extent=case['extent'],
                       reserved_kb=self.admission.reserved_kb, available_memory_kb=pressure.host_sample()['MemAvailable_kb'])
            try:
                return super().one_case(case)
            finally:
                self.event('case_released', case=case['name'])

    def run(self):
        # The original per-case method retains the frozen arm order, checked
        # resume, partial-output refusal, memory gate and exact child lifecycle.
        self.phase = 'large_parallel'
        self.state()
        run_case_pool(self, self.protocol['cases'], self.workers)
        self.phase = 'aggregate'
        self.state()
        command = ['bash', str(pressure.WRAPPER), 'python', '-m', 'scripts.drl.region_contraction.compare',
                   '--root', str(self.root), '--modes', 'baseline', 'remote', '--cases']
        command.extend(row['name'] for row in self.protocol['cases'])
        with (self.root / 'aggregate.log').open('a') as log:
            subprocess.run(command, cwd=pressure.REPO, env=dict(os.environ, DRL_CACHE=pressure.CACHE),
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        comparison = json.loads((self.root / 'comparison.json').read_text())
        self.state(finished_unix=time.time())
        state = json.loads((self.root / 'run-state.json').read_text())
        state.update(status='finished' if comparison['complete_pairing'] else 'finished_with_errors',
                     phase='finished', paired_cases=len(comparison['paired_cases']))
        pressure.write_json(self.root / 'run-state.json', state)


def inspect_service(root, pid, role, reader):
    source = Path(__file__).with_name('pressure_resources.py' if role == 'resources' else 'pressure_report.py')
    command = reader.command(pid)
    if str(source) not in command or Path(option(command, '--root')).resolve() != root:
        raise ValueError(f'{role} monitor command/root mismatch')
    if role == 'report' and '--watch' not in command:
        raise ValueError('report monitor is not the registered watch service')
    return dict(role=role, identity=identity(reader, pid), command=command,
                source_path=str(source), source_sha_on_disk=pressure.sha(source))


def restart_services(root, services, new_owner, *, reader=None):
    """Only exact monitor processes are retired; neither service owns workers."""
    reader = reader or ProcReader()
    receipt = dict(schema='region_pressure_handoff_services_v1', dispatcher_owner=new_owner,
                   started_unix=time.time(), previous=services, services=[], errors=[])
    for service in services:
        try:
            owner = service['identity']
            if original_process_alive(reader, owner):
                if not live_exact(reader, owner):
                    raise ValueError('monitor parent/identity changed before restart')
                current = inspect_service(root, owner['pid'], service['role'], reader)
                if current['command'] != service['command']:
                    raise ValueError('monitor command changed before restart')
                signal_exact(reader, owner, signal.SIGTERM)
                deadline = time.monotonic()+10
                while original_process_alive(reader, owner):
                    if time.monotonic()>deadline:
                        raise TimeoutError('old monitor did not release its owned service')
                    time.sleep(.1)
            command = [sys.executable, service['source_path'], '--root', str(root)]
            if service['role'] == 'resources':
                command += ['--expected-dispatcher-pid', str(new_owner['pid']), '--interval', '10']
            else:
                command += ['--watch']
            with (root / ('parallel-'+service['role']+'.log')).open('a') as log:
                process = subprocess.Popen(command, cwd=REPO, env=dict(os.environ,
                    MPLCONFIGDIR='/tmp/region-pressure-mpl'), stdin=subprocess.DEVNULL,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(.1)
            if process.poll() is not None:
                raise RuntimeError(f'new {service["role"]} monitor exited {process.returncode}; inspect its log')
            receipt['services'].append(dict(role=service['role'], command=command,
                identity=identity(reader, process.pid), source_sha=pressure.sha(service['source_path'])))
        except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
            # Monitoring failure remains visible but does not abort good policy
            # episodes or replace them with infrastructure errors.
            receipt['errors'].append(dict(role=service['role'], error=repr(exc)))
        pressure.write_json(root / 'handoff-services.json', receipt)
    receipt.update(status='running' if not receipt['errors'] else 'monitor_attention', updated_unix=time.time())
    pressure.write_json(root / 'handoff-services.json', receipt)
    return receipt

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--workers', type=int, choices=(2, 3, 4), default=4)
    parser.add_argument('--handoff-pid', type=int)
    parser.add_argument('--minimum-memory-gib', type=int, choices=(6,), default=6)
    parser.add_argument('--resource-pid', type=int)
    parser.add_argument('--report-pid', type=int)
    parser.add_argument('--handoff-timeout', type=float, default=6*3600)
    parser.add_argument('--dry-run', '--validate-only', action='store_true')
    parser.add_argument('--guard', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--guard-helper', help=argparse.SUPPRESS)
    parser.add_argument('--guard-old', help=argparse.SUPPRESS)
    args = parser.parse_args()
    root = args.root.resolve()
    if not 1 <= args.handoff_timeout <= 12*3600:
        parser.error('handoff timeout must be 1 second to 12 hours')
    if args.guard:
        helper, old = json.loads(args.guard_helper), json.loads(args.guard_old)
        receipt = json.loads((root / 'handoff.json').read_text())
        if receipt.get('helper_owner') != helper or receipt.get('old_owner') != old:
            raise ValueError('guardian registration mismatch')
        pressure.write_json(root / 'parallel-guardian.json', dict(helper_owner=helper, old_owner=old,
            guardian_owner=identity(ProcReader(), os.getpid()), created_unix=time.time()))
        guardian(root, helper, old, timeout=args.handoff_timeout)
        return
    if not args.handoff_pid:
        parser.error('--handoff-pid is required')
    # Read-only validation deliberately does not call initialize/mkdir.
    protocol = json.loads((root / 'protocol.json').read_text())
    if protocol.get('cases') != pressure.build_cases() or protocol.get('modes') != ['baseline', 'remote']:
        raise ValueError('frozen pressure matrix changed')
    pressure.verify_frozen(root, protocol)
    transition = Transition(root, protocol, args.handoff_pid, timeout=args.handoff_timeout)
    snapshot = transition.inspect()
    if bool(args.resource_pid) != bool(args.report_pid):
        parser.error('--resource-pid and --report-pid must be supplied together')
    services = [inspect_service(root, pid, role, transition.reader) for role, pid in
                (('resources', args.resource_pid), ('report', args.report_pid)) if pid is not None]
    if args.dry_run:
        print(json.dumps(dict(**snapshot, workers=args.workers, minimum_memory_gib=args.minimum_memory_gib,
            available_memory_kb=pressure.host_sample()['MemAvailable_kb'], monitor_services=services), ensure_ascii=False))
        return
    with (root / '.parallel-transition.lock').open('a') as transition_lock:
        fcntl.flock(transition_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        override_path = root / 'execution-override.json'
        override = dict(schema='region_pressure_execution_override_v1', protocol_sha=snapshot['protocol_sha'],
            helper_path=str(Path(__file__).resolve()), helper_sha=pressure.sha(__file__),
            original_owner=snapshot['old_owner'], current_arm=snapshot['active'], workers=args.workers,
            minimum_available_memory_kb=args.minimum_memory_gib*1024*1024, created_unix=time.time(),
            case_reservation_kb={'200m_or_less':2*1024*1024, '300m':3*1024*1024},
            memory_admission='Inflight reservations plus candidate <= initial MemAvailable minus6GiB; '
                             'current MemAvailable >=6GiB plus candidate reservation. Workers is a cap.',
            monitoring_services=services,
            authorization='User requested concurrent scenes to finish pressure evaluation faster.',
            scope='Scheduling only; frozen model/runtime/navigation/rewards/scenes/budgets remain unchanged.',
            timing_note='Original serial and continued concurrent phases have different host contention. '
                        'Episode wall time is not an isolated policy-speed comparison across phases.')
        with override_path.open('x', encoding='utf-8') as stream:
            json.dump(override, stream, ensure_ascii=False, allow_nan=False, indent=2); stream.write('\n')
        receipt = transition.handoff()
        # No main state/host/aggregate writes occur before exclusive lock transfer.
        with (root / '.dispatch.lock').open('a') as dispatch_lock:
            fcntl.flock(dispatch_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            pressure.verify_frozen(root, protocol)
            audit_outputs(root, protocol, [])
            transition.state('dispatch_lock_acquired', parallel_owner=transition.owner,
                             execution_override_sha=pressure.sha(override_path))
            dispatcher = ParallelDispatcher(root, protocol, args.workers,
                override['minimum_available_memory_kb'], pressure.sha(override_path))
            def interrupted(signum, frame):
                dispatcher.stop.set()
                raise KeyboardInterrupt(f'parallel dispatcher signal {signum}')
            signal.signal(signal.SIGTERM, interrupted)
            signal.signal(signal.SIGINT, interrupted)
            transition.state('parallel_running')
            dispatcher.state()
            service_receipt = restart_services(root, services, transition.owner)
            transition.state('parallel_running', monitoring_status=service_receipt['status'],
                             monitoring_errors=service_receipt['errors'])
            try:
                dispatcher.run()
                transition.state('parallel_finished')
            except BaseException as exc:
                pressure.write_json(root / 'parallel-dispatcher-error.json',
                    dict(error=repr(exc), traceback=traceback.format_exc()))
                state = json.loads((root / 'run-state.json').read_text())
                state.update(status='error', error=repr(exc), updated_unix=time.time())
                pressure.write_json(root / 'run-state.json', state)
                raise


if __name__ == '__main__':
    main()
