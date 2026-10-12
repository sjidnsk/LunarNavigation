"""One isolated, serial retry after the immutable 40-arm pressure batch exits."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[3]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
from scripts.drl.region_contraction import pressure
from scripts.drl.region_contraction.pressure_resources import ProcReader, descendant, parse_stat


def inspected_stat(reader, pid):
    try:
        return reader.stat(pid)
    except (FileNotFoundError, ProcessLookupError):
        return None
    except ValueError:
        try:
            row = parse_stat((reader.root/str(pid)/'stat').read_text())
        except (FileNotFoundError, ProcessLookupError):
            return None
        if row['state'] not in ('Z','X','x'):
            raise ValueError('process identity cannot be verified')
        return row


def live_natives(reader, known):
    live = []
    for pid, start in known.items():
        row = inspected_stat(reader, pid)
        if row and row['start_ticks'] == start and row['state'] not in ('Z','X','x'):
            live.append(pid)
    return live


def close_natives(job, reader, known):
    actions = []
    for signum in (signal.SIGTERM, signal.SIGKILL):
        for pid in live_natives(reader, known):
            try:
                fd = os.pidfd_open(pid)
            except ProcessLookupError:
                continue
            try:
                if pid in live_natives(reader, {pid:known[pid]}):
                    signal.pidfd_send_signal(fd, signum)
                    actions.append(dict(pid=pid, start_ticks=known[pid], signal=signum.name))
            except ProcessLookupError:
                pass
            finally:
                os.close(fd)
        deadline = time.monotonic()+2
        while live_natives(reader, known) and time.monotonic()<deadline:
            time.sleep(.05)
    pressure.write_json(job/'native-cleanup.json', dict(registered=known, actions=actions,
        remaining=live_natives(reader, known), updated_unix=time.time()))


def batch_gate(root, protocol, owner, reader=None):
    reader = reader or ProcReader()
    state = json.loads((root/'run-state.json').read_text())
    if state.get('pid') != owner['pid']:
        return 'blocked', 'formal dispatcher registration changed'
    try:
        live = inspected_stat(reader, owner['pid'])
    except (OSError, ValueError) as exc:
        return 'blocked', 'formal dispatcher exit cannot be verified: '+str(exc)
    if live and live['start_ticks'] != owner['start_ticks']:
        return 'blocked', 'formal dispatcher PID was reused'
    if state.get('status') == 'error':
        return 'blocked', 'formal batch stopped with a controller error'
    if state.get('status') not in ('finished', 'finished_with_errors'):
        return 'queued', 'waiting for formal batch terminal state'
    if state.get('terminal_episodes') != 40 or state.get('active') != []:
        return 'blocked', 'formal batch terminal is incomplete or has active workers'
    for case in protocol['cases']:
        for mode in case['mode_order']:
            try:
                row = pressure.validate_terminal(root/case['name']/mode, case, mode, protocol['frozen'])
            except (OSError, ValueError) as exc:
                return 'blocked', 'formal terminal validation failed: '+str(exc)
            if row is None:
                return 'blocked', 'formal arm lacks a terminal: '+case['name']+'/'+mode
    alive = live and live['state'] not in ('Z','X','x')
    return ('queued', 'waiting for formal dispatcher process exit') if alive else ('ready', 'formal batch closed')


def evaluation_command(root, case, mode, output):
    return ['bash', str(pressure.WRAPPER), 'python', str(Path(__file__).with_name('evaluate.py')),
        '--mode', mode, '--config', str(root/'config.json'), '--actor', str(root/'actor.pt'),
        '--output', str(output), '--family', case['family'], '--extent', str(case['extent']),
        '--seed', str(case['seed']), '--budget', str(case['budget']), '--max-remote-positions', '32']


def create_job(root, protocol, owner, case, mode, label):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', label):
        raise ValueError('retry label must be a single safe path component')
    source = root/case['name']/mode
    failure = pressure.validate_terminal(source, case, mode, protocol['frozen'])
    if failure is None or failure['status'] != 'error':
        raise ValueError('retry requires the preserved original error terminal')
    job = root/'retests'/label
    job.mkdir(parents=True)  # Atomic duplicate rejection, including partial retry jobs.
    request = dict(schema='region_pressure_retry_request_v1', case=case, mode=mode,
        formal_owner=owner, protocol_sha=pressure.sha(root/'protocol.json'), frozen=protocol['frozen'],
        helper_path=str(Path(__file__).resolve()), helper_sha=pressure.sha(__file__),
        source_failure=str(source/'summary.json'), source_failure_sha=pressure.sha(source/'summary.json'),
        authorization='User explicitly requested cave200-r2/A2 retest (annotation 1).',
        condition='40/40 formal terminal arms, no active arm, formal owner exited, exclusive dispatch lock.',
        timing_note='Separate serial retry; retained original error and excluded from the formal 40-arm matrix.',
        created_unix=time.time())
    with (job/'request.json').open('x', encoding='utf-8') as stream:
        json.dump(request, stream, ensure_ascii=False, allow_nan=False, indent=2); stream.write('\n')
    return job


def retry_state(job, status, **extra):
    pressure.write_json(job/'retry-state.json', dict(schema='region_pressure_retry_state_v1',
        status=status, pid=os.getpid(), request_sha=pressure.sha(job/'request.json'),
        updated_unix=time.time(), **extra))


def evaluate_retry(root, job, protocol, case, mode, *, poll_interval=3., native_timeout=60.):
    output = job/'evaluation'
    output.mkdir()  # Never replace a completed or partial previous attempt.
    pressure.write_json(output/'dispatch.json', dict(case=case, mode=mode, frozen=protocol['frozen']))
    command = evaluation_command(root, case, mode, output)
    reader, known, terminal_since, attention = ProcReader(), {}, None, None
    with (job/'evaluation.log').open('x') as log:
        process = subprocess.Popen(['python3', '-c', pressure.DEATH_BOUND, str(os.getpid()), *command],
            cwd=REPO, env=dict(os.environ, DRL_CACHE=pressure.CACHE), stdin=subprocess.DEVNULL,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            retry_state(job, 'running', worker_pid=process.pid, command=command,
                        launched_unix=time.time(), host=pressure.host_sample())
            while process.poll() is None:
                table = reader.table()
                for pid in table:
                    if pid != process.pid and descendant(table, pid, process.pid):
                        known[pid] = table[pid]['start_ticks']
                if (output/'summary.json').exists():
                    saved = pressure.validate_terminal(output, case, mode, protocol['frozen'])
                    if saved.get('pid') != process.pid:
                        raise ValueError('post-terminal retry worker PID mismatch')
                    terminal_since = terminal_since or time.monotonic()
                    native_live = live_natives(reader, known)
                    if native_live and time.monotonic()-terminal_since > native_timeout:
                        attention = 'registered native descendants remained live60s after terminal; bounded cleanup'
                        retry_state(job, 'attention', reason=attention, worker_pid=process.pid, result=saved)
                        break
                    if time.monotonic()-terminal_since > 15 and not native_live:
                        process.terminate()  # Exact Popen child; terminal followed native close.
                        break
                time.sleep(poll_interval)
            if process.poll() is None:
                pressure.close_owned(process)
            code = process.wait(timeout=20)
        finally:
            try:
                pressure.close_owned(process)
            finally:
                if live_natives(reader, known):
                    attention = attention or 'owned retry parent exited while registered native descendants remained live'
                close_natives(job, reader, known)
    result = pressure.validate_terminal(output, case, mode, protocol['frozen'])
    if result is None:
        result = dict(status='error', mode=mode, family=case['family'], extent_m=case['extent'],
            seed=case['seed'], budget=case['budget'], ended=0, successes=0, completed=False,
            infrastructure_failures=1, error_category='infrastructure', exit_code=code,
            error=f'owned retry worker exited {code} without terminal artifact', provenance=protocol['frozen'])
        pressure.write_json(output/'summary.json', result)
        pressure.write_json(output/'status.json', result)
    pressure.write_json(output/'process.json', dict(pid=process.pid, exit_code=code, command=command))
    retry_state(job, 'attention' if attention else result['status'], reason=attention, result=result,
                finished_unix=time.time(), host=pressure.host_sample())
    return result


def assert_request_identity(root, job, protocol, request_sha):
    pressure.verify_frozen(root, protocol)
    request = json.loads((job/'request.json').read_text())
    if (request_sha != pressure.sha(job/'request.json') or request['helper_sha'] != pressure.sha(__file__) or
        request['protocol_sha'] != pressure.sha(root/'protocol.json') or
        request['source_failure_sha'] != pressure.sha(request['source_failure'])):
        raise ValueError('immutable retry request identity changed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--case', required=True)
    parser.add_argument('--mode', choices=('baseline','remote'), required=True)
    parser.add_argument('--label', required=True)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(); root = args.root.resolve()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', args.label):
        raise ValueError('retry label must be a single safe path component')
    protocol = json.loads((root/'protocol.json').read_text())
    if protocol.get('cases') != pressure.build_cases() or protocol.get('modes') != ['baseline','remote']:
        raise ValueError('formal pressure matrix changed')
    pressure.verify_frozen(root, protocol)
    case = next(row for row in protocol['cases'] if row['name'] == args.case)
    owner = json.loads((root/'handoff.json').read_text())['parallel_owner']
    failure = pressure.validate_terminal(root/args.case/args.mode, case, args.mode, protocol['frozen'])
    if failure is None or failure['status'] != 'error':
        raise ValueError('source arm is not a preserved error terminal')
    gate, reason = batch_gate(root, protocol, owner)
    if args.dry_run:
        print(json.dumps(dict(status=gate, reason=reason, case=case, mode=args.mode,
            output=str(root/'retests'/args.label/'evaluation'), formal_owner=owner, source_failure=failure)))
        return
    job = create_job(root, protocol, owner, case, args.mode, args.label)
    request_sha = pressure.sha(job/'request.json')
    with (job/'.retry.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            def interrupted(signum, frame):
                raise KeyboardInterrupt(f'retry controller signal {signum}')
            signal.signal(signal.SIGINT, interrupted); signal.signal(signal.SIGTERM, interrupted)
            while gate == 'queued':
                assert_request_identity(root, job, protocol, request_sha)
                retry_state(job, gate, reason=reason, formal_owner=owner)
                time.sleep(10)
                gate, reason = batch_gate(root, protocol, owner)
            if gate != 'ready':
                retry_state(job, 'blocked', reason=reason); return
            with (root/'.dispatch.lock').open('a') as dispatch_lock:
                fcntl.flock(dispatch_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                assert_request_identity(root, job, protocol, request_sha)
                if batch_gate(root, protocol, owner)[0] != 'ready':
                    raise ValueError('formal serial admission changed after dispatch lock')
                evaluate_retry(root, job, protocol, case, args.mode)
                assert_request_identity(root, job, protocol, request_sha)
        except BaseException as exc:
            retry_state(job, 'blocked', reason=repr(exc), host=pressure.host_sample())
            raise


if __name__ == '__main__':
    main()
