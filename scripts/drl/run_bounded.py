#!/usr/bin/env python3
"""Supervise one training process with shared storage limits and bounded output.

Initialize the artifact root before invoking this script. The supervised command
runs in its own process group; SIGINT/SIGTERM are forwarded, and its original
return code is recorded even when storage pressure causes the interruption.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time


# The launcher deliberately uses only the standard library and this stdlib-only
# module from the matching checkout, independent of ROS/venv overlay imports.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
                       'ros2_ws/src/lunar_drl_exploration'))
from lunar_drl_exploration.artifacts import (  # noqa: E402
    ResourceLimitError, append_bounded, artifact_usage, atomic_json,
)


CONTROL_MAX_BYTES = 16 * 1024
STOP_RESERVE_BYTES = 32 * 1024


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def shell_exit(returncode):
    return 128 - returncode if returncode < 0 else returncode


def process_start_ticks(pid):
    # Linux process identity remains unambiguous after PID reuse or a reboot
    # when combined with the boot ID recorded in launch.json.
    return int(Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19])


def send_group(child, signum):
    if child is not None:
        try:
            os.killpg(child.pid, signum)
        except ProcessLookupError:
            pass


def close_owned_group(child):
    """EOF only closes capture; descendants with redirected stdio may remain."""
    send_group(child, signal.SIGTERM)
    deadline = time.monotonic() + 3.
    while time.monotonic() < deadline:
        try:
            os.killpg(child.pid, 0)
        except ProcessLookupError:
            return
        time.sleep(.02)
    send_group(child, signal.SIGKILL)


def supervise(args):
    # Validate the managed root before creating any file or spawning work.
    requested_root = Path(args.artifact_root).absolute()
    usage = artifact_usage(requested_root)
    root = Path(usage['root'])
    if root != requested_root.resolve():
        raise ValueError('--artifact-root must name the initialized root itself')
    if requested_root.is_symlink():
        raise ValueError('artifact root must not be a symlink')
    lock_fd = os.open(root / '.execution.lock',
                      os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('a training wrapper is already active for this artifact root') from exc
        return supervise_locked(args, root)
    finally:
        os.close(lock_fd)


def supervise_locked(args, root):
    child = None
    group_cleaned = False
    reason = 'child_exit'
    caught_signal = None
    stop_time = None
    error = None
    stop_stage = 0
    started_at = utc_now()
    old_handlers = {}

    def request_stop(signum, why):
        nonlocal reason, caught_signal, stop_time
        if stop_time is None:
            reason = why
            caught_signal = signum
            stop_time = time.monotonic()
            send_group(child, signum)

    def on_signal(signum, _frame):
        request_stop(signum, 'signal')

    def check_budget():
        usage = artifact_usage(root)
        for kind in ('total', 'diagnostic'):
            if usage[f'{kind}_max_bytes'] - usage[f'{kind}_bytes'] <= STOP_RESERVE_BYTES:
                raise ResourceLimitError(f'{kind} budget shutdown reserve',
                    usage[f'{kind}_bytes'], usage[f'{kind}_max_bytes'] - STOP_RESERVE_BYTES)

    for signum in (signal.SIGINT, signal.SIGTERM):
        old_handlers[signum] = signal.signal(signum, on_signal)
    selector = selectors.DefaultSelector()
    try:
        check_budget()
        atomic_json(root / 'exit.json', {'status': 'running', 'started_at': started_at,
                    'wrapper_pid': os.getpid()}, max_bytes=CONTROL_MAX_BYTES)
        environment = dict(os.environ, PYTHONUNBUFFERED='1', ROS_LOG_DIR=str(root / 'ros'))
        try:
            child = subprocess.Popen(args.command, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                start_new_session=True, env=environment, close_fds=True)
        except OSError as exc:
            atomic_json(root / 'exit.json', {'status': 'finished',
                'started_at': started_at, 'finished_at': utc_now(),
                'wrapper_pid': os.getpid(), 'child_returncode': None,
                'exit_code': 75, 'reason': 'launch_error', 'error': str(exc)},
                max_bytes=CONTROL_MAX_BYTES)
            raise
        if stop_time is not None:
            send_group(child, caught_signal)
        atomic_json(root / 'launch.json', {'started_at': started_at,
            'wrapper_pid': os.getpid(), 'child_pid': child.pid,
            'wrapper_start_ticks': process_start_ticks(os.getpid()),
            'child_start_ticks': process_start_ticks(child.pid),
            'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'child_process_group': child.pid, 'command': args.command,
            'cwd': os.getcwd(), 'artifact_root': str(root),
            'log_limit_bytes': args.log_limit_bytes, 'ros_log_dir': environment['ROS_LOG_DIR']},
            max_bytes=CONTROL_MAX_BYTES)
        os.set_blocking(child.stdout.fileno(), False)
        selector.register(child.stdout, selectors.EVENT_READ)
        next_budget_check = 0.
        child_exit_time = None
        while selector.get_map() or child.poll() is None:
            now = time.monotonic()
            if stop_time is None and now >= next_budget_check:
                try:
                    check_budget()
                except (ResourceLimitError, OSError, ValueError) as exc:
                    error = str(exc)
                    request_stop(signal.SIGINT, 'storage_limit')
                next_budget_check = now + .5
            if stop_time is not None:
                elapsed = now - stop_time
                if stop_stage == 0 and elapsed >= args.stop_grace_seconds:
                    send_group(child, signal.SIGTERM)
                    stop_stage = 1
                if stop_stage == 1 and elapsed >= args.stop_grace_seconds + 5.:
                    send_group(child, signal.SIGKILL)
                    stop_stage = 2
            for key, _ in selector.select(timeout=.1):
                try:
                    chunk = os.read(key.fd, 64 * 1024)
                except BlockingIOError:
                    continue
                if not chunk:
                    selector.unregister(key.fileobj)
                    continue
                if reason != 'storage_limit':
                    try:
                        append_bounded(root / 'runtime.log', chunk, args.log_limit_bytes)
                    except (ResourceLimitError, OSError, ValueError) as exc:
                        error = str(exc)
                        request_stop(signal.SIGINT, 'storage_limit')
                        # A prior user signal must not permit further failed writes.
                        reason = 'storage_limit'
            if child.poll() is not None:
                if child_exit_time is None:
                    child_exit_time = now
                # A crashed command can leave a descendant holding the pipe open.
                # Do not let it retain the root lock or write logs indefinitely.
                if now - child_exit_time > 1.:
                    send_group(child, signal.SIGTERM)
                if now - child_exit_time > 3.:
                    send_group(child, signal.SIGKILL)
                    break
        returncode = child.wait()
        close_owned_group(child)
        group_cleaned = True
        exit_code = shell_exit(returncode)
        if reason == 'storage_limit' and exit_code in (0, 130, 143, 137):
            exit_code = 75
        elif reason == 'signal' and exit_code == 0:
            exit_code = 128 + caught_signal
        status = {'status': 'finished', 'started_at': started_at, 'finished_at': utc_now(),
            'wrapper_pid': os.getpid(), 'child_pid': child.pid,
            'child_returncode': returncode, 'exit_code': exit_code,
            'reason': reason, 'signal': caught_signal, 'error': error,
            'forced_stop_stage': stop_stage}
        try:
            atomic_json(root / 'exit.json', status, max_bytes=CONTROL_MAX_BYTES)
        except (ResourceLimitError, OSError, ValueError) as exc:
            # A non-cooperating external writer can exhaust even the reserved
            # shutdown space. Preserve the prior record, never exceed the cap.
            print(json.dumps(dict(status, exit_record_error=str(exc))), file=sys.stderr)
            if exit_code == 0:
                exit_code = 75
        return exit_code
    finally:
        selector.close()
        if child is not None:
            if child.poll() is None:
                send_group(child, signal.SIGINT)
                try:
                    child.wait(timeout=args.stop_grace_seconds)
                except subprocess.TimeoutExpired:
                    send_group(child, signal.SIGTERM)
                    try:
                        child.wait(timeout=5.)
                    except subprocess.TimeoutExpired:
                        send_group(child, signal.SIGKILL)
                        child.wait()
            child.stdout.close()
            if not group_cleaned:
                close_owned_group(child)
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--artifact-root', required=True)
    parser.add_argument('--log-limit-bytes', type=int, default=64 * 1024 * 1024)
    parser.add_argument('--stop-grace-seconds', type=float, default=120.)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if args.command and args.command[0] == '--':
        args.command = args.command[1:]
    if not args.command:
        parser.error('a command after -- is required')
    if not 0 < args.log_limit_bytes <= 64 * 1024 * 1024:
        parser.error('--log-limit-bytes must be between 1 and 67108864')
    if not 0 < args.stop_grace_seconds <= 600:
        parser.error('--stop-grace-seconds must be between 0 and 600')
    try:
        return supervise(args)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'bounded launcher: {exc}', file=sys.stderr)
        return 75


if __name__ == '__main__':
    sys.exit(main())
