"""Run real child processes to verify bounded capture and supervision."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest


ROOT = Path(__file__).resolve().parents[4]
LAUNCHER = ROOT / 'scripts/drl/run_bounded.py'


def test_launcher_is_runnable_without_ros_or_numerical_imports():
    result = subprocess.run([sys.executable, '-S', str(LAUNCHER), '--help'],
                            capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr.decode()
    assert b'--artifact-root' in result.stdout


@pytest.fixture
def artifact_root(tmp_path):
    from lunar_drl_exploration.artifacts import initialize_artifact_root
    root = tmp_path / 'artifacts'
    initialize_artifact_root(root, total_max_bytes=1024 * 1024,
                             diagnostic_max_bytes=512 * 1024)
    return root


def command(root, code, *options):
    return [sys.executable, str(LAUNCHER), '--artifact-root', str(root),
            '--log-limit-bytes', '4096', '--stop-grace-seconds', '2',
            *options, '--', sys.executable, '-u', '-c', code]


def wait_for(predicate, process, seconds=10):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        assert process.poll() is None, process.communicate()
        time.sleep(.02)
    raise AssertionError('child readiness was not observed')


def test_large_output_keeps_recent_bytes_and_original_failure(artifact_root):
    result = subprocess.run(command(artifact_root,
        "import os,sys; os.write(1,b'x'*100000); "
        "os.write(2,b'ORIGINAL FAILURE\\n'); sys.exit(23)"),
        capture_output=True, timeout=10)
    assert result.returncode == 23, result.stderr.decode()
    content = (artifact_root / 'runtime.log').read_bytes()
    assert len(content) <= 4096
    assert content.endswith(b'ORIGINAL FAILURE\n')
    status = json.loads((artifact_root / 'exit.json').read_text())
    assert status['child_returncode'] == 23
    assert status['exit_code'] == 23
    assert status['reason'] == 'child_exit'


@pytest.mark.parametrize('forwarded_signal', [signal.SIGINT, signal.SIGTERM])
def test_signals_reach_child_and_wait_for_cleanup(artifact_root, forwarded_signal):
    code = ("import signal,time,sys; "
            "signal.signal(signal.SIGINT,lambda *a:sys.exit(31)); "
            "signal.signal(signal.SIGTERM,lambda *a:sys.exit(32)); "
            "print('READY',flush=True); time.sleep(60)")
    proc = subprocess.Popen(command(artifact_root, code),
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        log = artifact_root / 'runtime.log'
        wait_for(lambda: log.exists() and b'READY' in log.read_bytes(), proc)
        proc.send_signal(forwarded_signal)
        _, stderr = proc.communicate(timeout=10)
        assert proc.returncode == (31 if forwarded_signal == signal.SIGINT else 32), stderr
        status = json.loads((artifact_root / 'exit.json').read_text())
        assert status['reason'] == 'signal'
        assert status['signal'] == forwarded_signal
        assert status['child_returncode'] == proc.returncode
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=10)


def test_one_active_wrapper_per_artifact_root(artifact_root):
    first = subprocess.Popen(command(artifact_root,
        "import time; print('READY',flush=True); time.sleep(60)"),
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        log = artifact_root / 'runtime.log'
        wait_for(lambda: log.exists() and b'READY' in log.read_bytes(), first)
        before = (artifact_root / 'launch.json').read_bytes()
        second = subprocess.run(command(artifact_root, "print('SECOND')"),
                                capture_output=True, timeout=10)
        assert second.returncode != 0
        assert b'already active' in second.stderr
        assert (artifact_root / 'launch.json').read_bytes() == before
        assert b'SECOND' not in log.read_bytes()
    finally:
        first.send_signal(signal.SIGINT)
        first.communicate(timeout=10)


def test_budget_pressure_interrupts_without_growing_log(artifact_root):
    # A competing non-cooperating producer simulates an external ROS log. The
    # wrapper must detect it even when its own child produces no further output.
    proc = subprocess.Popen(command(artifact_root,
        "import signal,sys,time; "
        "signal.signal(signal.SIGINT,lambda *a:sys.exit(0)); "
        "print('READY',flush=True); time.sleep(60)"),
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        log = artifact_root / 'runtime.log'
        wait_for(lambda: log.exists() and b'READY' in log.read_bytes(), proc)
        initial_size = log.stat().st_size
        from lunar_drl_exploration.artifacts import artifact_usage
        current = artifact_usage(artifact_root)
        remaining = current['diagnostic_max_bytes'] - current['diagnostic_bytes']
        (artifact_root / 'unexpected.log').write_bytes(b'x' * (remaining - 24576))
        proc.communicate(timeout=10)
        assert proc.returncode == 75
        assert log.stat().st_size == initial_size
        status = json.loads((artifact_root / 'exit.json').read_text())
        assert status['reason'] == 'storage_limit'
        assert status['child_returncode'] == 0
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            proc.communicate(timeout=10)


def test_refuses_unmanaged_root_before_starting_child(tmp_path):
    marker = tmp_path / 'child-was-started'
    result = subprocess.run(command(tmp_path, f'open({str(marker)!r},"w").close()'),
                            capture_output=True, timeout=10)
    assert result.returncode != 0
    assert not marker.exists()


def test_launch_failure_leaves_finished_status(artifact_root):
    result = subprocess.run([sys.executable, str(LAUNCHER), '--artifact-root',
        str(artifact_root), '--', '/this/training-command-does-not-exist'],
        capture_output=True, timeout=10)
    assert result.returncode != 0
    status = json.loads((artifact_root / 'exit.json').read_text())
    assert status['status'] == 'finished'
    assert status['reason'] == 'launch_error'
    assert status['child_returncode'] is None
    assert status['exit_code'] == result.returncode


def test_launch_identity_and_signal_cover_child_process_group(artifact_root):
    # The direct child deliberately ignores SIGINT; its child must receive the
    # group signal, exit, and let the direct child finish without forced kill.
    grandchild_code = ("import signal,sys,time; "
        "signal.signal(signal.SIGINT,lambda *a:sys.exit(0)); "
        "print('GRANDCHILD READY',flush=True); time.sleep(60)")
    code = ("import subprocess,sys,signal; "
        "signal.signal(signal.SIGINT,signal.SIG_IGN); "
        f"p=subprocess.Popen([sys.executable,'-u','-c',{grandchild_code!r}]); "
        "sys.exit(p.wait())")
    proc = subprocess.Popen(command(artifact_root, code),
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        log = artifact_root / 'runtime.log'
        wait_for(lambda: log.exists() and b'GRANDCHILD READY' in log.read_bytes(), proc)
        launch = json.loads((artifact_root / 'launch.json').read_text())
        assert launch['wrapper_pid'] == proc.pid
        assert launch['wrapper_start_ticks'] == int(
            Path(f'/proc/{proc.pid}/stat').read_text().rsplit(')', 1)[1].split()[19])
        assert launch['boot_id'] == Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        assert launch['child_start_ticks'] > 0
        proc.send_signal(signal.SIGINT)
        _, stderr = proc.communicate(timeout=10)
        assert proc.returncode == 130, stderr
        status = json.loads((artifact_root / 'exit.json').read_text())
        assert status['child_returncode'] == 0
        assert status['forced_stop_stage'] == 0
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGTERM)
            proc.communicate(timeout=10)


def test_failed_child_leaves_no_detached_stdio_descendant(artifact_root):
    marker = artifact_root / 'descendant-pid.json'
    code = ("import json,subprocess,sys; "
        "p=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],"
        "stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL); "
        f"open({str(marker)!r},'w').write(json.dumps(p.pid)); "
        "sys.exit(23)")
    descendant = None
    try:
        result = subprocess.run(command(artifact_root, code),
                                capture_output=True, timeout=15)
        descendant = json.loads(marker.read_text())
        assert result.returncode == 23, result.stderr
        stat_path = Path(f'/proc/{descendant}/stat')
        assert not stat_path.exists() or stat_path.read_text().rsplit(')', 1)[1].split()[0] == 'Z'
        status = json.loads((artifact_root / 'exit.json').read_text())
        assert status['child_returncode'] == 23 and status['exit_code'] == 23
    finally:
        if descendant is not None:
            try:
                os.kill(descendant, signal.SIGKILL)
            except ProcessLookupError:
                pass
