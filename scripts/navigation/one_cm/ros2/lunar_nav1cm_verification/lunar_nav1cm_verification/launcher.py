"""有界启动器：只管理自己创建的三个进程，退出后离线认证。"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from time import monotonic, sleep
import uuid
from .contract import validate_environment
from .runtime import parser


def stop_children(children):
    for process in children.values():
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
    deadline = monotonic() + 8.
    for process in children.values():
        try:
            process.wait(timeout=max(.01, deadline - monotonic()))
        except subprocess.TimeoutExpired:
            process.terminate()
    deadline = monotonic() + 2.
    for process in children.values():
        try:
            process.wait(timeout=max(.01, deadline - monotonic()))
        except subprocess.TimeoutExpired:
            process.kill()
    for process in children.values():
        process.wait(timeout=2.)


def main(argv=None):
    p = parser(require_session=False)
    # launcher 生成 session；直接运行单节点仍要求显式 session。
    args = p.parse_args(argv)
    args.session = args.session or uuid.uuid4().hex
    os.environ.setdefault('ROS_DOMAIN_ID', '87')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    validate_environment(os.environ)
    if not 0 < args.timeout_s <= 300 or not 0 < args.startup_timeout_s <= 300:
        raise ValueError('bounded startup/runtime required')
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=False)
    children, logs = {}, {}
    stopped = False
    def stop(signum, frame):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        for role in ('probe', 'navigator', 'fixture'):
            logs[role] = (out / (role + '-console.log')).open('x', encoding='utf-8')
            command = [sys.executable, '-m', 'lunar_nav1cm_verification.runtime', '--role', role,
                       '--session', args.session, '--output-dir', str(out), '--build-dir', args.build_dir,
                       '--geometry-lib', args.geometry_lib, '--distance-field-lib', args.distance_field_lib,
                       '--timeout-s', str(args.timeout_s), '--startup-timeout-s', str(args.startup_timeout_s),
                       '--delay-ms', str(args.delay_ms)]
            children[role] = subprocess.Popen(command, stdout=logs[role], stderr=subprocess.STDOUT, env=dict(os.environ))
        deadline = monotonic() + args.startup_timeout_s + args.timeout_s + 5.
        while not stopped and monotonic() < deadline:
            if any(proc.poll() is not None for proc in children.values()):
                # fixture先结束是正常路径，给DDS可靠队列短时间排空。
                sleep(.3)
                break
            sleep(.1)
    finally:
        stop_children(children)
        for stream in logs.values():
            stream.close()
        (out / 'launcher.json').write_text(json.dumps(dict(session=args.session, launcher_pid=os.getpid(),
                   domain_id=os.environ['ROS_DOMAIN_ID'], children={role: dict(pid=proc.pid, returncode=proc.returncode)
                   for role, proc in children.items()}), indent=2), encoding='utf-8')
    from .offline import analyze
    try:
        summary = analyze(out, args.geometry_lib)
    except Exception as exc:
        summary = dict(pass_result=False, status='MISSING_OR_INVALID_EVIDENCE',
                       analysis_error=dict(type=type(exc).__name__, detail=str(exc)))
        summary['pass'] = False
        (out / 'probe-summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps(dict(pass_result=summary['pass'], status=summary['status'], output_dir=str(out))), flush=True)
    return 0 if summary['pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
