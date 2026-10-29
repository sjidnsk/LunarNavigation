#!/usr/bin/env python3
"""有界 Linux/Humble 验证入口；只使用标准输入/Path 与本次确切子进程。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

# 直接脚本与spawn重执行都建立明确包命名空间；不把源码父目录加入PYTHONPATH。
if not __package__:
    import importlib.util
    _package_name = 'nav1cm_p3_path_validation'
    if _package_name not in sys.modules:
        _package_spec = importlib.util.spec_from_file_location(
            _package_name, Path(__file__).with_name('__init__.py'),
            submodule_search_locations=[str(Path(__file__).resolve().parent)])
        _package_module = importlib.util.module_from_spec(_package_spec)
        sys.modules[_package_name] = _package_module
        _package_spec.loader.exec_module(_package_module)
    __package__ = _package_name

from .common import CASES, DURATIONS


def group_pids(group):
    found = []
    for folder in Path('/proc').iterdir():
        if folder.name.isdigit():
            try:
                if os.getpgid(int(folder.name))==group:
                    found.append(int(folder.name))
            except (ProcessLookupError, PermissionError):
                pass
    return found


def supervise(args):
    from .evaluate import analyze, apply_supervision_gate
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert os.environ.get('ROS_DOMAIN_ID'), '显式选择隔离域'
    assert os.environ.get('ROS_DISTRO')=='humble'
    script = Path(__file__).resolve()
    manifest = {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in script.parent.glob('*.py')}
    (output/'harness-source-sha256.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    all_results = []
    for case in args.cases:
        folder = output/case
        folder.mkdir()
        start = time.monotonic()+1.
        duration = DURATIONS[case]
        stop_at = start + (20. if case=='forward_spin' else 17. if case=='reverse' else duration)
        processes, files, results = [], [], []
        stop_s = supervisor_error = None
        print(json.dumps(dict(event='CASE_START',case=case,runtime_limit_s=duration+8)),flush=True)
        try:
            for role in ('consumer','planner','fixture'):
                if case=='empty_smoke' and role=='planner':
                    continue
                stream = (folder/(role+'.console.log')).open('x',encoding='utf-8')
                files.append(stream)
                command = [sys.executable,str(script),'--role',role,'--case',case,'--output',str(folder),
                           '--start',str(start),'--duration',str(duration),'--build-dir',args.build_dir]
                proc = subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True,cwd=output)
                processes.append((role,proc))
            while time.monotonic()<start+duration:
                now=time.monotonic()
                for role,proc in processes:
                    if role=='planner' and stop_s is None and now>=stop_at and proc.poll() is None:
                        stop_s=now
                        proc.send_signal(signal.SIGINT)
                    if proc.poll() is not None and (role!='planner' or stop_s is None):
                        # 提前退出保留 console/raw；其他本任务进程在 finally 中正常回收。
                        raise RuntimeError(f'{case}/{role} premature exit {proc.returncode}')
                time.sleep(.05)
        except Exception as exc:
            supervisor_error = repr(exc)
            (folder/'supervisor-error.txt').write_text(supervisor_error,encoding='utf-8')
        finally:
            for role,proc in processes:
                escalated=False
                if role!='planner' and proc.poll() is None:
                    # 两个输入/消费进程自身有期限；先让 finally 自然完成，避免
                    # 在 Python 解释器收尾阶段额外 SIGINT 造成 -2 假失败。
                    try:
                        proc.wait(timeout=3.)
                    except subprocess.TimeoutExpired:
                        pass
                if proc.poll() is None:
                    if role=='planner' and stop_s is None:
                        stop_s=time.monotonic()
                    proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    escalated=True
                    os.killpg(proc.pid,signal.SIGTERM)
                    try:
                        proc.wait(timeout=2.)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid,signal.SIGKILL)
                        proc.wait(timeout=2.)
                residual=group_pids(proc.pid)
                results.append(dict(role=role,pid=proc.pid,pgid=proc.pid,returncode=proc.returncode,
                                    escalated=escalated,remaining_group_pids=residual))
                # 只回收本 supervisor 创建的确切进程组，残留仍使验收失败。
                if residual:
                    os.killpg(proc.pid,signal.SIGTERM)
            for stream in files:
                stream.close()
        try:
            summary=analyze(folder,case,start,results,stop_s)
        except Exception as exc:
            summary=dict(case=case,pass_=False,analysis_error=repr(exc),processes=results)
        apply_supervision_gate(summary, folder, error=supervisor_error)
        summary.update(start_monotonic_s=start,planner_sigint_monotonic_s=stop_s)
        (folder/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
        all_results.append(summary)
        print(json.dumps(dict(event='CASE_END',**summary)),flush=True)
    summary=dict(pass_=all(r['pass_'] for r in all_results),cases=all_results)
    (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    return 0 if summary['pass_'] else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role',choices=('supervisor','fixture','planner','consumer'),default='supervisor')
    parser.add_argument('--cases',nargs='+',choices=CASES,default=list(CASES))
    parser.add_argument('--case',choices=CASES)
    parser.add_argument('--output',required=True)
    parser.add_argument('--build-dir',required=True)
    parser.add_argument('--start',type=float,default=0.)
    parser.add_argument('--duration',type=float,default=0.)
    args=parser.parse_args()
    if args.role=='supervisor':
        return supervise(args)
    if args.role=='planner':
        from .planner_probe import run
    elif args.role=='fixture':
        from .fixture import run
    else:
        from .consumer import run
    run(args)
    return 0


if __name__=='__main__':
    sys.exit(main())
