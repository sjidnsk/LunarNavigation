#!/usr/bin/env python3
"""真实安装短程联合验证；独立进程组监督，精确本任务清理。"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from evidence import DURATIONS,legacy

def supervise(args):
    from evaluate import analyze
    def members(group):
        found=[]
        for p in Path('/proc').iterdir():
            if p.name.isdigit():
                try:
                    if os.getpgid(int(p.name))==group:found.append(int(p.name))
                except (ProcessLookupError,PermissionError):pass
        return found
    assert os.environ.get('ROS_DOMAIN_ID')=='92'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    assert os.environ.get('ROS_DISTRO')=='humble'
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
    # 安装逐文件身份必须与已验证T2 receipt一致。
    receipt=json.loads(Path(args.installed_identity).read_text())
    for row in receipt['files']:
        assert hashlib.sha256(Path(row['installed']).read_bytes()).hexdigest()==row['sha256'],row
    for row in receipt['native']:
        assert hashlib.sha256(Path(row['path']).read_bytes()).hexdigest()==row['sha256'],row
    (output/'installed-identity.json').write_text(json.dumps(receipt,indent=2))
    script=Path(__file__).resolve()
    (output/'harness-sha256.json').write_text(json.dumps({str(p.relative_to(script.parent.parent)):
        hashlib.sha256(p.read_bytes()).hexdigest() for root in (script.parent,script.parent.parent/'p3_path_validation')
        for p in root.glob('*.py')},indent=2))
    summaries=[]
    for case in args.cases:
        folder=output/case;folder.mkdir()
        start=time.monotonic()+2.;end=start+DURATIONS[case]
        procs=[];streams=[];results=[];error=None;stop=None;intent={}
        try:
            for role in ('follower','planner','plant'):
                if case=='empty_smoke' and role=='planner':continue
                log=(folder/(role+'.console.log')).open('x');streams.append(log)
                cmd=[sys.executable,str(script),'--role',role,'--case',case,'--start',str(start),
                     '--duration',str(DURATIONS[case]),'--output',str(folder),'--build-dir',args.build_dir]
                p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=folder)
                procs.append((role,p))
            while time.monotonic()<end:
                now=time.monotonic()
                for role,p in procs:
                    if p.poll() is not None and role not in intent:
                        raise RuntimeError(f'{role} premature exit {p.returncode}')
                    if role=='planner' and case=='forward_spin' and now>=end-3. and role not in intent:
                        intent[role]=now;stop=now;p.send_signal(signal.SIGINT)
                time.sleep(.03)
        except Exception as exc:
            error=repr(exc);(folder/'supervisor-error.txt').write_text(error)
        finally:
            for role,p in procs:
                premature=p.poll() is not None and role not in intent and time.monotonic()<end-.05
                if role=='plant' and p.poll() is None and time.monotonic()>=end:
                    try:p.wait(timeout=2.)
                    except subprocess.TimeoutExpired:pass
                if p.poll() is None:
                    intent[role]=time.monotonic();p.send_signal(signal.SIGINT)
                escalated=False
                try:p.wait(timeout=6.)
                except subprocess.TimeoutExpired:
                    escalated=True;os.killpg(p.pid,signal.SIGTERM)
                    try:p.wait(timeout=2.)
                    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=2.)
                remaining=members(p.pid)
                results.append(dict(role=role,pid=p.pid,returncode=p.returncode,premature=premature,
                    remaining=remaining,escalated=escalated,stop_requested_s=intent.get(role)))
                if remaining:os.killpg(p.pid,signal.SIGTERM)
            for s in streams:s.close()
        try:summary=analyze(folder,case,results,error,stop)
        except Exception as exc:summary=dict(pass_=False,analysis_error=repr(exc),processes=results,supervisor_error=error)
        (folder/'summary.json').write_text(json.dumps(summary,indent=2))
        print(json.dumps(summary),flush=True);summaries.append(summary)
    (output/'summary.json').write_text(json.dumps(dict(pass_=all(s['pass_'] for s in summaries),cases=summaries),indent=2))
    return 0 if all(s['pass_'] for s in summaries) else 1
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--role',default='supervisor',choices=['supervisor','plant','follower','planner'])
    p.add_argument('--case',choices=DURATIONS)
    p.add_argument('--cases',nargs='+',choices=DURATIONS,default=list(DURATIONS))
    p.add_argument('--output',required=True);p.add_argument('--build-dir',required=True)
    p.add_argument('--installed-identity')
    p.add_argument('--start',type=float,default=0.);p.add_argument('--duration',type=float,default=0.)
    args=p.parse_args()
    if args.role=='supervisor':return supervise(args)
    if args.role=='planner':from planner_probe import run
    elif args.role=='plant':from plant import run
    else:from follower import run
    run(args);return 0
if __name__=='__main__':sys.exit(main())
