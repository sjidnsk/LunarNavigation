"""Owned, bounded course-two training. Monitoring never owns learner lifetime."""
import argparse
from contextlib import contextmanager
from dataclasses import replace
from fractions import Fraction
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from .course_two_scene import STAGES, RANGES, FOVS, make_case

DEFAULT_ROOT=Path('/home/kai/CodexDownloads/lunar_navigation/drl-course-two-20261010/managed')
DEFAULT_SOURCE=Path('/home/kai/CodexDownloads/lunar_navigation/drl-a2-ordered-continuation-20261007/managed/run/resume.pt')
DIAGNOSTICS=Path('/home/kai/WS/lunar-navigation/analysis-artifacts/paper1-completed-course-20261010/scenes.py')
EVALUATION_PROTOCOL='course_two_post_initialization_v2'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def write(path,value):
    from .artifacts import atomic_json
    atomic_json(Path(path),value,max_bytes=16*1024**2)


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for data in iter(lambda:stream.read(1024**2),b''):h.update(data)
    return h.hexdigest()


def runtime_digest():
    import importlib.util
    files=[(p.name,sha(p)) for p in sorted(Path(__file__).parent.glob('*.py'))]
    spec=importlib.util.find_spec('lunar_drl_terrain_native')
    if spec and spec.origin:files.append(('terrain_native',sha(spec.origin)))
    return hashlib.sha256(json.dumps(files).encode()).hexdigest()


def sensor_stage_ready(root,stage):
    path=Path(root)/'sensor-readiness.json'
    if not path.exists():return False
    record=read(path)
    return record.get('runtime')==runtime_digest() and stage in record.get('ready_stages',[])


def remaining_budget(target,current):
    if type(target) is not int or type(current) is not int or not 0<=current<=target:
        raise ValueError('checkpoint exceeds the frozen training boundary')
    return target-current


@contextmanager
def root_lock(root):
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    with (root/'controller.lock').open('a+') as stream:
        try:fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as exc:raise RuntimeError('course controller already running') from exc
        try:yield
        finally:fcntl.flock(stream,fcntl.LOCK_UN)


def process_ticks(pid):
    try:
        fields=Path(f'/proc/{pid}/stat').read_text().rsplit(') ',1)[1].split()
        return None if fields[0]=='Z' else fields[19]
    except (FileNotFoundError,ProcessLookupError):return None


def owned_process(root):
    path=Path(root)/'owner.json'
    if not path.exists():return None
    record=read(path);pid=record.get('pid');ticks=process_ticks(pid)
    if ticks is None:return None
    if ticks!=record.get('start_ticks') or record.get('root')!=str(Path(root).resolve()):
        raise RuntimeError('controller process identity mismatch; no signal sent')
    command=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
    if b'lunar_drl_exploration.course_two_control' not in command or b'run' not in command:
        raise RuntimeError('controller command identity mismatch; no signal sent')
    return record


def request_stop(root):
    owner=owned_process(root)
    if owner is None:return dict(status='not_running')
    os.kill(owner['pid'],signal.SIGINT)
    return dict(status='save_and_stop_requested',pid=owner['pid'])


def validation_cases(stage):
    index=STAGES.index(stage);base=2026101000+index*10000
    pairs=[(10.,90.)] if stage in ('A1','A2') else [(r,90.) for r in RANGES] if stage=='B' else [(r,f) for r in RANGES for f in FOVS]
    per_pair=80 if stage in ('A1','A2') else 24 if stage=='B' else 8
    cases=[]
    for pair_index,(r,fov) in enumerate(pairs):
        for j in range(per_pair):
            # Approach begins after the ordinary native initialization scan.
            # FOV-only blindness can disappear during that scan, so it is tested
            # separately. A1 crosses every bearing with both terrain families.
            category='O' if stage!='A1' and (j//8)%3==2 else 'U'
            ordinal=pair_index*per_pair+j
            case=make_case(base+ordinal,stage,ordinal,range_m=r,fov_deg=fov,
                visibility=category,family='moon' if (j//8+pair_index)%2==0 else 'cave')
            cases.append(dict(case_id=f'{stage}-new-{ordinal:03}',kind='new',case=case))
        if fov<360:
            for j in range(16):
                ordinal=2000+pair_index*16+j
                cases.append(dict(case_id=f'{stage}-fov-{pair_index:02}-{j:02}',kind='visible',
                    case=make_case(base+ordinal,stage,ordinal,range_m=r,fov_deg=fov,visibility='F',
                        family='moon' if j//8==0 else 'cave')))
        for j in range(2):
            ordinal=1000+pair_index*2+j
            cases.append(dict(case_id=f'{stage}-visible-{pair_index:02}-{j}',kind='visible',
                case=make_case(base+ordinal,stage,ordinal,range_m=r,fov_deg=fov,visibility='V',
                    family='moon' if j==0 else 'cave')))
    from .ordered_curriculum import TASKS,area_budget
    extents=(50.,50.+2*50./9.,50.+4*50./9.,50.+7*50./9.,100.)
    for task in TASKS:
        family,start=task.split('_')
        for j,extent in enumerate(extents):
            cases.append(dict(case_id=f'old-{task}-{j}',kind='old',spec=dict(seed=2026100410 if j==3 else 2026100400+j,
                family=family,extent=extent,start_mode=start,curriculum_task=task,episode_budget=area_budget(extent))))
    cases.extend(dict(case_id=f'D{i}',kind='diagnostic',scene=f'D{i}',budget=32) for i in range(1,9))
    return cases


def promotion_gate(rows,baseline,*,stage):
    expected={c['case_id']:c for c in validation_cases(stage)}
    if len(rows)!=len(expected) or {r['case_id'] for r in rows}!=set(expected):
        raise ValueError('incomplete course-two evaluation')
    if any(r['kind']!=expected[r['case_id']]['kind'] for r in rows):
        raise ValueError('course-two evaluation group identity changed')
    previous={r['case_id']:r for r in baseline}
    retained=[r for r in rows if r['kind'] in ('old','diagnostic')]
    if any(r['case_id'] not in previous for r in retained):raise ValueError('incomplete retained baseline')
    current=[r for r in rows if r['kind']=='new']
    def success(r):return bool(r['completed']) and not r['collisions'] and not r.get('error')
    def approached(r):return bool(r.get('first_roi')) and not r.get('initial_roi_visible') and not r.get('error')
    checks={'complete_records':not any(r.get('error') for r in rows+baseline),
        'no_collisions':not any(r['collisions'] for r in rows),
        'initial_visibility':not any(r.get('initial_roi_visible') for r in current),
        'approach':sum(map(approached,current))/len(current)>=.9,
        'coverage':sum(map(success,current))/len(current)>=.8}
    for axis in ('bearing_bin','range_m','fov_deg'):
        groups={r[axis] for r in current}
        for group in groups:
            subset=[r for r in current if r[axis]==group]
            checks[f'approach_{axis}_{group}']=sum(map(approached,subset))/len(subset)>=.8
    for pair in {(r['range_m'],r['fov_deg']) for r in current}:
        subset=[r for r in current if (r['range_m'],r['fov_deg'])==pair]
        checks[f'pair_{pair}']=sum(map(success,subset))/len(subset)>=.75 and sum(map(approached,subset))/len(subset)>=.75
    old=[r for r in retained if r['kind']=='old']
    checks['old_completion']=sum(map(success,old))>=sum(success(previous[r['case_id']]) for r in old)-1
    diagnostics=[r for r in retained if r['kind']=='diagnostic']
    checks['diagnostic_retention']=all(success(r) for r in diagnostics if success(previous[r['case_id']]))
    import statistics
    ratios=[r['decisions']/max(1,previous[r['case_id']]['decisions']) for r in old
        if success(r) and success(previous[r['case_id']])]
    checks['old_decision_cost']=bool(ratios) and statistics.median(ratios)<=1.2
    return dict(passed=all(checks.values()),checks=checks,
        approach_rate=sum(map(approached,current))/len(current),completion_rate=sum(map(success,current))/len(current))


def run_dir(root):return Path(root)/'training/run'


def range_cases(anchor):
    return [dict(case_id=f'B-easy-{anchor:g}-{i:02}',kind='new',case=make_case(
        2026600000+int(anchor)*100+i,'B',i,range_m=anchor,fov_deg=90.,easy=True,
        visibility='U',family='moon' if i%2==0 else 'cave')) for i in range(24)]


def range_gate(rows,anchor):
    expected={c['case_id'] for c in range_cases(anchor)}
    if len(rows)!=len(expected) or {r['case_id'] for r in rows}!=expected:
        raise ValueError('incomplete range anchor check')
    valid=not any(r.get('error') or r['collisions'] or r['initial_roi_visible'] for r in rows)
    approach=sum(bool(r['first_roi']) for r in rows)/len(rows)
    complete=sum(bool(r['completed']) for r in rows)/len(rows)
    return dict(passed=valid and approach>=.9 and complete>=.8,anchor=anchor,
        approach_rate=approach,completion_rate=complete)


def schedule(root):
    import torch
    from .checkpoint import TrainingState
    state=TrainingState(torch.load(run_dir(root)/'resume.pt',map_location='cpu',weights_only=False))
    state.validate()
    return dict(state.record['schedule'])


def prepare(root, source):
    """A new directory and full-state fork; never writes the supplied checkpoint."""
    import shutil
    import torch
    from .config import training_config_from_record,config_record
    from .checkpoint import TrainingState,CheckpointManager
    from .evaluation import export_actor
    root=Path(root).resolve();source=Path(source).resolve()
    if root.exists() and any(root.iterdir()):raise ValueError('prepare requires a new empty experiment directory')
    record=torch.load(source,map_location='cpu',weights_only=False)
    original=training_config_from_record(record['config'])
    config=replace(original,ordered_curriculum=False,course_two=True,
        save_interval_s=300.,snapshot_max_bytes=512*1024**2,total_output_max_bytes=12*1024**3,
        metrics_max_bytes=16*1024**2,owned_pss_limit_bytes=20*1024**3,system_reserve_bytes=4*1024**3,
        artifact_root=str(root/'training'),output_dir=str(run_dir(root)))
    state=TrainingState.fork_from(source,config,experiment='course-two')
    from .training import state_checksum
    preserved={key:state_checksum(record[key]) for key in ('learner','replay','schedule','rng','collector_state')}
    if any(state_checksum(state.record[key])!=value for key,value in preserved.items()):
        raise ValueError('full learned state changed during course-two fork')
    source_sha=sha(source)
    root.mkdir(parents=True,exist_ok=True);(root/'frozen').mkdir();(root/'logs').mkdir();(root/'evaluation').mkdir()
    shutil.copyfile(source,root/'frozen/source-resume.pt')
    shutil.copyfile(DIAGNOSTICS,root/'frozen/diagnostic_scenes.py')
    if sha(root/'frozen/source-resume.pt')!=source_sha:raise ValueError('source backup differs')
    manager=CheckpointManager(config.output_dir,config.save_interval_s,
        total_max_bytes=config.total_output_max_bytes,snapshot_max_bytes=config.snapshot_max_bytes,
        artifact_root=config.artifact_root)
    manager.save(state)
    export_actor(source,root/'frozen/block31.pt')
    write(root/'config.json',config_record(config))
    write(root/'identity.json',dict(schema='course_two_control_v1',root=str(root),source=str(source),
        source_sha=source_sha,runtime=runtime_digest(),diagnostic_sha=sha(DIAGNOSTICS),
        baseline_transitions=record['schedule']['transitions'],baseline_updates=record['schedule']['updates'],
        preserved_state_sha256=preserved,created_unix=time.time()))
    write(root/'state.json',dict(status='ready',stage='A1',phase='training',stage_block=0,
        streak=0,pilot_checked=False,stage_start=record['schedule']['transitions'],
        target_transitions=record['schedule']['transitions']+2048,history=[],updated_unix=time.time()))
    for stage in STAGES:write(root/f'cases-{stage}.json',validation_cases(stage))
    for anchor in RANGES[1:]:write(root/f'cases-B-easy-{anchor:g}.json',range_cases(anchor))
    return dict(root=str(root),checkpoint=str(run_dir(root)/'resume.pt'),status='ready',source_sha=source_sha)


def start_background(root):
    root=Path(root).resolve()
    owner=owned_process(root)
    if owner:return dict(status='already_running',pid=owner['pid'])
    state=read(root/'state.json')
    if state['status'] not in ('ready','paused'):raise ValueError('run is '+state['status']+'; automatic restart is disabled')
    if read(root/'identity.json')['runtime']!=runtime_digest():raise ValueError('runtime changed since experiment preparation')
    with root_lock(root):pass
    with (root/'logs/controller.log').open('ab') as stream:
        process=subprocess.Popen([sys.executable,'-m','lunar_drl_exploration.course_two_control',
            'run','--root',str(root)],stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,
            start_new_session=True)
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:
        if process.poll() is not None:raise RuntimeError('controller startup failed; see logs/controller.log')
        path=root/'owner.json'
        if path.exists() and read(path).get('pid')==process.pid:
            return dict(status='running',pid=process.pid,root=str(root))
        time.sleep(.1)
    raise RuntimeError('controller startup acknowledgement timed out')


class Controller:
    def __init__(self,root):
        self.root=Path(root).resolve();self.state=read(self.root/'state.json')
        self.identity=read(self.root/'identity.json');self.stopping=False;self.child=None
        self.evaluation_children={};self.evaluation_signalled=set()

    def save(self,**fields):
        self.state.update(fields,updated_unix=time.time())
        write(self.root/'state.json',self.state)

    def interrupt(self,*_):
        if self.stopping:return
        self.stopping=True
        if self.child is not None and self.child.poll() is None:
            self.child.send_signal(signal.SIGINT)
        from .course_two_parallel import interrupt_children
        interrupt_children(self)

    def spawn(self,args,label,**env):
        if self.stopping:return False
        with (self.root/'logs'/f'{label}.log').open('ab') as stream:
            self.child=subprocess.Popen(args,stdout=stream,stderr=subprocess.STDOUT,
                env=dict(os.environ,**env))
            if self.stopping:self.child.send_signal(signal.SIGINT)
            self.save(child_pid=self.child.pid,child_label=label)
            while self.child.poll() is None:time.sleep(.2)
            code=self.child.returncode
        self.child=None
        self.last_child_code=code
        if self.stopping:
            if label.startswith('train-') and code!=0:
                raise RuntimeError('training stop failed; last good checkpoint preserved; see '+label+'.log')
            return False
        if code:raise RuntimeError(f'{label} exited {code}; see logs/{label}.log')
        return True

    def train(self):
        current=schedule(self.root)
        remaining=remaining_budget(self.state['target_transitions'],current['transitions'])
        credit=Fraction(current['credit'])
        if remaining or credit>=1:
            self.save(phase='training',status='running')
            args=[sys.executable,'-m','lunar_drl_exploration.cli','train','--config',str(self.root/'config.json'),
                '--resume','--max-transitions',str(remaining),'--domain-base','180']
            if not self.spawn(args,f"train-{self.state['stage']}-{self.state['target_transitions']}",
                DRL_COURSE_TWO_STAGE=self.state['stage'],
                DRL_COURSE_TWO_RANGE_LIMIT=str(self.state.get('b_range_limit',100)),
                DRL_COURSE_TWO_RANGE_EASY='1' if self.state.get('b_easy') else '0'):return False
            current=schedule(self.root)
        if current['transitions']!=self.state['target_transitions'] or Fraction(current['credit'])>=1:
            raise RuntimeError('trainer did not reach the exact saved boundary')
        return True

    def screen(self,actor,cases,label):
        from .course_two_parallel import screen
        return screen(self,actor,cases,label)

    def reevaluate(self):
        """Reassess frozen boundaries under one repaired protocol, without training."""
        plan=self.state['reevaluation'];stage=self.state['stage']
        cases=read(self.root/self.state['evaluation_case_file'])
        if cases!=validation_cases(stage):raise ValueError('reassessment frozen case identity changed')
        baseline=None
        for index,label in enumerate(plan['labels']):
            if self.stopping:return
            actor=self.root/'frozen/block31.pt' if label=='baseline' else self.root/(label+'.pt')
            expected=plan.get('actors',{}).get(label)
            if expected and sha(actor)!=expected:raise ValueError('reassessment actor identity changed')
            # Completed cases are verified and reused only within this new protocol.
            rows=self.screen(actor,cases,label)
            if rows is None:return
            if label=='baseline':baseline=rows
            else:
                gate=promotion_gate(rows,baseline,stage=stage)
                block=int(label.rsplit('block',1)[1])
                history=[r for r in self.state['history'] if (r['stage'],r['block'])!=(stage,block)]
                history.append(dict(stage=stage,block=block,actor_sha=sha(actor),
                    evaluation_protocol=EVALUATION_PROTOCOL,**gate))
                self.save(history=sorted(history,key=lambda r:(r['stage'],r['block'])))
            plan=dict(plan,next=index+1)
            self.save(reevaluation=plan)
        write(self.root/self.state['evaluation_directory']/'comparison.json',dict(status='finished',
            evaluation_protocol=EVALUATION_PROTOCOL,runtime=runtime_digest(),
            cases_sha=sha(self.root/self.state['evaluation_case_file']),
            baseline_actor_sha=sha(self.root/'frozen/block31.pt'),history=self.state['history'],
            note='Frozen-model reassessment only; no training or automatic promotion.'))
        self.save(status='needs_attention',phase='reevaluation_complete',streak=0)

    def run(self):
        from .evaluation import export_actor
        if self.identity['runtime']!=runtime_digest():raise ValueError('runtime changed since preparation')
        with root_lock(self.root):
            write(self.root/'owner.json',dict(pid=os.getpid(),start_ticks=process_ticks(os.getpid()),root=str(self.root)))
            for s in (signal.SIGINT,signal.SIGTERM):signal.signal(s,self.interrupt)
            self.save(status='running',controller_pid=os.getpid())
            try:
                if self.state.get('reevaluation'):
                    if not sensor_stage_ready(self.root,self.state['stage']):
                        self.save(status='needs_attention',phase='sensor_preflight_required');return
                    self.reevaluate();return
                while not self.stopping:
                    if not sensor_stage_ready(self.root,self.state['stage']):
                        self.save(status='needs_attention',phase='sensor_preflight_required');return
                    if not self.train():break
                    if not self.state['pilot_checked']:
                        stats=read(run_dir(self.root)/'course-summary.json')['by_task']['c2_A1']
                        if stats['first_roi']==0:
                            self.save(pilot_checked=True,pilot_stats=stats,status='needs_attention',phase='pilot_no_region_gain');return
                        self.save(pilot_checked=True,pilot_stats=stats,target_transitions=self.state['stage_start']+8192)
                        continue
                    if self.state['stage']=='B' and self.state.get('b_easy'):
                        anchor=self.state['b_range_limit'];boundary=self.state['target_transitions']
                        label=f'B-easy-{anchor:g}-{boundary}'
                        actor=self.root/(label+'.pt')
                        if not actor.exists():export_actor(run_dir(self.root)/'resume.pt',actor)
                        rows=self.screen(actor,read(self.root/f'cases-B-easy-{anchor:g}.json'),label)
                        if rows is None:break
                        gate=range_gate(rows,anchor)
                        fields=dict(range_history=self.state.get('range_history',[])+[dict(boundary=boundary,**gate)])
                        if gate['passed'] and anchor==100:
                            fields.update(b_easy=False)
                            if boundary%8192!=self.state['stage_start']%8192:
                                fields['target_transitions']=min(self.state['stage_start']+32768,
                                    self.state['stage_start']+((boundary-self.state['stage_start'])//8192+1)*8192)
                            self.save(**fields);continue
                        if boundary>=self.state['stage_start']+32768:
                            self.save(**fields,status='needs_attention',phase='range_budget_exhausted');return
                        if gate['passed']:fields['b_range_limit']=RANGES[RANGES.index(anchor)+1]
                        fields['target_transitions']=min(boundary+2048,self.state['stage_start']+32768)
                        self.save(**fields);continue
                    stage=self.state['stage'];block=self.state['stage_block']+1
                    label=f'{stage}-block{block}';actor=self.root/(label+'.pt')
                    # Re-export deterministically from this boundary, even after an
                    # evaluation pause. Cached cases verify the Actor content hash.
                    if not actor.exists():
                        export_actor(run_dir(self.root)/'resume.pt',actor)
                        write(actor.with_suffix('.json'),dict(checkpoint_sha=sha(run_dir(self.root)/'resume.pt'),actor_sha=sha(actor)))
                    else:
                        proof=read(actor.with_suffix('.json'))
                        if proof!=dict(checkpoint_sha=sha(run_dir(self.root)/'resume.pt'),actor_sha=sha(actor)):
                            raise ValueError('evaluation Actor does not match training boundary')
                    cases=read(self.root/f'cases-{stage}.json')
                    baseline=self.screen(self.root/'frozen/block31.pt',
                        [c for c in cases if c['kind'] in ('old','diagnostic')],'baseline')
                    if baseline is None:break
                    rows=self.screen(actor,cases,label)
                    if rows is None:break
                    gate=promotion_gate(rows,baseline,stage=stage)
                    history=self.state['history']+[dict(stage=stage,block=block,actor_sha=sha(actor),**gate)]
                    streak=self.state['streak']+1 if gate['passed'] else 0
                    advance=dict(history=history,stage_block=block,streak=streak)
                    if streak>=2:
                        index=STAGES.index(stage)
                        if index==len(STAGES)-1:
                            self.save(**advance,status='finished',phase='course_complete');return
                        boundary=self.state['target_transitions']
                        advance.update(stage=STAGES[index+1],stage_block=0,streak=0,
                            stage_start=boundary,target_transitions=boundary+8192)
                        if advance['stage']=='B':advance.update(b_easy=True,b_range_limit=20.,target_transitions=boundary+2048)
                        self.save(**advance)
                    elif self.state['target_transitions']>=self.state['stage_start']+32768:
                        self.save(**advance,status='needs_attention',phase='stage_budget_exhausted');return
                    else:self.save(**advance,target_transitions=min(self.state['target_transitions']+8192,self.state['stage_start']+32768))
            except BaseException as exc:
                self.save(status='failed',phase='error',error=f'{type(exc).__name__}: {exc}')
                raise
            finally:
                if self.stopping and self.state['status']!='failed':
                    self.save(status='paused',phase='saved_exit',saved_schedule=schedule(self.root))
                self.save(child_pid=None)


def status(root):
    root=Path(root).resolve()
    result=read(root/'state.json')
    owner=owned_process(root)
    result['live_pid']=None if owner is None else owner['pid']
    if owner is None and result['status']=='running':result['status']='interrupted_unverified'
    for name,path in [('progress',run_dir(root)/'progress.json'),('evaluation',root/'eval-progress.json')]:
        if path.exists():result[name]=dict(read(path),file_age_s=time.time()-path.stat().st_mtime)
    checkpoint=run_dir(root)/'resume.pt'
    result['checkpoint']=dict(path=str(checkpoint),age_s=time.time()-checkpoint.stat().st_mtime)
    return result


def monitor(root):
    import curses
    from .terminal import format_status
    identity=read(Path(root)/'identity.json');config=read(Path(root)/'config.json')
    def display(screen):
        screen.timeout(1000);message=''
        try:curses.curs_set(0)
        except curses.error:pass
        while True:
            s=status(root);p=s.get('progress');lines=[
                '课程二训练监看  |  Q / Ctrl+C: 关闭监看  P: 保存停止  R: 恢复训练',
                f"阶段 {s['stage']} | {s['status']} / {s['phase']} | 控制器 PID {s['live_pid'] or '—'}",
                f"本段上限 {s['target_transitions']-s['stage_start']:,} 条 | 已完成评测块 {s['stage_block']} | 连续通过 {s['streak']}/2",
                f"完整检查点距今 {s['checkpoint']['age_s']:.0f}s | {s['checkpoint']['path']}"]
            if p:
                lines.append(f"课程二新增 {p['transitions']-identity['baseline_transitions']:,} 条 / {p['updates']-identity['baseline_updates']:,} 次更新 | 指标距今 {p['file_age_s']:.0f}s")
                lines+=format_status(p,environments=config['environments'],warmup=config['warmup'],phase=s['phase'])
                for task,counts in p.get('course',{}).get('by_task',{}).items():
                    if counts['transitions']:
                        total=counts['ended']-counts['zero_decisions'];rate=f"{counts['successes']/total:.1%}" if total else '—'
                        lines.append(f"{task}: 转移 {counts['transitions']} | 策略成功/结束 {counts['successes']}/{total} ({rate}) | 初始已完成 {counts['initial_completed']} | 接近 {counts['first_roi']}")
                for env,info in p.get('environments',{}).items():
                    sensor=info.get('sensor',{})
                    if sensor:lines.append(f"E{env}: 固定传感器 {sensor['range_m']:g}m / {sensor['fov_deg']:g}° | {info.get('scenario_id') or '原课程复习'}")
            if s['phase']=='evaluation':lines.append(str(s.get('evaluation')))
            if s.get('error'):lines.append('错误: '+s['error'])
            lines.append(message)
            screen.erase();height,width=screen.getmaxyx()
            for i,line in enumerate(lines[:height-1]):
                try:screen.addnstr(i,0,line,max(0,width-1))
                except curses.error:pass
            screen.refresh();key=screen.getch()
            if key in (ord('q'),ord('Q'),3):return
            try:
                if key in (ord('p'),ord('P')):message=str(request_stop(root))
                elif key in (ord('r'),ord('R')):message=str(start_background(root))
            except Exception as exc:message=str(exc)
    try:curses.wrapper(display)
    except KeyboardInterrupt:pass


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('prepare','start','resume','run','stop','status','monitor','evaluate-one'))
    parser.add_argument('--root',type=Path,default=DEFAULT_ROOT)
    parser.add_argument('--source',type=Path,default=DEFAULT_SOURCE)
    parser.add_argument('--request',type=Path);parser.add_argument('--output',type=Path)
    parser.add_argument('--slot',type=int,default=0);parser.add_argument('--progress',type=Path)
    parser.add_argument('--job-id',default=None)
    args=parser.parse_args();root=args.root.resolve()
    if args.command=='prepare':result=prepare(root,args.source)
    elif args.command in ('start','resume'):result=start_background(root)
    elif args.command=='run':return Controller(root).run()
    elif args.command=='stop':result=request_stop(root)
    elif args.command=='status':result=status(root)
    elif args.command=='monitor':return monitor(root)
    else:
        from .course_two_eval import evaluate_case
        if not args.request or not args.output:parser.error('evaluate-one requires --request and --output')
        result=evaluate_case(root,args.request,args.output,slot=args.slot,progress=args.progress,job_id=args.job_id)
        if result['status']!='finished':raise RuntimeError(result['row']['error'])
        result={key:value for key,value in result.items() if key!='row'}
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
