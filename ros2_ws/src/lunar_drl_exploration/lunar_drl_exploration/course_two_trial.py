"""Recoverable qualification and bounded C3 training from the full baseline."""
from collections import Counter,deque
from dataclasses import replace
from fractions import Fraction
from pathlib import Path
import os
import shutil
import signal
import statistics
import subprocess
import sys
import time
from . import course_two_control as ctl

DEFAULT_ROOT=Path('/home/kai/CodexDownloads/lunar_navigation/drl-course-two-distance-sensor-10-30m-extended-20261012/managed')
OLD_ROOT=ctl.DEFAULT_ROOT
SOURCE_SHA='cf58f56b1aaf5411645123ffc76b027eaefa4146391cac6ef2e8ae1fc755f6bc'
ARMS=('C3',)


def qualification_jobs(training,development):
    return [('rebuilt',r,False) for r in training]+[('development',r,False) for r in development]


def cached_qualification(output,proof,identity):
    output=Path(output);result=ctl.read(output);actual=result.get('proof',{})
    if actual!=proof:
        # Explicit, reviewed resource-only migrations retain original proof bytes.
        adopted=identity.get('qualification_cache_adoptions',{}).get(output.name)
        required=dict(sha=ctl.sha(output),from_runtime=actual.get('runtime'),to_runtime=proof['runtime'])
        if adopted!=required or dict(actual,runtime=proof['runtime'])!=proof:
            raise ValueError('invalid qualification identity: '+output.name)
    if result.get('status')!='passed':raise ValueError('invalid qualification status: '+output.name)
    return result


def gate(rows,baseline):
    if any(r.get('kind')=='simple' for r in rows+baseline):
        from .course_two_simple import evaluation_gate
        return evaluation_gate(rows,baseline)
    def identity(items):return {(r['case_id'],r['kind'],r.get('capability'),r.get('promotion_required',True)) for r in items}
    if (len(rows)!=190 or len(baseline)!=190 or len(identity(rows))!=190 or identity(rows)!=identity(baseline)
            or Counter(r['kind'] for r in rows)!=dict(new=80,old=20,diagnostic=8,visible=18,capability=64)):
        raise ValueError('trial evaluation identity/count mismatch')
    success=lambda r:bool(r['completed']) and not r['collisions'] and not r.get('error')
    previous={r['case_id']:r for r in baseline}
    old=[r for r in rows if r['kind']=='old'];new=[r for r in rows if r['kind']=='new']
    ratios=[r['decisions']/max(1,previous[r['case_id']]['decisions']) for r in old
        if success(r) and success(previous[r['case_id']])]
    retained=dict(data=not any(r.get('error') for r in rows+baseline),
        collisions=not any(r['collisions'] for r in rows if r.get('promotion_required',True)),old_success=sum(map(success,old))>=19,
        old_decisions=bool(ratios) and statistics.median(ratios)<=1.2,
        diagnostics=all(success(r) for r in rows if r['kind']=='diagnostic' and r.get('promotion_required',True)
            and success(previous[r['case_id']])))
    development={}
    from .course_two_task_scope import CAPABILITIES
    capabilities={r.get('capability') for r in rows if r['kind']=='capability'}
    roles=CAPABILITIES if capabilities==set(CAPABILITIES) else ('cover','finish','return','detour')
    for role in roles:
        group=[r for r in rows if r.get('capability')==role]
        if len(group)!=16:raise ValueError('development identity/count mismatch')
        count=sum(map(success,group));before=sum(success(previous[r['case_id']]) for r in group)
        development[role]=dict(success=count,baseline=before,total=16)
        retained[role]=count>=before-1
    approached=lambda r:bool(r.get('first_roi')) and not r.get('initial_roi_visible') and not r.get('error')
    checks=dict(retained,approach=sum(map(approached,new))>=72,new_complete=sum(map(success,new))>=64,
        initial_visibility=not any(r.get('initial_roi_visible') for r in new),
        development=all(g['success']>=14 for g in development.values()))
    # Preserve the existing direction/sensor subgroup thresholds as well.
    for axis in ('bearing_bin','range_m','fov_deg'):
        for value in {r[axis] for r in new if axis in r}:
            group=[r for r in new if r.get(axis)==value]
            checks[f'{axis}:{value}']=sum(map(approached,group))/len(group)>=.8
    return dict(passed=all(checks.values()),stop=not all(retained.values()),checks=checks,
        development=development,old_success=sum(map(success,old)),new_complete=sum(map(success,new)),
        first_roi=sum(map(approached,new)),median_old_decision_ratio=statistics.median(ratios) if ratios else None,
        next_action='end_block_and_review')


def prepare(root):
    import torch
    from .checkpoint import TrainingState
    from .config import training_config_from_record,config_record
    source=OLD_ROOT/'frozen/source-resume.pt'
    if ctl.sha(source)!=SOURCE_SHA:raise ValueError('block31 full source changed')
    root=Path(root).resolve()
    if root.exists() and any(root.iterdir()):raise ValueError('prepare requires empty managed directory')
    record=torch.load(source,map_location='cpu',weights_only=False);TrainingState(record).validate()
    config=replace(training_config_from_record(record['config']),environments=12)
    for name in ('frozen','logs','qualification','arms'): (root/name).mkdir(parents=True,exist_ok=True)
    for name in ('source-resume.pt','block31.pt','diagnostic_scenes.py'):
        shutil.copyfile(OLD_ROOT/'frozen'/name,root/'frozen'/name)
    from .course_two_simple import development_cases,SAMPLING
    from .course_two_task_scope import SCOPE
    from .course_two_sensors import PROFILE,semantics
    for kind in ('training','development'):
        ctl.write(root/'frozen'/f'{kind}-requests.json',[])
    scene_config=replace(config,ordered_curriculum=False,course_two=True,course_two_variant='C3',
        course_two_task_scope=SCOPE,course_two_sampling=SAMPLING,course_two_scene_profile=PROFILE)
    cases=development_cases(scene_config)
    ctl.write(root/'frozen/cases-base.json',cases)
    ctl.write(root/'cases.json',cases)
    ctl.write(root/'config.json',config_record(config))
    for name in ('evaluation-settings.json','qualification-settings.json','training-settings.json'):
        ctl.write(root/name,dict(workers=12))
    identities={p.name:ctl.sha(p) for p in (root/'frozen').iterdir()}
    ctl.write(root/'identity.json',dict(schema='course_two_trial_v5',task_scope=SCOPE,
        sampling=SAMPLING,scene_profile=PROFILE,scene_settings=semantics(),runtime=ctl.runtime_digest(),
        source_sha=SOURCE_SHA,frozen=identities,diagnostic_sha=identities['diagnostic_scenes.py'],
        baseline_transitions=record['schedule']['transitions'],baseline_updates=record['schedule']['updates'],
        pilot_admissions=2048,pilot_updates=512,prefill_limit=4096,stage='A1',arms=list(ARMS)))
    total=len(qualification_jobs(ctl.read(root/'frozen/training-requests.json'),
        ctl.read(root/'frozen/development-requests.json')))
    ctl.write(root/'state.json',dict(status='ready',phase='baseline',stage='A1',arm=None,
        results={},qualification_done=0,qualification_total=total,created_unix=time.time()))
    return dict(root=str(root),status='ready',runtime=ctl.runtime_digest())


def owner(root):
    path=Path(root)/'owner.json'
    if not path.exists():return None
    value=ctl.read(path);pid=value['pid'];ticks=ctl.process_ticks(pid)
    if ticks is None:return None
    if (ticks!=value.get('start_ticks') or value.get('root')!=str(Path(root).resolve()) or
            b'lunar_drl_exploration.course_two_trial' not in Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')):
        raise ValueError('trial owner identity mismatch')
    return value


def start(root):
    root=Path(root).resolve();live=owner(root)
    if live:return dict(status='already_running',pid=live['pid'])
    state=ctl.read(root/'state.json')
    if state['status'] not in ('ready','paused'):raise ValueError('trial requires attention: '+state['status'])
    if ctl.read(root/'identity.json')['runtime']!=ctl.runtime_digest():raise ValueError('trial runtime changed')
    with ctl.root_lock(root):pass
    with (root/'logs/controller.log').open('ab') as stream:
        child=subprocess.Popen([sys.executable,'-m','lunar_drl_exploration.course_two_trial','run','--root',str(root)],
            stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:
        if child.poll() is not None:raise RuntimeError('trial startup failed; see controller.log')
        live=owner(root)
        if live and live['pid']==child.pid:return dict(status='running',pid=child.pid,root=str(root))
        time.sleep(.1)
    raise RuntimeError('trial startup acknowledgement timed out')


def stop(root):
    live=owner(root)
    if live:os.kill(live['pid'],signal.SIGINT)
    return dict(status='save_and_stop_requested' if live else 'not_running')


def checkpoint(path):
    import torch
    from .checkpoint import TrainingState
    state=TrainingState(torch.load(path,map_location='cpu',weights_only=False));state.validate()
    return state


class Trial(ctl.Controller):
    def spawn(self,args,label,**env):
        from .processes import _LAUNCH
        return super().spawn([sys.executable,'-c',_LAUNCH,str(os.getpid()),*args],label,**env)

    def qualify_all(self):
        if self.identity.get('sampling')=='scene_episode_v1':
            cases=ctl.read(self.root/'frozen/cases-base.json')
            if ctl.read(self.root/'cases.json')!=cases:raise ValueError('three-scene development cases changed')
            self.save(phase='baseline',qualification_workers=[])
            return True
        from .processes import _LAUNCH
        from .course_two_parallel import worker_count
        workers=worker_count(self.root/'qualification-settings.json')
        train=ctl.read(self.root/'frozen/training-requests.json');dev=ctl.read(self.root/'frozen/development-requests.json')
        jobs=qualification_jobs(train,dev)
        pending=deque();running={};results={}
        for label,request,original in jobs:
            key=label+'-'+request['request_id'];out=self.root/'qualification'/(key+'.json')
            proof=dict(runtime=self.identity['runtime'],config_sha=ctl.sha(self.root/'config.json'),request=request,original=original)
            if out.exists():
                results[key]=cached_qualification(out,proof,self.identity)
            else:pending.append((label,request,original,key,out))
        def publish():
            self.save(phase='qualification',qualification_done=len(results),qualification_total=len(jobs),
                qualification_concurrency=workers,
                qualification_workers=[dict(slot=slot,pid=j[0].pid,role=j[1]['role'],
                    output=str(j[3]),progress=str(j[3])+'.progress.json') for slot,j in running.items()])
        try:
            publish()
            while pending or running:
                if self.stopping:return False
                for slot in range(workers):
                    if slot in running or not pending:continue
                    label,request,original,key,out=pending.popleft()
                    req=self.root/'qualification'/(key+'.request.json');ctl.write(req,request)
                    args=[sys.executable,'-m','lunar_drl_exploration.course_two_qualification',
                        '--config',str(self.root/'config.json'),'--request',str(req),'--output',str(out),'--slot',str(slot)]
                    if original:args.append('--original')
                    with (self.root/'logs'/(key+'.log')).open('ab') as stream:
                        process=subprocess.Popen([sys.executable,'-c',_LAUNCH,str(os.getpid()),*args],
                            stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True,
                            env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1'))
                    running[slot]=(process,request,key,out);self.evaluation_children[slot]=process
                for slot,(process,request,key,out) in list(running.items()):
                    code=process.poll()
                    if code is None:continue
                    if self.stopping:return False
                    if code:raise RuntimeError('qualification failed: '+key+'; see logs')
                    result=ctl.read(out)
                    if result['status']!='passed':raise ValueError('qualification rejected '+key)
                    results[key]=result;del running[slot];del self.evaluation_children[slot]
                publish();time.sleep(.3)
        finally:
            from .course_two_parallel import interrupt_children
            interrupt_children(self)
            for process,_,_,_ in running.values():
                try:process.wait(timeout=30)
                except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
            self.evaluation_children.clear();self.evaluation_signalled.clear()
        selected=[results['rebuilt-'+r['request_id']]['spec'] for r in train]
        ctl.write(self.root/'rebuilt-catalog.json',dict(schema='course_two_qualified_catalog_v2',
            status='passed',runtime=self.identity['runtime'],specs=selected,templates=train))
        cases=ctl.read(self.root/'frozen'/('cases-base.json' if self.identity.get('task_scope') else 'cases-v2.json'))
        for r in dev:
            result=results['development-'+r['request_id']];spec=result['spec']
            cases.append(dict(case_id='dev-'+r['request_id'],kind='capability',
                capability=r.get('capability',spec['course_two_case']['role']),spec=spec,
                qualified_handover_hash=result['evidence']['metadata']['prelude']['handover_hash']))
        ctl.write(self.root/'cases.json',cases)
        self.save(phase='baseline',qualification_workers=[])
        return True

    def arm_config(self,arm):
        from .config import training_config_from_record
        from .course_two_parallel import worker_count
        if arm!='C3':raise ValueError('only C3 training is enabled')
        catalog=self.root/'rebuilt-catalog.json'
        simple=self.identity.get('sampling')=='scene_episode_v1'
        return replace(training_config_from_record(ctl.read(self.root/'config.json')),
            environments=worker_count(self.root/'training-settings.json',default=12),
            ordered_curriculum=False,course_two=True,course_two_variant=arm,
            course_two_task_scope=self.identity.get('task_scope',''),
            course_two_sampling=self.identity.get('sampling',''),
            course_two_scene_profile=self.identity.get('scene_profile',''),
            course_two_catalog='' if simple else str(catalog),course_two_catalog_sha='' if simple else ctl.sha(catalog),
            save_interval_s=300.,snapshot_max_bytes=512*1024**2,total_output_max_bytes=12*1024**3,
            metrics_max_bytes=16*1024**2,owned_pss_limit_bytes=20*1024**3,system_reserve_bytes=4*1024**3,
            artifact_root=str(self.root/'arms'/arm/'training'),output_dir=str(self.root/'arms'/arm/'training/run'))

    def initialize_arm(self,arm):
        from .config import config_record
        from .checkpoint import TrainingState,CheckpointManager
        config=self.arm_config(arm);directory=self.root/'arms'/arm;path=Path(config.output_dir)/'resume.pt'
        if path.exists():checkpoint(path).validate(config);return config
        state=TrainingState.fork_from(self.root/'frozen/source-resume.pt',config,experiment='course-two-rebalance')
        manager=CheckpointManager(config.output_dir,config.save_interval_s,total_max_bytes=config.total_output_max_bytes,
            snapshot_max_bytes=config.snapshot_max_bytes,artifact_root=config.artifact_root)
        manager.save(state);ctl.write(directory/'config.json',config_record(config))
        return config

    def pilot(self,arm,baseline):
        from .evaluation import export_actor
        from .training import state_checksum
        config=self.initialize_arm(arm);directory=self.root/'arms'/arm
        current=Path(config.output_dir)/'resume.pt';prepared=directory/'prefill.pt'
        if not prepared.exists():
            from .replay import ReplayBuffer
            from .course_two_rebalance import preparation_ready
            state=checkpoint(current)
            replay=ReplayBuffer.from_snapshot(state.record['replay'],config.replay_max_bytes)
            replay.enable_course_two_sampling(completion_bonus=config.completion_bonus, task_scope=config.course_two_task_scope, sampling=config.course_two_sampling, scene_profile=config.course_two_scene_profile)
            ready=preparation_ready(arm,replay)
            del replay
            remaining=4096-(state.record['schedule']['transitions']-self.identity['baseline_transitions'])
            if not ready:
                if remaining<=0:raise ValueError('prefill budget exhausted')
                self.save(phase='prefill',arm=arm,target_transitions=state.record['schedule']['transitions']+remaining)
                args=[sys.executable,'-m','lunar_drl_exploration.cli','train','--config',str(directory/'config.json'),
                    '--resume','--collect-only','--max-transitions',str(remaining),'--domain-base','180']
                if not self.spawn(args,'prefill-'+arm,DRL_COURSE_TWO_STAGE='A1'):return False
            state=checkpoint(current)
            source=checkpoint(self.root/'frozen/source-resume.pt')
            if state_checksum(state.record['learner'])!=state_checksum(source.record['learner']):
                raise ValueError('prefill changed model/optimizer/temperature')
            from .replay import ReplayBuffer
            replay=ReplayBuffer.from_snapshot(state.record['replay'],config.replay_max_bytes)
            index=replay.enable_course_two_sampling(completion_bonus=config.completion_bonus, task_scope=config.course_two_task_scope, sampling=config.course_two_sampling, scene_profile=config.course_two_scene_profile)
            if not preparation_ready(arm,replay):
                ctl.write(directory/'missing-replay.json',index.status());raise ValueError('prefill missing replay buckets')
            shutil.copyfile(current,prepared)
            ctl.write(directory/'prefill-proof.json',dict(learner_sha=state_checksum(state.record['learner']),
                source_learner_sha=state_checksum(source.record['learner']),schedule=state.record['schedule'],
                replay=index.status(),checkpoint_sha=ctl.sha(prepared)))
        prepared_state=checkpoint(prepared);target=prepared_state.record['schedule']['transitions']+2048
        state=checkpoint(current);remaining=ctl.remaining_budget(target,state.record['schedule']['transitions'])
        self.save(phase='training',arm=arm,target_transitions=target)
        if remaining or Fraction(state.record['schedule']['credit'])>=1:
            args=[sys.executable,'-m','lunar_drl_exploration.cli','train','--config',str(directory/'config.json'),
                '--resume','--max-transitions',str(remaining),'--domain-base','180']
            if not self.spawn(args,'train-'+arm,DRL_COURSE_TWO_STAGE='A1'):return False
        state=checkpoint(current)
        if (state.record['schedule']['transitions']!=target or
                state.record['schedule']['updates']-prepared_state.record['schedule']['updates']!=512):
            raise ValueError('pilot stopped before exact 2048/512 boundary; inspect learner progress')
        frozen=directory/'pilot.pt'
        if not frozen.exists():shutil.copyfile(current,frozen)
        elif ctl.sha(frozen)!=ctl.sha(current):raise ValueError('pilot checkpoint changed during evaluation')
        actor=self.root/(arm+'.pt')
        if not actor.exists():export_actor(frozen,actor)
        rows=self.screen(actor,ctl.read(self.root/'cases.json'),arm)
        if rows is None:return False
        result=dict(gate(rows,baseline),checkpoint_sha=ctl.sha(frozen),actor_sha=ctl.sha(actor))
        ctl.write(directory/'gate.json',result)
        history=dict(self.state['results']);history[arm]=result;self.save(results=history)
        return True

    def extend(self,baseline):
        """User-authorized larger budget; evaluate every immutable block."""
        from .evaluation import export_actor
        blocks=self.identity['continuation_blocks'];size=self.identity['block_admissions']
        if (type(blocks) is not int or type(size) is not int or not 1<=blocks<=16
                or size<=0 or size%4 or self.identity['pilot_admissions']!=blocks*size
                or self.identity['pilot_updates']!=blocks*size//4):
            raise ValueError('invalid extension budget')
        config=self.initialize_arm('C3');directory=self.root/'arms/C3'
        current=Path(config.output_dir)/'resume.pt'
        start=checkpoint(directory/'prefill.pt').record['schedule']
        for index in range(1,blocks+1):
            if self.stopping:return False
            label=f'C3-block{index:02d}'
            if label in self.state['results']:continue
            target=start['transitions']+index*size
            state=checkpoint(current);remaining=ctl.remaining_budget(target,state.record['schedule']['transitions'])
            self.save(phase='training',arm='C3',target_transitions=target,
                block_index=index,blocks_total=blocks,block_start_transitions=start['transitions']+(index-1)*size)
            if remaining or Fraction(state.record['schedule']['credit'])>=1:
                args=[sys.executable,'-m','lunar_drl_exploration.cli','train','--config',str(directory/'config.json'),
                    '--resume','--max-transitions',str(remaining),'--domain-base','180']
                if not self.spawn(args,'train-'+label,DRL_COURSE_TWO_STAGE='A1'):return False
            state=checkpoint(current)
            if (state.record['schedule']['transitions']!=target or
                    state.record['schedule']['updates']!=start['updates']+index*size//4):
                raise ValueError('extended block stopped before its exact training boundary')
            frozen=directory/f'block{index:02d}.pt'
            if not frozen.exists():shutil.copyfile(current,frozen)
            elif ctl.sha(frozen)!=ctl.sha(current):raise ValueError('extended checkpoint changed during evaluation')
            actor=self.root/(label+'.pt')
            if not actor.exists():export_actor(frozen,actor)
            self.save(candidate_label=label)
            rows=self.screen(actor,ctl.read(self.root/'cases.json'),label)
            if rows is None:return False
            result=dict(gate(rows,baseline),checkpoint_sha=ctl.sha(frozen),actor_sha=ctl.sha(actor),
                block_index=index,additional_transitions=index*size,additional_updates=index*size//4,
                continuation_authorized=True)
            ctl.write(directory/f'gate-block{index:02d}.json',result)
            ctl.write(directory/'gate.json',result)
            history=dict(self.state['results']);history[label]=result;self.save(results=history)
        return True

    def run(self):
        if ctl.runtime_digest()!=self.identity['runtime']:raise ValueError('trial runtime changed')
        for name,digest in self.identity['frozen'].items():
            if ctl.sha(self.root/'frozen'/name)!=digest:raise ValueError('frozen input changed: '+name)
        with ctl.root_lock(self.root):
            ctl.write(self.root/'owner.json',dict(pid=os.getpid(),start_ticks=ctl.process_ticks(os.getpid()),root=str(self.root)))
            for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,self.interrupt)
            self.save(status='running',controller_pid=os.getpid())
            try:
                if not self.qualify_all():return
                baseline=self.screen(self.root/'frozen/block31.pt',ctl.read(self.root/'cases.json'),'baseline')
                if baseline is None:return
                if self.identity.get('continuation_blocks'):
                    if not self.extend(baseline):return
                else:
                    for arm in ARMS:
                        if self.stopping:return
                        if arm not in self.state['results'] and not self.pilot(arm,baseline):return
                self.save(status='needs_attention',phase='training_review_ready',arm=None,
                    reason='C3 training block evaluated; review regression before further training')
            except Exception as exc:
                self.save(status='needs_attention',error=f'{type(exc).__name__}: {exc}');raise
            finally:
                if self.stopping:self.save(status='paused')
                self.save(child_pid=None,qualification_workers=[],evaluation_workers=[])


def main():
    import argparse,json
    p=argparse.ArgumentParser();p.add_argument('command',choices=('prepare','start','resume','run','stop','status'))
    p.add_argument('--root',type=Path,default=DEFAULT_ROOT);a=p.parse_args();root=a.root.resolve()
    if a.command=='prepare':result=prepare(root)
    elif a.command in ('start','resume'):result=start(root)
    elif a.command=='stop':result=stop(root)
    elif a.command=='run':return Trial(root).run()
    else:result=dict(ctl.read(root/'state.json'),live=owner(root))
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
