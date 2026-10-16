"""Four ordered exploration tasks with durable evaluation-gated promotion."""
import argparse
from fractions import Fraction
import os
from pathlib import Path
import signal
import time
from controller import (Controller, HERE, LEGACY, identity, read, write, sha, digest,
                        source_commit, prepare, verify_gate, root_lock, start_background,
                        request_stop, validate_evaluation, CASE_KEYS, PAIR_KEYS, decision_budget)

TASKS=('moon_inside','cave_inside','moon_outside','cave_outside')
BUDGET=dict(mode='linear_area',min_extent_m=50.,max_extent_m=100.,round_to=32,min_decisions=256,max_decisions=512)


def default_plan():
    return dict(schema='ordered_course_v1',tasks=list(TASKS),warmup=1024,block_transitions=8192,
        block_updates=2048,current_cases=5,rehearsal_cases=5,success_rate=.8,max_loop_fraction=.2,
        consecutive_passes=2,max_blocks=[20,10,20,10],extents=[50.,50.+2*50./9.,50.+4*50./9.,50.+7*50./9.,100.],
        evaluation_seed_base=2026100400,scene_seed_overrides={3:2026100410},budget=BUDGET)


def validate_plan(plan):
    if not isinstance(plan,dict) or set(plan)!=set(default_plan()) or plan['schema']!='ordered_course_v1':
        raise ValueError('invalid course plan schema')
    if plan['tasks']!=list(TASKS):raise ValueError('course task order changed')
    for key in ('warmup','block_transitions','block_updates','current_cases','rehearsal_cases','consecutive_passes'):
        if type(plan[key]) is not int or plan[key]<=0:raise ValueError('positive course count required: '+key)
    if plan['current_cases']!=5 or plan['rehearsal_cases']!=5:raise ValueError('course evaluation is five current and five retained cases')
    if len(plan['max_blocks'])!=4 or any(type(v) is not int or v<=0 for v in plan['max_blocks']):
        raise ValueError('four positive phase block limits required')
    if len(plan['extents'])!=5:raise ValueError('five evaluation extents required')
    for extent in plan['extents']:decision_budget(extent,plan['budget'])
    for key in ('success_rate','max_loop_fraction'):
        if type(plan[key]) not in (int,float) or not 0<plan[key]<=1:raise ValueError('invalid course gate '+key)
    if type(plan['evaluation_seed_base']) is not int or not 0<=plan['evaluation_seed_base']<2**32-100:
        raise ValueError('invalid evaluation seed namespace')
    overrides=plan['scene_seed_overrides']
    if overrides not in ({3:2026100410},{'3':2026100410}) or any(str(k)!='3' or type(v) is not int or not 0<=v<2**32 for k,v in overrides.items()):
        raise ValueError('invalid fixed scene seed overrides')
    return plan


def course_cases(phase,phase_block,plan):
    if type(phase) is not int or not 0<=phase<4 or type(phase_block) is not int or phase_block<1:
        raise ValueError('invalid course phase/block')
    tasks=[phase]*5
    if phase:tasks += [(i+phase_block-1)%phase for i in range(5)]
    cases=[]
    for i,task_index in enumerate(tasks):
        family,start_mode=TASKS[task_index].split('_')
        ordinal=i%5;extent=plan['extents'][ordinal]
        seed=plan['scene_seed_overrides'].get(ordinal,plan['scene_seed_overrides'].get(str(ordinal),plan['evaluation_seed_base']+ordinal))
        cases.append(dict(case_index=i,family=family,start_mode=start_mode,curriculum_task=TASKS[task_index],
            seed=seed,extent_m=extent,budget=decision_budget(extent,plan['budget'])))
    return cases


def promotion_gate(rows,phase,plan):
    current=[r for r in rows if r['curriculum_task']==TASKS[phase]]
    old=[r for r in rows if r['curriculum_task'] in TASKS[:phase]]
    if len(current)!=5 or len(old)!=(5 if phase else 0) or len(rows)!=len(current)+len(old):
        raise ValueError('incomplete or unexpected course gate cases')
    rate=sum(bool(r['completed']) and not r['collisions'] for r in current)/len(current)
    retention=sum(bool(r['completed']) and not r['collisions'] for r in old)/len(old) if old else 1.
    decisions=sum(r['decisions'] for r in current)
    loops=sum(r['zero_gain_two_point_loop_steps'] for r in current)
    loop_fraction=loops/decisions if decisions else 0.
    return dict(passed=rate>=plan['success_rate'] and retention>=plan['success_rate'] and loop_fraction<=plan['max_loop_fraction'],
        current_success_rate=rate,retained_success_rate=retention,loop_fraction=loop_fraction,
        current_successes=sum(bool(r['completed']) and not r['collisions'] for r in current))


def remaining_training(target_t,target_u,schedule):
    t,u=schedule['transitions'],schedule['updates'];credit=Fraction(str(schedule['credit']))
    if (any(type(v) is not int or v<0 for v in (t,u)) or t>target_t or u>target_u or credit<0):
        raise ValueError('checkpoint exceeds requested course boundary')
    if t<target_t:return target_t-t
    if u<target_u:
        if credit<target_u-u:raise ValueError('course boundary lacks earned update credit')
        return 0
    if credit or schedule.get('unfinished_reservations',0):raise ValueError('course boundary is not quiescent')
    return None


def initial_state(plan):
    return dict(schema='ordered_course_state_v1',plan_digest=digest(plan),phase=0,blocks=0,
                phase_blocks=0,streak=0,warmup_complete=False,status='running',history=[])


def advance_state(state,gate,plan,actor_sha):
    phase=state['phase'];block=state['blocks']+1;local=state['phase_blocks']+1
    state.update(blocks=block,phase_blocks=local,streak=state['streak']+1 if gate['passed'] else 0)
    state['history'].append(dict(block=block,phase=phase,task=TASKS[phase],phase_block=local,actor_sha=actor_sha,**gate))
    if state['streak']>=plan['consecutive_passes']:
        if phase==3:state['status']='finished'
        else:state.update(phase=phase+1,phase_blocks=0,streak=0)
    elif local>=plan['max_blocks'][phase]:state['status']='needs_attention'
    return state


def apply_a2_entry_result(state, rows, plan, actor_sha, checkpoint_sha):
    """Apply the explicitly authorized entry gate without inventing a training block."""
    entry = state.get('a2_entry_screen', {})
    if (state.get('phase') != 1 or not state.get('warmup_complete')
            or entry.get('status') != 'pending'):
        raise ValueError('A2 entry assessment requires a pending cave_inside continuation')
    gate = promotion_gate(rows, 1, plan)
    state['a2_entry_screen'] = dict(entry, status='finished', gate=gate,
        actor_sha=actor_sha, checkpoint_sha=checkpoint_sha,
        promotion_rule='user_authorized_single_complete_entry_gate_v1',
        source_phase=1, destination_phase=2 if gate['passed'] else 1)
    if gate['passed']:
        state.update(phase=2, phase_blocks=0, streak=0)
    return state


class CourseController(Controller):
    def __init__(self,root):
        super().__init__(root)
        self.plan=validate_plan(read(self.root/'course-plan.json'))
        self.config=read(self.root/'config.json')
        if not self.config.get('ordered_curriculum') or self.config['warmup']!=self.plan['warmup']:
            raise ValueError('ordered course config/warmup mismatch')
        if self.config['update_ratio']*self.plan['block_transitions']!=self.plan['block_updates']:
            raise ValueError('course block update ratio mismatch')
        if self.config.get('ordered_budget_rule')!=self.plan['budget']:
            raise ValueError('training and evaluation budgets differ')
        state_path=self.root/'course-state.json'
        self.course=read(state_path) if state_path.exists() else initial_state(self.plan)
        if self.course.get('plan_digest')!=digest(self.plan):raise ValueError('owned course plan changed')
        if not state_path.exists():write(state_path,self.course)
        self.manifest.update(stages=4,course_phase=self.course['phase'],course_task=TASKS[self.course['phase']],
                             course_plan=str(self.root/'course-plan.json'),
                             policy_graph_mode=self.config.get('policy_graph_mode', 'fine'))

    def save_course(self):
        write(self.root/'course-state.json',self.course)

    def train_boundary(self,target_t,target_u,phase,label):
        self.record(phase='warmup' if label=='course-warmup' else 'training',course_phase=phase,
                    course_task=TASKS[phase],targets=dict(transitions=target_t,updates=target_u))
        remaining=remaining_training(target_t,target_u,self.schedule())
        if remaining is None:return True
        args=['bash',str(HERE/'run.sh'),'python',str(HERE/'train_live.py'),'train','--config',str(self.root/'config.json'),
              '--max-transitions',str(remaining),'--domain-base','190']
        if (self.root/'run/resume.pt').exists():args.append('--resume')
        if not self.spawn(args,label,DRL_STAGE=str(self.course['blocks']+1),DRL_COURSE_PHASE=str(phase)):return False
        if remaining_training(target_t,target_u,self.schedule()) is not None:raise ValueError('course trainer missed exact boundary')
        return True

    def screen_course(self,block,phase,local_block,actor,*,label=None):
        cases=course_cases(phase,local_block,self.plan);label=label or f'course-block{block}'
        write(self.root/'cases.json',dict(schema='course_cases_v1',cases=cases,cases_digest=digest(cases)))
        frozen=self.root/f'{label}-cases.json'
        if frozen.exists() and read(frozen)['cases']!=cases:raise ValueError('frozen course cases changed')
        write(frozen,dict(cases=cases,cases_digest=digest(cases)))
        rows=[];semantic=None
        export=read(actor.with_suffix('.json'))
        if export['actor_sha']!=sha(actor):raise ValueError('course exported Actor changed')
        for case in cases:
            index=case['case_index'];output=self.root/f'{label}-case-{index:02}.json'
            request=dict(case,actor=str(actor),actor_sha=sha(actor),config=str(self.root/'config.json'),
                config_sha=sha(self.root/'config.json'),run_id=self.identity['run_id'],label=label,legacy=False,
                wrapper=str(HERE/'run.sh'),source_commit=source_commit(),root=str(self.root),
                source_checkpoint=str(self.root/'run/resume.pt'),source_checkpoint_sha=export['checkpoint_sha'])
            self.record(phase='evaluation',label=label,stage=block,course_phase=phase,course_task=TASKS[phase],
                actor_sha=request['actor_sha'],case_index=index,eval_completed=len(rows),eval_total=len(cases),
                eval_budget=case['budget'],**{k:case[k] for k in ('family','start_mode','extent_m','seed')})
            write(self.root/'evaluation-request.json',request)
            if not self.spawn(['bash',str(HERE/'run.sh'),'python',str(HERE/'eval_case.py'),
                               '--request',str(self.root/'evaluation-request.json'),'--output',str(output)],f'{label}-{index:02}'):return None
            result=read(output);row=result['cases'][0]
            expected=dict(case,config_digest=row['config_digest'],**{k:row[k] for k in PAIR_KEYS})
            validate_evaluation(result,actor_sha=request['actor_sha'],semantic_id=row['semantic_id'],cases=[expected])
            if semantic is not None and semantic!=row['semantic_id']:raise ValueError('course evaluation semantics changed')
            semantic=row['semantic_id'];rows.append(row)
        expected=[dict(c,config_digest=r['config_digest'],**{k:r[k] for k in PAIR_KEYS}) for c,r in zip(cases,rows)]
        result=dict(status='finished',exit_code=0,label=label,stage=block,course_phase=phase,phase_block=local_block,
                    actor_sha=sha(actor),semantic_id=semantic,cases=rows,expected_cases=expected)
        validate_evaluation(result,actor_sha=sha(actor),semantic_id=semantic,cases=expected)
        write(self.root/f'{label}-summary.json',result)
        self.record(eval_completed=len(cases))
        return result

    def screen_a2_entry(self, recorder):
        """Evaluate the restored learner before collecting or updating anything."""
        if self.config.get('policy_graph_mode', 'fine') != 'a2':
            raise ValueError('A2 entry assessment requires the A2 effective configuration')
        from lunar_drl_exploration.evaluation import export_actor
        checkpoint = self.root/'run/resume.pt'
        actor = self.root/'a2-entry.pt'
        proof_path = actor.with_suffix('.json')
        checkpoint_sha = sha(checkpoint)
        updates = self.schedule()['updates']
        if not proof_path.exists():
            export_actor(checkpoint, actor)
            write(proof_path, dict(actor_sha=sha(actor), checkpoint_sha=checkpoint_sha, updates=updates))
        proof = read(proof_path)
        if (proof['checkpoint_sha'] != checkpoint_sha or proof['actor_sha'] != sha(actor)
                or proof['updates'] != updates):
            raise ValueError('A2 entry Actor/checkpoint changed before assessment completed')
        result = self.screen_course(self.course['blocks'], 1, self.course['phase_blocks'] + 1,
                                    actor, label='a2-entry')
        if result is None:
            return False
        recorder.fixed_checkpoint(checkpoint, actor)
        apply_a2_entry_result(self.course, result['cases'], self.plan, sha(actor), checkpoint_sha)
        self.save_course()
        phase = self.course['phase']
        self.record(course_phase=phase, course_task=TASKS[phase],
                    entry_gate=self.course['a2_entry_screen']['gate'], phase='entry_gate_complete')
        return True

    def run(self,*,ready_fd=None,startup_token=None,lock_fd=None):
        verify_gate(self.root)
        signal.signal(signal.SIGINT,self.interrupt);signal.signal(signal.SIGTERM,self.interrupt)
        from curves import CurveRecorder
        recorder=CurveRecorder(self.root);recorder.start()
        try:
            self.record();self.schedule();self.announce_ready(ready_fd,startup_token,lock_fd)
            if self.course['status']!='running':
                self.record(status=self.course['status'],phase='course_complete' if self.course['status']=='finished' else 'gate_limit',exit_code=0)
                return
            if not self.course['warmup_complete']:
                if not self.train_boundary(self.plan['warmup'],0,0,'course-warmup'):return
                from fixed_probes import freeze_index
                freeze_index(self.root/'fixed-probes')
                self.course['warmup_complete']=True;self.save_course()
            if self.course.get('a2_entry_screen', {}).get('status') == 'pending':
                if not self.screen_a2_entry(recorder):return
            while not self.stop and self.course['status']=='running':
                phase=self.course['phase'];block=self.course['blocks']+1;local=self.course['phase_blocks']+1
                target_t=self.plan['warmup']+block*self.plan['block_transitions'];target_u=block*self.plan['block_updates']
                self.record(stage=block,phase_block=local,course_streak=self.course['streak'])
                if not self.train_boundary(target_t,target_u,phase,f'course-train{block}'):return
                from lunar_drl_exploration.evaluation import export_actor
                actor=self.root/f'course-block{block}.pt';proof_path=actor.with_suffix('.json')
                # Export recovery is idempotent; an interrupted proof write re-exports the same checkpoint.
                if not proof_path.exists():
                    export_actor(self.root/'run/resume.pt',actor)
                    write(proof_path,dict(actor_sha=sha(actor),checkpoint_sha=sha(self.root/'run/resume.pt'),updates=target_u))
                proof=read(proof_path)
                if proof['checkpoint_sha']!=sha(self.root/'run/resume.pt') or proof['actor_sha']!=sha(actor) or proof['updates']!=target_u:
                    raise ValueError('course export provenance differs from boundary')
                result=self.screen_course(block,phase,local,actor)
                if result is None:return
                gate=promotion_gate(result['cases'],phase,self.plan)
                recorder.fixed_checkpoint(self.root/'run/resume.pt',actor)
                advance_state(self.course,gate,self.plan,sha(actor))
                self.save_course()
            self.record(status=self.course['status'],phase='course_complete' if self.course['status']=='finished' else 'gate_limit',exit_code=0)
        except Exception as exc:
            self.record(status='failed',phase='error',error=f'{type(exc).__name__}: {exc}',exit_code=1)
            raise
        finally:
            try:recorder.close()
            except Exception as exc:
                self.record(status='failed',phase='error',error=f'curves: {exc}',exit_code=1)
                raise
            finally:
                if self.stop:
                    self.schedule();self.record(status='paused',phase='saved_exit',exit_code=130)
                write(self.root/'exit.json',dict(run_id=self.identity['run_id'],pid=os.getpid(),status=self.manifest['status'],
                    exit_code=self.manifest['exit_code'],actuals=self.manifest['actuals'],updated_unix=time.time()))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('prepare','start','resume','run','stop','status','monitor'))
    parser.add_argument('--root',required=True,type=Path);parser.add_argument('--config',type=Path)
    parser.add_argument('--run-id');parser.add_argument('--lock-fd',type=int)
    parser.add_argument('--ready-fd',type=int);parser.add_argument('--startup-token')
    args=parser.parse_args(argv);root=args.root.resolve()
    if args.command=='prepare':
        if args.config is None:parser.error('--config required for prepare')
        value=prepare(root,args.config)
        if not (root/'course-plan.json').exists():write(root/'course-plan.json',default_plan())
        print(value);return
    owned=identity(root)
    if args.command=='stop':return request_stop(root)
    if args.command=='status':print(read(root/'manifest.json'));return
    if args.command=='monitor':
        import curses
        from course_dashboard import display
        return curses.wrapper(lambda screen:display(screen,root))
    if args.command in ('start','resume'):
        print(start_background(root,entry=HERE/'course_controller.py'));return
    if args.run_id!=owned['run_id']:raise ValueError('controller run identity mismatch')
    inherited=(args.lock_fd,args.ready_fd,args.startup_token)
    if any(v is not None for v in inherited) and any(v is None for v in inherited):raise ValueError('incomplete startup identity')
    try:
        with root_lock(root,args.lock_fd) as lock_fd:
            CourseController(root).run(ready_fd=args.ready_fd,startup_token=args.startup_token,lock_fd=lock_fd)
    finally:
        if args.ready_fd is not None:os.close(args.ready_fd)


if __name__=='__main__':main()
