"""Persistent sampled learning statistics, immutable probes, strict paired screens."""
import csv
import io
import math
from pathlib import Path
import subprocess
import threading
import time
from controller import HERE, read, write, sha, digest, validate_evaluation, CASE_KEYS, PAIR_KEYS

KEYS=('critic1_loss','critic2_loss','actor_loss','target_mean','alpha_position','alpha_heading',
      'position_entropy','heading_conditional_entropy','position_entropy_target','heading_entropy_target',
      'position_entropy_gap','heading_entropy_gap','position_alpha_at_cap','heading_alpha_at_cap',
      'position_max_probability','raw_score_span','score_saturation_fraction')


def empty_history():
    return dict(schema='task_frontier_curves_v1',last_sampled_update=0,buckets={},fixed=[],evaluations=[],reward=[],
        notes=['Sampled progress only; absent/overwritten optimizer samples are not reconstructed.',
               'Fixed measured states are diagnostics, not independent evaluation.',
               'Only complete frozen paired screens with matching scene and budget enter evaluation curves.'])


def ingest(data,rows):
    for row in rows:
        learning=dict(row.get('learning') or {})
        update=learning.get('updates',row.get('updates',0))
        if not learning or update<=data['last_sampled_update']:continue
        data['last_sampled_update']=update
        for prefix,entropy,target in (('position','position_entropy','position_entropy_target'),
                                      ('heading','heading_conditional_entropy','heading_entropy_target')):
            if entropy in learning and target in learning:learning[prefix+'_entropy_gap']=learning[entropy]-learning[target]
            if 'alpha_'+prefix in learning:learning[prefix+'_alpha_at_cap']=float(learning['alpha_'+prefix]>=1e-4*(1-1e-6))
        bucket=data['buckets'].setdefault(str((update-1)//100),dict(first=update,last=update,samples=0,metrics={}))
        bucket['last']=update;bucket['samples']+=1
        for key in KEYS:
            if key not in learning:continue
            value=float(learning[key])
            if not math.isfinite(value):raise ValueError('nonfinite learning curve '+key)
            stats=bucket['metrics'].setdefault(key,dict(count=0,sum=0.,sum_sq=0.,min=value,max=value))
            stats['count']+=1;stats['sum']+=value;stats['sum_sq']+=value*value
            stats['min']=min(stats['min'],value);stats['max']=max(stats['max'],value)


def csv_bytes(rows,required):
    columns=list(dict.fromkeys([*required,*(key for row in rows for key in row)]))
    buffer=io.StringIO(newline='');writer=csv.DictWriter(buffer,fieldnames=columns)
    writer.writeheader();writer.writerows(rows)
    return buffer.getvalue().encode('utf-8')


def fixed_scores(checkpoint,actor_path,probe_root,semantic_id):
    import numpy as np
    import torch
    from lunar_drl_exploration.model import Actor,Critic
    from lunar_drl_exploration.config import ModelConfig
    from lunar_drl_exploration.checkpoint import TrainingState
    from lunar_drl_exploration.batch import pack_observations,pack_privileged
    from fixed_probes import load_probes
    index,probes=load_probes(probe_root)
    checkpoint_sha,actor_sha=sha(checkpoint),sha(actor_path)
    state=TrainingState(torch.load(checkpoint,map_location='cpu',weights_only=False));state.validate()
    learner=state.record['learner'];config=ModelConfig(**learner['model_config'])
    actor=Actor(config);actor.load_state_dict(learner['actor']);actor.eval()
    exported=torch.load(actor_path,map_location='cpu',weights_only=True)
    if exported['version']!=learner['updates'] or any(not torch.equal(v,exported['state_dict'][k]) for k,v in learner['actor'].items()):
        raise ValueError('fixed scores Actor/checkpoint mismatch')
    critics=[Critic(config),Critic(config)]
    for model,name in zip(critics,('q1','q2')):model.load_state_dict(learner[name]);model.eval()
    rows=[]
    with torch.inference_mode():
        for metadata,inputs in probes:
            observation=inputs['observation']
            measured=pack_observations([observation],'cpu')
            policy=actor.forward_packed(measured)  # Actor has no privileged argument.
            truth=pack_privileged([observation],[inputs['privileged']],inputs['scenes'],'cpu')
            q1,q2=[model.forward_packed(measured,truth).cpu().numpy() for model in critics]
            probs=policy.probs.cpu().numpy();q=np.minimum(q1,q2)
            actor_order=np.argsort(-probs,kind='stable');q_order=np.argsort(-q,kind='stable')
            actor_rank=np.argsort(actor_order);q_rank=np.argsort(q_order)
            nodes=observation.action_nodes
            # Feature columns use physical heading bins, action_yaws are their selected angles.
            headings=np.rint(observation.action_yaws/(np.pi/4)).astype(int)%8
            current_inside=bool(observation.features[observation.current_index,2])
            for action,node in enumerate(nodes):
                inside=bool(observation.features[node,2]);heading=headings[action]
                position_probability=float(probs[nodes==node].sum())
                rows.append(dict(updates=learner['updates'],checkpoint_sha=checkpoint_sha,actor_sha=actor_sha,
                    semantic_id=semantic_id,probe_set_hash=index['probe_set_hash'],probe_hash=metadata['probe_hash'],
                    action=action,node_id=int(observation.node_ids[node]),goal_x=float(observation.goals[action,0]),
                    goal_y=float(observation.goals[action,1]),goal_yaw=float(observation.goals[action,2]),
                    inside_count=float(observation.features[node,3+heading])*100.,
                    outside_count=float(observation.features[node,11+heading])*100.,
                    history=float(observation.features[node,19+heading]),inside=inside,
                    exit_action=current_inside and not inside,return_action=not current_inside and inside,
                    probability=float(probs[action]),position_probability=position_probability,
                    q1=float(q1[action]),q2=float(q2[action]),min_q=float(q[action]),
                    actor_rank=int(actor_rank[action])+1,q_rank=int(q_rank[action])+1))
    if sha(checkpoint)!=checkpoint_sha or sha(actor_path)!=actor_sha:raise ValueError('model changed during fixed scoring')
    return rows


class CurveRecorder:
    def __init__(self,root):
        self.root=Path(root);self.directory=self.root/'curves'
        self.directory.mkdir(parents=True,exist_ok=True)
        self.path=self.directory/'history.json'
        self.data=read(self.path) if self.path.exists() else empty_history()
        self.lock=threading.RLock();self.closed=threading.Event();self.thread=None;self.error=None
    def start(self):
        def loop():
            while True:
                try:self.sample();self.persist()
                except Exception as exc:
                    self.error=f'{type(exc).__name__}: {exc}'
                    write(self.root/'curves-error.json',dict(error=self.error,time=time.time()))
                if self.closed.wait(30):break
        self.thread=threading.Thread(target=loop,daemon=False);self.thread.start()
    def sample(self):
        with self.lock:
            metrics=self.root/'run/metrics.jsonl'
            rows=[]
            if metrics.exists():
                for line in metrics.read_text(encoding='utf-8').splitlines():
                    try:rows.append(__import__('json').loads(line))
                    except ValueError:continue
            ingest(self.data,rows)
            frozen_cases=read(self.root/'cases.json')['cases'] if (self.root/'cases.json').exists() else []
            evaluation=[]
            baseline_path=self.root/'baseline-summary.json'
            baseline=read(baseline_path) if baseline_path.exists() else None
            for path in sorted(self.root.glob('*-summary.json')):
                if path.name=='reward-summary.json':continue
                result=read(path)
                if not frozen_cases or result.get('status')!='finished':continue
                cases=result['cases']
                expected=result.get('expected_cases') or [dict(case,config_digest=row['config_digest'],**{key:(baseline['cases'][i][key]
                    if baseline else row[key]) for key in PAIR_KEYS}) for i,(case,row) in enumerate(zip(frozen_cases,cases))]
                if len(expected)!=len(cases) or (not result.get('expected_cases') and len(expected)!=len(frozen_cases)):continue
                validate_evaluation(result,actor_sha=result['actor_sha'],semantic_id=result['semantic_id'],cases=expected)
                for row in cases:
                    evaluation.append(dict(label=result['label'],stage=result['stage'],actor_sha=result['actor_sha'],
                        semantic_id=result['semantic_id'],**{key:row.get(key) for key in (*CASE_KEYS,*PAIR_KEYS,
                        'start_mode','curriculum_task','decisions','final_coverage','completed','zero_gain_ratio','max_zero_gain_run',
                        'zero_gain_two_point_loop_steps','reverse_edge_count','outside_steps','max_outside_run','task_exits','task_returns')}))
            self.data['evaluations']=evaluation
            course_path=self.root/'run/course-summary.json'
            if course_path.exists():
                course=read(course_path);total_samples=sum(v['samples'] for v in course['by_task'].values())
                total_transitions=sum(v['transitions'] for v in course['by_task'].values())
                self.data.setdefault('course',[])
                new=[]
                for task,stats in course['by_task'].items():
                    coverage=course.get('coverage_stats',{}).get(task,{});count=coverage.get('count',0);recent=coverage.get('recent',[])
                    new.append(dict(updates=course.get('updates',0),phase=course['phase'],task=task,**stats,
                        coverage_samples=count,coverage_sum=coverage.get('sum',0.),
                        mean_final_coverage=coverage.get('sum',0.)/count if count else None,
                        recent_final_coverage=sum(recent)/len(recent) if recent else None,
                        recent_coverage_samples=len(recent),missing_coverage_samples=max(0,stats['ended']-count),
                        transition_fraction=stats['transitions']/total_transitions if total_transitions else 0.,
                        sample_fraction=stats['samples']/total_samples if total_samples else 0.,
                        success_rate=stats['successes']/stats['ended'] if stats['ended'] else 0.,
                        mean_episode_steps=stats['episode_steps']/stats['ended'] if stats['ended'] else 0.))
                self.data['course']=[r for r in self.data['course'] if r['updates']!=course.get('updates',0)]+new
                self.data['course']=self.data['course'][-4096:]
            path=self.root/'run/reward-summary.json'
            if path.exists():
                value=read(path);totals=value['totals'];updates=value['training_boundary']['updates']
                row=dict(updates=updates,**totals['reward_components'],reward_sum=totals['reward_sum'],
                         reward_mismatch_count=totals['reward_mismatch_count'],zero_gain=totals['zero_gain'],
                         new_area_m2=totals['new_area_m2'],removed_frontiers_in=totals['removed_frontiers_in'],
                         removed_frontiers_out=totals['removed_frontiers_out'])
                self.data['reward']=[r for r in self.data['reward'] if r['updates']!=updates]+[row]
                self.data['reward']=self.data['reward'][-1024:]
    def fixed_checkpoint(self,checkpoint,actor):
        if not (self.root/'fixed-probes/index.json').exists():
            from fixed_probes import freeze_index
            freeze_index(self.root/'fixed-probes')
        model_sha=sha(checkpoint)
        with self.lock:
            if any(r['checkpoint_sha']==model_sha for r in self.data['fixed']):return
        rows=fixed_scores(checkpoint,actor,self.root/'fixed-probes',read(self.root/'identity.json')['semantic_id'])
        with self.lock:
            self.data['fixed'].extend(rows)
            if rows:
                update=rows[0]['updates'];bucket=self.data['buckets'].setdefault(str(max(0,update-1)//100),dict(first=update,last=update,samples=0,metrics={}))
                for key in ('q1','q2'):
                    values=[row[key] for row in rows]
                    bucket['metrics']['fixed_'+key+'_mean']=dict(count=len(values),sum=sum(values),sum_sq=sum(v*v for v in values),min=min(values),max=max(values))
        self.persist()
    def persist(self):
        from lunar_drl_exploration.artifacts import atomic_write,atomic_json
        with self.lock:
            learning=[]
            for key,bucket in sorted(self.data['buckets'].items(),key=lambda pair:int(pair[0])):
                for metric,stats in bucket['metrics'].items():
                    learning.append(dict(bucket=int(key),first_update=bucket['first'],last_update=bucket['last'],
                                         metric=metric,mean=stats['sum']/stats['count'],**stats))
            data={'learning':(learning,['bucket','metric','count','sum','sum_sq','min','max','mean']),
                  'fixed':(self.data['fixed'],['updates','probe_set_hash','actor_sha','q1','q2','min_q','actor_rank','q_rank']),
                  'evaluation':(self.data['evaluations'],['stage','family','final_coverage','completed']),
                  'reward':(self.data['reward'],['updates','exploration','motion','step','completion'])}
            atomic_json(self.path,self.data,max_bytes=16*1024**2)
            if 'course' in self.data:
                payload=csv_bytes(self.data['course'],['updates','phase','task','transitions','samples','ended','successes','episode_steps'])
                atomic_write(self.directory/'course.csv',lambda stream:stream.write(payload),max_bytes=16*1024**2)
            for name,(rows,columns) in data.items():
                payload=csv_bytes(rows,columns)
                atomic_write(self.directory/f'{name}.csv',lambda stream:stream.write(payload),max_bytes=16*1024**2)
                png=subprocess.check_output(['/usr/bin/python3',str(HERE/'plot_curves.py'),name,str(self.directory/f'{name}.csv')],timeout=30)
                atomic_write(self.directory/f'{name}.png',lambda stream:stream.write(png),max_bytes=1024**2)
    def close(self):
        self.closed.set()
        if self.thread:self.thread.join()
        self.sample();self.persist()
        if self.error:raise RuntimeError('curve recording failed: '+self.error)
