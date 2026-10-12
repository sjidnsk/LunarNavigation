"""Three ordinary scene types; adaptive collection and equal episode sampling."""
from collections import Counter,defaultdict
from copy import deepcopy
import math
import statistics
import numpy as np
from .course_two import CourseTwo,COUNTERS

TASKS=('moon_outside','moon_inside','cave_inside')
SAMPLING='scene_episode_v1'


class SimpleCourse(CourseTwo):
    def __init__(self,state=None,success_coverage=.99):
        self.stage='A1';self.phase=0;self.task='three_scenes';self.rehearsal=0.
        self.success_coverage=success_coverage;self.weights=dict.fromkeys(TASKS,1/3)
        self.active={};self.episodes={};self.by_task={t:dict.fromkeys(COUNTERS,0) for t in TASKS}
        self.coverage_stats={t:dict(count=0,sum=0.,recent=[]) for t in TASKS}
        self.phase_transitions=dict.fromkeys(TASKS,0);self.phase_samples=dict.fromkeys(TASKS,0)
        self.bootstrap_updates=0;self.missing_replay_tasks=[];self.last_sampling_weights={}
        self.action_seconds=dict.fromkeys(TASKS,1.);self.supply_capacity=dict.fromkeys(TASKS,0)
        self.scene_issued=dict.fromkeys(TASKS,0)
        if state is not None:
            if state.get('schema')!='three_scene_course_v1' or state.get('weights')!=self.weights:
                raise ValueError('three-scene curriculum identity changed')
            for name in ('by_task','coverage_stats','phase_transitions','phase_samples','action_seconds','scene_issued'):
                value=deepcopy(state[name])
                if set(value)!=set(TASKS):raise ValueError('three-scene task identity changed')
                setattr(self,name,value)
            if any(not math.isfinite(v) or v<=0 for v in self.action_seconds.values()):
                raise ValueError('invalid scene service time')
            for values in self.by_task.values():
                if set(values)!=set(COUNTERS) or any(type(v)is not int or v<0 for v in values.values()):
                    raise ValueError('invalid three-scene counters')
            for name in ('phase_transitions','phase_samples','scene_issued'):
                if any(type(v)is not int or v<0 for v in getattr(self,name).values()):
                    raise ValueError('invalid scene dispatch/sample counters')
            if max(self.scene_issued.values())-min(self.scene_issued.values())>1:
                raise ValueError('unbalanced scene dispatch counters')
            from .ordered_curriculum import COVERAGE_WINDOW
            for task,stats in self.coverage_stats.items():
                if not isinstance(stats,dict) or set(stats)!={'count','sum','recent'}:
                    raise ValueError('invalid scene coverage fields')
                n,total,recent=stats['count'],stats['sum'],stats['recent']
                if (type(n)is not int or not 0<=n<=self.by_task[task]['ended'] or
                        type(total)not in (int,float) or not math.isfinite(total) or not 0<=total<=n or
                        not isinstance(recent,list) or len(recent)!=min(n,COVERAGE_WINDOW) or
                        any(type(v)not in (int,float) or not math.isfinite(v) or not 0<=v<=1 for v in recent)):
                    raise ValueError('invalid scene coverage counters')

    def set_supply(self,status):
        self.supply_capacity={t:status.get('episode_capacity',{}).get(t,0) for t in TASKS}

    def next_task(self):
        # Balance newly issued complete scenes, not concurrent slots or steps.
        # Fast cave episodes free workers that can immediately take slower moon
        # scenes. Resume reactivates saved specs without issuing them again.
        task=min(TASKS,key=lambda t:self.scene_issued[t])
        self.scene_issued[task]+=1
        return task

    def select_pending(self,pending,reservations):
        # Never idle a ready environment while another scene type is slow.
        candidates=[e for e in pending if e in self.active and e not in reservations]
        projected=Counter(self.phase_transitions)
        projected.update(self.active[e] for e in reservations if e in self.active)
        return min(candidates,key=lambda e:(projected[self.active[e]],e)) if candidates else None

    def record_progress(self,env,progress):
        super().record_progress(env,progress)
        duration=progress.get('last_execution',{}).get('wall_s')
        if isinstance(duration,(int,float)) and math.isfinite(duration) and duration>0:
            task=self.active[env]
            self.action_seconds[task]=.8*self.action_seconds[task]+.2*max(.01,min(120.,duration))

    def ready_for_learning(self,counts):return all(counts.get(t,0)>0 for t in TASKS)

    def sampling_weights(self,counts):
        if not self.ready_for_learning(counts):raise ValueError('missing scene replay')
        self.last_sampling_weights=dict(self.weights);return dict(self.weights)

    def summary(self):
        result=super().summary()
        result.update(schema='three_scene_course_v1',rehearsal=0.,learning_protocol=SAMPLING,
            action_seconds=dict(self.action_seconds),scene_issued=dict(self.scene_issued),
            dispatch_rule='least_issued_scene',
            active_slots=dict(Counter(self.active.values())),supply_capacity=dict(self.supply_capacity))
        return result


def fresh_scene(config,task,ordinal,*,seed=None):
    from .ordered_curriculum import area_budget
    if task not in TASKS or not 0<=ordinal<2**24:raise ValueError('invalid scene sequence')
    seed=((config.seed+3)<<32)+(TASKS.index(task)<<24)+ordinal if seed is None else seed
    extent=float(np.random.default_rng(seed).uniform(*config.curriculum_extents_m[0]))
    family,start=task.split('_')
    spec=dict(seed=seed,family=family,extent=extent,start_mode=start,curriculum_task=task,
        episode_budget=area_budget(extent,config.ordered_budget_rule))
    if family=='cave':spec['task_scope']='whole_cave_v1'
    if task=='moon_outside':spec['outside_distance_max_m']=20.
    if config.course_two_scene_profile:
        from .course_two_sensors import sensor_spec
        spec.update(sensor=sensor_spec(seed,ordinal),scene_profile=config.course_two_scene_profile)
        if task=='moon_outside':
            spec.update(outside_distance_min_m=10.,outside_distance_max_m=50.,context_margin_m=62.)
    return spec


def development_cases(config):
    if config.course_two_scene_profile:
        from dataclasses import replace
        from .course_two_sensors import FOVS
        legacy=replace(config,course_two_scene_profile='')
        cases=[]
        for j,task in enumerate(TASKS):
            for i in range(20):
                if i<5:
                    spec=fresh_scene(legacy,task,i,seed=2026101600000+j*100+i)
                else:
                    k=i-5
                    spec=fresh_scene(config,task,k,seed=2026101700000+j*100+k)
                    spec['sensor']=dict(range_m=(10.,20.,30.)[k%3],fov_deg=FOVS[k%5])
                cases.append(dict(case_id=f'three-{task}-{i:02}',kind='simple',capability=task,
                    spec=spec,scene_profile=config.course_two_scene_profile,
                    evaluation_role='regression' if i<5 else 'adaptation'))
        return cases
    return [dict(case_id=f'three-{task}-{i:02}',kind='simple',capability=task,
        spec=fresh_scene(config,task,i,seed=2026101600000+j*100+i))
        for j,task in enumerate(TASKS) for i in range(20)]


def evaluation_gate(rows,baseline):
    if any(r.get('scene_profile') for r in rows+baseline):
        from .course_two_sensors import evaluation_gate as sensor_gate
        return sensor_gate(rows,baseline)
    identity=lambda items:{(r['case_id'],r['kind'],r.get('capability')) for r in items}
    if (len(rows)!=60 or len(baseline)!=60 or len(identity(rows))!=60 or identity(rows)!=identity(baseline)
            or any(r['kind']!='simple' for r in rows)
            or Counter(r.get('capability') for r in rows)!=dict.fromkeys(TASKS,20)):
        raise ValueError('three-scene evaluation identity/count mismatch')
    success=lambda r:bool(r['completed']) and not r['collisions'] and not r.get('error')
    previous={r['case_id']:r for r in baseline};development={}
    retained=dict(data=not any(r.get('error') for r in rows+baseline),
        collisions=not any(r['collisions'] for r in rows))
    for task in TASKS:
        group=[r for r in rows if r['capability']==task]
        count=sum(map(success,group));before=sum(success(previous[r['case_id']]) for r in group)
        ratios=[r['decisions']/max(1,previous[r['case_id']]['decisions']) for r in group
            if success(r) and success(previous[r['case_id']])]
        ratio=statistics.median(ratios) if ratios else None
        development[task]=dict(success=count,baseline=before,total=20,median_decision_ratio=ratio)
        retained[task]=count>=before-1
        # No paired completions means efficiency is not measurable, not a failure.
        retained[task+'_decisions']=ratio is None or ratio<=1.2
    checks=dict(retained,development=all(g['success']>=18 for g in development.values()))
    return dict(passed=all(checks.values()),stop=not all(retained.values()),checks=checks,
        development=development,next_action='end_block_and_review')


class SceneIndex:
    def __init__(self,completion_bonus=20.,task_scope='whole_cave_v1',scene_profile=''):
        if task_scope!='whole_cave_v1':raise ValueError('three scenes require whole cave scope')
        if scene_profile not in ('','distance_sensor_10_30_v1'):raise ValueError('invalid scene profile')
        self.completion_bonus=completion_bonus;self.task_scope=task_scope;self.sampling=SAMPLING
        self.scene_profile=scene_profile
        self.rows={};self.pools=defaultdict(lambda:defaultdict(set));self.excluded=set();self.last_batch={}

    @property
    def bytes_used(self):return 2048*len(self.rows)+128*len(self.excluded)

    def add(self,index,transition,scenes):
        if index in self.rows or index in self.excluded:raise ValueError('duplicate scene replay index')
        d=scenes[transition.privileged.scene_id].generator_descriptor;t=d.get('curriculum_task')
        if (t not in TASKS or t=='cave_inside' and d.get('task_scope')!=self.task_scope or
                self.scene_profile and d.get('scene_profile')!=self.scene_profile):
            self.excluded.add(index);return
        self.rows[index]=(transition.episode_id,t,transition.step_index)
        self.pools[t][transition.episode_id].add(index)

    def remove(self,index):
        if index in self.excluded:self.excluded.remove(index);return
        episode,task,_=self.rows.pop(index);pool=self.pools[task];pool[episode].remove(index)
        if not pool[episode]:del pool[episode]
        if not pool:del self.pools[task]

    def status(self):
        capacities={t:sum(min(4,len(v)) for v in self.pools.get(t,{}).values()) for t in TASKS}
        missing=[t for t in TASKS if capacities[t]<22]
        return dict(ready=not missing,missing=missing,task_scope=self.task_scope,learning_protocol=SAMPLING,
            scene_profile=self.scene_profile,
            pool_counts={t:sum(map(len,self.pools.get(t,{}).values())) for t in TASKS},
            episode_counts={t:len(self.pools.get(t,{})) for t in TASKS},episode_capacity=capacities,
            phase_quotas=dict.fromkeys(TASKS,'21–22'),scope_excluded_transitions=len(self.excluded))

    def _choose(self,task,count,rng):
        pool=self.pools.get(task,{})
        episodes=list(rng.permutation(sorted(pool)))
        candidates={e:list(rng.permutation(sorted(pool[e])))[:4] for e in episodes}
        selected=[]
        for round in range(4):
            for e in episodes:
                if round<len(candidates[e]):selected.append(int(candidates[e][round]))
                if len(selected)==count:return selected
        raise ValueError('missing diverse scene replay: '+task)

    def sample_indices(self,rng):
        if not self.status()['ready']:raise ValueError('missing scene replay buckets')
        extra=TASKS[int(rng.integers(3))]
        selected=[i for t in TASKS for i in self._choose(t,22 if t==extra else 21,rng)]
        rng.shuffle(selected)
        self.last_batch=dict(phases=dict(Counter(self.rows[i][1] for i in selected)),
            unique_transitions=len(set(selected)),episodes=len({self.rows[i][0] for i in selected}),
            max_per_episode=max(Counter(self.rows[i][0] for i in selected).values()))
        return selected

    def required_indices(self):
        rng=np.random.default_rng(0);result=set()
        for t in TASKS:
            capacity=self.status()['episode_capacity'][t]
            if capacity:result.update(self._choose(t,min(22,capacity),rng))
        return result
