"""Explicit A1 rebuilding arms; admission quotas are not learning quotas."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from .course_two import CourseTwo, COUNTERS, OLD_TASKS

SCHEMA='region_sensor_rebalance_v2'
VARIANTS=('C0','C1','C2','C3')


class CatalogExhausted(RuntimeError):
    pass


class RebalancedCourse(CourseTwo):
    def __init__(self, variant, state=None, success_coverage=.99):
        if variant not in VARIANTS:raise ValueError('invalid rebalance variant')
        self.variant=variant;self.stage='A1';self.phase=0;self.task='c2_A1'
        self.rehearsal=.5;self.success_coverage=success_coverage
        self.weights=dict.fromkeys(OLD_TASKS,.125)
        self.weights.update({'c2_A1':.5} if variant in ('C0','C1') else
            {'c2_A1_cover':.25,'c2_A1_approach':.125,'c2_A1_finish':.125})
        self.active={};self.episodes={}
        self.by_task={task:dict.fromkeys(COUNTERS,0) for task in self.weights}
        self.coverage_stats={task:dict(count=0,sum=0.,recent=[]) for task in self.weights}
        self.phase_transitions=dict.fromkeys(self.weights,0)
        self.phase_samples=dict.fromkeys(self.weights,0)
        self.bootstrap_updates=0;self.missing_replay_tasks=[];self.last_sampling_weights={}
        if state is not None:
            if (state.get('schema')!=SCHEMA or state.get('variant')!=variant
                    or state.get('stage')!='A1' or state.get('weights')!=self.weights):
                raise ValueError('rebalance variant/stage identity changed')
            for name in ('by_task','coverage_stats','phase_transitions','phase_samples'):
                value=deepcopy(state[name])
                if set(value)!=set(self.weights):raise ValueError('rebalance task counters changed')
                setattr(self,name,value)
            for row in self.by_task.values():
                if set(row)!=set(COUNTERS) or any(type(v) is not int or v<0 for v in row.values()):
                    raise ValueError('invalid rebalance counters')
            for rows in (self.phase_transitions,self.phase_samples):
                if any(type(v) is not int or v<0 for v in rows.values()):raise ValueError('invalid phase counters')

    def ready_for_learning(self, counts):
        return all(counts.get(task,0)>0 for task in self.weights)

    def sampling_weights(self, counts):
        if not self.ready_for_learning(counts):raise ValueError('rebalance replay missing task')
        old=.2 if self.variant=='C0' else .5
        weights=dict.fromkeys(OLD_TASKS,old/4)
        new={task:counts[task] for task in self.weights if task.startswith('c2_')}
        weights.update({task:(1-old)*count/sum(new.values()) for task,count in new.items()})
        self.last_sampling_weights=weights
        return weights

    def summary(self):
        result=super().summary()
        result.update(schema=SCHEMA,variant=self.variant,rehearsal=.5,
            learning_protocol='phase_episode_v2' if self.variant=='C3' else 'task_weighted_v2')
        return result


def load_catalog(config):
    if config.course_two_sampling=='scene_episode_v1':
        from .course_two_simple import TASKS
        return {t:[] for t in TASKS},[]
    payload=Path(config.course_two_catalog).read_bytes()
    if hashlib.sha256(payload).hexdigest()!=config.course_two_catalog_sha:
        raise ValueError('qualified catalog hash changed')
    catalog=json.loads(payload)
    if catalog.get('schema')!='course_two_qualified_catalog_v2' or catalog.get('status')!='passed':
        raise ValueError('catalog is not native qualified')
    from .course_two_control import runtime_digest
    if catalog.get('runtime')!=runtime_digest():raise ValueError('catalog runtime changed')
    result={}
    for spec in catalog['specs']:
        if config.course_two_task_scope and spec['family']=='cave':
            actual=spec.get('course_two_case',{}).get('task_scope',spec.get('task_scope'))
            if actual!=config.course_two_task_scope:raise ValueError('qualified cave task scope changed')
        result.setdefault(spec['curriculum_task'],[]).append(spec)
    expected=set(RebalancedCourse(config.course_two_variant).weights)
    if set(result)!=expected:raise ValueError('catalog does not supply variant tasks')
    return result,catalog.get('templates',[])


def preparation_ready(variant,replay):
    index=replay.course_two_index
    if variant in ('C2','C3'):return index.status()['ready']
    new={ep for ep,phase,_ in index.rows.values() if phase.startswith('new_')}
    return (len(new)>=4 and all(len(index.pools.get(t,{}))>=4 for t in OLD_TASKS)
        and sum(n for t,n in replay.task_counts.items() if t.startswith('c2_'))>=64)


def fresh_spec(config,task,ordinal,templates):
    """Paired, per-task seed sequence independent of worker completion order."""
    if config.course_two_sampling=='scene_episode_v1':
        from .course_two_simple import fresh_scene
        return fresh_scene(config,task,ordinal)
    from .ordered_curriculum import area_budget
    import numpy as np
    tasks=list(RebalancedCourse(config.course_two_variant).weights)
    if not 0<=ordinal<2**24:raise CatalogExhausted('task seed namespace exhausted')
    seed=((config.seed+2)<<32)+(tasks.index(task)<<24)+ordinal
    if task in OLD_TASKS:
        extent=float(np.random.default_rng(seed).uniform(*config.curriculum_extents_m[0]))
        family,start=task.split('_')
        spec=dict(seed=seed,family=family,extent=extent,start_mode=start,curriculum_task=task,
            episode_budget=area_budget(extent,config.ordered_budget_rule))
        if config.course_two_task_scope and family=='cave':spec['task_scope']=config.course_two_task_scope
        return spec
    if config.course_two_variant in ('C0','C1'):
        from .course_two_scene import make_case
        case=make_case(seed,'A1',ordinal)
    else:
        from .course_two_rebalanced_scene import make_case
        from .course_two_scene import _identity
        role=task.rsplit('_',1)[1]
        choices=[r for r in templates if r['role']==role]
        if not choices:raise CatalogExhausted('qualified role template missing: '+role)
        request=deepcopy(choices[ordinal%len(choices)]);request['seed']=seed
        if config.course_two_task_scope and request.get('task_scope')!=config.course_two_task_scope:
            raise ValueError('fresh scene template task scope changed')
        request['request_ordinal']=ordinal;request['split']='train_continuation'
        # Rotate terrain/role pairing and U/O bearings on successive cycles.
        cycle=ordinal//len(choices)
        if cycle%2:request['family']='cave' if request['family']=='moon' else 'moon'
        if role=='approach':request['bearing_bin']=(request['bearing_bin']+cycle)%8
        request['request_id']=_identity(request)
        case=make_case(request);case.pop('scenario_id')
        case['qualification_request']=request;case['scenario_id']=_identity(case)
    from .course_two_qualification import spec_for
    return spec_for(case)


def course_for(config,state=None):
    if config.course_two_sampling=='scene_episode_v1':
        from .course_two_simple import SimpleCourse
        return SimpleCourse(state,success_coverage=config.success_coverage)
    return RebalancedCourse(config.course_two_variant,state=state,success_coverage=config.success_coverage)
