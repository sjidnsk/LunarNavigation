"""Measured navigation predecessors; no replay admission or synthetic known map."""
from collections import Counter
import math
import time
import numpy as np
from .course_two_rebalanced_scene import validate_start


def handover_hash(env):
    from dataclasses import asdict
    import hashlib,json
    digest=hashlib.sha256()
    digest.update(json.dumps(dict(pose=asdict(env.plant.pose),sensor=asdict(env.sensor)),sort_keys=True).encode())
    arrays=[env.privileged.observed,env.task.polygon]
    for observation in (env.observation,env.fine_observation):
        arrays.extend(getattr(observation,name) for name in
            ('positions','features','edges','edge_lengths','context','action_nodes','action_yaws','goals'))
    for value in arrays:
        a=np.ascontiguousarray(value)
        digest.update(str((a.dtype,a.shape)).encode());digest.update(a.tobytes())
    return digest.hexdigest()


def inside_task(env,xy=None):
    from .geometry import polygon_mask
    x,y=(env.plant.pose.x,env.plant.pose.y) if xy is None else xy
    return bool(polygon_mask((1,1),(x-.1,y-.1),.2,env.task.polygon)[0,0])


def prepare_start(env,case,*,construct=False,inside=None,progress=None):
    goals=case.get('prelude_goals',[])
    limit=case.get('max_prelude_decisions',512)
    if len(goals)>limit:raise ValueError('prelude limit exceeded')
    inside=(lambda:inside_task(env)) if inside is None else inside
    started=time.monotonic();p=env.plant
    before=(p.distance_m,p.turn_rad,p.simulation_s)
    executed=[];executions=[];visits=Counter();return_start=None
    def coverage():return env.reference.coverage_ratio(env.privileged.observed)
    def execute(goal):
        if time.monotonic()-started>1200:raise TimeoutError('frozen prelude exceeds 1200 seconds')
        executions.append(env.prelude_goal(tuple(map(float,goal))))
        executed.append(list(map(float,goal)))
        if progress:progress(dict(phase='prelude',decisions=len(executed),coverage=coverage(),
            distance_m=p.distance_m-before[0],wall_s=time.monotonic()-started))
    for goal in goals:execute(goal)
    if construct and case['role'] in ('finish','return'):
        if goals:raise ValueError('cannot reconstruct an already frozen predecessor')
        from .contracts import Pose
        from .feature_schema import IN_GAIN_SLICE
        from .sensor import SensorModel
        while not (.9<=coverage()<=.98 and inside()==(case['role']=='finish')):
            if len(executed)>=limit or coverage()>.98 or p.collision:
                raise ValueError('bounded measured predecessor cannot form requested start')
            observation=env.observation
            positions=observation.goals
            if not len(positions):raise ValueError('prelude has no measured candidate')
            headings=np.rint(observation.action_yaws/(math.pi/4)).astype(int)%8
            gains=observation.features[observation.action_nodes,IN_GAIN_SLICE.start+headings]
            distances=np.linalg.norm(positions[:,:2]-[p.pose.x,p.pose.y],axis=1)
            score=gains/(1+distances/10)
            for i,goal in enumerate(positions):score[i]/=(1+visits[tuple(np.round(goal,2))])**2
            if coverage()>=.9:
                if return_start is None:return_start=p.distance_m
                allowed=np.array([inside_task(env,g[:2])==(case['role']=='finish') for g in positions])
                allowed&=distances<=case['return_route_max_m']-(p.distance_m-return_start)
                score=np.where(allowed,1/(.1+distances),-np.inf)
            candidates=np.argsort(-score,kind='stable')
            goal=None
            # Truth is used only to reject a predecessor that would overshoot.
            # The learner never sees the reference or this selector.
            reference=env.reference.mask();known=env.reference.unpack(env.privileged.observed)
            for i in candidates[:48]:
                if not np.isfinite(score[i]):continue
                g=positions[i]
                visible=SensorModel.observe(env.terrain,Pose(*g),env.sensor).mask
                expected=np.count_nonzero((known|visible)&reference)/env.reference.cell_count
                if expected<=.98:
                    goal=g;break
            if goal is None:raise ValueError('no safe predecessor endpoint below 98 percent')
            visits[tuple(np.round(goal,2))]+=1
            if visits[tuple(np.round(goal,2))]>6:raise ValueError('predecessor candidate cycle')
            execute(goal)
    actual=coverage();is_inside=inside()
    validate_start(case['role'],actual,is_inside)
    if case['role']=='return' and return_start is not None and p.distance_m-return_start>case['return_route_max_m']+1e-6:
        raise ValueError('return predecessor exceeds path bound')
    record=dict(recipe=case.get('prelude_recipe'),goals=executed,executions=executions,decisions=len(executed),
        distance_m=p.distance_m-before[0],turn_rad=p.turn_rad-before[1],
        simulation_s=p.simulation_s-before[2],wall_s=time.monotonic()-started,
        coverage=actual,inside=is_inside,handover_hash=handover_hash(env),
        return_distance_m=None if return_start is None else p.distance_m-return_start)
    env.set_policy_start(record)
    return record


def certify_completion(env,*,limit=512,progress=None,return_limit=None):
    """Construct an actual suffix; keep it outside learner inputs and replay."""
    from .contracts import Pose
    from .sensor import SensorModel
    started=time.monotonic();distance=env.plant.distance_m;goals=[];visits=Counter()
    reference=env.reference.mask();return_distance=None
    for step in range(limit):
        coverage=env.reference.coverage_ratio(env.privileged.observed)
        if return_limit is not None and return_distance is None and inside_task(env):
            return_distance=env.plant.distance_m-distance
        if env.completed:
            if return_limit is not None and (return_distance is None or return_distance>return_limit):
                raise ValueError('completion route did not return within bound')
            return dict(goals=goals,decisions=len(goals),coverage=coverage,
                distance_m=env.plant.distance_m-distance,return_distance_m=return_distance,
                wall_s=time.monotonic()-started,completed=True)
        known=env.reference.unpack(env.privileged.observed);best=None
        for goal in env.observation.goals:
            key=tuple(np.round(goal,2));d=math.hypot(goal[0]-env.plant.pose.x,goal[1]-env.plant.pose.y)
            gain=np.count_nonzero(SensorModel.observe(env.terrain,Pose(*goal),env.sensor).mask&reference&~known)
            score=(gain+1e-3)/(1+d/10)/(1+visits[key])**2
            if return_limit is not None and return_distance is None:
                if not inside_task(env,goal[:2]) or d>return_limit-(env.plant.distance_m-distance):continue
            if best is None or score>best[0]:best=(score,goal,key)
        if best is None:raise ValueError('no bounded completion witness')
        _,goal,key=best;visits[key]+=1
        if visits[key]>6:raise ValueError('completion witness cycle')
        env.prelude_goal(tuple(goal));goals.append(list(map(float,goal)))
        if progress:progress(dict(phase='completion_certificate',decisions=len(goals),
            coverage=env.reference.coverage_ratio(env.privileged.observed),distance_m=env.plant.distance_m-distance))
    raise ValueError('completion witness exceeds decision budget')


def resolve_generated(env,case):
    env._preparation_deadline=time.monotonic()+2400.
    try:return _resolve_generated(env,case)
    finally:del env._preparation_deadline


def _resolve_generated(env,case):
    """Same bounded native gate for new training seeds after the initial catalog."""
    from .course_two_rebalanced_scene import make_case,build_case
    from .course_two_scene import _identity
    request=case['qualification_request'];rejected=[]
    start=time.monotonic()
    for attempt in range(request.get('max_geometry_attempts',100)):
        candidate=make_case(request,attempt=attempt)
        try:
            terrain,pose,task,sensor,descriptor=build_case(candidate,env.config.platform)
            env.episode_budget=candidate['episode_budget']
            env.reset_scene(terrain,pose,task,sensor=sensor,descriptor=descriptor)
            def bound(_):
                if time.monotonic()>env._preparation_deadline:raise TimeoutError('online native qualification exceeded 2400 seconds')
            prelude=prepare_start(env,candidate,construct=True,progress=bound)
            witness=None
            if candidate['role'] in ('finish','return'):
                witness=certify_completion(env,progress=bound,
                    return_limit=10. if candidate['role']=='return' else None)
            if prelude['goals']:
                candidate.pop('scenario_id');candidate['prelude_goals']=prelude['goals']
                candidate['scenario_id']=_identity(candidate)
            # A completed certification trajectory is never the learner start.
            if witness is not None:
                from .course_two_qualification import spec_for
                env.reset(**spec_for(candidate))
            env.episode_metadata.update(resolved_course_two_case=candidate,native_qualification=dict(
                status='passed',rejections=rejected,completion_certificate=witness,
                total_wall_s=time.monotonic()-start))
            return env.observation,env.privileged
        except ValueError as exc:
            rejected.append(dict(attempt=attempt,error=str(exc)));env.close()
    raise ValueError('native qualification exhausted attempts: '+str(rejected))
