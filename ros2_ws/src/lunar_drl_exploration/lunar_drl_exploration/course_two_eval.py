"""One restartable evaluation case using the unchanged native ROS closed loop."""
from dataclasses import replace
import importlib.util
from pathlib import Path
import signal
import time
from .course_two_control import read,write,sha,runtime_digest


def validate_policy_start(case,coverage):
    if case['kind']=='new' and coverage!=0.:
        raise ValueError('evaluation initialization exposed approach target: '+case['case_id'])
    if (case['kind']=='visible' and case['case']['visibility_class']=='V' and coverage<=0.):
        raise ValueError('evaluation initialization lost visible target: '+case['case_id'])


def case_metadata(case):
    return {key:case[key] for key in ('capability','scene_profile','evaluation_role','promotion_required','task_scope')
        if key in case}


def evaluate_case(root, request_path, output, *, slot=0, progress=None, job_id=None):
    import torch
    from .config import training_config_from_record
    from .runtime import ActorPolicy
    from .ros_env import RosExplorationEnv
    from .evaluation import GeometryMetrics
    from .course_two_parallel import validate_slot,EVALUATION_DOMAIN_BASE,evaluation_domain
    from .evaluation_navigation import load_evaluation_navigation
    validate_slot(slot)
    progress=Path(progress) if progress is not None else root/'eval-progress.json'
    request=read(request_path);case=request['case'];actor=Path(request['actor'])
    navigation=load_evaluation_navigation(root)
    proof=dict(actor_sha=sha(actor),case=case,config_sha=sha(root/'config.json'),runtime=runtime_digest())
    if (root/'evaluation-navigation.json').exists():proof['evaluation_navigation']=navigation
    if Path(output).exists():
        cached=read(output)
        if cached.get('proof')!=proof:raise ValueError('cached evaluation identity changed')
        return cached
    torch.set_num_threads(1)
    config=training_config_from_record(read(root/'config.json'))
    config=replace(config,environments=1,stagnation_window_steps=0,
        target_rtf=20. if case['kind']=='diagnostic' else config.target_rtf)
    policy=ActorPolicy.load(actor)
    env=RosExplorationEnv(config,slot,domain_base=EVALUATION_DOMAIN_BASE,**navigation)
    def interrupted(*_):raise KeyboardInterrupt('evaluation interrupted')
    previous={s:signal.signal(s,interrupted) for s in (signal.SIGINT,signal.SIGTERM)}
    row=dict(case_id=case['case_id'],kind=case['kind'],completed=False,collisions=0,error=None,
        decisions=0,initial_roi_visible=False,first_roi=None,trajectory=[],**case_metadata(case))
    started=time.monotonic()
    trace=sample=visual=visual_hook=None
    try:
        if case['kind'] in ('new','visible'):
            c=case['case']
            env.reset(c['seed'],c['family'],c['roi_width_m'],episode_budget=c['episode_budget'],
                start_mode='outside',curriculum_task=c['curriculum_task'],course_two_case=c)
            row.update(bearing_bin=c['bearing_bin'],range_m=c['sensor']['range_m'],fov_deg=c['sensor']['fov_deg'],
                visibility_class=c['visibility_class'],blind_gap_m=c['blind_gap_m'])
        elif case['kind'] in ('old','simple'):
            env.reset(**case['spec'])
            if case['kind']=='simple':row['capability']=case['capability']
        elif case['kind']=='capability':
            env.reset(**case['spec'])
            from .course_two_rebalanced_scene import validate_start
            from .course_two_prelude import inside_task
            validate_start(case['spec']['course_two_case']['role'],
                env.reference.coverage_ratio(env.privileged.observed),inside_task(env))
            prelude=env.episode_metadata.get('prelude')
            row.update(capability=case['capability'],prelude=prelude,
                handover_hash=prelude['handover_hash'],qualified_handover_hash=case.get('qualified_handover_hash'),
                handover_matches_qualification=prelude['handover_hash']==case.get('qualified_handover_hash'))
        else:
            path=root/'frozen/diagnostic_scenes.py'
            if sha(path)!=read(root/'identity.json')['diagnostic_sha']:raise ValueError('diagnostic geometry changed')
            spec=importlib.util.spec_from_file_location('course_two_frozen_diagnostics',path)
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            terrain,pose,task,metadata=module.build_scene(case['scene'],config.platform)
            env.episode_budget=case['budget'];env.reset_scene(terrain,pose,task,descriptor=metadata)
            for goal in metadata.get('prelude_goals',[]):
                result=env._execute_goal(tuple(goal));snap=env._snapshot()
                native,report=env.core.observe(snap,env.adapter.velocity)
                if result.outcome!=0 or not report.available:raise RuntimeError('diagnostic prelude failed')
                observation,fine,diagnostics,stats=env._prepare_policy_state(native)
                env.observation=observation;env.native_observation=native;env.fine_observation=fine
                env.privileged=env._privileged(fine);env.report=report
                env.policy_graph_diagnostics=diagnostics;env.policy_graph_stats=stats
        initial=env.reference.coverage_ratio(env.privileged.observed)
        row.update(initial_roi_visible=initial>0,initial_coverage=initial,
            initial_completed=bool(env.completed),
            initial_inside_task=env._initial_inside_task,
            reference_id=env.reference.reference_id,reference_area_m2=env.reference.area_m2,
            sensor=env.episode_metadata['sensor'],
            start_boundary_distance_m=env.episode_metadata.get('start_boundary_distance_m'),
            initialization_turn_rad=env.episode_metadata.get('initialization_turn_rad'),
            initialization_simulation_s=env.episode_metadata.get('initialization_simulation_s'),
            policy_start_pose=env.episode_metadata.get('initial_pose'))
        validate_policy_start(case,initial)
        from .exploration_metrics import attach_trace
        trace,sample=attach_trace(env,initial)
        protocol_path=root/'comparison-protocol.json'
        if protocol_path.exists() and case['case_id'] in read(protocol_path)['visualization_cases']:
            from .exploration_visuals import VisualCapture
            visual=VisualCapture(env,root/'visualizations'/actor.stem/case['case_id']/(job_id or 'standalone'),case['case_id'],proof['actor_sha'])
            visual_hook=policy.actor.register_forward_hook(lambda module,args,output:visual.probabilities(output[0].probs))
            row['visualization_directory']=str(visual.directory.relative_to(root))
        row["decision_budget"]=env.episode_budget
        geometry=GeometryMetrics((env.plant.pose.x,env.plant.pose.y),config.goal_position_tolerance_m)
        row['trajectory'].append([env.plant.pose.x,env.plant.pose.y,env.plant.pose.yaw,initial])
        write(progress,dict(case_id=row['case_id'],actor=str(actor),decisions=0,coverage=initial,
            updated_unix=time.time(),phase='policy',slot=slot,domain_id=env.domain_id,job_id=job_id))
        while not env.terminated and not env.plant.collision and env.steps<env.episode_budget:
            inference_started=time.perf_counter_ns()
            action=policy(env.observation)
            trace.inference_ms.append((time.perf_counter_ns()-inference_started)/1e6)
            if visual is not None:visual.action(action,env.observation)
            transition=env.step(action)
            coverage=env.reference.coverage_ratio(transition.next_privileged.observed)
            sample(coverage)
            trace.decision(env.steps,coverage)
            geometry.observe((env.plant.pose.x,env.plant.pose.y),transition.parts.new_area_m2)
            row['trajectory'].append([env.plant.pose.x,env.plant.pose.y,env.plant.pose.yaw,coverage])
            row.update(decisions=env.steps,first_roi=env._first_roi,first_entry=env._first_entry)
            write(progress,dict(case_id=row['case_id'],actor=str(actor),decisions=env.steps,
                coverage=coverage,updated_unix=time.time(),phase='policy',first_roi=row['first_roi'],
                slot=slot,domain_id=env.domain_id,job_id=job_id,
                decision_budget=env.episode_budget,metric_samples=len(trace.samples),
                policy_distance_m=trace.samples[-1][0],policy_simulation_s=trace.samples[-1][1]))
            if transition.truncated:break
        row.update(completed=env.completed,collisions=int(env.plant.collision),
            final_coverage=env.reference.coverage_ratio(env.privileged.observed),distance_m=env.plant.distance_m,
            covered_area_m2=env.reference.covered_area(env.privileged.observed),
            truncation_reason=env.truncation_reason,**geometry.record())
    except Exception as exc:
        row['error']=f'{type(exc).__name__}: {exc}'
    finally:
        if trace is not None:
            row["exploration_metrics"]=trace.summary(env.reference.area_m2)
            row["coverage_curve"]=trace.samples
            row["decision_curve"]=trace.decisions
        if visual_hook is not None:visual_hook.remove()
        if visual is not None:visual.close()
        env.close()
        for signum,handler in previous.items():signal.signal(signum,handler)
    result=dict(proof=proof,row=row,wall_s=time.monotonic()-started,
        execution=dict(slot=slot,domain_id=evaluation_domain(slot),job_id=job_id),
        status='finished' if row['error'] is None else 'failed')
    import numpy as np
    values=env._mppi_compute_ms;events=env._mppi_deadline_events
    row['mppi_compute']=dict(observed_solves=len(values),deadline_missed=len(events),
        percentiles_ms=dict(zip(('p50','p95','p99'),map(float,np.percentile(values,[50,95,99])))) if values else {},
        max_ms=max(values) if values else None)
    result['mppi_deadline_events']=events
    write(output,result)
    return result
