"""Prepare, seal and resume a validation-only frozen checkpoint comparison."""
import argparse
from importlib import import_module
from collections import Counter
import copy
import math
import os
from pathlib import Path
import shutil
import signal
import time
import numpy as np
from .course_two_control import read, write, sha, runtime_digest, root_lock, process_ticks
from .evaluation_navigation import load_evaluation_navigation, validate_evaluation_navigation
from .exploration_metrics import curve_auc
from .region_relation_evaluation import validate_manifest

PACKAGES=('lunar_incremental_navigation_core','lunar_incremental_navigation_ros',
          'lunar_pure_wheeled_controller','lunar_planning_msgs')


def successful(row):
    coverage=row.get('final_coverage')
    return (row.get('completed') is True and row.get('collisions')==0 and not row.get('error')
            and type(coverage) in (int,float) and math.isfinite(coverage) and .99<=coverage<=1.)


def finite_summary(values):
    values=[float(v) for v in values if v is not None and math.isfinite(v)]
    return dict(available=len(values),median=float(np.median(values)) if values else None,
                p90=float(np.percentile(values,90)) if values else None,
                mean=float(np.mean(values)) if values else None)


def prepare(source, actors, root, protocol_path, navigation_path):
    source,actors,root=map(Path,(source,actors,root))
    if root.exists():raise FileExistsError('new comparison root required: '+str(root))
    manifest=copy.deepcopy(read(source/'validation.json'))
    if manifest.get('split')!='validation':raise ValueError('validation split required')
    cases=validate_manifest(manifest)
    protocol=read(protocol_path);navigation=validate_evaluation_navigation(read(navigation_path))
    if len(cases)!=120 or protocol['episode_budget']!=512:raise ValueError('full120 validation and512 budget required')
    order=protocol['checkpoint_order']
    if sorted(order)!=list(range(1,25)) or order[0]!=20:raise ValueError('24 checkpoints with frozen reference block20 required')
    ids={c['case_id'] for c in cases}
    selected=protocol['visualization_cases']
    if len(selected)!=4 or len(set(selected))!=4 or not set(selected)<=ids:
        raise ValueError('four distinct validation replays required')
    counts=Counter(next(c['capability'] for c in cases if c['case_id']==cid) for cid in selected)
    if counts!=Counter(moon_outside=2,cave_inside=2):raise ValueError('two replays per task required')
    if navigation['controller_backend']!=protocol['controller_backend']:
        raise ValueError('protocol and explicit controller disagree')
    actor_records=[dict(block=b,file=f'block-{b:02d}.pt',sha256=sha(actors/f'block-{b:02d}.pt')) for b in range(1,25)]
    protocol['actors']=actor_records
    protocol['budget_counts_prior']=dict(Counter(str(c['spec']['episode_budget']) for c in cases))
    for case in cases:case['spec']['episode_budget']=512
    config=read(source/'config.json')
    config.update(artifact_root=str(root.resolve()),output_dir=str(root.resolve()/'run'),curriculum_budgets=[512]*3)
    config.setdefault('ordered_budget_rule',{}).update(min_decisions=512,max_decisions=512)
    root.mkdir(parents=True);(root/'actors').mkdir();(root/'logs').mkdir()
    for actor in actor_records:shutil.copy2(actors/actor['file'],root/'actors'/actor['file'])
    for name,value in [('config.json',config),('validation.json',manifest),
                       ('comparison-protocol.json',protocol),('evaluation-navigation.json',navigation),
                       ('evaluation-settings.json',dict(workers=8,error_policy='count_failure')),
                       ('state.json',dict(status='paused',phase='validation',block=20,evaluation_done=0,
                            evaluation_total=120,evaluation_workers=[],campaign_completed_checkpoints=0))]:
        write(root/name,value)


def check_inputs(root):
    if (root/'test.json').exists():raise ValueError('test selection forbidden')
    cases=validate_manifest(read(root/'validation.json'))
    if len(cases)!=120 or any(c['spec']['episode_budget']!=512 for c in cases):
        raise ValueError('full validation with512 decisions required')
    navigation=load_evaluation_navigation(root);protocol=read(root/'comparison-protocol.json')
    if navigation['controller_backend']!=protocol['controller_backend']:
        raise ValueError('protocol and explicit controller disagree')
    for actor in protocol['actors']:
        if sha(root/'actors'/actor['file'])!=actor['sha256']:raise ValueError('actor changed')


def _require_frozen_environment(stage):
    from ament_index_python.packages import get_package_prefix
    if Path(__file__).resolve().parent!=stage/'source/lunar_drl_exploration':
        raise ValueError('Python package is outside the frozen snapshot')
    for package in PACKAGES:
        if Path(get_package_prefix(package)).resolve()!=stage/'install'/package:
            raise ValueError('native package outside frozen snapshot: '+package)
    modules=('lunar_drl_exploration','lunar_drl_exploration.course_two_control',
             'lunar_planning_msgs.srv._get_policy_map','lunar_planning_msgs.msg._path_reference',
             'lunar_planning_msgs.msg._tracking_status','lunar_planning_msgs.msg._policy_map_tile',
             'lunar_planning_msgs.action._navigate_to_pose','lunar_pure_wheeled_controller.node',
             'lunar_pure_wheeled_controller.mppi','lunar_pure_wheeled_controller.mppi_map',
             'lunar_pure_wheeled_controller.mppi_path_follower')
    for name in modules:
        path=Path(import_module(name).__file__).resolve()
        prefix=stage/('source' if name.startswith('lunar_drl_exploration') else 'install')/name.split('.')[0]
        if not path.is_file() or not path.is_relative_to(prefix):
            raise ValueError('Python interface/controller import outside frozen installation: '+name)


def seal(root,stage):
    root,stage=Path(root).resolve(),Path(stage).resolve()
    if (root/'identity.json').exists():raise FileExistsError('refusing to reseal an existing identity')
    if root.is_relative_to(stage) or stage.is_relative_to(root):raise ValueError('separate snapshot and output directories required')
    check_inputs(root)
    _require_frozen_environment(stage)
    identity=dict(evaluation_only=True,runtime=runtime_digest(),source_snapshot=str(stage),
                  campaign_sha=sha(Path(__file__)),decision_budget=512)
    for name in ('config','validation','comparison-protocol','evaluation-navigation','evaluation-settings'):
        identity[name+'_sha']=sha(root/(name+'.json'))
    files={str(p.relative_to(stage)):sha(p) for p in stage.rglob('*')
           if p.is_file() and '__pycache__' not in p.parts and 'ros-logs' not in p.parts}
    write(root/'identity.json',identity)
    write(root/'sealed-runtime.json',dict(identity=identity,files_sha256=files))


def verify(root):
    check_inputs(root);identity=read(root/'identity.json');record=read(root/'sealed-runtime.json')
    if identity!=record['identity']:raise ValueError('sealed identity changed')
    _require_frozen_environment(Path(identity['source_snapshot']).resolve())
    if runtime_digest()!=identity['runtime'] or sha(Path(__file__))!=identity['campaign_sha']:
        raise ValueError('frozen source identity changed')
    for name in ('config','validation','comparison-protocol','evaluation-navigation','evaluation-settings'):
        if sha(root/(name+'.json'))!=identity[name+'_sha']:raise ValueError('input changed: '+name)
    stage=Path(identity['source_snapshot'])
    for name,digest in record['files_sha256'].items():
        if sha(stage/name)!=digest:raise ValueError('snapshot changed: '+name)


def summarize_checkpoint(root,block,result):
    rows=result['rows'];budget_path=root/'physical-auc-budgets.json'
    if block==20:
        budgets={r['case_id']:dict(distance_m=r.get('exploration_metrics',{}).get('policy_distance_m'),
                    simulation_s=r.get('exploration_metrics',{}).get('policy_simulation_s')) for r in rows}
        if budget_path.exists() and read(budget_path)!=budgets:raise ValueError('reference physical budgets changed')
        write(budget_path,budgets)
    budgets=read(budget_path)
    out=dict(block=block,actor_sha=sha(root/'actors'/f'block-{block:02d}.pt'),gate=result['gate'],
             collision_episodes=sum(bool(r.get('collisions')) for r in rows),
             errors=sum(bool(r.get('error')) for r in rows),by_task={},visualizations=[])
    for task in ('moon_outside','cave_inside'):
        group=[r for r in rows if r['capability']==task]
        physical_auc={0:[],1:[]};inference=[]
        for row in group:
            metrics=row.get('exploration_metrics',{});curve=row.get('coverage_curve')
            for field,index in [('distance_m',0),('simulation_s',1)]:
                limit=budgets.get(row['case_id'],{}).get(field)
                if curve and limit is not None and limit>0:
                    physical_auc[index].append(curve_auc(curve,index,limit))
            inference.extend(metrics.get('inference_ms',[]))
            if row.get('visualization_directory'):
                out['visualizations'].append(dict(case_id=row['case_id'],directory=row['visualization_directory'],
                    completed=row.get('completed'),error=row.get('error')))
        measures=dict(episodes=len(group),successes=sum(successful(r) for r in group),
                      common_distance_auc=finite_summary(physical_auc[0]),
                      common_time_auc=finite_summary(physical_auc[1]),inference_ms=finite_summary(inference))
        for q in ('90','95','99'):
            for field in ('distance_m','simulation_s'):
                measures[f'{field}_to_{q}']=finite_summary([
                    (r.get('exploration_metrics',{}).get('thresholds',{}).get(q) or {}).get(field) for r in group])
        for field in ('max_no_gain_distance_m','max_no_gain_simulation_s','new_area_per_meter_m2',
                      'collection_wall_s','decision_auc_512'):
            measures[field]=finite_summary([r.get('exploration_metrics',{}).get(field) for r in group])
        out['by_task'][task]=measures
    values=[t['common_time_auc']['mean'] for t in out['by_task'].values()]
    out['macro_time_auc']=float(np.mean(values)) if all(v is not None for v in values) else None
    return out


def select(root,summaries):
    safe=[s for s in summaries if s['collision_episodes']==0 and s['errors']==0]
    if not safe:return None,{}
    best=max(s['gate']['macro_success_rate'] for s in safe)
    finalists=[s for s in safe if s['gate']['macro_success_rate']==best]
    records={};common=None;paired={}
    for candidate in finalists:
        block=candidate['block'];directory=root/'evaluation'/f'validation-block-{block:02d}-budget512'
        records[block]={}
        for p in directory.glob('validation-*.json'):
            row=read(p)['row'];records[block][row['case_id']]=row
        successful_ids={cid for cid,row in records[block].items() if successful(row)}
        common=successful_ids if common is None else common&successful_ids
    for candidate in finalists:
        thresholds=[records[candidate['block']][cid].get('exploration_metrics',{}).get('thresholds',{}).get('99')
                    for cid in common or []]
        paired[candidate['block']]=dict(common_scenes=len(common or []),
            L99=finite_summary([t['distance_m'] for t in thresholds if t is not None]),
            T99=finite_summary([t['simulation_s'] for t in thresholds if t is not None]))
    def ranking(candidate):
        pair=paired[candidate['block']]
        return (candidate['macro_time_auc'] if candidate['macro_time_auc'] is not None else -1,
                -pair['L99']['mean'] if pair['L99']['mean'] is not None else -math.inf,
                -pair['T99']['mean'] if pair['T99']['mean'] is not None else -math.inf)
    return max(finalists,key=ranking)['block'],paired


def run(root):
    from .region_relation_run import Run
    root=Path(root).resolve();verify(root);protocol=read(root/'comparison-protocol.json')
    with root_lock(root):
        controller=Run(root);summaries=[]
        for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,controller.interrupt)
        write(root/'owner.json',dict(root=str(root),pid=os.getpid(),start_ticks=process_ticks(os.getpid()),
                                    mode='validation_comparison_only'))
        try:
            for block in protocol['checkpoint_order']:
                if controller.stopping:break
                controller.save(status='running',block=block,controller_pid=os.getpid(),error=None,
                    campaign_completed_checkpoints=len(summaries),campaign_total_checkpoints=24,campaign_total_episodes=2880)
                result=controller.screen(root/'actors'/f'block-{block:02d}.pt','validation',f'validation-block-{block:02d}-budget512')
                if controller.stopping:break
                if result is None:raise RuntimeError('validation returned no result without a pause')
                summary=summarize_checkpoint(root,block,result);summaries.append(summary)
                write(root/f'block-{block:02d}-metrics.json',summary)
                write(root/'comparison-progress.json',dict(completed_checkpoints=len(summaries),total_checkpoints=24,
                    completed_episodes=120*len(summaries),summaries=summaries))
                print(f'checkpoint {block:02d} finished ({len(summaries)}/24)',flush=True)
            if controller.stopping:
                controller.save(status='paused',evaluation_workers=[],child_pid=None)
                return
            if len(summaries)!=24:raise RuntimeError('cannot complete a partial campaign')
            selected,paired=select(root,summaries)
            write(root/'selection.json',dict(status='validation_only_candidate',selected_block=selected,
                criteria=protocol['ranking'],paired_finalists=paired,summaries=summaries,
                note='No independent test or deployment performed.'))
            controller.save(status='complete',phase='complete',campaign_completed_checkpoints=24,
                campaign_completed_episodes=2880,selected_validation_block=selected,completed_unix=time.time(),
                evaluation_workers=[],child_pid=None)
        except BaseException as exc:
            controller.interrupt();controller.save(status='needs_attention',error=f'{type(exc).__name__}: {exc}')
            raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('prepare');p.add_argument('--source-root',type=Path,required=True)
    p.add_argument('--actors-dir',type=Path,required=True);p.add_argument('--protocol',type=Path,required=True)
    p.add_argument('--navigation',type=Path,required=True);p.add_argument('--root',type=Path,required=True)
    p=sub.add_parser('seal');p.add_argument('--root',type=Path,required=True);p.add_argument('--stage',type=Path,required=True)
    for name in ('verify','run'):
        p=sub.add_parser(name);p.add_argument('--root',type=Path,required=True)
    args=parser.parse_args()
    if args.command=='prepare':prepare(args.source_root,args.actors_dir,args.root,args.protocol,args.navigation)
    elif args.command=='seal':seal(args.root,args.stage)
    elif args.command=='verify':verify(args.root)
    else:run(args.root)
