"""有界固定输入重放；全部样本保留，候选热启动不代表内部求解器状态恢复。"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import sys
from time import perf_counter, process_time
import numpy as np
try:
    from .nav1cm.contracts import CellRect, WorldSnapshot, VehicleState, Route, LocalReference, Trajectory, WheelLimits
    from .nav1cm.maps import extract_window, build_distance_field
    from .nav1cm.local_mpcc import MpccPlanner, select_mode
except ImportError:
    from nav1cm.contracts import CellRect, WorldSnapshot, VehicleState, Route, LocalReference, Trajectory, WheelLimits
    from nav1cm.maps import extract_window, build_distance_field
    from nav1cm.local_mpcc import MpccPlanner, select_mode


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def restore_frame(frame):
    w=frame['world']
    world=WorldSnapshot(w['epoch'],tuple(CellRect(**r) for r in w['blocked']),
                        tuple(CellRect(**r) for r in w['known_free']),w['resolution_m'])
    state=VehicleState(**frame['state'])
    route=Route(**frame['route'])
    reference=LocalReference(**frame['reference'])
    grid=extract_window(world,state,frame['allow_unknown'])
    if grid.digest()!=frame['grid_digest']:
        raise ValueError('reconstructed grid digest differs from recorded input')
    raw=frame.get('recorded_candidate')
    candidate=None
    if raw:
        candidate=Trajectory(**{k:v for k,v in raw.items() if k!='digest'})
        if candidate.digest()!=raw['digest']:
            raise ValueError('recorded candidate digest mismatch')
    if reference.route_version!=route.version or reference.map_epoch!=world.epoch:
        raise ValueError('reference identity mismatch')
    return state,grid,route,reference,candidate


def restore_previous_warm(planner, previous_frame, previous, state, grid, reference):
    """保持原 warm key 身份；只恢复同地图、同路线和同模式的候选。"""
    current_key=(grid.digest(),reference.route_version,select_mode(state,reference))
    previous_key=(previous_frame['grid_digest'],previous_frame['reference']['route_version'],
                  previous.mode if previous is not None else None)
    detail={'restored':False,'previous_key':list(previous_key),'current_key':list(current_key)}
    if previous is None: reason='NO_PREVIOUS_CANDIDATE'
    elif previous_key[0]!=current_key[0]: reason='GRID_DIGEST_MISMATCH'
    elif previous_key[1]!=current_key[1]: reason='ROUTE_VERSION_MISMATCH'
    elif previous_key[2]!=current_key[2]: reason='MODE_MISMATCH'
    else:
        planner._warm=(previous_key,previous.states.copy(),previous.controls.copy())
        detail['restored']=True
        reason='RESTORED'
    detail['reason']=reason
    return detail


def summarize_frames(frames, rows):
    """纯函数：分别保留首次、稳态和全部样本；兼容旧总字段使用全部样本。"""
    summaries=[]
    for index,frame in enumerate(frames):
        selected=[r for r in rows if r['frame_index']==index]
        recorded=frame['recorded_timings_ms'].get('planner_solve')
        groups={}
        for name,group in (('first_call',[r for r in selected if r['first_call']]),
                           ('steady',[r for r in selected if not r['first_call']]),('all',selected)):
            values=[r['timings_ms']['solve'] for r in group]
            groups[name]={'count':len(values),
                'solve_ms':dict(zip(('p50','p95','p99','max'),map(float,np.percentile(values,[50,95,99,100])))) if values else None,
                'recorded_solve_peak_reached':bool(recorded is not None and max(values)>=recorded) if values else None}
        summaries.append({'frame_index':index,'source_cycle':frame['cycle'],'recorded_solve_ms':recorded,
            'solve_ms':groups['all']['solve_ms'],
            'recorded_solve_peak_reached':groups['all']['recorded_solve_peak_reached'],'groups':groups,
            'interpretation':'threshold observation only; no root cause established'})
    return summaries


def reanalyze(summary_path, replay_inputs_path, samples_path):
    """只读旧证据，返回新摘要；不运行求解器、不覆盖原文件或实际执行源码身份。"""
    summary=json.loads(Path(summary_path).read_text(encoding='utf-8-sig'))
    frames=json.loads(Path(replay_inputs_path).read_text(encoding='utf-8-sig'))
    rows=[json.loads(line) for line in Path(samples_path).read_text(encoding='utf-8-sig').splitlines() if line.strip()]
    summary['frames']=summarize_frames(frames,rows)
    summary['analysis_metadata']={'schema':'solver-timing-groups-v2','execution_rerun':False,
        'analysis_source_sha256':sha(__file__),'original_summary_sha256':sha(summary_path),
        'replay_inputs_sha256':sha(replay_inputs_path),'samples_sha256':sha(samples_path),
        'runtime_source_identity':'preserved original source_sha256/sdk_source_sha256/library_sha256'}
    return summary


def run(snapshot,build_dir,output_dir,repeats=30,distance_field_lib=None,restore_previous_candidate=False):
    if repeats<1 or repeats>1000: raise ValueError('repeats must be 1..1000')
    data=json.loads(Path(snapshot).read_text(encoding='utf-8-sig'))
    frames=list(data['frames'])
    # 独立正常运动输入，不冒充历史峰值帧；与峰值使用相同限值/时域。
    moving_state=VehicleState(0.,0.,0.,min(.1,data['limits']['linear_speed_mps']),0.)
    moving_world=WorldSnapshot(1,(),(CellRect(-400,-400,400,400),))
    moving_grid=extract_window(moving_world,moving_state,False)
    frames.append({'cycle':'synthetic-moving','input_kind':'generated-open-moving',
        'world':{'epoch':1,'blocked':[],'known_free':[{'x0':-400,'y0':-400,'x1':400,'y1':400}],'resolution_m':.01},
        'state':vars(moving_state),'allow_unknown':False,'grid_digest':moving_grid.digest(),
        'route':{'version':1,'map_epoch':1,'points':[[0.,0.],[2.,0.]],'goal':[2.,0.]},
        'reference':{'route_version':1,'map_epoch':1,'points':[[0.,0.],[2.,0.]],'s':[0.,2.],'progress_s':0.},
        'recorded_timings_ms':{}})
    restored=[restore_frame(f) for f in frames]
    out=Path(output_dir);out.mkdir(parents=True,exist_ok=False)
    # 保存原始输入字节及 SHA，不只保存结果摘要。
    (out/'input-snapshot.json').write_bytes(Path(snapshot).read_bytes())
    (out/'replay-inputs.json').write_text(json.dumps(frames,indent=2),encoding='utf-8')
    wall,cpu=perf_counter(),process_time()
    planner=MpccPlanner(WheelLimits(**data['limits']),build_dir,n=data['horizon_n'],dt=data['horizon_dt_s'])
    cold={'wall_ms':(perf_counter()-wall)*1000,'cpu_ms':(process_time()-cpu)*1000,'build_ms':planner.build_ms}
    backend=None
    if distance_field_lib:
        try: from .nav1cm.field_native import NativeDistanceField
        except ImportError: from nav1cm.field_native import NativeDistanceField
        backend=NativeDistanceField(distance_field_lib)
    rows=[]
    with (out/'samples.jsonl').open('x',encoding='utf-8') as stream:
        for index,(frame,items) in enumerate(zip(frames,restored)):
            state,grid,route,reference,_=items
            field=build_distance_field(grid,backend=backend)
            for cycle in range(repeats+1):
                # 每帧第零次是首次调用；之后是同输入正常重复，全部保留。
                restored_warm=False
                restore_detail={'restored':False,'reason':'NOT_FIRST_CALL' if cycle else ('NOT_REQUESTED' if not restore_previous_candidate else 'NO_PREVIOUS_FRAME')}
                if cycle==0:
                    planner._warm=None
                    if restore_previous_candidate and index>0:
                        restore_detail=restore_previous_warm(planner,frames[index-1],restored[index-1][-1],
                                                             state,grid,reference)
                        restored_warm=restore_detail['restored']
                wall,cpu=perf_counter(),process_time()
                result=planner.plan(state,grid,reference,distance_field=field,field_grid_digest=grid.digest())
                row={'frame_index':index,'source_cycle':frame['cycle'],'input_kind':frame.get('input_kind','recorded-exact'),'repeat':cycle,'first_call':cycle==0,
                     'previous_candidate_restored':restored_warm,'previous_candidate_restore':restore_detail,'reason':result.reason,
                     'timings_ms':result.timings_ms,'attempts':result.attempts,
                     'wall_ms':(perf_counter()-wall)*1000,'cpu_ms':(process_time()-cpu)*1000,
                     'grid_digest':grid.digest(),'candidate_digest':result.candidate.digest() if result.candidate else None,
                     'recorded_timings_ms':frame['recorded_timings_ms']}
                rows.append(row);stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
    source=Path(__file__).parent
    libraries={str(p.resolve()):sha(p) for p in planner.build_dir.rglob('*') if p.suffix in ('.so','.dll','.dylib')}
    if distance_field_lib: libraries[str(Path(distance_field_lib).resolve())]=sha(distance_field_lib)
    sdk_sources={}
    for name,module in tuple(sys.modules.items()):
        filename=getattr(module,'__file__',None)
        if name.startswith('acados_template') and filename and Path(filename).is_file():
            sdk_sources[str(Path(filename).resolve())]=sha(filename)
            for ancestor in Path(filename).resolve().parents:
                for library_name in ('libacados.so','libhpipm.so','libblasfeo.so','acados.dll','hpipm.dll','blasfeo.dll'):
                    library=ancestor/'lib'/library_name
                    if library.is_file():libraries[str(library)]=sha(library)
    summary={'cold_initialization':cold,'snapshot_sha256':sha(snapshot),'platform':platform.platform(),
             'python':platform.python_version(),'source_sha256':{str(p.relative_to(source)):sha(p) for p in source.rglob('*.py')},
             'sdk_source_sha256':sdk_sources,'library_sha256':libraries,'limits':data['limits'],'horizon_n':planner.N,'horizon_dt_s':planner.dt,
             'warm_restore_requested':restore_previous_candidate,
             'warm_restore_boundary':'candidate controls only; reset remains enabled; internal acados state not reproduced',
             'sample_count':len(rows),'frames':[],'not_run':['Humble/DDS','Orin vehicle','physical actuators']}
    summary['frames']=summarize_frames(frames,rows)
    summary['analysis_metadata']={'schema':'solver-timing-groups-v2','execution_rerun':True,'analysis_source_sha256':sha(__file__)}
    (out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False),encoding='utf-8')
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('snapshot','build-dir','output-dir'):parser.add_argument('--'+name,required=True)
    parser.add_argument('--repeats',type=int,default=30)
    parser.add_argument('--distance-field-lib')
    parser.add_argument('--restore-previous-candidate',action='store_true')
    args=parser.parse_args()
    print(json.dumps(run(**vars(args)),indent=2))
