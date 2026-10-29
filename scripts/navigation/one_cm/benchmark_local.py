"""固定参考独立认证基准；保留每个失败、停车、超时和原始样本。"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import platform
import resource
from time import perf_counter
import numpy as np
from nav1cm.contracts import CellRect, Route, VehicleState, WheelLimits, WorldSnapshot
from nav1cm.maps import extract_window, build_distance_field
from nav1cm.reference import extract_reference
from nav1cm.certification import certify
from nav1cm.local_mpcc import MpccPlanner
from nav1cm.seeds import brake_candidate


def cases():
    free=(CellRect(-400,-400,400,400),)
    straight=((0.,0.),(2.,0.))
    return [
        ('open',straight,(),free,False),
        ('turn-spin',((0.,0.),(0.,2.)),(),free,False),
        ('reverse',((0.,0.),(-2.,0.)),(),free,False),
        ('corridor',straight,(CellRect(-250,65,250,80),CellRect(-250,-80,250,-65)),free,False),
        ('thin-obstacle',straight,(CellRect(110,-30,111,30),),free,False),
        ('bypass',((0.,0.),(.6,-.1),(1.4,-.4),(2.,-.4)),(CellRect(110,35,111,60),),free,False),
        ('unknown-deny',straight,(),(CellRect(-65,-50,65,50),),False),
        ('unknown-allow',straight,(),(CellRect(-65,-50,65,50),),True),
        ('blocked',straight,(CellRect(60,-250,70,250),),free,False),
    ]


def percentiles(values):
    return dict(zip(('p50','p95','p99','max'),map(float,np.percentile(values,[50,95,99,100]))))


def run(build_dir, output_dir, repeats=100, geometry_lib=None):
    out=Path(output_dir); out.mkdir(parents=True,exist_ok=False)
    limits=WheelLimits(); t=perf_counter(); planner=MpccPlanner(limits,build_dir)
    cold_ms=(perf_counter()-t)*1000
    backend=None
    if geometry_lib:
        from nav1cm.geometry_native import NativeGeometry
        backend=NativeGeometry(geometry_lib)
    rows=[]
    with (out/'samples.jsonl').open('x',encoding='utf-8') as stream:
        for name,points,blocked,free,allow in cases():
            world=WorldSnapshot(1,blocked,free); state=VehicleState(0,0,0,0,0)
            route=Route(1,1,np.array(points),points[-1])
            # 第0周期同样保留，冷启动不混入稳态分位数。
            for cycle in range(repeats+1):
                start=perf_counter(); t=perf_counter()
                grid=extract_window(world,state,allow); window_ms=(perf_counter()-t)*1000
                t=perf_counter(); field=build_distance_field(grid); field_ms=(perf_counter()-t)*1000
                t=perf_counter(); reference,_=extract_reference(route,state,grid); ref_ms=(perf_counter()-t)*1000
                result=planner.plan(state,grid,reference,distance_field=field,field_grid_digest=grid.digest())
                t=perf_counter(); brake=None; certificate=None; brake_certificate=None
                if result.candidate is not None:
                    certificate=certify(result.candidate,grid,limits,reference.route_version,backend=backend)
                cert_ms=(perf_counter()-t)*1000
                brake_seed_ms=0.
                if certificate is None or not certificate.valid:
                    t=perf_counter()
                    brake=brake_candidate(state,grid,reference,limits)
                    brake_seed_ms=(perf_counter()-t)*1000
                    t=perf_counter()
                    brake_certificate=certify(brake,grid,limits,reference.route_version,backend=backend)
                    cert_ms+=(perf_counter()-t)*1000
                t=perf_counter()
                success=bool(certificate is not None and certificate.matches(result.candidate,grid,limits,reference.route_version))
                cert_ms+=(perf_counter()-t)*1000
                total_ms=(perf_counter()-start)*1000
                row={'case':name,'cycle':cycle,'steady':cycle>0,'mpcc_success':success,
                    'planner_reason':result.reason,'certificate':certificate.reason.value if certificate else None,
                    'brake_certificate':brake_certificate.reason.value if brake_certificate else None,
                    'candidate_source':result.candidate.source if result.candidate else None,
                    'attempts':result.attempts,'timings_ms':dict(result.timings_ms,window=window_ms,field=field_ms,
                    reference_extract=ref_ms,brake_seed=brake_seed_ms,certification=cert_ms,total=total_ms),'timeout':total_ms>50,
                    'rss_peak_kib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                    'grid_digest':grid.digest(),'route_version':reference.route_version,'map_epoch':grid.epoch}
                if result.candidate:
                    row['trajectory']={'times':result.candidate.times.tolist(),'states':result.candidate.states.tolist(),
                        'controls':result.candidate.controls.tolist(),'mode':result.candidate.mode,'digest':result.candidate.digest()}
                stream.write(json.dumps(row)+'\n'); stream.flush(); rows.append(row)
    steady=[r for r in rows if r['steady']]
    sources={str(p.relative_to(Path(__file__).parent)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(Path(__file__).parent.rglob('*')) if p.suffix in ('.py','.cpp','.sh')}
    summary={'cold_build_ms':cold_ms,'steady_count':len(steady),'total_count':len(rows),'platform':platform.platform(),
        'python':platform.python_version(),'source_sha256':sources,'acados_sha':'43354449192aa51557abda37d1bc17d907077354',
        'geometry_backend':'native-v1' if backend else 'python',
        'geometry_library_sha256':hashlib.sha256(Path(geometry_lib).read_bytes()).hexdigest() if backend else None,
        'limits':vars(limits),'horizon_nodes':20,'dt':.2,
        'all_stages_ms':{key:percentiles([r['timings_ms'][key] for r in steady]) for key in steady[0]['timings_ms']},
        'timeout_rate':sum(r['timeout'] for r in steady)/len(steady),'rss_peak_kib':max(r['rss_peak_kib'] for r in rows),
        'cases':{name:{'count':len(rs),'success':sum(r['mpcc_success'] for r in rs),
            'reasons':dict(Counter((r['certificate'] or r['planner_reason']) for r in rs)),
            'timeout_rate':sum(r['timeout'] for r in rs)/len(rs),'total_ms':percentiles([r['timings_ms']['total'] for r in rs])}
            for name in dict.fromkeys(r['case'] for r in steady) for rs in [[r for r in steady if r['case']==name]]},
        'not_run':['ROS 2 interface/build','DDS','rosbag','vehicle control','P3','closed-loop navigation']}
    (out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--build-dir',required=True)
    parser.add_argument('--output-dir',required=True); parser.add_argument('--repeats',type=int,default=100)
    parser.add_argument('--geometry-lib')
    args=parser.parse_args()
    if args.repeats<1: parser.error('repeats must be positive')
    run(args.build_dir,args.output_dir,args.repeats,args.geometry_lib)
