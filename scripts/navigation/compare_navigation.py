#!/usr/bin/env python3
"""Serial matched comparison, with isolated per-algorithm child RSS/caches.

Build lunar_adaptive_navigation_benchmark with Release first. This script only
writes its explicit output directory and never starts/stops training or a robot.
"""
import argparse,hashlib,json,os,platform,statistics,subprocess,time
from pathlib import Path

PATTERNS=['open','long_wall','rocks','risk_band','rough_cost','u_cave','maze',
          'dead_ends','winding_cave','unknown_shortcut','enclosed','diagonal_trap']
def percentile(values,p):
    if not values:return None
    v=sorted(values);k=(len(v)-1)*p;a=int(k);return v[a]+(v[min(a+1,len(v)-1)]-v[a])*(k-a)
def summarize(rows):
    groups={}
    for r in rows:
        scale='large' if r['extent_m']>=500 else 'medium' if r['extent_m']>=100 else 'small'
        key=(r['algorithm'],r['budget_scope'],scale,r['phase'])
        groups.setdefault(key,[]).append(r)
    out=[]
    for key,data in sorted(groups.items()):
        safe=[r for r in data if r['status']=='AVAILABLE' and r['safe']]
        out.append(dict(zip(['algorithm','budget_scope','scale','phase'],key),
          count=len(data),safe_routes=len(safe),unsafe_available=sum(r['status']=='AVAILABLE' and not r['safe'] for r in data),
          timeouts=sum(r['status']=='TIMEOUT' for r in data),no_route=sum(r['status']=='NO_ROUTE' for r in data),
          median_ms=percentile([r['pipeline_total_ms'] for r in data],.5),
          p95_ms=percentile([r['pipeline_total_ms'] for r in data],.95),
          max_ms=max(r['pipeline_total_ms'] for r in data),
          search_median_ms=percentile([r['search_ms'] for r in data],.5),
          max_peak_rss_mib=max(r['peak_rss_kib'] for r in data)/1024,
          max_cache_mib=max(r['cache_bytes'] for r in data)/(1024**2)))
    return out

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--binary',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--quick',action='store_true');ap.add_argument('--cpu',type=int)
    ap.add_argument('--resume',action='store_true');args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    if args.cpu is not None:os.sched_setaffinity(0,{args.cpu})
    scales=[(128,.2)] if args.quick else [(128,.2),(512,.2),(2048,.5)]
    seeds=[13] if args.quick else [13,29,47,83]
    cases=[(p,n,r,s) for p in PATTERNS for n,r in scales for s in seeds]
    if not args.quick:cases += [(p,4096,.5,s) for p in ['open','long_wall','rocks'] for s in [13,47]]
    binary=args.binary.resolve();binary_sha=hashlib.sha256(binary.read_bytes()).hexdigest()
    library=binary.parent/'liblunar_incremental_navigation_core.so'
    if not library.exists():raise RuntimeError('freeze the linked core library beside the benchmark binary first')
    library_sha=hashlib.sha256(library.read_bytes()).hexdigest()
    manifest=dict(started=time.time(),binary=str(binary),binary_sha=binary_sha,
       library=str(library),library_sha=library_sha,cases=cases,
       budget_ms=500,budget_scopes=['pipeline','search'],iterations=3,
       host=platform.platform(),cpu_affinity=sorted(os.sched_getaffinity(0)),
       notes=['same immutable snapshots and platform for all algorithms',
       'pipeline scope charges map preprocessing against the same 500ms total',
       'search scope measures 500ms planning after map preparation; cold map cost still reported',
       'reference is an 8-neighbor grid A-star, not a continuous-space optimum',
       'peak RSS includes common immutable input; algorithm caches isolated by fork',
       'all timings local host; no Orin or real vehicle inference'])
    manifest_path=args.output/'manifest.json'
    if args.resume and manifest_path.exists():
        old=json.loads(manifest_path.read_text())
        assert (old['binary_sha'],old.get('library_sha'))==(binary_sha,library_sha),'binary/library changed; use a new output directory'
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
    rows=[]
    for index,case in enumerate(cases,1):
        p,n,r,s=case;path=args.output/f'{p}-{n}-{r}-{s}.jsonl'
        if not(args.resume and path.exists()):
            env=dict(os.environ,LD_LIBRARY_PATH=str(binary.parent)+':'+os.environ.get('LD_LIBRARY_PATH',''))
            result=subprocess.run([str(binary),p,str(n),str(r),str(s),'3','500'],capture_output=True,text=True,timeout=180,env=env)
            (args.output/f'{path.stem}.stderr.log').write_text(result.stderr)
            if result.returncode:raise RuntimeError(f'{case}: {result.returncode}: {result.stderr[-1000:]}')
            path.write_text(result.stdout)
        current=[json.loads(line) for line in path.read_text().splitlines()]
        assert len(current)==42,(case,len(current))
        # Every paired comparison must use the same base map and request.
        matched={}
        for row in current:
            key=(row['budget_scope'],row['phase'],row['iteration'])
            signature=tuple(row[k] for k in ['map_hash','fine_revision','start_x','start_y','goal_x','goal_y'])
            assert key not in matched or matched[key]==signature,(case,key)
            matched[key]=signature
        rows.extend(current)
        (args.output/'progress.json').write_text(json.dumps(dict(done=index,total=len(cases),last=case,rows=len(rows)))+'\n')
        print(f'{index}/{len(cases)} {case}',flush=True)
    (args.output/'results.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    summary=summarize(rows);(args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    unsafe=[r for r in rows if r['algorithm']=='adaptive' and r['status']=='AVAILABLE' and not r['safe']]
    assert hashlib.sha256(binary.read_bytes()).hexdigest()==binary_sha,'binary changed during experiment'
    assert hashlib.sha256(library.read_bytes()).hexdigest()==library_sha,'library changed during experiment'
    manifest.update(finished=time.time(),row_count=len(rows),adaptive_unsafe_count=len(unsafe),runtime_unchanged=True)
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(rows=len(rows),adaptive_unsafe_count=len(unsafe))),flush=True)
    if unsafe:raise RuntimeError(f'{len(unsafe)} adaptive routes failed independent validation')
if __name__=='__main__':main()
