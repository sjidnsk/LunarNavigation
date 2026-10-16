"""Prepare frozen single-run inputs and explicitly qualify geometry in bounded workers."""
from collections import deque
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from .course_two_control import read,write,sha,runtime_digest,root_lock
from .region_relation_run import make_config,qualification_eligible
from .region_relation_evaluation import TASKS,validate_manifest


def _digest(record):
    return hashlib.sha256(json.dumps(record,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def _requests(config,task,count,split):
    from .course_two_simple import fresh_scene
    from .ordered_curriculum import area_budget
    bases={'validation':2026102400000,'test':2026102500000,'train':2026102600000}
    requests=[]
    for ordinal in range(count):
        specs=[fresh_scene(config,task,ordinal,seed=bases[split]+TASKS.index(task)*1000000+ordinal+attempt*100000000,
                           split=split) for attempt in range(8)]
        if split in ('validation','test'):
            # Every group of five spans all extent bins; anchors rotate bins.
            extent=30.+((ordinal//5+ordinal%5)%5+.5)*14.
            sensor={'range_m':10.,'fov_deg':90.}
            if ordinal%5:
                expanded=ordinal-ordinal//5-1
                fov=(60.,90.,120.,180.,360.)[expanded%5]
                # Swap within each15-case cycle to keep exact10m/90° at20%.
                if expanded%15==6:fov=120.
                elif expanded%15==7:fov=90.
                sensor={'range_m':(10.,20.,30.)[expanded%3],'fov_deg':fov}
            for spec in specs:
                spec.update(extent=extent,episode_budget=area_budget(extent,config.ordered_budget_rule),
                            sensor=dict(sensor))
                if task=='moon_outside':
                    lower=10.+10.*(ordinal%4)
                    spec.update(outside_distance_min_m=lower,outside_distance_max_m=lower+10.)
        if 'sensor' in specs[0]:
            for spec in specs:spec['sensor']=dict(specs[0]['sensor'])
        case=dict(case_id=f'{split}-{task}-{ordinal:03d}',kind='simple',capability=task,
                  spec=specs[0],dataset_split=split,scene_profile=config.course_two_scene_profile)
        requests.append(dict(case=case,backup_specs=specs))
    return requests


def _pending_identity(root):
    pending=read(root/'preparation.json')
    if pending.get('root')!=str(root):raise ValueError('preparation root moved')
    if pending['runtime']!=runtime_digest() or pending['config_sha']!=sha(root/'config.json'):
        raise ValueError('preparation source/config changed')
    for name,expected in pending['input_files'].items():
        if sha(root/name)!=expected:raise ValueError('frozen preparation input changed: '+name)
    return pending


def prepare(root,protocol_file):
    from .artifacts import initialize_artifact_root
    from .cave_topology import manifest_hash,manifest_record
    from .config import config_record
    root=Path(root).absolute()
    source=Path(__file__).resolve().parents[4]
    if root.is_symlink() or source==root.resolve() or source in root.resolve().parents:
        raise ValueError('experiment output must be outside the source worktree')
    if root.exists() and any(root.iterdir()):
        if (root/'preparation.json').is_file():
            pending=_pending_identity(root)
            if pending['source_protocol_sha']!=sha(protocol_file):
                raise ValueError('protocol changed')
            return pending
        raise ValueError('refuse nonempty unmanaged preparation output')
    config=make_config(root,manifest_hash())
    initialize_artifact_root(root,config.total_output_max_bytes,
                             diagnostic_max_bytes=2*1024**3)
    for folder in ('requests','qualification','logs','actors','evaluation'):
        (root/folder).mkdir()
    protocol=read(protocol_file)
    protocol.update(status='preparing',training_started=False)
    write(root/'config.json',config_record(config));write(root/'protocol.json',protocol)
    write(root/'mother-manifest.json',manifest_record())
    input_names=['protocol.json','mother-manifest.json']
    all_requests=[]
    for split,count in (('validation',60),('test',100)):
        tasks=TASKS if len(config.course_two_scene_tasks)==3 else config.course_two_scene_tasks
        requests=[request for task in tasks for request in _requests(config,task,count,split)]
        write(root/f'{split}-requests.json',{'split':split,'requests':requests})
        input_names.append(f'{split}-requests.json');all_requests.extend(requests)
    training=_requests(config,'cave_inside',64,'train')
    write(root/'train-cave-requests.json',{'split':'train','requests':training})
    input_names.append('train-cave-requests.json');all_requests.extend(training)
    for request in all_requests:
        name='requests/'+request['case']['case_id']+'.json'
        write(root/name,request);input_names.append(name)
    pending=dict(schema='region_relation_preparation_v1',root=str(root),runtime=runtime_digest(),
                 config_sha=sha(root/'config.json'),source_protocol_sha=sha(protocol_file),
                 input_files={name:sha(root/name) for name in input_names},
                 request_ids=[r['case']['case_id'] for r in all_requests])
    pending['run_id']='region_relation_v2-'+_digest(pending)[:24]
    write(root/'preparation.json',pending)
    write(root/'state.json',dict(status='preparing',completed_blocks=0,run_id=pending['run_id']))
    return pending


def _qualified(root,case_id,pending):
    request=read(root/'requests'/f'{case_id}.json')
    output=root/'qualification'/f'{case_id}.json'
    if not output.exists():return None
    result=read(output)
    proof=dict(request=request,config_sha=pending['config_sha'],runtime=pending['runtime'])
    if result.get('status')!='passed' or result.get('proof')!=proof:
        raise ValueError('qualification cache failed or source changed: '+case_id)
    case=result['case'];index=result.get('accepted_attempt')
    if type(index) is not int or not 0<=index<8 or case.get('spec')!=request['backup_specs'][index]:
        raise ValueError('qualification did not use a predefined backup: '+case_id)
    if {k:v for k,v in case.items() if k!='spec'}!={k:v for k,v in request['case'].items() if k!='spec'}:
        raise ValueError('qualified case identity changed')
    if not qualification_eligible(result['evidence'],case['capability']):
        raise ValueError('cached qualification is not eligible')
    return result


def qualify(root,workers=8,*,popen_factory=None):
    """Explicit action, bounded DDS200..207; no qualification starts on import."""
    from .processes import _LAUNCH
    if type(workers) is not int or not 1<=workers<=8:raise ValueError('workers must be 1..8')
    root=Path(root).absolute();factory=popen_factory or subprocess.Popen
    with root_lock(root):
        pending=_pending_identity(root)
        if (root/'identity.json').exists():
            identity=read(root/'identity.json')
            if identity.get('run_id')!=pending['run_id'] or identity.get('runtime')!=pending['runtime']:
                raise ValueError('frozen experiment identity changed')
            for split,digest in identity['manifests'].items():
                if sha(root/f'{split}.json')!=digest:raise ValueError('frozen manifest changed')
            for cid,digest in identity['qualification_shas'].items():
                if sha(root/'qualification'/f'{cid}.json')!=digest:raise ValueError('frozen qualification changed')
            return identity
        todo=deque(cid for cid in pending['request_ids'] if _qualified(root,cid,pending) is None)
        active={}
        try:
            while todo or active:
                for slot in range(workers):
                    if slot in active or not todo:continue
                    cid=todo.popleft()
                    args=[sys.executable,'-m','lunar_drl_exploration.region_relation_run','qualify-one',
                          '--root',str(root),'--request',str(root/'requests'/f'{cid}.json'),
                          '--output',str(root/'qualification'/f'{cid}.json'),'--slot',str(slot)]
                    stream=(root/'logs'/f'qualify-{cid}.log').open('ab')
                    try:
                        process=factory([sys.executable,'-c',_LAUNCH,str(os.getpid()),*args],
                            stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,
                            start_new_session=True,env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1'))
                    except BaseException:
                        stream.close();raise
                    active[slot]=(process,stream,cid)
                for slot,(process,stream,cid) in list(active.items()):
                    code=process.poll()
                    if code is None:continue
                    stream.close();del active[slot]
                    if code:raise RuntimeError('qualification worker failed: '+cid)
                    if _qualified(root,cid,pending) is None:raise RuntimeError('worker produced no qualification: '+cid)
                if todo or active:time.sleep(.05)
        finally:
            deadline=time.monotonic()+30
            for process,stream,cid in active.values():
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:process.wait(timeout=max(.01,deadline-time.monotonic()))
                    except subprocess.TimeoutExpired:
                        process.kill();process.wait(timeout=5)
                stream.close()
        _pending_identity(root)
        qualified={cid:_qualified(root,cid,pending) for cid in pending['request_ids']}
        physical={}
        for cid,result in qualified.items():
            terrain_sha=result['evidence'].get('terrain_sha256')
            if not isinstance(terrain_sha,str) or len(terrain_sha)!=64 or any(c not in '0123456789abcdef' for c in terrain_sha):
                raise ValueError('missing/invalid physical terrain hash: '+cid)
            if terrain_sha in physical:
                raise ValueError('duplicate physical terrain in frozen splits: '+physical[terrain_sha]+' / '+cid)
            physical[terrain_sha]=cid
        accepted={cid:result['case'] for cid,result in qualified.items()}
        tasks=tuple(read(root/'config.json').get('course_two_scene_tasks', TASKS))
        if len(tasks)==3:tasks=TASKS
        manifests={}
        for split in ('validation','test'):
            requests=read(root/f'{split}-requests.json')['requests']
            manifests[split]={'split':split,'cases':[accepted[r['case']['case_id']] for r in requests]}
        manifests['quick']={'split':'quick','cases':[case for task in tasks
            for case in [c for c in manifests['validation']['cases'] if c['capability']==task][:20]]}
        for split,manifest in manifests.items():
            if len(tasks)==2:manifest['active_tasks']=list(tasks)
            validate_manifest(manifest);write(root/f'{split}.json',manifest)
        write(root/'train-cave-qualified.json',{'cases':[accepted[cid] for cid in pending['request_ids'] if cid.startswith('train-')]})
        identity=dict(schema='region_relation_run_identity_v1',run_id=pending['run_id'],
            runtime=pending['runtime'],config_sha=pending['config_sha'],qualified=True,
            manifests={split:sha(root/f'{split}.json') for split in manifests},
            protocol_sha=sha(root/'protocol.json'),mother_manifest_sha=sha(root/'mother-manifest.json'),
            physical_terrain_sha256_by_case={cid:result['evidence']['terrain_sha256'] for cid,result in qualified.items()},
            qualification_shas={cid:sha(root/'qualification'/f'{cid}.json') for cid in qualified})
        write(root/'identity.json',identity)
        write(root/'state.json',dict(status='ready',completed_blocks=0,run_id=pending['run_id']))
        return identity


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=('prepare','qualify'));parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--protocol',type=Path);parser.add_argument('--workers',type=int,default=8)
    args=parser.parse_args()
    if args.command=='prepare':
        if args.protocol is None:parser.error('--protocol required')
        prepare(args.root,args.protocol)
    else:qualify(args.root,args.workers)


if __name__=='__main__':main()
