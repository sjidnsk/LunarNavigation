"""Capture normal measured decisions once, rebuild offline, freeze cross-model inputs.

Neutral primitives/typed arrays use the current trusted-local transport envelope;
no native classes, hidden reconstructed map, extra observe/poll, or 19D padding.
"""
from dataclasses import fields
import hashlib
import math
from pathlib import Path
import numpy as np
from controller import read, write, sha, digest


SCHEMA = 'measured_fixed_probe_v1'


def plain(value):
    from collections.abc import Mapping
    from lunar_drl_exploration.contracts import freeze_array
    if isinstance(value, np.ndarray): return freeze_array(value)
    if hasattr(value, '__dataclass_fields__'): return {f.name:plain(getattr(value,f.name)) for f in fields(value)}
    if isinstance(value, Mapping): return {k:plain(v) for k,v in value.items()}
    if isinstance(value, tuple): return tuple(plain(v) for v in value)
    if isinstance(value, list): return [plain(v) for v in value]
    if isinstance(value, np.generic): return value.item()
    return value


def content_hash(value):
    """Versioned structural hash preserving typed array bytes (including NaN payloads)."""
    h=hashlib.sha256(SCHEMA.encode())
    def visit(v):
        if isinstance(v,np.ndarray):
            h.update(b'array');h.update(v.dtype.str.encode());h.update(str(v.shape).encode());h.update(v.tobytes(order='C'))
        elif isinstance(v,dict):
            h.update(b'dict')
            for k in sorted(v,key=repr):visit(k);visit(v[k])
        elif isinstance(v,(tuple,list)):
            h.update(type(v).__name__.encode());h.update(str(len(v)).encode())
            for item in v:visit(item)
        elif isinstance(v,float) and not math.isfinite(v):h.update(repr(v).encode())
        else:
            import json
            h.update(type(v).__name__.encode());h.update(json.dumps(v,sort_keys=True,allow_nan=False).encode())
    visit(value)
    return h.hexdigest()


def capture_before_observe(core,snapshot,velocity):
    return dict(schema=SCHEMA, snapshot=plain(snapshot), velocity=tuple(velocity),
        task=plain(core.task), sensor=plain(core.sensor), graph_config=plain(core.config),
        platform=plain(core.builder.platform), pre_builder=dict(base_points=plain(core.builder._base_points),
        epoch=plain(core.builder._epoch), lattice_origin=plain(core.builder._lattice_origin)))


def finish_observe(pending,core,observation,report):
    if not report.available: raise ValueError('unavailable measured probe')
    if (observation.epoch != pending['snapshot']['epoch'] or observation.revision != report.revision
            or report.revision != pending['snapshot']['revision'] or core.frontiers.revision != report.revision):
        raise ValueError('probe decision revision mismatch')
    records=np.asarray(core.history._records,dtype=np.float64).reshape(-1,3)
    pending.update(history=dict(xy=plain(records[:,:2]), heading=plain(records[:,2].astype(np.int64)),
        tolerance_m=core.history.tolerance_m), workspace=plain(core.analyzer.workspace), report=plain(report),
        coverage=dict(bounds=plain(core.coverage.bounds), coordinate_identity=plain(core.coverage.coordinate_identity),
            identity=plain(core.coverage.identity), revision=core.coverage.revision,
            cell_offset=plain(core.coverage.cell_offset)), frontiers=plain(core.frontiers))
    return pending


class FrozenCoverage:
    def __init__(self,bundle):
        self.__dict__.update(bundle['coverage'])
        self.workspace=bundle['workspace']
    def mask(self,bounds):
        x0,y0,x1,y1=bounds
        result=np.zeros((y1-y0,x1-x0),bool)
        wx0,wy0,wx1,wy1=self.workspace['bounds']
        ax,ay,bx,by=max(x0,wx0),max(y0,wy0),min(x1,wx1),min(y1,wy1)
        if bx>ax and by>ay:result[ay-y0:by-y0,ax-x0:bx-x0]=self.workspace['known'][ay-wy0:by-wy0,ax-wx0:bx-wx0]
        return result


def rebuild_fixed_observation(bundle):
    from lunar_drl_exploration.contracts import Pose, TaskSpec, SensorSpec, TaskReport
    from lunar_drl_exploration.maps import PolicyMapTile, PolicyMapSnapshot
    from lunar_drl_exploration.config import GraphConfig, PlatformConfig
    from lunar_drl_exploration.graph import GraphBuilder
    from lunar_drl_exploration.history import DirectionHistory
    from lunar_drl_exploration.task_analysis import measured_workspace
    from lunar_drl_exploration.frontiers import build_frontiers
    if bundle['schema'] != SCHEMA: raise ValueError('measured probe schema')
    values=dict(bundle['snapshot']);values['pose']=Pose(**values['pose'])
    values['tiles']={key:PolicyMapTile(**tile) for key,tile in values['tiles'].items()}
    snapshot=PolicyMapSnapshot(**values)
    task,sensor=TaskSpec(**bundle['task']),SensorSpec(**bundle['sensor'])
    graph=dict(bundle['graph_config']);graph['platform']=PlatformConfig(**bundle['platform'])
    history=DirectionHistory(bundle['history']['tolerance_m'])
    history._records=[(float(x),float(y),int(heading)) for (x,y),heading in zip(bundle['history']['xy'],bundle['history']['heading'])]
    workspace=measured_workspace(snapshot,task,sensor,FrozenCoverage(bundle))
    if content_hash(plain(workspace)) != content_hash(bundle['workspace']): raise ValueError('measured workspace/K mismatch')
    frontiers=build_frontiers(snapshot,workspace,task)
    if content_hash(plain(frontiers)) != content_hash(bundle['frontiers']): raise ValueError('frontier rebuild mismatch')
    builder=GraphBuilder(GraphConfig(**graph));builder.workspace=workspace
    builder._base_points=np.array(bundle['pre_builder']['base_points'],copy=True)
    builder._epoch=bundle['pre_builder']['epoch'];builder._lattice_origin=bundle['pre_builder']['lattice_origin']
    observation = builder.build(snapshot,TaskReport(**bundle['report']),history,task,sensor,bundle['velocity'],frontiers=frontiers)
    policy_graph = bundle.get('policy_graph')
    if policy_graph is not None:
        if (set(policy_graph) != {'mode', 'max_remote_positions'}
                or policy_graph['mode'] != 'a2'):
            raise ValueError('unsupported fixed probe policy graph')
        from lunar_drl_exploration.policy_graph import prepare_policy_graph
        observation = prepare_policy_graph(observation,
            max_remote_positions=policy_graph['max_remote_positions']).observation
    return observation


def validate_exact_alignment(rebuilt,captured,privileged=None,scenes=None):
    from lunar_drl_exploration.feature_schema import require_observation_identity
    require_observation_identity(rebuilt);require_observation_identity(captured)
    if content_hash(plain(rebuilt)) != content_hash(plain(captured)):
        raise ValueError('rebuilt observation/node/action/features differ from captured decision')
    if privileged is not None:
        actions=privileged.actions
        if actions is None or len(actions.action_positions)!=len(captured.goals):
            raise ValueError('probe privileged action count')
        if (not np.array_equal(actions.positions[actions.action_positions],captured.goals[:,:2])
                or not np.array_equal(actions.action_yaws,captured.goals[:,2])):
            raise ValueError('probe privileged goals differ')
        scene=scenes[privileged.scene_id]
        if len(privileged.observed) != len(scene.packed_reference): raise ValueError('probe observed/reference shape')
        if np.any(actions.support_indices<0) or np.any(actions.support_indices>=len(scene.positions)):
            raise ValueError('probe support indices')



def save_probe(root,bundle,observation,privileged,scenes,provenance):
    from lunar_drl_exploration.replay import dumps_transport
    from lunar_drl_exploration.artifacts import atomic_write
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    if (root/'index.json').exists(): raise ValueError('fixed probe set is frozen')
    rebuilt=rebuild_fixed_observation(bundle)
    validate_exact_alignment(rebuilt,observation,privileged,scenes)
    inputs=dict(observation=observation,privileged=privileged,scenes=scenes)
    probe_hash=content_hash(dict(measured=bundle,inputs=plain(inputs)))
    name='probe-'+probe_hash
    for suffix,value in (('measured',bundle),('inputs',inputs)):
        path=root/f'{name}-{suffix}.bin'
        payload=dumps_transport(value)
        atomic_write(path,lambda stream:stream.write(payload))
    from lunar_drl_exploration.feature_schema import frontier_semantics
    provenance=dict(provenance,frontier_semantics=frontier_semantics())
    record=dict(schema=SCHEMA,probe_hash=probe_hash,provenance=provenance,
        measured=name+'-measured.bin',inputs=name+'-inputs.bin',
        measured_sha=sha(root/f'{name}-measured.bin'),inputs_sha=sha(root/f'{name}-inputs.bin'))
    write(root/f'{name}.json',record)
    return record


def freeze_index(root):
    root=Path(root)
    if (root/'index.json').exists(): return read(root/'index.json')
    entries=[read(path) for path in sorted(root.glob('probe-*.json'))]
    if not entries: raise ValueError('no exactly rebuilt measured probes captured')
    value=dict(schema=SCHEMA,selection='first worker: up to two actual inside/boundary/outside decisions per category, maximum six; frozen after warmup',
               missing_categories=sorted({'inside','boundary','outside'}-{r['provenance'].get('category') for r in entries}),
               probes=entries,probe_set_hash=digest([r['probe_hash'] for r in entries]))
    write(root/'index.json',value)
    return value


def load_probes(root):
    from lunar_drl_exploration.replay import loads_transport
    root=Path(root);index=read(root/'index.json')
    if index['probe_set_hash'] != digest([r['probe_hash'] for r in index['probes']]): raise ValueError('probe set hash')
    result=[]
    for row in index['probes']:
        if sha(root/row['inputs']) != row['inputs_sha'] or sha(root/row['measured']) != row['measured_sha']:
            raise ValueError('fixed probe file changed')
        measured=loads_transport((root/row['measured']).read_bytes())
        inputs=loads_transport((root/row['inputs']).read_bytes())
        if content_hash(dict(measured=measured,inputs=plain(inputs))) != row['probe_hash']: raise ValueError('probe content hash')
        validate_exact_alignment(inputs['observation'],inputs['observation'],inputs['privileged'],inputs['scenes'])
        result.append((row,inputs))
    return index,result


def state_category(observation,sensor_range,world_polygon):
    from lunar_drl_exploration.graph import _inside
    point=observation.positions[observation.current_index].astype(float)
    polygon=np.asarray(world_polygon,dtype=float)
    if not _inside(point[None,:],polygon)[0]:return 'outside'
    edges=np.roll(polygon,-1,axis=0)-polygon
    length=(edges*edges).sum(axis=1)
    ratio=np.divide(((point-polygon)*edges).sum(axis=1),length,out=np.zeros_like(length),where=length>0)
    closest=polygon+np.clip(ratio,0,1)[:,None]*edges
    return 'boundary' if np.linalg.norm(closest-point,axis=1).min()<=sensor_range else 'inside'


class ProbeCapture:
    """Installed only in a new-overlay evaluation/smoke process; no extra sensing."""
    def __init__(self,root,provenance):
        self.root=Path(root);self.provenance=provenance;self.pending={};self.serial=0
        self.enabled=True;self.original=None
    def install(self):
        from lunar_drl_exploration.decision import DecisionCore
        self.original=DecisionCore.observe
        def observed(core,snapshot,velocity=(0.,0.)):
            if not self.enabled:return self.original(core,snapshot,velocity)
            before=capture_before_observe(core,snapshot,velocity)
            observation,report=self.original(core,snapshot,velocity)
            self.serial+=1
            self.pending[id(core)]=(observation,finish_observe(before,core,observation,report)) if report.available else None
            return observation,report
        DecisionCore.observe=observed
    def bind(self,env,observation,privileged,decision_index):
        pending=self.pending.pop(id(env.core),None)
        if pending is None or len(observation.goals)<2:return None
        captured,bundle=pending
        if getattr(env.config, 'policy_graph_mode', 'fine') == 'a2':
            validate_exact_alignment(env.native_observation, captured)
            bundle['policy_graph'] = dict(mode='a2',
                max_remote_positions=env.config.remote_frontier_positions)
            from lunar_drl_exploration.policy_graph import prepare_policy_graph
            validate_exact_alignment(observation, prepare_policy_graph(captured,
                max_remote_positions=env.config.remote_frontier_positions).observation)
        else:
            validate_exact_alignment(observation,captured)
        if (self.root/'index.json').exists():return None
        category=state_category(observation,bundle['sensor']['range_m'],bundle['task']['polygon'])
        entries=[read(path) for path in self.root.glob('probe-*.json')]
        if len(entries)>=6 or sum(r['provenance'].get('category')==category for r in entries)>=2:return None
        provenance=dict(self.provenance,episode_metadata=plain(env.episode_metadata),
            initial_pose=getattr(env,'probe_initial_pose',None),category=category,decision_index=decision_index,capture_serial=self.serial,terrain_id=env.terrain.terrain_id,
            reference_id=env.reference.reference_id,epoch=observation.epoch,revision=observation.revision,
            goal_hash=content_hash(observation.goals),history_count=len(bundle['history']['xy']),
            processed_stamp_ns=getattr(env.adapter,'processed_stamp_ns',None),
            latest_sensor_stamp_ns=getattr(env,'_latest_sensor_ns',None),
            sensor_frames=env.progress().get('sensor_frames'))
        return save_probe(self.root,bundle,observation,privileged,dict(env.static_scenes),provenance)
    def close(self):
        if self.original is not None:
            from lunar_drl_exploration.decision import DecisionCore
            DecisionCore.observe=self.original
            self.original=None
        self.pending.clear()
