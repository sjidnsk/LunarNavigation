"""Lossless episode replay capture; immutable observation/control inputs."""
from pathlib import Path
import gzip
import json
import math
import threading
import time
import zipfile
import numpy as np

DELTA_DTYPE=np.dtype([('i','<u4'),('o','u1'),('b','u1'),('m','u1')])


def encode_delta(index,observed,intrinsic,navigation):
    data=np.empty(len(index),DELTA_DTYPE)
    for name,value in zip(data.dtype.names,(index,observed,intrinsic,navigation)):
        data[name]=value
    return data.tobytes()


def decode_delta(data):
    return np.frombuffer(data,DELTA_DTYPE)


def json_default(value):
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,np.ndarray):return value.tolist()
    raise TypeError(f'unsupported replay value: {type(value).__name__}')


class VisualCapture:
    def __init__(self,env,directory,case_id,actor_sha):
        self.env=env;self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=False)
        self.lock=threading.RLock();self.closed=False;self.wall_s=0.;self.pose_count=0;self.map_count=0;self.action_count=0
        self.last_probs=None;self.tiles={};self.map_events=[]
        self.distance0=env.plant.distance_m;self.time0=env.plant.simulation_s
        self.files={name:gzip.open(self.directory/(name+'.jsonl.gz'),'wt',compresslevel=1)
                    for name in ('trajectory','decisions','references')}
        self.maps=zipfile.ZipFile(self.directory/'maps.zip','w',zipfile.ZIP_DEFLATED,compresslevel=1)
        self.meta=dict(schema='exploration_replay_v1',case_id=case_id,actor_sha=actor_sha,
            origin=list(env.terrain.origin),resolution_m=env.terrain.resolution_m,
            shape=list(env.terrain.shape),frame='map',sensor=env.episode_metadata['sensor'],
            decision_budget=env.episode_budget,policy_start_pose=list(vars(env.plant.pose).values()),
            map_delta_dtype=[['i','uint32_le'],['o','uint8_observed_union'],['b','uint8_intrinsic'],['m','uint8_navigation']],
            note='Actual plant integration poses, native map deltas, selected actions/probabilities and published references. Truth arrays are context only and must be hidden in unknown areas by the viewer.')
        np.savez_compressed(self.directory/'scene.npz',intrinsic=env.terrain.intrinsic,
            navigation=env.terrain.navigation,reference=env.reference.packed_mask,
            task_polygon=env.task.polygon if hasattr(env,'task') else env.core.task.polygon)
        self._advance=env.plant.advance
        def advance(v,w,dt):
            try:return self._advance(v,w,dt)
            finally:self.pose([v,w])
        env.plant.advance=advance
        self._apply=env._apply_known
        def applied(snapshot):
            result=self._apply(snapshot);self.map(snapshot);return result
        env._apply_known=applied
        from lunar_planning_msgs.msg import PathReference
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscription=env.node.create_subscription(PathReference,env.namespace+'/reference',self.reference,qos)
        self.pose([env.plant.linear_mps,env.plant.angular_radps])
        snapshot=getattr(env.adapter,'snapshot',None)
        if snapshot is not None:self.map(snapshot)
        self.flush_manifest()

    def _event(self,name,data):
        with self.lock:
            if self.closed:return
            self.files[name].write(json.dumps(data,default=json_default,separators=(',',':'))+'\n')

    def pose(self,command):
        started=time.perf_counter();p=self.env.plant.pose
        self._event('trajectory',[self.env.plant.simulation_s-self.time0,p.x,p.y,p.yaw,
            self.env.plant.distance_m-self.distance0,self.env.plant.linear_mps,self.env.plant.angular_radps,*command,bool(self.env.plant.collision)])
        self.pose_count+=1;self.wall_s+=time.perf_counter()-started

    def probabilities(self,values):
        self.last_probs=values.detach().cpu().numpy().astype(float).reshape(-1).tolist()

    def action(self,index,observation):
        started=time.perf_counter();goals=np.asarray(observation.goals).tolist()
        if self.last_probs is None or len(self.last_probs)!=len(goals):
            raise ValueError('recorded action probabilities do not match candidates')
        self._event('decisions',dict(decision=self.env.steps+1,simulation_s=self.env.plant.simulation_s-self.time0,
            distance_m=self.env.plant.distance_m-self.distance0,index=int(index),goal=goals[index],
            candidates=goals,probabilities=self.last_probs,pose=vars(self.env.plant.pose)))
        self.action_count+=1;self.wall_s+=time.perf_counter()-started;self.flush_manifest()

    def reference(self,msg):
        points=[]
        for point in msg.path.poses:
            p=point.pose;q=p.orientation
            points.append([p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
        self._event('references',dict(simulation_s=self.env.plant.simulation_s-self.time0,
            stamp_ns=int(msg.path.header.stamp.sec)*1000000000+msg.path.header.stamp.nanosec,
            frame=msg.path.header.frame_id,session=list(msg.session_id.uuid),revision=msg.segment_revision,
            map_revision=msg.traversability_revision,state=msg.state,final=msg.reaches_final_goal,points=points))

    def map(self,snapshot):
        started=time.perf_counter();h,w=self.env.terrain.shape
        offset=np.rint((np.asarray(self.env.terrain.origin[:2])-snapshot.origin[:2])/snapshot.resolution_m).astype(int)
        parts=[]
        for key,tile in snapshot.tiles.items():
            old=self.tiles.get(key)
            if old is not None and old[0] is tile and old[1]==tuple(offset):continue
            tx,ty=key;x0=tx*256-offset[0];y0=ty*256-offset[1]
            left,right=max(0,x0),min(w,x0+256);top,bottom=max(0,y0),min(h,y0+256)
            if right<=left or bottom<=top:continue
            o=tile.observed.reshape(256,256);b=tile.intrinsic_states.reshape(256,256);m=tile.states.reshape(256,256)
            sl=(slice(top-y0,bottom-y0),slice(left-x0,right-x0))
            if old is None or old[1]!=tuple(offset):changed=(o[sl]!=0)|(b[sl]!=0)|(m[sl]!=0)
            else:
                prev=old[0];changed=(o[sl]!=prev.observed.reshape(256,256)[sl])|(b[sl]!=prev.intrinsic_states.reshape(256,256)[sl])|(m[sl]!=prev.states.reshape(256,256)[sl])
            yy,xx=np.nonzero(changed)
            if len(xx):
                iy,ix=yy+top-y0,xx+left-x0;index=(yy+top)*w+xx+left
                parts.append(encode_delta(index,o[iy,ix],b[iy,ix],m[iy,ix]))
            self.tiles[key]=(tile,tuple(offset))
        if parts:
            with self.lock:
                if self.closed:return
                name=f'{self.map_count:06d}.bin';self.maps.writestr(name,b''.join(parts))
                self.map_events.append(dict(member=name,simulation_s=self.env.plant.simulation_s-self.time0,
                    distance_m=self.env.plant.distance_m-self.distance0,revision=snapshot.revision,
                    origin=list(snapshot.origin),cells=sum(len(p)//7 for p in parts)))
                self.map_count+=1
        self.wall_s+=time.perf_counter()-started

    def flush_manifest(self):
        with self.lock:
            meta=dict(self.meta,pose_samples=self.pose_count,map_updates=self.map_count,decisions=self.action_count,
                collection_wall_s=self.wall_s,status='complete' if self.closed else 'recording')
            temporary=self.directory/'manifest.tmp';temporary.write_text(json.dumps(meta,default=json_default,ensure_ascii=False,indent=2));temporary.rename(self.directory/'manifest.json')

    def close(self):
        self.env.node.destroy_subscription(self.subscription)
        with self.lock:
            if self.closed:return
            self.closed=True
            for f in self.files.values():f.close()
            self.maps.close()
            (self.directory/'map-events.json').write_text(json.dumps(self.map_events,default=json_default,separators=(',',':')))
            self.flush_manifest()
