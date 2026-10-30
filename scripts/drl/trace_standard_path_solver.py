"""仅供隔离诊断显式启用；包装现有求解器，原返回值和采样流保持不变。"""
import json
import math
import time
from dataclasses import asdict
from pathlib import Path
import numpy as np


def install(path):
    from lunar_pure_wheeled_controller.standard_path_follower import StandardPathFollower as cls
    original_score=cls._score;original_update=cls.update;original_accept=cls._accept_pending
    stream=Path(path).open('x',encoding='utf-8',buffering=1)
    last={};captured=set();maps={};saved=set()
    def emit(event,**values):
        stream.write(json.dumps(dict(wall=time.monotonic(),event=event,**values),default=lambda x:x.tolist() if isinstance(x,np.ndarray) else vars(x),allow_nan=True)+'\n')
    def stage(s):
        return dict(direction=s.direction,spin_delta=s.spin_delta,length=s.path.length,points=s.path.original)
    def context(e,state):
        active=e.execution_reference
        return dict(state=asdict(state),stage_index=e._stage_index,stages=[stage(s) for s in e._stages],
            active=None if active is None else vars(active),path=e.path.original if e.path else None,
            policy=asdict(e.policy),config=asdict(e.config),progress_m=e.progress_m,
            local_progress_m=e._local_progress,no_progress_s=e.no_progress_s,anchor=e._last_pose,
            translation_credit_m=getattr(e,'_translation_credit_m',0.),
            phase=e.phase,failure=e.failure,direction=e.direction,last_command=e._last_command,
            history=[(t,u.tolist()) for t,u in e._command_history],model_time=e._model_time,state_age=e._state_age_s)
    def score(e,state,controls,target,target_yaw,dt):
        costs,poses=original_score(e,state,controls,target,target_yaw,dt)
        now=time.monotonic()
        distance=math.dist((state.x_m,state.y_m),e.path.points[-1])
        if (distance<.2 or e.no_progress_s>2.) and now-last.get(id(e),-1e9)>=1.:
            last[id(e)]=now;captured.add(id(e));maps[id(e)]=e.map
            correction=e.config.control_cost*np.sum(e._nominal[None]*(controls-e._nominal[None])/np.array([e.config.linear_std,e.config.angular_std])**2,axis=(1,2))
            total=costs+correction;valid=np.isfinite(total)
            weights=np.zeros(len(total))
            if valid.any():
                weights[valid]=np.exp(-(total[valid]-total[valid].min())/e.config.temperature)
                weights/=weights.sum()
            distance2,heading=e._reference_errors(poses)
            near=(e.phase=='FINAL_ALIGN' or distance<=e.policy.goal_position_tolerance_m)
            desired=e.path.final_yaw if near else heading-(np.pi if e.direction<0 else 0.)
            if not near and e.progress_m>=e.path.length-1e-10:
                delta=np.asarray(e.path.points[-1])-poses[:,:,:2]
                desired=np.arctan2(delta[:,:,1],delta[:,:,0])-(np.pi if e.direction<0 else 0.)
            error=np.arctan2(np.sin(poses[:,:,2]-desired),np.cos(poses[:,:,2]-desired))
            endpoint=60*np.sum((poses[:,-1,:2]-target)**2,axis=-1)
            lateral=15*np.mean(distance2/e.lateral_error_scale_m**2,axis=1)
            yaw=(30. if near else 5.)*np.mean((error/e.heading_error_scale_rad)**2,axis=1)
            smooth=.02*np.mean(np.sum(np.diff(controls,axis=1)**2,axis=-1),axis=1)
            emit('score',context=context(e,state),target=target,target_yaw=target_yaw,near=near,
                nominal=e._nominal.copy(),samples=controls,cost=costs,correction=correction,total=total,weights=weights,
                endpoint=endpoint,lateral=lateral,yaw=yaw,smooth=smooth,final_pose=poses[:,-1],
                speed_limit=e.diagnostics.get('speed_limit_mps'),
                first_target_step=e._first_target_step,
                map_metadata=dict(resolution=e.map.resolution,origin=e.map.origin,revision=e.map.revision,
                    epoch=e.map.epoch,stationary_anchor=e.map.stationary_anchor))
        return costs,poses
    def accept(e,state):
        pending=e._pending
        old=e._stages[e._stage_index] if e._stages else None
        old_description=None if old is None else stage(old)
        result=original_accept(e,state)
        if pending is not None and e.no_progress_s>2.:
            emit('accept',old=old_description,incoming=[stage(s) for s in pending.stages],
                retained_old=bool(old is not None and e._stages and e._stages[0] is old),context=context(e,state))
        return result
    def update(e,state,*args,**kwargs):
        result=original_update(e,state,*args,**kwargs)
        if id(e) in captured or e.failure:
            captured.discard(id(e))
            if e.failure and id(e) not in saved and id(e) in maps:
                grid=maps[id(e)];saved.add(id(e))
                np.savez_compressed(Path(path).with_suffix('.map.npz'),states=grid.states,costs=grid.costs,
                    assumed=grid.assumed,eligible=grid.eligible)
            emit('result',context=context(e,state),result=asdict(result),diagnostics=e.diagnostics)
        return result
    cls._score=score;cls._accept_pending=accept;cls.update=update
    def uninstall():
        cls._score=original_score;cls._accept_pending=original_accept;cls.update=original_update
        stream.close()
    return uninstall
