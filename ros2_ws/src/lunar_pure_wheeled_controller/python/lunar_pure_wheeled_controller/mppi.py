"""Sampling-based MPPI for already footprint-inflated local navigation maps.

Gaussian control-sequence perturbations, importance-weighted updates and warm
starts follow the MPPI formulation. Weighted outputs are independently rolled
out and certified; a safe sampled sequence is retained when averaging crosses
an obstacle. This is a project backend, not the Nav2 plugin.
"""
from dataclasses import dataclass
import math
import time
import numpy as np
from .execution import ExecutionResult
from .path_geometry import Polyline, angle
from .tracking import TrackingCommand, TrackingPolicy, _valid_policy


@dataclass(frozen=True)
class MppiPolicy:
    batch_size: int = 32
    time_steps: int = 40
    model_dt: float = .05
    iterations: int = 1
    temperature: float = .15
    linear_std: float = .12
    angular_std: float = .35
    control_cost: float = .002
    seed: int = 17
    compute_budget_s: float = .05
    state_age_budget_s: float = .05
    nominal_response_time_s: float | None = None

    def __post_init__(self):
        if any(not isinstance(v,int) or isinstance(v,bool) or v<1 for v in (self.batch_size,self.time_steps,self.iterations)) or self.batch_size<8:
            raise ValueError('invalid MPPI dimensions')
        if any(not math.isfinite(v) or v<=0 for v in (self.model_dt,self.temperature,self.linear_std,self.angular_std,self.compute_budget_s,self.state_age_budget_s)) or not math.isfinite(self.control_cost) or self.control_cost<0:
            raise ValueError('invalid MPPI parameters')
        if self.nominal_response_time_s is not None and (not math.isfinite(self.nominal_response_time_s) or self.nominal_response_time_s<0):
            raise ValueError('invalid nominal response calibration')


class MppiExecutor:
    requires_map = True

    def __init__(self,policy: TrackingPolicy,mppi_policy=None,**_):
        if not _valid_policy(policy):raise ValueError('invalid execution policy')
        self.policy=policy;self.config=mppi_policy or MppiPolicy()
        self.rng=np.random.default_rng(self.config.seed)
        self.identity=None;self.path=None;self.map=None;self.final=True
        self.phase='WAITING';self.failure=None;self.progress_m=0.;self.error=0.;self.heading_error=0.;self.direction=1
        self._last_command=np.zeros(2);self._model_time=0.;self._state_age_s=0.;self._command_history=[];self._nominal=np.zeros((self.config.time_steps,2))
        self._first_target_step=self.config.model_dt
        self._best_progress=0.;self.no_progress_s=0.;self._last_pose=None
        self.diagnostics={};self.predicted_trajectory=np.empty((0,3))

    def note_command(self,v,w,stamp_s=None):
        self._last_command=np.array([v,w],float)
        stamp=self._model_time if stamp_s is None else float(stamp_s)
        record=(stamp,self._last_command.copy())
        if self._command_history and stamp==self._command_history[-1][0]:self._command_history[-1]=record
        else:self._command_history.append(record)
        self._command_history=self._command_history[-64:]
    def note_angular_command(self,w):self.note_command(self._last_command[0],w)

    def discard_prediction(self):self._nominal[:]=0.

    def clear(self):
        self.path=None;self.phase='WAITING';self.failure=None
        self._nominal[:]=0.;self.note_command(0.,0.);self.no_progress_s=0.;self._last_pose=None;self._command_history=[]

    def set_map(self,grid):
        if self.map is None or grid.epoch!=self.map.epoch:self._nominal[:]=0.
        self.map=grid

    def set_path(self,points,identity=None,final=True):
        path=Polyline(points)
        if identity is not None and self.identity is not None and identity[0]==self.identity[0]:
            if identity[1]<self.identity[1]:return False
            if identity==self.identity:
                if self.path is not None and path.original!=self.path.original:self.failure='REFERENCE_MUTATED'
                return False
            if self.failure is not None:
                self.identity=identity
                return True
            # Keep physical net-progress time across rolling revisions.
        else:self.no_progress_s=0.;self._last_pose=None
        self.path=path;self.identity=identity;self.final=final;self.progress_m=0.;self._best_progress=0.
        self.failure=None;self.phase='BRAKING';self._nominal[:]=0.
        return True

    def _result(self,v=0.,w=0.):
        # Only the transport owner may record an actually published command.
        return ExecutionResult(TrackingCommand(float(v),float(w),self.phase=='COMPLETED',self.failure if self.phase=='FAILED' else None),self.phase,self.progress_m,self.error,self.heading_error,self.direction)

    def _stopped(self,state):
        return math.hypot(state.linear_mps,state.lateral_mps)<=self.policy.stopped_linear_mps and abs(state.angular_radps)<=self.policy.stopped_angular_radps

    def _rollout(self,state,controls,dt,*,delay=True,delay_ticks=None,history_delay_ticks=None,project=True):
        p=self.policy;n,h,_=controls.shape
        # Candidate intents and published targets are different interfaces.
        # Project candidates once; committed commands (including emergency
        # zero-stop) enter the actuator model unchanged.
        if project:controls=self._project_targets(controls,dt)
        positions=np.empty((n,h+1,3));positions[:,0]=[state.x_m,state.y_m,state.yaw_rad]
        actual=np.empty_like(controls);v=np.full(n,state.linear_mps);w=np.full(n,state.angular_radps)
        # The nominal optimizer predicts the calibrated expected response.
        # Future compute time is not an actuator queue. Worst-case response and
        # publication delay remain independently covered by _brake_safe.
        response=p.control_response_time_s if self.config.nominal_response_time_s is None else self.config.nominal_response_time_s
        latency=int(math.ceil(response/dt)) if delay else 0
        wait=int(math.ceil(self._state_age_s/dt)) if delay else 0
        holds=np.full(n,latency+wait,dtype=int) if delay_ticks is None else np.broadcast_to(delay_ticks,(n,)).astype(int)
        transport=np.minimum(holds,latency) if history_delay_ticks is None else np.broadcast_to(history_delay_ticks,(n,)).astype(int)
        times=np.array([r[0] for r in self._command_history]);history=np.array([r[1] for r in self._command_history])
        # Target selection depends only on the fixed history and candidate, not
        # on the simulated actuator state. Batch it once across the horizon.
        ticks=np.arange(h)[None,:]
        prefix=ticks<holds[:,None]
        targets=controls[np.arange(n)[:,None],np.maximum(0,ticks-holds[:,None])]
        if np.any(prefix) and len(times):
            query=self._model_time+ticks*dt-transport[:,None]*dt
            indices=np.searchsorted(times,query+1e-10,side='right')-1
            prior=np.where((indices>=0)[:,:,None],history[np.maximum(indices,0)],0.)
            targets=np.where(prefix[:,:,None],prior,targets)
        elif np.any(prefix):targets=np.where(prefix[:,:,None],0.,targets)
        tv=np.clip(targets[:,:,0],-p.max_reverse_mps if p.allow_reverse else 0.,p.max_linear_mps)
        tw=np.clip(targets[:,:,1],-p.max_angular_radps,p.max_angular_radps)
        integrated=actual if actual.dtype==np.float64 else np.empty(actual.shape,dtype=float)
        for t in range(h):
            accel=np.where(abs(tv[:,t])>abs(v),p.max_linear_accel_mps2,p.max_linear_decel_mps2)
            v=v+np.clip(tv[:,t]-v,-accel*dt,accel*dt)
            w=w+np.clip(tw[:,t]-w,-p.max_angular_accel_radps2*dt,p.max_angular_accel_radps2*dt)
            integrated[:,t,0]=v;integrated[:,t,1]=w
        if integrated is not actual:actual[:]=integrated
        # Keep the original left-to-right addition order, including the initial
        # pose, while evaluating independent arc increments in a single batch.
        theta=integrated[:,:,1]*dt
        positions[:,1:,2]=theta
        np.cumsum(positions[:,:,2],axis=1,out=positions[:,:,2])
        yaw=positions[:,:-1,2];v=integrated[:,:,0];w=integrated[:,:,1]
        turning=abs(w)>1e-12;denom=np.where(turning,w,1.)
        positions[:,1:,0]=np.where(turning,v/denom*(np.sin(yaw+theta)-np.sin(yaw)),v*dt*np.cos(yaw))
        positions[:,1:,1]=np.where(turning,v/denom*(np.cos(yaw)-np.cos(yaw+theta)),v*dt*np.sin(yaw))
        np.cumsum(positions[:,:,0],axis=1,out=positions[:,:,0])
        np.cumsum(positions[:,:,1],axis=1,out=positions[:,:,1])
        return positions,actual

    def _stationary_supported(self,state,poses,velocity):
        """Reuse native stationary authority without certifying translation.

        The authority belongs to this map response's exact anchor. Reference
        session/revision handling and map freshness remain node prerequisites.
        No UNKNOWN cell becomes FREE and ordinary start prefixes are unchanged.
        """
        anchor=self.map.stationary_anchor
        if anchor is None or not self.final or self.path is None or abs(state.lateral_mps)>1e-12 or abs(state.linear_mps)>1e-12:
            return np.zeros(len(poses),dtype=bool)
        # Check original vertices too: Polyline deduplicates very short edges.
        if any(np.any(abs(np.array(p[:2])-anchor)>1e-9) for p in self.path.original):
            return np.zeros(len(poses),dtype=bool)
        at_anchor=np.all(abs(poses[:,:,:2]-anchor)<=1e-9,axis=(1,2))
        stationary=np.all(abs(velocity[:,:,0])<=1e-12,axis=1)
        states,_,_=self.map.sample(poses[:,:,:2])
        return at_anchor & stationary & np.all(states!=2,axis=1)

    def _brake_safe(self,state,first,dt):
        p=self.policy
        latency=int(math.ceil(p.control_response_time_s/dt))
        wait=int(math.ceil((self._state_age_s+self.config.compute_budget_s)/dt))
        hold=latency+wait+1
        stop=int(math.ceil(max(p.max_linear_mps,p.max_reverse_mps,abs(state.linear_mps))/p.max_linear_decel_mps2/dt))+int(math.ceil(max(p.max_angular_radps,abs(state.angular_radps))/p.max_angular_accel_radps2/dt))+2
        # The delayed prefix consumes rollout steps too; retain the complete
        # commanded hold and braking tail after the longest prefix.
        u=np.zeros((len(first),latency+wait+hold+stop,2));u[:,:hold]=first[:,None,:]
        # Evaluate latency and computation/source-age independently. Batching
        # scenarios preserves the finite model set without repeated Python loops.
        scenarios=[(l,q) for l in range(latency+1) for q in range(wait+1)]
        controls=np.tile(u,(len(scenarios),1,1))
        holds=np.repeat([l+q for l,q in scenarios],len(first))
        transport=np.repeat([l for l,q in scenarios],len(first))
        poses,actual=self._rollout(state,controls,dt,delay_ticks=holds,history_delay_ticks=transport,project=False)
        valid=self._stationary_supported(state,poses,actual)
        remaining=~valid
        if np.any(remaining):
            valid[remaining]=self.map.valid_motion(poses[remaining],actual[remaining],dt,p.execution_clearance_m)
        valid &= (abs(actual[:,-1,0])<=p.stopped_linear_mps) & (abs(actual[:,-1,1])<=p.stopped_angular_radps)
        return valid.reshape(len(scenarios),len(first)).all(axis=0)

    def certify_publication(self,state,linear,angular,state_age_s):
        """Recheck the committed target against actual physical state age.

        Wall-clock performance targets never grant or revoke authority. The
        existing full-stop model includes prior commands and response delays.
        Its compute envelope also covers this certificate's publication work.
        """
        if not math.isfinite(state_age_s) or state_age_s<0 or self.map is None:
            return False
        previous=self._state_age_s
        try:
            self._state_age_s=float(state_age_s)
            return bool(self._brake_safe(state,np.array([[linear,angular]]),self.config.model_dt)[0])
        finally:
            self._state_age_s=previous

    def _limit_target_step(self,previous,raw,step):
        # Rate limits belong to successive published targets, not a delayed
        # measurement of the actuator. The rollout models physical slew below.
        p=self.policy
        pv,pw=previous[:,0],previous[:,1]
        v=pv+np.clip(raw[:,0]-pv,-p.max_linear_decel_mps2*step,p.max_linear_accel_mps2*step)
        tw=np.where(abs(raw[:,0])>p.stopped_linear_mps,np.clip(raw[:,1],-abs(raw[:,0])*p.max_curvature_per_m,abs(raw[:,0])*p.max_curvature_per_m),raw[:,1])
        tw=np.where((abs(raw[:,0])<=p.stopped_linear_mps)&(abs(pv)>p.stopped_linear_mps),0.,tw)
        w=pw+np.clip(tw-pw,-p.max_angular_accel_radps2*step,p.max_angular_accel_radps2*step)
        lo=np.maximum(-abs(v)*p.max_curvature_per_m,pw-p.max_angular_accel_radps2*step)
        hi=np.minimum(abs(v)*p.max_curvature_per_m,pw+p.max_angular_accel_radps2*step)
        moving=abs(v)>p.stopped_linear_mps
        w=np.where(moving&(lo<=hi),np.clip(w,lo,hi),w)
        stop=moving&(lo>hi)
        return np.stack((np.where(stop,0.,v),np.where(stop,0.,w)),axis=1)

    def _project_targets(self,raw,step):
        projected=np.empty(raw.shape,dtype=float)
        previous=np.broadcast_to(self._last_command,(len(raw),2))
        for t in range(raw.shape[1]):
            previous=self._limit_target_step(previous,raw[:,t],min(self._first_target_step,step) if t==0 else step)
            projected[:,t]=previous
        return projected

    def _first_commands(self,state,raw,step):
        return self._limit_target_step(np.broadcast_to(self._last_command,(len(raw),2)),raw,step)

    def _score(self,state,controls,target,target_yaw,dt):
        xy,actual=self._rollout(state,controls,dt)
        p=self.policy
        states,_,assumed=self.map.sample(xy[:,:,:2])
        entered=np.maximum.accumulate(states==1,axis=1)
        valid=self._stationary_supported(state,xy,actual)
        remaining=~valid
        if np.any(remaining):
            valid[remaining]=self.map.valid_points(xy[remaining,:,:2],p.execution_clearance_m,~entered[remaining]).all(axis=1)
            valid[remaining] &= ~np.any(entered[remaining]&assumed[remaining],axis=1)
        # Curvature/lateral acceleration constraints apply to moving states;
        # stationary spin remains supported, including its acceleration phase.
        moving=abs(actual[:,:,0])>p.stopped_linear_mps
        valid &= np.all(~moving | ((abs(actual[:,:,1])<=abs(actual[:,:,0])*p.max_curvature_per_m+1e-9)&(abs(actual[:,:,0]*actual[:,:,1])<=p.max_lateral_accel_mps2+1e-9)),axis=1)
        _,terrain,_=self.map.sample(xy[:,:,:2])
        cost=self._tracking_cost(state,controls,target,target_yaw,xy,terrain.mean(axis=1)*.2)
        return np.where(valid,cost,np.inf),xy

    def _tracking_cost(self,state,controls,target,target_yaw,xy,terrain_cost=0.):
        p=self.policy
        final=xy[:,-1,:2];distance=np.sum((final-target)**2,axis=1)
        # Local reference samples avoid jumping to a nearby future branch.
        arcs=np.linspace(self.progress_m,min(self.path.length,self.progress_m+.8),17)
        reference=np.array([self.path.point_at(s) for s in arcs])
        deviation=np.min(np.sum((xy[:,:,None,:2]-reference[None,None,:,:])**2,axis=-1),axis=-1).mean(axis=1)
        yaw_error=np.arctan2(np.sin(xy[:,-1,2]-target_yaw),np.cos(xy[:,-1,2]-target_yaw))
        near=(self.phase=="FINAL_ALIGN" or np.linalg.norm(np.array([state.x_m,state.y_m])-self.path.points[-1])<=p.goal_position_tolerance_m*.5)
        heading=yaw_error**2 if near else np.minimum(yaw_error**2,(math.pi-abs(yaw_error))**2)
        cost=60.*distance+15.*deviation+terrain_cost+(30. if near else 5.)*heading
        cost+=.02*np.mean(np.sum(np.diff(controls,axis=1)**2,axis=-1),axis=1)
        return cost

    def _certify_sequences(self,state,sequences,step,*,costs=None):
        p=self.policy;c=self.config
        poses,velocity=self._rollout(state,sequences,c.model_dt)
        valid=self._stationary_supported(state,poses,velocity)
        remaining=~valid
        if np.any(remaining):
            valid[remaining]=self.map.valid_motion(poses[remaining],velocity[remaining],c.model_dt,p.execution_clearance_m)
        moving=abs(velocity[:,:,0])>p.stopped_linear_mps
        valid &= np.all(~moving|((abs(velocity[:,:,1])<=abs(velocity[:,:,0])*p.max_curvature_per_m+1e-9)&(abs(velocity[:,:,0]*velocity[:,:,1])<=p.max_lateral_accel_mps2+1e-9)),axis=1)
        indices=np.flatnonzero(valid)
        if costs is not None:
            # Fallback selection needs only the least-cost certified candidate.
            # Stable ties preserve the original argmin over sequence order.
            # Each tested candidate retains every delay/full-stop scenario.
            indices=indices[np.argsort(np.asarray(costs)[indices],kind='stable')]
            valid[:]=False
            certificates={}
            for index in indices:
                first=self._first_commands(state,sequences[index:index+1,0],step)
                # Same projected target means the exact same full-stop problem
                # within this call; no state/map/history can change in between.
                key=first.tobytes()
                if key not in certificates:
                    certificates[key]=bool(self._brake_safe(state,first,c.model_dt)[0])
                if certificates[key]:
                    valid[index]=True
                    break
        elif len(indices):
            first=self._first_commands(state,sequences[indices,0],step)
            valid[indices] &= self._brake_safe(state,first,c.model_dt)
        return valid,poses

    def _limit_sample_speeds(self,state,samples,near):
        return samples

    def _control_noise(self):
        c=self.config
        blocks=int(math.ceil(c.time_steps/8))
        noise=self.rng.normal(size=(c.batch_size,blocks,2))*[c.linear_std,c.angular_std]
        return np.repeat(noise,8,axis=1)[:,:c.time_steps]

    def update(self,state,dt,now_s=None,state_age_s=0.):
        started=time.perf_counter();p=self.policy;c=self.config
        self.diagnostics={'valid_samples':0,'weighted_updates':0}
        self._model_time=self._model_time+dt if now_s is None else float(now_s)
        self._state_age_s=float(state_age_s)
        if not all(math.isfinite(v) for v in (state.x_m,state.y_m,state.yaw_rad,state.linear_mps,state.angular_radps,state.lateral_mps,dt,self._model_time,self._state_age_s)) or dt<=0 or not 0<=self._state_age_s<=c.state_age_budget_s:
            self.failure='INVALID_STATE';self.phase='FAILED';return self._result()
        # The first target uses this control interval; subsequent prediction
        # knots retain the fixed model interval. Publication uses the same cap.
        self._first_target_step=min(dt,c.model_dt)
        if self.path is None:return self._result()
        if self.failure:
            self.phase='FAILED' if self._stopped(state) else 'BRAKING';return self._result()
        if self.requires_map and self.map is None:self.phase='WAITING';return self._result()
        projection=self.path.project(state.x_m,state.y_m,reach=self.progress_m+.75)
        self.progress_m=max(self.progress_m,projection.s);self.error=projection.error
        goal=np.array(self.path.points[-1]);distance=float(np.linalg.norm(goal-[state.x_m,state.y_m]))
        yaw_error=angle(self.path.final_yaw-state.yaw_rad)
        self.diagnostics.update(distance_m=distance,goal_yaw_error_rad=yaw_error,
                                velocity=[state.linear_mps,state.angular_radps],dt=dt,
                                nominal_first=self._nominal[0].tolist())
        position_retained=(self.phase=='FINAL_ALIGN' and distance<=p.goal_position_tolerance_m)
        near=distance<=p.goal_position_tolerance_m*.5 or position_retained
        position=np.array([[state.x_m,state.y_m]])
        pose_safe=not self.requires_map
        if self.requires_map:
            pose_safe=self._stationary_supported(state,
                np.array([[[state.x_m,state.y_m,state.yaw_rad]]]),
                np.array([[[state.linear_mps,state.angular_radps]]]))[0]
            if not pose_safe:pose_safe=self.map.valid_segments(position,position,p.execution_clearance_m)[0]
        # Tighter optimization targets must not redefine the public arrival
        # contract. Completion still requires a safe pose and measured stop.
        if distance<=p.goal_position_tolerance_m and pose_safe and (not self.final or abs(yaw_error)<=p.goal_yaw_tolerance_rad):
            self.phase='COMPLETED' if self._stopped(state) else 'BRAKING'
            return self._result()
        # A fixed physical anchor and path-travel direction survive rolling
        # revisions. Crossing a corner need not hit a particular waypoint.
        if self._last_pose is None:
            ahead=self.path.point_at(min(self.path.length,self.progress_m+.35))
            vector=np.array(ahead)-[state.x_m,state.y_m]
            vector=vector/max(float(np.linalg.norm(vector)),1e-12)
            self._last_pose=(np.array([state.x_m,state.y_m]),vector,abs(yaw_error))
        anchor,travel,best_angle=self._last_pose
        advance=float(np.dot(np.array([state.x_m,state.y_m])-anchor,travel))
        if advance>.03 or (distance<=p.goal_position_tolerance_m and abs(yaw_error)<best_angle-.01):
            ahead=self.path.point_at(min(self.path.length,self.progress_m+.35))
            vector=np.array(ahead)-[state.x_m,state.y_m]
            vector=vector/max(float(np.linalg.norm(vector)),1e-12)
            self._last_pose=(np.array([state.x_m,state.y_m]),vector,abs(yaw_error));self.no_progress_s=0.
        else:self.no_progress_s+=min(dt,.2)
        if self.no_progress_s>p.no_progress_timeout_s:
            self.failure='TRACKING_STALLED';self.phase='FAILED' if self._stopped(state) else 'BRAKING';return self._result()
        target=np.array(self.path.point_at(min(self.path.length,self.progress_m+.35)))
        desired=math.atan2(target[1]-state.y_m,target[0]-state.x_m)
        self.direction=-1 if p.allow_reverse and abs(angle(desired-state.yaw_rad))>math.pi*.5 else 1
        travel_yaw=desired-(math.pi if self.direction<0 else 0.)
        self.heading_error=angle(travel_yaw-state.yaw_rad)
        near=distance<=p.goal_position_tolerance_m*.5 or position_retained
        if near:
            target=goal;travel_yaw=self.path.final_yaw;self.phase='FINAL_ALIGN'
            self._nominal[:,0]=0.
        error=angle(travel_yaw-state.yaw_rad)
        # Seed controls accelerate exploration; optimization still considers both
        # signs and time-varying sequences and uses a genuine weighted update.
        seed=np.array([0. if near or abs(error)>.2 else self.direction*min(p.max_linear_mps,p.max_reverse_mps),np.clip(1.5*error,-p.max_angular_radps,p.max_angular_radps)])
        if abs(seed[0])>0:seed[1]=np.clip(seed[1],-abs(seed[0])*p.max_curvature_per_m,abs(seed[0])*p.max_curvature_per_m)
        if not np.any(self._nominal):self._nominal[:]=seed
        best=None;best_xy=None;fallback=None;fallback_costs=None
        for _ in range(c.iterations):
            noise=self._control_noise()
            samples=self._nominal[None]+noise
            samples[0]=self._nominal;samples[1]=seed;samples[2]=0.
            # Pure rotations and same-geometry varying speeds remain reachable
            # even when random noise rarely satisfies a tight curvature limit.
            samples[3]=[0.,p.max_angular_radps];samples[4]=[0.,-p.max_angular_radps]
            samples[5]=[p.max_linear_mps,0.];samples[6]=[-p.max_reverse_mps if p.allow_reverse else 0.,0.]
            samples[7]=[0.,np.clip(error/(c.time_steps*c.model_dt),-p.max_angular_radps,p.max_angular_radps)]
            for i,sign in enumerate((1.,-1.),8):
                if i>=len(samples):break
                samples[i]=[self.direction*min(p.max_linear_mps,p.max_reverse_mps),0.]
                samples[i,:16]=0.
                samples[i,:8,1]=sign*.25
            if near:samples[:,:,0]=0.
            samples[:,:,0]=np.clip(samples[:,:,0],-p.max_reverse_mps if p.allow_reverse else 0.,p.max_linear_mps)
            samples[:,:,1]=np.clip(samples[:,:,1],-p.max_angular_radps,p.max_angular_radps)
            samples=self._limit_sample_speeds(state,samples,near)
            costs,xy=self._score(state,samples,target,travel_yaw,c.model_dt)
            costs+=c.control_cost*np.sum(self._nominal[None]*(samples-self._nominal[None])/np.array([c.linear_std,c.angular_std])**2,axis=(1,2))
            valid=np.isfinite(costs);self.diagnostics['valid_samples']+=int(valid.sum())
            if not np.any(valid):break
            order=np.argsort(costs)
            indices=np.unique(np.concatenate((order[:8],np.arange(min(10,len(samples))))))
            indices=indices[np.isfinite(costs[indices])]
            fallback=samples[indices].copy();fallback_costs=costs[indices]
            weights=np.zeros(c.batch_size);weights[valid]=np.exp(-(costs[valid]-costs[valid].min())/c.temperature);weights/=weights.sum()
            self._nominal=np.sum(weights[:,None,None]*samples,axis=0)
            self.diagnostics['weighted_updates']+=1
        if fallback is not None:
            certified,poses=self._certify_sequences(state,self._nominal[None],min(dt,c.model_dt))
            if certified[0]:
                best=self._nominal.copy();best_xy=poses[0]
                self.diagnostics['selected']='weighted'
            else:
                certified,poses=self._certify_sequences(state,fallback,min(dt,c.model_dt),costs=fallback_costs)
                feasible=np.flatnonzero(certified)
                if len(feasible):
                    chosen=int(feasible[np.argmin(fallback_costs[feasible])])
                    best=fallback[chosen];best_xy=poses[chosen]
                self.diagnostics['selected']='sampled_fallback'
        if best is None:
            self._nominal[:]=0.
            if not self.requires_map or self._brake_safe(state,np.zeros((1,2)),c.model_dt)[0]:
                self.phase='BRAKING'
                return self._result()
            self.failure='MPPI_STOPPING_UNSAFE';self.phase='FAILED' if self._stopped(state) else 'BRAKING';return self._result()
        # _certify_sequences already certified this exact projected first target
        # with the same state, map, history and interval. No mutation of those
        # inputs occurs before publication; reuse its full braking certificate.
        first=self._first_commands(state,best[None,0,:],min(dt,c.model_dt))[0]
        v,w=first
        self._nominal=np.concatenate((best[1:],best[-1:]),axis=0)
        self.predicted_trajectory=best_xy;self.phase='FINAL_ALIGN' if near else 'TRACKING'
        self.diagnostics['compute_ms']=(time.perf_counter()-started)*1000.
        return self._result(v,w)
