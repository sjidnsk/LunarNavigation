"""MPPI reference-path following, independent of map/planner authority.

Obstacle routing and reference invalidation belong to navigation. This backend
predicts and constrains vehicle motion; it does not certify obstacle avoidance.
"""
import math
from dataclasses import replace
import numpy as np
from .path_geometry import Polyline, angle
from .mppi import MppiExecutor


class MppiPathFollower(MppiExecutor):
    requires_map = False

    def _kinematically_valid(self,velocity):
        p=self.policy
        moving=abs(velocity[:,:,0])>p.stopped_linear_mps
        return np.all(~moving|((abs(velocity[:,:,1])<=abs(velocity[:,:,0])*p.max_curvature_per_m+1e-9)
            &(abs(velocity[:,:,0]*velocity[:,:,1])<=p.max_lateral_accel_mps2+1e-9)),axis=1)

    def _score(self,state,controls,target,target_yaw,dt):
        poses,velocity=self._rollout(state,controls,dt)
        valid=self._kinematically_valid(velocity)
        cost=self._tracking_cost(state,controls,target,target_yaw,poses)
        return np.where(valid,cost,np.inf),poses

    def _certify_sequences(self,state,sequences,step,*,costs=None):
        # Weighted controls define a new nonlinear trajectory. Recompute its
        # motion constraints rather than averaging sampled poses.
        poses,velocity=self._rollout(state,sequences,self.config.model_dt)
        return self._kinematically_valid(velocity),poses

    def _project_targets(self,raw,step):
        """Batch time-independent intent limits; retain slew recurrence order."""
        p=self.policy
        output=np.empty(raw.shape,dtype=float)
        intent_v=raw[:,:,0];intent_w=raw[:,:,1]
        curvature=abs(intent_v)*p.max_curvature_per_m
        intent_w=np.where(abs(intent_v)>p.stopped_linear_mps,
            np.minimum(np.maximum(intent_w,-curvature),curvature),intent_w)
        pv=np.full(len(raw),self._last_command[0]);pw=np.full(len(raw),self._last_command[1])
        for t in range(raw.shape[1]):
            interval=min(self._first_target_step,step) if t==0 else step
            v=pv+np.minimum(np.maximum(intent_v[:,t]-pv,-p.max_linear_decel_mps2*interval),p.max_linear_accel_mps2*interval)
            target_w=np.where((abs(intent_v[:,t])<=p.stopped_linear_mps)&(abs(pv)>p.stopped_linear_mps),0.,intent_w[:,t])
            bound=p.max_angular_accel_radps2*interval
            w=pw+np.minimum(np.maximum(target_w-pw,-bound),bound)
            curvature=abs(v)*p.max_curvature_per_m
            lo=np.maximum(-curvature,pw-bound);hi=np.minimum(curvature,pw+bound)
            moving=abs(v)>p.stopped_linear_mps
            w=np.where(moving&(lo<=hi),np.minimum(np.maximum(w,lo),hi),w)
            stop=moving&(lo>hi)
            pv=np.where(stop,0.,v);pw=np.where(stop,0.,w)
            output[:,t,0]=pv;output[:,t,1]=pw
        return output

    lateral_error_scale_m = .01
    heading_error_scale_rad = .05

    def _reference_errors(self,poses):
        """Exact projection onto a bounded local portion of the polyline."""
        xy=poses[:,:,:2]
        if not self.path.lengths:
            distance=np.sum((xy-self.path.points[0])**2,axis=-1)
            heading=np.full(distance.shape,self.path.final_yaw)
            return distance,heading
        lo=max(0.,self.progress_m-.1);hi=min(self.path.length,self.progress_m+.8)
        # Polyline geometry is immutable. Keep one bounded cache, keyed by
        # object identity so rolling revisions and temporary corner prefixes
        # cannot reuse stale coordinates or headings.
        cache=getattr(self,'_reference_geometry',None)
        if cache is None or cache[0] is not self.path:
            points=np.asarray(self.path.points)
            base=points[:-1].copy();vector=points[1:]-base
            arc_start=np.asarray(self.path.arcs[:-1]);arc_end=np.asarray(self.path.arcs[1:])
            lengths=np.asarray(self.path.lengths);headings=np.asarray(self.path.headings)
            for array in (base,vector,arc_start,arc_end,lengths,headings):
                array.setflags(write=False)
            cache=(self.path,base,vector,arc_start,arc_end,lengths,headings)
            self._reference_geometry=cache
        _,base,vector,arc_start,arc_end,lengths,headings=cache
        # Strict endpoint exclusion matches the original end <= start check.
        first=np.searchsorted(arc_end,lo,side='right')
        last=np.searchsorted(arc_start,hi,side='left')
        if first>=last:
            return np.sum((xy-self.path.points[-1])**2,axis=-1),np.full(xy.shape[:-1],self.path.headings[-1])
        start=np.maximum(lo,arc_start[first:last]);end=np.minimum(hi,arc_end[first:last])
        fraction=(start-arc_start[first:last])/lengths[first:last]
        size=(end-start)/lengths[first:last]
        ax=base[first:last,0]+vector[first:last,0]*fraction
        ay=base[first:last,1]+vector[first:last,1]*fraction
        vx=vector[first:last,0]*size;vy=vector[first:last,1]*size
        # Preserve the two-term operation order while avoiding an extra
        # coordinate dimension, reductions, and their temporary arrays.
        dx=xy[:,:,0,None]-ax;dy=xy[:,:,1,None]-ay
        t=np.clip((dx*vx+dy*vy)/(vx*vx+vy*vy),0.,1.)
        distance=(dx-t*vx)**2+(dy-t*vy)**2
        index=np.argmin(distance,axis=-1)
        return np.take_along_axis(distance,index[:,:,None],axis=-1)[:,:,0],headings[first:last][index]

    def _tracking_cost(self,state,controls,target,target_yaw,xy,terrain_cost=0.):
        distance,heading=self._reference_errors(xy)
        near=(self.phase=='FINAL_ALIGN' or np.linalg.norm(np.array([state.x_m,state.y_m])-self.path.points[-1])<=self.policy.goal_position_tolerance_m)
        desired=self.path.final_yaw if near else heading-(np.pi if self.direction<0 else 0.)
        if not near and self.progress_m>=self.path.length-1e-10:
            goal=np.asarray(self.path.points[-1]);delta=goal-xy[:,:,:2]
            desired=np.arctan2(delta[:,:,1],delta[:,:,0])-(np.pi if self.direction<0 else 0.)
        error=np.arctan2(np.sin(xy[:,:,2]-desired),np.cos(xy[:,:,2]-desired))
        endpoint=np.sum((xy[:,-1,:2]-target)**2,axis=-1)
        progress_scale=.1 if getattr(self,'_tracking_corner',False) else 1.
        cost=60.*endpoint/progress_scale**2+15.*np.mean(distance/self.lateral_error_scale_m**2,axis=1)
        cost+=(30. if near else 5.)*np.mean((error/self.heading_error_scale_rad)**2,axis=1)
        cost+=.02*np.mean(np.sum(np.diff(controls,axis=1)**2,axis=-1),axis=1)
        return cost

    def _control_noise(self):
        c=self.config;blocks=(c.time_steps+7)//8
        noise=np.zeros((c.batch_size,blocks,2))
        # Slots 0..9 are deterministic proposals supplied by the shared solver.
        pairs=max(0,(c.batch_size-10)//2)
        if pairs:
            draw=self.rng.normal(size=(pairs,blocks,2))*[c.linear_std,c.angular_std]
            draw[:,:,0]-=draw[:,:,0].mean(axis=0)
            noise[10:10+2*pairs:2]=draw
            noise[11:10+2*pairs:2]=draw*[1.,-1.]
        return np.repeat(noise,8,axis=1)[:,:c.time_steps]

    def clear(self):
        super().clear()
        self._reference_geometry=None
        self._passed_corner=None
        self._corner_target=None

    def set_path(self,points,identity=None,final=True):
        if self.identity is None or identity is None or identity[0]!=self.identity[0]:
            self._passed_corner=None
            self._corner_target=None
        accepted=super().set_path(points,identity,final)
        target=getattr(self,'_corner_target',None)
        if accepted and target is not None:
            upcoming=[i for i in range(1,len(self.path.headings))
                if abs(angle(self.path.headings[i]-self.path.headings[i-1]))>=self.policy.corner_angle_rad
                and self.path.points[i]!=getattr(self,'_passed_corner',None)]
            if not upcoming or self.path.points[upcoming[0]]!=target[0]:
                self._corner_target=None
            else:
                delta=abs(angle(self.path.headings[upcoming[0]]-target[1]))
                compatible=min(delta,abs(math.pi-delta)) if self.policy.allow_reverse else delta
                if compatible>self.policy.goal_yaw_tolerance_rad:
                    self._corner_target=None
        return accepted

    def update(self,state,dt,now_s=None,state_age_s=0.):
        route=self.path
        if route is None or self.failure is not None:
            return super().update(state,dt,now_s,state_age_s)
        corner=None
        if all(math.isfinite(x) for x in (state.x_m,state.y_m,state.yaw_rad)):
            projection=route.project(state.x_m,state.y_m,reach=self.progress_m+.75)
            for i in range(1,len(route.headings)):
                active=getattr(self,'_corner_target',None)
                if active is not None and route.points[i]!=active[0]:continue
                if active is None and route.arcs[i]<projection.s-self.lateral_error_scale_m*.5:continue
                if abs(angle(route.headings[i]-route.headings[i-1]))<self.policy.corner_angle_rad:continue
                position=route.points[i]
                if getattr(self,'_passed_corner',None)==position:continue
                reverse=self.policy.allow_reverse and abs(angle(route.headings[i-1]-state.yaw_rad))>math.pi*.5
                target=getattr(self,'_corner_target',None)
                yaw=target[1] if target is not None and target[0]==position else route.headings[i]-(math.pi if reverse else 0.)
                self._corner_target=(position,yaw)
                corner=(i,yaw)
                break
        if corner is None:return super().update(state,dt,now_s,state_age_s)
        i,yaw=corner;policy=self.policy;final=self.final
        # A sharp polyline bend has no finite-curvature moving solution.
        # Track its vertex, stop, turn, then continue the same external route.
        points=[(*p,route.headings[min(j,len(route.headings)-1)]) for j,p in enumerate(route.points[:i+1])]
        points[-1]=(*route.points[i],yaw)
        self.path=Polyline(points);self.final=True
        self.policy=replace(policy,goal_position_tolerance_m=min(policy.corner_position_tolerance_m,self.lateral_error_scale_m*.5))
        self._tracking_corner=True
        try:result=super().update(state,dt,now_s,state_age_s)
        finally:
            self.path=route;self.policy=policy;self.final=final;self._tracking_corner=False
        if result.phase=='COMPLETED':
            self._passed_corner=route.points[i];self._corner_target=None;self.phase='TRACKING'
            self._nominal[:]=0.;self._last_pose=None;self.no_progress_s=0.
            return self._result()
        return result

    def _limit_sample_speeds(self,state,samples,near):
        p=self.policy
        heading=max(0.,math.cos(self.heading_error))**2
        # Arc projection can reach the endpoint while the vehicle still has
        # lateral/overshoot error. Keep physical endpoint correction possible.
        physical=math.hypot(state.x_m-self.path.points[-1][0],state.y_m-self.path.points[-1][1])
        remaining=max(0.,self.path.length-self.progress_m,physical)
        braking=math.sqrt(2.*p.max_linear_decel_mps2*remaining)
        horizon=max(self.config.model_dt,self.config.time_steps*self.config.model_dt)
        approach=remaining/horizon
        cap=min(p.max_linear_mps,p.max_reverse_mps,braking,approach)*heading
        samples[:,:,0]=np.clip(samples[:,:,0],-cap if p.allow_reverse else 0.,cap)
        self.diagnostics['speed_limit_mps']=cap
        return samples
