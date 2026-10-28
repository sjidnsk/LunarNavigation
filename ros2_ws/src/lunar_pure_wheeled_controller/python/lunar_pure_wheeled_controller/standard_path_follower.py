"""标准 Path 执行：有序屏障、物理进度和受认证的停车意图。"""
from dataclasses import replace
import math
import numpy as np
from .mppi import MppiExecutor
from .mppi_path_follower import MppiPathFollower
from .path_geometry import angle
from .standard_path_reference import StandardPathReference


class StandardPathFollower(MppiPathFollower):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.reference=None;self._pending=None;self._stages=[];self._stage_index=0
        self._local_progress=0.;self._total_progress=0.;self._stop_reason=None
        self._switch_brake=False

    def set_standard_path(self, points, identity=None):
        try:reference=StandardPathReference(points,self.policy)
        except (ValueError,TypeError,OverflowError):
            self.request_stop('EMPTY_OR_INVALID_PATH');return False
        self.identity=identity
        if self.reference is not None and reference.points==self.reference.points and self._stop_reason is None:
            return True
        self._pending=reference;self._stop_reason=None
        return True

    def request_stop(self, reason):
        # 停车是意图，不能伪造实际执行过零指令或清除延迟历史。
        self._stop_reason=str(reason);self._pending=None
        self.discard_prediction()

    def _travel_direction(self, state, desired):
        return self._stages[self._stage_index].direction or 1

    def _accept_pending(self, state):
        reference=self._pending
        if reference is None:return
        stages=list(reference.stages)
        # 只沿已执行的有序前缀匹配，不在整条新路线上搜索自交近邻。
        executed=self._stages[:self._stage_index]
        if executed and len(stages)>len(executed) and all(
                a.path.original==b.path.original and a.direction==b.direction
                for a,b in zip(executed,stages)):
            stages=stages[len(executed):]
        candidate=stages[0]
        compatible=self.path is None
        if self.path is not None:
            old=self._stages[self._stage_index]
            actual=self.path.project(state.x_m,state.y_m,reach=self._local_progress+.75).s
            self._total_progress+=max(0.,actual-self._local_progress)
            self._local_progress=max(self._local_progress,actual)
            pos=self.path.point_at(self._local_progress)
            prior=self.path.project(*pos,reach=self._local_progress+.1)
            projections=[candidate.path.project(*pos,reach=.75)]
            if prior.index<len(candidate.path.points)-1:
                projections.append(candidate.path.project(*pos,index=prior.index,reach=self._local_progress+.75))
            projection=min(projections,key=lambda p:(p.error,p.s))
            old_projection=self.path.project(*pos,reach=self._local_progress+.1)
            old_body=old.body_yaw_at(old_projection)
            new_body=candidate.body_yaw_at(projection)
            compatible=(old.direction==candidate.direction and projection.error<=.1 and
                abs(angle(old_body-new_body))<=self.policy.goal_yaw_tolerance_rad)
            # 裁剪不得吞掉尚未完成的原地旋转或终点车体姿态。
            at_end=math.dist((state.x_m,state.y_m),self.path.points[-1])<=self.policy.goal_position_tolerance_m
            unfinished=at_end and abs(angle(self.path.final_yaw-state.yaw_rad))>self.policy.goal_yaw_tolerance_rad
            if unfinished and math.dist(candidate.path.points[0],self.path.points[-1])<=.1:
                stages.insert(0,old);compatible=True
        self._switch_brake=not compatible and not self._stopped(state)
        self.reference=reference;self._pending=None;self._stages=stages;self._stage_index=0
        self.path=stages[0].path;self.final=True
        self._local_progress=self.path.project(state.x_m,state.y_m,reach=.75).s if compatible else 0.
        if not compatible:self.discard_prediction()

    def _orient_progress_anchor(self, state):
        # 换向或越过有序屏障后更新方向，保留已经累积的物理停滞时间。
        ahead=self.path.point_at(min(self.path.length,self._local_progress+.35))
        vector=np.asarray(ahead)-[state.x_m,state.y_m]
        vector=vector/max(float(np.linalg.norm(vector)),1e-12)
        self._last_pose=(np.array([state.x_m,state.y_m]),vector,
            abs(angle(self.path.final_yaw-state.yaw_rad)))

    def _stop(self, state, dt, now_s, state_age_s, *, switching=False):
        self._model_time=self._model_time+dt if now_s is None else float(now_s)
        self._state_age_s=float(state_age_s)
        self.diagnostics={'stop_reason':self._stop_reason or 'REFERENCE_DISCONTINUITY'}
        self.phase='BRAKING'
        safe_state=all(math.isfinite(v) for v in (state.x_m,state.y_m,state.yaw_rad,
            state.linear_mps,state.angular_radps,state.lateral_mps,dt,self._state_age_s))
        if not safe_state or dt<=0 or not 0<=state_age_s<=self.config.state_age_budget_s:
            self.failure='INVALID_STATE';self.phase='FAILED';return self._result()
        if self._stopped(state):
            if switching:
                self._switch_brake=False
                self._orient_progress_anchor(state)
            else:self.phase='WAITING'
            return self._result()
        first=self._first_commands(state,np.zeros((1,2)),min(dt,self.config.model_dt))
        if self.map is not None and self._brake_safe(state,first,self.config.model_dt)[0]:
            return self._result(*first[0])
        # 无地图/认证失败时只有零目标保底，不能声称已认证停车。
        self.diagnostics['stop_certified']=False
        return self._result()

    def _reference_errors(self, poses):
        distance,heading=super()._reference_errors(poses)
        # MPPI 基类按行进方向评分；在此保留标准 Path 的中间车体 yaw。
        if self.path.lengths:
            points=self.path.original;xy=poses[:,:,:2]
            a=np.array([p[:2] for p in points[:-1]]);b=np.array([p[:2] for p in points[1:]])
            vec=b-a;length=np.sum(vec*vec,axis=1)
            fraction=np.clip(np.sum((xy[:,:,None,:]-a)*vec,axis=-1)/length,0.,1.)
            d=np.sum((xy[:,:,None,:]-a-fraction[:,:,:,None]*vec)**2,axis=-1)
            index=np.argmin(d,axis=-1)
            yaw=np.array([p[2] for p in points]);delta=np.arctan2(np.sin(np.diff(yaw)),np.cos(np.diff(yaw)))
            body=yaw[:-1]+fraction*delta
            heading=np.take_along_axis(body,index[:,:,None],axis=-1)[:,:,0]
            heading=heading+(math.pi if self.direction<0 else 0.)
        return distance,heading

    def update(self,state,dt,*,now_s=None,state_age_s=0.):
        if self._stop_reason is not None:
            return self._stop(state,dt,now_s,state_age_s)
        if not all(math.isfinite(v) for v in (state.x_m,state.y_m,state.yaw_rad)):
            return MppiExecutor.update(self,state,dt,now_s,state_age_s)
        self._accept_pending(state)
        if self._switch_brake:
            return self._stop(state,dt,now_s,state_age_s,switching=True)
        if self.path is None:return MppiExecutor.update(self,state,dt,now_s,state_age_s)
        total=self._total_progress;previous=self._local_progress
        self.progress_m=previous
        policy=self.policy
        # 中间屏障必须实到，不能借终点宽容差切掉拐角。
        if self._stage_index+1<len(self._stages):
            self.policy=replace(policy,goal_position_tolerance_m=min(.005,policy.corner_position_tolerance_m))
        try:result=MppiExecutor.update(self,state,dt,now_s,state_age_s)
        finally:self.policy=policy
        self._local_progress=self.progress_m
        self._total_progress=total+max(0.,self._local_progress-previous)
        self.progress_m=self._total_progress
        if result.phase=='COMPLETED':
            if self._stage_index+1<len(self._stages):
                self._stage_index+=1;self.path=self._stages[self._stage_index].path
                self._local_progress=0.;self.phase='TRACKING';self.discard_prediction()
                self._orient_progress_anchor(state)
            else:self.phase='WAITING'
            return self._result()
        if result.phase=='BRAKING' and self.failure is None:
            return self._stop(state,dt,self._model_time,state_age_s)
        return self._result(result.command.linear_x_mps,result.command.angular_z_radps)
