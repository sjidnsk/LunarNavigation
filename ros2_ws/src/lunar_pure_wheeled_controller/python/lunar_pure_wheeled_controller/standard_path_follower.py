"""标准 Path 执行：有序屏障、物理进度和受认证的停车意图。"""
from dataclasses import replace
import math
import numpy as np
from .mppi import MppiExecutor
from .mppi_path_follower import MppiPathFollower
from .path_geometry import Polyline, angle
from .standard_path_reference import StandardPathReference


class StandardPathFollower(MppiPathFollower):
    tracks_task_progress = False
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.config=replace(self.config,state_age_budget_s=None)
        self.reference=None;self._pending=None;self._stages=[];self._stage_index=0
        self._local_progress=0.;self._total_progress=0.;self._stop_reason=None
        self._switch_brake=False
        self.execution_reference=None;self._execution_pending=None
        self._execution_seen={};self._execution_retired={}
        self.last_execution_state=0

    def set_standard_path(self, points, identity=None):
        try:reference=StandardPathReference(points,self.policy)
        except (ValueError,TypeError,OverflowError):
            self.request_stop('EMPTY_OR_INVALID_PATH');return False
        self.identity=identity
        if self.reference is not None and reference.points==self.reference.points and self._stop_reason is None:
            self._pending=None
            return True
        self._pending=reference;self._stop_reason=None
        return True

    def request_stop(self, reason):
        # 停车是意图，不能伪造实际执行过零指令或清除延迟历史。
        reason=str(reason)
        ordinary=('EXECUTION_PAIR_WAIT','STALE_PATH','NO_PATH')
        # 传输/路径等待不能覆盖恢复停车屏障或人工停止。
        if self._stop_reason is None or reason not in ordinary or self._stop_reason in ordinary:
            self._stop_reason=reason
        self._pending=None
        self._execution_pending=None
        self.discard_prediction()

    def _recover_failure(self,state):
        if self.execution_reference is not None or self._execution_pending is not None:return
        if self.failure!='MPPI_STOPPING_UNSAFE':return
        # 调用点已验证最新反馈与时间；实测静止不能代替延迟队列/完整尾部证书。
        certified=(self.map is not None and
            bool(self._brake_safe(state,np.zeros((1,2)),self.config.model_dt)[0]))
        self.diagnostics.update(stop_certified=certified,measured_stopped=self._stopped(state),
            stopping_retry=True)
        if certified:
            self.failure=None
            self.diagnostics['recovered_failure']='MPPI_STOPPING_UNSAFE'
        # 不修改真实速度、已发布指令历史、游标、物理进度锚点或停滞时钟。

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
        matched_projection=None
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
            if compatible:matched_projection=projection
            # 裁剪不得吞掉尚未完成的原地旋转或终点车体姿态。
            at_end=math.dist((state.x_m,state.y_m),self.path.points[-1])<=self.policy.goal_position_tolerance_m
            unfinished=at_end and abs(angle(self.path.final_yaw-state.yaw_rad))>self.policy.goal_yaw_tolerance_rad
            if unfinished and math.dist(candidate.path.points[0],self.path.points[-1])<=.1:
                stages.insert(0,old);compatible=True
                matched_projection=old_projection
        self._switch_brake=not compatible and not self._stopped(state)
        self.reference=reference;self._pending=None;self._stages=stages;self._stage_index=0
        self.path=stages[0].path;self.final=True
        if compatible and matched_projection is not None:
            # 沿兼容匹配的游标投影实际状态，不能退回路径开头的窗口。
            actual=self.path.project(state.x_m,state.y_m,index=matched_projection.index,
                reach=matched_projection.s+.75)
            self._local_progress=max(matched_projection.s,actual.s)
        else:
            self._local_progress=self.path.project(state.x_m,state.y_m,reach=.75).s if compatible else 0.
        if not compatible:
            self.discard_prediction()

    def _stop(self, state, dt, now_s, state_age_s, *, switching=False):
        self._model_time=self._model_time+dt if now_s is None else float(now_s)
        self._state_age_s=float(state_age_s)
        self.diagnostics={'stop_reason':self._stop_reason or 'REFERENCE_DISCONTINUITY'}
        self.phase='BRAKING'
        safe_state=all(math.isfinite(v) for v in (state.x_m,state.y_m,state.yaw_rad,
            state.linear_mps,state.angular_radps,state.lateral_mps,dt,self._state_age_s))
        if not safe_state or dt<=0 or not self.config.accepts_state_age(state_age_s):
            self.failure='INVALID_STATE';self.phase='FAILED';return self._result()
        if self._stopped(state):
            if switching:
                self._switch_brake=False
            else:self.phase='WAITING'
            return self._result()
        first=self._first_commands(state,np.zeros((1,2)),min(dt,self.config.model_dt))
        if self.map is not None and self._brake_safe(state,first,self.config.model_dt)[0]:
            return self._result(*first[0])
        # 无地图/认证失败时只有零目标保底，不能声称已认证停车。
        self.diagnostics['stop_certified']=False
        return self._result()

    def _seed_sequence(self,state,seed,near):
        fallback=super()._seed_sequence(state,seed,near)
        if near or not self.path.lengths or self.progress_m>=self.path.length-1e-10:
            return fallback
        # 仅对已有有序几何作时间参数化，不在执行器中重新规划路线。
        # 与实际候选使用同一接近/制动限速，速度与yaw随弧长变化保持配套。
        p=self.policy;c=self.config
        sequence=np.zeros((c.time_steps,2))
        sequence[:,0]=self.direction*min(p.max_linear_mps,p.max_reverse_mps)
        sequence=self._limit_sample_speeds(state,sequence[None],False)[0]
        speed=abs(sequence[0,0])
        if speed<=1e-12:return fallback
        points=np.asarray(self.path.original)
        lengths=np.linalg.norm(np.diff(points[:,:2],axis=0),axis=1)
        arcs=np.concatenate(([0.],np.cumsum(lengths)))
        delta=np.arctan2(np.sin(np.diff(points[:,2])),np.cos(np.diff(points[:,2])))
        yaw=np.concatenate(([points[0,2]],points[0,2]+np.cumsum(delta)))
        samples=np.minimum(arcs[-1],self.progress_m+speed*c.model_dt*np.arange(c.time_steps+1))
        body=np.interp(samples,arcs,yaw)
        sequence[:,1]=np.diff(body)/c.model_dt
        # 沿用原seed的恒定角误差反馈，不把初始偏差误当作路径曲率。
        sequence[:,1]+=1.5*angle(body[0]-state.yaw_rad)
        sequence[:,1]=np.clip(sequence[:,1],-p.max_angular_radps,p.max_angular_radps)
        if p.motion_model=='curvature_limited':
            bound=abs(sequence[:,0])*p.max_curvature_per_m
            sequence[:,1]=np.clip(sequence[:,1],-bound,bound)
        return sequence

    def _reference_errors(self, poses):
        distance,index,fraction=self._reference_projection(poses)
        if index is None:return distance,np.full(distance.shape,self.path.final_yaw)
        # 标准Path的车体yaw与位置共用同一有序局部投影，不再搜索远期邻近路段。
        cache=getattr(self,'_reference_body_geometry',None)
        if cache is None or cache[0] is not self.path:
            yaw=np.asarray([p[2] for p in self.path.original])
            delta=np.arctan2(np.sin(np.diff(yaw)),np.cos(np.diff(yaw)))
            yaw.setflags(write=False);delta.setflags(write=False)
            cache=(self.path,yaw,delta)
            self._reference_body_geometry=cache
        _,yaw,delta=cache
        heading=yaw[index]+fraction*delta[index]+(math.pi if self.direction<0 else 0.)
        return distance,heading

    def _continuous_spin_target(self,state):
        # 分段保留圈数与反向顺序，但同中心同向分段不是停车屏障。
        current=self._stages[self._stage_index]
        if current.direction!=0 or abs(current.spin_delta)<=1e-9:return self.path
        end=self._stage_index
        while end+1<len(self._stages):
            following=self._stages[end+1]
            if (following.direction!=0 or following.spin_delta*current.spin_delta<=0 or
                    math.dist(current.path.points[-1],following.path.points[-1])>1e-8):break
            end+=1
        sign=1. if current.spin_delta>0 else -1.
        at_center=math.dist((state.x_m,state.y_m),current.path.points[-1])<=self.policy.goal_position_tolerance_m
        while self._stage_index<end and at_center:
            start=current.path.final_yaw-current.spin_delta
            offset=sign*angle(state.yaw_rad-start)
            # 起点后方的反向偏差不是已经执行的旋转，不能由末端wrap符号跳段。
            if offset<abs(current.spin_delta)-1e-9:break
            self._stage_index+=1
            current=self._stages[self._stage_index]
            self.path=current.path
        if self._stage_index==end:return self.path
        start=current.path.final_yaw-current.spin_delta
        remaining=(abs(current.spin_delta)-sign*angle(state.yaw_rad-start)+
            sum(abs(stage.spin_delta) for stage in self._stages[self._stage_index+1:end+1]))
        # 沿既有有序参考前视，最多四分之一圈，避免最短角目标反转或丢圈。
        x,y=current.path.points[-1]
        return Polyline(((x,y,state.yaw_rad+sign*min(math.pi/2,remaining)),))

    def _update_standard(self,state,dt,*,now_s=None,state_age_s=0.):
        if self._stop_reason is not None:
            return self._stop(state,dt,now_s,state_age_s)
        if not all(math.isfinite(v) for v in (state.x_m,state.y_m,state.yaw_rad)):
            return MppiExecutor.update(self,state,dt,now_s,state_age_s)
        self._accept_pending(state)
        if self._switch_brake:
            return self._stop(state,dt,now_s,state_age_s,switching=True)
        if self.path is None:return MppiExecutor.update(self,state,dt,now_s,state_age_s)
        target=self._continuous_spin_target(state)
        stage_path=self.path
        total=self._total_progress;previous=self._local_progress
        self.progress_m=previous
        policy=self.policy
        # 中间屏障必须实到，不能借终点宽容差切掉拐角。
        if self._stage_index+1<len(self._stages) and not (
                self.execution_reference is not None and self.execution_reference.intent==3):
            self.policy=replace(policy,goal_position_tolerance_m=min(.005,policy.corner_position_tolerance_m))
        self.path=target
        try:result=MppiExecutor.update(self,state,dt,now_s,state_age_s)
        finally:
            self.policy=policy
            self.path=stage_path
        self._local_progress=self.progress_m
        self._total_progress=total+max(0.,self._local_progress-previous)
        self.progress_m=self._total_progress
        if result.phase=='COMPLETED':
            if self._stage_index+1<len(self._stages):
                self._stage_index+=1;self.path=self._stages[self._stage_index].path
                self._local_progress=0.;self.phase='TRACKING';self.discard_prediction()
            else:self.phase='WAITING'
            return self._result()
        if result.phase=='BRAKING' and self.failure is None:
            return self._stop(state,dt,self._model_time,state_age_s)
        return self._result(result.command.linear_x_mps,result.command.angular_z_radps)

    @property
    def reference_accepted(self):
        return self.execution_reference is not None and self._execution_pending is None

    def offer_execution_reference(self, points, reference):
        # 身份不可原地变异；相同请求重发也不算一次新的接管。
        try:
            values=tuple(points)
            # 停车授权无需位置目标，空Path不得伪造世界原点目标。
            geometry=(None if not values and reference.intent in (1,2,4)
                else StandardPathReference(values,self.policy))
            key=reference.session_id
            signature=(reference.revision,reference.path_digest,reference.intent,
                reference.stamp_ns,reference.position_tolerance_m,reference.yaw_tolerance_rad,
                reference.stopped_linear_mps,reference.stopped_angular_rps,
                geometry.points if geometry is not None else ())
            if not key or reference.intent not in range(5) or reference.revision<0:return False
            if key in self._execution_retired:return False
            old=self._execution_seen.get(key)
            if old is not None and reference.revision<=old[0]:return False
            if any(not math.isfinite(v) or v<=0 for v in signature[4:8]):return False
        except (AttributeError,ValueError,TypeError,OverflowError):return False
        self._execution_seen.pop(key,None)
        self._execution_seen[key]=signature
        while len(self._execution_seen)>128:
            active_key=(self.execution_reference.session_id
                if self.execution_reference is not None else None)
            oldest=next(k for k in self._execution_seen if k not in (key,active_key))
            self._execution_seen.pop(oldest)
        self._execution_pending=(geometry,reference)
        return True

    def wait_for_pose(self):
        # 退休几何但保留参考身份、真实命令历史和故障/进展账本。
        previous=self._stop_reason
        self.request_stop('WAITING_FOR_POSE')
        if previous is not None and previous not in (
                'EXECUTION_PAIR_WAIT','STALE_PATH','NO_PATH','WAITING_FOR_POSE','WAITING_FOR_MAP','WAITING_FOR_FRESH_REFERENCE'):
            self._stop_reason=previous
        self._retire_execution_geometry()
        self.phase='WAITING';self.last_execution_state=0

    def _execution_policy(self, reference):
        return replace(self.policy,goal_position_tolerance_m=reference.position_tolerance_m,
            goal_yaw_tolerance_rad=reference.yaw_tolerance_rad,
            stopped_linear_mps=reference.stopped_linear_mps,
            stopped_angular_radps=reference.stopped_angular_rps)

    def _execution_certified(self,state,dt,now_s,state_age_s):
        # 独立检查完整停车尾部（含真实发布历史），实测零速本身不构成认证。
        if (not all(math.isfinite(v) for v in (state.x_m,state.y_m,state.yaw_rad,
                state.linear_mps,state.angular_radps,state.lateral_mps,dt,state_age_s))
                or dt<=0 or not self.config.accepts_state_age(state_age_s)):return False
        self._model_time=self._model_time if now_s is None else float(now_s)
        self._state_age_s=state_age_s
        return self.map is not None and bool(self._brake_safe(
            state,np.zeros((1,2)),self.config.model_dt)[0])

    def _retire_execution_geometry(self):
        self.reference=None;self._pending=None;self.path=None;self._stages=[]
        self._stage_index=0;self._local_progress=0.;self._total_progress=0.
        self._switch_brake=False;self.discard_prediction()

    def _alignment_extension(self,state,geometry,active,requested):
        # 局部末yaw只是horizon端点：只续接完整覆盖旧剩余屏障的同向旋转。
        if (active is None or active.session_id!=requested.session_id or
                active.intent!=3 or requested.intent!=3 or self.reference is None):return None
        if any(getattr(active,name)!=getattr(requested,name) for name in (
                'position_tolerance_m','yaw_tolerance_rad',
                'stopped_linear_mps','stopped_angular_rps')):return None
        old_center=self.reference.points[-1][:2];new_center=geometry.points[-1][:2]
        position=(state.x_m,state.y_m);tolerance=requested.position_tolerance_m
        if (max(math.dist(old_center,new_center),math.dist(position,old_center),
                math.dist(position,new_center))>tolerance or
                math.hypot(state.linear_mps,state.lateral_mps)>requested.stopped_linear_mps):return None
        remaining=self._stages[self._stage_index:]
        if not remaining or not geometry.stages:return None
        sign=1. if remaining[0].spin_delta>0 else -1.
        if any(stage.direction!=0 or sign*stage.spin_delta<=1e-9
                for stage in (*remaining,*geometry.stages)):return None
        # 原始相邻采样也检查符号，不让stage构造的数值零阈值累计吞掉微反向。
        # 原参考含反向时采取保守停稳接管，再从新的单向参考建立续接资格。
        for points in (self.reference.points,geometry.points):
            if any(sign*angle(b[2]-a[2])<0. for a,b in zip(points,points[1:])):return None
        if sign*state.angular_radps < -requested.stopped_angular_rps:return None
        current=remaining[0]
        start=current.path.final_yaw-current.spin_delta
        offset=sign*angle(state.yaw_rad-start)
        incoming_offset=sign*angle(geometry.points[0][2]-start)
        yaw_tolerance=requested.yaw_tolerance_rad
        # 新起点必须贴合实测，且仍在当前stage可达前缀内，不能跳到后续stage。
        if (not -yaw_tolerance<=offset<=abs(current.spin_delta)+yaw_tolerance or
                not -yaw_tolerance<=incoming_offset<=abs(current.spin_delta)+yaw_tolerance or
                abs(angle(geometry.points[0][2]-state.yaw_rad))>yaw_tolerance):return None
        current_end=abs(current.spin_delta)-offset
        old_end=current_end+sum(abs(stage.spin_delta) for stage in remaining[1:])
        # 相邻yaw逐段lift；覆盖旧同向承诺角度，不能用末首angle吞掉多圈。
        lifted=sign*angle(geometry.points[0][2]-state.yaw_rad)
        for a,b in zip(geometry.points,geometry.points[1:]):
            lifted+=sign*angle(b[2]-a[2])
        if lifted<old_end-1e-9:return None
        future=max(0.,lifted-current_end)
        if future<=1e-9:return ()
        # 当前未完成锚不动；未来同向采样按最新端点重新归并，避免每次
        # 20Hz延展留下一个短角停车屏障。累计方向、圈数及旧承诺角度不变。
        yaw=current.path.final_yaw
        points=[(*old_center,yaw)]
        while future>1e-9:
            delta=min(math.pi/2,future)
            yaw+=sign*delta;points.append((*old_center,yaw));future-=delta
        return StandardPathReference(points,self.policy).stages

    def update(self,state,dt,*,now_s=None,state_age_s=0.):
        contract=self._execution_pending
        active=self.execution_reference
        if contract is None and active is None:
            return self._update_standard(state,dt,now_s=now_s,state_age_s=state_age_s)
        policy=self.policy
        requested=contract[1] if contract is not None else active
        self.policy=self._execution_policy(requested)
        try:
            if contract is not None:
                geometry,requested=contract
                new_session=active is None or active.session_id!=requested.session_id
                extension=self._alignment_extension(state,geometry,active,requested)
                continuous_align=extension is not None
                recovering=self._stop_reason in ('WAITING_FOR_POSE','WAITING_FOR_MAP','WAITING_FOR_FRESH_REFERENCE')
                needs_stop=new_session or requested.intent!=0 or recovering
                certified=(self._execution_certified(state,dt,now_s,state_age_s)
                    if needs_stop else True)
                if needs_stop and (not certified or
                        ((new_session or recovering or (requested.intent==3 and not continuous_align))
                            and not self._stopped(state))):
                    result=self._stop(state,dt,now_s,state_age_s)
                    if result.phase=='WAITING':self.phase='BRAKING'
                    return self._execution_result(
                        result.command.linear_x_mps,result.command.angular_z_radps)
                if new_session:
                    if active is not None:
                        self._execution_retired[active.session_id]=None
                        while len(self._execution_retired)>256:
                            self._execution_retired.pop(next(iter(self._execution_retired)))
                    self._retire_execution_geometry()
                    self.failure=None;self._stop_reason=None
                if requested.intent!=0 and not continuous_align:
                    self._retire_execution_geometry()
                    if self._stop_reason in (None,'EXECUTION_PAIR_WAIT','STALE_PATH','NO_PATH',
                                             'WAITING_FOR_POSE','WAITING_FOR_MAP','WAITING_FOR_FRESH_REFERENCE'):
                        self._stop_reason=None
                if requested.intent==3 and not continuous_align:
                    # 退休旧平移屏障，保留新终端参考中的有序旋转和跨界方向。
                    x,y=geometry.points[-1][:2]
                    geometry=StandardPathReference(tuple((x,y,p[2]) for p in geometry.points),self.policy)
                if requested.intent in (0,3) and not continuous_align:
                    self._pending=geometry
                    self._accept_pending(state)
                    if requested.intent==3:self.phase='FINAL_ALIGN'
                if continuous_align:
                    # 固定当前stage与已执行前缀，仅重建未来同向表示片段。
                    self._stages[self._stage_index+1:]=extension
                self.execution_reference=requested;active=requested
                self._execution_pending=None
                # 只有配对参考实际接管才结束传输等待；手动停止与任务故障另行保留。
                if requested.intent==0 and self._stop_reason in (
                        'EXECUTION_PAIR_WAIT','STALE_PATH','NO_PATH','WAITING_FOR_POSE','WAITING_FOR_MAP','WAITING_FOR_FRESH_REFERENCE'):
                    self._stop_reason=None
            if self.failure is not None:
                result=self._stop(state,dt,now_s,state_age_s)
                self.phase='FAILED'
                return self._execution_result(result.command.linear_x_mps,result.command.angular_z_radps)
            if active.intent in (1,2,4):
                result=self._stop(state,dt,now_s,state_age_s)
                if not self._execution_certified(state,dt,now_s,state_age_s):self.phase='BRAKING'
                return self._execution_result(result.command.linear_x_mps,result.command.angular_z_radps)
            if active.intent==3:self.phase='FINAL_ALIGN'
            result=self._update_standard(state,dt,now_s=now_s,state_age_s=state_age_s)
            if active.intent==3 and abs(result.command.linear_x_mps)>1e-9:
                result=self._stop(state,dt,now_s,state_age_s)
                self.phase='BRAKING'
            if active.intent==3 and self.phase=='WAITING':
                self.phase='COMPLETED' if self._execution_certified(
                    state,dt,now_s,state_age_s) else 'BRAKING'
            return self._execution_result(result.command.linear_x_mps,result.command.angular_z_radps)
        finally:self.policy=policy

    def _execution_result(self,v=0.,w=0.):
        self.last_execution_state=6 if self.failure else {
            'WAITING':0,'BRAKING':1,'ALIGNING':2,'TRACKING':3,
            'FINAL_ALIGN':4,'COMPLETED':5,'FAILED':6}.get(self.phase,0)
        return self._result(v,w)
