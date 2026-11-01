"""P3实测运动的有界延迟预测；命令辅助减速，不作为已实现速度。"""
from collections import deque
import math
from nav1cm.contracts import VehicleState


class CommandPrediction:
    def __init__(self, max_commands=512, spin_prepare_s=0.):
        if isinstance(spin_prepare_s,bool) or not math.isfinite(spin_prepare_s) or spin_prepare_s<0:
            raise ValueError('spin_prepare_s must be finite and nonnegative')
        self.spin_prepare_s=float(spin_prepare_s)
        self._spin_dir=0
        self._spin_ready_s=None
        self.commands=deque(maxlen=max_commands)
        self.state=None
        self.stamp=None
        self.sample_s=None
        self.clock=None
        self._cache=None
        self._observed_velocity=None

    def issue(self, now_s, command):
        command=tuple(command)
        if self.commands and now_s < self.commands[-1][0]:
            self.commands.clear()
            self.sample_s=None
            self._cache=None
            self._spin_dir=0
            self._spin_ready_s=None
        # Match the deployed chassis: sign changes restart preparation; magnitude does not.
        spin_dir=(1 if command[1]>0 else -1) if abs(command[0])<1e-9 and abs(command[1])>1e-9 else 0
        if spin_dir != self._spin_dir:
            self._spin_ready_s=now_s+self.spin_prepare_s if spin_dir else None
            self._spin_dir=spin_dir
        if not self.commands or command != self.commands[-1][1]:
            self.commands.append((now_s,command,self._spin_ready_s))

    def correct(self, state, stamp_ns, now_s, clock_ns):
        if self.clock is not None and clock_ns < self.clock:
            self.commands.clear()
            self._spin_dir=0
            self._spin_ready_s=None
            self._cache=None
            self.state=state
            self._observed_velocity=(state.v,state.w)
            self.stamp=stamp_ns
            self.sample_s=None
            self.clock=clock_ns
            return
        self.clock=clock_ns
        if self.stamp is not None and stamp_ns<=self.stamp:
            return
        velocity=(state.v,state.w)
        if self.state is not None and self.stamp is not None:
            dt=(stamp_ns-self.stamp)/1e9
            yaw_delta=math.atan2(math.sin(state.yaw-self.state.yaw),math.cos(state.yaw-self.state.yaw))
            mid=self.state.yaw+yaw_delta/2
            chord_scale=1. if abs(yaw_delta)<1e-9 else 2*math.sin(yaw_delta/2)/yaw_delta
            forward=((state.x-self.state.x)*math.cos(mid)+(state.y-self.state.y)*math.sin(mid))/(dt*chord_scale)
            velocity=(forward,yaw_delta/dt)
        self.state=state
        self._observed_velocity=velocity
        self.stamp=stamp_ns
        self._cache=None
        age=(clock_ns-stamp_ns)/1e9
        self.sample_s=now_s-age if age>=0 else None

    def covered(self, now_s):
        return (self.state is not None and self.sample_s is not None and
                0<=now_s-self.sample_s<=30 and bool(self.commands) and
                self.commands[0][0]<=self.sample_s)

    def predict(self, now_s, limits):
        # 没有覆盖采样时刻的已发命令就保留实测，不推测此前命令。
        start=self.sample_s
        if (self.state is None or start is None or now_s<start or now_s-start>30 or
                not self.commands or self.commands[0][0]>start):
            return self.state
        state=self.state
        cached=self._cache is not None and start<=self._cache[0]<=now_s
        if cached:
            start,state=self._cache
        x,y,yaw,v,w=(state.x,state.y,state.yaw,state.v,state.w)
        if not cached:
            v,w=self._observed_velocity
        entries=list(self.commands)
        for i,(issued,command,ready_s) in enumerate(entries):
            end=min(now_s,entries[i+1][0] if i+1<len(entries) else now_s)
            begin=max(start,issued)
            duration=end-begin
            if duration<=0:continue
            # 每段积分以速度到达目标的时刻分割，中点unicycle，最大20ms。
            while duration>1e-10:
                dt=min(.02,duration)
                cursor=end-duration
                preparing=ready_s is not None and cursor<ready_s-1e-10
                # 已发指令只说明请求变化；没有P3运动证据就不预测自行加速。
                # 停车/反向先按已有减速度减到零，反向运动由随后P3确认。
                target_command=((0.,0.) if preparing else tuple(
                    math.copysign(min(abs(observed),abs(requested)),observed)
                    if observed*requested>0 else 0.
                    for observed,requested in zip((v,w),command)))
                if preparing:dt=min(dt,ready_s-cursor)
                for speed,target,rate in ((v,target_command[0],limits.linear_accel_mps2),
                                          (w,target_command[1],limits.angular_accel_rps2)):
                    reach=abs(target-speed)/rate
                    if reach>1e-10:dt=min(dt,reach)
                dv=max(-limits.linear_accel_mps2*dt,min(limits.linear_accel_mps2*dt,target_command[0]-v))
                dw=max(-limits.angular_accel_rps2*dt,min(limits.angular_accel_rps2*dt,target_command[1]-w))
                vm,wm=v+dv/2,w+dw/2
                x+=vm*math.cos(yaw+wm*dt/2)*dt
                y+=vm*math.sin(yaw+wm*dt/2)*dt
                yaw+=wm*dt
                v+=dv; w+=dw
                duration-=dt
        value=VehicleState(x,y,yaw,v,w)
        self._cache=(now_s,value)
        return value
