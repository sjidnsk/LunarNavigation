"""有符号线速度的无状态递推；加速/制动按模长定义，换向不附加等待。"""
import numpy as np

# 高频小数组直接使用 ndarray.clip，省去通用分派并保留原限幅内核。


def speed_slew(current,target,dt,acceleration,deceleration):
    """正能力及非负时间由调用者验证；支持NumPy广播和连续折点取值。"""
    current=np.asarray(current);target=np.asarray(target);dt=np.asarray(dt)
    # 对称能力退化为原限幅，保留既有数值运算顺序和快速路径。
    if acceleration==deceleration:
        return current+(target-current).clip(-acceleration*dt,acceleration*dt)
    rate=np.where(abs(target)>abs(current),acceleration,deceleration)
    same_direction=current+(target-current).clip(-rate*dt,rate*dt)
    zero_time=abs(current)/deceleration
    reversal=(np.sign(current)*np.maximum(abs(current)-deceleration*dt,0.)+
              np.sign(target)*np.minimum(abs(target),acceleration*np.maximum(dt-zero_time,0.)))
    return np.where(current*target<0.,reversal,same_direction)


def speed_slew_times(current,target,acceleration,deceleration):
    """返回零速及目标速折点，供连续运动认证使用同一分段模型。"""
    current=np.asarray(current);target=np.asarray(target)
    reversing=current*target<0.
    zero_time=abs(current)/deceleration
    rate=np.where(abs(target)>abs(current),acceleration,deceleration)
    goal_time=np.where(reversing,zero_time+abs(target)/acceleration,abs(target-current)/rate)
    return np.where(reversing,zero_time,np.where(target==0.,goal_time,0.)),goal_time



def prepare_targets(raw,policy):
    """一次批量处理与时间无关的速度/曲率意图限制。"""
    v,w=raw[...,0],raw[...,1]
    if policy.motion_model=='differential_drive':
        v=v.clip(-policy.max_reverse_mps if policy.allow_reverse else 0.,policy.max_linear_mps)
        w=w.clip(-policy.max_angular_radps,policy.max_angular_radps)
    else:
        bound=abs(v)*policy.max_curvature_per_m
        w=np.where(abs(v)>policy.stopped_linear_mps,w.clip(-bound,bound),w)
    return np.stack((v,w),axis=-1)


def target_step(previous,intent,dt,policy):
    """单步和批量共用的双轴指令递推；物理执行器仍用各自实测初态。"""
    p=policy;pv,pw=previous[:,0],previous[:,1]
    v=speed_slew(pv,intent[:,0],dt,p.max_linear_accel_mps2,p.max_linear_decel_mps2)
    tw=intent[:,1]
    if p.motion_model=='curvature_limited':
        tw=np.where((abs(intent[:,0])<=p.stopped_linear_mps)&(abs(pv)>p.stopped_linear_mps),0.,tw)
    bound=p.max_angular_accel_radps2*dt
    w=pw+(tw-pw).clip(-bound,bound)
    if p.motion_model=='differential_drive':
        return np.stack((v.clip(-p.max_reverse_mps if p.allow_reverse else 0.,p.max_linear_mps),
                         w.clip(-p.max_angular_radps,p.max_angular_radps)),axis=1)
    lo=np.maximum(-abs(v)*p.max_curvature_per_m,pw-bound)
    hi=np.minimum(abs(v)*p.max_curvature_per_m,pw+bound)
    moving=abs(v)>p.stopped_linear_mps
    w=np.where(moving&(lo<=hi),w.clip(lo,hi),w)
    stop=moving&(lo>hi)
    return np.stack((np.where(stop,0.,v),np.where(stop,0.,w)),axis=1)
