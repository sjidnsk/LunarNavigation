"""独立冻结80b8948内核的逐位对照。"""
from types import SimpleNamespace as S
import numpy as np
import pytest
from lunar_pure_wheeled_controller import speed_slew as candidate
OLD_SOURCE='"""有符号线速度的无状态递推；加速/制动按模长定义，换向不附加等待。"""\nimport numpy as np\n\n\ndef speed_slew(current,target,dt,acceleration,deceleration):\n    """正能力及非负时间由调用者验证；支持NumPy广播和连续折点取值。"""\n    current=np.asarray(current);target=np.asarray(target);dt=np.asarray(dt)\n    # 对称能力退化为原限幅，保留既有数值运算顺序和快速路径。\n    if acceleration==deceleration:\n        return current+np.clip(target-current,-acceleration*dt,acceleration*dt)\n    rate=np.where(abs(target)>abs(current),acceleration,deceleration)\n    same_direction=current+np.clip(target-current,-rate*dt,rate*dt)\n    zero_time=abs(current)/deceleration\n    reversal=(np.sign(current)*np.maximum(abs(current)-deceleration*dt,0.)+\n              np.sign(target)*np.minimum(abs(target),acceleration*np.maximum(dt-zero_time,0.)))\n    return np.where(current*target<0.,reversal,same_direction)\n\n\ndef speed_slew_times(current,target,acceleration,deceleration):\n    """返回零速及目标速折点，供连续运动认证使用同一分段模型。"""\n    current=np.asarray(current);target=np.asarray(target)\n    reversing=current*target<0.\n    zero_time=abs(current)/deceleration\n    rate=np.where(abs(target)>abs(current),acceleration,deceleration)\n    goal_time=np.where(reversing,zero_time+abs(target)/acceleration,abs(target-current)/rate)\n    return np.where(reversing,zero_time,np.where(target==0.,goal_time,0.)),goal_time\n\n\n\ndef prepare_targets(raw,policy):\n    """一次批量处理与时间无关的速度/曲率意图限制。"""\n    v,w=raw[...,0],raw[...,1]\n    if policy.motion_model==\'differential_drive\':\n        v=np.clip(v,-policy.max_reverse_mps if policy.allow_reverse else 0.,policy.max_linear_mps)\n        w=np.clip(w,-policy.max_angular_radps,policy.max_angular_radps)\n    else:\n        bound=abs(v)*policy.max_curvature_per_m\n        w=np.where(abs(v)>policy.stopped_linear_mps,np.clip(w,-bound,bound),w)\n    return np.stack((v,w),axis=-1)\n\n\ndef target_step(previous,intent,dt,policy):\n    """单步和批量共用的双轴指令递推；物理执行器仍用各自实测初态。"""\n    p=policy;pv,pw=previous[:,0],previous[:,1]\n    v=speed_slew(pv,intent[:,0],dt,p.max_linear_accel_mps2,p.max_linear_decel_mps2)\n    tw=intent[:,1]\n    if p.motion_model==\'curvature_limited\':\n        tw=np.where((abs(intent[:,0])<=p.stopped_linear_mps)&(abs(pv)>p.stopped_linear_mps),0.,tw)\n    bound=p.max_angular_accel_radps2*dt\n    w=pw+np.clip(tw-pw,-bound,bound)\n    if p.motion_model==\'differential_drive\':\n        return np.stack((np.clip(v,-p.max_reverse_mps if p.allow_reverse else 0.,p.max_linear_mps),\n                         np.clip(w,-p.max_angular_radps,p.max_angular_radps)),axis=1)\n    lo=np.maximum(-abs(v)*p.max_curvature_per_m,pw-bound)\n    hi=np.minimum(abs(v)*p.max_curvature_per_m,pw+bound)\n    moving=abs(v)>p.stopped_linear_mps\n    w=np.where(moving&(lo<=hi),np.clip(w,lo,hi),w)\n    stop=moving&(lo>hi)\n    return np.stack((np.where(stop,0.,v),np.where(stop,0.,w)),axis=1)\n'
old=S();exec(OLD_SOURCE,old.__dict__)
@pytest.mark.parametrize('model',['differential_drive','curvature_limited'])
@pytest.mark.parametrize('accel,decel',[(.1,.5),(.5,.1),(.5,.5)])
@pytest.mark.parametrize('reverse',[True,False])
def test_frozen_kernel_exact(model,accel,decel,reverse):
 p=S(motion_model=model,max_linear_accel_mps2=accel,max_linear_decel_mps2=decel,max_angular_accel_radps2=.3,max_reverse_mps=.1,allow_reverse=reverse,max_linear_mps=.2,max_angular_radps=.4,max_curvature_per_m=1.,stopped_linear_mps=.001)
 rng=np.random.default_rng(20261031);edge=np.array([-np.inf,-.5,-.2,-.1,-.001,-0.,0.,.001,.1,.2,.5,np.inf,np.nan]);a,b=np.meshgrid(edge,edge)
 previous=np.concatenate((np.stack((a.ravel(),b.ravel()),axis=1),rng.uniform(-.6,.6,(512,2))));intent=previous[::-1].copy()
 for dt in (0.,.02,.1,.3,1.):
  with np.errstate(invalid='ignore'):
   for name,args in [('speed_slew',(previous[:,0],intent[:,0],dt,accel,decel)),('speed_slew_times',(previous[:,0],intent[:,0],accel,decel)),('prepare_targets',(intent,p)),('target_step',(previous,intent,dt,p))]:
    expected=getattr(old,name)(*args);actual=getattr(candidate,name)(*args)
    np.testing.assert_array_equal(actual,expected)
    np.testing.assert_array_equal(np.signbit(actual),np.signbit(expected))


def test_scalar_and_broadcast_time_exact():
 for current,target in [(0.,-.2),(-.01,.2),(.3,0.),(-0.,0.),(np.nan,.1)]:
  for dt in (.0,.02,.3,np.array([.0,.02,.3])):
   for accel,decel in ((.1,.5),(.5,.5)):
    expected=old.speed_slew(current,target,dt,accel,decel)
    actual=candidate.speed_slew(current,target,dt,accel,decel)
    np.testing.assert_array_equal(actual,expected)
    np.testing.assert_array_equal(np.signbit(actual),np.signbit(expected))


def test_common_kernel_removes_generic_clip_dispatch(monkeypatch):
 calls=[];clip=np.clip
 def record(*args,**kwargs):
  calls.append(1)
  return clip(*args,**kwargs)
 monkeypatch.setattr(np,'clip',record)
 x=np.array([-.1,0.,.1])
 old.speed_slew(x,-x,.05,.5,.5)
 assert calls==[1]
 calls.clear()
 candidate.speed_slew(x,-x,.05,.5,.5)
 assert calls==[]
