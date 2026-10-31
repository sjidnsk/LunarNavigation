"""离线固定周期前缀：仅调用公开动力学 step，不外推制动段控制。"""
import numpy as np
from .contracts import Trajectory, VehicleState
from .dynamics import step


def action_prefix(candidate, measured, duration_s=.05):
    observed = np.array([measured.x, measured.y, measured.yaw, measured.v, measured.w])
    if not np.allclose(candidate.states[0, :5], observed, atol=1e-6, rtol=0):
        raise ValueError('INITIAL_STATE_MISMATCH')
    current = candidate.states[0].copy()
    states, controls, times = [current.copy()], [], [0.]
    for index, control in enumerate(candidate.controls):
        remaining = duration_s-times[-1]
        if remaining <= 1e-12: break
        dt = min(float(candidate.times[index+1]-candidate.times[index]), remaining)
        current = step(current, control, dt)
        states.append(current.copy()); controls.append(control); times.append(times[-1]+dt)
    remaining = duration_s-times[-1]
    if remaining > 1e-12:
        if abs(current[3]) > 1e-10 or abs(current[4]) > 1e-10:
            raise ValueError('moving candidate does not cover execution period')
        control = np.zeros(3)
        current = step(current, control, remaining)
        states.append(current); controls.append(control); times.append(duration_s)
    return Trajectory(candidate.map_epoch, candidate.route_version, times, states,
                      controls, candidate.mode, candidate.source)


def execute_action(action, measured):
    """调用方已认证 action；返回纯模型预测状态（不是实测反馈），不直接复制候选末态。"""
    current = np.array([measured.x, measured.y, measured.yaw, measured.v, measured.w, action.states[0, 5]])
    if not np.allclose(current[:5], action.states[0,:5], atol=1e-6, rtol=0):
        raise ValueError('INITIAL_STATE_MISMATCH')
    for index, control in enumerate(action.controls):
        current = step(current, control, float(action.times[index+1]-action.times[index]))
    return VehicleState(*current[:5])


def certified_command(result, measured, limits, *, command_hold_s=None, backend=None):
    """仅将已认证的当前短动作积分为速度设定值；不写回在线定位。"""
    if result is None or measured is None:
        return 0., 0.
    candidate, action = result.candidate, result.action
    if candidate is None or action is None or result.certificate is None or result.action_certificate is None:
        return 0., 0.
    if (not result.certificate.matches(candidate, result.grid, limits, candidate.route_version) or
            not result.action_certificate.matches(action, result.grid, limits, candidate.route_version)):
        return 0., 0.
    observed=np.array([measured.x,measured.y,measured.yaw,measured.v,measured.w])
    if not np.allclose(observed,action.states[0,:5],atol=1e-6,rtol=0):
        return 0.,0.
    if command_hold_s is not None:
        from .certification import certify
        # 容差内实测也不得借用候选末态：从同一真实采样初态重积分短段。
        current=np.array([measured.x,measured.y,measured.yaw,measured.v,measured.w,action.states[0,5]])
        states=[current.copy()]
        for index,control in enumerate(action.controls):
            current=step(current,control,float(action.times[index+1]-action.times[index]))
            states.append(current.copy())
        integrated=Trajectory(action.map_epoch,action.route_version,action.times,states,
            action.controls,action.mode,action.source)
        proof=command_hold_proof(integrated,result.grid,limits,command_hold_s)
        certificate=certify(proof,result.grid,limits,candidate.route_version,backend=backend)
        if not certificate.matches(proof,result.grid,limits,candidate.route_version):
            return 0., 0.
        return float(current[3]),float(current[4])
    predicted=execute_action(action,measured)
    if not np.isfinite([predicted.v,predicted.w]).all():return 0.,0.
    return float(predicted.v), float(predicted.w)


def command_hold_proof(action, grid, limits, command_hold_s):
    """P3采样初态的模型证明：短动作、有限末速持令、显式停车尾。

    不补偿来源到发布之间的物理延迟，不代表绝对调度/执行安全保证。
    """
    if isinstance(command_hold_s,bool) or not np.isfinite(command_hold_s) or command_hold_s<=0:
        raise ValueError('command_hold_s must be explicitly finite and positive')
    from .contracts import LocalReference
    from .seeds import brake_candidate, join_trajectories
    current=action.states[-1].copy()
    times=list(action.times);states=list(action.states);controls=list(action.controls)
    remaining=float(command_hold_s)
    while remaining>1e-12:
        dt=min(.02,remaining)
        control=np.zeros(3)
        current=step(current,control,dt)
        times.append(times[-1]+dt);states.append(current.copy());controls.append(control)
        remaining-=dt
    held=Trajectory(action.map_epoch,action.route_version,times,states,controls,action.mode,action.source)
    reference=LocalReference(action.route_version,grid.epoch,[[current[0],current[1]]],[0.],current[5])
    brake=brake_candidate(VehicleState(*current[:5]),grid,reference,limits)
    from dataclasses import replace
    return replace(join_trajectories(held,brake),mode=action.mode,source=action.source)
