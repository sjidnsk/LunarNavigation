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
        certificate=certify(proof,result.grid,limits,candidate.route_version,backend=backend,check_braking_tail=False)
        if not certificate.matches(proof,result.grid,limits,candidate.route_version,check_braking_tail=False):
            return 0., 0.
        return float(current[3]),float(current[4])
    predicted=execute_action(action,measured)
    if not np.isfinite([predicted.v,predicted.w]).all():return 0.,0.
    return float(predicted.v), float(predicted.w)


def command_hold_proof(action, grid, limits, command_hold_s):
    """P3采样初态的模型证明：实际响应与有限末速持令，不附加停车尾。

    不补偿来源到发布之间的物理延迟，不代表绝对调度/执行安全保证。
    """
    if isinstance(command_hold_s,bool) or not np.isfinite(command_hold_s) or command_hold_s<=0:
        raise ValueError('command_hold_s must be explicitly finite and positive')
    current=action.states[-1].copy()
    times=list(action.times);states=list(action.states);controls=list(action.controls)
    remaining=float(command_hold_s)
    while remaining>1e-12:
        dt=min(.02,remaining)
        control=np.zeros(3)
        current=step(current,control,dt)
        times.append(times[-1]+dt);states.append(current.copy());controls.append(control)
        remaining-=dt
    return Trajectory(action.map_epoch,action.route_version,times,states,controls,action.mode,action.source)


def certified_velocity_command(result, measured, limits, *, reference_interval_s,
                               last_command, elapsed_s, command_hold_s, backend=None, diagnostics=None):
    """首 shooting 区间速度参考；发布记忆只决定 slew。

    证明起于冻结周期的 P3 实测或命令响应模型估计初态；估计不是实测到达证据。
    """
    from time import perf_counter
    started=perf_counter()
    if diagnostics is not None:
        diagnostics.clear()
    def record(**values):
        if diagnostics is not None: diagnostics.update(values)
    def finish(command, reason=None):
        record(command_reject_reason=reason, command_total_ms=(perf_counter()-started)*1000)
        return command
    from .certification import certify
    from .contracts import LocalReference
    from .seeds import braking_prefix, join_trajectories
    if result is None or measured is None or result.candidate is None:
        return finish((0.,0.), 'MISSING_RESULT_STATE_OR_CANDIDATE')
    candidate=result.candidate
    phase = getattr(result, 'reference_phase_s', None)
    record(reference_phase_s=phase, source_initial_vw=candidate.states[0,3:5].tolist(),
           command_elapsed_s=elapsed_s)
    if (candidate.source=='brake' or getattr(result,'status',None)=='ARRIVED' or
            phase is None and (result.certificate is None or not result.certificate.matches(
                candidate,result.grid,limits,candidate.route_version))):
        return finish((0.,0.), 'BRAKE_SOURCE' if candidate.source=='brake' else
                      'ARRIVED' if getattr(result,'status',None)=='ARRIVED' else 'SOURCE_CERTIFICATE_MISMATCH')
    if not np.isfinite([reference_interval_s,elapsed_s]).all() or min(reference_interval_s,elapsed_s)<=0:
        return finish((0.,0.), 'INVALID_COMMAND_INTERVAL')
    try:
        if phase is None:
            reference=action_prefix(candidate,measured,reference_interval_s).states[-1,3:5]
        else:
            from .reference_buffer import _at
            if (not np.isfinite(phase) or phase < 0 or phase >= candidate.times[-1] or
                    candidate.mode not in ('forward','reverse','spin')):
                return finish((0.,0.), 'INVALID_REFERENCE_PHASE_OR_MODE')
            reference=_at(candidate,min(float(candidate.times[-1]),phase+reference_interval_s))[0][3:5]
    except ValueError:
        return finish((0.,0.), 'INVALID_REFERENCE')
    previous=np.asarray(last_command,dtype=float)
    if previous.shape!=(2,) or not np.isfinite(previous).all():return finish((0.,0.), 'INVALID_PREVIOUS_COMMAND')
    record(previous_vw=previous.tolist())
    # 反向先归零，避免把一个周期跨过零速的目标解释为瞬时反向。
    target=reference.copy()
    record(target_raw_vw=target.tolist())
    target=np.where(previous*target<0.,0.,target)
    record(target_vw=target.tolist())
    rates=np.array([limits.linear_accel_mps2,limits.angular_accel_rps2])
    issued=previous+np.clip(target-previous,-rates*elapsed_s,rates*elapsed_s)
    record(issued_vw=issued.tolist())
    construction_started=perf_counter()
    current=np.array([measured.x,measured.y,measured.yaw,measured.v,measured.w,candidate.states[0,5]])
    mode='spin' if abs(issued[0])<1e-10 else ('forward' if issued[0]>0 else 'reverse')
    ref=LocalReference(candidate.route_version,result.grid.epoch,[[current[0],current[1]]],[0.],current[5])
    prefix=None
    if (mode=='spin' and abs(current[3])>1e-10 or current[3]*issued[0]<0.):
        prefix=braking_prefix(measured,result.grid,ref,limits,retain_angular=mode=='spin')
        current=prefix.states[-1].copy()
    duration=float(np.max(np.abs(issued-current[3:5])/rates))
    states=[current.copy()];controls=[];times=[0.]
    if duration>1e-12:
        control=np.array([(issued[0]-current[3])/duration,(issued[1]-current[4])/duration,0.])
        remaining=duration
        while remaining>1e-12:
            dt=min(.02,remaining);current=step(current,control,dt)
            states.append(current.copy());controls.append(control.copy());times.append(times[-1]+dt)
            remaining-=dt
    response=Trajectory(candidate.map_epoch,candidate.route_version,times,states,
        np.asarray(controls).reshape(-1,3),mode,candidate.source)
    if prefix is not None:response=join_trajectories(prefix,response)
    proof=command_hold_proof(response,result.grid,limits,command_hold_s)
    record(command_proof_construction_ms=(perf_counter()-construction_started)*1000)
    certification_started=perf_counter()
    certificate=certify(proof,result.grid,limits,candidate.route_version,backend=backend,check_braking_tail=False)
    record(command_certification_ms=(perf_counter()-certification_started)*1000,
           command_certificate_reason=certificate.reason.value)
    if not certificate.matches(proof,result.grid,limits,candidate.route_version,check_braking_tail=False):
        return finish((0.,0.), 'ISSUED_CERTIFICATE_MISMATCH')
    return finish((float(issued[0]),float(issued[1])))
