from types import SimpleNamespace
from test_standard_path_follower import follower, tick
from lunar_pure_wheeled_controller.tracking import TrackingState


def ref(revision=1, intent=0, session='a', **changes):
    values=dict(session_id=session, revision=revision, path_digest=str(revision), intent=intent,
        stamp_ns=revision, position_tolerance_m=.1, yaw_tolerance_rad=.05,
        stopped_linear_mps=.01, stopped_angular_rps=.01)
    values.update(changes)
    return SimpleNamespace(**values)


def test_received_is_not_accepted_and_revisions_cannot_mutate():
    e=follower(); path=((0.,0.,0.),(1.,0.,0.)); r=ref()
    assert e.offer_execution_reference(path,r)
    assert e.execution_reference is None and not e.reference_accepted
    assert not e.offer_execution_reference(path,ref(path_digest='changed'))
    tick(e,TrackingState(0.,0.,0.))
    assert e.execution_reference is r and e.reference_accepted
    assert not e.offer_execution_reference(path,ref(revision=0))


def test_hold_retires_old_translation_without_chasing_micro_path():
    e=follower(); e.offer_execution_reference(((0.,0.,0.),(1.,0.,0.)),ref())
    tick(e,TrackingState(0.,0.,0.))
    r=ref(2,2); e.offer_execution_reference(((0.,0.,0.),(.00045,0.,0.)),r)
    result=tick(e,TrackingState(0.,0.,0.))
    assert e.execution_reference is r and e.path is None and not e._stages
    assert result.command.linear_x_mps==0. and result.phase=='WAITING'


def test_final_align_requires_measured_stop_and_independent_certificate():
    e=follower(); e.offer_execution_reference(((0.,0.,0.),(1.,0.,0.)),ref())
    tick(e,TrackingState(0.,0.,0.)); e.note_command(.1,0.)
    history=list(e._command_history)
    r=ref(2,3); e.offer_execution_reference(((.00045,0.,0.),(.00045,0.,1.)),r)
    result=e.update(TrackingState(0.,0.,0.,.1),.05)
    assert result.phase=='BRAKING' and e.execution_reference is not r
    assert len(e._command_history)==len(history)
    e._brake_safe=lambda *a: [False]
    e.update(TrackingState(0.,0.,0.),.05)
    assert e.execution_reference is not r
    e._brake_safe=lambda *a: [True]
    result=e.update(TrackingState(0.,0.,0.),.05)
    assert e.execution_reference is r and all(s.direction==0 for s in e._stages)
    assert result.phase=='BRAKING' and 0.<=result.command.linear_x_mps<=.1


def test_failure_latched_until_new_session_certified_stop():
    e=follower(); path=((0.,0.,0.),(1.,0.,0.))
    e.offer_execution_reference(path,ref()); tick(e,TrackingState(0.,0.,0.))
    e.failure='TRACKING_STALLED'
    e.offer_execution_reference(path,ref(2)); tick(e,TrackingState(0.,0.,0.))
    assert e.failure=='TRACKING_STALLED' and e.last_execution_state==6
    e.offer_execution_reference(path,ref(session='b'))
    e._brake_safe=lambda *a: [False]
    e.update(TrackingState(0.,0.,0.),.05)
    assert e.failure=='TRACKING_STALLED'
    e._brake_safe=lambda *a: [True]
    e.update(TrackingState(0.,0.,0.),.05)
    assert e.failure is None and e.execution_reference.session_id=='b'
    assert not e.offer_execution_reference(path,ref(3,session='a'))


def test_owner_position_set_survives_multiple_terminal_rotations():
    e=follower(); r=ref(intent=3)
    e.offer_execution_reference(((0.,0.,0.),(0.,0.,1.),(0.,0.,2.)),r)
    result=e.update(TrackingState(.08,0.,0.),.05)
    assert result.command.linear_x_mps==0. and result.command.angular_z_radps>0.
    result=e.update(TrackingState(.08,0.,1.),.05)
    assert e._stage_index==1
    result=e.update(TrackingState(.08,0.,1.),.05)
    assert result.command.linear_x_mps==0. and result.command.angular_z_radps>0.


def test_terminal_angle_wrap_and_completion():
    import math
    e=follower(); r=ref(intent=3)
    e.offer_execution_reference(((0.,0.,math.pi-.03),(0.,0.,-math.pi+.03)),r)
    result=e.update(TrackingState(0.,0.,math.pi-.03),.05)
    assert result.command.angular_z_radps>0. and result.command.linear_x_mps==0.
    result=e.update(TrackingState(0.,0.,-math.pi+.03),.05)
    assert e.last_execution_state==5 and result.phase=='COMPLETED'


def test_pending_new_session_brakes_and_preserves_actual_identity():
    e=follower(); path=((0.,0.,0.),(1.,0.,0.)); old=ref()
    e.offer_execution_reference(path,old); e.update(TrackingState(0.,0.,0.),.05)
    e.offer_execution_reference(path,ref(session='b'))
    result=e.update(TrackingState(0.,0.,0.,.1),.05)
    assert result.phase=='BRAKING' and e.execution_reference is old
    assert not e.reference_accepted


def test_follow_keeps_unfinished_real_yaw_barrier():
    import math
    e=follower(); e.offer_execution_reference(((0.,0.,0.),(0.,0.,math.pi/2),(0.,1.,math.pi/2)),ref())
    e.update(TrackingState(0.,0.,0.),.05)
    e.offer_execution_reference(((0.,0.,math.pi/2),(0.,1.,math.pi/2)),ref(2))
    result=e.update(TrackingState(0.,0.,0.),.05)
    assert result.command.linear_x_mps==0. and result.command.angular_z_radps>0.
    assert e._stages[0].direction==0


def test_unsafe_stop_request_cannot_claim_terminal_acceptance():
    e=follower(); path=((0.,0.,0.),(1.,0.,0.)); old=ref()
    e.offer_execution_reference(path,old); e.update(TrackingState(0.,0.,0.),.05)
    e.offer_execution_reference(((0.,0.,0.),),ref(2,1))
    e._brake_safe=lambda *a: [False]
    result=e.update(TrackingState(0.,0.,0.),.05)
    assert e.execution_reference is old and not e.reference_accepted
    assert result.phase=='BRAKING' and e.last_execution_state==1


def test_external_stop_cancels_pending_takeover():
    e=follower(); path=((0.,0.,0.),(1.,0.,0.))
    e.offer_execution_reference(path,ref())
    e.request_stop('REFERENCE_EXPIRED')
    result=e.update(TrackingState(0.,0.,0.),.05)
    assert e.execution_reference is None and result.command.linear_x_mps==0.



def test_stop_intents_allow_empty_geometry_but_motion_intents_do_not():
    for intent in (1,2,4):
        e=follower(); r=ref(intent=intent)
        assert e.offer_execution_reference((),r)
        result=e.update(TrackingState(.7,.4,1.),.05)
        assert e.execution_reference is r and e.path is None and not e._stages
        assert result.phase=='WAITING' and result.command.linear_x_mps==0.
    for intent in (0,3):
        assert not follower().offer_execution_reference((),ref(intent=intent))


def test_pending_session_returns_actual_bounded_stop_command():
    e=follower(); path=((0.,0.,0.),(1.,0.,0.))
    e.offer_execution_reference(path,ref()); e.update(TrackingState(0.,0.,0.),.05)
    e.note_command(.1,0.)
    e.offer_execution_reference(path,ref(session='b'))
    result=e.update(TrackingState(0.,0.,0.,.1),.05)
    assert result.phase=='BRAKING' and 0.<result.command.linear_x_mps<.1
    assert e.execution_reference.session_id=='a' and not e.reference_accepted


def test_identity_registries_are_bounded_and_keep_recent_retirements():
    e=follower(); e._brake_safe=lambda *a: [True]
    for i in range(300):
        r=ref(session=str(i),intent=2)
        assert e.offer_execution_reference(((0.,0.,0.),),r)
        e.update(TrackingState(0.,0.,0.),.05)
    assert len(e._execution_seen)<=128 and len(e._execution_retired)<=256
    assert not e.offer_execution_reference(((0.,0.,0.),),ref(2,session='298'))
    # 未接受的新身份洪泛也不能挤掉当前实际接受身份的revision保护。
    for i in range(400,600):
        assert e.offer_execution_reference(((0.,0.,0.),),ref(session=str(i),intent=2))
    assert len(e._execution_seen)<=128
    assert not e.offer_execution_reference(((0.,0.,0.),),ref(session='299',intent=2))



def test_final_align_rolling_same_goal_preserves_rotation():
    e=follower(); path=((0.,0.,0.),(0.,0.,1.))
    e.offer_execution_reference(path,ref(intent=3)); e.update(TrackingState(0.,0.,0.),.05)
    e.note_command(0.,.1)
    history=list(e._command_history)
    e.offer_execution_reference(((0.,0.,.01),(0.,0.,1.)),ref(2,3))
    result=e.update(TrackingState(0.,0.,.01,0.,.1),.05)
    assert result.phase=='FINAL_ALIGN' and result.command.angular_z_radps>=.1
    assert e.execution_reference.revision==2 and e.reference_accepted
    assert len(e._command_history)==len(history)


def test_final_align_changed_goal_or_owner_set_still_requires_stop():
    for path,changes in [(((0.,0.,.01),(0.,0.,-.2)),{}),
                         (((.2,0.,.01),(.2,0.,1.)),{}),
                         (((0.,0.,.01),(0.,0.,1.)),dict(position_tolerance_m=.08))]:
        e=follower(); e.offer_execution_reference(((0.,0.,0.),(0.,0.,1.)),ref(intent=3))
        e.update(TrackingState(0.,0.,0.),.05); e.note_command(0.,.1)
        e.offer_execution_reference(path,ref(2,3,**changes))
        result=e.update(TrackingState(0.,0.,.01,0.,.1),.05)
        assert result.phase=='BRAKING' and not e.reference_accepted
        assert e.execution_reference.revision==1



def test_final_align_small_odom_reanchor_keeps_rotation():
    e=follower(); e.offer_execution_reference(((0.,0.,0.),(0.,0.,1.)),ref(intent=3))
    e.update(TrackingState(0.,0.,0.),.05); e.note_command(0.,.1)
    e.offer_execution_reference(((.0001,0.,.01),(.0001,0.,1.)),ref(2,3))
    result=e.update(TrackingState(.0001,0.,.01,0.,.1),.05)
    assert result.phase=='FINAL_ALIGN' and result.command.angular_z_radps>=.1
    assert e.execution_reference.revision==2 and e.reference_accepted


def test_final_align_reanchor_outside_owner_set_requires_stop():
    for center,actual in [(.1001,0.),(.05,-.051),(.05,.1001)]:
        e=follower(); e.offer_execution_reference(((0.,0.,0.),(0.,0.,1.)),ref(intent=3))
        e.update(TrackingState(0.,0.,0.),.05); e.note_command(0.,.1)
        e.offer_execution_reference(((center,0.,.01),(center,0.,1.)),ref(2,3))
        result=e.update(TrackingState(actual,0.,.01,0.,.1),.05)
        assert result.phase=='BRAKING' and not e.reference_accepted



def test_accepted_follow_recovers_transport_stop_only_after_takeover():
    for reason in ('EXECUTION_PAIR_WAIT','STALE_PATH','NO_PATH'):
        e=follower(); path=((0.,0.,0.),(1.,0.,0.))
        e.offer_execution_reference(path,ref()); e.update(TrackingState(0.,0.,0.),.05)
        e.request_stop(reason)
        assert e.offer_execution_reference(path,ref(2)) and e._stop_reason==reason
        result=e.update(TrackingState(0.,0.,0.),.05)
        assert e.execution_reference.revision==2 and e._stop_reason is None
        assert result.phase=='TRACKING' and result.command.linear_x_mps>0.


def test_follow_takeover_preserves_user_stop_and_task_failure():
    path=((0.,0.,0.),(1.,0.,0.))
    e=follower(); e.offer_execution_reference(path,ref())
    e.update(TrackingState(0.,0.,0.),.05); e.request_stop('USER_STOP')
    e.offer_execution_reference(path,ref(2))
    result=e.update(TrackingState(0.,0.,0.),.05)
    assert e._stop_reason=='USER_STOP' and result.command.linear_x_mps==0.
    e=follower(); e.offer_execution_reference(path,ref())
    e.update(TrackingState(0.,0.,0.),.05); e.failure='TRACKING_STALLED'
    e.request_stop('EXECUTION_PAIR_WAIT'); e.offer_execution_reference(path,ref(2))
    result=e.update(TrackingState(0.,0.,0.),.05)
    assert e.failure=='TRACKING_STALLED' and e.last_execution_state==6
    assert result.command.linear_x_mps==0.



def spin_path(yaws, x=0.):
    import math
    return tuple((x,0.,math.atan2(math.sin(y),math.cos(y))) for y in yaws)


def test_dynamic_horizon_refresh_preserves_current_barrier():
    e=follower(); e.offer_execution_reference(spin_path([0.,.3,.6]),ref(intent=3))
    e.update(TrackingState(0.,0.,0.),.05); e.note_command(0.,.1)
    old=tuple(e._stages); history=list(e._command_history)
    for revision in range(2,12):
        yaw=.001*revision
        e.offer_execution_reference(spin_path([yaw,.3,.6,.6+yaw]),ref(revision,3))
        result=e.update(TrackingState(0.,0.,yaw,0.,.1),.05)
        assert result.phase=='FINAL_ALIGN' and result.command.angular_z_radps>=.1
        assert e.reference_accepted and e.execution_reference.revision==revision
        assert tuple(e._stages[:len(old)])==old and e._stage_index==0
        assert len(e._command_history)==len(history)
    assert e._stages[-1].path.final_yaw>.6


def test_continuation_lifts_across_pi_and_preserves_multiturn_commitment():
    import math
    for initial,end in [(3.1,3.5),(0.,7.)]:
        old_yaws=[initial+i*.2 for i in range(int(round((end-initial)/.2))+1)]
        if old_yaws[-1]<end:old_yaws.append(end)
        e=follower(); e.offer_execution_reference(spin_path(old_yaws),ref(intent=3))
        e.update(TrackingState(0.,0.,initial),.05); e.note_command(0.,.1)
        old=tuple(e._stages)
        new_yaws=[initial+.01]+old_yaws[1:]+[end+.1]
        e.offer_execution_reference(spin_path(new_yaws),ref(2,3))
        result=e.update(TrackingState(0.,0.,initial+.01,0.,.1),.05)
        assert result.phase=='FINAL_ALIGN' and e.reference_accepted
        assert e._stages[0] is old[0]
        assert sum(s.spin_delta for s in e._stages)>sum(s.spin_delta for s in old)


def test_rolling_spin_rejects_reverse_incomplete_coverage_and_skipped_prefix():
    cases=[([0.,.4],[.01,.5,.2]),([0.,.4,.2],[.01,.5]),
           ([0.,1.,2.,3.,4.,5.,6.,7.],[.01,.8]),
           ([0.,.5,1.,1.5,2.],[1.7,2.,2.2])]
    for old_yaws,new_yaws in cases:
        e=follower(); e.offer_execution_reference(spin_path(old_yaws),ref(intent=3))
        e.update(TrackingState(0.,0.,0.),.05); e.note_command(0.,.1)
        old=tuple(e._stages)
        actual=new_yaws[0]
        e.offer_execution_reference(spin_path(new_yaws),ref(2,3))
        result=e.update(TrackingState(0.,0.,actual,0.,.1),.05)
        assert result.phase=='BRAKING' and not e.reference_accepted
        assert tuple(e._stages)==old



def test_continuation_preserves_nonzero_cursor_and_physical_clocks():
    e=follower(); e.offer_execution_reference(spin_path([0.,1.,2.,3.,4.]),ref(intent=3))
    e.update(TrackingState(0.,0.,0.),.05)
    e.update(TrackingState(0.,0.,1.),.05)
    assert e._stage_index==1
    e.note_command(0.,.1); old=tuple(e._stages); history=list(e._command_history)
    e.no_progress_s=.7; anchor=e._last_pose
    e._update_standard=lambda *a,**k: e._result(0.,.1)
    e.offer_execution_reference(spin_path([1.01,2.,3.,4.,4.1]),ref(2,3))
    e.update(TrackingState(0.,0.,1.01,0.,.1),.05)
    assert e.reference_accepted and e._stage_index==1
    assert tuple(e._stages[:2])==old[:2]
    assert sum(s.spin_delta for s in e._stages[1:])>=sum(s.spin_delta for s in old[1:])
    assert e.no_progress_s==.7 and e._last_pose is anchor
    assert len(e._command_history)==len(history)


def test_continuation_rejects_actual_reverse_velocity_and_multiturn_reversal():
    for new_yaws,w in [([.01,.5],-.1),([.01,1.,2.,3.,4.,5.,6.,7.,6.8],.1)]:
        e=follower(); e.offer_execution_reference(spin_path([0.,.4]),ref(intent=3))
        e.update(TrackingState(0.,0.,0.),.05); e.note_command(0.,w)
        e.offer_execution_reference(spin_path(new_yaws),ref(2,3))
        result=e.update(TrackingState(0.,0.,.01,0.,w),.05)
        assert result.phase=='BRAKING' and not e.reference_accepted



def test_hundred_horizon_refreshes_do_not_accumulate_short_stop_barriers():
    import math
    e=follower(); e.offer_execution_reference(spin_path([0.,.4]),ref(intent=3))
    e.update(TrackingState(0.,0.,0.),.05); e.note_command(0.,.1)
    anchor=e._stages[0]
    for revision in range(2,102):
        yaw=.001*revision;end=.4+.02*revision
        samples=[yaw]
        while samples[-1]+.1<end:samples.append(samples[-1]+.1)
        samples.append(end)
        e.offer_execution_reference(spin_path(samples),ref(revision,3))
        result=e.update(TrackingState(0.,0.,yaw,0.,.1),.05)
        assert result.phase=='FINAL_ALIGN' and e.reference_accepted
        assert e._stages[0] is anchor and e._stage_index==0
        assert len(e._stages)<=1+math.ceil((end-.4)/(math.pi/2))
        assert all(abs(stage.spin_delta)>=math.pi/2-1e-9 for stage in e._stages[1:-1])
    # 非末尾future stage均为完整pi/2片段，而非100个horizon小尾点。
    assert len(e._stages)==3
    assert abs(e._stages[1].spin_delta-math.pi/2)<1e-9



def test_micro_reverse_samples_cannot_disappear_below_stage_zero_threshold():
    micro=[.2-i*1e-10 for i in range(101)]
    for old_yaws,new_yaws in [([0.,.4],[.01]+micro+[.5]),
                              ([0.]+micro+[.4],[.01,.5])]:
        e=follower(); e.offer_execution_reference(spin_path(old_yaws),ref(intent=3))
        e.update(TrackingState(0.,0.,0.),.05); e.note_command(0.,.1)
        e.offer_execution_reference(spin_path(new_yaws),ref(2,3))
        result=e.update(TrackingState(0.,0.,.01,0.,.1),.05)
        assert result.phase=='BRAKING' and not e.reference_accepted



def test_retired_terminal_stage_rebinds_angle_without_erasing_stall_balance():
    e=follower(); e.offer_execution_reference(((0.,0.,0.),(1.,0.,0.)),ref())
    e.update(TrackingState(0.,0.,0.),.05)
    e._last_pose=(e._last_pose[0],e._last_pose[1],0.)
    e.no_progress_s=14.8; history=list(e._command_history)
    e.offer_execution_reference(spin_path([0.,1.]),ref(2,3))
    e.update(TrackingState(0.,0.,0.),.05)
    assert e._last_pose[2]==1. and 14.8<e.no_progress_s<15.
    assert len(e._command_history)==len(history)
    e.update(TrackingState(0.,0.,.02,0.,.1),.05)
    assert e.no_progress_s==0. and e.failure is None
    assert abs(e._last_pose[2]-.98)<1e-9


def test_static_terminal_rolling_uses_remaining_stall_budget_and_latches_failure():
    e=follower(no_progress_timeout_s=.3)
    e.offer_execution_reference(((0.,0.,0.),(1.,0.,0.)),ref())
    e.update(TrackingState(0.,0.,0.),.05); e.no_progress_s=.15
    e.offer_execution_reference(spin_path([0.,1.]),ref(2,3))
    e.update(TrackingState(0.,0.,0.),.05)
    anchor=e._last_pose
    assert e.no_progress_s>=.2
    for revision in range(3,103):
        e.offer_execution_reference(spin_path([0.,1.]),ref(revision,3))
        e.update(TrackingState(0.,0.,0.),.05)
        assert e._last_pose is anchor
    assert e.failure=='TRACKING_STALLED' and e.last_execution_state==6
    assert e.no_progress_s>.3


def test_brake_to_follow_rebinds_direction_but_preserves_stall_clock():
    import numpy as np
    e=follower(); e.offer_execution_reference(((0.,0.,0.),(1.,0.,0.)),ref())
    e.update(TrackingState(0.,0.,0.),.05)
    e.no_progress_s=.7
    e.offer_execution_reference((),ref(2,1)); e.update(TrackingState(0.,0.,0.),.05)
    assert e.path is None
    e.offer_execution_reference(((0.,0.,0.),(0.,1.,1.57)),ref(3))
    e.update(TrackingState(0.,0.,0.),.05)
    assert np.allclose(e._last_pose[1],(0.,1.))
    assert e.no_progress_s>=.7


def test_failed_task_cannot_revive_when_explicit_terminal_rebinds():
    e=follower(); e.offer_execution_reference(((0.,0.,0.),(1.,0.,0.)),ref())
    e.update(TrackingState(0.,0.,0.),.05)
    e.failure='TRACKING_STALLED'; e.no_progress_s=15.1
    e.offer_execution_reference(spin_path([0.,1.]),ref(2,3))
    e.update(TrackingState(0.,0.,0.),.05)
    assert e.failure=='TRACKING_STALLED' and e.no_progress_s==15.1
    assert e.last_execution_state==6


def test_progress_is_physical_for_plain_and_contract_translation_and_spin():
    """传输/参考刷新不生成进展；真实位移或朝向误差下降才清钟。"""
    for contract in (False,True):
        for spin in (False,True):
            e=follower()
            path=spin_path([0.,1.]) if spin else ((0.,0.,0.),(1.,0.,0.))
            if contract:e.offer_execution_reference(path,ref(intent=3 if spin else 0))
            else:e.set_standard_path(path,1)
            e.update(TrackingState(0.,0.,0.),.05)
            e.no_progress_s=.4
            history=list(e._command_history)
            anchor=e._last_pose
            if contract:e.offer_execution_reference(path,ref(2,3 if spin else 0))
            else:e.set_standard_path(path,2)
            e.update(TrackingState(0.,0.,0.),.05)
            assert abs(e.no_progress_s-.45)<1e-9
            assert e._last_pose is anchor
            assert len(e._command_history)==len(history)
            actual=(TrackingState(0.,0.,.02,0.,.1) if spin
                    else TrackingState(.04,0.,0.,.1))
            e.update(actual,.05)
            assert e.no_progress_s==0. and e.failure is None


def test_stage_rebinding_is_not_physical_progress_or_command_publication():
    """改变阶段比较基准时保留时间余额和已发布命令事实。"""
    import numpy as np
    e=follower()
    e.offer_execution_reference(((0.,0.,0.),(1.,0.,0.)),ref())
    e.update(TrackingState(0.,0.,0.),.05)
    e.no_progress_s=.7
    history=[(t,u.copy()) for t,u in e._command_history]
    e.offer_execution_reference((),ref(2,1))
    e.update(TrackingState(0.,0.,0.),.05)
    e.offer_execution_reference(((0.,0.,1.57),(0.,1.,1.57)),ref(3))
    e.update(TrackingState(0.,0.,0.),.05)
    assert abs(e.no_progress_s-.75)<1e-9
    assert np.allclose(e._last_pose[1],(0.,1.))
    assert len(e._command_history)==len(history)
    assert all(t==old_t and np.array_equal(u,old_u)
               for (t,u),(old_t,old_u) in zip(e._command_history,history))

def test_brake_to_follow_while_still_moving_rebinds_without_requiring_stop():
    """FOLLOW许可不因进展锚归并新增停稳条件，保留原合同交接语义。"""
    import numpy as np
    e=follower()
    e.offer_execution_reference(((0.,0.,0.),(1.,0.,0.)),ref())
    e.update(TrackingState(0.,0.,0.),.05)
    e.note_command(0.,.1);e.no_progress_s=.7
    e.offer_execution_reference((),ref(2,1))
    e.update(TrackingState(0.,0.,0.,0.,.1),.05)
    assert e.path is None and e.execution_reference.intent==1
    e.offer_execution_reference(((0.,0.,1.57),(0.,1.,1.57)),ref(3))
    e.update(TrackingState(0.,0.,.01,0.,.05),.05)
    assert e.reference_accepted and e.execution_reference.intent==0
    assert np.allclose(e._last_pose[1],(0.,1.))
    assert abs(e._last_pose[2]-1.56)<1e-9
    assert abs(e.no_progress_s-.75)<1e-9 and e.failure is None

def test_plain_initial_path_waits_for_map_before_binding_physical_progress():
    """无地图等待不提前建立行进锚，也不把等待期间外力位移算成进展。"""
    e=follower();grid=e.map;e.map=None;e.no_progress_s=.7
    e.set_standard_path(((0.,0.,0.),(1.,0.,0.)),1)
    e.update(TrackingState(0.,0.,0.),.05)
    assert e._last_pose is None and e.no_progress_s==.7
    e.set_map(grid)
    e.update(TrackingState(.04,0.,0.),.05)
    assert abs(e.no_progress_s-.75)<1e-9
    assert abs(e._last_pose[0][0]-.04)<1e-9

def test_pose_wait_retires_old_motion_and_recovery_requires_stop_and_new_proof():
    e=follower();old=ref();path=((0.,0.,0.),(1.,0.,0.))
    e.offer_execution_reference(path,old);tick(e,TrackingState(0.,0.,0.))
    e.note_command(.1,0.);history=list(e._command_history)
    e.wait_for_pose()
    assert e.path is None and e._execution_pending is None and e.failure is None
    assert e._command_history==history
    assert not e.offer_execution_reference(path,old)
    result=e.update(TrackingState(0.,0.,0.),.05)
    assert result.command.linear_x_mps==0. and e._stop_reason=='WAITING_FOR_POSE'
    fresh=ref(2);assert e.offer_execution_reference(path,fresh)
    e._brake_safe=lambda *a:[False]
    e.update(TrackingState(0.,0.,0.),.05)
    assert e.execution_reference is old
    e._brake_safe=lambda *a:[True]
    e.update(TrackingState(0.,0.,0.,.1),.05)
    assert e.execution_reference is old
    e.update(TrackingState(0.,0.,0.),.05)
    assert e.execution_reference is fresh and e._stop_reason is None


def test_pose_wait_barrier_survives_intermediate_parking_reference():
    e=follower();path=((0.,0.,0.),(1.,0.,0.));old=ref()
    e.offer_execution_reference(path,old);tick(e,TrackingState(0.,0.,0.));e.wait_for_pose()
    e.request_stop('EXECUTION_PAIR_WAIT')
    assert e._stop_reason=='WAITING_FOR_POSE'
    parked=ref(2,4);assert e.offer_execution_reference((),parked)
    e._brake_safe=lambda *a:[True]
    e.update(TrackingState(0.,0.,0.,.1),.05)
    assert e.execution_reference is old
    fresh=ref(3);e.offer_execution_reference(path,fresh)
    e.update(TrackingState(0.,0.,0.,.1),.05)
    assert e.execution_reference is old
    e.update(TrackingState(0.,0.,0.),.05)
    assert e.execution_reference is fresh


def test_pose_wait_does_not_erase_manual_stop():
    e=follower();e.request_stop('MANUAL_STOP');e.wait_for_pose();e.request_stop('NO_PATH')
    assert e._stop_reason=='MANUAL_STOP'


def test_manual_stop_survives_pose_wait_parking_and_same_session_follow():
    e=follower();path=((0.,0.,0.),(1.,0.,0.))
    e.offer_execution_reference(path,ref());tick(e,TrackingState(0.,0.,0.))
    e.request_stop('MANUAL_STOP');e.wait_for_pose();e._brake_safe=lambda *a:[True]
    e.offer_execution_reference((),ref(2,4));e.update(TrackingState(0.,0.,0.),.05)
    e.offer_execution_reference(path,ref(3));result=e.update(TrackingState(0.,0.,0.),.05)
    assert e._stop_reason=='MANUAL_STOP'
    assert result.command.linear_x_mps==0. and result.command.angular_z_radps==0.


def test_continuity_mode_preserves_age_and_independent_brake_veto():
    from dataclasses import replace
    e=follower();e.config=replace(e.config,state_age_budget_s=None)
    e.offer_execution_reference(((0.,0.,0.),(1.,0.,0.)),ref())
    ages=[]
    e._brake_safe=lambda *a:(ages.append(e._state_age_s) or [False])
    e.update(TrackingState(0.,0.,0.),.05,now_s=1.,state_age_s=9.)
    assert ages and all(a==9. for a in ages)
    assert not e.reference_accepted


def test_delayed_stationary_reference_passes_actual_complete_brake_certificate():
    e=follower();r=ref();e.offer_execution_reference(((0.,0.,0.),(1.,0.,0.)),r)
    e.update(TrackingState(0.,0.,0.),.05,now_s=1.,state_age_s=5.)
    assert e.reference_accepted and e.execution_reference is r
    assert e._state_age_s==5. and e.failure is None
