from dataclasses import FrozenInstanceError, replace
from types import SimpleNamespace as NS
import pytest
from scripts.navigation.one_cm.nav1cm.execution_contract import *
from tests.navigation.one_cm.test_p3_session import make_session
from scripts.navigation.one_cm.nav1cm.navigation_task import NavigationTaskCoordinator


def ref(task, revision=1):
    return ExecutionReference(execution_session_id(task.active.task_id, task.active.generation),revision,'a'*64,FOLLOW,1)


def test_metadata_is_immutable_and_validated():
    r=ExecutionReference('a'*32,1,'b'*64,FOLLOW,1)
    with pytest.raises(FrozenInstanceError): r.intent=HOLD
    with pytest.raises(ValueError): replace(r,intent=9)
    with pytest.raises(ValueError): replace(r,revision=True)


def test_hash_includes_headers_and_exact_quaternion():
    stamp=NS(sec=1,nanosec=2)
    header=lambda: NS(frame_id='map',stamp=NS(sec=stamp.sec,nanosec=stamp.nanosec))
    pose=NS(header=header(),pose=NS(position=NS(x=1.,y=2.,z=3.),orientation=NS(x=.1,y=.2,z=.3,w=.4)))
    path=NS(header=header(),poses=[pose])
    initial=path_fingerprint(path)
    pose.header.stamp.nanosec+=1
    assert path_fingerprint(path)!=initial
    pose.header.stamp.nanosec-=1
    pose.pose.orientation.w+=1e-15
    assert path_fingerprint(path)!=initial


def test_feedback_binding_timeout_and_fault_is_read_only():
    session,Goal=make_session(); task=NavigationTaskCoordinator(session)
    task.configure_execution_feedback(required=True)
    task.submit('a',Goal(0,3,0),0.)
    r=ref(task); task.register_execution_reference(r,0.)
    assert task.check_execution_fault(1.) is None
    assert task.check_execution_fault(2.1)
    assert task.active.phase=='PLANNING' and session.goal is not None
    s=ExecutionStatus(r.session_id,r.revision,r.path_digest,r.intent,3,True,1)
    assert not task.offer_execution_status(replace(s,path_digest='b'*64),.1)
    assert task.offer_execution_status(s,.1)
    assert not task.offer_execution_status(s,.2)
    assert task.check_execution_fault(.2) is None
    assert task.offer_execution_status(replace(s,state=6,stamp_ns=2,reason='TRACKING_BAD'),.3)
    assert task.check_execution_fault(.3)=='TRACKING_BAD'
    task.fail_tracking('TRACKING_BAD',.3)
    assert task.active.phase=='ERROR_STOPPING' and session.goal is None
    packet=session.tick(.4)
    proposal=task.observe(packet,.4,state_updated=False,defer_commit=True)
    assert task.active.phase=='ERROR_STOPPING'
    task.commit_observation(proposal)
    assert task.active.outcome is None
    task.observe(packet,.5,state_updated=True)
    assert task.active.outcome=='INTERNAL_ERROR' and task.active.reason_code=='TRACKING_BAD'


def test_catalog_handles_publish_faster_than_feedback_and_latches_failure():
    session,Goal=make_session(); task=NavigationTaskCoordinator(session)
    task.configure_execution_feedback(required=True)
    task.submit('a',Goal(0,3,0),0.)
    r=ref(task); task.register_execution_reference(r,0.)
    task.register_execution_reference(replace(r,revision=2),.05)
    status=ExecutionStatus(r.session_id,1,r.path_digest,r.intent,3,True,1)
    assert task.offer_execution_status(status,.1)
    assert task.offer_execution_status(replace(status,state=6,stamp_ns=2,reason='FAILED_OLD_ACCEPTED'),.2)
    task.register_execution_reference(replace(r,revision=3),.25)
    assert task.offer_execution_status(replace(status,revision=3,state=3,stamp_ns=3),.3)
    assert task.check_execution_fault(.3)=='FAILED_OLD_ACCEPTED'
    task.submit('b',Goal(0,-3,0),.4)
    assert not task.offer_execution_status(replace(status,state=6,stamp_ns=4),.5)
    assert task.check_execution_fault(.5) is None


def test_arrival_requires_accepted_stop_intent_or_completion_and_independent_hold():
    session,Goal=make_session(); task=NavigationTaskCoordinator(session)
    task.configure_execution_feedback(required=True)
    task.submit('a',Goal(0,0,0),0.)
    r=ref(task); task.register_execution_reference(r,0.)
    packet=session.tick(0.)
    task.observe(packet,0.); task.observe(packet,.4)
    assert task.active.outcome is None
    status=ExecutionStatus(r.session_id,1,r.path_digest,FOLLOW,0,True,1)
    task.offer_execution_status(status,.4)
    task.observe(packet,.5); task.observe(packet,.9)
    assert task.active.outcome is None
    hold=replace(r,revision=2,intent=HOLD)
    task.register_execution_reference(hold,1.)
    task.register_execution_reference(replace(hold,revision=3),1.05)
    assert task.offer_execution_status(replace(status,revision=2,intent=HOLD,stamp_ns=2,accepted=False),1.1)
    task.observe(packet,1.1); task.observe(packet,1.5)
    assert task.active.outcome is None
    assert task.offer_execution_status(replace(status,revision=2,intent=HOLD,stamp_ns=3),1.6)
    task.observe(packet,1.6)
    task.observe(packet,2.,state_updated=False)
    assert task.active.outcome is None
    task.observe(packet,2.)
    assert task.active.outcome=='GOAL_REACHED'


def test_catalog_bounded_and_state_can_progress_from_tracking_to_braking():
    session,Goal=make_session(); task=NavigationTaskCoordinator(session)
    task.submit('a',Goal(0,3,0),0.)
    r=ref(task)
    for revision in range(1,140):
        assert task.register_execution_reference(replace(r,revision=revision),0.)
    assert len(task._references)==128
    status=ExecutionStatus(r.session_id,139,r.path_digest,FOLLOW,3,True,1)
    assert task.offer_execution_status(status,.1)
    assert task.offer_execution_status(replace(status,state=1,stamp_ns=2),.2)
    assert not task.offer_execution_status(replace(status,revision=138,stamp_ns=3),.3)
    assert not task.offer_execution_status(replace(status,revision=1,stamp_ns=4),.4)


def test_delayed_failure_of_previously_accepted_revision_is_not_lost():
    session,Goal=make_session(); task=NavigationTaskCoordinator(session)
    task.configure_execution_feedback(required=True)
    task.submit('a',Goal(0,3,0),0.)
    r=ref(task); task.register_execution_reference(r,0.)
    status=ExecutionStatus(r.session_id,1,r.path_digest,FOLLOW,3,True,1)
    task.offer_execution_status(status,.1)
    task.register_execution_reference(replace(r,revision=2),.2)
    task.offer_execution_status(replace(status,revision=2,stamp_ns=2),.3)
    assert task.offer_execution_status(replace(status,state=6,reason='DELAYED_FAILURE',stamp_ns=3),.4)
    assert task.check_execution_fault(.4)=='DELAYED_FAILURE'


def test_unaccepted_feedback_keeps_communication_alive_without_acceptance():
    session,Goal=make_session(); task=NavigationTaskCoordinator(session)
    task.configure_execution_feedback(required=True)
    task.submit('a',Goal(0,3,0),0.)
    r=ref(task);task.register_execution_reference(r,0.)
    status=ExecutionStatus(r.session_id,1,r.path_digest,FOLLOW,0,False,1)
    assert task.offer_execution_status(status,1.9)
    assert task.check_execution_fault(2.1) is None
    assert task._execution_accepted_reference is None
    assert task.check_execution_fault(3.91)=='EXECUTION_FEEDBACK_TIMEOUT'


def test_action_uuid_identity_preserved_and_zero_bounds_rejected():
    assert execution_session_id('a'*32,2)=='a'*32
    r=ExecutionReference('a'*32,1,'b'*64,HOLD,1)
    for field in ('revision','stamp_ns','position_tolerance_m','yaw_tolerance_rad','stopped_linear_mps','stopped_angular_rps'):
        with pytest.raises(ValueError): replace(r,**{field:0})


def test_status_predating_reference_is_rejected_and_invalidation_is_not_arrival():
    session,Goal=make_session(); task=NavigationTaskCoordinator(session)
    task.configure_execution_feedback(required=True)
    task.submit('a',Goal(0,0,0),0.)
    r=replace(ref(task),stamp_ns=10,intent=INVALIDATED)
    task.register_execution_reference(r,0.)
    status=ExecutionStatus(r.session_id,1,r.path_digest,INVALIDATED,0,True,9)
    assert not task.offer_execution_status(status,.1)
    assert task.offer_execution_status(replace(status,stamp_ns=10),.2)
    packet=session.tick(.2)
    task.observe(packet,.2);task.observe(packet,.6)
    assert task.active.outcome is None
    assert task.offer_execution_status(replace(status,stamp_ns=11,state=6,accepted=False,reason='REQUEST_REJECTED'),.7)
    assert task.check_execution_fault(.7)=='REQUEST_REJECTED'


def test_delayed_accepted_failure_bypasses_global_stamp_without_regressing_gate():
    session,Goal=make_session(); task=NavigationTaskCoordinator(session)
    task.configure_execution_feedback(required=True)
    task.submit('a',Goal(0,3,0),0.)
    r=replace(ref(task),revision=10,stamp_ns=90)
    task.register_execution_reference(r,0.)
    old=ExecutionStatus(r.session_id,10,r.path_digest,FOLLOW,3,True,99)
    assert task.offer_execution_status(old,.1)
    task.register_execution_reference(replace(r,revision=11,stamp_ns=95),.2)
    newer=replace(old,revision=11,stamp_ns=101)
    assert task.offer_execution_status(newer,.3)
    assert task.offer_execution_status(replace(old,state=6,stamp_ns=100,reason='LATE_FAILURE'),.4)
    assert task.check_execution_fault(.4)=='LATE_FAILURE'
    assert task._execution_status is newer
    assert task._execution_stamp==101 and task._execution_revision==11
    assert not task.offer_execution_status(replace(old,state=5,stamp_ns=102),.5)
    assert task._execution_status is newer


def test_failed_same_revision_cannot_be_overwritten_with_completed():
    session,Goal=make_session(); task=NavigationTaskCoordinator(session)
    task.configure_execution_feedback(required=True)
    task.submit('a',Goal(0,3,0),0.)
    r=ref(task);task.register_execution_reference(r,0.)
    failed=ExecutionStatus(r.session_id,1,r.path_digest,FOLLOW,6,True,10,reason='FAILED')
    assert task.offer_execution_status(failed,.1)
    assert not task.offer_execution_status(replace(failed,state=5,stamp_ns=11),.2)
    assert task._execution_status is failed


def accepted_reference_after_catalog_rollover():
    session, Goal = make_session()
    task = NavigationTaskCoordinator(session)
    task.configure_execution_feedback(required=True)
    task.submit('a', Goal(0, 3, 0), 0.)
    reference = ref(task)
    task.register_execution_reference(reference, 0.)
    status = ExecutionStatus(reference.session_id, 1, reference.path_digest, FOLLOW, 1, True, 10)
    assert task.offer_execution_status(status, .01)
    for revision in range(2, 131):
        now = revision * .05
        task.register_execution_reference(replace(reference, revision=revision), now)
        status = replace(status, stamp_ns=revision + 10)
        assert task.offer_execution_status(status, now)
        assert task.check_execution_fault(now) is None
    return task, reference, status


def test_active_accepted_reference_heartbeat_survives_catalog_rollover():
    task, reference, status = accepted_reference_after_catalog_rollover()
    assert len(task._references) == 128
    assert task.check_execution_fault(8.49) is None
    assert task.check_execution_fault(8.51) == 'EXECUTION_FEEDBACK_TIMEOUT'


@pytest.mark.parametrize('changes', [
    {'session_id': 'b' * 32}, {'path_digest': 'b' * 64}, {'intent': HOLD},
    {'stamp_ns': 140}, {'stamp_ns': 139},
])
def test_pinned_reference_still_enforces_exact_identity_and_time(changes):
    task, reference, status = accepted_reference_after_catalog_rollover()
    assert not task.offer_execution_status(replace(status, **changes), 6.6)
    assert task.check_execution_fault(8.51) == 'EXECUTION_FEEDBACK_TIMEOUT'


def test_pinned_reference_is_replaced_only_by_real_acceptance_and_reset_on_new_task():
    task, reference, status = accepted_reference_after_catalog_rollover()
    newer = replace(status, revision=130, accepted=False, stamp_ns=141)
    assert task.offer_execution_status(newer, 6.6)
    assert task.check_execution_fault(8.51) is None
    assert task._execution_accepted_reference is reference
    assert task.check_execution_fault(8.61) == 'EXECUTION_FEEDBACK_TIMEOUT'
    assert task.offer_execution_status(replace(newer, accepted=True, stamp_ns=142), 6.7)
    assert not task.offer_execution_status(replace(status, stamp_ns=143), 6.8)
    assert not task.offer_execution_status(replace(status, state=5, stamp_ns=144), 6.9)
    # 旧pin已经退休且不在最近128条中，不能无界保存旧执行身份。
    assert not task.offer_execution_status(replace(status, state=6, stamp_ns=145), 7.)
    session, Goal = make_session()
    task.submit('b', Goal(0, -3, 0), 7.1)
    assert not task.offer_execution_status(replace(newer, accepted=True, stamp_ns=146), 7.2)
    assert task.check_execution_fault(9.11) == 'EXECUTION_FEEDBACK_TIMEOUT'


def test_pinned_accepted_failure_is_latched_after_catalog_rollover():
    task, reference, status = accepted_reference_after_catalog_rollover()
    failure = replace(status, state=6, accepted=False, stamp_ns=141, reason='LATE_ACCEPTED_FAILURE')
    assert task.offer_execution_status(failure, 6.6)
    assert task.check_execution_fault(6.6) == 'LATE_ACCEPTED_FAILURE'
    assert not task.offer_execution_status(replace(status, state=5, stamp_ns=142), 6.7)


def test_pinned_delayed_failure_does_not_regress_newer_unaccepted_status():
    task, reference, status = accepted_reference_after_catalog_rollover()
    newer = replace(status, revision=130, accepted=False, stamp_ns=150)
    assert task.offer_execution_status(newer, 6.6)
    failure = replace(status, state=6, stamp_ns=149, reason='LATE_ACCEPTED_FAILURE')
    assert task.offer_execution_status(failure, 6.7)
    assert task.check_execution_fault(6.7) == 'LATE_ACCEPTED_FAILURE'
    assert task._execution_status is newer
    assert not task.offer_execution_status(replace(status, state=5, stamp_ns=151), 6.8)


def test_pinned_terminal_reference_cannot_resume_after_catalog_rollover():
    task, reference, status = accepted_reference_after_catalog_rollover()
    assert task.offer_execution_status(replace(status, state=5, stamp_ns=141), 6.6)
    for revision in range(131, 261):
        task.register_execution_reference(replace(reference, revision=revision), 6.7)
    assert not task.offer_execution_status(replace(status, stamp_ns=142), 6.8)
    assert task.offer_execution_status(replace(status, state=6, stamp_ns=143, reason='TERMINAL_FAILURE'), 6.9)
    assert task.check_execution_fault(6.9) == 'TERMINAL_FAILURE'


def test_matched_pose_wait_heartbeat_preserves_goal_without_faking_acceptance():
    session,Goal=make_session();task=NavigationTaskCoordinator(session)
    task.configure_execution_feedback(required=True,timeout_s=2.)
    task.submit('a',Goal(0,3,0),0.)
    r=ref(task);task.register_execution_reference(r,0.)
    waiting=ExecutionStatus(r.session_id,r.revision,r.path_digest,FOLLOW,0,False,1,'WAITING_FOR_POSE')
    assert task.offer_execution_status(waiting,3.)
    assert task.check_execution_fault(3.1) is None
    assert task._execution_accepted_reference is None
    assert task.active.outcome is None and session.goal.y==3
    assert task.check_execution_fault(5.1)=='EXECUTION_FEEDBACK_TIMEOUT'
    assert task.offer_execution_status(replace(waiting,stamp_ns=2),6.)
    assert task.offer_execution_status(replace(waiting,stamp_ns=3,reason='WAITING_FOR_FRESH_REFERENCE'),6.1)
    assert task.check_execution_fault(7.9) is None
    assert task.check_execution_fault(8.10001)=='EXECUTION_FEEDBACK_TIMEOUT'


def test_pose_wait_does_not_mask_real_failed_reference():
    session,Goal=make_session();task=NavigationTaskCoordinator(session)
    task.configure_execution_feedback(required=True);task.submit('a',Goal(0,3,0),0.)
    r=ref(task);task.register_execution_reference(r,0.)
    assert task.offer_execution_status(ExecutionStatus(r.session_id,1,r.path_digest,FOLLOW,6,False,1,'UNSAFE'),.1)
    assert not task.offer_execution_status(ExecutionStatus(r.session_id,1,r.path_digest,FOLLOW,0,False,2,'WAITING_FOR_POSE'),.2)
    assert task.check_execution_fault(.2)=='UNSAFE'
