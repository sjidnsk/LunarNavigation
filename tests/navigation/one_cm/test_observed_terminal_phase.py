"""终点阶段只由观测确认，候选仍从规划估计起步。"""
import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import VehicleState
from tests.navigation.one_cm.test_pose_goal import coordinator, world
from tests.navigation.one_cm.test_p3_session import make_session


@pytest.mark.parametrize("asynchronous", [False, True])
@pytest.mark.parametrize("estimated_distance,observed_distance,ready", [
    (.0198, .3, False), (.3, .0198, True), (.0198, .099, False)])
def test_position_entry_uses_observation(asynchronous, estimated_distance, observed_distance, ready):
    c = coordinator(asynchronous)
    c.goal_position_target_m = .02
    estimate = VehicleState(estimated_distance, 0., 0., .02, 0.)
    observed = VehicleState(observed_distance, 0., 0., .02, 0.)
    result = c.tick(world(), estimate, (0., 0.), 0., goal_yaw=1., observed_state=observed)
    assert (result.reference.terminal_yaw == 1.) is ready
    np.testing.assert_allclose(result.candidate.states[0, :5], [estimate.x, estimate.y, estimate.yaw, estimate.v, estimate.w])


@pytest.mark.parametrize("asynchronous", [False, True])
def test_completed_position_stays_completed_after_observed_drift(asynchronous):
    c = coordinator(asynchronous)
    c.goal_position_target_m = .02
    near = VehicleState(.0198, 0., 0., .02, 0.)
    far = VehicleState(.3, 0., 0., .02, 0.)
    c.tick(world(), near, (0., 0.), 0., goal_yaw=1., observed_state=near)
    retained = c.tick(world(), far, (0., 0.), .05, goal_yaw=1., observed_state=near)
    assert retained.reference.terminal_yaw == 1.
    reopened = c.tick(world(), near, (0., 0.), .1, goal_yaw=1., observed_state=far)
    assert reopened.reference.terminal_yaw == 1.


@pytest.mark.parametrize("asynchronous", [False, True])
@pytest.mark.parametrize("change", ["xy", "yaw", "generation", "cancel"])
def test_observed_completion_resets_with_existing_goal_identity(asynchronous, change):
    c = coordinator(asynchronous)
    c.goal_position_target_m = .02
    near = VehicleState(.01, 0., 0., .02, 0.)
    c.tick(world(), near, (0., 0.), 0., goal_yaw=1., observed_state=near)
    goal, yaw = (0., 0.), 1.
    if change == "xy": goal = (.01, 0.)
    elif change == "yaw": yaw = 1.5
    elif change == "generation": c.replace_goal(goal, goal_yaw=yaw, generation=17)
    else: c.invalidate_goal(generation=18)
    result = c.tick(world(), near, goal, .05, goal_yaw=yaw,
                    observed_state=VehicleState(.09, 0., 0., .02, 0.))
    assert result.reference.terminal_yaw is None


@pytest.mark.parametrize("asynchronous", [False, True])
def test_observed_fully_satisfied_pose_keeps_existing_acceptance(asynchronous):
    c = coordinator(asynchronous)
    c.goal_position_target_m = .02
    observed = VehicleState(.09, 0., 1., 0., 0.)
    result = c.tick(world(), observed, (0., 0.), 0., goal_yaw=1., observed_state=observed)
    assert result.status == "ARRIVED"


def test_p3_session_prediction_cannot_start_final_yaw_or_complete():
    session, Goal = make_session()
    session.direct_control = True
    session.coordinator.goal_position_target_m = .02
    session.offer_state(VehicleState(.3, 0., 0., .02, 0.))
    assert session.offer_goal(Goal(0., 0., 1.))
    packet = session.tick(0., planning_state=VehicleState(.0198, 0., 0., .02, 0.))
    assert packet.result.reference.terminal_yaw is None
    assert not session.coordinator._position_completed
    assert not session.arrival_current(packet)


@pytest.mark.parametrize("asynchronous", [False, True])
def test_xy_only_uses_observed_braking_decision_with_estimated_candidate(asynchronous):
    c = coordinator(asynchronous)
    c.goal_position_target_m = .02
    observed = VehicleState(.3, 0., 0., 0., 0.)
    estimate = VehicleState(.0198, 0., 0., 0., 0.)
    result = c.tick(world(), estimate, (0., 0.), 0., observed_state=observed)
    assert result.status == "WAITING_GLOBAL"
    result = c.tick(world(), observed, (0., 0.), .05, observed_state=estimate)
    assert result.status == "GOAL_BRAKING"
    assert not c._position_completed


def test_p3_session_keeps_final_yaw_after_real_position_drift():
    session, Goal = make_session()
    session.direct_control = True
    session.coordinator.goal_position_target_m = .02
    assert session.offer_goal(Goal(0., 0., 1.))
    near = VehicleState(.0198, 0., 0., .02, 0.)
    far = VehicleState(.3, 0., 0., .02, 0.)
    session.offer_state(near)
    assert session.tick(0., planning_state=near).result.reference.terminal_yaw == 1.
    assert session.tick(.05, planning_state=far).result.reference.terminal_yaw == 1.
    session.offer_state(far)
    packet = session.tick(.1, planning_state=near)
    assert packet.result.reference.terminal_yaw == 1.
    assert not session.arrival_current(packet)
