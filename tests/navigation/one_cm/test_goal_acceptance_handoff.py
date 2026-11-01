"""优化目标与完成容差分开；普通滚动更新不清空可用来源。"""
import pytest
from scripts.navigation.one_cm.nav1cm.contracts import VehicleState, Route, WorldSnapshot, CellRect
from tests.navigation.one_cm.test_p3_session import make_session


@pytest.mark.parametrize('distance', [.03, .08, .099])
def test_stopped_pose_inside_acceptance_can_finish_without_two_cm(distance):
    session, Goal = make_session()
    session.direct_control = True
    session.coordinator.goal_position_target_m = .02
    assert session.offer_goal(Goal(0, 0, 0))
    session.offer_state(VehicleState(distance, 0, 0, 0, 0))
    packet = session.tick(0.)
    assert packet.result.status == 'ARRIVED'
    assert session.arrival_current(packet)


def test_near_goal_jitter_does_not_restart_initial_route_alignment():
    session, Goal = make_session()
    session.direct_control = True
    c = session.coordinator
    c.goal_position_target_m = .02
    c.initial_travel_direction = 'forward'
    assert session.offer_goal(Goal(0, 0, 0))
    for k, distance in enumerate([.019, .031, .035, .021]):
        session.offer_state(VehicleState(distance, 0, .25-k*.01, 0, -.02))
        packet = session.tick(k*.05)
        assert packet.result.status == 'GOAL_TURNING'
        assert packet.candidate.source.startswith('terminal-yaw:bounded')
    assert c.local_worker.requests == []


@pytest.mark.parametrize('distance,speed,ready', [(.08,.05,False),(.015,.05,True),(.11,0.,False),(.08,0.,True)])
def test_position_target_still_governs_moving_approach(distance,speed,ready):
    session, Goal = make_session()
    c = session.coordinator
    c.goal_position_target_m = .02
    assert session.offer_goal(Goal(0,0,0))
    state = VehicleState(distance,0,0,speed,0)
    result = c._update_route(session.world,state,(0,0),0.,[],goal_yaw=0.)
    assert result[1] == ready
    if speed or distance > .1:
        assert not c.goal_reached(state)


def test_predicted_acceptance_cannot_finish_for_measured_pose_outside():
    session, Goal = make_session()
    session.direct_control = True
    session.coordinator.goal_position_target_m = .02
    assert session.offer_goal(Goal(0,0,0))
    session.offer_state(VehicleState(.12,0,0,0,0))
    packet = session.tick(0.,planning_state=VehicleState(.08,0,0,0,0))
    assert packet.result.status == 'WAITING_GLOBAL'
    assert not session.coordinator._position_completed
    assert not session.arrival_current(packet)
    session.offer_state(VehicleState(.08,0,0,0,0))
    packet = session.tick(.05,planning_state=session.state)
    assert packet.result.status == 'ARRIVED'
    assert session.arrival_current(packet)


def test_map_and_route_refresh_continue_existing_finite_reference():
    from tests.navigation.one_cm.test_reference_source_buffer import source, make_source_coordinator
    from scripts.navigation.one_cm.nav1cm.execution import certified_velocity_command
    original = source()
    c, world = make_source_coordinator(original)
    c.velocity_reference = True
    first = c.tick(world,VehicleState(.05,0,0,.2,0),(2,0),.05,goal_yaw=0.,need_action=False)
    assert first.status == 'BUFFER_EXECUTING'
    c.route = Route(2,2,[[0,0],[2,0]],(2,0))
    c.progress = None
    world = WorldSnapshot(2,(),(CellRect(-1000,-1000,1000,1000),))
    state = VehicleState(.06,0,0,.2,0)
    refreshed = c.tick(world,state,(2,0),.1,goal_yaw=0.,need_action=False)
    assert refreshed.status == 'BUFFER_EXECUTING'
    assert c.buffer.source is original
    assert any(e['kind']=='BUFFER_SOURCE_AWAITING_HANDOVER' for e in refreshed.events)
    command = certified_velocity_command(refreshed,state,c.limits,reference_interval_s=.2,
        last_command=(.2,0.),elapsed_s=.05,command_hold_s=1.2)
    assert command[0] > 0


def test_acceptance_edge_uses_same_observed_stop_rule_with_prediction():
    results=[]
    for predicted in (False, True):
        session, Goal = make_session()
        session.direct_control=True
        session.coordinator.goal_position_target_m=.02
        assert session.offer_goal(Goal(0,0,0))
        measured=VehicleState(.09999,0,0,.0099,0)
        session.offer_state(measured)
        kwargs={'planning_state':VehicleState(.09999,0,0,0,0)} if predicted else {}
        packet=session.tick(0.,**kwargs)
        results.append((packet.result.status,session.arrival_current(packet)))
    assert results == [('ARRIVED',True),('ARRIVED',True)]
