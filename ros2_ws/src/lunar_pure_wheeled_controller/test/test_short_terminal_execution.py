import math
import pytest
from lunar_pure_wheeled_controller.execution import PathExecutor
from lunar_pure_wheeled_controller.tracking import TrackingPolicy, TrackingState
from lunar_pure_wheeled_controller.path_geometry import angle

START=(29.13449403148658, 20.2080871167586, -0.08940547519658691)
GOAL=(29.200000762939453, 20.200000762939453, .7853981633974483)


def executor():
    p=TrackingPolicy(goal_position_tolerance_m=.1, goal_yaw_tolerance_rad=.05, max_angular_radps=1.)
    e=PathExecutor(p)
    heading=math.atan2(GOAL[1]-START[1], GOAL[0]-START[0])
    e.set_path(((START[0],START[1],heading),GOAL,GOAL), identity=('saved053',1499))
    return e


def test_short_tail_rotates_until_curvature_feasible_instead_of_rejecting():
    e=executor()
    result=e.update(TrackingState(*START),.05)
    assert result.phase=='ALIGNING'
    assert result.command.linear_x_mps==0
    assert result.command.angular_z_radps!=0
    assert result.command.failure_reason is None


@pytest.mark.parametrize('reverse',[False,True])
def test_saved_terminal_pose_completes_with_public_tolerances(reverse):
    e=executor(); s=TrackingState(START[0],START[1],angle(START[2]+(math.pi if reverse else 0.)))
    dt=.05
    for _ in range(1000):
        result=e.update(s,dt)
        assert result.command.failure_reason is None
        if result.command.complete:
            assert math.hypot(GOAL[0]-s.x_m,GOAL[1]-s.y_m)<=.1
            assert abs(angle(GOAL[2]-s.yaw_rad))<=.05
            break
        v=result.command.linear_x_mps;w=result.command.angular_z_radps
        vv=s.linear_mps+max(-.5*dt,min(.5*dt,v-s.linear_mps))
        ww=s.angular_radps+max(-.5*dt,min(.5*dt,w-s.angular_radps))
        if abs(ww)<1e-12:dx=vv*dt*math.cos(s.yaw_rad);dy=vv*dt*math.sin(s.yaw_rad)
        else:dx=vv/ww*(math.sin(s.yaw_rad+ww*dt)-math.sin(s.yaw_rad));dy=vv/ww*(math.cos(s.yaw_rad)-math.cos(s.yaw_rad+ww*dt))
        if result.phase=='TRACKING' and abs(v)>1e-10:
            assert abs(w/v)<=e.policy.max_curvature_per_m+1e-9
        s=TrackingState(s.x_m+dx,s.y_m+dy,angle(s.yaw_rad+ww*dt),vv,ww)
    else:pytest.fail('saved terminal reference did not complete')


@pytest.mark.parametrize('reverse', [False, True])
def test_feasible_offset_heading_is_preserved_on_short_reference(reverse):
    e = PathExecutor(TrackingPolicy(goal_position_tolerance_m=.1,
                                    goal_yaw_tolerance_rad=.05))
    e.set_path(((0., 0., 0.), (.1, 0., 0.)), identity=('offset', 1))
    s = TrackingState(0., .009, angle(-.079 + (math.pi if reverse else 0.)))
    first = e.update(s, .05)
    assert first.phase == 'TRACKING'
    assert first.command.failure_reason is None
    for _ in range(1000):
        result = e.update(s, .05)
        assert result.command.failure_reason is None
        if result.command.complete:
            assert math.hypot(.1-s.x_m, s.y_m) <= .1
            assert abs(angle(s.yaw_rad)) <= .05
            break
        v, w = result.command.linear_x_mps, result.command.angular_z_radps
        if abs(w) < 1e-12:
            dx, dy = v*.05*math.cos(s.yaw_rad), v*.05*math.sin(s.yaw_rad)
        else:
            dx = v/w*(math.sin(s.yaw_rad+w*.05)-math.sin(s.yaw_rad))
            dy = v/w*(math.cos(s.yaw_rad)-math.cos(s.yaw_rad+w*.05))
        s = TrackingState(s.x_m+dx, s.y_m+dy, angle(s.yaw_rad+w*.05), v, w)
    else:
        pytest.fail('feasible offset reference did not complete')


@pytest.mark.parametrize('reverse', [False, True])
def test_alignment_and_tracking_report_same_lookahead_heading(reverse):
    e = PathExecutor(TrackingPolicy(goal_position_tolerance_m=.1,
                                    goal_yaw_tolerance_rad=.05))
    e.set_path(((0., 0., 0.), (3., 0., 0.)), identity=('chord', 1))
    yaw = angle(math.atan2(-.39, .5) + (math.pi if reverse else 0.))
    state = TrackingState(0., .39, yaw)
    for _ in range(2):
        result = e.update(state, .05)
        assert result.phase == 'TRACKING'
        assert abs(result.heading_error_rad) < 1e-10
        assert result.direction == (-1 if reverse else 1)
        assert result.command.failure_reason is None


SAVED_OFFSET_REFERENCES = [{'pose': [15.070129636259894, -10.503139385190497, 0.3252917395297879],
  'points': [[14.02822434800397, -11.193650801591128],
             [29.800000000000008, -7.800000000000003]],
  'goal_yaw': 4.71238898038469,
  'angular': 0.17395178926226973},
 {'pose': [26.51434173971568, 14.280500675875523, 2.9974479985760096],
  'points': [[26.864125144504193, 15.56993628811811],
             [26.000000000000007, 14.199999999999996],
             [26.60000000000001, 3.3999999999999955],
             [30.40000000000001, -0.4000000000000049]],
  'goal_yaw': 3.9269908169872414,
  'angular': -0.10143175970041768},
 {'pose': [34.7968443345985, -5.701210009128256, 2.4999385028260552],
  'points': [[35.165170997387946, -6.633230797501862],
             [28.600000000000012, -5.578870698741412e-15],
             [28.600000000000012, 9.199999999999996]],
  'goal_yaw': 2.356194490192345,
  'angular': 0.35000000000000003}]


def integrate_command(state, command, dt=.05):
    v = state.linear_mps + max(-.5*dt, min(.5*dt, command.linear_x_mps-state.linear_mps))
    w = state.angular_radps + max(-.5*dt, min(.5*dt, command.angular_z_radps-state.angular_radps))
    if abs(w) < 1e-12:
        dx, dy = v*dt*math.cos(state.yaw_rad), v*dt*math.sin(state.yaw_rad)
    else:
        dx = v/w*(math.sin(state.yaw_rad+w*dt)-math.sin(state.yaw_rad))
        dy = v/w*(math.cos(state.yaw_rad)-math.cos(state.yaw_rad+w*dt))
    return TrackingState(state.x_m+dx, state.y_m+dy, angle(state.yaw_rad+w*dt), v, w)


@pytest.mark.parametrize('saved', SAVED_OFFSET_REFERENCES)
@pytest.mark.parametrize('reverse', [False, True])
def test_large_offset_reference_completes_without_alignment_tracking_cycle(saved, reverse):
    e = PathExecutor(TrackingPolicy(goal_position_tolerance_m=.1,
                                    goal_yaw_tolerance_rad=.05, max_angular_radps=1.))
    points = [(*p, 0.) for p in saved['points']]
    points[-1] = (*points[-1][:2], saved['goal_yaw'])
    e.set_path(points, identity=('saved-offset', 1))
    yaw = (angle(math.atan2(points[1][1]-points[0][1], points[1][0]-points[0][0])+math.pi)
           if reverse else saved['pose'][2])
    state = TrackingState(*saved['pose'][:2], yaw, 0., 0. if reverse else saved['angular'])
    goal = points[-1]
    for _ in range(2500):
        result = e.update(state, .05)
        assert result.command.failure_reason is None
        if result.command.complete:
            assert math.hypot(goal[0]-state.x_m, goal[1]-state.y_m) <= .1
            assert abs(angle(goal[2]-state.yaw_rad)) <= .05
            assert result.direction == (-1 if reverse else 1)
            break
        if result.phase == 'TRACKING' and abs(result.command.linear_x_mps)>1e-10:
            assert abs(result.command.angular_z_radps/result.command.linear_x_mps) <= e.policy.max_curvature_per_m+1e-9
        state = integrate_command(state, result.command)
    else:
        pytest.fail('offset reference did not complete within 125 simulation seconds')


def test_turn_vertex_is_reached_before_switching_to_next_segment():
    e = PathExecutor(TrackingPolicy(goal_position_tolerance_m=.1,
                                    goal_yaw_tolerance_rad=.05,execution_clearance_m=.01))
    e.set_path(((0., 0., 0.), (1., 0., 0.), (1., 1., math.pi/2)),
               identity=('tight-corner', 1))
    result = e.update(TrackingState(.98, 0., 0.), .05)
    assert result.phase == 'TRACKING'
    assert result.command.linear_x_mps > 0.
    assert abs(result.command.angular_z_radps) < 1e-10


def test_cross_track_consumption_limits_speed_before_braking_excursion():
    e = PathExecutor(TrackingPolicy(goal_position_tolerance_m=.1,
                                    goal_yaw_tolerance_rad=.05,execution_clearance_m=.01))
    e.set_path(((0., 0., 0.), (2., 0., 0.)), identity=('error-speed', 1))
    # Error and pursuit heading are inside existing alignment limits, but
    # consume most of the small execution margin at the measured speed.
    yaw = math.atan2(-.004, .5) + .07
    curvature = 2 * (-math.sin(yaw)*.5 - math.cos(yaw)*.004) / (.5**2+.004**2)
    result = e.update(TrackingState(0., .004, yaw, .2, .2*curvature), .05)
    assert abs(result.command.linear_x_mps) < .2
    assert result.phase in ('TRACKING', 'BRAKING')


def test_corner_tail_uses_measured_distance_when_arc_projection_is_at_stop():
    e = PathExecutor(TrackingPolicy(goal_position_tolerance_m=.1,
                                    goal_yaw_tolerance_rad=.05,execution_clearance_m=.01))
    e.set_path(((0.,0.,0.),(1.,0.,0.),(1.,1.,math.pi/2)),identity=('corner-tail',1))
    e.update(TrackingState(.98,.003,math.atan2(-.003,.02)),.05)
    result=e.update(TrackingState(.999,.003,math.atan2(-.003,.001)),.05)
    assert result.phase == 'TRACKING'
    assert result.command.linear_x_mps > 0.


def test_shallow_certified_bend_is_not_skipped_by_lookahead():
    e=PathExecutor(TrackingPolicy())
    e.set_path(((0.,0.,0.),(1.,0.,0.),(2.,.1,0.)),identity=('shallow',1))
    assert e.stops[0] == pytest.approx(1.)
    straight=PathExecutor(TrackingPolicy())
    straight.set_path(((0.,0.,0.),(1.,0.,0.),(2.,0.,0.)),identity=('straight',1))
    assert straight.stops == (2.,)


def test_tight_corner_completes_with_two_control_ticks_of_command_delay():
    from collections import deque
    e=PathExecutor(TrackingPolicy(goal_position_tolerance_m=.1,
                                  goal_yaw_tolerance_rad=.05,
                                  execution_clearance_m=.01,max_angular_radps=1.))
    e.set_path(((0.,0.,0.),(1.,0.,0.),(1.,1.,math.pi/2)),identity=('delayed',1))
    state=TrackingState(0.,0.,0.)
    pending=deque([e._result().command]*2)
    for _ in range(2500):
        result=e.update(state,.05)
        assert result.command.failure_reason is None
        if result.command.complete:
            assert math.hypot(state.x_m-1,state.y_m-1)<=.1
            return
        pending.append(result.command)
        state=integrate_command(state,pending.popleft())
    pytest.fail('delayed corner did not complete')
