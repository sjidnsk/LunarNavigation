"""无 ROS 消息转换：保留有符号反馈、最新 TF、真实车体 yaw 和参考时间。"""
from types import SimpleNamespace as NS
import math

import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm.contracts import Route, Trajectory
from tests.navigation.one_cm.test_p3_session import api


def quaternion(yaw, scale=1.):
    return NS(x=0., y=0., z=scale * math.sin(yaw / 2), w=scale * math.cos(yaw / 2))


def pose(x=0., y=0., yaw=0., scale=1.):
    return NS(position=NS(x=x, y=y, z=0.), orientation=quaternion(yaw, scale))


def tf(x=0., y=0., yaw=0., parent='map', child='odom'):
    return NS(header=NS(frame_id=parent, stamp=NS(sec=0, nanosec=0)), child_frame_id=child,
              transform=NS(translation=NS(x=x, y=y, z=0.), rotation=quaternion(yaw, 3.)))


def test_zero_stamp_repeated_tf_updates_and_chain_inverse_work():
    module = api('ros_conversion')
    latest = module.LatestTransforms()
    latest.update(NS(transforms=[tf(1., 2., .2)]))
    assert latest.lookup('map', 'odom').x == pytest.approx(1.)
    latest.update(NS(transforms=[tf(3., 4., .4)]))
    latest.update(NS(transforms=[tf(.5, .0, .1, parent='odom', child='goal_frame')]))
    transform = latest.lookup('map', 'goal_frame')
    assert transform.x == pytest.approx(3 + .5 * math.cos(.4))
    assert transform.yaw == pytest.approx(.5)
    identity = transform.compose(latest.lookup('goal_frame', 'map'))
    assert (identity.x, identity.y, identity.yaw) == pytest.approx((0., 0., 0.))


def test_goal_empty_frame_and_nonunit_quaternion_and_missing_tf():
    module = api('ros_conversion')
    transforms = module.LatestTransforms()
    msg = NS(header=NS(frame_id=''), pose=pose(1, 2, .7, 5.))
    goal = module.goal_from_message(msg, transforms)
    assert (goal.x, goal.y, goal.yaw) == pytest.approx((1., 2., .7))
    msg.header.frame_id = 'goal_frame'
    with pytest.raises(module.MissingTransform):
        module.goal_from_message(msg, transforms)
    transforms.update(NS(transforms=[tf(2., -1., .3, child='goal_frame')]))
    goal = module.goal_from_message(msg, transforms)
    assert goal.yaw == pytest.approx(1.)


def test_odometry_pose_header_transform_and_twist_signed_in_child_frame():
    module = api('ros_conversion')
    transforms = module.LatestTransforms()
    transforms.update(NS(transforms=[tf(2., 1., math.pi / 2)]))
    msg = NS(header=NS(frame_id='odom'), child_frame_id='base_link',
             pose=NS(pose=pose(1., 0., .1, 2.)),
             twist=NS(twist=NS(linear=NS(x=-.17, y=9., z=2.), angular=NS(z=-.2))))
    state = module.state_from_odometry(msg, transforms)
    assert (state.x, state.y, state.yaw, state.v, state.w) == pytest.approx((2, 2, math.pi / 2 + .1, -.17, -.2))


@pytest.mark.parametrize('mode,points,yaws', [
    ('reverse', [[0, 0], [-.02, 0], [-.04, 0]], [0., 0., 0.]),
    ('spin', [[0, 0], [0, 0], [0, 0]], [.0, .1, .2]),
])
def test_paths_keep_all_candidate_poses_body_yaw_and_ros_relative_time(mode, points, yaws):
    module = api('ros_conversion')
    states = np.zeros((3, 6)); states[:, :2] = points; states[:, 2] = yaws
    candidate = Trajectory(1, 2, [0, .1, .25], states, np.zeros((2, 3)), mode)
    route = Route(2, 1, [[0, 0], [-1, 0]], (-1, 0))
    start_ns = 123_456_000_000
    paths = module.paths(route, candidate, start_ns)
    assert paths.local.frame_id == paths.global_route.frame_id == 'map'
    assert paths.local.stamp_ns == start_ns
    assert len(paths.local.poses) == 3
    assert [p.yaw for p in paths.local.poses] == pytest.approx(yaws)
    assert [p.stamp_ns for p in paths.local.poses] == [start_ns, start_ns + 100_000_000, start_ns + 250_000_000]
    assert [(p.x, p.y) for p in paths.local.poses] == [tuple(p) for p in points]
    # ROS 时钟回退只改变本次消息的时间锚，不改变内部单调时钟。
    assert module.paths(route, candidate, 1_000_000).local.stamp_ns == 1_000_000
    assert module.paths(None, None, start_ns).local.poses == ()


@pytest.mark.parametrize('start_ns', [0, 1234567890000000000, 987654321999999999])
@pytest.mark.parametrize('times,offsets', [
    ([0., .2, .2000000001], [0, 200000000, 200000001]),
    ([0., .2, .20000000001, .2000000001, .2000000002], [0, 200000000, 200000001, 200000002, 200000003]),
    ([0., 1e-12, 2e-12, 3e-12], [0, 1, 2, 3]),
    ([0., .1, .25], [0, 100000000, 250000000]),
])
def test_local_integer_nanoseconds_stay_strict_without_dropping_spin_poses(start_ns, times, offsets):
    module = api('ros_conversion')
    states = np.zeros((len(times), 6))
    states[:, 2] = np.arange(len(times)) * .1
    candidate = Trajectory(1, 2, times, states, np.zeros((len(times) - 1, 3)), 'spin', 'conversion-only')
    original_digest = candidate.digest()
    route = Route(2, 1, [[0, 0], [1, 0]], (1, 0))
    result = module.paths(route, candidate, start_ns)
    stamps = [pose.stamp_ns for pose in result.local.poses]
    assert stamps == [start_ns + offset for offset in offsets]
    assert stamps[0] == result.local.stamp_ns == start_ns
    assert all(right > left for left, right in zip(stamps, stamps[1:]))
    assert len(result.local.poses) == len(times)
    assert [(pose.x, pose.y) for pose in result.local.poses] == [(0., 0.)] * len(times)
    assert [pose.yaw for pose in result.local.poses] == pytest.approx(states[:, 2])
    assert [pose.stamp_ns for pose in result.global_route.poses] == [start_ns] * 2
    assert candidate.digest() == original_digest


def test_paths_publish_actual_smoothed_reference_separately_from_raw_route():
    module=api('ros_conversion')
    raw=Route(1,1,[[0,0],[2,0],[2,2]],(2,2))
    shaped=Route(1,1,[[0,0],[1.8,0],[1.95,.05],[2,.2],[2,2]],(2,2))
    output=module.paths(raw,None,123,global_reference=shaped)
    assert [(p.x,p.y) for p in output.global_route.poses]==[tuple(x) for x in raw.points]
    assert [(p.x,p.y) for p in output.global_reference.poses]==[tuple(x) for x in shaped.points]
    assert not output.local.poses
    empty=module.paths(None,None,124)
    assert not empty.global_route.poses and not empty.global_reference.poses and not empty.local.poses
