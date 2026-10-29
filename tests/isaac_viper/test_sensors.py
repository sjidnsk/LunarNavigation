"""独立数值夹具；不把它们作为真实传感器运行验收。"""
import math
import struct
from types import SimpleNamespace

import numpy as np
import pytest


def test_intrinsics_and_depth_projection_optical_range():
    from tools.isaac_viper.sensors import camera_intrinsics, depth_points
    k = camera_intrinsics(4, 2, 24, 48, 24)
    assert k == pytest.approx([2, 0, 1.5, 0, 2, .5, 0, 0, 1])
    # 独立手算：左上角 [-.75,-.25,1]；斜距超过10m的右下点不能进入有效云。
    depth = np.array([[1, np.nan, 0, -1], [np.inf, 9.9, 20, 10.]])
    xyz = depth_points(depth, k, .05, 10)
    assert xyz.shape == (1, 3)
    assert xyz[0] == pytest.approx([-.75, -.25, 1])


def test_lidar_effective_fov_range_and_valid_flag():
    from tools.isaac_viper.sensors import effective_lidar
    xyz = np.array([[1, 1, 0], [1, -1, 0], [1, 1.01, 0], [-1, 0, 0],
                    [10, 0, 0], [10.01, 0, 0], [.01, 0, 0], [1, 0, 0]])
    out = effective_lidar(xyz, np.array([64]*7+[0]), .05, 10, 90)
    assert out == pytest.approx(xyz[[0, 1, 4]])


def test_quaternion_velocity_and_optical_basis():
    from tools.isaac_viper.sensors import rotation_matrix, wxyz_to_xyzw, OPTICAL_XYZW
    assert wxyz_to_xyzw([.5, .5, -.5, -.5]) == [.5, -.5, -.5, .5]
    r = rotation_matrix([0, 0, math.sqrt(.5), math.sqrt(.5)])
    assert r.T @ np.array([0, 2, 0]) == pytest.approx([2, 0, 0], abs=1e-12)
    optical = rotation_matrix(OPTICAL_XYZW)
    assert optical @ np.array([0, 0, 1]) == pytest.approx([1, 0, 0])
    assert optical @ np.array([1, 0, 0]) == pytest.approx([0, -1, 0])
    assert optical @ np.array([0, 1, 0]) == pytest.approx([0, 0, -1])
    with pytest.raises(ValueError):
        rotation_matrix([0, 0, 0, 0])


def test_pointcloud_layout_little_endian_xyz():
    from tools.isaac_viper.sensors import pointcloud_bytes
    data = pointcloud_bytes([[1, -2, 3], [4, 5, 6]])
    assert len(data) == 24
    assert struct.unpack('<ffffff', data) == (1, -2, 3, 4, 5, 6)


@pytest.mark.parametrize('index', range(6))
@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_twist_rejects_nonfinite_in_every_component(index, value):
    from tools.isaac_viper.ros_bridge import validated_twist
    values = [0.] * 6
    values[index] = value
    with pytest.raises(ValueError):
        validated_twist(values)


@pytest.mark.parametrize('index', [1, 2, 3, 4])
def test_twist_rejects_unsupported_motion(index):
    from tools.isaac_viper.ros_bridge import validated_twist
    values = [0.] * 6
    values[index] = .01
    with pytest.raises(ValueError):
        validated_twist(values)


def test_twist_si_and_domain_guard():
    from tools.isaac_viper.ros_bridge import validated_twist, assert_ros_environment
    assert validated_twist([.2, 0, 0, 0, 0, -.1]) == (.2, 0, -.1)
    with pytest.raises(ValueError):
        validated_twist([0]*5)
    with pytest.raises(RuntimeError):
        assert_ros_environment({'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '1'})
    assert_ros_environment({'ROS_DOMAIN_ID': '117', 'ROS_LOCALHOST_ONLY': '1', 'ROS_DISTRO': 'humble'})


def test_mailbox_keeps_latest_immutable_command_and_real_receive_time():
    from tools.isaac_viper.ros_bridge import LatestCommandMailbox
    box = LatestCommandMailbox()
    values = [.1, 0, 0, 0, 0, .02]
    box.receive(values, 1.)
    values[0] = 9.
    first = box.take_after(0)
    assert first.values[0] == .1
    box.receive([.2, 0, 0, 0, 0, 0], 1.25)
    latest = box.take_after(first.sequence)
    assert latest.planar == (.2, 0., 0.)
    assert latest.rx_wall == 1.25
    assert box.take_after(latest.sequence) is None
    # 真正的纯Drive确认旧接收时间不会在稍后消费时被刷新来绕过watchdog。
    from tools.isaac_viper.drive import DriveController
    from tools.isaac_viper.config import load_profile
    from pathlib import Path
    c = DriveController(load_profile(Path(__file__).resolve().parents[2]/'config/isaac_viper.yaml'))
    c.receive_twist(*latest.planar, latest.rx_wall)
    assert c.step(.01, 2., [0.]*4, (0., 0.))['reason'] == 'command_timeout'


def test_mailbox_latches_invalid_before_new_valid_command():
    from tools.isaac_viper.ros_bridge import LatestCommandMailbox
    box = LatestCommandMailbox()
    box.receive([.2, 0, 0, 0, 0, 0], 1.)
    box.receive([0, 0, float('nan'), 0, 0, 0], 1.1)
    box.receive([-.2, 0, 0, 0, 0, 0], 1.2)
    invalid = box.take_after(0)
    assert invalid.planar is None
    assert invalid.reason == 'invalid_twist'
    fresh = box.take_after(invalid.sequence)
    assert fresh.planar == (-.2, 0., 0.)


def test_mailbox_clear_prevents_stale_motion_and_supports_background_receipts():
    from tools.isaac_viper.ros_bridge import LatestCommandMailbox
    from threading import Thread, Event
    box = LatestCommandMailbox()
    done = Event()
    def receive():
        for i in range(50):
            box.receive([.1, 0, 0, 0, 0, 0], 1.+i*.01)
        done.set()
    thread = Thread(target=receive)
    thread.start()
    assert done.wait(2.)
    thread.join()
    latest = box.take_after(0)
    assert latest.sequence == 50
    assert latest.rx_wall == pytest.approx(1.49)
    box.clear(2.)
    cleared = box.take_after(latest.sequence)
    assert cleared.planar is None and cleared.reason == 'cleared'


@pytest.mark.parametrize('keep_open,expected', [(False,'stopped_by_user'),(True,'requires_runtime_restart')])
def test_stop_latches_before_first_status_tick_and_cannot_resume_old_views(keep_open, expected):
    from tools.isaac_viper.run_scene import RuntimeLifecycle
    life = RuntimeLifecycle(keep_open)
    assert life.control_state(True) == 'running'
    assert life.control_state(False) == 'paused'
    assert life.control_state(True) == 'running'  # Pause/Resume不结束会话。
    life.notify_stop()  # 在.5s甚至写盘计时仍为0时Stop，随后同frame Play。
    assert life.control_state(True) == expected
    with pytest.raises(RuntimeError):
        life.guard_views()
    with pytest.raises(RuntimeError):
        life.guard_views(write=True)
    assert life.view_access_after_stop == 1
    assert life.target_write_after_stop == 1
