"""只验证CPU消息边界；ROS生成setter和真实GPU运行由独立验证覆盖。"""
from array import array
from pathlib import Path
from types import SimpleNamespace
import math
from itertools import count
import struct

import numpy as np
import pytest

from tools.isaac_viper.config import load_profile
from tools.isaac_viper.ros_bridge import RosBridge


class Message:
    """仅替代不可用的ROS消息依赖，保留桥接生成的全部字段与原始data类型。"""
    def __init__(self, **kwargs):
        self.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id='')
        self.__dict__.update(kwargs)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def bridge_fixture():
    bridge = RosBridge.__new__(RosBridge)
    bridge.p = load_profile(Path(__file__).resolve().parents[2]/'config/isaac_viper.yaml')
    bridge.frames = bridge.p['ros']['frames']
    bridge.types = {name: Message for name in ('Image', 'CameraInfo', 'PointCloud2', 'PointField')}
    bridge.pubs = {key: Publisher() for key in bridge.p['ros']['topics'] if key != 'command'}
    bridge.counts = {}
    bridge.k = [2., 0., 1., 0., 2., .5, 0., 0., 1.]
    bridge.events = []
    bridge.telemetry = bridge.events.append
    return bridge


def assert_payload(message, expected):
    assert isinstance(message.data, array) and message.data.typecode == 'B'
    assert bytes(message.data) == expected


@pytest.mark.parametrize('empty', [False, True])
def test_cloud_payload_roundtrip_noncontiguous_empty_and_owns_storage(empty):
    bridge = bridge_fixture()
    backing = np.arange(1, 13, dtype='>f8').reshape(4, 3)
    xyz = backing[::2] if not empty else backing[:0]
    bridge.cloud('lidar_raw', 3.125, 'scan', xyz)
    msg = bridge.pubs['lidar_raw'].messages[-1]
    expected = b'' if empty else struct.pack('<6f', 1, 2, 3, 7, 8, 9)
    assert_payload(msg, expected)
    assert (msg.height, msg.width, msg.point_step, msg.row_step) == (1, 0 if empty else 2, 12, 0 if empty else 24)
    assert [(f.name, f.offset, f.datatype, f.count) for f in msg.fields] == [('x',0,7,1), ('y',4,7,1), ('z',8,7,1)]
    assert msg.is_bigendian is False and msg.is_dense is True
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id) == (3, 125000000, 'scan')
    backing[:] = -123
    assert bytes(msg.data) == expected
    bridge.cloud('lidar_raw', 3.25, 'scan', xyz)
    assert bridge.pubs['lidar_raw'].messages[-1].data is not msg.data


def camera_samples():
    rgb = np.arange(36, dtype=np.uint8).reshape(2, 6, 3)[:, ::2]
    depth = np.array([[1,99,2,99,3,99], [4,99,5,99,6,99]], dtype='>f4')[:, ::2]
    return {'left':(2.5, rgb, depth, {'source':'fixture'}),
            'right':(2.5, rgb, depth, {'source':'fixture'})}


def test_camera_payloads_roundtrip_endian_strides_ownership_and_wire_fields():
    bridge = bridge_fixture()
    samples = camera_samples()
    bridge.cameras(samples)
    rgb_expected = bytes([0,1,2,6,7,8,12,13,14,18,19,20,24,25,26,30,31,32])
    depth_expected = struct.pack('<6f', 1,2,3,4,5,6)
    for key in ('left_image','right_image'):
        msg = bridge.pubs[key].messages[-1]
        assert_payload(msg, rgb_expected)
        assert (msg.height, msg.width, msg.encoding, msg.step) == (2,3,'rgb8',9)
        assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (2,500000000)
    depth_msg = bridge.pubs['depth'].messages[-1]
    assert_payload(depth_msg, depth_expected)
    assert (depth_msg.height, depth_msg.width, depth_msg.encoding, depth_msg.step) == (2,3,'32FC1',12)
    cloud = bridge.pubs['depth_points'].messages[-1]
    assert_payload(cloud, struct.pack('<18f', -.5,-.25,1, 0,-.5,2, 1.5,-.75,3, -2,1,4, 0,1.25,5, 3,1.5,6))
    left_info = bridge.pubs['left_camera_info'].messages[-1]
    right_info = bridge.pubs['right_camera_info'].messages[-1]
    assert left_info.k == bridge.k
    assert left_info.p[3] == 0 and right_info.p[3] == pytest.approx(-.28)
    assert bridge.pubs['left_image'].messages[-1].header.frame_id == bridge.frames['left_optical']
    assert bridge.pubs['right_image'].messages[-1].header.frame_id == bridge.frames['right_optical']
    samples['left'][1][:] = 0
    samples['left'][2][:] = 0
    assert bytes(bridge.pubs['left_image'].messages[-1].data) == rgb_expected
    assert bytes(bridge.pubs['right_image'].messages[-1].data) == rgb_expected
    assert bytes(depth_msg.data) == depth_expected
    assert bridge.pubs['left_image'].messages[-1].data is not bridge.pubs['right_image'].messages[-1].data


def assert_bridge_timing(timing, topics):
    assert isinstance(timing, dict)
    for key in ('depth_math','lidar_filter_math','msg_pack','publish','sensor_log'):
        assert math.isfinite(timing[key]) and timing[key] >= 0
    assert set(timing['publish_by_topic_ms']) == set(topics)
    assert sum(timing['publish_by_topic_ms'].values()) == pytest.approx(timing['publish'])


def test_camera_lidar_timing_covers_publish_topics_and_empty_samples(monkeypatch):
    from tools.isaac_viper import ros_bridge
    monkeypatch.setattr(ros_bridge.time, 'monotonic_ns', count(0, 1_000_000).__next__)
    bridge = bridge_fixture()
    timing = bridge.cameras(camera_samples())
    assert_bridge_timing(timing, ['left_image','left_camera_info','depth','depth_points','right_image','right_camera_info'])
    assert timing['depth_math'] == 1 and timing['lidar_filter_math'] == 0
    assert (timing['msg_pack'], timing['publish'], timing['sensor_log']) == (6,6,1)
    lidar = bridge.lidar((3., np.array([[1.,0.,0.],[0.,1.,0.]]), np.array([64,64])))
    assert_bridge_timing(lidar, ['lidar_raw','lidar_effective'])
    assert lidar['lidar_filter_math'] == 1 and lidar['depth_math'] == 0
    assert (lidar['msg_pack'], lidar['publish'], lidar['sensor_log']) == (2,2,1)
    for empty in (bridge.cameras({}), bridge.lidar(None)):
        assert_bridge_timing(empty, [])
        assert all(ms == 0 for key,ms in empty.items() if key != 'publish_by_topic_ms')


def test_sensor_frame_phases_and_npz_saved_only_once(tmp_path, monkeypatch):
    from tools.isaac_viper import run_scene, sensors as sensor_module
    publish_frame = getattr(run_scene, 'publish_sensor_frame', None)
    assert callable(publish_frame), 'render sensor phases need a CPU-testable collector'
    monkeypatch.setattr(sensor_module, 'active_render_products', lambda stage: [{'path':stage,'camera':['left']}])
    samples = camera_samples()
    lidar = (3., np.array([[1.,0.,0.]]), np.array([64]))
    sensors = SimpleNamespace(camera_samples=lambda:samples, lidar_sample=lambda:lidar)
    bridge, saved, status = bridge_fixture(), set(), {}
    first = publish_frame(sensors, bridge, '/Render/Fixture', tmp_path, saved, status)
    assert set(first['phase_ms']) == {'active_products','camera_readback','lidar_readback','npz_first_only','camera_pack_publish','lidar_pack_publish'}
    assert all(math.isfinite(ms) and ms >= 0 for ms in first['phase_ms'].values())
    assert first['phase_ms']['npz_first_only'] > 0
    assert_bridge_timing(first['camera_phase_ms'], ['left_image','left_camera_info','depth','depth_points','right_image','right_camera_info'])
    assert_bridge_timing(first['lidar_phase_ms'], ['lidar_raw','lidar_effective'])
    assert saved == {'left','right','lidar'}
    assert status['active_render_products'] == [{'path':'/Render/Fixture','camera':['left']}]
    assert status['render_products_after_final_camera_setup'] is True
    paths = sorted(tmp_path.glob('first-*.npz'))
    assert len(paths) == 3
    previous = {p:p.read_bytes() for p in paths}
    with np.load(tmp_path/'first-left-camera.npz') as npz:
        np.testing.assert_array_equal(npz['rgb'], samples['left'][1])
        np.testing.assert_array_equal(npz['depth'], samples['left'][2])
        assert npz['simulation_time'] == 2.5
    samples['left'][1][:] = 0
    second = publish_frame(sensors, bridge, '/Render/Fixture', tmp_path, saved, status)
    assert second['phase_ms']['npz_first_only'] == 0
    assert {p:p.read_bytes() for p in paths} == previous
