"""隔离域117的canonical ROS接口；运行时才导入Kit捆绑Humble。"""
from array import array
from contextlib import contextmanager
import math
import os
import time
from dataclasses import dataclass
from threading import Lock, Thread, Event
import numpy as np

from tools.isaac_viper.sensors import (OPTICAL_XYZW, camera_intrinsics, depth_points,
    effective_lidar, rotation_matrix, wxyz_to_xyzw)


def byte_payload(values):
    """独立拥有的uint8载荷，保留C序字节；不引用SDK的可变读回缓冲。"""
    contiguous = np.ascontiguousarray(values)
    payload = array('B')
    # memoryview不能将零维长度的多维buffer cast为B；空消息仍拥有独立array。
    if contiguous.size:
        payload.frombytes(memoryview(contiguous).cast('B'))
    return payload


class PhaseTimings:
    """monotonic_ns分段计时，调用方将结果并入原有每帧事件。"""
    def __init__(self, keys):
        self.ms = dict.fromkeys(keys, 0.)

    @contextmanager
    def measure(self, key):
        start = time.monotonic_ns()
        try:
            yield
        finally:
            self.ms[key] += (time.monotonic_ns()-start)/1e6


class _SensorTimings(PhaseTimings):
    def __init__(self):
        super().__init__(('depth_math', 'lidar_filter_math', 'msg_pack', 'publish', 'sensor_log'))
        self.ms['publish_by_topic_ms'] = {}

    def publish(self, bridge, key, msg):
        start = time.monotonic_ns()
        bridge.publish(key, msg)
        elapsed = (time.monotonic_ns()-start)/1e6
        self.ms['publish'] += elapsed
        topics = self.ms['publish_by_topic_ms']
        topics[key] = topics.get(key, 0.) + elapsed


def validated_twist(values):
    if len(values) != 6 or any(isinstance(v, bool) or not math.isfinite(v) for v in values):
        raise ValueError('Twist requires six finite components')
    if any(values[i] != 0 for i in (1, 2, 3, 4)):
        raise ValueError('unsupported non-planar/lateral Twist')
    return float(values[0]), 0., float(values[5])


def assert_ros_environment(env=None):
    env = os.environ if env is None else env
    if (env.get('ROS_DOMAIN_ID') != '117' or env.get('ROS_LOCALHOST_ONLY') != '1'
            or env.get('ROS_DISTRO') != 'humble'):
        raise RuntimeError('required ROS_DOMAIN_ID=117 ROS_LOCALHOST_ONLY=1 ROS_DISTRO=humble')


@dataclass(frozen=True)
class CommandReceipt:
    sequence: int
    rx_wall: float
    values: tuple
    planar: tuple | None
    reason: str


class LatestCommandMailbox:
    """O(1)最新不可变命令；非法事件先锁存消费，避免被后来的合法包静默覆盖。"""

    def __init__(self):
        self._lock = Lock()
        self._sequence = 0
        self._latest = self._invalid = None

    def receive(self, values, rx_wall):
        values = tuple(values)
        try:
            planar = validated_twist(values)
        except (ValueError, TypeError):
            planar = None
        with self._lock:
            self._sequence += 1
            receipt = CommandReceipt(self._sequence, rx_wall, values, planar,
                'received' if planar is not None else 'invalid_twist')
            self._latest = receipt
            if planar is None:
                self._invalid = receipt

    def take_after(self, sequence):
        with self._lock:
            if self._invalid is not None and self._invalid.sequence > sequence:
                receipt, self._invalid = self._invalid, None
                return receipt
            return self._latest if self._latest is not None and self._latest.sequence > sequence else None

    def clear(self, rx_wall):
        with self._lock:
            self._sequence += 1
            self._latest = self._invalid = CommandReceipt(self._sequence, rx_wall, (), None, 'cleared')

    def stats(self):
        with self._lock:
            return self._sequence, self._latest.rx_wall if self._latest is not None else None


class RosBridge:
    def __init__(self, profile, controller, com, telemetry):
        assert_ros_environment()
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import Twist, TransformStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image, CameraInfo, Imu, PointCloud2, PointField
        from tf2_msgs.msg import TFMessage
        from rosgraph_msgs.msg import Clock
        self.rclpy = rclpy
        self.types = dict(Twist=Twist, TransformStamped=TransformStamped, Odometry=Odometry,
            Image=Image, CameraInfo=CameraInfo, Imu=Imu, PointCloud2=PointCloud2,
            PointField=PointField, TFMessage=TFMessage, Clock=Clock)
        rclpy.init(args=[])
        self.node = rclpy.create_node('viper_engineering_bridge')
        self.p, self.controller, self.com, self.telemetry = profile, controller, com, telemetry
        self.frames, self.topics = profile['ros']['frames'], profile['ros']['topics']
        self.pubs = {}
        for key in self.topics:
            if key == 'command':
                continue
            typename = ('Odometry' if key == 'odometry' else 'TFMessage' if key == 'tf' else
                        'Clock' if key == 'clock' else 'CameraInfo' if 'camera_info' in key else
                        'Imu' if key == 'imu' else 'PointCloud2' if key in ('depth_points', 'lidar_raw', 'lidar_effective') else 'Image')
            self.pubs[key] = self.node.create_publisher(self.types[typename], self.topics[key],
                10 if key in ('clock', 'tf', 'odometry') else qos_profile_sensor_data)
        self.subscription = self.node.create_subscription(Twist, self.topics['command'], self.command, 10)
        self.k = camera_intrinsics(*profile['sensors']['stereo']['resolution'],
            *[profile['sensors']['stereo'][k] for k in ('focal_length_mm', 'horizontal_aperture_mm', 'vertical_aperture_mm')])
        self.counts = {}
        self.mailbox = LatestCommandMailbox()
        self.consumed_sequence = 0
        self._stop_executor, self._executor_error = Event(), None
        from rclpy.executors import SingleThreadedExecutor
        self._executor = SingleThreadedExecutor(context=self.node.context)
        self._executor.add_node(self.node)
        self._executor_thread = Thread(target=self._serve_dds, name='viper-dds', daemon=True)
        self._executor_thread.start()

    def command(self, msg):
        # 仅DDS线程运行：不碰Drive/Kit/PhysX，不做文件IO。
        values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        self.mailbox.receive(values, time.monotonic())

    def _serve_dds(self):
        try:
            while not self._stop_executor.is_set() and self.rclpy.ok():
                self._executor.spin_once(timeout_sec=.01)
        except Exception as exc:
            if not self._stop_executor.is_set():
                self._executor_error = repr(exc)

    def check_executor(self):
        if self._executor_error is not None:
            raise RuntimeError('DDS executor failed: '+self._executor_error)

    def consume_command(self):
        """只由物理主线程在反馈读取之后调用；不刷新真实接收时间。"""
        self.check_executor()
        receipt = self.mailbox.take_after(self.consumed_sequence)
        if receipt is None:
            return
        self.consumed_sequence = receipt.sequence
        if receipt.planar is None:
            self.controller.receive_twist(float('nan'), 0, 0, receipt.rx_wall)
            self.telemetry({'event':'rejected_twist' if receipt.reason == 'invalid_twist' else 'command_cleared',
                'components':[repr(v) for v in receipt.values], 'rx_wall':receipt.rx_wall,
                'rx_sequence':receipt.sequence})
        else:
            accepted = self.controller.receive_twist(*receipt.planar, receipt.rx_wall)
            self.telemetry({'event':'twist', 'components':receipt.values, 'accepted':accepted,
                'rx_wall':receipt.rx_wall, 'rx_sequence':receipt.sequence})

    @property
    def rx_count(self):
        return self.mailbox.stats()[0]

    @property
    def last_rx_wall(self):
        return self.mailbox.stats()[1]

    def clear_commands(self):
        self.mailbox.clear(time.monotonic())

    @staticmethod
    def stamp(msg, seconds):
        ns = round(seconds*1e9)
        msg.sec, msg.nanosec = divmod(ns, 1_000_000_000)

    def header(self, msg, stamp, frame):
        self.stamp(msg.header.stamp, stamp)
        msg.header.frame_id = frame

    def publish(self, key, msg):
        self.pubs[key].publish(msg)
        self.counts[key] = self.counts.get(key, 0) + 1

    def transform(self, parent, child, xyz, xyzw, stamp):
        msg = self.types['TransformStamped']()
        self.header(msg, stamp, parent)
        msg.child_frame_id = child
        for axis, val in zip('xyz', xyz):
            setattr(msg.transform.translation, axis, float(val))
        for axis, val in zip('xyzw', xyzw):
            setattr(msg.transform.rotation, axis, float(val))
        return msg

    def state(self, stamp, pose, velocity):
        clock = self.types['Clock']()
        self.stamp(clock.clock, stamp)
        self.publish('clock', clock)
        odom = self.types['Odometry']()
        self.header(odom, stamp, self.frames['odom'])
        odom.child_frame_id = self.frames['base']
        for axis, value in zip('xyz', pose[:3]):
            setattr(odom.pose.pose.position, axis, float(value))
        for axis, value in zip('xyzw', pose[3:]):
            setattr(odom.pose.pose.orientation, axis, float(value))
        r = rotation_matrix(pose[3:])
        for vector, values in ((odom.twist.twist.linear, r.T @ velocity[:3]),
                               (odom.twist.twist.angular, r.T @ velocity[3:])):
            for axis, value in zip('xyz', values):
                setattr(vector, axis, float(value))
        self.publish('odometry', odom)
        f = self.frames
        # map/odom共享实际仿真世界坐标；COM根为base_link，地面原点固定偏移。
        transforms = [self.transform(f['map'], f['odom'], [0]*3, [0,0,0,1], stamp),
            self.transform(f['odom'], f['base'], pose[:3], pose[3:], stamp),
            self.transform(f['base'], f['footprint'], -np.asarray(self.com), [0,0,0,1], stamp)]
        for key in ('left', 'right', 'lidar', 'imu'):
            sensor = self.p['sensors']['stereo' if key in ('left','right') else key]
            xyz = sensor[key+'_origin_m' if key in ('left','right') else 'origin_m']
            frame = f[key+'_optical' if key in ('left','right') else key]
            transforms.append(self.transform(f['footprint'], frame, xyz,
                OPTICAL_XYZW if key in ('left','right') else [0,0,0,1], stamp))
        self.publish('tf', self.types['TFMessage'](transforms=transforms))

    def cloud(self, key, stamp, frame, xyz, *, timing=None):
        timing = _SensorTimings() if timing is None else timing
        with timing.measure('msg_pack'):
            msg = self.types['PointCloud2']()
            self.header(msg, stamp, frame)
            msg.height, msg.width = 1, len(xyz)
            msg.fields = [self.types['PointField'](name=name, offset=i*4, datatype=7, count=1)
                          for i, name in enumerate('xyz')]
            msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 12, len(xyz)*12, True
            msg.data = byte_payload(np.asarray(xyz, dtype='<f4').reshape(-1, 3))
        timing.publish(self, key, msg)
        return timing.ms

    def cameras(self, samples):
        timing = _SensorTimings()
        bounds = self.p['sensors']['effective_observation']
        for side, (stamp, rgb, depth, sample_timing) in samples.items():
            frame = self.frames[side+'_optical']
            with timing.measure('msg_pack'):
                msg = self.types['Image']()
                self.header(msg, stamp, frame)
                msg.height, msg.width = rgb.shape[:2]
                msg.encoding, msg.step, msg.data = 'rgb8', rgb.shape[1]*3, byte_payload(rgb)
            timing.publish(self, side+'_image', msg)
            with timing.measure('msg_pack'):
                info = self.types['CameraInfo']()
                self.header(info, stamp, frame)
                info.width, info.height = rgb.shape[1], rgb.shape[0]
                info.distortion_model, info.d, info.k = 'plumb_bob', [0.]*5, self.k
                info.r = [1.,0.,0.,0.,1.,0.,0.,0.,1.]
                fx, fy, cx, cy = self.k[0], self.k[4], self.k[2], self.k[5]
                tx = -fx*self.p['sensors']['stereo']['baseline_m'] if side == 'right' else 0.
                info.p = [fx,0.,cx,tx, 0.,fy,cy,0., 0.,0.,1.,0.]
            timing.publish(self, side+'_camera_info', info)
            if side == 'left':
                with timing.measure('msg_pack'):
                    msg = self.types['Image']()
                    self.header(msg, stamp, frame)
                    msg.height, msg.width = depth.shape[:2]
                    msg.encoding, msg.step = '32FC1', depth.shape[1]*4
                    msg.data = byte_payload(np.asarray(depth, dtype='<f4'))
                timing.publish(self, 'depth', msg)
                with timing.measure('depth_math'):
                    xyz = depth_points(depth, self.k, bounds['minimum_range_m'], bounds['maximum_range_m'])
                self.cloud('depth_points', stamp, frame, xyz, timing=timing)
                with timing.measure('sensor_log'):
                    self.telemetry({'event':'depth', 'time':stamp, 'points':len(xyz),
                        'finite_pixels':int(np.isfinite(depth).sum()), 'timing':str(sample_timing),
                        'range_max':float(np.linalg.norm(xyz,axis=1).max()) if len(xyz) else None})
        return timing.ms

    def imu(self, sample):
        if sample is None:
            return
        msg = self.types['Imu']()
        self.header(msg, sample['time'], self.frames['imu'])
        for axis, value in zip('xyzw', wxyz_to_xyzw(sample['orientation'])):
            setattr(msg.orientation, axis, float(value))
        for field in ('linear_acceleration', 'angular_velocity'):
            for axis, value in zip('xyz', sample[field]):
                setattr(getattr(msg, field), axis, float(value))
        self.publish('imu', msg)
        self.telemetry({'event':'imu', **{key: np.asarray(value).tolist() for key,value in sample.items()}})

    def lidar(self, sample):
        timing = _SensorTimings()
        if sample is None:
            return timing.ms
        stamp, xyz, flags = sample
        with timing.measure('lidar_filter_math'):
            valid = ((flags.astype(np.uint32)&64)!=0) & np.isfinite(xyz).all(axis=1)
            raw = xyz[valid]
            bounds = self.p['sensors']['effective_observation']
            effective = effective_lidar(xyz, flags, bounds['minimum_range_m'], bounds['maximum_range_m'], bounds['horizontal_fov_deg'])
        self.cloud('lidar_raw', stamp, self.frames['lidar'], raw, timing=timing)
        self.cloud('lidar_effective', stamp, self.frames['lidar'], effective, timing=timing)
        with timing.measure('sensor_log'):
            self.telemetry({'event':'lidar', 'time':stamp, 'rays':len(xyz), 'raw_valid':len(raw),
                'effective':len(effective), 'effective_range_max':float(np.linalg.norm(effective,axis=1).max()) if len(effective) else None,
                'effective_azimuth_max':float(np.abs(np.arctan2(effective[:,1],effective[:,0])).max()) if len(effective) else None})
        return timing.ms

    def close(self):
        self._stop_executor.set()
        self._executor_thread.join(timeout=2.)
        self._executor.shutdown(timeout_sec=1.)
        self.node.destroy_node()
        self.rclpy.shutdown()
