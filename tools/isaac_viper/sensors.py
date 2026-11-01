"""真实传感器的几何转换；SDK 导入延迟到运行端，纯数学可独立验证。"""
import math
import numpy as np

# optical: X右/Y下/Z前，相对车体X前/Y左/Z上。
OPTICAL_XYZW = [-.5, .5, -.5, .5]


def camera_intrinsics(width, height, focal, horizontal_aperture, vertical_aperture):
    return [width*focal/horizontal_aperture, 0., (width-1)/2,
            0., height*focal/vertical_aperture, (height-1)/2, 0., 0., 1.]


def depth_points(depth, k, minimum, maximum):
    depth = np.asarray(depth).squeeze()
    v, u = np.indices(depth.shape)
    xyz = np.stack(((u-k[2])*depth/k[0], (v-k[5])*depth/k[4], depth), axis=-1).reshape(-1, 3)
    ranges = np.linalg.norm(xyz, axis=1)
    return xyz[np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0) &
               (ranges >= minimum) & (ranges <= maximum)].astype(np.float32)


def effective_lidar(xyz, flags, minimum, maximum, fov):
    xyz = np.asarray(xyz)
    ranges = np.linalg.norm(xyz, axis=1)
    azimuth = np.abs(np.arctan2(xyz[:, 1], xyz[:, 0]))
    good = ((np.asarray(flags, dtype=np.uint32) & 64) != 0)
    good &= np.isfinite(xyz).all(axis=1) & (ranges >= minimum) & (ranges <= maximum)
    good &= azimuth <= math.radians(fov/2) + 1e-12
    return xyz[good].astype(np.float32)


def wxyz_to_xyzw(q):
    return [float(q[1]), float(q[2]), float(q[3]), float(q[0])]


def rotation_matrix(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError('invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def pointcloud_bytes(xyz):
    return np.asarray(xyz, dtype='<f4').reshape(-1, 3).tobytes()


def active_render_products(stage):
    """从当前活动stage重读关系，不沿用相机/viewport最终设置前的快照。"""
    return [{'path':str(prim.GetPath()),
             'camera':[str(p) for p in prim.GetRelationship('camera').GetTargets()]}
            for prim in stage.Traverse() if prim.GetTypeName() == 'RenderProduct']


class NativePhysicsProbe:
    """只读原生四轮材料/offset与地形射线；不将射线冒充RTX或solver contact。"""
    def __init__(self, sim_view, stage, manager):
        paths = [f'/World/ProjectVIPER/{c}_wheel' for c in ('fl','fr','rl','rr')]
        self.view = sim_view.create_rigid_body_view(paths)
        self.paths = [str(p) for p in self.view.prim_paths]
        if self.view.count != 4 or set(self.paths) != set(paths):
            raise RuntimeError(f'native wheel probe identity mismatch: {self.paths}')
        self.stage, self.manager = stage, manager

    @staticmethod
    def _array(array):
        a = np.asarray(array)
        def preserve(value):
            if isinstance(value, list):
                return [preserve(v) for v in value]
            return value if math.isfinite(value) else str(value)
        return {'shape':list(a.shape), 'values':preserve(a.tolist()),
                'all_finite':bool(np.isfinite(a).all())}

    def capture(self, phase, *, include_raycast=False):
        poses = self.view.get_transforms().copy()
        result = {'event':'native_physics_probe', 'phase':phase,
            'time':float(self.manager.get_simulation_time()), 'paths':self.paths,
            'count':self.view.count, 'max_shapes':self.view.max_shapes,
            'actual_dt':float(self.manager.get_physics_dt('/World/PhysicsScene')),
            'actual_steps_per_second':self.stage.GetPrimAtPath('/World/PhysicsScene').GetAttribute('physxScene:timeStepsPerSecond').Get(),
            'material_columns':['static_friction','dynamic_friction','restitution'],
            'material_properties':self._array(self.view.get_material_properties().copy()),
            'contact_offsets':self._array(self.view.get_contact_offsets().copy()),
            'rest_offsets':self._array(self.view.get_rest_offsets().copy()),
            'wheel_world_poses_xyzw':poses.tolist()}
        if include_raycast:
            import carb
            import omni.physx
            query = omni.physx.get_physx_scene_query_interface()
            rays = []
            for path, pose in zip(self.paths, poses):
                center, hits = pose[:3], []
                def collect(hit):
                    hits.append({'collision':str(hit.collision), 'rigid_body':str(hit.rigid_body),
                        'material':str(hit.material), 'position':[float(v) for v in hit.position],
                        'normal':[float(v) for v in hit.normal], 'distance':float(hit.distance),
                        'face_index':int(hit.face_index)})
                    return True
                any_hit = query.raycast_all(carb.Float3(float(center[0]),float(center[1]),float(center[2]+.5)),
                    carb.Float3(0,0,-1), 2., collect, True)
                hits.sort(key=lambda h:h['distance'])
                for hit in hits:
                    if hit['collision'].startswith('/World/Environment/'):
                        delta = center-np.asarray(hit['position'])
                        hit['center_vertical_distance_m'] = float(delta[2])
                        hit['triangle_plane_distance_m'] = float(delta@np.asarray(hit['normal']))
                rays.append({'wheel':path, 'origin_world':center.tolist(),
                    'ray_start_world':[float(center[0]),float(center[1]),float(center[2]+.5)],
                    'direction_world':[0,0,-1], 'range_m':2., 'any_hit':bool(any_hit), 'hits':hits})
            result['raycast_all'] = rays
            result['raycast_is_not_solver_contact_skin'] = True
        return result


class RealSensors:
    """PhysX IMU + RTX相机/雷达；不提供代理数据回退。"""

    def __init__(self, stage, profile):
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera, Lidar, LidarSensor
        from isaacsim.sensors.experimental.physics import IMU, IMUSensor
        import omni.replicator.core as rep
        rig = '/World/ProjectVIPER/chassis/FrontSensors'
        # /Render 整体禁用来自 Task2。先屏蔽旧叶再激活父节点，且期间不更新app。
        if not stage.GetPrimAtPath('/Render').IsActive():
            stage.OverridePrim('/Render').SetActive(True)
            for prim in list(stage.Traverse()):
                if str(prim.GetPath()).startswith('/Render/') and prim.GetTypeName() in ('RenderProduct', 'RenderSettings'):
                    prim.SetActive(False)
        stereo = profile['sensors']['stereo']
        width, height = stereo['resolution']
        for side in ('left', 'right'):
            # Task2只创建设备几何/UsdCamera；SDK包装现有prim要求显式schema。
            if not stage.GetPrimAtPath(rig+'/'+side+'_camera').ApplyAPI('OmniSensorAPI'):
                raise RuntimeError('cannot apply OmniSensorAPI to camera')
        self.cameras = {side: CameraSensor(RtxCamera(rig+'/'+side+'_camera',
                           tick_rate=stereo['frequency_hz'], reset_xform_op_properties=False), resolution=(height, width),
                           annotators=['rgb', 'distance_to_image_plane']) for side in ('left', 'right')}
        self.times = {}
        for side, sensor in self.cameras.items():
            timer = rep.AnnotatorRegistry.get_annotator('IsaacReadSimulationTime')
            timer.initialize(resetOnStop=True)
            timer.attach(str(sensor.render_product.GetPath()))
            self.times[side] = timer
        # 原mount为Xform；另建传感器子prim以保留冻结外参。
        self.imu = IMUSensor(IMU(rig+'/imu/PhysX', translations=[[0, 0, 0]],
            linear_acceleration_filter_size=1,
            angular_velocity_filter_size=1, orientation_filter_size=1))
        lidar = Lidar(rig+'/lidar/RTX', translations=[[0, 0, 0]],
            tick_rate=profile['sensors']['lidar']['frequency_hz'], accumulate_outputs=True,
            aux_output_level='BASIC', attributes={
                'omni:sensor:Core:elementsCoordsType': 'CARTESIAN',
                'omni:sensor:Core:outputFrameOfReference': 'SENSOR',
                # 保留全部128 emitter/channel/fireTime数组；只降发射pattern频率，
                # 每10Hz扫描360个azimuth tick，降低与正式训练共享GPU时的RTX负载。
                'omni:sensor:Core:patternFiringRateHz': 3600,
                'omni:sensor:Core:nearRangeM': .05,
                'omni:sensor:Core:farRangeM': 200.})
        self.lidar = LidarSensor(lidar, annotators=['generic-model-output'])
        self.last = {}

    def imu_sample(self):
        if not self.imu.get_sensor_reading(read_gravity=True).is_valid:
            return None
        sample = self.imu.get_data(read_gravity=True)
        stamp = float(sample['time'])
        if stamp <= self.last.get('imu', -1):
            return None
        self.last['imu'] = stamp
        return {key: np.asarray(value).copy() if isinstance(value, np.ndarray) else value
                for key, value in sample.items()}

    def camera_samples(self):
        samples = {}
        for side, sensor in self.cameras.items():
            timing = self.times[side].get_data()
            stamp = float(timing['simulationTime'])
            if stamp <= self.last.get(side, -1):
                continue
            rgb, info = sensor.get_data('rgb')
            depth, depth_info = sensor.get_data('distance_to_image_plane')
            if rgb is None or depth is None:
                continue
            self.last[side] = stamp
            samples[side] = (stamp, rgb.numpy().copy(), depth.numpy().copy(), timing)
        return samples

    def lidar_sample(self):
        from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data
        from isaacsim.sensors.experimental.rtx.generic_model_output import CoordsType
        data, info = self.lidar.get_data('generic-model-output')
        if data is None:
            return None
        gmo = parse_generic_model_output_data(data)
        if gmo is None or not gmo.numElements:
            return None
        if gmo.elementsCoordsType != CoordsType.CARTESIAN:
            raise RuntimeError('LiDAR coordinates are not CARTESIAN')
        stamp = int(gmo.timestampNs)
        if stamp <= self.last.get('lidar', -1):
            return None
        self.last['lidar'] = stamp
        xyz = np.column_stack((gmo.x, gmo.y, gmo.z)).copy()
        return stamp/1e9, xyz, np.asarray(gmo.flags).copy()
