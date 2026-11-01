"""薄 ROS 推理入口：只加载 Actor，复用共享部署生命周期。"""
import json
from pathlib import Path


def read_policy_settings(path,model):
    from .config import canonical_model_config, config_record
    from .contracts import SensorSpec
    record=json.loads(Path(path).read_text(encoding='utf-8'))
    if canonical_model_config(record['model'])!=config_record(model):
        raise ValueError('actor model/config mismatch')
    mode=record.get('policy_graph_mode','fine');cap=record.get('remote_frontier_positions',32)
    if mode not in ('fine','a2') or type(cap) is not int or cap<0:
        raise ValueError('invalid observed policy graph settings')
    return dict(sensor=SensorSpec(**record['sensor']),policy_graph_mode=mode,remote_frontier_positions=cap)


def attach_node(node):
    from .config import GraphConfig,load_platform_config
    from .runtime import ActorPolicy,InferenceRuntime
    defaults=dict(actor_path='',actor_config_path='',navigation_backend='incremental_v2',platform_config='',
        task_topic='/Car/T4/exploration/task',status_topic='/Car/T4/exploration/drl_status',
        action_name='/Car/T4/navigation/navigate_to_pose',policy_map_service='/Car/T4/mapping/get_policy_map',
        odometry_topic='/Car/T3/localization/odometry',tf_topic='/tf',tf_static_topic='/tf_static',
        global_route_topic='/Car/T4/planning/global_route',map_frame='map',odom_frame='odom',base_frame='base_link')
    values={k:node.declare_parameter(k,v).value for k,v in defaults.items()}
    if not values['actor_path']:raise ValueError('explicit actor_path required')
    actor=Path(values.pop('actor_path'));config_path=values.pop('actor_config_path')
    policy=ActorPolicy.load(actor)
    settings=read_policy_settings(config_path or actor.parent.parent/'config.json',policy.actor.config)
    platform_path=values.pop('platform_config')
    platform=load_platform_config(Path(platform_path) if platform_path else None)
    if values['navigation_backend']=='nav1cm':
        from .navigation_backend import validate_nav1cm_platform
        validate_nav1cm_platform(platform)
    values.pop('base_frame')
    return InferenceRuntime.attach(node,policy,graph_config=GraphConfig(platform=platform),**settings,**values)


def main(args=None):
    import time
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('lunar_drl_inference')
    runtime=None
    try:
        runtime=attach_node(node)
        rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if runtime is not None:
            runtime.cancel();deadline=time.monotonic()+10.
            while runtime.adapter.inflight and time.monotonic()<deadline and rclpy.ok():
                rclpy.spin_once(node,timeout_sec=.05)
            runtime.adapter.close()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()