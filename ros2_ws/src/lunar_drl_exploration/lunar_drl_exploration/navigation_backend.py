"""显式选择可选导航后端；默认保持既有增量导航。"""


def adapter_class(navigation_backend='incremental_v2'):
    if navigation_backend == 'incremental_v2':
        from .ros_messages import RosNavigationAdapter
        return RosNavigationAdapter
    if navigation_backend == 'nav1cm':
        from .nav1cm_navigation import Nav1cmNavigationAdapter
        return Nav1cmNavigationAdapter
    raise ValueError('unknown navigation_backend: '+str(navigation_backend))


def create_navigation_adapter(node, *, navigation_backend='incremental_v2', **kwargs):
    cls=adapter_class(navigation_backend)
    if navigation_backend == 'incremental_v2':
        kwargs.pop('tf_static_topic',None)
    else:
        for name in ('diagnostics_topic','path_reference_topic','tracking_status_topic'):
            kwargs.pop(name,None)
    return cls(node,**kwargs)


def owned_nav1cm_specs(namespace, *, platform_config, build_dir, use_sim_time):
    """与公共组合一致的进程规格，全部主题在调用者私有命名空间内。"""
    ns=namespace
    inputs=dict(map_topic=ns+'/grid',odometry_topic=ns+'/odom',tf_topic=ns+'/tf',
        tf_static_topic=ns+'/tf_static',use_sim_time=use_sim_time,
        map_qos_reliability='reliable',map_qos_durability='transient_local')
    mapping=dict(inputs,platform_config=platform_config,exploration_map_topic=ns+'/map',
                 policy_map_service=ns+'/get_map',coarse_resolution_m=1.)
    planning=dict(inputs,wheel_config=platform_config,action_name=ns+'/navigate',
        goal_topic=ns+'/goal',global_route_topic=ns+'/route',local_path_topic=ns+'/path',
        build_dir=build_dir,allow_unknown=False)
    follower=dict(inputs,platform_config=platform_config,input_mode='incremental_path',
        controller_backend='mppi_path_follower',standard_map_source='direct_p3',
        p3_map_topic=ns+'/grid',path_topic=ns+'/path',command_topic=ns+'/cmd_vel',
        mppi_diagnostics_topic=ns+'/mppi_diagnostics',mppi_nominal_response_time_s=.15,allow_unknown=False)
    follower.pop('map_topic')
    return [('lunar_incremental_navigation_ros','lunar_exploration_map_node',mapping),
            ('lunar_nav1cm_planner','navigation_task_node',planning),
            ('lunar_pure_wheeled_controller','lunar_pure_wheeled_controller_node.py',follower)]


def validate_nav1cm_platform(platform):
    """新模式显式采用公共0.2m能力文件，不能静默沿用旧训练净空。"""
    if float(platform.capability.get('minimum_clearance_m',-1.)) != .2:
        raise ValueError('nav1cm requires the shared 0.2 m clearance platform profile')
