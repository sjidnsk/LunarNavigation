"""轮式平台薄入口；节点、任务和控制均复用公共导航组合。"""
import importlib.util
import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resource(relative):
    source = Path(__file__).resolve().parents[1] / relative
    if source.is_file():
        return source
    return Path(get_package_share_directory('lunar_pure_exploration_ros')) / relative


def _render_rviz(values, output_dir):
    # 直接复用公共渲染器与模板，不维护第二套话题或面板配置。
    helper_path = _resource('scripts/simulation/p4_topics.py')
    if not helper_path.is_file():
        helper_path = _resource('launch/p4_topics.py')
    spec = importlib.util.spec_from_file_location('wheel_p4_topics', helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    contract = helper.load_topic_contract(_resource('config/p4_topics.yaml').parent.parent)
    contract['launch'].update({key: values[key] for key in contract['launch']})
    template_path = _resource('scripts/simulation/p4_navigation.rviz')
    if not template_path.is_file():
        template_path = _resource('rviz/p4_navigation.rviz')
    template = yaml.safe_load(template_path.read_text(encoding='utf-8'))
    rendered = helper.render_rviz_config(template, contract)
    rendered['Visualization Manager']['Global Options']['Fixed Frame'] = values['map_frame']
    if output_dir:
        Path(output_dir).mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', suffix='.rviz',
                                     prefix='wheel-navigation-', dir=output_dir or None, delete=False) as stream:
        yaml.safe_dump(rendered, stream, allow_unicode=True, sort_keys=False)
        return stream.name


def _strings(values):
    return {key: str(value).lower() if isinstance(value, bool) else str(value)
            for key, value in values.items()}


# P4 内部接口沿用唯一公共命名来源；轮式配置只覆盖平台速度出口。
DEFAULTS = _strings(yaml.safe_load(_resource('config/p4_topics.yaml').read_text(encoding='utf-8'))['launch'])
DEFAULTS.update(_strings(yaml.safe_load(_resource('config/wheel_navigation.yaml').read_text(encoding='utf-8'))))


def _compose(context):
    config_file = LaunchConfiguration('config_file').perform(context).strip()
    platform = LaunchConfiguration('platform_type').perform(context).strip() or 'wheel'
    if platform not in {'wheel', 'legged'}:
        raise RuntimeError('unsupported navigation platform_type')
    values = dict(DEFAULTS)
    if platform == 'legged':
        values.update(_strings(yaml.safe_load(_resource('config/legged_navigation.yaml').read_text(encoding='utf-8'))))
    values['platform_selector'] = platform
    if config_file:
        overrides = yaml.safe_load(Path(config_file).read_text(encoding='utf-8'))
        if not isinstance(overrides, dict) or set(overrides) - set(DEFAULTS):
            raise RuntimeError('config_file must contain wheel navigation launch arguments')
        values.update(_strings(overrides))
    explicit = {key: LaunchConfiguration(key).perform(context).strip() for key in DEFAULTS}
    values.update({key: value for key, value in explicit.items() if value})
    alias = LaunchConfiguration('start_controller').perform(context).strip()
    if alias and not explicit['enable_controller']:
        values['enable_controller'] = alias
    values['build_dir'] = values['build_dir'].strip() or os.environ.get('NAV1CM_BUILD_DIR', '').strip()
    if not values['build_dir']:
        raise RuntimeError('build_dir or NAV1CM_BUILD_DIR must specify an external build cache')
    values['platform_config'] = values['platform_config'] or str(_resource('config/' + platform + '_orin.yaml'))
    start_rviz = LaunchConfiguration('start_rviz').perform(context).strip() or 'false'
    if start_rviz not in {'true', 'false'}:
        raise RuntimeError('start_rviz must be true/false')
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(_resource('launch/nav1cm_exploration.launch.py'))),
        launch_arguments=values.items())]
    if start_rviz == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', output='screen',
                            arguments=['-d', _render_rviz(values, LaunchConfiguration('rviz_output_dir').perform(context).strip())],
                            parameters=[{'use_sim_time': values['use_sim_time'] == 'true'}]))
    return actions


def generate_launch_description():
    # 空缺参数由运行配置填充；显式 launch 参数始终优先于 config_file。
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value='') for key in DEFAULTS],
        DeclareLaunchArgument('platform_type', default_value='wheel'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('start_controller', default_value='', description='Legacy alias for enable_controller'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('rviz_output_dir', default_value='', description='RViz generated config directory; defaults to system temporary directory'),
        OpaqueFunction(function=_compose),
    ])
