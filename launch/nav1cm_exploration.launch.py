"""显式新导航组合；默认仅规划，策略与控制均由操作者选择。"""
import os
import yaml
from math import isfinite
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 这些值只属于新入口；既有 exploration_navigation 和训练默认保持原样。
def _topic_defaults():
    config = Path(__file__).resolve().parents[1] / 'config/p4_topics.yaml'
    if not config.is_file():
        config = Path(get_package_share_directory('lunar_pure_exploration_ros')) / 'config/p4_topics.yaml'
    return yaml.safe_load(config.read_text(encoding='utf-8'))['launch']


DEFAULTS = {'global_period_s': '15.0',
 'global_soft_clearance_m': '0.4',
 'global_clearance_weight': '0.5',
 'command_hold_s': '0.0',
 'command_spin_prepare_s': '0.0',
 'control_backend': 'path_mppi',
 'exploration_strategy': 'none',
 'enable_controller': 'false',
 'drl_actor': '',
 'allow_unknown': 'false',
 'build_dir': '',
 'platform_config': '',
 'planner_params_file': '',
 'use_sim_time': 'false',
 'map_frame': 'map',
 'odom_frame': 'odom',
 'base_frame': 'base_link',
 'map_qos_reliability': 'reliable',
 'map_qos_durability': 'volatile',
 'p3_map_timeout_s': '3.0',
 'odometry_timeout_s': '0.5',
 'mppi_time_steps': '40',
 'mppi_compute_envelope_s': '0.05',
 'initial_travel_direction': '',
 'coarse_resolution_m': '0.2',
 **_topic_defaults()}

def _compose(context):
    values = {k: LaunchConfiguration(k).perform(context).strip() or v for k,v in DEFAULTS.items()}
    for name in ('global_soft_clearance_m','global_clearance_weight'):
        values[name] = float(values[name])
        if not isfinite(values[name]) or values[name] < 0:
            raise ValueError(name + ' must be finite and nonnegative')
    values["command_spin_prepare_s"] = float(values["command_spin_prepare_s"])
    if not isfinite(values["command_spin_prepare_s"]) or values["command_spin_prepare_s"] < 0:
        raise ValueError("command_spin_prepare_s must be finite and nonnegative")
    values["global_period_s"] = float(values["global_period_s"])
    if not isfinite(values["global_period_s"]) or values["global_period_s"] <= 0:
        raise ValueError("global_period_s must be finite and positive")
    values["p3_map_timeout_s"] = float(values["p3_map_timeout_s"])
    if not isfinite(values["p3_map_timeout_s"]) or values["p3_map_timeout_s"] < 0:
        raise ValueError("p3_map_timeout_s must be finite and nonnegative")
    values["odometry_timeout_s"] = float(values["odometry_timeout_s"])
    if not isfinite(values["odometry_timeout_s"]) or values["odometry_timeout_s"] <= 0:
        raise ValueError("odometry_timeout_s must be finite and positive")
    if values["initial_travel_direction"] not in ("", "forward", "reverse"):
        raise ValueError("initial_travel_direction must be forward or reverse")
    values["mppi_time_steps"] = int(values["mppi_time_steps"])
    if values["mppi_time_steps"] <= 0:
        raise ValueError("mppi_time_steps must be positive")


    values['mppi_compute_envelope_s'] = float(values['mppi_compute_envelope_s'])
    if not isfinite(values['mppi_compute_envelope_s']) or values['mppi_compute_envelope_s'] <= 0:
        raise ValueError('mppi_compute_envelope_s must be finite and positive')
    if values["control_backend"] not in {"path_mppi", "mpcc_cmd_vel"}:
        raise RuntimeError("control_backend 必须为 path_mppi/mpcc_cmd_vel")
    strategy = values["exploration_strategy"]
    if strategy not in {"none", "frontier", "drl"}:
        raise RuntimeError("exploration_strategy 必须为 none/frontier/drl")
    for key in ("enable_controller", "use_sim_time", "allow_unknown"):
        if values[key] not in {"true", "false"}: raise RuntimeError(key+" 必须为 true/false")
    if strategy == "drl" and not values["drl_actor"]:
        raise RuntimeError("drl 策略需要显式 drl_actor；权重加载与校验由可选 DRL 包负责")
    values["build_dir"] = values["build_dir"] or os.environ.get("NAV1CM_BUILD_DIR", "")
    if not values["build_dir"]: raise RuntimeError("build_dir 必须指定仓库外缓存，或设置 NAV1CM_BUILD_DIR")
    config = values["platform_config"] or str(Path(get_package_share_directory("lunar_nav1cm_planner"))/"config/wheel.yaml")
    frames = {k: values[k] for k in ("map_frame","odom_frame","base_frame")}
    inputs = {k: values[k] for k in ("map_topic","odometry_topic","tf_topic","tf_static_topic")}
    qos = {k: values[k] for k in ("map_qos_reliability","map_qos_durability")}
    if qos["map_qos_reliability"] not in {"reliable","best_effort"} or qos["map_qos_durability"] not in {"volatile","transient_local"}:
        raise RuntimeError("无效地图输入 QoS")
    clock = {"use_sim_time": values["use_sim_time"] == "true"}
    def node(package, executable, params):
        files = [values['planner_params_file']] if executable == 'navigation_task_node' and values['planner_params_file'] else []
        return Node(package=package, executable=executable, output="screen", parameters=[*files, {**params,**clock}])
    actions = [
        node("lunar_incremental_navigation_ros", "lunar_exploration_map_node", {
            **frames, **inputs, **qos, "platform_config": config,
            "exploration_map_topic": values["exploration_map_topic"], "policy_map_service": values["policy_map_service"],
            "coarse_resolution_m": float(values["coarse_resolution_m"])}),
        node("lunar_nav1cm_planner", "navigation_task_node", {
            **inputs, **qos, "map_frame": frames["map_frame"], "wheel_config": config,
            "global_period_s":values["global_period_s"],
            "global_soft_clearance_m":values["global_soft_clearance_m"],
            "global_clearance_weight":values["global_clearance_weight"],
            "command_hold_s":float(values["command_hold_s"]),
            "command_spin_prepare_s":values["command_spin_prepare_s"],
            "control_backend":values["control_backend"],
            "enable_controller":values["enable_controller"]=="true",
            "command_topic":values["command_topic"],
            "execution_contract_enabled":values["enable_controller"]=="true" and values["control_backend"]=="path_mppi",
            "odometry_timeout_s":values["odometry_timeout_s"],
            "initial_travel_direction":values["initial_travel_direction"],
            "execution_reference_topic":values["execution_reference_topic"],"tracking_status_topic":values["tracking_status_topic"],
            **{k: values[k] for k in ("action_name","goal_topic","global_route_topic","local_path_topic","build_dir")},
            "allow_unknown": values["allow_unknown"] == "true"}),
    ]
    shared_policy = {**{k: values[k] for k in ("task_topic","status_topic","odometry_topic","tf_topic","tf_static_topic")},
                     **frames, "platform_config": config}
    if strategy == "frontier":
        actions.append(node("lunar_pure_exploration_ros", "incremental_exploration_node", {
            **shared_policy, **{k: values[k] for k in ("current_goal_topic","frontiers_topic","task_boundary_topic","task_map_markers_topic")},
            "diagnostics_topic": values["exploration_diagnostics_topic"], "platform_selector": "wheel", "navigation_action": values["action_name"],
            "exploration_map_topic": values["exploration_map_topic"]}))
    elif strategy == "drl":
        actions.append(node("lunar_drl_exploration", "lunar_drl_inference_node", {
            **shared_policy, "navigation_backend": "nav1cm", "actor_path": values["drl_actor"],
            **{k: values[k] for k in ("action_name","policy_map_service","tf_static_topic","global_route_topic")}}))
    if values["enable_controller"] == "true" and values["control_backend"] == "path_mppi":
        actions.append(node("lunar_pure_wheeled_controller", "lunar_pure_wheeled_controller_node.py", {
            **frames, **qos, **{k: values[k] for k in ("odometry_topic","tf_topic","tf_static_topic","command_topic")},
            "platform_config": config, "input_mode": "incremental_path", "controller_backend": "mppi_path_follower",
            "execution_contract_enabled":True,
            "execution_reference_topic":values["execution_reference_topic"],"tracking_status_topic":values["tracking_status_topic"],
            "standard_map_source": "direct_p3", "p3_map_topic": values["map_topic"],
            "p3_map_timeout_s": values["p3_map_timeout_s"],
            "odometry_timeout_s": values["odometry_timeout_s"], "mppi_time_steps": values["mppi_time_steps"],


            "mppi_compute_envelope_s": values["mppi_compute_envelope_s"],
            "path_topic": values["local_path_topic"], "mppi_diagnostics_topic": values["diagnostics_topic"],
            "mppi_nominal_response_time_s": .15, "allow_unknown": values["allow_unknown"] == "true"}))
    return actions

def generate_launch_description():
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v) for k,v in DEFAULTS.items()],
                              OpaqueFunction(function=_compose)])
