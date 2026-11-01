"""轮式薄入口静态组合合同；不代表 ROS/Humble/Orin 实测。"""
import importlib.util
from pathlib import Path
import pytest
import yaml
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("wheel_launch_helper", ROOT / "tests/launch/test_exploration_navigation_launch.py")
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


def load(monkeypatch):
    path = ROOT / "launch/wheel_navigation.launch.py"
    assert path.is_file(), "轮式正式薄入口尚未实现"
    monkeypatch.setattr(helper, "LAUNCH", path)
    module, node, include = helper._launch_module(monkeypatch)
    return module, node, include


def compose(monkeypatch, **values):
    module, node, include = load(monkeypatch)
    actions = module._compose(helper._Context({"build_dir": "/outside/cache", **values}))
    return module, actions, node, include


def test_thin_entry_has_one_shared_include_and_idle_defaults(monkeypatch):
    module, actions, node, include = compose(monkeypatch)
    assert len(actions) == 1 and isinstance(actions[0], include)
    assert Path(actions[0].source.path).name == "nav1cm_exploration.launch.py"
    args = actions[0].launch_arguments
    assert args["enable_controller"] == "false"
    assert args["exploration_strategy"] == "frontier"
    assert args["control_backend"] == "mpcc_cmd_vel"
    source = (ROOT / "launch/wheel_navigation.launch.py").read_text(encoding="utf-8")
    assert not any(term in source for term in ("p4_tcp_runtime", "lease_topic", "telemetry_topic", "socket"))


def test_effective_wheel_profile_matches_tcp_capability():
    expected = yaml.safe_load((ROOT / "config/wheel.yaml").read_text(encoding="utf-8"))
    expected["capability"].update(yaml.safe_load((ROOT / "config/tcp_wheel_speed.yaml").read_text(encoding="utf-8")))
    actual = yaml.safe_load((ROOT / "config/wheel_orin.yaml").read_text(encoding="utf-8"))
    assert actual == expected


def test_budgets_topics_and_platform_are_shared(monkeypatch):
    _, actions, _, _ = compose(monkeypatch)
    args = actions[0].launch_arguments
    feedback = yaml.safe_load((ROOT / "config/tcp_p3_feedback.yaml").read_text(encoding="utf-8"))
    for key, value in feedback.items():
        assert float(args[key]) == (0.0 if key == "command_spin_prepare_s" else value)
    topics = yaml.safe_load((ROOT / "config/p4_topics.yaml").read_text(encoding="utf-8"))["launch"]
    for key, value in topics.items():
        assert args[key] == ("/Car/T5/Car_Cmd_Vel" if key == "command_topic" else value)
    assert Path(args["platform_config"]).name == "wheel_orin.yaml"
    assert args["allow_unknown"] == "true"
    assert args["initial_travel_direction"] == "forward"
    assert args["map_qos_reliability"] == "reliable"
    assert args["map_qos_durability"] == "transient_local"
    assert float(args["coarse_resolution_m"]) == .2


def test_build_cache_is_required_and_overrides_survive(monkeypatch):
    module, _, _ = load(monkeypatch)
    monkeypatch.delenv("NAV1CM_BUILD_DIR", raising=False)
    with pytest.raises(RuntimeError, match="build_dir|NAV1CM_BUILD_DIR"):
        module._compose(helper._Context({}))
    monkeypatch.setenv("NAV1CM_BUILD_DIR", "/env/cache")
    actions = module._compose(helper._Context({"enable_controller": "true", "platform_config": "/private/wheel.yaml", "control_backend": "path_mppi"}))
    assert actions[0].launch_arguments["build_dir"] == "/env/cache"
    assert actions[0].launch_arguments["enable_controller"] == "true"
    assert actions[0].launch_arguments["platform_config"] == "/private/wheel.yaml"
    assert actions[0].launch_arguments["control_backend"] == "path_mppi"


def test_legacy_controller_alias_and_optional_rviz(monkeypatch):
    _, actions, node, _ = compose(monkeypatch, start_controller="true", start_rviz="true")
    assert actions[0].launch_arguments["enable_controller"] == "true"
    assert len(actions) == 2 and isinstance(actions[1], node)
    assert actions[1].kwargs["package"] == "rviz2"
    assert Path(actions[1].kwargs["arguments"][1]).suffix == ".rviz"


def test_scripts_default_to_shared_entry_and_keep_legacy_paths():
    navigation = (ROOT / "scripts/orin/start_navigation.sh").read_text(encoding="utf-8")
    assert "wheel_navigation.launch.py" in navigation
    assert "enable_controller:=false" in navigation
    assert "incremental_v2" in navigation and "legged" in navigation
    hardware = (ROOT / "scripts/orin/start_hardware_navigation.sh").read_text(encoding="utf-8")
    assert "${ROS_DOMAIN_ID:-10}" in hardware
    assert "/home/yanfa/program" not in hardware and "P4/input/map_to_odom" not in hardware
    assert "start_navigation.sh" in hardware
    exploration = (ROOT / "scripts/orin/start_exploration.sh").read_text(encoding="utf-8")
    assert "wheel_navigation.launch.py" in exploration
    assert "ROS_SETUP" in exploration and "LUNAR_INSTALL_ROOT" in exploration


def test_installed_resources_cover_entry_and_profile():
    cmake = (ROOT / "ros2_ws/src/lunar_pure_exploration_ros/CMakeLists.txt").read_text(encoding="utf-8")
    for resource in ("wheel_navigation.launch.py", "wheel_navigation.yaml", "wheel_orin.yaml", "p4_navigation.rviz", "p4_topics.py"):
        assert resource in cmake


def test_adapter_arguments_expand_to_one_common_task_server(monkeypatch):
    _, actions, _, _ = compose(monkeypatch)
    forwarded = actions[0].launch_arguments
    monkeypatch.setattr(helper, "LAUNCH", ROOT / "launch/nav1cm_exploration.launch.py")
    shared, _, _ = helper._launch_module(monkeypatch)
    graph = shared._compose(helper._Context(forwarded))
    assert [node.kwargs["executable"] for node in graph] == [
        "lunar_exploration_map_node", "navigation_task_node", "incremental_exploration_node"]
    maps, task, frontier = [node.kwargs["parameters"][0] for node in graph]
    assert task["enable_controller"] is False
    assert task["control_backend"] == "mpcc_cmd_vel"
    assert task["wheel_config"] == maps["platform_config"] == frontier["platform_config"]
    assert frontier["navigation_action"] == task["action_name"]
    assert task["command_topic"] == "/Car/T5/Car_Cmd_Vel"
    assert not any("START" in str(node.kwargs) for node in graph)


def test_config_file_defaults_and_explicit_override_precedence(monkeypatch, tmp_path):
    config = tmp_path / "wheel-runtime.yaml"
    config.write_text("global_period_s: 20.0\ncontrol_backend: path_mppi\nenable_controller: true\n", encoding="utf-8")
    _, actions, _, _ = compose(monkeypatch, config_file=str(config), global_period_s="30.0")
    assert actions[0].launch_arguments["global_period_s"] == "30.0"
    assert actions[0].launch_arguments["control_backend"] == "path_mppi"
    assert actions[0].launch_arguments["enable_controller"] == "true"


def test_installed_share_resource_fallback(monkeypatch):
    module, _, _ = load(monkeypatch)
    monkeypatch.setattr(module, "__file__", "/installed/share/lunar_pure_exploration_ros/launch/wheel_navigation.launch.py")
    monkeypatch.setattr(module, "get_package_share_directory", lambda _: str(ROOT))
    assert module._resource("config/wheel_orin.yaml") == ROOT / "config/wheel_orin.yaml"


def test_third_script_argument_keeps_planner_ros_params_semantics(monkeypatch):
    script = (ROOT / "scripts/orin/start_navigation.sh").read_text(encoding="utf-8")
    assert 'planner_params_file:="${3:-}"' in script
    assert 'config_file:="${3:-}"' not in script
    _, actions, _, _ = compose(monkeypatch, planner_params_file="/outside/planner.yaml")
    forwarded = actions[0].launch_arguments
    assert forwarded["planner_params_file"] == "/outside/planner.yaml"
    monkeypatch.setattr(helper, "LAUNCH", ROOT / "launch/nav1cm_exploration.launch.py")
    shared, _, _ = helper._launch_module(monkeypatch)
    graph = shared._compose(helper._Context(forwarded))
    task = graph[1].kwargs
    assert task["parameters"][0] == "/outside/planner.yaml"
    assert isinstance(task["parameters"][1], dict)
    assert task["parameters"][1]["enable_controller"] is False
    assert len(graph[0].kwargs["parameters"]) == len(graph[2].kwargs["parameters"]) == 1


def test_rviz_uses_public_contract_goal_and_exploration_panel(monkeypatch, tmp_path):
    _, actions, _, _ = compose(monkeypatch, start_rviz="true", rviz_output_dir=str(tmp_path),
                               goal_topic="/private/goal", task_topic="/private/task", map_frame="world")
    rendered_path = Path(actions[1].kwargs["arguments"][1])
    assert rendered_path.parent == tmp_path
    view = yaml.safe_load(rendered_path.read_text(encoding="utf-8"))
    panel = next(p for p in view["Panels"] if p["Class"] == "lunar_navigation_rviz/ExplorationPanel")
    assert panel["Task Topic"] == "/private/task"
    tools = view["Visualization Manager"]["Tools"]
    assert next(t for t in tools if t["Class"] == "rviz_default_plugins/SetGoal")["Topic"] == "/private/goal"
    assert view["Visualization Manager"]["Global Options"]["Fixed Frame"] == "world"
    assert "/planning_demo" not in rendered_path.read_text(encoding="utf-8")


def test_default_composition_and_rviz_dependencies_have_no_reverse_cycle():
    import xml.etree.ElementTree as ET
    manifest = ET.parse(ROOT / "ros2_ws/src/lunar_pure_exploration_ros/package.xml").getroot()
    runtime = {element.text for element in manifest.findall("exec_depend")}
    assert {"lunar_nav1cm_planner", "lunar_incremental_navigation_ros", "lunar_navigation_rviz", "rviz2"} <= runtime
    plugin = ET.parse(ROOT / "ros2_ws/src/lunar_navigation_rviz/package.xml").getroot()
    assert "lunar_pure_exploration_ros" not in {element.text for element in plugin}


def test_runtime_build_explicitly_discovers_rviz_plugin():
    script = (ROOT / "scripts/orin/build.sh").read_text(encoding="utf-8")
    runtime = next(line for line in script.splitlines() if "runtime) packages=" in line)
    assert "lunar_navigation_rviz" in runtime
