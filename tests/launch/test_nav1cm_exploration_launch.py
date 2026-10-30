"""新入口真实组合函数的静态图合同；不声称 ROS 运行通过。"""
import importlib.util
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[2]
_spec=importlib.util.spec_from_file_location("legacy_launch_tests", ROOT/"tests/launch/test_exploration_navigation_launch.py")
helper=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(helper)

def load(monkeypatch):
    monkeypatch.setattr(helper,"LAUNCH",ROOT/"launch/nav1cm_exploration.launch.py")
    assert helper.LAUNCH.exists(), "显式 nav1cm 探索组合尚未实现"
    return helper._launch_module(monkeypatch)[0]

def graph(monkeypatch,**values):
    module=load(monkeypatch)
    return module._compose(helper._Context({"build_dir":"/outside/task3/cache",**values}))

def nodes(actions):
    return [a.kwargs for a in actions]

def test_default_no_goal_source_and_no_controller(monkeypatch):
    g=nodes(graph(monkeypatch))
    assert [(n["package"],n["executable"]) for n in g]==[
        ("lunar_incremental_navigation_ros","lunar_exploration_map_node"),
        ("lunar_nav1cm_planner","navigation_task_node")]

def test_frontier_singular_common_action_and_shared_source(monkeypatch):
    g=nodes(graph(monkeypatch,exploration_strategy="frontier",enable_controller="true"))
    assert [n["executable"] for n in g]==["lunar_exploration_map_node","navigation_task_node","incremental_exploration_node","lunar_pure_wheeled_controller_node.py"]
    maps,planner,frontier,follower=[n["parameters"][0] for n in g]
    assert planner["action_name"]==frontier["navigation_action"]
    assert planner["map_topic"]==maps["map_topic"]==follower["p3_map_topic"]
    assert planner["wheel_config"]==maps["platform_config"]==follower["platform_config"]
    assert maps["map_qos_reliability"]==planner["map_qos_reliability"]=="reliable"
    assert maps["map_qos_durability"]==planner["map_qos_durability"]==follower["map_qos_durability"]=="volatile"
    assert follower["map_qos_reliability"]=="reliable"
    assert planner["allow_unknown"] is False
    assert follower["mppi_nominal_response_time_s"]==.15
    assert follower["input_mode"]=="incremental_path"
    assert follower["controller_backend"]=="mppi_path_follower"
    assert follower["standard_map_source"]=="direct_p3"
    assert planner["local_path_topic"]==follower["path_topic"]
    assert "path_reference_topic" not in follower

def test_drl_optional_hook_requires_actor_and_is_singular(monkeypatch):
    with pytest.raises(RuntimeError,match="drl_actor"):
        graph(monkeypatch,exploration_strategy="drl")
    g=nodes(graph(monkeypatch,exploration_strategy="drl",drl_actor="/outside/actor.pt"))
    assert [n["executable"] for n in g]==["lunar_exploration_map_node","navigation_task_node","lunar_drl_inference_node"]
    hook=g[-1]
    assert hook["package"]=="lunar_drl_exploration"
    p=hook["parameters"][0]
    assert p["navigation_backend"]=="nav1cm" and p["actor_path"]=="/outside/actor.pt"
    assert {"task_topic","status_topic","action_name","policy_map_service","odometry_topic","tf_topic","tf_static_topic","global_route_topic","map_frame","odom_frame"}<=p.keys()

@pytest.mark.parametrize("values",[{"exploration_strategy":"invalid"},{"enable_controller":"perhaps"},{"build_dir":""}])
def test_invalid_composition_explains_input(monkeypatch,values):
    with pytest.raises(RuntimeError): graph(monkeypatch,**values)


def test_frontier_uses_shared_static_tf_topic(monkeypatch):
    g=nodes(graph(monkeypatch,exploration_strategy="frontier",tf_static_topic="/private/latched_static"))
    assert all(n["parameters"][0]["tf_static_topic"]=="/private/latched_static" for n in g)


def test_execution_contract_topics_only_reach_actual_execution_participants(monkeypatch):
    g=nodes(graph(monkeypatch,exploration_strategy='frontier',enable_controller='true',
        execution_reference_topic='/private/execution',tracking_status_topic='/private/tracking'))
    maps,planner,frontier,follower=[n['parameters'][0] for n in g]
    for participant in (planner,follower):
        assert participant['execution_contract_enabled'] is True
        assert participant['execution_reference_topic']=='/private/execution'
        assert participant['tracking_status_topic']=='/private/tracking'
    for unrelated in (maps,frontier):
        assert 'execution_reference_topic' not in unrelated
        assert 'tracking_status_topic' not in unrelated


@pytest.mark.parametrize('timeout',['0.0','3.0'])
def test_exploration_passes_map_age_policy_only_to_follower(monkeypatch,timeout):
    g=nodes(graph(monkeypatch,enable_controller='true',p3_map_timeout_s=timeout))
    assert g[-1]['parameters'][0]['p3_map_timeout_s']==float(timeout)
    assert all('p3_map_timeout_s' not in n['parameters'][0] for n in g[:-1])


@pytest.mark.parametrize('timeout',['-1.0','nan','inf'])
def test_exploration_rejects_invalid_map_age_policy(monkeypatch,timeout):
    with pytest.raises(ValueError):graph(monkeypatch,enable_controller='true',p3_map_timeout_s=timeout)


@pytest.mark.parametrize('budget',[.05,.2])
def test_publication_envelope_reaches_only_controller(monkeypatch,budget):
    g=nodes(graph(monkeypatch,enable_controller='true',mppi_compute_envelope_s=str(budget)))
    assert g[-1]['parameters'][0]['mppi_compute_envelope_s']==budget
    assert all('mppi_compute_envelope_s' not in n['parameters'][0] for n in g[:-1])


@pytest.mark.parametrize('budget',['0','-.1','nan','inf'])
def test_invalid_publication_envelope_is_rejected(monkeypatch,budget):
    with pytest.raises(ValueError):graph(monkeypatch,enable_controller='true',mppi_compute_envelope_s=budget)
