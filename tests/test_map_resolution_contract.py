"""策略地图默认跟随源图，并允许模型固定物理输入尺度。"""
import importlib.util
from pathlib import Path
import pytest

PATH = Path(__file__).resolve().parents[1] / "ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/navigation_backend.py"

def api():
    spec = importlib.util.spec_from_file_location("map_backend_contract", PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def spec_values(**kwargs):
    specs = api().owned_nav1cm_specs("/private", platform_config="/wheel.yaml",
        build_dir="/cache", use_sim_time=False, **kwargs)
    return specs

def test_policy_default_follows_source_without_changing_topics():
    specs = spec_values()
    mapping = specs[0][2]
    assert mapping["policy_resolution_m"] == 0.0
    assert mapping["coarse_resolution_m"] == 0.0
    assert mapping["policy_map_service"] == "/private/get_map"
    assert specs[1][2]["action_name"] == "/private/navigate"

@pytest.mark.parametrize("resolution", [.05, .2, .4])
def test_policy_can_keep_explicit_actor_resolution(resolution):
    specs = spec_values(map_resolution_m=resolution)
    mapping = specs[0][2]
    assert mapping["policy_resolution_m"] == resolution
    assert mapping["coarse_resolution_m"] == 0.0
    assert "policy_resolution_m" not in specs[1][2]
    assert "policy_resolution_m" not in specs[2][2]

@pytest.mark.parametrize("resolution", [-.05, float("nan"), float("inf")])
def test_policy_resolution_rejects_nonrepresentable_values(resolution):
    with pytest.raises(ValueError, match="resolution"):
        spec_values(map_resolution_m=resolution)
def test_owned_navigation_and_map_view_share_one_facts_topic():
    specs = spec_values()
    assert specs[0][2]["base_map_facts_topic"] == "/private/base_facts"
    assert specs[1][2]["base_map_facts_topic"] == specs[0][2]["base_map_facts_topic"]
    assert "base_map_facts_topic" not in specs[2][2]
