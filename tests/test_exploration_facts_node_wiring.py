"""静态接线合同；不替代真实ROS订阅/服务运行验证。"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'ros2_ws/src/lunar_incremental_navigation_ros/src/exploration_map_node.cpp'


def test_shared_mode_subscribes_facts_instead_of_raw_p3():
    text = SOURCE.read_text(encoding='utf-8')
    assert 'text("base_map_facts_topic", "")' in text
    branch = text.split('if (!base_map_facts_topic.empty()) {', 1)[1]
    shared, raw = branch.split('} else {', 1)
    assert 'create_subscription<std_msgs::msg::String>' in shared
    assert '.reliable().transient_local()' in shared
    assert 'create_subscription<grid_map_msgs::msg::GridMap>' not in shared
    assert 'create_subscription<grid_map_msgs::msg::GridMap>' in raw
    callback = shared.split('[this]', 1)[1]
    assert 'std::lock_guard lock(mutex)' in callback
    assert 'latest_facts = std::move(message)' in callback
    assert 'Wake()' in callback
    assert 'ParseExplorationFacts' not in callback
    assert 'ApplyFacts' not in callback


def test_worker_consumes_latest_once_and_uses_fact_derivation_only():
    text = SOURCE.read_text(encoding='utf-8')
    worker = text.split('void Run() {', 1)[1]
    assert 'facts_message = std::exchange(latest_facts, {})' in worker
    fact_branch = worker.split('if (facts_message) {', 1)[1].split('const bool transform_changed', 1)[0]
    assert 'ParseExplorationFacts(facts_message->data)' in fact_branch
    assert 'facts.frame != map_frame' in fact_branch
    assert 'ApplyFacts(facts, stamp)' in fact_branch
    assert 'AdaptExplorationElevation' not in fact_branch
    assert 'MakePlatformWindowElevationEvaluator' not in worker
    assert 'if (base_map_facts_topic.empty() && input.local_map' in worker
    assert 'create_wall_timer' not in text


def test_empty_facts_revoke_service_and_clear_previous_occupancy_geometry():
    text = SOURCE.read_text(encoding='utf-8')
    empty = text.split('if (!capture.map.fine) {', 1)[1].split('capture.map.local_window_size_m', 1)[0]
    assert 'BuildExplorationMapPublication' in empty
    assert 'std::atomic_store(&published' in empty
    assert 'auto cleared = *last_published_grid' in empty
    assert 'std::fill(cleared.data.begin(), cleared.data.end(), -1)' in empty
    assert 'publisher->publish(cleared)' in empty
    assert 'last_published_grid = std::move(cleared)' in empty
    assert 'base_map_facts_topic.empty() ? epoch : map->SourceEpoch()' in text
    assert 'previous->response.epoch == capture.map.epoch' in text
