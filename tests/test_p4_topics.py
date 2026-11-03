"""正式话题唯一配置与RViz生成，不依赖ROS。"""
from copy import deepcopy
from pathlib import Path
import yaml
import pytest
from scripts.simulation.p4_topics import load_topic_contract, launch_topic_arguments, render_rviz_config
ROOT = Path(__file__).resolve().parents[1]

def test_contract_uses_car_t4_and_preserves_external_interfaces():
    contract = load_topic_contract(ROOT)
    topics = contract['launch']
    assert topics['goal_topic'] == '/Car/T4/navigation/goal'
    assert topics['command_topic'] == '/Car/T4/control/cmd_vel'
    assert topics['action_name'] == '/Car/T4/navigation/navigate_to_pose'
    assert topics['map_topic'] == '/Car/T3/mapping/grid_map'
    assert topics['odometry_topic'] == '/Car/T3/localization/odometry'
    assert topics['tf_topic'] == '/tf' and topics['tf_static_topic'] == '/tf_static'
    assert all(v.startswith('/Car/T4/') for k,v in topics.items() if k not in {'map_topic','odometry_topic','tf_topic','tf_static_topic'})
    assert contract['vehicle'] == dict(command_topic='/car/cmd_vel',mode_topic='/car/set_mode',lease_topic='/car/external_control_lease',telemetry_topic='/car/telemetry',connected_topic='/car/ue_connected')

def test_namespace_override_changes_only_p4_and_keeps_semantic_hierarchy():
    default = launch_topic_arguments(ROOT)
    isolated = launch_topic_arguments(ROOT, namespace='/bounded')
    for key, value in default.items():
        expected = '/bounded'+value[len('/Car/T4'):] if value.startswith('/Car/T4/') else value
        assert isolated[key] == expected
    assert isolated['command_topic'] == '/bounded/control/cmd_vel'
    assert isolated['action_name'] == '/bounded/navigation/navigate_to_pose'

@pytest.mark.parametrize('namespace',['relative','/','/with space','/Car/T4/'])
def test_rejects_ambiguous_namespace(namespace):
    with pytest.raises(ValueError): launch_topic_arguments(ROOT,namespace=namespace)

def test_rviz_renderer_uses_selected_configuration_without_mutating_template():
    contract = load_topic_contract(ROOT)
    contract['launch']['goal_topic'] = '/private/navigation/goal'
    contract['visualization']['boundary_draft'] = '/private/exploration/draft'
    template = {'Panels':[{'Class':'lunar_navigation_rviz/ExplorationPanel','Task Topic Key':'task_topic','Status Topic Key':'status_topic','Boundary Topic Key':'boundary_draft'}],
      'Visualization Manager':{'Displays':[{'Class':'rviz_default_plugins/Path','Topic Key':'global_reference_topic','Topic':{'Reliability Policy':'Reliable'}}],
      'Tools':[{'Class':'rviz_default_plugins/SetGoal','Topic Key':'goal_topic'}]}}
    before = deepcopy(template)
    actual = render_rviz_config(template,contract)
    assert template == before
    panel = actual['Panels'][0]
    assert panel['Task Topic'] == contract['launch']['task_topic']
    assert panel['Boundary Topic'] == '/private/exploration/draft'
    assert actual['Visualization Manager']['Tools'][0]['Topic'] == '/private/navigation/goal'
    display = actual['Visualization Manager']['Displays'][0]
    assert display['Topic']['Value'] == contract['launch']['global_route_topic']+'_reference'
    assert display['Topic']['Reliability Policy'] == 'Reliable'
    assert 'Topic Key' not in display

def test_rviz_renderer_refuses_missing_contract_key():
    with pytest.raises(KeyError):render_rviz_config({'Topic Key':'typo'},load_topic_contract(ROOT))

def test_explicit_source_controls_contract(tmp_path):
    contract = load_topic_contract(ROOT)
    contract['launch']['goal_topic'] = '/Car/T4/navigation/changed'
    (tmp_path/'config').mkdir()
    (tmp_path/'config/p4_topics.yaml').write_text(yaml.safe_dump(contract),encoding='utf-8')
    assert load_topic_contract(tmp_path)['launch']['goal_topic'].endswith('/changed')
    assert launch_topic_arguments(tmp_path)['goal_topic'].endswith('/changed')
    assert load_topic_contract(ROOT)['launch']['goal_topic'].endswith('/goal')


def test_complete_rviz_template_is_resolved_from_contract():
    contract = load_topic_contract(ROOT)
    template = yaml.safe_load((ROOT/'scripts/simulation/p4_navigation.rviz').read_text(encoding='utf-8'))
    actual = render_rviz_config(template, contract)
    text = yaml.safe_dump(actual)
    assert 'Topic Key' not in text
    assert '/p4_p3_debug' not in text and '/goal_pose' not in text and '/lunar_rviz/' not in text
    assert actual['Panels'][1]['Task Topic'] == contract['launch']['task_topic']
    for display in actual['Visualization Manager']['Displays']:
        if 'Topic' in display:
            known = set(contract['launch'].values()) | set(contract['visualization'].values())
            known.add(contract['launch']['global_route_topic']+'_reference')
            assert display['Topic']['Value'] in known
    goal_tool = next(tool for tool in actual['Visualization Manager']['Tools'] if tool['Class'].endswith('/SetGoal'))
    assert goal_tool['Topic'] == contract['launch']['goal_topic']


def test_stereo_images_use_shared_p3_topics_and_small_queues():
    contract = load_topic_contract(ROOT)
    template = yaml.safe_load((ROOT/'scripts/simulation/p4_navigation.rviz').read_text(encoding='utf-8'))
    contract['visualization']['p3_left_image'] = '/test/left'
    contract['visualization']['p3_right_image'] = '/test/right'
    actual = render_rviz_config(template, contract)
    images = [d for d in actual['Visualization Manager']['Displays'] if d['Class']=='rviz_default_plugins/Image']
    assert [d['Topic']['Value'] for d in images] == ['/test/left','/test/right']
    assert all(d['Enabled'] and d['Topic']['Depth']==1 and d['Topic']['Durability Policy']=='Volatile' for d in images)
    assert next(d for d in actual['Visualization Manager']['Displays'] if d['Class']=='rviz_default_plugins/Odometry')['Keep']==1


def test_exploration_task_map_is_opt_in_in_template_and_rendered_config():
    template = yaml.safe_load((ROOT/'scripts/simulation/p4_navigation.rviz').read_text(encoding='utf-8'))
    for document in (template, render_rviz_config(template, load_topic_contract(ROOT))):
        task_map = next(d for d in document['Visualization Manager']['Displays'] if d['Name']=='Exploration task map')
        assert task_map['Enabled'] is False


def test_duplicate_pose_goal_is_disabled_while_deletable_candidate_markers_remain_enabled():
    template = yaml.safe_load((ROOT/'scripts/simulation/p4_navigation.rviz').read_text(encoding='utf-8'))
    actual = render_rviz_config(template, load_topic_contract(ROOT))
    displays = {display['Name']: display for display in actual['Visualization Manager']['Displays']}
    assert displays['Current exploration goal']['Enabled'] is False
    assert displays['Exploration frontiers']['Enabled'] is True
    assert displays['Exploration frontiers']['Class'] == 'rviz_default_plugins/MarkerArray'
    assert displays['Current exploration goal']['Topic']['Value'] == load_topic_contract(ROOT)['launch']['current_goal_topic']
