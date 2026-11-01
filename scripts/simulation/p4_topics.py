"""P4接口命名、隔离覆盖与RViz模板实例化；不涉及控制决策。"""
from copy import deepcopy
from pathlib import Path
import re
import yaml

def load_topic_contract(source=None):
    source = Path(source) if source is not None else Path(__file__).resolve().parents[2]
    contract = yaml.safe_load((source / 'config/p4_topics.yaml').read_text(encoding='utf-8'))
    for group in ('launch', 'visualization', 'vehicle'):
        if not isinstance(contract.get(group), dict) or not all(
                isinstance(value, str) and value.startswith('/') for value in contract[group].values()):
            raise ValueError('invalid P4 topic contract group: ' + group)
    return contract

def launch_topic_arguments(source=None, *, namespace=None, contract=None):
    topics = dict((load_topic_contract(source) if contract is None else contract)['launch'])
    if namespace is None:
        return topics
    if not re.fullmatch(r'(?:/[A-Za-z_][A-Za-z0-9_]*)+', namespace):
        raise ValueError('namespace must be an absolute ROS namespace without trailing slash')
    return {key: namespace + value[len('/Car/T4'):] if value.startswith('/Car/T4/') else value
            for key, value in topics.items()}

def render_rviz_config(template, contract):
    """只把模板中的名称键替换为选定契约，保留样式、QoS和调用者原值。"""
    topics = {**contract['launch'], **contract['visualization']}
    topics['global_reference_topic'] = topics['global_route_topic'] + '_reference'
    def render(value):
        if isinstance(value, list):
            return [render(item) for item in value]
        if not isinstance(value, dict):
            return value
        result = {key: render(item) for key, item in value.items()
                  if key != 'Topic Key' and not key.endswith(' Topic Key')}
        if 'Topic Key' in value:
            topic = topics[value['Topic Key']]
            if value.get('Class') == 'rviz_default_plugins/SetGoal':
                result['Topic'] = topic
            else:
                result['Topic'] = {**result.get('Topic', {}), 'Value': topic}
        for key, topic_key in value.items():
            if key.endswith(' Topic Key'):
                result[key[:-4]] = topics[topic_key]
        return result
    return render(deepcopy(template))
