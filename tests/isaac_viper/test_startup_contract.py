"""启动必须先核对只读 USD；纯边界测试不初始化 Kit/ROS，PXR 用冻结场景。"""
import hashlib
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import yaml

from tools.isaac_viper.config import load_profile
from tools.isaac_viper import run_scene, ros_bridge  # 预加载numpy，避免sys.modules隔离重载扩展

REPO = Path(__file__).parents[2]


def profile_file(tmp_path, change=None):
    """真正经 load_profile 的 YAML；变更不是绕过配置检查的 semantic dict。"""
    raw = yaml.safe_load((REPO/'config/isaac_viper.yaml').read_text(encoding='utf-8'))
    cap = yaml.safe_load((REPO/'config/wheel_orin.yaml').read_text(encoding='utf-8'))
    if change:
        change(raw, cap)
    raw['sources'] = {'capability': 'capability.yaml',
                      'exploration': str(REPO/'config/exploration_navigation.yaml')}
    (tmp_path/'capability.yaml').write_text(yaml.safe_dump(cap), encoding='utf-8')
    path = tmp_path/'profile.yaml'
    path.write_text(yaml.safe_dump(raw), encoding='utf-8')
    return path


@pytest.mark.parametrize('validation', [
    {'passed': False, 'errors': ['left_camera: focal_length_mm']},
    ValueError('unreadable USD'),
])
def test_main_rejects_before_physics_or_ros_and_preserves_diagnostic(tmp_path, validation):
    """缺失 gate 会越过不一致/读取异常进入物理/ROS准备；诊断必须先落盘。"""
    stage = tmp_path/'frozen.usda'
    stage.write_text('#usda 1.0', encoding='utf-8')
    output = tmp_path/'run'
    def validate(*args):
        if isinstance(validation, Exception):
            raise validation
        return validation
    fake_validator = SimpleNamespace(validate_stage=validate)
    # 真实SDK依赖不可本地初始化；仅替代App bootstrap，物理/ROS后续入口不得调用。
    def app_stub(*args, **kwargs):
        return SimpleNamespace(close=lambda **kwargs: None)
    def forbidden_object(*args, **kwargs):
        raise AssertionError('physics/ROS object created before contract rejection')
    with patch.dict(sys.modules, {'tools.isaac_viper.validate_stage': fake_validator,
                                  'isaacsim': SimpleNamespace(SimulationApp=app_stub),
                                  'isaacsim.core.experimental.utils.app': SimpleNamespace(enable_extension=forbidden_object)}), \
         patch.object(ros_bridge, 'RosBridge', forbidden_object), \
         patch.dict(os.environ, ROS_DOMAIN_ID='117', ROS_LOCALHOST_ONLY='1', ROS_DISTRO='humble'), \
         patch.object(sys, 'argv', ['run_scene', '--stage', str(stage), '--profile',
                      str(REPO/'config/isaac_viper.yaml'), '--output-dir', str(output)]):
        with pytest.raises(RuntimeError, match='stage/profile contract'):
            run_scene.main()
    report = json.loads((output/'stage-profile-contract.json').read_text(encoding='utf-8'))
    assert report['passed'] is False
    assert report['errors']
    assert report['stage_sha256'] == hashlib.sha256(stage.read_bytes()).hexdigest()
    assert json.loads((output/'status.json').read_text(encoding='utf-8'))['state'] == 'failed'


@pytest.mark.parametrize('section,key,value', [
    ('imu', 'frequency_hz', 50), ('stereo', 'frequency_hz', 30),
    ('lidar', 'frequency_hz', 30), ('imu', 'implementation', 'other'),
    ('lidar', 'implementation', 'other'),
])
def test_runtime_rejects_configuration_it_cannot_apply(tmp_path, section, key, value):
    path = profile_file(tmp_path, lambda p, c: p['sensors'][section].update({key: value}))
    profile = load_profile(path)  # 合法配置也不能越过实际执行器能力限制。
    fake = SimpleNamespace(validate_stage=lambda *a: {'passed': True, 'errors': []})
    with patch.dict(sys.modules, {'tools.isaac_viper.validate_stage': fake}):
        with pytest.raises(RuntimeError, match='stage/profile contract'):
            run_scene.require_stage_profile(path, profile, tmp_path/'contract.json')
    assert section in ' '.join(json.loads((tmp_path/'contract.json').read_text())['errors'])
