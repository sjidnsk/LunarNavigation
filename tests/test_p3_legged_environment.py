"""Every deployed P4 entrypoint must discover the same foot P3 domain."""
import json
import os
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
CASES = [
    ('start_hardware_navigation.sh', []),
    ('start_static_tf_input.sh', []),
    ('start_navigation.sh', []),
    ('start_exploration.sh', []),
    ('start_legged_controller.sh', []),
    ('publish_navigation_goal.sh', ['2', '1']),
    ('publish_exploration_task.sh', ['probe', '-5', '-5', '5', '5']),
]

@pytest.mark.parametrize('script,args', CASES)
@pytest.mark.parametrize('override,domain,uri', [
    ({}, '69', None),
    ({'P3_DOG_DOMAIN': '71'}, '71', None),
    ({'P3_DOG_DOMAIN': '71', 'ROS_DOMAIN_ID': '72', 'CYCLONEDDS_URI': '<CycloneDDS/>'}, '72', '<CycloneDDS/>'),
])
def test_entrypoint_environment(tmp_path, script, args, override, domain, uri):
    fake = tmp_path / 'bin'
    fake.mkdir()
    for name in ['ros2', 'flock']:
        path = fake / name
        path.write_text('#!/usr/bin/python3\nimport json,os\nprint(json.dumps({k:os.environ.get(k) for k in ["ROS_DOMAIN_ID","ROS_LOCALHOST_ONLY","RMW_IMPLEMENTATION","CYCLONEDDS_URI"]}))\n')
        path.chmod(0o755)
    setup = tmp_path / 'bash_env'
    setup.write_text('source() { case "$1" in /opt/ros/*|*/ros2_ws/install/*) return 0 ;; *) builtin source "$@" ;; esac; }\n')
    env = dict(os.environ)
    for key in ['ROS_DOMAIN_ID','P3_DOG_DOMAIN','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','CYCLONEDDS_URI']:
        env.pop(key, None)
    env.update(override, BASH_ENV=str(setup), PATH=str(fake)+':'+env['PATH'], P4_RUNTIME_DIR=str(tmp_path/'runtime'))
    result = subprocess.run(['bash', str(ROOT/'scripts/orin'/script), *args], env=env, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    values = json.loads(result.stdout)
    assert values == dict(ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='0', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp', CYCLONEDDS_URI=uri or 'file://'+str(ROOT/'config/cyclonedds_p3.xml'))
