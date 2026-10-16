import json
import importlib.util
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[4]


def test_humble_launcher_verifies_identity_before_overlay(tmp_path):
    setup = tmp_path/'setup.bash'
    setup.write_text('export ROS_DISTRO=humble\n')
    cache = tmp_path/'cache'
    (cache/'install').mkdir(parents=True)
    touched = tmp_path/'overlay-touched'
    (cache/'install/local_setup.bash').write_text(f'touch {touched}\n')
    env = dict(os.environ, DRL_ROS_SETUP=str(setup), DRL_CACHE=str(cache), DRL_PYTHON='/bin/true')
    result = subprocess.run(['bash', str(ROOT/'scripts/drl/_run.sh'), '--help'], env=env, capture_output=True)
    assert result.returncode != 0
    assert not touched.exists()
    spec = importlib.util.spec_from_file_location('build_cache', ROOT/'scripts/drl/build_cache.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    (cache/'.drl-source.json').write_text(json.dumps({'source_root':str(ROOT), 'ros_distro':'humble', 'native_source_sha256':module.native_digest(ROOT)}))
    result = subprocess.run(['bash', str(ROOT/'scripts/drl/_run.sh'), '--help'], env=env, capture_output=True)
    assert result.returncode == 0, result.stderr
    assert touched.exists()


def test_jazzy_launcher_refused(tmp_path):
    setup = tmp_path/'setup.bash'
    setup.write_text('export ROS_DISTRO=jazzy\n')
    env = dict(os.environ, DRL_ROS_SETUP=str(setup), DRL_CACHE=str(tmp_path/'cache'))
    result = subprocess.run(['bash', str(ROOT/'scripts/drl/_run.sh')], env=env, capture_output=True)
    assert result.returncode != 0
    assert b'ROS_DISTRO=humble' in result.stderr
