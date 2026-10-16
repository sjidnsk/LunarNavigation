import json
import importlib.util
import os
from pathlib import Path
import subprocess
import sys

import pytest

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


def fake_runtime(tmp_path):
    setup=tmp_path/'setup.bash'
    setup.write_text('export ROS_DISTRO=humble\n')
    cache=tmp_path/'cache';(cache/'install').mkdir(parents=True)
    (cache/'install/local_setup.bash').write_text('export LAUNCHER_FAKE_OVERLAY=1\n')
    spec=importlib.util.spec_from_file_location('launcher_build_cache',ROOT/'scripts/drl/build_cache.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    (cache/'.drl-source.json').write_text(json.dumps(dict(source_root=str(ROOT),
        ros_distro='humble',native_source_sha256=module.native_digest(ROOT))))
    capture=tmp_path/'captured.json'
    python=tmp_path/'fake-python'
    child_code='import json,os,sys; print(json.dumps([sys.flags.optimize,os.environ.get("PYTHONOPTIMIZE")]))'
    python.write_text('#!'+sys.executable+'\n'+
        'import json,os,subprocess,sys\n'
        'from pathlib import Path\n'+
        'child=subprocess.check_output([sys.executable,"-c",'+repr(child_code)+'],text=True)\n'+
        'Path(os.environ["LAUNCHER_CAPTURE"]).write_text(json.dumps(dict('
        'optimize=sys.flags.optimize,env=os.environ.get("PYTHONOPTIMIZE"),'
        'child=json.loads(child),argv=sys.argv[1:],overlay=os.environ.get("LAUNCHER_FAKE_OVERLAY"))))\n')
    python.chmod(0o755)
    env={k:v for k,v in os.environ.items() if k!='PYTHONOPTIMIZE'}
    env.update(DRL_ROS_SETUP=str(setup),DRL_CACHE=str(cache),DRL_PYTHON=str(python),
        LAUNCHER_CAPTURE=str(capture))
    return env,capture


@pytest.mark.parametrize('requested',[None,'1'])
def test_humble_launcher_exports_optimization_to_python_children(tmp_path,requested):
    env,capture=fake_runtime(tmp_path)
    if requested is not None:env['PYTHONOPTIMIZE']=requested
    result=subprocess.run(['bash',str(ROOT/'scripts/drl/_run.sh'),'--help'],
        env=env,capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    record=json.loads(capture.read_text())
    assert record['optimize']==1
    assert record['env']=='1'
    assert record['child']==[1,'1']
    assert record['argv']==['-m','lunar_drl_exploration.cli','--help']
    assert record['overlay']=='1'


@pytest.mark.parametrize('requested',['0','2',''])
def test_humble_launcher_rejects_explicit_unsupported_optimization(tmp_path,requested):
    env,capture=fake_runtime(tmp_path);env['PYTHONOPTIMIZE']=requested
    result=subprocess.run(['bash',str(ROOT/'scripts/drl/_run.sh'),'--help'],
        env=env,capture_output=True,text=True)
    assert result.returncode==2
    assert 'requires PYTHONOPTIMIZE=1' in result.stderr
    assert 'Humble' in result.stderr
    assert not capture.exists()


def test_readme_documents_humble_launcher_and_owned_build_prerequisites():
    text=(ROOT/'ros2_ws/src/lunar_drl_exploration/README.md').read_text()
    section=text.split('## Async training and operator entrypoints',1)[1].split('`--max-transitions`',1)[0]
    assert '/opt/ros/humble/setup.bash' in section
    assert 'PYTHONOPTIMIZE=1' in section and 'PYTHONOPTIMIZE=0' in section
    assert 'scripts/drl/build.sh' in section
    assert 'export DRL_CACHE=' in section and 'export DRL_PYTHON=' in section
    assert 'config/drl_candidate_geometry_v3.yaml' in section
    assert '/jazzy' not in section
