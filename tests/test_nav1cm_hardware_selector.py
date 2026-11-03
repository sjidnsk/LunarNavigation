"""硬件wrapper仅测试shell选择，所有ROS命令由临时fake拦截。"""
import json,os,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def launch_args(tmp_path,mode=None):
    fake=tmp_path/'bin';fake.mkdir()
    log=tmp_path/'argv.json'
    ros=fake/'ros2';ros.write_text('#!/usr/bin/env python3\nimport json,os,sys\nopen(os.environ["CAPTURE"],"w").write(json.dumps(sys.argv[1:]))\n');ros.chmod(0o755)
    setup=tmp_path/'bash-env';setup.write_text('source() { :; }\n')
    env=dict(os.environ,PATH=str(fake)+os.pathsep+os.environ['PATH'],BASH_ENV=str(setup),CAPTURE=str(log),
             P4_RUNTIME_DIR=str(tmp_path/'runtime'),CYCLONEDDS_URI='<CycloneDDS/>')
    env.pop('LUNAR_NAVIGATION_MODE',None)
    args=['bash',str(ROOT/'scripts/orin/start_hardware_navigation.sh')]+([] if mode is None else [mode])
    result=subprocess.run(args,env=env,capture_output=True,text=True,timeout=10)
    assert result.returncode==0,result.stderr
    return json.loads(log.read_text())


def test_hardware_default_uses_shared_navigation_entry(tmp_path):
    args=launch_args(tmp_path)
    assert args[:3]==['launch','lunar_pure_exploration_ros','wheel_navigation.launch.py']
    assert 'enable_controller:=false' in args
    assert 'planner_params_file:=' in args
    assert not any('exploration_navigation' in item for item in args)


def test_explicit_hardware_legacy_retains_old_config(tmp_path):
    args=launch_args(tmp_path,'incremental_v2')
    assert args[:3]==['launch','lunar_pure_exploration_ros','exploration_navigation.launch.py']
    assert 'start_exploration:=false' in args and 'start_navigation:=true' in args
    assert any(item.endswith('config/orin_hardware.yaml') for item in args)
