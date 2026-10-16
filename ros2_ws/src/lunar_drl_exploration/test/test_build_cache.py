"""Build entry must own every path before colcon writes an overlay."""
import json
import os
from pathlib import Path
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[4]


def run_build(tmp_path, cache, *args):
    setup = tmp_path / 'setup.bash'
    setup.write_text('export ROS_DISTRO=humble\n')
    binary = tmp_path / 'bin'
    binary.mkdir(exist_ok=True)
    colcon = binary / 'colcon'
    colcon.write_text('#!/bin/sh\nmkdir -p "$DRL_CACHE"\nprintf called > "$DRL_CACHE/colcon-called"\n')
    colcon.chmod(0o755)
    env = dict(os.environ, DRL_CACHE=str(cache), DRL_ROS_SETUP=str(setup),
               PATH=str(binary) + ':' + os.environ['PATH'])
    return subprocess.run(['bash', str(ROOT / 'scripts/drl/build.sh'), *args],
                          env=env, capture_output=True, text=True)


def test_cold_cache_source_link_and_idempotence(tmp_path):
    cache = tmp_path / 'cache'
    result = run_build(tmp_path, cache, '--symlink-install')
    assert result.returncode == 0, result.stderr
    link = cache / 'build/lunar_drl_exploration/native'
    assert link.is_symlink()
    assert link.resolve() == ROOT / 'ros2_ws/src/lunar_drl_exploration/native'
    identity = json.loads((cache / '.drl-source.json').read_text())
    assert identity['source_root'] == str(ROOT)
    assert identity['ros_distro'] == 'humble'
    assert run_build(tmp_path, cache, '--symlink-install').returncode == 0


def test_unknown_nonempty_cache_is_preserved(tmp_path):
    cache = tmp_path / 'cache'
    cache.mkdir()
    (cache / 'unowned').write_text('keep')
    result = run_build(tmp_path, cache)
    assert result.returncode != 0
    assert not (cache / 'colcon-called').exists()
    assert (cache / 'unowned').read_text() == 'keep'


def test_foreign_source_marker_is_rejected(tmp_path):
    cache = tmp_path / 'cache'
    cache.mkdir()
    (cache / '.drl-source.json').write_text(json.dumps({'source_root': '/foreign', 'ros_distro': 'humble'}))
    result = run_build(tmp_path, cache)
    assert result.returncode != 0
    assert not (cache / 'colcon-called').exists()


def test_native_directory_or_foreign_link_is_preserved(tmp_path):
    for kind in ('directory', 'link', 'parent-link'):
        cache = tmp_path / kind
        assert run_build(tmp_path, cache).returncode == 0
        (cache / 'colcon-called').unlink()
        package = cache / 'build/lunar_drl_exploration'
        native = package / 'native'
        native.unlink(missing_ok=True)
        if kind == 'directory':
            native.mkdir()
        elif kind == 'link':
            native.symlink_to('/foreign')
        else:
            package.rmdir()
            package.symlink_to(tmp_path / 'outside')
        result = run_build(tmp_path, cache)
        assert result.returncode != 0
        assert not (cache / 'colcon-called').exists()
        assert native.is_dir() if kind == 'directory' else package.is_symlink() if kind == 'parent-link' else native.is_symlink()


def test_output_override_is_rejected_before_writing(tmp_path):
    for option in ('--build-base', '--install-base', '--log-base', '--base-paths', '--merge-install'):
        cache = tmp_path / option[2:]
        result = run_build(tmp_path, cache, option, '/foreign')
        assert result.returncode != 0
        assert not (cache / 'colcon-called').exists()


def parsed_colcon_overrides(tmp_path, arguments):
    """Installed parser only: never create a log or invoke a build verb."""
    script = '''
import json, sys
from pathlib import Path
from colcon_core.location import set_default_config_path
from colcon_core.command import (create_parser, get_verb_extensions,
    create_subparser, add_parsers_without_arguments, add_parser_arguments)
set_default_config_path(path=Path(sys.argv[2]))
parser=create_parser()
verbs=get_verb_extensions(group_name='colcon_core.verb')
subparser=create_subparser(parser,'colcon',verbs,attribute='verb_name')
add_parsers_without_arguments(parser,subparser,verbs,attribute='verb_name')
known,_=parser.parse_known_args(['build'])
add_parser_arguments(known.verb_parser,known.verb_extension)
args=parser.parse_args(['--log-base','/owned/log','build',
    '--base-paths','/owned/src','--packages-up-to','lunar_drl_exploration',
    '--build-base','/owned/build','--install-base','/owned/install',
    *json.loads(sys.argv[1])])
print(json.dumps({key:getattr(args,key) for key in
    ('build_base','install_base','base_paths','paths','packages_select_build_failed')}))
'''
    result = subprocess.run(['/usr/bin/python3', '-c', script, json.dumps(arguments),
                             str(tmp_path / 'parser-config')], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert not (tmp_path / 'parser-config').exists()
    return json.loads(result.stdout)


@pytest.mark.parametrize('option,field,value', [
    ('--build-bas', 'build_base', '/foreign/build'),
    ('--install-bas', 'install_base', '/foreign/install'),
    ('--base-pat', 'base_paths', ['/foreign/src']),
    ('--path', 'paths', ['/foreign/src']),
])
@pytest.mark.parametrize('equals', [False, True])
def test_real_parser_abbreviated_override_refused_by_entry(tmp_path, option, field, value, equals):
    text = value[0] if isinstance(value, list) else value
    arguments = [option + '=' + text] if equals else [option, text]
    assert parsed_colcon_overrides(tmp_path, arguments)[field] == value
    cache = tmp_path / 'cache'
    result = run_build(tmp_path, cache, *arguments)
    assert result.returncode != 0, 'installed colcon would override the owned path'
    assert 'override refused' in result.stderr
    assert not cache.exists()


def test_real_parser_abbreviated_package_selection_refused(tmp_path):
    arguments = ['--packages-select-build-faile']
    assert parsed_colcon_overrides(tmp_path, arguments)['packages_select_build_failed'] is True
    cache = tmp_path / 'cache'
    result = run_build(tmp_path, cache, *arguments)
    assert result.returncode != 0
    assert 'override refused' in result.stderr
    assert not cache.exists()


@pytest.mark.parametrize('option', [
    '--packages-up-to-rege=foreign', '--packages-skip=foreign',
    '--packages-ignore-rege=foreign', '--merge-ins', '--log-bas=/foreign/log',
])
def test_protected_package_and_output_prefixes_refused_before_cache(tmp_path, option):
    cache = tmp_path / 'cache'
    result = run_build(tmp_path, cache, option)
    assert result.returncode != 0
    assert 'override refused' in result.stderr
    assert not cache.exists()


def build_module():
    import importlib.util
    spec=importlib.util.spec_from_file_location('build_cache_regression',ROOT/'scripts/drl/build_cache.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('relative',[
    'lunar_drl_exploration/lunar_drl_exploration/model.py',
    'lunar_incremental_navigation_ros/src/node.cpp',
    'lunar_pure_wheeled_controller/python/lunar_pure_wheeled_controller/execution.py',
    'lunar_planning_msgs/msg/TrackingStatus.msg',
])
def test_cache_digest_covers_all_runtime_inputs(tmp_path,relative):
    source=tmp_path/'source';path=source/'ros2_ws/src'/relative
    path.parent.mkdir(parents=True);path.write_text('old\n')
    module=build_module();old=module.native_digest(source)
    path.write_text('new\n')
    assert module.native_digest(source)!=old


def test_failed_rebuild_invalidates_success_identity_and_can_retry(tmp_path):
    cache=tmp_path/'cache'
    assert run_build(tmp_path,cache).returncode==0
    colcon=tmp_path/'bin/colcon';colcon.write_text('#!/bin/sh\nexit 17\n');colcon.chmod(0o755)
    env=dict(os.environ,ROS_DISTRO='humble',PATH=str(colcon.parent)+':'+os.environ['PATH'])
    command=['python3',str(ROOT/'scripts/drl/build_cache.py'),str(ROOT),str(cache)]
    failed=subprocess.run(command,env=env,capture_output=True,text=True)
    assert failed.returncode!=0
    verify=subprocess.run(['python3',str(ROOT/'scripts/drl/build_cache.py'),'--verify',str(ROOT),str(cache)],env=env,capture_output=True,text=True)
    assert verify.returncode!=0, 'failed colcon must not leave a launchable success identity'
    assert run_build(tmp_path,cache).returncode==0
