"""Build entry must own every path before colcon writes an overlay."""
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[4]


def run_build(tmp_path, cache, *args):
    setup = tmp_path / 'setup.bash'
    setup.write_text('export ROS_DISTRO=jazzy\n')
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
    assert identity['ros_distro'] == 'jazzy'
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
    (cache / '.drl-source.json').write_text(json.dumps({'source_root': '/foreign', 'ros_distro': 'jazzy'}))
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
