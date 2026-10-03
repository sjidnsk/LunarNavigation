"""Own one external Jazzy cache, then keep its writer lock through colcon."""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys


def owned_directory(path):
    path = Path(os.path.abspath(path))
    for component in [*reversed(path.parents), path]:
        if component.is_symlink():
            raise ValueError('cache directory cannot traverse symlink: ' + str(component))
        if component.exists() and not component.is_dir():
            raise ValueError('cache parent is not a directory: ' + str(component))
    path.mkdir(parents=True, exist_ok=True)
    return path


def main():
    source = Path(sys.argv[1]).resolve(strict=True)
    cache_arg = sys.argv[2]
    args = sys.argv[3:]
    forbidden = ('--build-base', '--install-base', '--log-base', '--base-paths',
                 '--merge-install', '--packages-ignore', '--packages-select', '--packages-up-to')
    if any(arg.split('=', 1)[0] in forbidden for arg in args):
        raise ValueError('output/source/package override refused; choose a new DRL_CACHE')
    cache = owned_directory(cache_arg)
    fd = os.open(cache / '.drl-build.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    marker = cache / '.drl-source.json'
    identity = dict(source_root=str(source), ros_distro='jazzy')
    if marker.is_symlink():
        raise ValueError('source marker cannot be a symlink')
    if marker.exists():
        previous = json.loads(marker.read_text(encoding='utf-8'))
        if any(previous.get(key) != value for key, value in identity.items()):
            raise ValueError('foreign cache identity; preserve it and choose a new DRL_CACHE')
    elif any(path.name != '.drl-build.lock' for path in cache.iterdir()):
        raise ValueError('unmarked nonempty cache; preserve it and choose a new DRL_CACHE')
    native_source = source / 'ros2_ws/src/lunar_drl_exploration/native'
    for name in ('graph.cpp', 'terrain.cpp', 'visibility.cpp', 'grid_buffer.hpp'):
        if not (native_source / name).is_file():
            raise ValueError('native source missing: ' + name)
    package = owned_directory(cache / 'build/lunar_drl_exploration')
    native = package / 'native'
    if native.is_symlink():
        if native.resolve(strict=True) != native_source:
            raise ValueError('foreign native link; choose a new DRL_CACHE')
    elif native.exists():
        raise ValueError('native path is not the owned source symlink')
    else:
        native.symlink_to(native_source, target_is_directory=True)
    for name in ('install', 'log'):
        owned_directory(cache / name)
    identity['source_commit'] = subprocess.check_output(
        ['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    temporary = cache / '.drl-source.tmp'
    with temporary.open('x', encoding='utf-8') as stream:
        json.dump(identity, stream, ensure_ascii=False, sort_keys=True)
        stream.write('\n')
    temporary.replace(marker)
    os.set_inheritable(fd, True)
    os.execvp('colcon', ['colcon', '--log-base', str(cache / 'log'), 'build',
        '--base-paths', str(source / 'ros2_ws/src'), '--packages-up-to', 'lunar_drl_exploration',
        '--executor', 'sequential', '--build-base', str(cache / 'build'),
        '--install-base', str(cache / 'install'), *args])


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError) as exc:
        raise SystemExit('DRL build refused: ' + str(exc))
