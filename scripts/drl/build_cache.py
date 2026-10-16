"""Own one external Humble cache, then keep its writer lock through colcon."""
import fcntl
import hashlib
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


def native_digest(source):
    """Fingerprint installed Python, native, ROS interfaces and capabilities.

    Keep the historical function/field name for callers, but cover the whole
    owned runtime dependency set so a Python-only edit invalidates the cache.
    """
    digest = hashlib.sha256()
    for package in ("lunar_drl_exploration", "lunar_incremental_navigation_core",
                    "lunar_incremental_navigation_ros", "lunar_pure_wheeled_controller",
                    "lunar_planning_msgs", "lunar_pure_exploration_msgs"):
        root = source / "ros2_ws/src" / package
        for path in sorted(root.rglob("*")):
            relative=path.relative_to(root)
            if any(part in ('test','tests','benchmark','__pycache__') for part in relative.parts):
                continue
            if path.is_file() and (path.suffix in (".py", ".cpp", ".hpp", ".h", ".xml", ".yaml", ".yml", ".msg", ".srv", ".action")
                    or path.name in ("setup.cfg", "CMakeLists.txt") or 'resource' in relative.parts):
                digest.update(str(path.relative_to(source)).encode())
                digest.update(path.read_bytes())
    return digest.hexdigest()


def main():
    verify = sys.argv[1] == "--verify"
    argv = sys.argv[2:] if verify else sys.argv[1:]
    source = Path(argv[0]).resolve(strict=True)
    cache_arg = argv[1]
    args = argv[2:]
    protected = ('--build-base', '--install-base', '--log-base', '--base-paths',
                 '--paths', '--merge-install', '--packages-')
    # argparse accepts unambiguous long-option abbreviations, including =value.
    # Reject even ambiguous protected prefixes before creating any cache path;
    # every package-selection extension also changes the owned dependency set.
    options = [arg.split('=', 1)[0] for arg in args if arg.startswith('--') and len(arg) > 2]
    if any(option.startswith('--packages-') or
           any(name.startswith(option) for name in protected) for option in options):
        raise ValueError('output/source/package override refused; choose a new DRL_CACHE')
    if os.environ.get("ROS_DISTRO") != "humble":
        raise ValueError("requires ROS_DISTRO=humble")
    candidate = Path(cache_arg).resolve()
    if candidate == source or source in candidate.parents:
        raise ValueError("cache must be outside source worktree")
    if verify:
        lock_path = Path(cache_arg) / ".drl-build.lock"
        if lock_path.exists():
            verify_fd = os.open(lock_path, os.O_RDONLY | os.O_NOFOLLOW)
            try:
                fcntl.flock(verify_fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
            finally:
                os.close(verify_fd)
        marker = Path(cache_arg) / ".drl-source.json"
        expected = dict(source_root=str(source), ros_distro="humble")
        if marker.is_symlink() or not marker.is_file():
            raise ValueError("missing or unsafe source marker")
        previous = json.loads(marker.read_text(encoding="utf-8"))
        if any(previous.get(k) != v for k, v in expected.items()):
            raise ValueError("foreign cache identity")
        if previous.get("native_source_sha256") != native_digest(source):
            raise ValueError("runtime source changed; rebuild the owned cache")
        return
    cache = owned_directory(cache_arg)
    fd = os.open(cache / '.drl-build.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    marker = cache / '.drl-source.json'
    identity = dict(source_root=str(source), ros_distro='humble')
    if marker.is_symlink():
        raise ValueError('source marker cannot be a symlink')
    owner = cache / '.drl-owner.json'
    if owner.is_symlink():
        raise ValueError('cache owner cannot be a symlink')
    ownership = owner if owner.exists() else marker
    if ownership.exists():
        previous = json.loads(ownership.read_text(encoding='utf-8'))
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
    # Ownership survives failure; only a successful build may publish a launch
    # identity. A failed cache can be retried but cannot be used by _run.sh.
    if not owner.exists():
        with owner.open('x', encoding='utf-8') as stream:
            json.dump(identity, stream, ensure_ascii=False, sort_keys=True)
            stream.write('\n')
    marker.unlink(missing_ok=True)
    identity['native_source_sha256'] = native_digest(source)
    identity['source_commit'] = subprocess.check_output(
        ['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    os.set_inheritable(fd, True)
    result = subprocess.run(['colcon', '--log-base', str(cache / 'log'), 'build',
        '--base-paths', str(source / 'ros2_ws/src'), '--packages-up-to', 'lunar_drl_exploration',
        '--executor', 'sequential', '--build-base', str(cache / 'build'),
        '--install-base', str(cache / 'install'), *args], pass_fds=(fd,))
    if result.returncode:
        raise SystemExit(result.returncode if result.returncode > 0 else 128-result.returncode)
    if native_digest(source) != identity['native_source_sha256']:
        raise ValueError('source changed during build; rebuild before launching')
    temporary = cache / '.drl-source.tmp'
    with temporary.open('x', encoding='utf-8') as stream:
        json.dump(identity, stream, ensure_ascii=False, sort_keys=True)
        stream.write('\n')
    temporary.replace(marker)



if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError) as exc:
        raise SystemExit('DRL build refused: ' + str(exc))
