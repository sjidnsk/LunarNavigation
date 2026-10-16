"""Native navigation participates in formal training/evaluation provenance."""
from pathlib import Path
import hashlib
from ament_index_python.packages import get_package_prefix


def native_navigation_files():
    """Resolve the same ament prefixes used by OwnedProcesses.start()."""
    result = []
    for package, relative in (
            ('lunar_incremental_navigation_core', 'lib/liblunar_incremental_navigation_core.so'),
            ('lunar_incremental_navigation_ros', 'lib/liblunar_incremental_navigation_ros.so'),
            ('lunar_incremental_navigation_ros', 'lib/lunar_incremental_navigation_ros/lunar_incremental_navigation_node')):
        path = Path(get_package_prefix(package)) / relative
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
        result.append(('native:' + package + ':' + relative, digest.hexdigest()))
    return result


def controller_execution_files():
    """Fingerprint the installed executor resolved by the ROS package prefix."""
    prefix = Path(get_package_prefix('lunar_pure_wheeled_controller'))
    modules = sorted({path for pattern in (
        'lib/python*/site-packages/lunar_pure_wheeled_controller/*.py',
        'local/lib/python*/dist-packages/lunar_pure_wheeled_controller/*.py')
        for path in prefix.glob(pattern)})
    entry = prefix / 'lib/lunar_pure_wheeled_controller/lunar_pure_wheeled_controller_node.py'
    if not modules or not entry.is_file():
        raise RuntimeError('installed wheeled controller execution files are missing')
    return [('controller:' + str(path.relative_to(prefix)),
             hashlib.sha256(path.read_bytes()).hexdigest()) for path in [*modules, entry]]
