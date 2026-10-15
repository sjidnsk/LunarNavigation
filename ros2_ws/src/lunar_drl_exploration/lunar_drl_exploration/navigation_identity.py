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
