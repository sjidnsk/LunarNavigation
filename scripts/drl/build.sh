#!/usr/bin/env bash
set -e
DRL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u ROS_PACKAGE_PATH -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_DISTRO \
  -u ROS_VERSION -u ROS_PYTHON_VERSION \
  bash --noprofile --norc -c '
    set -e
    DRL_ROOT=$1; shift
    DRL_CACHE=${DRL_CACHE:-$HOME/.cache/lunar-drl-redesign/humble}
    source "${DRL_ROS_SETUP:-/opt/ros/humble/setup.bash}"
    set -u
    if [[ "$ROS_DISTRO" != humble ]]; then echo "Use a separate Humble build cache" >&2; exit 2; fi
    cd "$DRL_ROOT"
    export CMAKE_BUILD_PARALLEL_LEVEL=${DRL_BUILD_JOBS:-2}
    exec python3 "$DRL_ROOT/scripts/drl/build_cache.py" "$DRL_ROOT" "$DRL_CACHE" "$@"
  ' drl-build "$DRL_ROOT" "$@"
