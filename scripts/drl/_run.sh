#!/usr/bin/env bash
# Run in a clean child shell so inherited worktree overlays cannot select old code.
set -e
DRL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u ROS_PACKAGE_PATH -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_DISTRO \
  -u ROS_VERSION -u ROS_PYTHON_VERSION \
  bash --noprofile --norc -c '
    set -e
    DRL_ROOT=$1; shift
    if [[ "${PYTHONOPTIMIZE-1}" != 1 ]]; then
      echo "DRL Humble launcher requires PYTHONOPTIMIZE=1; unset it or explicitly set 1 (NumPy grid messages require this mode)" >&2
      exit 2
    fi
    DRL_ROS_SETUP=${DRL_ROS_SETUP:-/opt/ros/humble/setup.bash}
    DRL_CACHE=${DRL_CACHE:-$HOME/.cache/lunar-drl-redesign/humble}
    DRL_PYTHON=${DRL_PYTHON:-$HOME/.cache/lunar-drl-training/venv/bin/python}
    source "$DRL_ROS_SETUP"
    python3 "$DRL_ROOT/scripts/drl/build_cache.py" --verify "$DRL_ROOT" "$DRL_CACHE"
    source "$DRL_CACHE/install/local_setup.bash"
    set -u
    if [[ "$ROS_DISTRO" != humble ]]; then
      echo "DRL local launcher requires a separate Humble cache" >&2; exit 2
    fi
    # Humble 生成消息的调试检查不接受 NumPy 网格；子进程必须继承同一模式。
    export PYTHONOPTIMIZE=1
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 MKL_NUM_THREADS=4 NUMEXPR_NUM_THREADS=4
    cd "$DRL_ROOT"
    exec "$DRL_PYTHON" -m lunar_drl_exploration.cli "$@"
  ' drl "$DRL_ROOT" "$@"
