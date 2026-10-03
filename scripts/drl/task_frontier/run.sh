#!/usr/bin/env bash
set -euo pipefail
CONTROL_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u ROS_PACKAGE_PATH -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_DISTRO \
  -u ROS_VERSION -u ROS_PYTHON_VERSION \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /home/kai/.cache/lunar-drl-task-frontier/jazzy/install/local_setup.bash
    export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 MKL_NUM_THREADS=4 NUMEXPR_NUM_THREADS=4
    control_dir=$1; shift
    if [[ ${1:-} == python ]]; then
      shift
      exec /home/kai/.cache/lunar-drl-training/venv/bin/python "$@"
    fi
    exec /home/kai/.cache/lunar-drl-training/venv/bin/python "$control_dir/controller.py" "$@"
  ' task-frontier "$CONTROL_DIR" "$@"
