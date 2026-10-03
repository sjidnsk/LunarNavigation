#!/usr/bin/env bash
set -euo pipefail
CONTROL_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u ROS_PACKAGE_PATH -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_DISTRO \
  -u ROS_VERSION -u ROS_PYTHON_VERSION \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/jazzy/setup.bash
    control_dir=$1; shift
    if [[ -n ${DRL_CACHE:-} ]]; then
      python3 - "$control_dir" "$DRL_CACHE" <<"PY"
import json, sys
from pathlib import Path
source=Path(sys.argv[1]).resolve().parents[2]
cache=Path(sys.argv[2]).absolute()
if any(p.is_symlink() for p in [cache,*cache.parents]):raise SystemExit("unsafe cache symlink")
marker=cache/".drl-source.json"
if marker.is_symlink():raise SystemExit("unsafe cache marker")
identity=json.loads(marker.read_text())
if identity.get("source_root")!=str(source) or identity.get("ros_distro")!="jazzy":raise SystemExit("foreign cache identity")
native=cache/"build/lunar_drl_exploration/native"
if not native.is_symlink() or native.resolve()!=source/"ros2_ws/src/lunar_drl_exploration/native":raise SystemExit("foreign native source")
PY
    fi
    export DRL_CACHE=${DRL_CACHE:-/home/kai/.cache/lunar-drl-task-frontier/jazzy}
    source "$DRL_CACHE/install/local_setup.bash"
    export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 MKL_NUM_THREADS=4 NUMEXPR_NUM_THREADS=4
    if [[ ${1:-} == python ]]; then
      shift
      exec /home/kai/.cache/lunar-drl-training/venv/bin/python "$@"
    fi
    exec /home/kai/.cache/lunar-drl-training/venv/bin/python "$control_dir/controller.py" "$@"
  ' task-frontier "$CONTROL_DIR" "$@"
