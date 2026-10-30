#!/usr/bin/env bash
# 在P4调试时显式约束所有工具写入，不修改用户或系统配置。
if [[ -n "${LUNAR_P4_ROOT:-}" ]]; then
  export TMPDIR="$LUNAR_P4_ROOT/tmp" XDG_CACHE_HOME="$LUNAR_P4_ROOT/.cache"
  export ROS_HOME="$LUNAR_P4_ROOT/run/ros-home" ROS_LOG_DIR="$LUNAR_P4_ROOT/log/ros"
  export PYTHONPYCACHEPREFIX="$LUNAR_P4_ROOT/.cache/pycache" MPLCONFIGDIR="$LUNAR_P4_ROOT/.cache/mpl"
  export PIP_CACHE_DIR="$LUNAR_P4_ROOT/.cache/pip" COLCON_HOME="$LUNAR_P4_ROOT/.cache/colcon"
  export NAV1CM_BUILD_DIR="${NAV1CM_BUILD_DIR:-$LUNAR_P4_ROOT/.cache/nav1cm}"
  export LUNAR_OBJ_TCP_SIM_BUILD_BASE="$LUNAR_P4_ROOT/ros2-humble"
  export LUNAR_OBJ_TCP_SIM_OVERLAY="$LUNAR_OBJ_TCP_SIM_BUILD_BASE/install"
  export ACADOS_SOURCE_DIR="$LUNAR_P4_ROOT/deps/acados-v040"
  export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export PATH="$LUNAR_P4_ROOT/.venv/bin:$PATH"
  export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
  mkdir -p "$TMPDIR" "$ROS_HOME" "$ROS_LOG_DIR" "$XDG_CACHE_HOME" "$COLCON_HOME"
fi
