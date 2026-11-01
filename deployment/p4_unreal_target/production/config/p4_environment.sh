#!/usr/bin/env bash
# 固定正式叠加链；source本文件不会启动进程。
export P4_ROOT=/home/yanfa/P4
unset QT_QPA_FONTDIR COLCON_PYTHON_EXECUTABLE AMENT_PYTHON_EXECUTABLE QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH QML2_IMPORT_PATH QML_IMPORT_PATH AMENT_PREFIX_PATH AMENT_CURRENT_PREFIX COLCON_PREFIX_PATH COLCON_CURRENT_PREFIX CMAKE_PREFIX_PATH CMAKE_MODULE_PATH ROS_PACKAGE_PATH PYTHONPATH PYTHONHOME PYTHONUSERBASE LD_LIBRARY_PATH LD_PRELOAD CYCLONEDDS_URI
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
source "$P4_ROOT/ros2-humble/install/local_setup.bash"
source /home/yanfa/Env_X/InterFace/install/local_setup.bash
source "$P4_ROOT/build/current/install/local_setup.bash"
export PATH="$P4_ROOT/.venv/bin:$PATH"
export VIRTUAL_ENV="$P4_ROOT/.venv"
export ACADOS_SOURCE_DIR="$P4_ROOT/deps/acados-v040"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=10 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
export TMPDIR="$P4_ROOT/tmp" TMP="$P4_ROOT/tmp" TEMP="$P4_ROOT/tmp"
export XDG_CACHE_HOME="$P4_ROOT/.cache" CUDA_CACHE_PATH="$P4_ROOT/.cache/cuda"
export XDG_CONFIG_HOME="$P4_ROOT/.cache/config" XDG_DATA_HOME="$P4_ROOT/.cache/data"
export COLCON_HOME="$P4_ROOT/.cache/colcon" PIP_CACHE_DIR="$P4_ROOT/.cache/pip" PYTHONPYCACHEPREFIX="$P4_ROOT/.cache/pycache"
export MPLCONFIGDIR="$P4_ROOT/.cache/matplotlib" NAV1CM_BUILD_DIR="$P4_ROOT/.cache/nav1cm"
export ROS_HOME="$P4_ROOT/run/ros-home" ROS_LOG_DIR="$P4_ROOT/run/ros-log"
mkdir -p "$TMPDIR" "$XDG_CACHE_HOME" "$ROS_HOME" "$ROS_LOG_DIR"
cd "$P4_ROOT"
