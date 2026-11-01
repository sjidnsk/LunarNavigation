#!/usr/bin/env bash
set -eo pipefail
platform_type="${1:-legged}"
use_sim_time="${2:-false}"
navigation_backend="${3:-${LUNAR_NAVIGATION_MODE:-nav1cm}}"
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${LUNAR_INSTALL_ROOT:-$repository_root/ros2_ws/install}/setup.bash"
set -u
extra_args=()
if (( $# > 3 )); then extra_args=("${@:4}"); fi
if [[ "$navigation_backend" == nav1cm ]]; then
  # 与 start_navigation 二选一；只启动一套公共导航/探索服务，等待外部任务 START。
  exec ros2 launch lunar_pure_exploration_ros wheel_navigation.launch.py \
      platform_type:="$platform_type" \
    exploration_strategy:=frontier enable_controller:=false use_sim_time:="$use_sim_time" \
    build_dir:="${NAV1CM_BUILD_DIR:-}" "${extra_args[@]}"
elif [[ "$navigation_backend" != incremental_v2 ]]; then
  echo 'navigation_backend 必须为 incremental_v2 或 nav1cm' >&2
  exit 2
fi
exec ros2 launch lunar_pure_exploration_ros exploration_navigation.launch.py \
  stack_mode:=incremental_v2 platform_type:="$platform_type" use_sim_time:="$use_sim_time" \
  start_navigation:=false start_exploration:=true "${extra_args[@]}"
