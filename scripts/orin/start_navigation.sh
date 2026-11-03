#!/usr/bin/env bash
set -eo pipefail
platform_type="${1:-legged}"
use_sim_time="${2:-false}"
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# 保留四个历史位置参数；第五个参数显式选择旧栈。
stack_mode="${5:-${LUNAR_NAVIGATION_MODE:-nav1cm}}"
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${LUNAR_INSTALL_ROOT:-$repository_root/ros2_ws/install}/setup.bash"
source "$repository_root/scripts/orin/p3_environment.sh"
set -u
extra_args=()
if (( $# > 5 )); then extra_args=("${@:6}"); fi
case "$stack_mode" in
  nav1cm)
    exec ros2 launch lunar_pure_exploration_ros wheel_navigation.launch.py \
      platform_type:="$platform_type" \
      planner_params_file:="${3:-}" use_sim_time:="$use_sim_time" start_rviz:="${4:-false}" \
      build_dir:="${NAV1CM_BUILD_DIR:-}" enable_controller:=false "${extra_args[@]}" ;;
  incremental_v2)
    exec ros2 launch lunar_pure_exploration_ros exploration_navigation.launch.py \
      stack_mode:=incremental_v2 config_file:="${3:-$repository_root/config/exploration_navigation.yaml}" \
      platform_type:="$platform_type" use_sim_time:="$use_sim_time" start_rviz:="${4:-false}" \
      start_navigation:=true start_exploration:=false "${extra_args[@]}" ;;
  *) echo '未知导航模式：仅支持 nav1cm 或 incremental_v2' >&2; exit 2 ;;
esac
