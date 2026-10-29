#!/usr/bin/env bash
set -eo pipefail

platform_type="${1:-wheel}"
use_sim_time="${2:-false}"
navigation_backend="${3:-incremental_v2}"
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/humble/setup.bash
source "$repository_root/ros2_ws/install/setup.bash"
set -u
if [[ "$navigation_backend" == "nav1cm" ]]; then
  if [[ "$platform_type" != "wheel" ]]; then
    echo "nav1cm 前沿入口当前仅轮式能力" >&2
    exit 2
  fi
  exec ros2 launch lunar_pure_exploration_ros nav1cm_exploration.launch.py \
    exploration_strategy:=frontier enable_controller:=false use_sim_time:="$use_sim_time" \
    build_dir:="${NAV1CM_BUILD_DIR:-}"
elif [[ "$navigation_backend" != "incremental_v2" ]]; then
  echo "navigation_backend 必须为 incremental_v2 或 nav1cm" >&2
  exit 2
fi
exec ros2 launch lunar_pure_exploration_ros exploration_navigation.launch.py \
  stack_mode:=incremental_v2 \
  platform_type:="$platform_type" \
  use_sim_time:="$use_sim_time" \
  start_navigation:=false \
  start_exploration:=true
