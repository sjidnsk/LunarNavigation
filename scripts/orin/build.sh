#!/usr/bin/env bash
set -eo pipefail
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_scope="${1:-runtime}"
case "$build_scope" in
  runtime) packages=(lunar_incremental_navigation_ros lunar_pure_exploration_ros lunar_pure_wheeled_controller lunar_nav1cm_planner lunar_navigation_rviz) ;;
  demos) packages=(lunar_integrated_exploration_demo) ;;
  *) echo "用法：bash scripts/orin/build.sh [runtime|demos]" >&2; exit 2 ;;
esac
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
set -u
cd "$repository_root/ros2_ws"
colcon build --merge-install --base-paths "$repository_root/ros2_ws/src" \
  "$repository_root/scripts/navigation/one_cm/ros2/lunar_nav1cm_planner" \
  --build-base "${LUNAR_BUILD_ROOT:-$repository_root/ros2_ws/build}" \
  --install-base "${LUNAR_INSTALL_ROOT:-$repository_root/ros2_ws/install}" --packages-up-to "${packages[@]}" \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
