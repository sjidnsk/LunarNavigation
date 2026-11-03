#!/usr/bin/env bash
set -eo pipefail
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${LUNAR_INSTALL_ROOT:-$repository_root/ros2_ws/install}/setup.bash"
source "$repository_root/scripts/orin/p3_environment.sh"
set -u
exec python3 "$repository_root/scripts/orin/navigation_goal.py" "$@"
