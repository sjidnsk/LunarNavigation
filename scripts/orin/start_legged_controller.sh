#!/usr/bin/env bash
set -eo pipefail
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$repository_root/scripts/orin/p3_environment.sh"
source /opt/ros/humble/setup.bash
source "$repository_root/ros2_ws/install/setup.bash"
set -u
exec ros2 launch lunar_pure_wheeled_controller pure_legged_controller.launch.py \
  config_file:="$repository_root/config/orin_legged_controller.yaml" \
  platform_config:="$repository_root/config/legged_orin.yaml" "$@"
