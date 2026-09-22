#!/usr/bin/env bash
# Start the complete wheel planning/control stack from the bundle root.
set -eo pipefail

platform="${1:-wheel}"
environment_mode="${2:-2}"

case "${platform}" in
  wheel) ;;
  *)
    echo "用法：$0 [wheel] [environment_mode]" >&2
    exit 2
    ;;
esac

bundle_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${bundle_root}/ros2_ws/install/setup.bash"

pids=()
cleanup() {
  trap - INT TERM EXIT
  for pid in "${pids[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

ros2 launch lunar_pure_planner_ros pure_planner.launch.py \
  platform_type:="${platform}" \
  wheel_planner_mode:="grid_traversability_v1" &
pids+=("$!")

ros2 launch lunar_pure_planner_ros local_traversability.launch.py \
  platform_type:="${platform}" &
pids+=("$!")

ros2 launch lunar_pure_planner_ros rviz_goal_bridge.launch.py \
  environment_mode:="${environment_mode}" &
pids+=("$!")

# This legacy stack publishes MotionReference, not incremental PathReference.
# Preserve its former controller defaults; deployment launch defaults are Orin-specific.
ros2 launch lunar_pure_wheeled_controller pure_wheeled_controller.launch.py \
  input_mode:=motion_reference \
  tf_topic:=/tf tf_timeout_s:=0.5 \
  max_angular_radps:=0.5 \
  goal_position_tolerance_m:=0.2 goal_yaw_tolerance_rad:=0.2 &
pids+=("$!")

wait -n "${pids[@]}"
