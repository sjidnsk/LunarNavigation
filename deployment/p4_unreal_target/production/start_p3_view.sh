#!/usr/bin/env bash
set -e
run="${1:?指定与mapping相同的P4/run/p3-*目录}"
if (( $# != 1 )); then exit 2; fi
case "$run" in /home/yanfa/P4/run/p3-*) ;; *) exit 2;; esac
run="$(realpath -m -- "$run")"
if [[ "$(dirname -- "$run")" != /home/yanfa/P4/run || "$(basename -- "$run")" != p3-* ]]; then exit 2; fi
source /home/yanfa/P4/config/p4_environment.sh
source /home/yanfa/P3/lidar_visual_fusion/scripts/env.sh
# 显示专用旧版消息/Python模块与RViz插件；只读叠加，不影响mapping组件。
visual_bundle=/home/yanfa/P3/roma_t3_algorithm_bundle_20260825
source "$visual_bundle/workspace/install_native/local_setup.bash"
source /home/yanfa/P3/lidar_visual_fusion/scripts/env.sh
export AMENT_PREFIX_PATH="$visual_bundle/integration_demo/rviz_plugins/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$visual_bundle/integration_demo/rviz_plugins/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
export ROS_DOMAIN_ID=57 ROS_LOCALHOST_ONLY=0
export T3_VISUAL_RUNTIME="$run/visual"
export ROS_HOME="$run/ros-home/view" ROS_LOG_DIR="$run/ros-log/view"
mkdir -p "$T3_VISUAL_RUNTIME" "$ROS_HOME" "$ROS_LOG_DIR"
for name in p3_visual_window.rviz monitor_window.json view_bounds.json view_settings.json; do
 if [[ ! -f "$T3_VISUAL_RUNTIME/$name" ]]; then cp "$P4_ROOT/config/p3/visual/$name" "$T3_VISUAL_RUNTIME/$name"; fi
done
cd "$T3_VISUAL_RUNTIME"
exec /home/yanfa/P3/roma_t3_algorithm_bundle_20260825/integration_demo/rviz_window/build/t3_visual_window "$T3_VISUAL_RUNTIME/p3_visual_window.rviz" "$T3_VISUAL_RUNTIME" --ros-args -p use_sim_time:=false
