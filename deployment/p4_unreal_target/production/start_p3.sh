#!/usr/bin/env bash
# 各组件独立前台运行；mapping退出不由可视化决定。
set -e
component="${1:-}"
case "$component" in mapping|relay|capture|wheel|monitor) ;; *) echo '用法: bash start_p3.sh {mapping|relay|capture|wheel|monitor} /home/yanfa/P4/run/p3-新名称' >&2; exit 2;; esac
run="${2:?必须指定本次P3共用的新运行目录}"
if (( $# != 2 )); then exit 2; fi
case "$run" in /home/yanfa/P4/run/p3-*) ;; *) echo '运行目录必须位于P4/run/p3-*' >&2; exit 2;; esac
run="$(realpath -m -- "$run")"
if [[ "$(dirname -- "$run")" != /home/yanfa/P4/run || "$(basename -- "$run")" != p3-* ]]; then exit 2; fi
source /home/yanfa/P4/config/p4_environment.sh
source /home/yanfa/P3/lidar_visual_fusion/scripts/env.sh
export ROS_DOMAIN_ID=57 ROS_LOCALHOST_ONLY=0
export ROS_HOME="$run/ros-home/$component" ROS_LOG_DIR="$run/ros-log/$component"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
cd "$run"
p3=/home/yanfa/P3/lidar_visual_fusion
out="$run/results/live"
case "$component" in
 mapping)
  mkdir -p "$out"
  cp "$P4_ROOT/config/p3/profile.yaml" "$out/profile.yaml"
  exec /usr/bin/python3 "$P4_ROOT/p3_runtime.py" mapping "$run" ;;
 relay) exec /usr/bin/python3 "$P4_ROOT/p3_runtime.py" relay "$run" ;;
 wheel)
  export ROS_DOMAIN_ID=10
  exec /usr/bin/python3 "$p3/scripts/sim_imu/wheel_speed_relay.py" ;;
 capture)
  export ROS_DOMAIN_ID=10
  exec "$p3/install/capture_transport/lib/sensor_capture_ros2/sensor_capture_node" --ros-args    -p tcp_host:=192.168.10.23 -p tcp_port:=6665 -p capture_freq:=2.0    -p batch_interval:=0.0 -p batch_count:=1000000000 -p timestamp_mode:=meta_relative    -p "save_dir:=$run/capture_scratch" -p save_data:=false -p auto_start:=true    -p big_endian:=false -p publish_thumbnail:=false -p parallel_publication:=true    -p publication_max_age_sec:=3.0 -p publication_batch_mib:=64    -p 'output_topics:=[rgb_0:=/Car/T5/Cam_Left/image_raw/color,rgb_1:=/Car/T5/Cam_Right/image_raw/color,cpulidar_0:=/Car/T5/OS1/points,tof_0_depth:=/Car/T5/TOF_Left/image_raw/depth,tof_1_depth:=/Car/T5/TOF_Right/image_raw/depth]' ;;
 monitor)
  # 显示专用旧版消息/Python模块与RViz插件；只读叠加，不影响mapping组件。
  visual_bundle=/home/yanfa/P3/roma_t3_algorithm_bundle_20260825
  source "$visual_bundle/workspace/install_native/local_setup.bash"
  source /home/yanfa/P3/lidar_visual_fusion/scripts/env.sh
  export AMENT_PREFIX_PATH="$visual_bundle/integration_demo/rviz_plugins/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$visual_bundle/integration_demo/rviz_plugins/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
  export T3_VISUAL_RUNTIME="$run/visual"
  mkdir -p "$T3_VISUAL_RUNTIME"
  exec /usr/bin/python3 "$p3/scripts/p3_visual_monitor.py" --ros-args    -r /T3/mapping/lidar_status:=/fusion/map_status    -r /Car/T5/Cam_Left/image_raw/color:=/fusion/left    -r /Car/T5/Cam_Right/image_raw/color:=/fusion/right    -r /Car/T3/metrics/frame_timing:=/fusion/learned_status    -p use_sim_time:=false -r /T3/semantic/current_pose:=/Car/T3/localization/odometry ;;
esac
