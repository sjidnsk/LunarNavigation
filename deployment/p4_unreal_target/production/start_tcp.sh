#!/usr/bin/env bash
set -e
source /home/yanfa/P4/config/p4_environment.sh
out="$P4_ROOT/run/tcp-$(date +%Y%m%d-%H%M%S)-$$"
export ROS_HOME="$out/ros-home" ROS_LOG_DIR="$out/ros-log"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
cd "$out"
exec /home/yanfa/Env_X/InterFace/install/lunar_car_ctrl/lib/lunar_car_ctrl/lunar_car_node --ros-args --params-file "$P4_ROOT/config/vehicle.yaml"
