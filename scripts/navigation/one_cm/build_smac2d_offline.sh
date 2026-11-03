#!/usr/bin/env bash
# 仅在获准隔离目录中运行；下载/解包，不安装系统包，不启动导航。
set -euo pipefail
cd /home/yanfa/P4/tmp/global-smac-comparison-20261102
apt-get download ros-humble-nav2-smac-planner ros-humble-nav2-costmap-2d ros-humble-nav2-core ros-humble-nav2-util ros-humble-nav2-msgs ros-humble-ompl ros-humble-nav2-voxel-grid ros-humble-bondcpp ros-humble-bond ros-humble-smclib nlohmann-json3-dev
for file in ./*.deb; do dpkg-deb -x "$file" extracted; done
source /opt/ros/humble/setup.bash
E=$PWD/extracted/opt/ros/humble
includes=()
for directory in /opt/ros/humble/include/* "$E"/include/*; do includes+=(-idirafter "$directory"); done
g++ -std=c++17 -O3 smac2d_offline.cpp "${includes[@]}" -idirafter "$E/include" -Iextracted/usr/include -I/usr/include/eigen3 -I"$E/include/ompl-1.7" -L"$E/lib" -L"$E/lib/aarch64-linux-gnu" -L/opt/ros/humble/lib -Wl,-rpath-link,"$E/lib" -Wl,-rpath-link,"$E/lib/aarch64-linux-gnu" -lnav2_smac_planner -lnav2_costmap_2d_core -lrclcpp -lrclcpp_lifecycle -lrcutils -lrcl -o smac2d_offline
export LD_LIBRARY_PATH=$E/lib:$E/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=217 ROS_LOG_DIR=$PWD/roslogs
# 调用示例：python3 benchmark_smac2d.py --requests suite.json --binary ./smac2d_offline --bounds -35 -40 35 30 --resolution .1 --output results-suite.json
