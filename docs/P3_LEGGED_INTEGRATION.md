# 足式 P3 → P4 联调（192.168.100.237）

2026-09-23 只读核对 `/home/yanfa/P3/lidar_visual_fusion` 源码。未修改、构建、启动或停止 P3。
足式正式入口是 `start_dog.sh` + `config/hardware237.yaml`，不是 wheel 的 `start_hardware.sh`。
足式 P3 输出域默认 69（P3_DOG_DOMAIN 可覆盖），传感器输入域 29；P4 订阅输出域 69。
轮式 P3 输出域 59 不适用于此默认配置。历史 run_state.json 是旧运行记录，不当作当前运行证明。

## 实际输入契约

| 输入 | P3 源码定义 | P4 处理 |
| --- | --- | --- |
| /Car/T3/mapping/grid_map | grid_map_msgs/msg/GridMap；odom；Reliable + TransientLocal + KeepLast(1) | 直接订阅 elevation，未知 NaN 保留 |
| /Car/T3/localization/odometry | nav_msgs/msg/Odometry；odom → base_link；Reliable + Volatile + depth 30 | 正式导航/探索/控制使用该经过 Guard 检查的输出 |
| /tf_static | tf2_msgs/msg/TFMessage；静态 map → odom，P3 launch 提供单位变换 | P4 只转发到 /P4/input/map_to_odom，保留源时间戳 |
| /tf | tf2_msgs/msg/TFMessage；Guard 输出对应的动态 odom → base_link | P4 位姿从正式 Odometry 获取，不把该变换当作 map → odom |

地图配置分辨率 0.2 m、局部窗口 64 m、发布计时器 1 s（有数据/dirty/资源压力门控，并非保证恒定 1 Hz）。
`elevation_only` 输出层：elevation、elevation_variance、height_range、roughness、observation_count。
P4 当前仅消费 elevation；其他层不被误认为可通行性标签，也不自动转成高程置信区间。
P3 的高程消息采用翻转双轴后的 Eigen 列主序、column_index/row_index、零环形起点，P4 已支持。
P3 本配置不发布旧 OccupancyGrid global_overview；P4 从高程自行派生 /Car/T4/mapping/exploration_map，探索订阅此图。
不订阅 /lio/odom 或 /fusion/ekf 来绕开 P3 正式输出门控。

源码依据（均为 P3 根目录下）：
- `start_dog.sh:4-8`、`config/hardware237.yaml`：域、配置、地图参数。
- `scripts/fusion.launch.py`：静态 map→odom 与传感器变换。
- `src/t3_lidar_visual_fusion/t3_lidar_visual_fusion/terrain_mapper.py:91-105,309-311,359-369`：发布者、层与坐标系。
- `src/t3_lidar_visual_fusion/t3_lidar_visual_fusion/legacy/grid_map_message.py:53-139`：矩阵、NaN 和消息编码。
- `src/t3_lidar_visual_fusion/t3_lidar_visual_fusion/odometry_guard.py:69-76` 与 `adaptive_guard.py:368-422`：正式里程计与合格 TF。

## P4 适配

所有正式脚本统一 source `scripts/orin/p3_environment.sh`：导航、探索、控制器、静态 TF 转发、导航目标与探索任务。
域选择为显式 ROS_DOMAIN_ID > P3_DOG_DOMAIN > 69。RMW 为 CycloneDDS，localhost 默认关闭。
自带 `config/cyclonedds_p3.xml` 对齐 P3 的 multicast/auto participant/上限 90 策略；显式 CYCLONEDDS_URI 优先。
没有复制 P3 实现到 P4，没有新建共享 TF 或伪造位姿。轮式分支不作此域修改。

## 操作

先由 P3 自己的操作流程启动足式链；P4 脚本不会启动 P3。
P4 构建完成后，每个终端执行：

```bash
cd /home/yanfa/P4
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=69
export CYCLONEDDS_URI="file://$PWD/config/cyclonedds_p3.xml"
source scripts/orin/p3_environment.sh
```

上述显式 export 用于清除此前手工设置的域 59/轮式 DDS 配置影响。
各占一个终端：

```bash
bash scripts/orin/start_static_tf_input.sh
# 另一个终端
bash scripts/orin/start_hardware_navigation.sh
# 控制器终端（真实足式驱动尚未接入）
bash scripts/orin/start_legged_controller.sh
# 可选探索终端
bash scripts/orin/start_exploration.sh
```

在设置同样环境的额外终端检查输入，P3 未运行时不应看到伪造样本：

```bash
ros2 topic info -v /Car/T3/mapping/grid_map
ros2 topic echo --once /Car/T3/localization/odometry
ros2 topic echo --once /P4/input/map_to_odom
ros2 param get /incremental_navigation platform_type
ros2 param get /incremental_exploration platform_selector
```

手工导航示例（map 坐标，替换成现场目标）：
`bash scripts/orin/publish_navigation_goal.sh 2 1 0`

探索示例（map 坐标矩形，替换成实际区域）：
`bash scripts/orin/publish_exploration_task.sh legged_task_01 -5 -5 5 5`

探索会自行调用导航；不要同时发送手工目标。P3 重启/定位原点变化后须结束旧任务、重启 P4 导航与 TF 适配。
静态 TF 重复仅证明适配器存活，不能代替新里程计；正式里程计中断时保留停车/等待机制。

## 验证边界

本轮只读源码核对和 P4 脚本环境回归已执行；同域实时 P3 样本与实车闭环尚未通过验收。
源码参数不等同于传感器实测性能。完整证据记录于 docs/validation/2026-09-23-p3-legged-interface-adaptation.md。
