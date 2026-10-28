# P3 标准 Path 规划接口

本包接收 `GridMap`、`Odometry`、`TFMessage` 和 `PoseStamped`，只输出两条 `nav_msgs/Path`。
不依赖 DRL 或验证 plant。业务输出没有 Action、ACK 或附加状态协议。

需要 Ubuntu 22.04 / ROS 2 Humble、`lunar_incremental_navigation_core` underlay，以及显式提供的
acados/CasADi SDK。构建不下载 SDK，不提供替代求解器。先 source Humble 与 core underlay，
配置 SDK 的 `PYTHONPATH` / `LD_LIBRARY_PATH` 后，用独立仓库外 build/install/log：

```bash
colcon --log-base "$RUN/log" build --base-paths "$SOURCE/scripts/navigation/one_cm/ros2/lunar_nav1cm_planner" \
  --packages-select lunar_nav1cm_planner --build-base "$RUN/build" --install-base "$RUN/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
source "$RUN/install/setup.bash"
ros2 launch lunar_nav1cm_planner planner.launch.py build_dir:="$RUN/mpcc"
```

`build_dir` 必须是显式的外部缓存路径，可用 `NAV1CM_BUILD_DIR` 提供。安装包包含 nav1cm、
三份 native 库、launch 和配置；运行不需要源码目录的 `PYTHONPATH`。native 库通过 ament
prefix 显式定位，terrain 能力通过安装的 `config/wheel.yaml` 显式传入。

默认话题可用同名参数或 ROS remap 修改：

| 参数 | 默认话题 | 类型 |
| --- | --- | --- |
| map_topic | /Car/T3/mapping/grid_map | grid_map_msgs/GridMap |
| odometry_topic | /Car/T3/localization/odometry | nav_msgs/Odometry |
| goal_topic | /Car/T4/navigation/goal | geometry_msgs/PoseStamped |
| tf_topic | /tf | tf2_msgs/TFMessage |
| tf_static_topic | /tf_static | tf2_msgs/TFMessage |
| global_route_topic | /Car/T4/planning/global_route | nav_msgs/Path |
| local_path_topic | /Car/T4/planning/local_path | nav_msgs/Path |

旧 `/P4/input/map_to_odom` 可设置为 `tf_topic`；该输入使用 reliable/volatile、depth 10，
无需 transient_local。标准 `/tf_static` 使用 reliable/transient_local。只取最新 TF，
零时间戳及其后续更新均有效，不要求三类输入严格时间同步。暂缺 TF 的地图/目标会保留并重试。
目标 frame 留空按 map；Odometry pose 使用 header frame，纵向速度用 child frame 中有符号
`linear.x`，角速度用 `angular.z`。不要求 child 名称等于能力文件的 base_frame_id。

`elevation` 可使用任意正源分辨率。地形判定先在源分辨率调用 core，局部始终
0.01 m、500×500、5×5 m。厘米网格不提高真实观测精度，NaN 保持 UNKNOWN。
运动/车体约束来自 `wheel.yaml → WheelLimits → 显式同名 ROS 参数`；默认车体 1.182×0.818 m、
双向 0.2 m/s、转速 1.0 rad/s、线/角加速度 0.5、曲率 1.0。局部允许未知由 `allow_unknown` 控制。

最新有效目标覆盖当前目标；矩形目标车体碰已知障碍才拒绝，未知/图外允许。无效新目标保留当前
目标；新目标无路或地图后来受阻也不取消。相同归一化位姿重发不重启规划，A→B→A 仍隔离旧结果。

全局 Path 是当前目标的几何路线。局部 Path 是当前反馈起算、经当前地图整段及制动尾段认证的
完整候选；倒车使用实际车体 yaw，原地转向保留同 xy 的全部点。header stamp 为本次 ROS `t0`，
局部 pose stamp 为 `t0+ti`，不只发布 50 ms 前缀。下游可按参考时序跟踪，也可按几何路径自行重定时；重定时后自行满足速度/转向/加速度/制动约束，不直接继承原时序认证。
空局部 Path 表示当前没有可执行路径，下游执行受控停车，不能当作取消目标。

地图处理与求解不阻塞 ROS 输入回调；调度与耗时记录使用单调时钟。标准 Path 模式以认证通过的
新 worker 结果建立固定 canonical source，按实测位置/朝向单调推进物理进度，不因反馈较参考慢
就按墙钟丢掉尚未走过的几何。暂停不消费几何，不为 source 设置墙钟 TTL。每 tick 由真实状态
派生受限接续并在当前图重新认证；派生轨迹和 bridge 不回写 source，也不充值来源余量。
bridge 与停车尾段都扣同一份有限弧长/转角预算，预留一个反应周期和真实刹停余量再截断、追加
受限制动。仅因超过 50 ms 不清空安全旧参考；来源耗尽或安全条件不满足仍受限停车。
全局原始正式结果字段及旧 epoch 保留，当前图重验独立记录。旧离线 strict elapsed 控制缓冲
保留独立语义，不作为标准 Path 下游必须精确跟时序的要求。

有界合成输入、独立三进程 DDS 和 Path 几何重定时运动验证见
[验证入口](../../p3_path_validation/README.md)。跨日实施与各环境证据见
[2026-10-27 接口验证报告](../../../../../docs/reports/2026-10-27-nav1cm-p3-interface-validation.md)。

源码/纯计算、Humble x86 消息、Orin 与实车证据分别记录。安装及单元测试不能替代 Orin/实车验收。
