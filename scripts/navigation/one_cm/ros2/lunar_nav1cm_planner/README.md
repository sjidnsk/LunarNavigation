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

## 可选任务 Action 入口

原 `planner_node` 保持原始 P3 / PoseStamped / 两条标准 Path 接口。需要任务结果时，改为启动 `navigation_task_node`；不要同时启动两个规划器。参数和能力配置与原入口一致，仍须显式设置仓库外 `build_dir`。

`action_name` 默认 `/Car/T4/navigation/navigate_to_pose`，消息复用 `lunar_planning_msgs/action/NavigateToPose`。Action UUID 是任务身份，ROS stamp 不参与身份判定。`has_target_yaw=false` 时等待实测位姿，采用接受时实测 yaw。传输接受后仍可能以 `INVALID_GOAL` 和具体原因结束；无效新请求保留旧有效任务。已接受目标后来受阻保持活动并反馈 `WAITING_FOR_PATH`。

有效人工目标或新 Action 替换旧请求；旧结果为 `outcome=CANCELED, reason_code=GOAL_REPLACED`，ROS 协议状态为 ABORTED，不表示车辆已经停稳。显式取消只匹配当前 UUID，立即退休旧规划身份和发布，发布通过当前图认证的受限制动；实测速度低于配置阈值后返回 `CANCELED_STOPPED`，ROS 协议状态为 CANCELED。

任务到达必须满足当前目标的 xy/yaw、实测线角速度容差，并保持 0.3 秒且有后续里程计测量；局部段结束、Path 存在、Action 接受均不代表到达。反馈 segment revision 仅报告真实当前路线版本，0 表示无；没有伪造执行 ACK。Action 使用异步 Future 和独立可重入回调组，单线程 executor 也不会因等待结果阻塞取消。

关闭入口会终结未完成 Action 为 `INTERNAL_ERROR/SHUTDOWN`，停止 Runtime 并有界等待 worker 退出。此入口不启动控制器、不访问真实 T5，也不启动探索或训练。


## 参数生效边界

本包`planner_node`与`navigation_task_node`的应用参数均仅启动时读取。启动时仍可通过launch、YAML或`--ros-args -p`覆盖；运行时`ros2 param set`明确拒绝，修改后须重启对应节点。`ros2 param describe`显示只读及restart说明，避免参数表已改而实际导航仍用旧值。ROS自身的`use_sim_time`保留原生更新路径，不在本包冻结范围内。

|参数组|启动时决定的对象|更改方式及保持原值的影响|
|---|---|---|
|`wheel_config`及`WheelLimits`同名参数（`linear_speed_mps`、`linear_accel_mps2`、`angular_speed_rps`、`motion_model`、车体/净空等）|加载的能力/地形判据、规划与认证模型|改启动配置并重启；不改则沿用原能力。只改规划器不等于跟踪器同步，部署仍须共用同一能力文件|
|`build_dir`、`horizon_steps`、`horizon_dt_s`|acados生成/缓存目录及模型时域|重启重建相应资源；不改则使用原时域/缓存指纹|
|`planning_period_s`、`execution_period_s`、`global_budget_s`|规划调度和预算|重启；不改则保持原调度节奏|
|`goal_tolerance_m`、`goal_position_target_m`、`goal_yaw_tolerance_rad`、`stopped_linear_mps`、`stopped_angular_rps`|目标/停稳合同；位置收敛目标仍受验收容差约束|重启；不改则保持原目标精度和停稳条件|
|`allow_unknown`、`initial_travel_direction`、`clearance_radius_m`|地图策略、起步偏好及全局几何包络|重启；不改则保持原规划行为，起步偏好不等于全程禁止另一方向|
|`map_frame`、所有`*_topic`、`map_qos_reliability`、`map_qos_durability`、`action_name`|坐标解释、真实订阅/发布和Action端点|重启重新接线；不改则仍连接原端点和QoS|
|`odometry_timeout_s`、`execution_contract_enabled`、`execution_feedback_timeout_s`|定位持续更新检测、执行合同与反馈路径|重启；不改则保持原检测和合同，不引入源数据年龄停车规则|

`minimum_clearance_m`与`clearance_radius_m`先按现有能力下限收敛，参数表回写最终值后才冻结，不能通过小覆盖绕过车体/净空要求。启动日志`startup_parameters`记录最终声明参数值（Action名另行记录），资源仍由原配置解析和factory建立；不新增热更新管理器。这些日志不是节点健康、规划成功或车辆到达证明。


生产全局路线按事件更新：新目标、当前路线被真实障碍变化阻断、无路线时新的障碍几何变化，
或实测位置确实无法投影回当前参考时，各触发一次全局请求。普通地图刷新、仅已知空闲区域变化、
原地转向以及不影响现路线的障碍变化保留路线版本与进度。障碍移除后仍有效的绕行也保留，
不自动追求新的最短路。无解或worker异常保留目标，等待新事件，不按秒重试。

已删除生产参数 `global_period_s`，旧启动覆盖应移除；`global_budget_s` 继续限制单次求解预算。
离线 `NavigationCoordinator` 的同名构造参数只在 `revalidate_current_map=False` 的原型模式下生效。
