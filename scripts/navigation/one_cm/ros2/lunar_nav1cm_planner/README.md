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
|`planning_period_s`、`execution_period_s`、`global_period_s`、`global_budget_s`|规划调度和预算|重启；不改则保持原调度节奏|
|`goal_tolerance_m`、`goal_position_target_m`、`goal_yaw_tolerance_rad`、`stopped_linear_mps`、`stopped_angular_rps`|目标/停稳合同；位置收敛目标仍受验收容差约束|重启；不改则保持原目标精度和停稳条件|
|`allow_unknown`、`initial_travel_direction`、`clearance_radius_m`|地图策略、起步偏好及全局几何包络|重启；不改则保持原规划行为，起步偏好不等于全程禁止另一方向|
|`map_frame`、所有`*_topic`、`map_qos_reliability`、`map_qos_durability`、`action_name`|坐标解释、真实订阅/发布和Action端点|重启重新接线；不改则仍连接原端点和QoS|
|`odometry_timeout_s`、`execution_contract_enabled`、`execution_feedback_timeout_s`|定位持续更新检测、执行合同与反馈路径|重启；不改则保持原检测和合同，不引入源数据年龄停车规则|

`minimum_clearance_m`与`clearance_radius_m`先按现有能力下限收敛，参数表回写最终值后才冻结，不能通过小覆盖绕过车体/净空要求。启动日志`startup_parameters`记录最终声明参数值（Action名另行记录），资源仍由原配置解析和factory建立；不新增热更新管理器。这些日志不是节点健康、规划成功或车辆到达证明。


生产全局路线默认每 `global_period_s: 15.0` 秒后台刷新一次；新目标、当前剩余路线被真实障碍阻断、
无路线时新的障碍几何变化或真实参考重入立即请求。单次提交后开始下一个周期，保持单在途请求。
普通地图刷新、原地转向、不影响剩余路线的障碍变化无需立即重算；已走历史障碍不退休剩余路线。
后台刷新保留有效路线、版本、进度及局部来源；新结果须通过目标身份及当前地图完整几何验证。
同一几何的前向后缀保留原版本和进度，不同合法路线可以接替，包括障碍移除后的更短通路。
无解、worker或submit异常保留目标及原有效路线，消费本次事件并等待下一周期或新的真实事件，
不按每tick重试，也不因失败次数结束长期任务。`global_budget_s` 仍限制单次求解预算。
`global_period_s` 为启动只读参数，修改启动覆盖后重启；TCP主入口及有界入口复用同一配置。


## MPCC直控持续发布

选择`control_backend=mpcc_cmd_vel`时，地图线程独占地图处理，计算线程独占session、规划和认证；独立`nav1cm-command`线程按`execution_period_s`（TCP配置0.05秒）持续发布当前有效速度。新结果通过短锁交接速度、已有输入序号和本次证明的绝对截止时刻；原证明覆盖时间从本次计算起点扣除，重发不刷新证明年龄。计算/日志/Path发布不在发令循环内。

新计算结果未到时继续消费旧证明剩余持令时间；证明到期、取消/替换或定位持续更新检查不通过时，撤销旧交接并发送零速。沿用现有定位检查、碰撞认证、平台限制与目标身份。只有计算线程能重新认证；转发器不续期。重复发布不重置新速度指令的slew计时。若认证期间旧命令已实际归零，交接按当前已发速度最多补算一次，保持原计算起点；不持锁认证、不追赶周期。

`MPCC_COMMAND`的`publication_state=queued`表示计算线程交接，`actual_published_vw`为该时刻最近实际发布值；实际持续流从速度话题采样。`nav1cm_continuous_command_dds`仅在随机测试话题验证阻塞/连续慢计算期间的真实DDS输出及取消，不接入车辆或P3，不替代现场闭环验收。
