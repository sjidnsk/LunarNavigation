# DRL 可选 nav1cm 导航后端

此模块默认仍为 `incremental_v2`。仅显式 `navigation_backend=nav1cm` 时选择新客户端；旧 `config_record` 与默认 resume 语义不增加字段。显式新后端的经验身份不同，不能作为旧训练经验的等价恢复。

新客户端只消费 `NavigateToPose` 当前请求 UUID 的 feedback/result、实测 Odometry、TF/静态 TF 与 `GetPolicyMap`。标准全局 Path 只估计路程，不证明到达；不依赖旧 PathReference、TrackingStatus 或规划诊断 ACK。普通 INVALID_GOAL 结果仍是普通导航失败；发送/结果传输丢失、服务端传输拒绝和 INTERNAL_ERROR 保持 InfrastructureError。取消结果还需新鲜实测速度停稳才交还。

`RosExplorationEnv` 与 `InferenceRuntime.attach` 共用 factory。新模式的自有组合启动 map-only、navigation_task_node 和标准 Path MPPI，使用独立命名空间；必须显式传入公共 0.2 m 净空能力配置，并让地图、规划器、控制器和策略图使用同一配置。禁止旧 UNKNOWN 起步补全。独立合成输入须先有真实观测支撑。

部署入口 `lunar_drl_inference_node` 只调用 `ActorPolicy.load` 和 `InferenceRuntime.attach`。它接收公共组合的 actor_path、navigation_backend、platform_config、task/status/action/map/odom/TF/route 及 frame 参数。默认从 Actor 的 `actors/../config.json` 读取只读模型、sensor 和 graph 配置；非此目录布局可显式给 `actor_config_path`。模型元数据不匹配即报错，不迁移权重、不回退随机策略。运行该入口的 Python 必须已有兼容 Torch；不要把整个 user-site 放进公共 ROS 子进程 PYTHONPATH。

原 `lunar-drl infer` 增加可选 `--navigation-backend`，默认仍为 incremental_v2；train 分发与参数未改。v5 Actor 输入通过 RosEnv/InferenceRuntime 共用纯 `prepare_policy_state`，复用原 `prepare_candidate_observation`/`prepare_policy_graph`，保持原字段、候选次序、metadata、网络与奖励。

`scripts/drl/verify_nav1cm_backend.py` 是独立合成短推理工具。必须先 `--declare-only` 冻结 actor/config/platform/source/fixture/duration，再对同一声明 `--run`；输出目录不可复用。工具使用域 92、本机发现、私有命令主题和独立命令驱动 plant；仅 plant 持有合成物理场景，策略只读观测地图。正式训练包、权重、账本和论文 cohort 不作为写入目标。

当前资格见 `docs/validation/2026-10-29-drl-nav1cm-backend.md`。在 C1 合格公共 overlay 上，固定权重已通过合成短窗的实际运动、观测推进与当前 Action 取消停稳检查；默认旧后端有界兼容 smoke 通过。45 秒内未到达目标，不可宣称策略有效或 Orin/实车就绪。公共 C1 安装必须位于 Task3 安装之后、本包安装之前；源码/部署验收仍由 ROOT 独立审查。

推理 CLI 的后端优先级为显式 `--navigation-backend` > 独立配置中的 `navigation_backend` > 默认 `incremental_v2`。配置的后端值与未知字段仍先验证；同名可选字段只由 infer 接受，train 参数及默认配置加载规则不变。

任务消息 START 使用 adapter/Runtime 同一个有效 `map_frame`（默认 map）。错误帧会记录拒绝并返回，不转换坐标、不覆盖当前任务，也不会因帧校验异常退出 ROS 订阅。


owned RosEnv 模式采用 TrainingConfig 已记录的位置/yaw 到达容差，三方一致透传：地图与标准Path follower为 `goal_position_tolerance_m`，planner为 `goal_tolerance_m`，三方yaw均为 `goal_yaw_tolerance_rad`。显式nav1cm且使用TrainingConfig默认时执行.05m与pi/12rad；自定义合法配置同样保留。直接调用进程规格helper未传容差时保持公共.1m/.05rad默认，公共launch与legacy默认配置身份不变。该修复已完成纯参数合同验证与独立Humble构建，待最终scoped复审；不代表完整RosEnv episode或策略效果已验收。
