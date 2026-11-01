# 前沿探索与 DRL 对接 NAV1CM：已确认设计

任务 NAV-1CM-AUTO-20261029，延续 NAV-1CM-20261026；公共 owner integration/pure-planner-orin，DRL 专用 owner dev/drl-exploration。用户已明确确认本方案并授权依次实施，无需重复请求计划审批：任务层复用现有 NavigateToPose Action，新规划器保持标准 Path 输出；共用任务层和地图接口 → 前沿探索 → DRL；不切换新训练任务。

## 目标与边界

前沿与DRL只负责选点，共用目标执行合同；规划器负责全局路线、局部MPCC与认证，MPPI负责实际执行。保持两条标准nav_msgs/Path，既有PoseStamped人工入口继续可用。新模式不启动旧导航节点取得地图，不强加PathReference、执行ACK或旧完整导航合同。

本地MAIN专题从已验收185ece29继续，在D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/worktree；原Windows integration工作区439178f2及他人修改不得写入/暂存/回退。DRL后续从实际远程owner a094b90206db9c9bef2b6cf075db55cf210ccc22建立独立专题；本地dev引用dfd422b7与其不同，不移动任何既有owner。DRL使用公共模块的独立Humble安装underlay，不为依赖合并整条主线。所有topic仅提交待owner验收，不推送、合并owner、PR、upstream或删除。

## 固定约束

- 输入 P3 GridMap elevation、Odometry、TF，接受任意有效正源分辨率及运行中变化、静态/零stamp TF；不新增严格时间戳、协方差或可选层门槛。
- 规划局部窗口5m×5m、500×500、0.01m；有效导航净空0.2m，实际车体与运动约束不放宽；地图重采样不增加物理精度。
- 全局允许UNKNOWN；局部allow_unknown默认false；真实UNKNOWN及已观测标记独立，暂不加入起步盲区补全。
- 普通计算超过50ms不单独触发制动；现有碰撞、运动及制动认证继续执行。
- 最新有效目标替换，无效新目标说明原因且保留原任务；已接受目标后来受阻仍保留，由策略决定何时取消/重选。
- 任务成功必须由当前目标的实际xy/yaw与停稳判定，沿用选定能力容差并保持0.3s；Path存在、局部段结束和Action传输接受均不充当实际到达。
- 明确取消停止旧任务后续路径、使在途旧结果失效并受限制动，实测停稳才结束取消；有效目标替换本身不强制车辆先停。
- 训练网络、观测字段定义、动作含义、奖励公式和现有默认后端保持；不启动/恢复/切换新或既有训练任务，不写冻结运行包、权重、账本或历史结果。
- 只在已获准的kai@10.63.250.129 Ubuntu22.04/Humble x86_64隔离环境验证；ROS_DOMAIN_ID=92、ROS_LOCALHOST_ONLY=1，实际控制输出私有主题，禁止连接真实T5。
- 场景最大100m，实际运动使用其中短距离；不跑公里长程。依赖、build/install/log、证据均仓库外D盘或远端任务artifact独立目录。

## 方案选择

用户选择共享薄任务层与独立探索地图组件：避免各策略重复执行状态机，同时不把旧接口整体搬回新规划器。仅Topic各写客户端会重复取消/完成/地图逻辑；整体重建旧导航接口扩大范围。复用已有NavigateToPose消息类型，保持旧端点可配置；不改Action/PathReference/TrackingStatus的消息布局。

## 任务接口

增加可选 navigation_task_node ROS入口，组合现有PlannerNode和任务适配组件；原planner_node仍只暴露原接口。任务Action默认/Car/T4/navigation/navigate_to_pose，目标和反馈均按Action UUID绑定，映射到规划器accepted_generation，不能把ROS stamp当任务ID，也不能解析日志推断状态。任务组件读取结构化内部会话事件；Runtime仍是session/coordinator唯一写入线程。

目标x/y/yaw来自现有Action；has_target_yaw=false时以接受时实测车体yaw补齐，等待位姿期间不伪造yaw。无效新目标必须以可解释INVALID_GOAL结果结束该请求，原有效任务继续；不能把所有Action拒绝当基础设施故障。手工PoseStamped有效输入与Action目标经过同一队列和校验，替换通知旧任务CANCELED/GOAL_REPLACED，但不谎称其已停稳。

反馈复用PLANNING/EXECUTING/REPLANNING与reason_code，必要等待用WAITING_FOR_MAP/WAITING_FOR_PATH区分；旧segment字段只作真实版本观测或0表示无，不伪造下游确认。暂时无路不是立即任务终止，不因局部Path短/空误报到达。显式cancel只影响匹配的活动UUID；旧取消、A→B→A、迟到回调/worker结果不得影响新任务。取消必须有界响应且支持shutdown，不能在单线程executor内阻塞取消回调。

纯任务合同建议放nav1cm/navigation_task.py：NavigationTaskCoordinator管理不可变TaskSnapshot；submit/cancel/observe接收会话事件与单调时间。Runtime提供offer_managed_goal(task_id,message)、request_cancel(task_id)、结构化event_sink。ROSTaskApi只负责Action传输/反馈/结果；逻辑不能依赖rclpy。

## 探索地图

优先复用现有ElevationPipeline、ExplorationMapProjector、PolicyMapExporter等纯地图组件，独立map-only节点不实例化IncrementalNavigationNode/规划session/action。输出/Car/T4/mapping/exploration_map OccupancyGrid与可配置GetPolicyMap服务。分辨率沿用原探索/DRL配置，独立于局部1cm；持久地图保留已累计区域，不把当前5m窗口充当全局探索图。

DRL分块快照保留已用的epoch/revision/base/full、geometry/frame、observed、intrinsic_states、navigation states、cost/elevation与当前实测anchor、真实起点连接。保留真实来源与已观测语义，既有全量/增量断链恢复可复用。新模式禁用UNKNOWN起步补全，不伪造assumed_start/stationary_support；起点未知时说明INPUT_UNAVAILABLE。旧GetPolicyMap导出行为保持，增加observed-only导出入口，不能更改旧服务消费者语义。

源图res/yaw/origin变化通过可逆坐标与保守重投影处理，不能只改分辨率标签。可选terrain统计存在时复用原规则，缺失仍允许elevation-only；高程完整性与整车可通过分别表达。只读快照交接，native/map计算不阻塞ROS订阅或服务回调。空/NaN、细薄障碍、局部更新、源分辨率切换与地图版本需有行为测试。APP自有confidence不在本轮晋级范围。

## 前沿与DRL接线

新组合入口显式选择exploration_strategy=none|frontier|drl，默认none；自动模式只有一个选点策略和一个任务Action server，控制器仍需显式启用。前沿复用已知可通行候选、失败记忆、覆盖及完成语义；新增WAITING_FOR_PATH处理与取消后重新评估。全局允许未知不能当作候选已知可达证明，无输入/无路/局部段结束均不能宣布探索完成。

DRL增加navigation_backend=nav1cm，默认incremental_v2。分离地图客户端和导航客户端，nav1cm模式不依赖旧PathReference/TrackingStatus/PlannerDiagnostics去推断任务完成。执行状态来自当前Action反馈与实测里程计，全局Path只估计路程，不作UUID或到达凭据。现有观测schema、图/候选/网络/奖励接口保持，变化分布单独评测，不能宣称旧权重效果自动不变。普通导航失败与基础设施异常继续分别分类，不放宽既有训练完整性规则。

DRL新后端使用已安装公共任务/地图/控制包underlay，旧默认代码路径与正式source manifest不切换；只增加明确可选后端与合成场景短评测。禁止访问或消费论文held-out test来调新接口。已有权重仅只读加载并验证schema/SHA；若没有兼容权重，必须明确记录该项未验收，不能用随机策略替代后宣称已有权重验证完成。

## 验收

Task1实际Humble Action客户端覆盖到达/朝向/取消/替换/无效/等待/旧UUID与清理；Task2真实native地图导出与旧导出回归、任意正源res/0stampTF及增量对照。Task3真实前沿→任务Action→真实planner→标准Path→真实MPPI→独立plant，覆盖选点、到达观测、受阻等待/重选、取消和完成语义。Task4固定已有权重在合成短场景用新后端推理、当前Action/观测/约束/退出可追溯，无训练优化器启动。所有运行安装/source/native身份与原始记录绑定，禁止只靠可视化或Action SUCCEEDED证明规划成功；正式全局成功仍需0/PLAN_FOUND/has_reference。

本轮不进行Orin/实车、生产DDS/真实P3、足式或正式训练部署。若集成暴露真实产品问题，仅在原任务边界内修复并重验受影响项；不把临时脚本/构建物/权重提交Git。
