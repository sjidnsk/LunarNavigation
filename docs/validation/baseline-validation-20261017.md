# 2026-10-17 稳定基线验收

本批次在 ROS 2 Humble、Linux x86_64、CPU 环境完成四目标真实构建和最小启动：最终 **53 包、4 组启动 PASS**。三条应用链路的后续验证已由用户取消：TCP 为 `USER_CANCELLED`（此前三次正式尝试均 `FAIL/RED`），轮式与足式均 `NOT_RUN`。本报告不授予应用完整闭环、硬件部署或策略效果资格。批次编号为 2026-10-17，收尾记录包含 2026-10-18。

## 构建代码与资格

下列 SHA 是实际构建代码对象，均非本报告文档提交的 HEAD。独立固定源码快照、fresh 非 symlink install、安装资源/配置/Python 消息 typesupport/native 库来源已核对；后续两文档提交与各自起点的全部非文档 Git 对象相同，因此不扩大原构建资格。

|长期线|实际构建代码 SHA|包数|构建|最小启动|
|---|---|---:|---|---|
|integration/pure-planner-orin|`16516d12db4e17370cc648b25852d0f61e751fee`|13|PASS|PASS|
|app/orin-unreal-simulation|`c96cbd9d44143ed1e96eb7801536c2ffd42b03cd`|14|PASS|PASS|
|app/orin-real-vehicle|`082c10c6d693eaa1640ba745cfda2a07441bb1ba`|13|PASS|PASS|
|app/orin-legged-vehicle|`eaebd297d0b0515cfe2ecb7ce6f8c697f10bfcd1`|13|PASS|PASS|

两条开发线本批次未独立构建或启动：`dev/terrain-confidence` 起点 `3dd07c33212896f11752f002c08e4d23e7cfb4f3`，`dev/drl-exploration` 起点 `336aea02efd91bf1e950f25ee94a5d77359f4a2c`；不得套用四目标的 PASS。

构建采用 Release、`BUILD_TESTING=OFF`、串行 executor、最多 `-j2`，CUDA 禁用、BLAS/OMP 单线程。实际导航/探索/控制器节点启动，核对正式 launch 参数、action/service/topic 发现和连续存活；未发应用目标。四目标 `GetPolicyMap` 的真实响应均为 `ready=false / INPUT_UNAVAILABLE`，只证明无地图/定位输入时接口存活。

TCP 原代码 `5a825efd63765de696332ba93fb78d63dcd1f274` 的 14 包构建成功，但探索启动真实失败：loader 拒绝已有可选 `start_blind_zone_margin_m`。最小修复仅兼容并校验该有限非负字段，未知/重复键仍拒绝，未改生产配置或探索几何。针对性回归先 RED 2 fail/2 pass，后 GREEN 4/4，加既有足式几何检查共 6/6；修复代码 C96 的 fresh 重建及最小启动通过。所有尝试合计 67 包次，最终资格仍为 53 包。

## 已取消的应用链路与失败记录

TCP 四次正式尝试均经真实 localhost socket、LoopbackVehicle 和安装后的 operator/ROS launch。下表 fixture SHA 与上表构建 SHA 分开：三种候选只改测试文件，复用 C96 install，均未审、未合入长期线、未纳入本次拟发布六候选。

|正式 TCP 尝试|测试/fixture SHA|实际结果|最大位移 m|细覆盖 / 粗覆盖|
|---|---|---|---:|---|
|原 30m fixture、原点|`c96cbd9d44143ed1e96eb7801536c2ffd42b03cd`|FAIL/RED：初始全覆盖、无非零转向和运动|0|100% / 100%|
|3m fixture、原点|`7b1a2bc579c3d460c02ce4e842011bff134385a1`|FAIL/RED：原 150s 完成等待超时|2.316091|94.0625% / 90.625%|
|4m fixture、原点|`750ba31fdf56158aec3d088757de26b19e4f939d`|FAIL/RED：原粗覆盖 ≥99% 断言失败|2.398762|99.25% / 98.4375%|
|4m fixture、固定起点 (-1.5,-1.5,0)|`116f5526147a65b0800ad9b0fc75c13fc0d5cff6`|USER_CANCELLED：39.24s 中止，pytest rc1|3.105576|90.6875% / 81.25%|

3m、4m 原点、最终取消尝试分别旁录到 118/88/35 次 `cycle_result=PLAN_FOUND`、118/88/35 个非空 `ACTIVE PathReference`，以及 2956/2156/583 条匹配 `TrackingStatus`；这仅支持增量找到路径、局部控制与反馈事实。取消不等于目标到达，探索状态中的局部 `GOAL_REACHED` 不替代完整任务的 `outcome=0 / GOAL_REACHED`。旧 `PlanMotion` 的 `planning_outcome=0`、`reason_code=PLAN_FOUND`、`has_reference=true` 三字段契约本轮 `NOT_RUN`，没有伪造这些字段。

4m 原点正式结果是 63/64 粗格，不能记通过。对实际观测点的离线诊断重建出西南角唯一未知粗格；固定法线/后退采样的 30 个候选均因 footprint 加 clearance 越任务边界被拒绝。**单角粗格的通用采样局限未修复**；诊断重放与固定起点离线预检都不是正式验收通过。原 150s 预算、28 个既有断言、物理阈值与生产默认未为凑通过而修改。

轮式 `app/orin-real-vehicle` 与足式 `app/orin-legged-vehicle` 应用链路均 `NOT_RUN`，没有因最小启动通过而获得闭环资格。三条应用链路总体验收均未通过，用户取消后未继续测试。

## 告警、排除项与后续边界

保留已见告警：planner 的 `builtin_interfaces::msg::Time` 显式构造警告；禁用 Python byte-compilation 的预期 setuptools 警告；有序 SIGINT 的 launch warning。TCP 另有 pytest `Unknown config option: pythonpath`、速度原始坐标轴未标定、地图 `NOT_EXTERNALLY_ALIGNED`；最后取消的 SIGINT 触发真实 `rclpy already-shutdown` 异常及 rc1。进程退出/清理完成不等于应用验收 PASS。

用户排除的旧 8 项 Synthetic/MetadataInvariant 轮式场景在所有分支均排除，未重跑：

- `SyntheticScenarioRunnerTest.ConcaveReachableFrontierPublishesReferenceAndRemainsExecuting`
- `SyntheticScenarioRunnerTest.PointReachableNarrowPassageSurvivesRealWfdAndCandidatePrefilter`
- `SyntheticScenarioRunnerTest.MapContentGrowthReevaluatesFailedCellAndExecutesOneCompleteTrajectory`
- `SyntheticScenarioRunnerTest.PartlyUnknownTaskCompletesBelowOneOnlyAfterEveryCandidateNoPath`
- `SyntheticScenarioRunnerTest.OutsideToInsideUsesProductionApproachThenWfdBeforeExhaustiveCompletion`
- `StampVersionCovariance/MetadataInvariantScenarioTest.MetadataOnlyUpdatePreservesExactActiveActionBehavior/0`、`/1`、`/2`

旧 terrain497/Python33、全仓 pytest/colcon test、`run_obj_tcp_sim.sh` 运行入口、Jazzy、GPU/CUDA、跨机 DDS、真实 UE 物理、P3 实测输入、原生 Orin ARM64、轮式实车驱动、足式步态/硬件制动、DRL 训练及策略效果均 `NOT_RUN`。真实 localhost TCP 的局部运动不能外推到生产 30m 探索、UE、实车或策略效果。

可信度新拟合器、返航/关键帧等长期研究继续暂缓晋级和整支整合；本批次未改动 DRL 训练。六线只追加公共验收文档和入口链接，保留各自生产/native/配置及既有修复，不因共享报告互授资格。

拟发布方式为六条长期线的精确 SHA 原子推送，拟定默认分支 `integration/pure-planner-orin`；此处仅记录计划，不宣称 GitHub 已发布或默认分支已设置。实际发布结果以发布后的独立核验回执为准。
