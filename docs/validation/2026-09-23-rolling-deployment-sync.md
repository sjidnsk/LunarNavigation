# 滚动局部规划部署分支同步（2026-09-23）

来源：`feat/rolling-local-planning` 的 `6868236f..b5059eb8`。按用户要求将同任务移动中持续规划同步到当前三个 `app/orin-*` 分支。历史 `release/orin-humble-20260911` 和独立旧架构 `feat/unreal-tcp-exploration-planning-test` 未改；未推送远端、未更新设备。

公共行为：单 worker 计算候选、协调器串行应用；更新无解/超时时保留仍有效的旧路径，取消/地图失效/对齐淘汰迟到结果；同任务正常跟踪可采用新引用，保留无进展和换向/曲率限制。每次计算仍调用全局引导查询并允许缓存复用，本次没有实现独立全局调度。

部署分支已存在命令斜率限制、平台能力加载和地图优化。移植保留这些行为：线/角命令变化以上一条发出命令为基准，真实反馈仍决定进度、偏离和停稳；新增回归同时覆盖开启/关闭滚动时的反馈尖峰。没有从其他部署分支复制整棵目录，未迁入 TCP 到非 TCP 分支。

原 [功能验证记录](2026-09-23-rolling-local-planning.md) 属于来源分支历史证据；本分支的新增验证以本页为准。所有构建/日志位于 `/home/kai/CodexDownloads/lunar_navigation/rolling-deployment-sync-20260923/`。

## 验证条件

Humble 容器 `phase2-humble-gridmap:local`，Release，源只读，网络禁用，独立 build/install；栈上限 32 MiB（继承源功能验证中快照测试所需条件）。测试结果不代表原生 Orin、DDS 联调或真实车体通过。构建复现脚本为 artifact 根目录的 `validate-humble.sh`，参数为源码工作树路径和 `real`/`legged`/`tcp` 标签。

## TCP 仿真分支

目标 `app/orin-unreal-simulation`，移植前 `fec37a1e`。

- OBJ/TCP：`ros2_ws/src/lunar_obj_tcp_sim/config/simulation.yaml` 新增 `rolling_replan_period_s: 0.5`、`rolling_replan_distance_m: 1.0`；`obj_tcp.launch.py` 将周期同时传给导航并派生控制器开关。设为 0 即同时关闭两端。
- P3/TCP 联调模板：`tools/tcp_deployment/templates/debug/p3_joint/navigation.yaml` 明确 0.5 s / 1 m，`controller.yaml` 明确开启。关闭时修改两端配置。原课题三输入、TCP 协议、发送速率、反馈超时、转向对齐与命令转发策略不变。

Humble 11 包构建成功；core 18/18、ROS 13/13、控制器/demo/launch Python 202 项通过（`tcp-humble.log`）。TCP 专项 96 项通过、7 项默认 opt-in 跳过、1 项环境相关测试单独排除（`tcp-loopback.log`）；Jazzy 配置与脚本测试 17 项通过（`tcp-launch-jazzy.log`）。

环境差异：`test_build_places_colcon_log_base_before_build_verb` 写死 `ROS_DISTRO=jazzy`，在仅有 Humble 的容器失败，在本机 Jazzy 通过。保留原失败，不修改无关构建脚本。首次本机宽集测试缺 exploration 消息 overlay，8 项导入失败；完整 Humble overlay 后已解决。

TCP 导航回环闭环：`tcp-loopback-final.log` 为 **1 passed in 75.86 s**。三个目标均返回 `GOAL_REACHED`，共收到 21 条 ACTIVE 引用，移动中出现 17 个不同的后续路径修订；最大命令及反馈线速度约 0.20 m/s。证据：`tcp/loopback-nav-final/test_formal_closed_loop_throug0/receipt.json`。

保留的失败与修正：原每目标 45 s 等待在最终朝向对齐超时；关闭滚动的对照也在第三目标超时（`tcp-loopback-baseline.log`），当时剩余角度约 1.57 rad、角速度 0.1 rad/s。仅把测试每目标等待上限统一为 90 s 后复测通过；未提高控制速度或修改到达条件。`moving_revisions` 按 session/revision 去重，并要求后续修订且线速度大于 0.01 m/s。这是本地 TCP 回环车辆模型证据，不是远程 Unreal 物理证据。

远程 Unreal 物理闭环、目标 Orin、实车：NOT_RUN。没有执行推送或设备部署。
