# 滚动局部规划部署分支同步（2026-09-23）

来源：`feat/rolling-local-planning` 的 `6868236f..b5059eb8`。按用户要求将同任务移动中持续规划同步到当前三个 `app/orin-*` 分支。历史 `release/orin-humble-20260911` 和独立旧架构 `feat/unreal-tcp-exploration-planning-test` 未改；未推送远端、未更新设备。

公共行为：单 worker 计算候选、协调器串行应用；更新无解/超时时保留仍有效的旧路径，取消/地图失效/对齐淘汰迟到结果；同任务正常跟踪可采用新引用，保留无进展和换向/曲率限制。每次计算仍调用全局引导查询并允许缓存复用，本次没有实现独立全局调度。

部署分支已存在命令斜率限制、平台能力加载和地图优化。移植保留这些行为：线/角命令变化以上一条发出命令为基准，真实反馈仍决定进度、偏离和停稳；新增回归同时覆盖开启/关闭滚动时的反馈尖峰。没有从其他部署分支复制整棵目录，未迁入 TCP 到非 TCP 分支。

原 [功能验证记录](2026-09-23-rolling-local-planning.md) 属于来源分支历史证据；本分支的新增验证以本页为准。所有构建/日志位于 `/home/kai/CodexDownloads/lunar_navigation/rolling-deployment-sync-20260923/`。

## 验证条件

Humble 容器 `phase2-humble-gridmap:local`，Release，源只读，网络禁用，独立 build/install；栈上限 32 MiB（继承源功能验证中快照测试所需条件）。测试结果不代表原生 Orin、DDS 联调或真实车体通过。构建复现脚本为 artifact 根目录的 `validate-humble.sh`，参数为源码工作树路径和 `real`/`legged`/`tcp` 标签。

## 足式分支

目标 `app/orin-legged-vehicle`，移植前 `dfa3d819`。

- 同步公共候选计算与轮式滚动支持，保留足式能力、5 cm 映射配置和该分支地形算法。
- 当前足式配置 `rolling_replan_period_s: 0.0`、`enable_rolling_updates: false`。本次源功能只支持轮式；不宣称已经实现足式移动中换路。
- 若本分支选择轮式平台，需要配套使用轮式配置与正周期；不要给足式配置强行设置正周期，节点会明确拒绝。

Humble 5 包构建成功；core 19/19、ROS 13/13、Python 224 项通过（`legged-humble.log`）。Jazzy 配置/部署 launch 测试 25 项通过（`legged-launch.log`）。以上针对已提交基线加本次移植的干净树；原工作树中正在进行的搜索优化未纳入提交，也不属于此次验证范围。

落地时使用已验证补丁，工作树和索引分别应用；索引只含本次改动。提交前后核对原有未提交文件和改动内容，保留其编辑状态，记录见 `legged-preservation.json`。
