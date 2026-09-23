# 滚动局部规划部署分支同步（2026-09-23）

来源：`feat/rolling-local-planning` 的 `6868236f..b5059eb8`。按用户要求将同任务移动中持续规划同步到当前三个 `app/orin-*` 分支。历史 `release/orin-humble-20260911` 和独立旧架构 `feat/unreal-tcp-exploration-planning-test` 未改；未推送远端、未更新设备。

公共行为：单 worker 计算候选、协调器串行应用；更新无解/超时时保留仍有效的旧路径，取消/地图失效/对齐淘汰迟到结果；同任务正常跟踪可采用新引用，保留无进展和换向/曲率限制。每次计算仍调用全局引导查询并允许缓存复用，本次没有实现独立全局调度。

部署分支已存在命令斜率限制、平台能力加载和地图优化。移植保留这些行为：线/角命令变化以上一条发出命令为基准，真实反馈仍决定进度、偏离和停稳；新增回归同时覆盖开启/关闭滚动时的反馈尖峰。没有从其他部署分支复制整棵目录，未迁入 TCP 到非 TCP 分支。

原 [功能验证记录](2026-09-23-rolling-local-planning.md) 属于来源分支历史证据；本分支的新增验证以本页为准。所有构建/日志位于 `/home/kai/CodexDownloads/lunar_navigation/rolling-deployment-sync-20260923/`。

## 验证条件

Humble 容器 `phase2-humble-gridmap:local`，Release，源只读，网络禁用，独立 build/install；栈上限 32 MiB（继承源功能验证中快照测试所需条件）。测试结果不代表原生 Orin、DDS 联调或真实车体通过。构建复现脚本为 artifact 根目录的 `validate-humble.sh`，参数为源码工作树路径和 `real`/`legged`/`tcp` 标签。

## 实车分支

目标 `app/orin-real-vehicle`，移植前 `d3b95a5a`。

- `config/exploration_navigation.yaml`：默认周期 0.5 s、提前距离 1 m、执行反馈开启。
- `config/orin_controller.yaml`：默认 `enable_rolling_updates: true`。关闭时同时将导航周期设为 0、控制器开关设为 false；控制器 launch 可传 `enable_rolling_updates:=false`。
- 足式适配器仍仅执行原分段语义：节点按 `platform_type` 限定轮式滚动，`orin_legged_controller.yaml` 明确 false。
- 保留 `RealSessionPorts(capability, profile)` 与平台化控制参数、默认执行确认，不回退为来源分支的较早接口。

Humble 5 包构建成功；core 18/18、ROS 13/13、Python 224 项通过（`real-humble.log`）。Jazzy 配置/部署 launch 测试 25 项通过（`real-launch.log`）。未启动实车控制器，未改 TCP 或设备文件。

后续足式默认开启和公共运行时验证见 [足式默认开启验证](2026-09-23-legged-rolling-default.md)。
