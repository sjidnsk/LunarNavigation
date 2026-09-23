# 足式持续规划默认开启（2026-09-23）

用户要求足式也默认开启移动中持续规划。本次适用于 `app/orin-legged-vehicle` 和同样带足式配置/启动入口的 `app/orin-real-vehicle`。TCP 当前轮式入口已开启，无需改变；历史 release 和只读 import 不变。未推送远端或部署设备。

## 行为

- 导航启用 tracking feedback 时，轮式/足式均默认 `rolling_replan_period_s=0.5`、`rolling_replan_distance_m=1.0`。移除仅轮式限制，保留跟踪反馈要求。
- 足式控制器配置默认 `enable_rolling_updates: true`；`incremental_reference` 两种平台均使用现有同会话修订接续逻辑。平台能力、速度/加速度限制及原有到达条件未改。
- 公共 `ComputeCycle` 已按能力分派 wheel/legged 规划器；继续使用单 worker、版本校验及有效旧路径保留机制，不另建足式调度器。
- 分开启动两端时，回退需同时设置导航周期为 0 和控制器 `enable_rolling_updates: false`。足式仍是现有平面 vx/omega 执行模式，不包含步态控制或真实驱动接入。

## 验证

基线：足式 `77982491`，实车 `6acb99b8`，均包含用户已提交的搜索优化，本次未修改搜索算法。独立 Humble 容器 `phase2-humble-gridmap:local`，Release、32 MiB 栈、只读源码挂载。

| 分支 | 构建 | core CTest | ROS CTest | Python |
|---|---|---|---|---|
| 足式 | 5 包成功 | 21/21 | 13/13 | 227 passed |
| 实车 | 5 包成功 | 20/20 | 13/13 | 227 passed |

新增 ROS 回归采用足式能力和规划端口替身，验证当前位置从 0.5 m 前进到 0.7 m、尚未到达目标时，发布 ACTIVE revision 2 并从当前位置起算；这证明足式调度/发布链路，不是物理闭环。新增控制器测试使用足式能力配置，验证默认开启、显式关闭，以及移动中接续后的 TRACKING 和正速度；配置测试验证部署两端默认开启。

红绿验证：修改前导航测试失败于默认周期为 0 和仅轮式异常，控制器测试失败于足式被强制关闭。控制器测试首次接续场景未先提供静止启动状态，停在既有 BRAKING 阶段；补上初始静止反馈后通过，未改制动逻辑。足式首次完整绿测在既有 `OdometryAndTfUseDeploymentBestEffortDepthTen` 用例处 60 s 超时；未改该用例或放宽阈值，完整复跑通过。该超时根因未确定，保留首次日志，不据此宣称无偶发问题。

证据根目录：`/home/kai/CodexDownloads/lunar_navigation/rolling-deployment-sync-20260923/`。
- `legged-default-red.log`：导航修改前失败。
- `legged-default-green.log`：首次 QoS 用例超时。
- `legged-default-recheck.log`：足式完整通过。
- `real-legged-default-green.log`：实车分支完整通过。

源码 UTF-8/Python 语法和 `git diff --check` 已检查。原生 Orin、真实足式驱动/步态、远程 Unreal 物理闭环：**NOT_RUN**。
