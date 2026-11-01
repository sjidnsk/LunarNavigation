# 方案 A 实施、验证与 P4 切换

任务 GLOBAL-ROUTE-TERMINAL-20261101；owner integration/pure-planner-orin；专题 hk/global-route-terminal-20261101，基线 adbc1d0。已验证并部署源码 7d7b5a7，尚未合入长期线、未推送。用户选择方案 A，并于 2026-11-01 明确授权“切换P4”。

## 实施边界

- 全局仍用原分解搜索；记录完整 portal，在选定通道内用漏斗拉直，候选必须不长于旧路线并通过同一碰撞与范围检查；失败回退旧可行路线。清障直连避免无用构图，包围盒筛选保持闭边界碰撞语义。不是跨通道最短保证。
- 终点整形复用现有 De Casteljau 细分和连续几何证明，增加有界三次 Bezier 连接；准确终点保持不变，切线相容最终朝向模 pi，等转角时优先显式目标朝向。无空间或预算不足回退。最终几何可能为朝向连接而比纯位置直线略长。
- 全局只产几何路线，局部 MPCC 负责运动参考；本次 TCP 继续使用既有 mpcc_cmd_vel 入口。未引入新阶段、身份或门禁。
- 完成判定修复两处证据生命周期：NOT_DUE 不再清除仍有效的到达保持；低频 P3 新观测保留到下一次有当前规划证据的周期使用。NOT_DUE 分支本身不能完成任务。真实离开、运动、证据失效仍清除。实测与当前估计两侧检查保留，防止历史停稳帧将当前运动误报完成。
- 在线默认朝向容差统一 5 度，包括标准 Path 控制器和 launch。旧非标准控制器默认及显式覆盖不变；位置验收 10 cm、位置优化目标 2 cm、停稳阈值和源时间保持均不改。

## 独立改动切片

| 提交 | 内容 |
|---|---|
| 84537f4 | 规划及合同默认 5 度 |
| aa8ebb8、aea8ab3 | 通道漏斗、直连与重复认证消除 |
| 87abf72、04c7e0d | 终点平滑和 90 度等转角处理 |
| ddf1090 | 完成证据生命周期 |
| 56a49cc | 真实 MPCC 集成回归 |
| 7d7b5a7 | 标准 Path 与 launch 默认一致 |

各实现批已独立复审。保留原始 RED 及修复后日志，不用历史通过替代本轮验证。

## 验证

| 层级 | 结果 |
|---|---|
| Windows 完整导航及相关 TCP/launch | 1211 passed、91 skipped |
| 隔离 Humble 完整原生导航 | 1093 passed、25 skipped |
| 最终 Humble planner 包 | 115 passed |
| 最终到达核心回归 | 46 passed |
| TCP/launch 入口 | 183 passed |
| 最终 Humble controller 包 | 701 passed |
| 真实 acados 终点几何到 MPCC 及完整认证 | 5 passed |
| 实际车辆/新目标自然到达 | NOT_RUN，等待用户 RViz 操作 |

以上集合有重叠，不能相加。源码清单 287 项、原生夹具 36 项及已安装变更产品 10 项已核对哈希。最初缺失原生夹具和脚本导致的失败日志原样保留，补齐后重跑通过。验证目录 /home/yanfa/P4/artifacts/global-route-A-20261101；本地日志与图在 D:/CodexDownloads/lunar_navigation。

同进程对照：单障碍路线 12.325840→11.097133 m；旋转场景 12.905594→11.097133 m。1000 个远障碍直连场景图节点 8002→0，查询中位 12.9527→0.8803 ms；小单障碍场景中位 .1796→.2563 ms，不能宣称所有场景更快。4000 次碰撞对照结果一致。终点平滑示例 4 m 直线变为 4.30589 m 几何连接；不是实车轨迹。

## 2026-11-01 实际切换

- 只写 /home/yanfa/P4。旧 10 个文件备份至 /home/yanfa/P4/run/global-route-A-7d7b5a7-20261101/backup/source；原运行状态和首次停止记录同目录保留。
- 旧导航与 RViz 退出。退出回执未及时确认停稳，未覆写该证据；随后独立读取 156 条实时车辆遥测，末值 vx≈-8.33e-11、vy≈4.25e-10 m/s、w≈-8.75e-8 rad/s，确认停车后续接部署。
- 新运行 /home/yanfa/P4/run/global-route-A-7d7b5a7-20261101；operator 176765，navigation launch 176795，实际 navigation_task_node 176810，RViz 176796。新规划器前缀 /home/yanfa/P4/artifacts/global-route-A-20261101/install/lunar_nav1cm_planner。
- 原车辆接口 PID 156544、P3 fusion PID 156548、采集 PID 156545 均不变。只读取 P3/Env_X；6665 与 6668 各一条既有 TCP 连接。
- 域 10，P3 输入域 57；地图和定位计数持续增长，RViz 已加载地图。ros2 param get 实际返回 goal_yaw_tolerance_rad=0.08726646259971647。
- 既有平台限速 .2 m/s、.1 rad/s，持令 1.2 s、全局 15 s、地图断档门槛 0。未重放旧目标，未自动发送新目标，切换后等待用户操作。
- 最终回执为 artifacts/global-route-A-20261101/deployment.json 和新运行 readiness.json。首次失败回执保存在 backup/initial-switch-attempt.json；隔离验证回执 production_switched=false 是切换前记录，保留不改。

剩余：用户已暂缓的终点微小残余 MODE 拒绝、切换后的真实行驶平顺性和自然完成尚未复测；Humble 调试机结果不能证明 Orin/实体实车。专题已提交并部署，长期线合入和工作树回收待后续明确处理，不自动合并或删除。
