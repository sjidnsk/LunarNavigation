# 全局通道与终点优化实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 保留分解搜索，通过通道拉直缩短冗长路线，平滑终点几何连接并减少重复计算，复审完成门槛。
**Architecture:** 全局搜索/漏斗只产几何路线；现有Bezier整形承担连续几何证明；MPCC负责动态可行性与速度控制；任务仅消费既有完成证据。无新ROS实体或门禁。
**Tech Stack:** Python/NumPy，pytest，ROS2 Humble隔离构建。
**Spec:** docs/superpowers/specs/2026-11-01-global-route-terminal-optimization.md（用户已选A）。

## Global Constraints

- 复用hk/global-route-terminal-20261101，基线bcdfbf1，owner integration/pure-planner-orin；ROOT唯一整合/远端入口。
- 保持5度、.1m位置验收、.02m优化目标、既有停稳/能力/净空/碰撞语义。无新超时、累计器、任务实体或ROS消息。
- 只优化所选自由通道，不宣称跨通道严格最短；路径不得变长、穿障或超域。
- 终点曲线不替局部选择运动方向；最终切线允许yaw或yaw+pi，剩余姿态仍可原地完成。
- P3/Env_X只读；运行安装不切换、不自动发目标、不merge/push/删工作树。

## 批1：全局通道拉直与确定性提速

- [x] global_planner.py保留当前图搜索；内部记录每个portal完整区间和两侧单元，按重建路径有序定向。
- [x] 先RED单障碍12.33m冗长、镜像/竖直/窄口/回折等案例；当前单通道候选漏斗拉直，经同physical障碍连续认证且不长于既有简化路径才接收。
- [x] 清障直连在端点/域检查后早返回，避免1000无关障碍仍建8002节点。保持预算和STORAGE/DOMAIN/GOAL语义。
- [x] 必要时在已有_segment_clear内加入保守包围盒筛选，API和边界接触语义不变，不复制几何工具。
- [x] 运行global/clearance/smoothing关联回归，独立复审，按明确路径提交。

## 批2：终点几何平滑连接

- [x] reference_smoothing.py复用已有曲线细分/连续证明，支持有界末段三次Bezier；terminal_yaw可选，None保持既有行为。
- [x] 起端沿原最后一段接续，末端保持准确goal位置且切线相容最终yaw模pi；选短转角，不制造回环/无谓绕行。曲线有障碍则缩短或回退可行原路。
- [x] coordinator._adopt_route透传既有_goal_yaw；worker/消息/Route公共身份不变。
- [x] 先RED直线90度终点、前进/倒车切线、端点精确、接续切线、障碍/预算回退及goal替换；全程保持同一预算。
- [x] 关联coordinator/async/ROS路径回归与独立复审后提交。

## 批3：完成判定复审及整体资格

- [x] 只读审查实测/预测两状态、.3秒保持、新源帧和停稳阈值，捕获真实状态机反例；没有证据不新增或删除保护，不泛化放宽参数。
- [x] ROOT验证二批衔接、完整导航及控制器相关回归；测量原/新路径长度、节点数量与阶段时间，不混用不同时域计时作倍数。
- [x] 必要源码/配置/测试上传P4新隔离目录，Humble包构建/测试；不写P3/Env_X，不控制车辆。
- [x] 更新任务、审查结果和剩余NOT_RUN；显式路径提交，保留来源。

## Review Focus

- Portal顺序左右定义错误，尤其向左回折和起终点落在分解边界。
- 重叠障碍与边界接触，浮点极短portal不能产生可穿越假通道。
- 最后几何曲线的连续碰撞证据及样条采样折线均需通过；不能仅检离散点。
- 目标yaw改变时旧全局结果/曲线不能被误接，缺空间时保持可行路线而非额外停车门禁。
- 源帧延迟与真实断档不混淆，预测停稳不能单独提交成功。

## 最终执行说明

各批实现及独立复审完成，验证与限制见 docs/validation/2026-11-01-global-route-A-implementation.md。通道拉直候选不比旧路线长；附加终点朝向平滑可能略增长度，以同一碰撞证明约束。用户随后明确“切换P4”，授权覆盖前述暂不切换的阶段边界；2026-11-01 已启用7d7b5a7新安装/RViz，未发送目标，P3/Env_X未改。未自动合入或推送。
