# P3延迟输入与无控制规划联调计划

> **For agentic workers:** 使用 superpowers:executing-plans；实现单写入，独立只读审查。

**Goal:** 适配实测P3输入时序并验证导航/标准前沿探索规划，不启动车辆控制。
**Architecture:** 复用现有map/odom/TF及任务接口；全局路线、局部参考、探索目标职责保持。源时间、接收时间、计算时间分别保留，不把延迟定位称作当前可执行状态。
**Tech Stack:** ROS2 Humble ARM，现有nav1cm/MPCC/前沿探索，Python/C++。
**Spec:** 用户2026-10-30“暂不开车辆控制…先适配P3地图与定位延迟”。

## 约束

只新P4写入，P3只读；不启跟踪器/TCP车辆桥，不发控制或伪造到达/反馈。导航/探索调试接口隔离前缀，实际P3源只订阅。原控制超时保持，未知/覆盖/碰撞检查保持。设计仅本地。

## 步骤

- [x] 1. 测量45秒P3 source age、接收gap、source推进、map/odom/TF相对时间、QoS及map->odom变化；审查实际消费链。
- [x] 2. 根据证据选择最小适配，先写失败回归。只读规划必须能拒绝断流、无效时间与缺失TF，不能把历史状态无限当当前；无必要不扩接口。
- [x] 3. P4隔离构建与适用测试，启动controller=false组合；人工目标规划成功需outcome0/PLAN_FOUND/has_reference，探索验证真实前沿/目标/Action交接与有界取消，不能伪造自然到达。
- [x] 4. RViz显示规划/前沿结果，独立审查并本地提交，记录实际边界与后续待办，不自动合入/push。

执行结果：完成测量、最小修复、适用测试与真实无控制联调；步骤3执行完成但完整功能验收未通过（局部认证/探索目标选择阻塞）。新TCP UNKNOWN结果另属tcp2，详见docs/validation/2026-10-30-p3-delayed-input-planning.md。
