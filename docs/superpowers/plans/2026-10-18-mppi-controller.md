# MPPI Controller Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement task-by-task.

**Goal:** 实现并评估可切换MPPI局部运动控制。
**Architecture:** 保留C++局部A*，Python控制包新增纯算法MPPI与地图适配，经现有node输出Twist和TrackingStatus。
**Tech Stack:** ROS2 Humble、Python3.10、NumPy。
**Spec:** ../specs/2026-10-18-mppi-controller-design.md

## Global Constraints
- 默认后端不变；前后上限0.2m/s；不得改正式评测/权重/历史输出。
- 公共到达0.10m/0.05rad且实测停稳；unknown只允许服务明确起点连接。
- 不合并、推送、提交或部署到正式服务。

## Tasks
- [x] 地图认证：先写薄障碍、unknown、边界净空、负tile、epoch/delta失败测试；实现mppi_map.py（GridMap、PolicyMapCache），含线段及margin认证。
- [x] MPPI：先写直行/倒车、边墙偏角、停止/未知、终点与限制测试；实现mppi.py（MppiPolicy、MppiExecutor），控制序列采样/softmax更新/最终重认证/停车认证。
- [x] ROS集成：先写无地图停车与有效地图移动测试；node.py新增controller_backend与policy_map_service；包增加NumPy依赖与测试注册，默认路径不变。
- [x] 评估：scripts/navigation/evaluate_mppi.py可复现实验，保存A/B原始轨迹和控制耗时。运行相关包全套Humble测试与ROS闭环参考/真实地图对照。
- [x] 完成：检查diff、任务登记记录实际验证边界和未完成事项，完整报告与可视化。

## Review Focus
- 加权后候选重新认证；薄障碍不能因离散步长漏过。
- 起点连接仅unknown且单向退出；delta不连续不得沿用。
- session/地图修订变化清热启动；停止失效不可重启旧参考。
- 正倒向相同限制、实测初速度与延迟、完整停车。
- ROS关联与计算预算需实测，不用纯算法推断硬件可用。

## Implementation and evaluation record
- Implemented in `feat/mppi-controller-20261018`; uncommitted, no production cutover.
- Defaults: 32 samples, 40 steps × 0.05 s, one iteration; forward/reverse 0.2 m/s.
- Independent review closed continuous assumed-prefix crossing, processed-map freshness, actual publication history, source-age/TF pairing, progress across reference updates, and complete delayed braking tail.
- Final raw results/report live outside Git: `/mnt/data/WS/lunar-navigation/analysis-artifacts/mppi-controller-20261018/`.
- Full actor 200-case acceptance and Orin/hardware remain NOT_RUN. Current paired synthetic checks do not establish production readiness.
- Evaluation completed with negative ROS readiness result: fixed-reference MPPI 1/6 versus baseline 5/6; native two single-action smokes both pass. Implementation/evaluation scope complete; production tuning/acceptance remains explicitly open.
