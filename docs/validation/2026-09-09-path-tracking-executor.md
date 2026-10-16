> 历史分支记录：`feat/drl-exploration-training` @ `c6473ae70eb4`。本文运行状态、接口和实验结论是当时快照；当前开发以 [统一入口](../DRL开发入口.md) 为准。

# 路径跟踪执行器重构验证

> 后续导航重构已移除本记录中的独立前沿选点与重复端点补丁；旧E7目标现在按未知目标拒绝。
> 控制器实现保持不变，当前导航契约与测试见[已知空间导航验证](2026-09-09-known-space-navigation.md)。


日期：2026-09-09。工作树：`feat/drl-exploration-training`，基于 `a1cb8bf`。
本文记录本次实际验证，不以历史测试替代当前实现证据。

## 实现

- `path_geometry.py`：不可变折线、线段投影、弧长前视和转角识别，消除稀疏顶点误判。
- `execution.py`：独立的有状态执行器；制动、对齐、正/反向跟踪、终点转向、停稳完成/失败。
- `limits.py`：联合计算平移及角速度的可行加速度区间，保持目标曲率。允许边前进/后退边转弯，`vy=0`。
- ROS 适配器保持缺失/陈旧 odometry、TF 停止规则；平台能力由 `wheel.yaml` 限制，平移上限0.2 m/s。
- 训练改用带 session/revision 的 `PathReference`；`TrackingStatus` 携带执行状态和实测速度。
  导航只接受当前参考的停稳 FAILED/COMPLETED；前者重规划，后者切段或确认最终成功。
- 非最终段终点朝向也执行。重复的已完成局部目标、无运动的非最终参考返回 `NO_PATH / NO_LOCAL_PROGRESS`。
- 训练 watchdog 改为 `simulation.no_progress_watchdog_s=120`，按执行进展刷新；不会仅因完整目标超过120秒取消。
  此语义由确定性测试验证；本次没有执行超过120秒的完整目标验收。

## Jazzy 真实闭环

`RUN_DRL_ROS=1 .../venv/bin/python -m pytest -q -s tests/drl/test_ros_closed_loop.py`：
**4 passed，106.97秒**。导航、控制器均为实际ROS节点；模拟器使用真实MotionPlant动力学及场景碰撞。

| 场景 | 本次结果 |
| --- | --- |
| seed7 前进1m目标 | GOAL_REACHED，实走0.835m，新增7.44m²，segment_revision=1 |
| 同位置转向 | GOAL_REACHED，平移0m，新增2.40m²，segment_revision=1 |
| 途中覆盖达标 | COVERAGE_REACHED，取消导航且停稳，实走0.1725m |
| 范围内不可达目标 | NO_PATH / NO_LOCAL_PROGRESS，实走2.4647m，新增17.04m²，未碰撞、未截断 |
| E7 原种子1201697069、原第三个连续目标 | 起步先转向；实际走4.0056m，未碰撞，观测面积323.40m² |

E7 是显式限定4m的原目标回归，跨过原1.3575m碰撞点；到达回归距离后关闭隔离实例，
**不宣称该约37m目标整体GOAL_REACHED**。另有无DDS的原始8m以上参考回放，使用相同种子岩石真值碰撞判定。
测试保留原有NO_PATH、观测增益、碰撞与截断断言，没有把取消结果当作不可达通过。

## 包和源码验证

| 验证层级 | 结果 |
| --- | --- |
| Python：DRL、控制器、接口 | 403 passed，4 skipped，21.15秒；4项opt-in真实ROS测试已在上一节单独执行 |
| Jazzy增量导航core | 15/15 CTest通过 |
| Jazzy增量导航ROS | 13/13 CTest通过 |
| Jazzy控制器 | 5/5 CTest通过 |
| Humble容器构建 | 10包成功，独立build/install |
| Humble增量导航core / ROS / 控制器 | 15/15、13/13、5/5 CTest通过 |
| Humble legacy规划器ROS | 19/19 CTest通过 |

Jazzy扩大检查的legacy规划器ROS首轮为18/19：
`PurePlanMotionServer.PublishesImmediatePhaseChangesAndThrottlesRepeatedPhaseFeedback`
等待4份Action反馈超时。其源码和测试本次未修改；单独重跑整个`pure_plan_motion_server_test`
通过（63个gtest用例，23.86秒），其余18项首次均通过。记录该时序不稳定性，不声称首轮全绿。
新增测试覆盖正反向弯道、速度/曲率/加速度联合限制、尖角、停稳完成、最终/非最终转向、
停滞、长线段、旧revision/失效/旧session、错误身份或未停稳反馈不得重规划，以及已完成目标不得循环重发。

构建目录分别为项目外 `~/.cache/lunar-drl-training/jazzy` 和 `.../humble`；
Humble使用Ubuntu22.04镜像 `phase2-humble-gridmap:local`，只读挂载源码。CTest按包串行执行。
构建中的既有Jazzy服务QoS弃用警告和legacy Pose初始化警告未作为通过依据。

## 边界与产物

- 本次未运行原生Orin、生产DDS/课题三实时输入、实车：**NOT_RUN**。
- 没有在控制器中使用真值地图、添加绕障规划器或接入车辆命令。真实闭环只使用`/lunar_training/*`；
  原有控制器单元测试在隔离DDS domain183/184运行。
- 长时间探索可靠性、任意地形的碰撞保证、策略收敛及相对前沿方法的优势：**NOT_RUN**。
- 正常跟踪约束速度和曲率；停止/失效输出零目标速度，由底层物理制动实现实际减速。
  本次MotionPlant验证不能替代底盘真实加减速和轮舵切换延迟验证。
- 用户旧运行 `eea0693051dc4b4b90849ba411a11876` 未改写：`resume.pt` 739552649字节，
  `metrics.jsonl` 52271字节，修改时间均保持9月8日23:53。未新建训练运行、重启训练或执行`--new-run`。
- 8环境、30分钟保存、项目内训练目录及保留策略保持原样。源码、测试、文档之外的验证产物均位于外部缓存，未纳入提交。
