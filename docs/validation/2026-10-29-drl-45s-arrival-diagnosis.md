# DRL 45秒未到达诊断：局部转向阶段混入最终朝向

任务NAV-1CM-AUTO-20261029-DIAG45，复用父任务NAV-1CM-AUTO-20261029及专题hk/drl-nav1cm-adapter-20261029。诊断基线6a3591bd，公共源码a63afaec；Windows分析原记录，获准kai Humble x86_64台式执行纯组件探针。当前处置：诊断完成，公共局部规划缺陷待修复；产品代码未改，默认入口和正式训练未切换。

## 结论

主要根因在公共局部规划器的模式选择与终点朝向代价：位置尚未到达时，为了对准行进方向进入spin模式，却将最终目标yaw同时放入该纯旋转段的终端代价。中间节点的航向目标和末端节点的最终朝向互相竞争，产生先向一侧转再反转的路径。跟踪器按照有序旋转段执行并保留未完成屏障，后续不断刷新类似旋转段，位置推进被阻断。

原45秒的现象和相同临界状态的受控组件对照一致。现有原记录没有逐周期局部Path/MPPI诊断，所以不声称恢复了原运行每一条局部路径；该根因由源码、原运动时间线、真实安装组件的单变量对照共同支持。它是另一处局部逻辑缺陷，尚未通过产品修复和完整闭环复验。

## 原记录逐层核对

- DRL：只发出一个目标，goal serial始终1，没有策略反复换目标。目标为(0.1,1.9,225°)，起点(0.1,0.1,0°)，直线距离1.8m。
- 全局：44个GLOBAL_RESULT均满足planning_outcome=0、reason_code=PLAN_FOUND、has_reference=true，持续有整体路径。
- 实际运动：最后一次非零平移在约28.20s。之后约16.8s位置维持在(0.08306,1.72503)，主要来回原地转向。
- 45s取消前：位置误差0.1757927705m，yaw误差68.3183°。当次共同节点使用位置0.1m、yaw0.05rad约2.86°的到达容差；两项都不满足。取消后summary中的yaw包含减速期间的后续变化，不能混成45s前的值。
- BUFFER_EXECUTING指存在可执行局部候选，不证明正在靠近导航目标；跟踪器FINAL_ALIGN指当前局部旋转段，也不能当作整个导航已到达位置。

原archive为ART/task4/receipts-final.tar.gz，SHA256 d6c9494eb16851d7ea650de0e7262c8fe8fce077755edf26a8b32e408237f4b4；解析前后保持。原node1、runtime4177行、plant944行及失败信息均未改。

## 具体源码链

公共路径scripts/navigation/one_cm/nav1cm/local_mpcc.py：

1. select_mode第47–61行：偏航误差大于0.3rad进入spin；平移已停止但角速度非零时第58行继续spin。
2. 第140、146–147行：spin的线速度和线加速度固定为0；这类候选只能转向，不能消除剩余位置误差。
3. _parameters第165–166行：只要参考包含最终yaw且参考终点进入时域，就给末节点最终yaw，没有区分“行进对准spin”与“已经到达位置后的最终spin”。第109行其他节点仍承担行进航向代价。
4. 第28s实际快照yaw约101.84°，去目标的行进方向约84.59°，触发对准spin；同段最终yaw却是225°。真实acados候选先从1.777rad降到1.597rad，再升到2.947rad，XY位移为0。其余三个真实快照同样产生带反转的旋转候选。

控制器ros2_ws/src/lunar_pure_wheeled_controller/python/lunar_pure_wheeled_controller：standard_path_reference.py将同XY yaw变化保留成旋转阶段；standard_path_follower.py第82行保留未完成姿态屏障；mppi.py第378–403行对当前阶段端点执行对齐并将平移采样置0。这与上游旋转Path一致，未发现“上游要求前进而跟踪器单独丢失跟踪”的证据。跟踪器滚动屏障逻辑参与了循环表现，不能从本次实验断言所有跟踪情况都已验收。

## 单次受控对照

先用28/30/35/40s四个真实状态逐一检查局部候选，每个状态只比较terminal_yaw有/无。然后从28s实际快照(距目标0.176985m、v=w=0)做一次双组固定12s组件闭环；两组除最终yaw输入外完全相同。

采用原安装MpccPlanner、StandardPathFollower/MPPI，5m×5m、1cm完全已知空图，原轮式车体1.182×0.818m、净空0.2m、v上限0.2m/s、w上限1rad/s、线/角加速度0.5、MPPI响应0.15s、原默认采样参数。每0.1s局部求解，每0.05s跟踪和受限速度积分。没有Actor、ROS节点、消息主题或训练导入。

| 唯一对照变量 | 12s内平移 | 最终位置误差 | 首次进入0.1m范围 | 局部模式 |
|---|---:|---:|---:|---|
| 保留最终yaw输入 | 0m | 0.176985m | 未进入 | spin118次、forward2次；实际平移tick为0 |
| 仅诊断时移除最终yaw输入 | 0.096811m | 0.080189m | 2.75s | spin15次、forward105次 |

保留yaw组在约90°至174°之间摆动，重现原运行的形态；两组均无follower failure。去掉yaw组只证明位置推进恢复，其最终朝向仍不满足原目标，绝不是完整GOAL_REACHED，也不是建议取消朝向需求。对照墙钟约2.44s，不与原DDS/Actor/异步闭环比较性能，不冒充原始地图精确回放。

探针实际加载原C1安装local_mpcc.py，SHA256 daabe7fa2516084f8a08dbf431f60c2bc254eb23f9649c85c2e75c29c60d0e61，与公共专题源码字节一致。仅在新诊断目录生成acados缓存；原产品/安装/权重/配置/账本均未修改。原逐子进程live映射等证据缺口继续保留。

## 后续建议与职责

优先修公共局部规划的阶段边界：行进对准→位置进位→最终朝向调整。位置未达时，对准旋转的目标为行进航向；位置满足后，最终旋转才以目标yaw为约束。最终位置、朝向和实测停稳条件继续保留，净空和动力学约束继续采用原值。具体实现及回归由公共owner integration/pure-planner-orin承担，复用其导航专题；DRL只负责选点，本轮没有需要修改策略网络或切换正式训练的证据。

下一轮修复需先把当前临界状态和大角度终点朝向形成失败回归，再验证位置推进、最终yaw、取消停稳、有序旋转屏障以及真实组合闭环。当前不声称已修好，也不把此对照扩展为实车/Orin/真实P3验收。

## 产物与处置

ART=D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/exploration-integration-20261029/diagnosis-45s-20261029。analysis.json/original-timeline.csv/diagnosis-45s.png为原始记录分析；input.json/probe.py/probe.json为四状态候选探针；closed_probe.py/closed-trace.jsonl/closed-summary.json为唯一双组实验，原日志和退出回执完整保留。

远端同ROOT/exploration-integration-20261029/diagnosis-45s-20261029，独立planner-cache/closed-cache/pycache/cache；本地mpl-cache在D盘。资源保留供后续修复回归，未清理原证据或其它工作树。只提交本诊断文档和原任务登记；未合并、推送、PR、设置upstream或删除。
