> 历史分支记录：`feat/drl-exploration-training` @ `c6473ae70eb4`。本文运行状态、接口和实验结论是当时快照；当前开发以 [统一入口](../DRL开发入口.md) 为准。

# 洞穴提前闭环检查（2026-09-09）

结论：未通过“洞穴探索可以完成”的检查。已完成动作的规划/跟踪正常，但出现局部重复观测；覆盖面积排除和耗尽定义也不适配当前洞穴几何。没有改动或中断持续训练。

## 环境和隔离

- 工作树：`feat/drl-exploration-training`，基线 `de54573` 加现有未提交实现。本次仅添加此验证记录。
- 本机 Jazzy；先加载 `~/.cache/lunar-drl-training/jazzy/install`，再加载 `~/.cache/lunar-known-space/jazzy/install`。
- 单个额外环境，DDS domain 190；持续训练使用 80–87。CPU 推理、算子线程 1；不使用评估 GPU。
- 原有导航器/控制器，运动上限 0.2 m/s、目标仿真倍率 10、10 m/90° 观测与遮挡均保持原配置。
- 评估调用 `evaluate(..., scenes=[...], write_metrics=False)`；不获取训练写锁，不写训练回放或检查点，不复制大模型。已保存 `resume.pt` 只读加载并冻结 Actor，不能视为实时训练 Actor。
- 场景：`cave_single_level`，seed `3000000001`，extent 50 m，start_index 0，生成器 v2。100 m 同种子仅做几何/覆盖模块审计，未做闭环。

## 实际闭环结果

| 策略 | 已完成动作 | 有效/原始覆盖率 | 路程 | 碰撞 | 结束性质 |
|---|---:|---:|---:|---:|---|
| 同图动作域 greedy | 128 | 5.1243% | 24.666 m | 0 | EVALUATION_TRUNCATED，预算 128，未完成任务 |
| 已保存 Actor，确定性推理 | 11 | 5.2078% | 77.428 m | 0 | 诊断复现后主动结束评估，不是自然任务结束 |

所有以上已完成动作均记录 `navigation_outcome=0`、`planning_cycle_result=PLAN_FOUND`、`has_reference=true`、`reason=GOAL_REACHED`。不由这些局部目标成功推断整张地图可达。

Greedy 的 128 动作中 120 个新增面积为零，最终只有 105.56 m² 观测。Actor 最后完整动作观测 107.28 m²，出现两个局部点之间往返。两者排除面积始终为零。Greedy 最终耗尽检查为 `INDETERMINATE_OPEN_MAP`。

Actor 原设置 128 动作；发现独立的覆盖语义问题并复现局部往返后，向唯一评估 PID 223628 发送 SIGINT，保留全部已完成动作证据。此项不能用于计算完整任务成功率；不是训练采样截断。持续训练 PID 221232 未收到信号。

## 覆盖语义的独立审计

以 0.2 m 真值栅格，计算“所有非不透明格 + 它们的八邻域首层障碍”的并集。该集合故意包含不连通空腔、忽略平台可达性、距离与朝向限制，因此只是当前首命中遮挡模型的乐观原始可观测面积上界，不是精确可探索面积。

| 任务尺度 | 任务面积 | 乐观可观测上界 | 原始覆盖率上界 | 乐观全观测输入下排除面积 | 耗尽检查 |
|---|---:|---:|---:|---:|---|
| 50 m | 2060 m² | 344.36 m² | 16.7165% | 0 m² | INDETERMINATE_OPEN_MAP |
| 100 m | 8240 m² | 937.72 m² | 11.3801% | 0 m² | INDETERMINATE_OPEN_MAP |

将该乐观并集仅在独立 Python 审计中聚合成与 PolicyMapStore 相同的 1 m 观测/阻塞比例，调用现有 `estimate_exclusion` 和 `check_exhaustion`。真值没有进入策略、ROS 导航、训练奖励或回放。

源代码解释：

1. `scene.py` 的任务区域是矩形，但洞穴空腔/通道只占其中一部分；其余大部分是不透明岩体内部。
2. `visibility.py` 只观测射线首个障碍格，不能把障碍后面的岩体内部标记为已观测。
3. `maps.py:estimate_exclusion` 要求 1 m 粗格内全部细格都已观测且 BLOCKED 才作为分隔障碍，只排除被封闭的未知分量，并始终保留连通地图外缘的未知区域。
4. `exhaustion.py` 同样使用全观测阻塞粗格作分隔，未知乐观可通行；一格薄的可见洞壁难以构成这种粗分隔，外侧未知岩体仍连通开放边界。

所以本轮证据不能支持“多训练一些就能达到 95%”。需要先使可探索面积、未知区域排除和耗尽判定共同适配洞穴首表面观测；同时单独排查观测动作收益估计与策略的局部重复。不能通过把岩体内部伪造为观测、降低目标或把预算截断计成功解决。

## 持续训练与产物

检查过程中同一 run_id `526cad1db0e74ca48c7451c58444fcfe` 的训练持续增长，记录时已到 13383 条转移、2399 次更新。原模型、经验、256 动作训练预算和 30 分钟保存配置均未更改。

小型复现脚本、每动作标量日志、JSON 汇总在 `/tmp/lunar-cave-check/`，未保存地图、轨迹视频或模型副本；未提交。复现入口：加载上述两个 Jazzy overlay、设置源码包 PYTHONPATH 后，使用训练 venv 运行 `run.py greedy 50 128` 或 `run.py learned 50 128`。`geometry.py` 为无 ROS 的独立审计。

范围：本机 Jazzy 检查；Humble、Orin、实车、其余种子及大尺度闭环均 NOT_RUN。没有修改实现、合并、提交或推送。

## 评估退出异常（本轮追加）

向评估父进程发送 SIGINT 后，`Worker.close` 对有未完成命令的 worker 使用协作 SIGTERM；`RosExplorationEnv._finish_navigation_before_close` 中 `cancel_goal_async()` 等待超时，报 `ROS request did not complete`，继而父进程报 `environment 0 close failed (1)`，退出码 1。不是正常评估结束；没有生成 learned-50.json，已完成动作仅以 learned-50.log 和 summary.json 为证。

取消请求为何超时尚未定位；不能归因于洞穴规划失败，也不能宣称退出路径通过。`RosExplorationEnv.close` 的 finally 仍执行子进程回收。逐个读取 /proc 进程环境确认 DDS domain 190 无残留；训练 PID 221232 仍存活。本次只记录该失败，未修改运行中代码。
