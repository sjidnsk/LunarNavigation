> 历史分支记录：`feat/drl-exploration-training` @ `c6473ae70eb4`。本文运行状态、接口和实验结论是当时快照；当前开发以 [统一入口](../DRL开发入口.md) 为准。

# DRL 探索训练系统验证记录

日期：2026-09-08；源码基线 `df469d1`，设计来源 `a0f190c`；实施分支
`feat/drl-exploration-training`，独立工作树 `drl-exploration-training`。

后续更新：本记录中的运动结果使用修正前 1.5 m/s 平台上限。2026-09-09 已按用户要求统一为
0.2 m/s，并发现原短程验收未覆盖的跟踪缺陷；当前就绪边界以
[运动一致性核对](2026-09-09-drl-motion-consistency.md)为准。下文保留历史实测，不外推到新配置。

实现了连续位姿探索的训练、评估、隔离推理和中断续训系统。终端进度条已经接入实际八环境采集，
每个环境显示原始与有效覆盖，公共行显示采样、更新、吞吐、回放和 30 分钟保存倒计时。
有限 `--max-transitions` 的百分比仅代表本次采样完成比例；无限训练不显示虚构的收敛百分比。
本次验证证明工程链路能运行和更新网络，**不证明策略已经收敛或优于前沿基线**。

## 实现与接口

- 新包 `ros2_ws/src/lunar_drl_exploration`；共享学习器、恰好 8 个 spawn worker、独立 DDS domain 80～87。
- 各回合重建自己拥有的导航与控制进程，使用 `/lunar_training/env_<id>/episode_<epoch>`；不接入车辆话题。
- 唯一导航地图所有者增加默认关闭的 `GetPolicyMap` 服务；Observed 来自同 fine snapshot 的有效原始高程。
- 局部窗口沿用 `BuildLocalPlanningWindow` / 64 m；默认 0.2 m 的 320×320 输入。全局 1 m / 32 m tile 包含任务未知区域。
- 连续相对 SE(2) Actor、多尺度 CNN/空间 token/注意力/GRU256、双 Q 与 target 的 Recurrent SAC；batch8/burn8/unroll16。
- microbatch1 梯度累积后才更新优化器，不改变有效 batch，不裁掉地图。奖励回放保存分项并按当前课程重组。
- 现有前沿节点通过 Action 代理提供可选示范目标，结果由真实导航链返回；没有新建前沿评分算法代替现有节点。
- 在线耗尽检查使用当前整张派生图（含任务外）、乐观连通区及忽略遮挡的量程增益上界；开放地图不判耗尽。
- `--reference` 为可选仿真真值审计；采样位置图检查转向与扫掠路径，观测使用相同扇形/遮挡模型，结果不进入策略或导航。
- 实时推理只装入 Actor，检查实际平台 hash、fine 分辨率和窗口几何；记录执行途中的 Visited。
  新任务与 map epoch 隔离。取消 ACK 不等于停止，等待 Action 终态后才报告目标达成/暂停/取消。

## 环境与资源

本机实查：Ubuntu 24.04，Python 3.12，ROS 2 Jazzy；AMD Ryzen 9 9955HX（16 核 32 线程），
31256 MiB RAM，RTX 5070 Ti Laptop GPU（12227 MiB VRAM），驱动 595.84。
验证期间 `/home` 846 GiB 分区约 265 GiB 可用。此前口述的 RTX 4080 / 32 GiB 显存不是这台机器实查配置。

PyTorch 2.8.0+cu128、Gymnasium 1.2.0 安装到项目外
`~/.cache/lunar-drl-training/venv`；Jazzy 与 Humble 分别使用该目录下的独立 build/install/log。
没有下载地形资产。模型和指标固定在项目内 `training-output/drl-exploration/`，由 Git 忽略。

| 配置 | 值 |
| --- | --- |
| 实例数 / 有效 batch | 8 / 8 个序列 |
| CPU 训练线程 | intra-op8 / inter-op1 |
| 回放内存 | 目标 8 GiB，上限 12 GiB |
| 主存预算 | min(20 GiB，启动可用主存 − 4 GiB)，统计进程树 RSS |
| GPU 预算 | 10 GiB，并留至少 1 GiB 设备空闲 |
| 保存 | 1800 秒单调墙钟；完整更新边界；正常退出与 SIGINT/SIGTERM 额外保存 |
| 存盘回放 | 最多 2 GiB、自包含完整序列片段 |
| 产物及临时写入峰值 | 20 GiB；另留至少 20 GiB 文件系统空闲 |
| 固定文件 | resume.pt、验证选出的 best.pt、至多 8 MiB metrics.jsonl |

## 已执行证据

| 层级 | 结果 |
| --- | --- |
| DRL Python 全集 | `python -m pytest -q tests/drl`：225 passed、3 skipped，19.40 s。3 项为需显式启用的真实 ROS 测试，下行单独执行。 |
| 本机真实 ROS 仿真 | `RUN_DRL_ROS=1 … -m pytest -q -s tests/drl/test_ros_closed_loop.py`：3 passed，50.54 s。 |
| 本机既有接口与控制器 | message/interface/launch 契约及 controller tests：128 passed，38.07 s；补充默认关闭地图服务的接口基线后，相关契约文件10 passed，0.02 s。 |
| Jazzy 构建 | 相关 10 包构建通过，增量构建 15.0 s；最终 Python 包重新安装构建 0.65 s。 |
| Jazzy 包测试 | navigation_core 15/15（最终3.28 s）、navigation_ros 13/13（2.02 s）、controller 4/4（1.97 s），按包串行。 |
| Humble 容器构建 | `phase2-humble-gridmap:local` / Ubuntu22.04 / Python3.10，源挂载只读；10 包构建通过（首次 3m13s，增量 35.6s）；最终 Python 包重新安装构建 0.48 s。 |
| Humble 包测试 | 默认8MiB栈下，navigation_core 15/15（3.31s）、navigation_ros 13/13（1.83s）、controller 4/4（2.03s），按包串行。 |
| 大地图 GPU 更新 | 默认 batch8/burn8/unroll16、1024 个全局 tile，实际 SAC 更新 25.224 s，峰值 allocated 2344.5 MiB / reserved 2416 MiB；100 tile 更新 4.35 s。 |
| 八环境实际采集与更新 | 200 条完成转移，2 次真实 GPU 更新；各环境 [25,26,25,25,26,24,25,24] 条，进度条显示全部 8 环境。 |
| 正式入口热身 | `scripts/drl/train.sh --max-transitions 8`：probe=false，8环境各完成1条正常随机热身转移并保存退出；未达到warmup阈值，更新数为0，不虚报学习。 |
| 重启续训 | 退出后同目录 `--resume --max-transitions 32`，同 run_id，累计 232 条 / 4 次更新，每个新环境再采 4 条。 |
| 真实 SIGINT | 对本次拥有的训练进程15s后发送SIGINT，正常返回0且reason=INTERRUPTED；累计248条/6次更新，各环境再采2条，保存峰值1591125944 bytes。 |
| SIGINT 后恢复 | 使用实际 shell 入口 `scripts/drl/_run.sh probe --resume --max-transitions 32`，继续至280条/8次更新，各环境再采4条；最终检查点经完整load校验通过。 |
| 保存容量 | 首次写入峰值795561617 bytes；最后恢复保存峰值1591128116 bytes，含旧文件和新临时文件。最终固定文件共795568649 bytes；只有resume.pt / metrics.jsonl，没有合格验证结果时不伪造best.pt。 |
| 实时地图契约 | 真实 cave20m / start_index1 导出的 hash=`018d73a5f013c370` 与推理计算一致；fine=.2m，裁剪窗口 51×77，通过 `validate_live_map`。 |
| 参考审计接口 | 同一真实回合调用 `reference_metrics(step=5m, heading=45°)`；返回参考 67.6m²、交集 67.6m²、集合外观测0；仅一个可达采样位置，明确属于粗采样参考，不是洞穴理论可探索面积。 |

八环境 `probe` 明确使用超出活动范围的目标来检验拒绝动作、IPC、梯度和恢复，每次最多两次真实更新。
它的采样速率和覆盖不是有效探索吞吐，也不是学习效果。正式训练没有这两项 probe 覆盖设置。
验证/测试新运行列表为 20/40 个布局，每布局三个固定起点；同布局几何不因起点改变。

在确认最终存档run_id与本次工程测试相符、计数280/8且持有目录写锁后，
已删除本次测试产生的resume.pt和metrics.jsonl。随后正式入口8条热身测试的产物也经独立run_id校验后清除。
训练输出目录无剩余文件，首次正式训练可直接使用train.sh；
没有留下工程probe模型冒充已训练策略。依赖与验证构建保留在项目外专用缓存中，源码审查记录为被忽略的小型文本。

真实闭环的关键记录：

| 场景/动作 | 实际结果 |
| --- | --- |
| seed7，1m 前进目标 | 里程 .813312m，新增7.0m²，Navigate `GOAL_REACHED`。 |
| 同位置45°转向 | 里程0，新增2.44m²，`GOAL_REACHED`。 |
| 区域内已观测障碍目标 | `NO_PATH` / `START_BLOCKED`；保留失败前6.54375m及55.0m²，非终止，独立无效目标惩罚。 |
| 长目标途中设置低覆盖阈值 | .36125m、新增3.24m²后覆盖达标，Action `CANCELED`，停稳后 `terminated=true`，没有双重无效目标惩罚。 |
| 区域外目标 | `INVALID_GOAL`，0里程/0新增，惩罚但不自动终止探索。 |
| reset | epoch/namespace 改变，旧导航与控制 PID 消失，里程和采集器状态重置。 |

移动、转向及途中停止同时核对 `PLAN_FOUND` 反馈、同 goal UUID 的 ACTIVE `PathReference`、segment revision
和最终 Navigate 结果；没有把 Action 通信成功或路径可视化当作完整导航成功。

## 回归修复与存档保护

定向测试覆盖：地图证据不由膨胀生成、首次障碍计面积、封闭区排除可撤销、历史数组不可改写、
负格键和公里级按需地图、循环经验不跨 epoch/episode、正常截断 bootstrap、真实优化器更新、
完整片段回放恢复、30 分钟计时注入、写入峰值、并发写入拒绝、写入失败保留旧文件。
检查点保存前检查 checksum、实际 Torch 读回、网络/温度/Adam 状态、随机状态和回放可恢复性；
缺少优化器状态、非有限值、错误 moment 形状或 step 类型均不会覆盖上一份可恢复状态。

最终审查发现显式 `--new-run` 可能残留上一运行的 `best.pt`，已由 `afb6431` 修复：
取得目录独占锁后、启动 worker 前，先预检再清除三个固定旧产物；拒绝符号链接或目录，保留其他文件。
新增 13 项回归覆盖启动失败、锁冲突、预检失败、旧模型清除及普通续训保留历史。
独立复审 checkpoint/training 58 项通过（17.40 s），最终整分支审查为 APPROVED。

本次完整回归发现原 debug 测试等待了调试地图却没有等待独立 Topic 的 PathReference 到达，
20 次重放复现异步交付竞态；改为等待断言所需的全部消息后，20/20 与整个 ROS 包通过。
服务首次定位尚未到达时也固定返回配置的 map_epoch，避免客户端把启动等待误判为旧回合。

Humble/GCC11 的既有 snapshot fixture 测试在默认8MiB栈上崩溃，而原二进制提高栈限后通过。
反汇编确认 TestBody 固定栈帧约6.75MiB；复用已有 tile 构造 helper 后降至约3.5KiB，
默认栈下目标用例及整个 core 包通过。此修复只改测试夹具，不改生产快照算法。

## 未执行与适用边界

- 长期多种子训练、完整 60/120 回合评估、覆盖达标率/路程相对前沿方法的优势：`NOT_RUN`。
- 大规模参考面积/排除误判评估：`NOT_RUN`；默认自动验证以估计达标率、原始覆盖、路程选模。
  `--reference` 才会报告已执行审计，且它本身受位置、朝向和可达图采样误差影响。
- 首版场景为单层地面高程 + 二维遮挡，不表达洞顶、多楼层、真实传感器/定位误差。
- 短测没有验证 8 GiB 回放、2 GiB 快照同时满负荷时的实际主存峰值；限量与预检已有行为测试。
- 原生策略推理的完整 DDS 任务流程、Humble GPU 八环境训练、Orin、跨机 DDS、真实 rosbag、实车：`NOT_RUN`。
  推理状态机用实际代码加可控 transport 验证取消/迟到接受/任务与 epoch 隔离，不能外推为实车证据。
- `best.pt` 的自动评分没有宣称通过真值可观测性审计；发布性能结论前运行独立参考审计和同口径基线。

操作命令见 [操作指令](../操作指令.md#drl-探索训练与续训)，验收工作分解见
[实施计划](../superpowers/plans/2026-09-08-drl-exploration-training.md)。
