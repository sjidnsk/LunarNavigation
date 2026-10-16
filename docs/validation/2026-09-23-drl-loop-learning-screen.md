> 历史分支记录：`test/drl-loop-learning` @ `d5de79c0b157`。本文运行状态、接口和实验结论是当时快照；当前开发以 [统一入口](../DRL开发入口.md) 为准。

# 学习式解环专项：工程验证与有限筛查

日期：2026-09-23。当前状态：工程验证及 GPU 恢复探针已通过；正式 3000×2 筛查首臂异常退出，未形成效果对照。
本文只记录实际执行证据；未完成阶段不推断为通过。

## 范围与版本

- 独立分支/工作树：`test/drl-loop-learning` / `.worktrees/drl-loop-learning`。
- 基线：`3d3513cc`；主体实现 `0a088a49`～`da524a14`；真实 IPC 回归 `d2421406`；包级测试路径修正 `be9f8abc`。
- 未收尾实验拒绝误重跑：`4e806f0e`；该修复在原筛查进程全部退出后进行，未更改运行中的冻结代码。
- 独立审查后的单次修复：`b7e3fc39`；末尾补齐该版本的回归、安装包和原生验证。
- 独立 Jazzy cache：`/home/kai/.cache/lunar-drl-loop-learning/jazzy/`。
- 运行产物根：`/home/kai/.cache/lunar-drl-loop-learning/experiments/20260923/`。
- 仅新 profile `loop-learning-v1` 启用诊断；默认 C0、旧 tanh C10 解析、旧命令和旧训练目录不变。
- 无运行时禁回头、目标黑名单、强制换目标、随机解困或循环罚分。地图、候选合法性、净空路径简化、导航、控制、车辆/TCP 不改。
- 本分支未合并、未推送。原开发树的 microbatch=8 两处未提交修改未纳入。

原检查点只读：

`/home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/drl-exploration-redesign/training-output/drl-coverage99-v1/resume.pt`

源 update=24296，admitted=98212；2535522730 字节。
SHA256：`71e2485ea4e53d59f2ca7a7f624c42715c0aba0967994cb459a3b7bd2fa95307`。
离线探针开始/结束、筛查前及异常退出后核验均未改变源文件。

## 已完成的离线门槛

原始产物：上述运行根下 `probe.json`，含五个输入的完整路径、mtime、字节数和 SHA256。
实际完成 4 个代表状态 × C0/旧 tanh10/新 softsign10 × 新/原 Adam = 24 次拟合，每次 300 次更新；耗时 123.18 s。

- 当前 48 个停滞状态的新候选数值审计通过；不能把这些状态当作 48 个独立场景。
- 固定 float32 反例 `[100,101,-201]`：新候选 argmax=1，梯度最大绝对值 0.0040907，有限且非零。
- 旧 tanh 的最高分并列和零梯度作为确定性风险测试保留；没有宣称修复旧 tanh，也没有用 raw argmax 或破平局规则掩盖问题。
- 完整门槛 `passed_offline_gate`，原 Adam 跨位置改善见样本 22。门槛是 Q 跟随能力，不是实际探索成功。

保留原 Adam，300 次优化后最佳 Q 动作的概率：

| 样本 | C0 | 旧 tanh10 | 新 softsign10 |
| --- | ---: | ---: | ---: |
| 0 | 0.911828 | 0.427575 | 0.254672 |
| 6 | 0.877574 | 0.385664 | 0.216447 |
| 22 | 0.000000030 | 0.483076 | 0.233705 |
| 26 | 0.000000031 | 0.122746 | 0.081130 |

样本 22 中，C0 概率由约 1.106e-6 继续降低；softsign 从 0.034706 学到 0.233705，贪心位置转到 Q 首选位置。
但样本 0 并非有界头更好，样本 6 保留了“Q 偏爱循环”的反例。不能推出“更听 Q 的话就会解环”。
新 Adam 与原 Adam 结果分别保留，不混成同一个结论；离线拟合模型未导出部署。

## 测量口径

- 解环：从实际世界坐标检测两点往返或连续 32 步零新增；切换朝向/第三点不算恢复。以真值可覆盖网格新增为准，记录恢复等待步数、后续 32 步新增、再次停滞及预算/错误删失。
- 回退：自然轨迹仅称“观测到回退后新增”；必要回退另用固定 U 形拓扑测试，不把脚本路线当策略学习效果。
- Critic：冻结 Actor/Q1/Q2/alpha，用同一随机策略采样；soft return 的熵从下一动作开始。预算截断/错误没有完整 Q 标签；已终止样本有选择偏差，单次回报不是精确 Q 真值。
- 回放：同一保存边界分别审计新采集窗口、内存回放、可恢复快照。采样仍为原 uniform/FIFO 语义；图锚点回退只作补充，不能代替物理位姿指标。
- 成功：无碰撞且真实覆盖达到 99%；`GOAL_REACHED` 不是探索完成。失败保留在已尝试分母，未尝试单列，缺失配对不删去。

## 已执行的工程测试

| 验证 | 当前实测结果 |
| --- | --- |
| Task 1 模型身份/兼容性/新数值形式 | 全包 345 passed / 2 skipped |
| Task 2 采集/TD/Actor 数值守卫、micro8/16/64 | 全包 356 passed / 2 skipped |
| Task 3 源检查点匹配的离线探针 | 全包 362 passed / 2 skipped；真实 24 次拟合通过离线门 |
| Task 4 物理恢复指标、回退场景 | 全包 372 passed / 3 skipped；原生 U 场景相关测试 3 passed，24.28 s |
| Task 5 三类经验审计 | 全包 379 passed / 3 skipped |
| Task 6 冻结随机策略 Q 校准 | 全包 387 passed / 3 skipped |
| Task 7 新 profile/身份/筛查停止门 | 全包 407 passed / 3 skipped |
| Task 8 真实 IPC、新 profile 两臂更新恢复、导出、三种故障 | 5 passed，23.58 s |
| Task 8 仓库根全包 pytest | 412 passed / 3 skipped，88.32 s（随后增加一个测试工作目录参数） |
| 受影响 Jazzy 包重建 | 1 package，0.69 s |
| 实际 colcon 包测试，修正测试路径后 | 413 passed / 3 skipped，95.71 s；test-result：416 tests / 0 errors / 0 failures / 3 skipped |
| 本轮独立原生导航/回退重验 | 安装包环境，5 passed，39.89 s；原生动作、参考路径、实际到位与子进程清理 |
| 两臂 GPU smoke + 新进程恢复 | 四次运行均通过；下表保留实际超额与耗时 |
| 首训练种子 raw/softsign10，各 3000 | raw 异常退出；softsign10、全部正式评估 NOT_RUN |
| 未收尾父状态的恢复防护 | 3 项 RED；修复后相关 34 passed，1.77 s；正常有序中断仍可恢复 |
| 防护修复后的包重建 | 1 package，0.74 s |
| 防护修复后的实际 colcon 包测试 | 416 passed / 3 skipped，86.48 s；test-result：419 tests / 0 errors / 0 failures / 3 skipped |
| 最后独立代码审查 | 已执行一次；3 Important + 1 Minor，后者按丢失已执行案例的影响升格修复，见下文 |
| 审查修复后源码全包回归 | 432 passed / 3 skipped，93.32 s；含真实 IPC 反向 NaN 故障验收 |
| 审查修复后安装包重建 | 1 package，0.78 s |
| 审查修复后实际 colcon 包测试 | 432 passed / 3 skipped，91.74 s；test-result：435 tests / 0 errors / 0 failures / 3 skipped |
| 审查修复后原生重验 | 安装包环境，5 passed，39.18 s，覆盖上述 opt-in 跳过项 |
| 原长训真实旧检查点兼容检查 | 只读加载并通过新版四处身份校验：admitted98212 / updates24296；未保存、未恢复训练 |

故障测试在真实 collector/worker IPC 和完整 128/8/6 网络路径中注入非有限策略、模拟 CUDA OOM 和磁盘 ENOSPC；原 `resume.pt` 哈希保持不变，拥有的子进程/线程退出。模拟 OOM 不冒称本机实际显存耗尽。

包测试失败记录也保留：自动发现因当前 setuptools 忽略 `tests_require`，首次执行 0 测试（exit5）；显式 pytest 第一次运行得到 411 passed / 1 failed / 3 skipped，失败是既有小课程测试按 cwd 读取 YAML。
补充无关工作目录回归后观察 RED，再改为从测试文件定位仓库配置；7 项相关测试及上表完整 colcon 重跑通过。未修改生产加载路径。

colcon 使用已有 Torch venv（系统 Python 未安装 Torch），仅在父进程末尾追加系统 colcon 模块路径，并显式 `--python-testing pytest`；没有安装或覆盖依赖。
安装 Python 包位于新 cache，已逐文件对比与源码一致，没有加载旧 DRL 安装。

GPU smoke 使用独立 `smoke/` 输出，warmup64、40 m、8 步回合；正式筛查仍为预热1024和512步，没有把探针训练接入正式矩阵。

| smoke | 新采集 | 累计采集 | 累计更新 | 墙钟 s |
| --- | ---: | ---: | ---: | ---: |
| raw 初次（请求160） | 163 | 163 | 23 | 54.25 |
| raw 新进程恢复（请求32） | 35 | 198 | 32 | 18.95 |
| softsign10 初次（请求160） | 162 | 162 | 23 | 55.51 |
| softsign10 新进程恢复（请求32） | 33 | 195 | 32 | 23.81 |

四次运行初始五网络 checksum 都为 `6a30ce3906917315a5ce9dc7b6c84f458c0b7d606cedf21f7743e7ae6a9a7fe5`，与 manifest CPU 预计算一致；恢复回合 ID 与先前不相交。
四次均完成真实 CUDA 更新、Actor 导出和重新加载，运行记录均确认拥有的子进程关闭；进程表另行确认无探针遗留。
初次 raw/softsign PyTorch 峰值 allocated 分别为435635712/396754944字节；这不是总设备显存，也不能外推到大地图。

## 正式协议与当前停止边界

训练种子固定 2026092301/02/03；每种子的两臂五套网络初始参数校验和相同且各自新建 Adam。
单因素为 raw C0 对照新 `centered_softsign_v1` C10。
batch64、microbatch8、gamma1、学习率1e-5、初始 alpha5e-5/上限1e-4、联合熵、预热1024、更新比0.25；奖励和99%阈值不改。
8 环境、目标 RTF30、10 m/90°、0.2 m/s、40～80 m 小地图、512 回合预算；GPU 串行。

筛查只允许首种子每臂 3000 新转移（异步超额另报）。每臂另有最多 896 个评估动作：两张已用 60 m 诊断地图各128 argmax，两地图×随机种子11/29各128 Q校准，U回退场景128。
这些评估不进入训练回放。正式矩阵虽然冻结未来三个种子每臂累计10000，但筛查结束必须停在 `awaiting_extended_review`，另获明确同意才可延长。

实际已扫描 103 份历史 JSON、153 个地图身份；60 m 月表/洞穴各10个新留出种子2026100101～2026100110无交集。
旧20案例与已诊断场景属于开发/回归集；新留出集仅留给未来 extended，不参与本次筛查选型。

## 尚未形成的结论

目前只能证明新候选在部分冻结 Q 状态上可学习，以及上列诊断/恢复工程契约通过；尚不能声称循环减少、覆盖提升或收敛。
本轮筛查没有有效两臂结果，不能从训练末次 loss 较小推断效果改善。
extended、120/250 m、Humble、Orin、目标 DDS、实车均 `NOT_RUN`。没有自动增加多步 TD、经验重采样或位置/朝向熵拆分。

## 正式筛查的异常与证据边界

实际启动版本为 `be9f8abc`，输出为运行根下 `paired/`，仅运行首个 C0 组。北京时间15:52:42，
`2026092301/raw/child-result.json` 留下 `status=failed`、`BrokenPipeError: [Errno 32] Broken pipe`，墙钟856.348 s。
父驱动PID205018和子进程PID205227均已不存在；逐项核对末次资源记录中的26个PID及训练/评估进程后无本轮遗留。
父 `paired-state.json` 和 `loop-report.json` 未完成收尾，仍写 `running`，**这是过期状态，不是正在训练**；原始证据不手工改写。
没有启动第二臂，也未自动重试失败臂。

| 项目 | C0 / raw | 新 softsign10 |
| --- | --- | --- |
| 实际状态 | 失败，父记录未收尾 | NOT_RUN |
| 最后滚动进度 | admitted2244 / updates305，墙钟843.495 s | 无 |
| 验证可加载的检查点 | admitted0 / updates0，39,199,871字节 | 无 |
| 固定 argmax 评估 | 0/2 尝试 | 0/2 尝试 |
| 随机策略 Q 校准 | 0/4 尝试；标签数不可评估 | 0/4 尝试；标签数不可评估 |
| U 回退评价 | NOT_RUN | NOT_RUN |

初始 checkpoint 保存于15:38:27，尚未到30分钟定时保存点，异常也没有覆盖最后好检查点。
因此2244/305只能表示末次已观测运行进度，不能作为可恢复进度；也不能说“从2244接着训”。
没有完成保存边界，三类经验审计在可恢复快照中均为初始空集；不能据此称训练期间没有正收益经验。
内存回放末次约312,130,957字节，但随进程退出已不可作完整审计。逐场景覆盖差值、真实解环率及完整 Q 偏差均不可评估。

末次记录：平均2.66转移/s（包含预热）；PyTorch peak allocated649,225,728字节、reserved3,007,315,968字节；
本轮进程PSS4,569,856,000字节、系统可用18,995,003,392字节、磁盘空闲235,518,574,592字节。
这些是保留的滚动观测，不是全程峰值保证；记录的退出异常不是OOM。
管道断开与会话中断时间相邻，但现存日志没有完整异常栈，不能据此断言最初是哪个进程或信号引发。

发现并修复的恢复漏洞：仅有父锁释放，不代表子进程已正常退出；原驱动可能把未收尾的 `running` 记录当作可恢复阶段，
再次启动并覆盖子结果。新检查在这种状态下拒绝自动恢复，保留父/子文件不变；父驱动已确认的 `interrupted` 仍正常恢复。
该修复不冒称解决外部强制终止，也不修补或重标记历史实验。源码身份已改变，原 `paired/` 不能作为新版本的续训目录。

下一步仍需先获得一次完整、同条件的两臂有限筛查及 Q/经验审计；如重新执行，应另行确认，在独立持久终端和新目录中运行同样3000×2上限。
离线样本22支持继续检验 Actor 学习受阻；样本6提示 Critic 可能偏爱错误循环。当前证据不能判定哪一项是主因，
更不能直接批准熵拆分、改价值目标或经验重采样。

## 最后独立审查与单次修复

独立审查范围 `3d3513cc..a7e69a20`，由独立上下文审查者只读检查；原结论为 `With fixes`，不是无问题通过。
按用户选择，仅进行一次整体审查；执行者随后用逐项 RED→GREEN 和完整回归验证修复，没有再派第二个审查者。

| 审查问题 | 修复与验证 |
| --- | --- |
| 漏验 collector 的第四处模型身份 | 将保存的 collector Actor 配置纳入 canonical 一致性检查；混搭/缺身份3项 RED，32项相关 GREEN；旧 C0/C10 四处缺 transform 的实际导出/恢复保持原含义 |
| 中断请求掩盖真实失败 | 子进程明确返回有序停止，父进程单列中断请求；真实故障始终 failed，不再变为可自动恢复；已完成子阶段不重跑。5项 RED，39项相关 GREEN |
| 健康前向后的非有限梯度/参数穿过安全边界 | opt-in 检查梯度、更新后五网络/温度、损失和优化器状态；在更新计数增长、发布或保存前抛错。5类故障 RED，29项相关 GREEN，保留一次合并的 GPU→CPU 诊断传输及 microbatch 等价性 |
| close 异常丢失循环指标或 U 案例 | 保留已积累的真实步数、覆盖/循环记录和原错误，再附加 close 错误。2项 RED，42项相关 GREEN；该项由 Minor 升格处理 |

四项均进入同一次修复轮次，源码全包结果见上表；未留下审查 Minor 待办。
恢复拒绝和数值守卫只保护实验有效性，不选择动作、不强制脱环。原筛查现场及输入文件仍只读保留；源码变化后不把新旧版本结果混成同条件配对。
表中24次冻结拟合、四次 GPU smoke 和首次原生测试均属于所列阶段版本的证据，并非审查修复后的正式筛查结果。

## 复核命令与记录位置

完整各任务命令、RED/GREEN、初次失败日志、审查包和裁决保存在运行根下 `execution-records/`（交付时从本任务忽略目录原样归档）。
该目录不进入 Git，构建和模型也不提交。`progress.md` 是逐项执行记录；`final-review.md` 是执行方对独立审查的结构化转录。
下列命令从专项工作树根执行，使用归档的隔离环境脚本；该脚本清除旧 ROS/Python overlay，再只加载 Jazzy 与专项 install。

```bash
cd /home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/drl-loop-learning
# 源码完整回归
bash /home/kai/.cache/lunar-drl-loop-learning/experiments/20260923/execution-records/drl_python \
  -m pytest ros2_ws/src/lunar_drl_exploration/test -q -p no:cacheprovider
# 实际包测试：已有 Torch venv 加载系统 colcon；必须显式选 pytest
bash /home/kai/.cache/lunar-drl-loop-learning/experiments/20260923/execution-records/drl_python \
  -c 'import sys; sys.path.append("/usr/lib/python3/dist-packages"); from colcon_core.command import main; sys.exit(main())' \
  --log-base /home/kai/.cache/lunar-drl-loop-learning/jazzy/test-log-final-review test \
  --base-paths ros2_ws/src --packages-select lunar_drl_exploration --executor sequential \
  --build-base /home/kai/.cache/lunar-drl-loop-learning/jazzy/build \
  --install-base /home/kai/.cache/lunar-drl-loop-learning/jazzy/install \
  --python-testing pytest --event-handlers console_direct+
colcon test-result --test-result-base /home/kai/.cache/lunar-drl-loop-learning/jazzy/build --verbose
# 先确认200/228 domain无他人作业，再单独运行原生层
LUNAR_DRL_RUN_ROS=1 bash /home/kai/.cache/lunar-drl-loop-learning/experiments/20260923/execution-records/installed_python \
  -m pytest ros2_ws/src/lunar_drl_exploration/test/test_ros_native_execution.py \
  ros2_ws/src/lunar_drl_exploration/test/test_loop_backtrack_scene.py -q -p no:cacheprovider
```

这些测试命令不等同于批准重做正式训练筛查；新目录3000×2、extended 和目标部署仍需分别确认。
