> 历史分支记录：`feat/drl-position-aux` @ `dcca338bc501`。本文运行状态、接口和实验结论是当时快照；当前开发以 [统一入口](../DRL开发入口.md) 为准。

# 位置辅助损失在线训练对照

## 用户确认的范围

从保留候选 26856 分出两组，普通独立熵 SAC 的位置辅助权重为 0，实验组为 0.001。
每组新增 8,192 条转移、执行 2,048 次更新；总计 16,384 条、4,096 次更新。
原候选只读保留。沿用统一经验回放，不标记或划分循环经验；不加运行时禁回头、黑名单或强制换目标。
采用正常环境交互、价值网络和两个熵系数联合更新，不再以冻结两个状态拟合的结果代替在线训练。

分支 `feat/drl-position-aux` 从 `dev/drl-exploration@32155954` 创建，先 cherry-pick
独立熵依赖 `d2af27d6` 为 `2d56d68e`。原开发线及独立熵工作树不变；未合并、推送或创建 PR。

## 实现

原 SAC Actor 损失完整保留。在独立熵模式下可选增加位置交叉熵：

- 当前策略的条件朝向分布为 `rho(h|v)`，位置边缘分布为 `p(v)`。
- `G(v) = sum_h rho(h|v) * [min(Q1,Q2)(v,h) - alpha_heading * log rho(h|v)]`。
- `teacher(v) = softmax_v(G(v) / alpha_position)`，仅在每个状态的合法位置之间归一化。
- `L_pos = mean_states[-sum_v teacher(v) * log p(v)]`。
- `L_actor = L_SAC + position_aux_weight * L_pos`。

每次 Critic 更新后，在本次 Actor 前向中用当前 Q、条件朝向和熵系数重算 teacher；
teacher 整体停止梯度。本批各 microbatch 在同一次 Actor optimizer.step 前计算，策略参数一致。
不同状态按状态数平均，位置不因朝向数量多而重复计权；低概率位置使用 log 空间，避免概率下溢切断梯度。
默认权重 0 保持原更新路径；旧检查点缺少此字段时明确按 0 解释。
正权重要求独立熵模式，普通 resume 拒绝隐式改变权重；显式 `paired-position-aux` fork 才允许改权重。

诊断只增加每次更新的 `actor_native_loss` 和 `position_aux_loss` 两个标量，继续使用原有滚动 metrics。
没有轨迹、视频、循环经验数据集或中间模型序列。

## 冻结实验协议

产物根目录：`/home/kai/CodexDownloads/lunar_navigation/drl-small-recovery-20260926`，以下 EVAL 指其 `evaluation`。

- 源：`entropy-extended-split/resume.pt`，SHA256
  `2a1239f0d24322aea9ed1682e11805d98781a006387e7071fbb7803922639eae`。
- 起点累计转移 108452、更新 26856、保留更新额度 1；每组终点 116644 / 28904 / 额度 1。
- 源保存经验 1368 条；两组启动均完整恢复。Actor、双 Q、双 target、五个 Adam、
  两个已经分化的温度、RNG、collector、curriculum、counters 和 replay 的初始身份必须一致；
  唯一从身份哈希排除的学习字段是 `position_aux_weight`。
- 正常均匀 replay，batch64 / microbatch8，update_ratio=0.25；gamma1、一步回报、Polyak0.005 不变。
- 继续现有小场景采集协议：extent60 m、每回合预算512、8环境。训练保留既有长期停滞截断。
- 两组串行，各训练时限4小时。停止必须排空已接收消息，精确核对额度、零在途、零超预算及子进程关闭。
- 各自完成数值/Adam 步数/检查点身份审计后导出最终 Actor；两组都完成才自动进入冻结评测。
- 冻结评测：此前开发集5图，每组重复2轮，共20回合；同图、同512预算、同60 m、同 joint argmax；
  关闭训练停滞截断。评测总时限4小时，控制器总时限12.5小时。
- 用99%无碰撞完成数、覆盖率、实际位姿零收益 ABABA 窗口、最长零收益段及共同成功子集路程比较。
  ABABA 窗口不是独立失败回合；短开发集不能证明泛化或已经根治循环。重复评测不消除异步ROS波动。

## 存储与执行

原根目录硬上限8 GiB、诊断128 MiB保持不变，不删除历史产物。
两组新检查点的 replay 保存上限统一缩小至64 MiB；只限制保存副本，不缩小本轮启动时的1368条恢复经验，
也不改变本轮内存回放容量。后续若从这两个新检查点续训，只能恢复各自较小的保存子集，应明确记录。

测得不含 replay 的检查点序列化约79.72 MB。峰值按三个240 MiB检查点槽位（两组+一次原子保存临时文件），
全部剩余诊断空间和16 MiB Actor导出空间预留。每槽含96 MiB非replay预算、两倍64 MiB字节编码预算、16 MiB增长余量。
共享存储监控仍是最终硬限制；每阶段重新检查峰值。训练失败不会自动重试或覆盖失败证据。

运行时 Python 包安装在独立缓存 `/home/kai/.cache/lunar-drl-position-aux/jazzy`，
复用既有 Jazzy 原生依赖；不覆盖旧包安装。`EVAL/phase12_python` 清理旧环境并加载固定依赖及新包。
启动前逐模块核对源码与安装文件 SHA256，要求源码已提交且工作树干净。

控制入口与产物：

- `EVAL/phase12_position_controller.py`：初始CUDA恢复证明、两组训练、末端审计、自动评测。
- `EVAL/phase12_position_launch.py`：原根目录 execution lock + 既有 bounded supervisor；
  控制文件独立放入 `EVAL/phase12-position-control`，不覆盖旧控制文件。
- `EVAL/phase12-source-proof.json`：两个实际CUDA恢复、零optimizer step、共同初始身份。
- `EVAL/phase12-position-manifest.json`：训练/评测分开记状态，不由某个检查点推断全流程完成。
- `EVAL/phase12-eval-comparison.json`：20回合全部完成后才生成正式汇总。

## 验证记录

- 隔离 Jazzy 包构建通过；提示仅为主动禁用 pyc 编译。
- 新损失/配置/恢复/CUDA/完整模型在线fork与resume测试：11 passed。
- 控制器精确边界、峰值存储、配置单变量、评测缺失/地图身份、外部依赖校验及启动器隔离检查：
  10 passed，3 subtests passed。
- CUDA真实源恢复证明通过：两组1368经验，位置温度 `2.4298073e-5`、朝向温度 `4.2460570e-7`，没有重置。
- DRL完整测试最终重跑：626 passed / 2 skipped（134.39秒）；新增集成测试的模型尺寸和保留额度预期已修正。
- 全仓裸测试存在同名 `test_plant` / `test_evidence` 收集冲突，按包单独执行。
  controller_demo 21 passed、integrated_demo 114 passed、wheeled_controller 150 passed。
- 仓库tests目录313 passed/2 skipped/12 failed；同一12项在未修改的独立熵工作树复现，失败清单及traceback保留于
  `EVAL/phase12-other-tests.log` 与 `EVAL/phase12-unchanged-baseline-failures.log`。
  包含 task7 rank/failure contract、4项incremental RViz闭环、4项300m仿真launch/output、
  2项pure_plan_motion_server、ROS wrapper isolation；本次没有修改这些路径。
- Humble、Orin、实车：NOT_RUN。本轮为本机Jazzy算法实验；改善循环的结论等待在线训练与完整冻结评测。

独立代码审查未发现损失或恢复逻辑问题；已补齐实际执行的旧子进程清理模块 SHA256，
并用改动外部依赖即拒绝继续的测试验证 guard。
