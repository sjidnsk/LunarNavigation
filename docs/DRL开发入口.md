> **2026-10-17 当前入口：[当前工作入口](当前工作入口.md)。** 清理后的分支/工作树、职责与公共差异以该入口及外部实际回执为准。
>
> 以下既有正文保留原阶段上下文；“原分支/工作树全部保留”等历史表述仅指当时，后续清理事实见当前入口。

# 强化学习统一开发入口

## 2026-10-17 开发整合与运行身份分离

开发整合线为 `dev/drl-exploration`，物理路径为
`/mnt/data/WS/lunar-navigation/lunar-runtime/.worktrees/drl-exploration-redesign`。
主线承载完整项目的已验收模块；DRL 训练/研究代码完成适用代码验收后可晋级，
模型收敛、策略效果与 Orin 实车输入适配分别验收。本专题候选等待独立审查与主 agent 合入。

2026-10-16 的 78 文件预算冻结版本随后由导航恢复版本替换为 **79 文件**。
原 [deployed 清单](drl/formal-runtime-source-manifest.json) 记录运行身份
`8470d67be1e6e5fc2dfa7e0a3351017bff35b07c630a9bd40246ba34c83b09da`；
冻结包是 `/mnt/data/WS/lunar-navigation/.runtime/candidate-geometry-v3/source-navigation-recovery-20261016-r2/lunar_drl_exploration`，
运行 cwd 为 `/mnt/data/WS/lunar-navigation/.worktrees/drl-navigation-recovery-20261016`。
此信息来自切换记录及本轮只读源码核验，不以历史 PID 推断当前进程存活状态。

本次开发修复只改变 `Trial.extend` 检查点引用生命周期，训练前和评测前释放 loaded state，
避免控制器在子进程期间持有完整回放。新的 [开发源码清单](drl/development-source-manifest.json)
仍为 79 文件，标记 `not_deployed`；与旧运行清单仅 `course_two_trial.py` 不同。
运行冻结包、配置和旧 manifest 不改写；默认 verifier 继续以旧 deployed 清单为准，
在新开发源码上报告该文件 changed 是预期漂移检测，不表示可覆盖运行包。

从本仓库根显式分别只读核验：

```bash
python3 scripts/drl/verify_formal_source.py --manifest docs/drl/development-source-manifest.json --package ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration
python3 scripts/drl/verify_formal_source.py --manifest docs/drl/formal-runtime-source-manifest.json --package /mnt/data/WS/lunar-navigation/.runtime/candidate-geometry-v3/source-navigation-recovery-20261016-r2/lunar_drl_exploration
```

逐项历史处置、奖励监看工具来源与主线兼容方案见
[2026-10-17 开发归并](drl/2026-10-17-development-integration.md)。以下日期段保留当时记录，
其中旧 78 文件、路径、PID、启动命令与验证数量不能覆盖本节；本次不切换训练运行入口。

---

## 2026-10-16 正式训练版本源码整合

唯一开发整合线为 `dev/drl-exploration`，工作树为项目根下
`lunar-runtime/.worktrees/drl-exploration-redesign`。已收录当前正式
`candidate_geometry_v3 / task_graph_v5` 架构、完整回放、课程与评测控制、
改进导航的训练接线、到达契约及动态行程预算；对应配置为
`config/drl_candidate_geometry_v3.yaml`，默认旧模型配置不静默切换。

详见[正式版本整合说明](drl/2026-10-16-formal-version-consolidation.md)；
运行 `python3 scripts/drl/verify_formal_source.py` 可只读核验78份训练Python源码。
模型、回放和实验产物不进入Git。

当前正式训练仍使用原 `.runtime/candidate-geometry-v3` 冻结运行副本，实验根为
`.runtime/candidate-geometry-v3/experiment-20261014`；本次Git整合不切换或重启该服务。
V5、旧课程和到达修复工作树仍保留，其中存在运行cwd或构建源路径依赖，不可据此删除。

以下日期段保留为历史记录，其旧训练、绝对路径和缓存不作为本次正式训练启动依据。

---

## 2026-10-12 统一整合入口

代码开发统一到 `dev/drl-exploration`，工作树为
`/home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/drl-exploration-redesign`。
本次整合当前 task-frontier/A2、课程一和课程二、回合固定传感器、12 路动态派发、
并行评测、暂停恢复、扩大预算的分段评测，以及图压缩压力分析工具。
独立整合与验收工作树是 `.worktrees/drl-integration-20261012`。

正在运行的 16384 条追加课程二仍使用 `.worktrees/drl-task-frontier` 与其已绑定缓存；
训练源码、Git 索引、分支、检查点和目录均保持原位。这是运行版本冻结期间的临时源路径，
并不建立第二条开发主线。后续代码改动进入 dev 的新专项工作树；本轮训练结束后，
迁移运行入口须从 dev 重新构建自己的缓存，并核对 runtime 身份/检查点兼容性。
独立验收缓存 `/home/kai/.cache/lunar-drl-consolidation/jazzy-20261012` 绑定整合工作树，
不能拿来跨路径直接启动 dev 或当前训练。

当前训练安排见 [追加训练](drl/2026-10-12-course-two-extended-training.md)，
退出修复见 [补采停止恢复](validation/2026-10-12-course-two-prefill-stop-recovery.md)。
当前任务前沿奖励、A2 紧凑策略图、真实位置/朝向动作和导航边界保持现有实现，
默认一步学习、分离位置/朝向熵、`position_aux_weight=0`。
旧位置辅助损失及不同模型版本的循环实验保留为 Git 历史和明确标注的历史文档，
没有重新启用或未经适配覆盖当前实现。

[工作树整合清单](drl/2026-10-12-worktree-consolidation.md) 列出每个来源、处理方式与验证结果。
旧工作树和分支均保留；后续不在旧设计/旧训练实验树继续开发。

---

以下为前期记录，日期对应当时状态；当前入口以上述 2026-10-12 部分为准。


## 2026-10-04 四阶段课程（当前训练方案）

同一专项工作树启用 `config/drl_ordered_curriculum.yaml`：区内月表 → 区内洞穴 → 区外月表 → 区外洞穴，一个SAC从零开始，各阶段继承完整学习状态。后续阶段按转移/采样当前80%、旧任务合计20%；面积预算256–512，不计算可探索面积。默认每8192转移评测当前5图及旧任务5图，按连续评测门槛晋级，未通过且达上限保存停止。旧混合训练已保存24,152转移/5,781更新。

设计、门槛、恢复与监看命令见[四阶段课程操作](drl/2026-10-04-ordered-curriculum-operations.md)。新产物根 `/home/kai/CodexDownloads/lunar_navigation/drl-ordered-curriculum-20261004/managed`；旧实验和26856保留。本专项仍未合并/推送。


当前专项为`feat/drl-task-frontier`，工作树`/home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/drl-task-frontier`，配置`config/drl_task_frontier.yaml`。实现/本机原生CUDA烟测完成；正式实验等待最终独立审阅门禁后由父控制器直接启动。开发整合线仍为`dev/drl-exploration`，不合并/推送本专项。

新入口与证据见[Task frontier验证](validation/2026-10-04-drl-task-frontier.md)及[操作指令](操作指令.md#task-frontier-20261004)。专项输出`/home/kai/CodexDownloads/lunar_navigation/drl-task-frontier-20261003/`，生产managed与独立task8烟测计数分开；两项Task1 deferred minor（当前入口索引、冷构建native链接）已解决。

以下为历史实验索引，其运行状态和cache不覆盖本轮专项。

历史专项：用户已确认[位置与朝向独立熵对照](validation/2026-09-27-drl-independent-entropy.md)，
分支`feat/drl-factorized-entropy`，工作树`.worktrees/drl-factorized-entropy`。
采用原25320完整起点、一步回报及相同预算；默认joint路径和部署决策保持兼容。
两臂共同使用温度学习率.003，唯一臂间变量为联合/独立熵控制；不能与旧实验直接作因果比较。
训练状态以原共用产物根`evaluation/phase5-entropy-manifest.json`为准，冻结效果尚未验证。

2026-09-27后续按用户确认推进[一步/八步soft Retrace对照](validation/2026-09-27-drl-multistep-experiment.md)：
专项`feat/drl-multistep`复用`.worktrees/drl-stagnation-budget`；工程验证及两臂有界训练已完成。
两臂均新增2048条/512更新；训练成功12次与11次，八步实际多步抽样占16.38%，尚无优势证据。
原冻结评测BrokenPipe中断后，仅补齐八步最后两图，21:00正常结束；最终一步4/5、八步2/5。
八步原成功图退化且关键moon2703仍循环，本轮已关闭，不扩训、不跑新种子/复核、不标best。
相关进程均已退出；原中断证据与补评证据分别保留，最终对照和后续只读诊断见该记录。
64个新经验起点只读诊断已完成：延迟面积收益确实存在，连续片段平均7.64步，
修正系数质量平均3.66；有联合动作排序差，但不能把Q估计当正确出口或认定唯一循环根因。
默认仍为一步、tau=.005；本轮新增两臂快照各512MiB，与全部历史产物共用下述8GiB根。
实施和结果状态以该记录为准，不把下面已关闭的Polyak阶段与本轮混为同一运行。

本轮小场景恢复入口：`config/drl_exploration_recovery.yaml` 配合
`scripts/drl/run_bounded.py`，共同存储根总额8 GiB、诊断128 MiB。
60 m月面/洞穴、每回合512步、最多2048条新增入库；原学习目标与已有状态保留。
操作和恢复参数见[操作指令](操作指令.md)，执行证据见
[有界恢复记录](validation/2026-09-27-drl-bounded-recovery.md)。

首轮2,048条已正常结束，保存边界100,260条/24,808次更新。
第二轮于2026-09-27 12:30正常结束，保存102,308条/25,320更新；训练完成13次探索不等于冻结成功率。
随后16回合冻结评测已完成：24808和25320在原4图均4/4，新4图均3/4；两者仍在新月面图持续循环，
第二轮没有证明解环或效率改善。另一次诊断探针取得9条真实往返transition，已完成匹配Actor/双Q/目标Q只读审计。
随后完成[目标网络更新速度对照](validation/2026-09-27-drl-polyak-experiment.md)：
共同25320起点、每臂2048条/512更新；10回合冻结评测中0.005完成2/5、0.02完成4/5，
但关键moon2703两臂仍循环，实验未通过解环门槛。所有进程正常结束，停止扩训、新种子和复核；
默认参数不变，不标best，完整起点和两臂证据保留，产物约4.41 GiB。
第二轮详细结果见[冻结执行记录](validation/2026-09-27-drl-phase2-frozen-evaluation.md)，
原定流程见[第二轮验收与后续安排](validation/2026-09-27-drl-phase2-and-next-steps.md)；
前序现场评分见[冻结初筛记录](validation/2026-09-27-drl-frozen-screen.md)。完整25320共同起点继续只读保留。

2026-09-26 新增独立修复入口：`fix/drl-stagnation-budget` /
`.worktrees/drl-stagnation-budget`，实现用户确认的“保持学习式解决、允许训练截断长期停滞回合”。
采用128步无新增且少数实际站位重复的训练采集限制，正式评估默认关闭；可用 `--fork-from`
保留已有模型/优化器/回放，在新目录明确变更采集规则。2026-09-27以`046ed473`合入本开发线，
并从原模型启动有界小场景恢复训练；原长训目录仍只读保留；
工程与效果边界见[本轮记录](validation/2026-09-26-drl-stagnation-budget.md)，
操作见[当前有界训练入口](操作指令.md)。这不代表网络已学会解环。

2026-09-20：按用户要求将当前观测机会重构整合到唯一开发线并启动正式课程训练。本文件是当前开发位置、版本和操作入口的索引；早期文档中的运行状态与命令不能覆盖本文件。

## 唯一活动开发线

- 分支：`dev/drl-exploration`（由原 `feat/drl-exploration-redesign` 改名，保留提交历史）。
- 工作树：`/home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/drl-exploration-redesign`。
- 本轮构建、安装、日志：`/home/kai/.cache/lunar-drl-stagnation/jazzy/`；操作前设置
  `DRL_CACHE`为该路径。旧`lunar-drl-redesign/jazzy`缓存不作为本轮恢复的安装证据。
- Python 依赖：`/home/kai/.cache/lunar-drl-training/venv/bin/python`。这里的 training 只是共享依赖目录名称，不表示使用旧训练分支。
- 本轮模型、回放、指标、实验结果：`/home/kai/CodexDownloads/lunar_navigation/drl-small-recovery-20260926/`，
  共用8 GiB总额/128 MiB诊断上限；历史`training-output/`保留为只读来源。

复用现有工作树，避免复制大模型或破坏已有构建路径。目录中的 redesign 是保留的物理路径，不再表示另一条活动开发线。

## 分支层级与流向

```text
integration/pure-planner-orin      公共地图、规划、控制和通用接口的集成主线
└── dev/drl-exploration           强化学习模块开发整合线
    ├── feat/drl-<主题>           专项功能
    ├── fix/drl-<问题>            专项修复
    ├── test/drl-<实验>           需要修改代码的试验
    └── docs/drl-<主题>           专项设计与文档
```

其他模块或场景开发线与 `dev/drl-exploration` 同级，不隶属于强化学习。上图的专项分支是命名和归属示例，本次不创建空分支或工作树。

- 专项从所属开发线派生，在独立工作树完成，验证后合回所属开发线；开发线负责整理可共同运行的模块版本。
- 纯超参数对比无需改代码，用同一提交、不同配置和输出子目录运行；需要改变算法的实验才派生测试分支。
- 共性修复在集成主线的专项分支完成并合入主线，再由开发线同步。若首先在模块中发现，提取公共部分回到主线，不将整个训练模块一并上送。
- 当前已同步的净空代价和路径简化器通过合并主线记录共同祖先；不只保留内容相同却彼此不相认的复制提交。
- 后续合并、发布和删除仍遵守用户授权；本次整合授权不等于后续所有分支自动合并或清理。

## 固定的代码基线

| 层次 | 当前实现与边界 |
| --- | --- |
| 地图与任务分析 | 原生增量地图、二维射线遮挡、任务内覆盖；只维护当前已知可达站位支持的任务观测机会。任务外连接和直接观测保留，不推演未来未知通道。真值可覆盖面积只用于训练/评估。 |
| 完成 | 训练/仿真真实可探索覆盖率≥99%即成功；未达标耗尽为非成功终止。部署保持当前机会耗尽；输入不可用及预算截断不是成功。 |
| 决策图 | `task_graph_v3`：净空优先基础选点、2 m 合法路径覆盖、8 m 局部连接、1.2 路程伸长稀疏化；保留必要补点、回退和任务外绕行。 |
| 动作 | `local_metric_pose_v3`：相邻候选位置与八方向观测朝向；探索层选目标，导航层生成路径。 |
| 学习 | `candidate_graph_sac_v2`；候选对应的 `candidate_truth_v1` 训练侧 Critic；Actor 不读取真值。 |
| Actor 实验 | 默认 C=0；有界 C=10 可通过配对驱动比较，尚未确定胜者。SAC 损失保持不变。 |
| 导航与控制 | 正式规划器、保留原路径净空的简化器、轮式净空代价权重 2.0、公共轮式跟踪器、轻量运动学平台。 |
| 运行 | 8 环境、30 倍目标倍率、0.2 m/s、10 m/90°；batch64/microbatch16、gamma=1.0、目标熵因子 .10、alpha 上限 1e-4；每30分钟保存。 |

整合后按用户要求采用净空优先基础选点B版，并保持观测补点规则。C=5敏感性探针仍不是正式算法。
当前决策图仍允许必要近邻节点。实现、对比与单线训练协议见[净空优先基线](validation/2026-09-19-drl-clearance-baseline.md)。

职责与设计详见：

- [完整架构](superpowers/specs/2026-09-12-drl-exploration-redesign.md)
- [决策图](superpowers/specs/2026-09-18-drl-decision-graph-consolidated.md)
- [候选对应 Critic](superpowers/specs/2026-09-18-drl-critic-action-alignment.md)
- [Actor 配对实验](superpowers/plans/2026-09-19-bounded-actor-experiment.md)
- [导航修正运行验证](validation/2026-09-19-clearance-runtime-rollout.md)

## 历史分支如何处理

以下是保留来源，不再作为当前 DRL 开发或实验入口。保留不代表其工作区改动已经全部合入。

| 分支 / 工作树 | 用途及本次处理 |
| --- | --- |
| `feat/drl-exploration-training` / `drl-exploration-training` | 早期循环 SAC 与训练闭环；含未提交修改，原样保留。不将旧模型、奖励与恢复协议并入新图策略。 |
| `fix/observability-coverage` / `observability-coverage` | 早期覆盖算法及运行优化；含未提交修改，原样保留。其 HEAD 是祖先不能证明这些修改已合入。 |
| `feat/drl-exploration-complete-design` / `drl-exploration-complete-design` | 早期连续动作设计，只作历史参考。 |
| `feat/exploration-coverage-definition` / `exploration-coverage-definition` | 早期覆盖定义，只作历史参考。 |
| `fix/path-simplifier-clearance`、`feat/clearance-cost-calibration` | 公共导航修正来源；正式简化器和净空权重已同步到当前训练线，不使用这些树的实验库覆盖训练安装目录。 |

不复制旧回放、不删模型、不删除脏工作树，不以形式上的全分支 merge 冒充兼容整合。后续如果需要回收旧工作树空间，应另行核对其中独有修改和产物。

## 统一命令

```bash
cd /home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/drl-exploration-redesign
scripts/drl/build.sh
scripts/drl/train.sh --help
scripts/drl/compare_actor.sh --help
```

正式训练、恢复、导出、评估和推理的参数见[操作指令](操作指令.md#drl-稀疏图探索独立-redesign-分支)。启动脚本清除继承的旧 overlay，再加载同一份 Jazzy 安装；不要手动注入旧工作树的库。

当前正式长训使用统一配置，无有限转移上限；模型、优化器、回放和课程从头开始，输出到项目内 `training-output/drl-coverage99-v1/`：

```bash
scripts/drl/train.sh --config config/drl_exploration.yaml
# 中断后使用相同配置恢复本轮训练：
scripts/drl/train.sh --config config/drl_exploration.yaml --resume
```

仍采用原始分数形式 C=0 的 Actor，并正常更新网络权重。按本轮确认采用 gamma=1.0，并对真实可探索覆盖率达到99%的最后一条转移追加一次 completion_bonus=5.0；其余面积/路程/转角/步代价及目标熵因子 .10、alpha 上限 1e-4 保持原值。旧 `baseline.sh` 是历史有限训练/分阶段评估驱动，不作为本轮正式长训入口。

旧 `training-output/actor-bounded-20260919/` 保留碰撞修正前的记录，目前为 interrupted，配对实验未完成，不能据此确定原分数或有界分数胜出。切换导航算法后不要继续把该目录的早期组与新运行组当成同一条件的对照。

模型恢复是否兼容由观察、动作、Critic 和配置协议决定，不由分支名字决定。净空优先基础选点已经写入完整恢复语义，旧行列顺序检查点不静默续训；新基线用新目录、新模型和新回放。历史结果仍可用于诊断。

## 验证记录

本次重新执行的构建和测试见[整合验证](validation/2026-09-19-drl-development-consolidation.md)。算法单元测试和本机 Jazzy 闭环不能证明长期收敛、两节点循环已消除、Humble/Orin 或实车可用。

2026-09-20 的[覆盖完成判据试验](validation/2026-09-20-coverage-completion.md)位于
`test/drl-coverage-completion`，尚未合入或采用。它实现测量侧覆盖下界，但固定月表/洞穴的剩余需求估计仍偏保守；正式开发和训练入口继续使用本文件上方的开发线。

同日按用户确认的职责边界，试验分支在 `cad9565` 重构为[当前观测机会耗尽](validation/2026-09-20-current-task-opportunities.md)：取消未来站位/通道推演和在线覆盖下界，80%/99% 仅由真值评估。观测补点由决策图在可执行已知位置中选择。该报告记录合入前证据，随后用户授权整理合入开发线并开训。

当前训练完整恢复语义为 `truth_coverage_or_current_opportunity_exhaustion_v1`；旧结束语义的完整检查点和回放不直接续用。旧模型与所有历史输出保留，新检查点可按上面的 `--resume` 命令恢复。当前只验证10 m/90°，窄视野/八方向匹配的既有限制及长期策略效果仍须单独验证。

本轮合入、重新构建、313项测试和正式开训证据见[2026-09-20整合与开训记录](validation/2026-09-20-drl-training-integration.md)。

此前奖励与折扣切换见[γ=1与完成奖励](validation/2026-09-20-drl-undiscounted-completion.md)。之前的 current-opportunities-v1 输出为旧奖励记录，不能用新配置直接 --resume；本轮新输出目录保持同一保存/恢复入口。

当前99%成功条件、奖励及验证见[覆盖达标成功](validation/2026-09-20-drl-coverage99-success.md)。旧耗尽奖励版本的完整检查点不直接续用，保留原产物；新目录从头训练。
