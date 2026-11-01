# 历史场景数据使用登记（P0-02）

- 研究编号：PAPER-RL-EXPLORATION-001。
- 登记日期：2026-10-20；机器记录观察时间见结构化证据。活动目录分次读取，属于快照。
- 目的：说明现有地图、母图、种子、起点被怎样使用，为P2正式数据协议提供事实依据。
- 核查方式：SSH只读读取远程KAI的既有清单、身份、逐场记录元数据和冻结源码；不重新执行任何实验。

## 1. 登记结论

1. quick 40场全部属于validation 120场，不能作为额外40个独立样本；此前另一结构版本也使用过该quick集合。
2. 当前512预算清单复用原validation的120个case ID及除预算外的完整场景配置。它是同一验证批次的新评测条件，不是新增独立场景。
3. 母图目录包含train 64、validation 24、test 24个母图；三者既存ID和结构签名无交集。这里只完成目录核对，没有重新运行图同构检验。
4. 旧test 200场既用于原block20评估，也出现在多轮导航、MPPI变体的机器评测中。必须保留这种历史用途；不能将它改名为“新建、从未使用的独立测试集”。
5. 作者本轮明确回答：“没有看过，也没有调整过”。此声明与机器执行事实分别保留；机器上有文件不等于作者读过结果，也不足以单独证明某次修改由该结果引发。
6. 当前512运行与原验证的120场记录对比：reference ID及起点x/y全部相同，88场起始yaw数值不同。运行身份也不同，不能将结果差异单独归因于决策预算。
7. 保留训练日志只覆盖15个可识别场景，不能据此宣称完整历史训练场景均已逐项检查。缺失范围已登记，不补造不存在的完整清单。

## 2. 数据批次与来源

远程主根目录：

~~~text
/mnt/data/WS/lunar-navigation/.runtime/candidate-geometry-v3
~~~

下表前四批清单位于experiment-20261014；512清单位于checkpoint-validation-512-20261019。完整路径、逐项SHA256、种子、母图ID、起点与传感器见附件。

| 数据批次 | 清单场数 | 场景组成 | 洞穴母图数 | 文件 |
| --- | ---: | --- | ---: | --- |
| 快速验证 | 40 | {'moon': 20, 'cave': 20} | 20 | quick.json |
| 原完整验证 | 120 | {'moon': 60, 'cave': 60} | 24 | validation.json |
| 旧测试 | 200 | {'moon': 100, 'cave': 100} | 24 | test.json |
| 训练洞穴资格检查 | 64 | {'cave': 64} | 64 | train-cave-qualified.json |
| 当前512选模验证 | 120 | {'moon': 60, 'cave': 60} | 24 | validation.json |

注意：表内544行是五份清单的成员记录，包含quick与validation、两轮validation的重复，不是544个独立场景。训练洞穴资格64场用于几何资格检查；不能把它等同于全部在线训练地图。

| 批次 | 已知历史用途 | 训练输入 | 选结构/模型与系统诊断 | 结果查看与后续使用 |
| --- | --- | --- | --- | --- |
| 在线训练生成场景 | 策略交互、replay与参数学习 | 是，实际训练总预算见P0-01 | 训练诊断可能被开发流程利用 | 完整逐场历史未保留；不判定为独立测试 |
| 训练洞穴资格64场 | train母图几何资格检查、备用请求核验 | 资格文件本身不证明逐场进入replay | 几何开发/资格用途 | 与在线训练使用记录分开 |
| quick 40 | 原24块快速验证；前代结构3轮quick | 未发现作为本次训练输入的逐项证据；不等于完整无泄漏证明 | 已参与开发与检查点观察，不作为结构选择之外的独立证据 | 具体每次人工/工具查看无法完全恢复 |
| validation 120 | 原6轮完整验证、当前24候选512预算比较 | 同上 | 明确用于检查点选择；不作独立最终测试 | 当前仍在运行，尚未冻结最佳检查点 |
| 旧test 200 | 原选定Actor测试；导航/MPPI变体评测 | 未发现作为训练输入的证据 | 系统变体比较/诊断已发生；据此调参的因果链未知 | 作者声明未查看、未据此调整；其他会话/工具历史仍未穷尽 |
| reward诊断示例 | 已训练轨迹中选择的成功/停滞各8例 | 来自训练交互 | 奖励与训练行为诊断 | 选例不构成独立评测样本，不能估算总体成功率 |

本次没有为了论文结论查看旧test的成功率、排名或收益数值；登记读取仅限身份、用途和场景信息。

## 3. 母图、种子和起点

### 3.1 母图身份

母图目录mother-manifest.json的SHA256：

~~~text
4aabf05d836c9bd6cbd5824c5b9e65d0f619dc1fc53acf8be0cc73642d39492d
~~~

- train/validation/test母图数为64/24/24；三组ID及structure_signature交集均为空。
- quick的20个洞穴场景覆盖20个validation母图；完整validation 60场洞穴覆盖24个母图；test 100场洞穴覆盖24个test母图。
- 训练资格64场覆盖64个train母图。不同几何种子、起点或传感器仍可能属于同一母图，后续划分须保留母图分组。
- 记录中的qualification物理地形SHA在validation与test间无交集。这是既存资格证据的比较，不是本轮重新生成、重新计算全部地图的结果。

### 3.2 在线训练种子与已观察记录

冻结源码course_two_simple.py中的fresh_scene默认split为train；新场景种子规则为：

~~~text
((config.seed + 3) << 32) + (TASKS.index(task) << 24) + ordinal
~~~

course_two_rebalance.py的scene_episode_v1分支调用该生成器；洞穴通过select_mother(split, ordinal)选取对应分组母图，序号按组内母图数取模。源码路径、SHA及行号摘录见证据。历史不同分支的种子规则也被保留，不能只凭一条公式断言所有历史episode均遵循同一路径。

本次留存metrics.jsonl共有704行，仅提取到15个唯一场景（moon 10、cave 5）。这15场种子与固定validation/test种子无交集；5场洞穴母图均属于train组。它们的实际起点、种子、地形SHA及场景描述已存入证据，结论仅覆盖这15场。

### 3.3 起点与运行身份

CSV为quick、原validation、旧test、512 validation各保存一轮对应checkpoint/runtime的逐场policy_start_pose；不是所有检查点的全部起点。64场资格清单的实际pose未另行抽取，标为NOT_CAPTURED，保留seed与start_mode。

对原block24验证和当前block20的512验证所保存记录：

| 项目 | 相同场数 / 120 |
| --- | ---: |
| reference_id | 120 |
| 起点x/y（记录数值完全相等） | 120 |
| 起始yaw（记录数值完全相等） | 32 |

原runtime为c8a6f92a56a2613ef23deff72ea34185f3264ce4968d4e8720be1659044f4a50，当前为35bd3876eb4bf4747d8751893f4ea87288e26de052cba1b6781457622b348dac。差异原因未在本次追查，不能声称两轮仅改变预算；P2/P3应以当前冻结实现为准，检查各候选之间条件是否一致。

## 4. 实际机器使用账目

观察范围为主根目录下11个experiment目录、当前512目录，以及明确的前代几何来源目录。下列数字是已存逐场结果文件数，可能包含缓存/复用，不代表本次新增执行次数、有效完成场数或独立样本数。0表示观察时未发现结果。

| 目录（完整前缀见证据） | 标签与记录数 |
| --- | --- |
| checkpoint-validation-512-20261019 | 16个检查点各120；block16已存3；合计1923 |
| experiment-20261014 | 24个quick各40；6个validation各120；final-selected-test 200 |
| experiment-execution-repair-20261018 | post-repair-test：200 |
| experiment-mppi-clearance-15mm-20261018 | mppi-clearance-15mm-test：194 |
| experiment-mppi-clearance-2cm-20261018 | 未发现对应逐场结果 |
| experiment-mppi-optimized-20261018 | mppi-test：1；mppi-test-sync2：51 |
| experiment-mppi-path-follower-20261018 | mppi-path-follower-test：57 |
| experiment-mppi-rolling-20261018 | mppi-rolling-test：54 |
| experiment-mppi-soft-no-refine-20261019 | mppi-soft-no-refine-test：200 |
| experiment-mppi-stationary-20261018 | mppi-stationary-test：29 |
| experiment-navigation-adopted-20261018 | adopted-test：0 |
| experiment-navigation-adopted-20261018-isolated | adopted-test：200 |
| experiment-entry-curriculum-20261014 | block-01-quick：40；block-02-quick：40；block-03-quick：40 |

- 11个experiment目录中的quick/validation/test清单字节SHA均与原清单相同；清单准备完成与实际产生结果分开登记。
- experiment-navigation-adopted-20261018的adopted-test为0，不能写成执行了200场；isolated目录另有200份结果。
- 2cm净空变体未发现逐场结果，不能写成已完成评测或已采用。
- 当前512快照记录1923份结果，不是最终完成进度。当前目录只发现validation清单，不据此保证过去其他目录未用过test。
- 附件保留每个目录的label、case ID集合、样例proof、Actor/runtime/config身份和时间，不复制性能结论。

## 5. 前代来源与几何资格复用

geometry-evidence-reuse.json明确记录来源：

~~~text
/mnt/data/WS/lunar-navigation/.runtime/region-relation-v2/experiment-entry-curriculum-20261014
~~~

回执登记384场资格证据复用，即64场训练资格、120场验证、200场测试；quick是子集，不再加40。回执说明沿用原fixed proof和manifest字节，不重标原runtime。前代来源目录另存3轮quick，每轮40场。因此不能把这些清单描述为当前结构首次生成、首次接触的数据。回执的几何不变声明属于历史来源说明；本次未重新执行资格实验。

## 6. 人工声明、未决项与后续动作

### 6.1 作者声明

2026-10-20，针对旧block20的200场test是否被查看、是否据此调整模型、奖励、导航/控制、参数或选模规则，作者答复：

> 没有看过，也没有调整过

证据类型为作者声明。机器侧已观察到的系统变体评测记录继续保留；不由此反推作者曾查看，也不把作者声明扩展为所有自动化会话都未读取结果。

### 6.2 后续安排

| 项目 | 责任/阶段 | 处理要求 | 阻塞本次用途登记 |
| --- | --- | --- | --- |
| 旧test的自动化/其他会话结果访问与选择因果 | 作者/Codex，P2数据协议 | 能定位到记录再补链；未知保持未知；暂按历史系统诊断批次管理 | 否，已显式登记不确定性 |
| 独立测试批次资格 | 作者，P2-01及正式实验协议 | 冻结研究与系统后决定保留为历史评测或另建隔离测试；不得重命名掩盖使用史 | 否，P0-02不批准最终测试 |
| 全量训练场景归档缺口 | Codex/作者，复现与实验协议 | 查找额外episode/manifest存档；后续训练记录逐场身份；没有证据不写全量无泄漏 | 否，缺口已记录 |
| 起始yaw与跨runtime条件差异 | Codex/作者，P2/P3 | 当前运行各候选条件一致性另核验；跨轮比较清楚标注系统和起点差异 | 否，不在本次修改运行 |
| 最佳检查点 | 正在运行的选优，P3 | 依最终selection与完整分母验收，不依据旧test作新选择 | 否 |

## 7. 附件、验收与边界

- [逐场用途与身份表](data-use-cases.csv)：544行清单成员记录，包含重复关系；按batch/case_id定位。
- [结构化证据](data-use-evidence.json)：清单完整spec、母图目录、来源SHA、13个目录的使用记录、起点proof、作者声明、冻结源码与限制。
- [训练与当前评测身份盘点](../records/baseline-inventory.md)：P0-01结果与当前来源链。
- 原始采集暂存于论文web/.local/p0-02-20261020，已忽略；正式登记依靠本节附件，不依赖临时文件。

本项验收范围为来源与使用历史登记。各批已知用途、未核实用途和来源均可追溯；复用与不确定性保持显式，不将已参与开发/选择的数据改名为新独立测试。本次未新增或运行测试、训练、评测、ROS验证；未改变远程进程、配置、模型或数据。论文实验质量、统计结论、独立性认证和目标平台验收仍按后续任务执行。
