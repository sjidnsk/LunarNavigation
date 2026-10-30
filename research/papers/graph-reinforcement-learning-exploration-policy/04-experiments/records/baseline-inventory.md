# 训练与当前评测身份盘点

## Material Passport

- 任务：P0-01；论文：图强化学习探索策略；执行者：Codex。
- 用户授权：查询已连接的远程台式训练信息，并完成第一项盘点任务。
- 运行/文件快照窗口：2026-10-20T19:00:44.962730+08:00 至 2026-10-20T19:07:32.028564+08:00，Asia/Shanghai。
- 历史回执补充核对与报告整理完成于：2026-10-20T19:14:54.644416+08:00。
- 工作性质：已有工件及进程的只读溯源盘点，不构成实验重跑、效果复现或贡献认证。
- 结构化证据：[baseline-inventory-evidence.json](baseline-inventory-evidence.json)。以下记录对应training、evaluation、actors、source_documents字段。
- 范围：KAI主机的本批训练、24检查点及当前验证批次；远程未写文件、未启停运行、未加载GPU模型。

## 1. 结论与用户说明核对

用户说明为“远程台式训练完成，当前评测选最佳检查点”。现场观察到：

1. 原训练state为complete、completed_blocks=24，run_id为candidate_geometry_v3-7f43c67dbc311861d6f9085a。
2. 持久累计采集198,525条、梯度更新49,152次；24份Actor来自同一训练种子20261014的不同预算块。
3. 当前比较run_id为checkpoint-validation-512-20261019，仍在运行，计划24×120=2,880场验证评测，最终selection.json尚未产生。
4. 24份评测Actor与identity声明、原训练文件分别校验SHA256，全部匹配。
5. 最终训练与当前评测runtime均按实际Python、terrain扩展、原生导航及控制器文件重算匹配；冻结评测包607文件全部匹配。
6. 旧训练流程已对block-20执行200场final-selected-test，必须保留这段数据使用历史。

本报告验收身份与证据盘点。P3的最终选模、独立测试资格、统计结论和部署验收仍由后续任务处理。

## 2. 路径与环境

| 项目 | 实际位置或现场记录 |
| --- | --- |
| 主机 | KAI，SSH别名kai-10-63-250-129 |
| 原训练根 | /mnt/data/WS/lunar-navigation/.runtime/candidate-geometry-v3/experiment-20261014 |
| 最终训练冻结Python包 | /mnt/data/WS/lunar-navigation/.runtime/candidate-geometry-v3/source-domain-busy-20261017/lunar_drl_exploration |
| 最后一次训练cwd记录 | /mnt/data/WS/lunar-navigation/.worktrees/drl-navigation-recovery-20261016 |
| 训练配置、身份、协议 | 原训练根/config.json、identity.json、protocol.json |
| 训练计数与运行记录 | 原训练根/run/run.json、progress.json、course-summary.json、metrics.jsonl |
| 训练日志 | 原训练根/logs/block-00.log至block-24.log |
| 模型 | 原训练根/actors/block-01.pt至block-24.pt |
| 完整续训检查点 | 原训练根/run/resume.pt，7,941,190,849字节，未加载及全量hash |
| 当前评测根 | /mnt/data/WS/lunar-navigation/.runtime/candidate-geometry-v3/checkpoint-validation-512-20261019 |
| 当前评测冻结stage | /mnt/data/WS/lunar-navigation/.runtime/checkpoint-validation-512-evaluation-humble-20261019 |
| 评测进程cwd | /mnt/data/WS/lunar-navigation/.worktrees/mppi-controller-20261018；cwd不代替实际冻结源码身份 |

现场为Ubuntu22.04系列、Linux6.8.0-138-generic、Intel i7-14700KF、RTX4080 SUPER。最后run.json记录28逻辑CPU、内存67,221,114,880字节。当前venv版本为Python3.10.12、torch2.10.0+cu128、numpy1.26.4；评测控制器ROS_DISTRO=humble。当前包版本为现场观察，未据此推断全部历史训练期间安装版本始终不变。

## 3. 模型、配置与源码身份

| 项目 | 核对结果 |
| --- | --- |
| 模型/输入 | candidate_geometry_v3 / task_graph_v5；policy_graph_mode=a2 |
| 模型规模配置 | width128、heads8、layers6、remote_frontier_positions32 |
| 学习器语义 | candidate_graph_sac_v3，来自run.json |
| 批次与优化 | batch64、microbatch8、lr1e-5、gamma1.0、polyak0.005、return_steps1 |
| 温度配置 | initial_alpha5e-5、maximum_alpha1e-4、maximum_position_alpha1e-3、温度lr0.003 |
| 训练采集 | 8环境；目标RTF30；积分步长0.05秒；观测2Hz |
| 双任务与采样 | moon_outside / cave_inside；scene_episode_v1，whole_cave_v1，C3 |
| 场景传感器 | distance_sensor_10_30_v1；config基础sensor为10m/90°，逐场实际值按场景规则生成 |
| 月表入场课程 | near_to_far_v1；末期entry_curriculum.stage=2 |
| 记录的Git锚点 | run/run.json.code_revision=0647b8272c4a9df6bffc36da6f1e9d6a97d82e02；实际身份以下方冻结文件为准 |
| 最终训练runtime | c8a6f92a56a2613ef23deff72ea34185f3264ce4968d4e8720be1659044f4a50 |
| 当前评测runtime | 35bd3876eb4bf4747d8751893f4ea87288e26de052cba1b6781457622b348dac |

最终训练Python79文件与仓库历史清单docs/drl/active-training-source-20261017.json逐项匹配。按实际terrain扩展、导航及控制器补齐后重算runtime，共94条文件身份，匹配训练identity。评测Python81文件及对应运行组合共99条身份，重算匹配；封存607文件、共13,045,135字节全部匹配。

核对仅读取文件内容，没有import训练包、加载模型或执行实验。训练与评测配置SHA各自绑定实际用途，24份权重相等性连接二者的模型来源。

### 关键文件字节SHA256

| 文件标识（完整路径见结构化证据） | SHA256 |
| --- | --- |
| train_config.json | 8d159d7e55fcb42aa23aef21900595199d8371f809543b3b83ef2ef1618deefb |
| train_protocol.json | 1349a1780a968739e5dbd27543e26f336aad2dc5c9b79e09977efe76980f8999 |
| train_validation.json | 5bc7c7676fb6537888fc333921e2f5568928123861194574a4028b46a7d1fcad |
| train_identity.json | 88874af292129930e92ea02718d195e992b5c5f12f02143a37c0195ace8b98a8 |
| eval_config.json | 16f383c1691e07735783a153f003b40b8464115db95d9e3145cc6208b32273a7 |
| eval_protocol.json | 1349a1780a968739e5dbd27543e26f336aad2dc5c9b79e09977efe76980f8999 |
| eval_validation.json | 0153cd1762286fbe1df8fa2f55069f0cdf5ec7517c14006d5f4f808eed5276ad |
| eval_identity.json | cb8874c82b5d6b206774949eeb5c5b76dd3e062c863ed98f9217313e40f12555 |
| eval_comparison-protocol.json | 28de6d89daf6d8840a12441d874fd5395367b584c110db4241b5e174e8a4cc65 |

## 4. 初始化、训练预算与结束记录

初始化回执为from_scratch=true、resume=false、parent_checkpoint=null、seed20261014、初始replay_count=0、updates=0。初始模型摘要为c49fd3b0820adda6993b28c8f22357a24f1d05b7253bdab1b28f7cad58c87cb9。原启动回执时间为2026-10-14T22:00:54+08:00。

| 预算组成 | 持久记录 |
| --- | --- |
| 补采 | 893条 |
| 预热 | 1,024条；block-00末尾累计1,917、更新0 |
| 正式训练 | 24×8,192=196,608条 |
| 持久采集总计 | 893+1,024+196,608=198,525条 |
| 梯度更新 | 24×2,048=49,152次 |
| 按任务采集 | moon_outside153,144；cave_inside45,381；合计198,525 |
| 进入训练批次的样本次数 | 两任务各1,572,864，总计3,145,728=49,152×64，包含重放重复采样 |

block-24.log末尾明确记录“已保存并停止”、累计198,525及更新49,152；其mtime为2026-10-17T20:57:11.799375+08:00。Actor24文件mtime为2026-10-17T20:58:04.093298+08:00。这些是文件更新时刻，不代替精确进程退出回执。

包含旧末次测试的整个旧流程state完成时间为2026-10-18T00:42:12+08:00。末期run.json的wall_s约6,667秒，仅属于最后一次子训练调用；全流程包含跨块评估、暂停恢复与故障处置，全部训练耗时需另行汇总。持久采样数也不代表全部失败、回退及丢弃计算成本。

## 5. 恢复链与执行语义变更

本run先随机初始化，后续从本run的resume.pt恢复。六条回执按时间排列，old_runtime/new_runtime从初始d661ecdb16ac连续衔接至最终c8a6f92a56a2，链条逐项核对闭合。末期run.json.observations.resume=true，保留恢复模型摘要及replay快照摘要。每个时期的源码、配置及结果各自归属，训练过程包含以下已定位变更：

| 时间 | 回执目录 | runtime前缀 | 回执SHA256 |
| --- | --- | --- | --- |
| 2026-10-15T01:13:13+08:00 | candidate-geometry-error-policy-20261015 | d661ecdb16ac → e295dbc48694 | 17f04e81361bb5d2adffaaa114fd6c6da566627af76bade583e7435107e9c296 |
| 2026-10-15T19:48:03+08:00 | navigation-cutover-20261015 | e295dbc48694 → 33f5ca1470b1 | 481c825af71d6bb6b6215d913cd9f26e8504047dcc722beaa9a9c7e601b70417 |
| 2026-10-15T22:18:37+08:00 | arrival-contract-cutover-20261015 | 33f5ca1470b1 → 98132f1e6e8f | 44cadbbd52ed3016961f960be4b747a4ce3032cffdc9f07e46e6937c4db4974c |
| 2026-10-16T11:19:02+08:00 | navigation-budget-cutover-20261016 | 98132f1e6e8f → 1c57d3118181 | d5a890b19a4ceb1bfeb787a1862199e5036c449ac1a9e88eb6457a124cd16449 |
| 2026-10-16T18:14:39+08:00 | navigation-recovery-cutover-20261016 | 1c57d3118181 → 8470d67be1e6 | efa9e456f23b515edbfd59cf5b8d62b25b4458c08de81498411a4b96c6160c41 |
| 2026-10-17T01:07:08+08:00 | observed-runtime | 8470d67be1e6 → c8a6f92a56a2 | 20bdf74cc7b0674839390501de6889fd4deb005dfc4990f5f6ec4c0cb3dcb37c |

- 到达条件从位置0.05m/朝向约0.2618rad迁至0.1m/0.05rad；记录说明历史replay保留且未重新标注。
- 导航预算、watchdog与导航恢复均有单独adoption；DomainBusy处理形成最终训练runtime。
- 原state保留诊断DDS域重叠恢复：退回检查点为59,261转移/14,336更新，另有49转移/11更新未持久化。block-07退出-9也保留在历史错误字段。
- 这些预算分开登记，不把已知被丢弃计算合并成新的独立训练重复。
- 旧transient训练服务已卸载，最初逐字ExecStart未现场恢复。启动README、launch-receipt、冻结模块入口、最终run_python.sh和恢复回执均已定位，详见结构化证据historical_entry、final_wrapper。
- 原始初始化证据与最后resume证据分别覆盖相应阶段，不据单一末期run.json回写整个历史过程。

## 6. 候选检查点清单

原训练和当前评测目录各24份同名Actor，每份4,348,841字节。下表核对列为“与评测identity一致 / 与原训练文件一致”。完整路径和mtime见结构化证据actors字段。

| 块 | 文件 | 字节 | SHA256 | 核对 |
| --- | --- | --- | --- | --- |
| 01 | block-01.pt | 4,348,841 | 6951491300d144cb9e170859804707eb3613bd0fafd956ae205549eab0120f7d | 匹配 / 匹配 |
| 02 | block-02.pt | 4,348,841 | 93931c85f85c3528a9907480c70b944bb83050a7ccbb0c8c01a1b9e3387f7866 | 匹配 / 匹配 |
| 03 | block-03.pt | 4,348,841 | 964f83abd0759ee9ae6f5a4d66ce5a17073e06225901c09b8fcb08bd12e1ecd1 | 匹配 / 匹配 |
| 04 | block-04.pt | 4,348,841 | fc2312d87968bf84becbc81f55dfa4a7784e9539211c973bffc9f1e9d664d0a0 | 匹配 / 匹配 |
| 05 | block-05.pt | 4,348,841 | 38801cbfc3b77dd6d3f95fc65337ab85bc2f001799a5d4b6e64f3fd8c0bb58e5 | 匹配 / 匹配 |
| 06 | block-06.pt | 4,348,841 | 736d7badc7d91bdefd5c6ad969f2bf3ca3d2731e23f3b61121449de3da6b8faf | 匹配 / 匹配 |
| 07 | block-07.pt | 4,348,841 | 615a4bf3662d4a80740125d3253075ab2b858132425a183a0c64e8298f77042a | 匹配 / 匹配 |
| 08 | block-08.pt | 4,348,841 | 692add30aa469649a623965bf19fc41c214df83ddd4d1fa54c132bffddc0f432 | 匹配 / 匹配 |
| 09 | block-09.pt | 4,348,841 | 988109f96ae703ae802b6ccfb380fdae0134194e701d8e791833cc13d4cbb006 | 匹配 / 匹配 |
| 10 | block-10.pt | 4,348,841 | a6af13f4bf3dac91a26069e82ace466c96768e5016c9d8b833384587c8011336 | 匹配 / 匹配 |
| 11 | block-11.pt | 4,348,841 | 347f8a467eb4753b8888b0c977f0fdb33e268951738bf12c551759ff86d80ebe | 匹配 / 匹配 |
| 12 | block-12.pt | 4,348,841 | 0c97d194bbe14a514cff383f0246be2e3fa468c00f095596c06c2ac1a1f94c51 | 匹配 / 匹配 |
| 13 | block-13.pt | 4,348,841 | 0718b9bb5cefc320b80859e3b63ac9ec992e5a1318c057d9e74823ee33d3da8a | 匹配 / 匹配 |
| 14 | block-14.pt | 4,348,841 | 1033619880427c44c295e5fc5b031decb8617c9ce20c3e3ec7657d1003fd5167 | 匹配 / 匹配 |
| 15 | block-15.pt | 4,348,841 | 6d6e184dcda2b98e25a1936456028ffd204379953c82ba19e95888a5f0c0247e | 匹配 / 匹配 |
| 16 | block-16.pt | 4,348,841 | def207abafe9f77eb8a7eafb5712c1aa6db5d3a9f780584c1468d19f1b7f123e | 匹配 / 匹配 |
| 17 | block-17.pt | 4,348,841 | 7d9a34d8a3dccdb73c4911071d99cec5a4b68bbb2e18f3fd084ed43da755ce11 | 匹配 / 匹配 |
| 18 | block-18.pt | 4,348,841 | e686319857c03164e7c1380e5c5ff8e94a2c238dfe6a7992b477955b0ae69622 | 匹配 / 匹配 |
| 19 | block-19.pt | 4,348,841 | 400401cb7110165aa7093ff2361453a153c8faa832dc916cefdc6f869bfcfd35 | 匹配 / 匹配 |
| 20 | block-20.pt | 4,348,841 | 847ae32b42e3dfb2f16446a3f9be977fa68825002f2a80d41d236473e3ff0f08 | 匹配 / 匹配 |
| 21 | block-21.pt | 4,348,841 | 7f787e076ea14eb7d6748e5d3fae9c56307a42e5aecb2c23e30cebe4a61fb9fd | 匹配 / 匹配 |
| 22 | block-22.pt | 4,348,841 | 7409b45d53a749bfbfbe86c1fe4724f378ecebc4a2d85b76ac3e60d1bbc411a3 | 匹配 / 匹配 |
| 23 | block-23.pt | 4,348,841 | 36974a70c3cb730b91b14e4ace3d052d24dc480440be9bcfe27de64b160f2f29 | 匹配 / 匹配 |
| 24 | block-24.pt | 4,348,841 | 076bc9b33a6d3789b420612d25837c353c2a210014b8f851cccc1da96e0d6d9b | 匹配 / 匹配 |

24份模型属于一个训练种子不同预算块，独立训练种子数量为1。当前选优使用这些Actor文件；完整replay/优化器恢复工件的全量封存不属于这24份Actor核对。

## 7. 当前评测入口、协议与进度快照

现场PID3719450，service为lunar-checkpoint-validation-512-20261019.service，active/running，ActiveEnterTimestamp=2026-10-19 19:06:52 CST。实际ExecStart：

~~~text
/bin/bash /mnt/data/WS/lunar-navigation/.runtime/checkpoint-validation-512-evaluation-humble-20261019/run_python.sh /mnt/data/WS/lunar-navigation/analysis-artifacts/checkpoint-validation-512-20261019/campaign.py
~~~

campaign.py字节SHA256为2ce82c693a72e693bf163ff47640c830996266c56e3ba518c2198f50d331bde2，与identity.campaign_sha一致。冻结stage/source与stage/install进入实际进程环境；仓库通用compare_checkpoints.py不作为本次现场脚本的替代证明。

| 项目 | 实际协议与运行记录 |
| --- | --- |
| 数据用途 | validation；24×120=2,880场 |
| 每场决策上限 | 512；覆盖达到99%可提前结束 |
| 场景 | 冻结validation.json，所有检查点使用相同场景及逐场传感器 |
| 控制/规划 | mppi_path_follower，步同步true，nominal_response_time0；额外硬净空0，关闭长度精化，软净空范围0.2m/权重2 |
| 并发 | 协议初始8；concurrency-change-20261019.json记录改为12；settings/state现场均为12 |
| 错误 | count_failure；失败保留在分母 |
| 公共物理AUC | block20先评估，冻结每场物理时间/路程预算 |
| 排序 | 优先零碰撞/零执行错误；再宏平均安全SR99、公共预算时间AUC及共同成功场景L99/T99 |
| 输出 | evaluation/逐场JSON、block-NN-metrics.json、comparison-progress.json，最终selection.json |

观察时点2026-10-20T19:07:32.028564+08:00：15/24检查点完成，汇总已完成1800场；当前block15完成112/120，逐场结果文件共1912份，selection.json尚不存在。

这是一次观察快照，各活动文件分别读取，不是跨文件原子事务；不推定最终排名或剩余耗时。P3保存最终选择时仍需逐项核对selection.json、实际tie-break实现、协议要求及完整分母。

## 8. 旧测试使用与未决项

原训练state已有selected_actor=block-20.pt、final_test.episodes=200和旧evaluation_label=final-selected-test。其split及manifest_sha256保存在结构化证据training.state.final_test。当前比较目录未复制test.json，仍须保留原200场测试已执行的事实，不把旧结果归入当前MPPI/512决策的运行身份。

| 编号 | 未决项与处置 | 责任 / 后续任务 | 阻塞P0-01 |
| --- | --- | --- | --- |
| I1 | 旧200场测试的使用、结果查看与后续独立验证隔离 | 作者/Codex，P0-02、P2-01 | 否，原始来源已定位 |
| I2 | 当前比较未结束，等待真实选优和模型冻结 | 当前评测/作者，P3-01至04 | 否 |
| I3 | 完整7.94GB续训检查点未全量SHA；24候选已核对 | 需要完整恢复封存时补充 | 否，当前选优不读取完整文件 |
| I4 | 全历史耗时与未持久计算量未完整汇总 | 作者/Codex，P4-05及开销说明 | 否，持久预算及已知回退已核对 |
| I5 | 最初transient训练服务的逐字ExecStart未恢复 | 后续复现材料整理 | 否，启动/源码/配置/预算/恢复可定位 |

P0-02将据历史使用事实登记数据用途，P2再确定后续独立测试安排。本报告不宣称当前论文已有未接触的独立测试集。

## 9. 验收记录与边界

- 原训练、评测、配置、源码、模型、日志与恢复回执均能按路径定位。
- 24候选双向SHA匹配；训练和评测runtime重算匹配；训练79Python与清单匹配；封存607文件匹配。
- 198,525条持久转移与49,152次更新按协议、末期运行记录和末尾日志交叉核对一致。
- 初始化、同run恢复、课程及执行语义变更已记录；用户说明和现场事实已区分。
- 未决项已明确来源、责任与后续任务，不将未核实信息标为已验证。
- 本轮未运行训练、评测、重现实验、单元测试或ROS/设备验证。当前Humble评测只作已有进程观察；Jazzy、Orin/实车NOT_RUN。
- 本项通过表示身份盘点已完成，后续选模、实验质量、数据独立性、统计和部署依原清单验收。

## 10. P0-02后续登记（2026-10-20）

旧200场使用历史已形成[数据使用登记](../protocols/data-use-register.md)。作者确认“没有看过，也没有调整过”；机器侧另有导航/MPPI变体使用同一test清单的记录，二者分列。quick 40属于validation 120子集；当前512评测复用同一场景配置，但与原验证的已存起始朝向存在差异。I1的用途盘点部分已完成，独立测试资格和实验隔离仍交P2处理；不影响正在进行的P3选模。
