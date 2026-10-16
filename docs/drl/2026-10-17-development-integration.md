# DRL 开发归并与完整模块候选（2026-10-17）

开发基线 `0647b8272c4a9df6bffc36da6f1e9d6a97d82e02`；兼容主线 `fdcf27b0c17093e7e79b62efd995a67c83dcacc2`。专题 `chore/drl-integration-20261016` 的目录日期沿用批次编号。独立审查后由主 agent 决定回合开发线与主线；本说明不宣称已部署。

## 源码、历史与诊断处理

10 个 DRL 历史专项共 437 项核对记录（297 项与当前 canonical 精确一致、138 项旧 DRL 差异、2 项 pressure 证据）；436 个可提交路径已按原字节保存，JPG 保留在 Git 外。完整逐文件判定外置于本批次 `drl_pending_triage.json`，451 项/12 专项总历史提交见 `historical-commits.json`。旧模型、60/20/20 协议、outside/position-temperature 迁移、旧 benchmark 的 zip/task_graph_v4 假设仅保留历史，不覆盖当前 v3/v5。

| 来源分支 | 保全提交 | 路径数 | 当前处置 |
| --- | --- | ---: | --- |
| `fix/drl-full-replay-resume-20261014` | `bae8c0e51e9e173bb50a80d6b38d6c110c667fc4` | 76 | 已采用的相同字节不重复导入；旧实验保全，适用增量见下表 |
| `feat/drl-input-action-v5-20261014` | `e2b2b5bf9a65f47c9e3d51c1adb405bfe41da4c3` | 124 | 已采用的相同字节不重复导入；旧实验保全，适用增量见下表 |
| `feat/drl-region-relation-v2-20261014` | `ef7cefa805fbdba165d78e8b06e4620171c86070` | 64 | 已采用的相同字节不重复导入；旧实验保全，适用增量见下表 |
| `fix/drl-checkpoint-memory-20261013` | `4c660a303c864d209e70e0d92c044886625c6531` | 22 | 已采用的相同字节不重复导入；旧实验保全，适用增量见下表 |
| `perf/drl-message-aggregation-20261013` | `d2c965a027f0df773bb8ea20bb773ea56817a2d7` | 5 | 已采用的相同字节不重复导入；旧实验保全，适用增量见下表 |
| `feat/drl-outside-specialist-20261013` | `5123340f49a987689af98b97afb7279cd44b97ff` | 31 | 已采用的相同字节不重复导入；旧实验保全，适用增量见下表 |
| `feat/drl-position-temperature-20261013` | `d7de50d41e782052a70f73f8b012930082697d0d` | 12 | 已采用的相同字节不重复导入；旧实验保全，适用增量见下表 |
| `feat/drl-region-contraction-20261006` | `37b5964c688e895ff870c5dcc35e47b206318fd5` | 17 | 已采用的相同字节不重复导入；旧实验保全，适用增量见下表 |
| `feat/drl-region-query-20261013` | `5a0d017cea6a0792280d50999166f212df015a50` | 5 | 已采用的相同字节不重复导入；旧实验保全，适用增量见下表 |
| `feat/drl-task-frontier` | `bc8e60d7ce32b0c74ddfc7bb788c36c9417aed7b` | 80 | 已采用的相同字节不重复导入；旧实验保全，适用增量见下表 |

| 定点迁入内容 | 精确来源 |
| --- | --- |
| [2026-10-13-checkpoint-memory-recovery.md](2026-10-13-checkpoint-memory-recovery.md) | `fix/drl-checkpoint-memory-20261013@4c660a303c864d209e70e0d92c044886625c6531`，正文加历史警示后保留 |
| [2026-10-13-fused-edge-message-performance.md](2026-10-13-fused-edge-message-performance.md) | `perf/drl-message-aggregation-20261013@d2c965a027f0df773bb8ea20bb773ea56817a2d7`，正文加历史警示后保留 |
| [2026-10-13-position-temperature.md](2026-10-13-position-temperature.md) | `feat/drl-position-temperature-20261013@d7de50d41e782052a70f73f8b012930082697d0d`，正文加历史警示后保留 |
| [2026-10-13-replay-snapshot-budget.md](2026-10-13-replay-snapshot-budget.md) | `fix/drl-checkpoint-memory-20261013@4c660a303c864d209e70e0d92c044886625c6531`，正文加历史警示后保留 |
| [2026-10-13-outside-specialist.md](2026-10-13-outside-specialist.md) | `feat/drl-outside-specialist-20261013@5123340f49a987689af98b97afb7279cd44b97ff`，正文加历史警示后保留 |
| [2026-10-13-outside-specialist.md](../superpowers/plans/2026-10-13-outside-specialist.md) | `feat/drl-outside-specialist-20261013@5123340f49a987689af98b97afb7279cd44b97ff`，正文加历史警示后保留 |
| [2026-10-13-region-conditioned-query.md](2026-10-13-region-conditioned-query.md) | `feat/drl-region-query-20261013@5a0d017cea6a0792280d50999166f212df015a50`，正文加历史警示后保留 |
| [2026-10-14-input-action-v5.md](../superpowers/plans/2026-10-14-input-action-v5.md) | `feat/drl-input-action-v5-20261014@e2b2b5bf9a65f47c9e3d51c1adb405bfe41da4c3`，正文加历史警示后保留 |
| 生命周期回归 | `fix/drl-checkpoint-memory-20261013@4c660a303c864d209e70e0d92c044886625c6531`；原测试逐字节迁入，先 RED 后最小修复 |
| 奖励监看 | `feat/drl-region-query-20261013@5a0d017cea6a0792280d50999166f212df015a50`；`course_two_trial_rewards.py`、panel/dashboard 接线及 9 个原用例增量；保留当前手工流式 SHA、运行身份与错误显示 |

奖励监看工具是只读观察接口，按环境与回合累计接近辅助与总奖励。半行、缺步、日志裁剪或缺辅助字段不能伪造完整回报。面板接线不启动训练；旧 `bounded_potential.sh`、`region_query.sh` 与旧实验路径仅保留历史，当前训练仍按已资格的 `region_relation_run` 身份运行。`benchmark_sparse_message.py` 未适配流式 checkpoint，不迁作当前诊断。

## 主线兼容

六处双方修改包括 core CMake、fine traversability 头、local planning 头、起点 patch、地形物理及 wheel A*。现代 ort 合并实际只有三个文本冲突：AGENTS、起点 patch、接口契约测试。起点 patch 保留主线外围求导 `derivative_support` 和 dev `local_window` geometry；原地支持、测量接口、捕获 fixture 与主线共享 phase/cancel/deadline、async/final-arrival/rolling-controller 共存。治理采用已审查主线新规则；接口测试保留主线按成员/主题/QoS 的更强约束，不回退为计数。

完整代码候选包含 DRL Python/native、ROS PolicyMapTile/GetPolicyMap 接口、SAC/采集/恢复/导出/评测/课程、必要配置脚本测试及公共导航依赖。代码资格与模型效果独立；真实 Orin 输入适配未实现，硬件、实车、外部 TCP、GPU 训练、学习收敛均 NOT_RUN。当前完整全包 Humble 构建及按包回归证据见批次外部 `task-3-report.md`；旧历史通过数不代表本轮结果。

## 源身份

开发清单与 deployed 清单的显式命令见 [开发入口](../DRL开发入口.md)。原 manifest 保持逐字节，测试固定旧 manifest SHA 并分别核验开发 79 文件；原 changed/missing/extra 漂移检测不削弱。未部署源码与运行缓存不混用。

## 本轮验证边界

完整 Humble 新缓存构建 13/13 包通过；core 20 组/311 用例、ROS 14 组/119 用例、controller 6 组/198 用例全部通过。生命周期/恢复/课程/面板/源清单定向 50 项通过；两个 demo 包分别 21 与 114 项通过。原 deployed 清单、冻结包、开发树与训练 cwd 的原 79 文件核验仍通过；本候选仅对开发清单通过，对旧 deployed 清单准确报告 course_two_trial.py 漂移。

region_contraction 工具 185 项最终通过。benchmark 测试原 fixture 缺现行强制的 architecture_version，在开发 BASE 同样失败；仅为 fixture 补显式 region_relation_v2，未放宽加载器或核心断言。首次工具全测还出现一次进程退出竞态，定向重跑和最终全测通过，不能据此宣称竞态已修复；当前 region_relation/candidate_geometry 包与入口不调用该独立旧 pressure handoff 工具。

根目录扩大测试的既有 Task7 枚举、operator 后台捕获顺序、demo NO_PATH 换候选失败均在主线 fdcf27b0 自身完整 12 包新构建中复现。新 DRL 配置与主线 isolation 白名单的冲突已按六个确切文件名补齐，保留精确集合约束。Humble 的 Float32MultiArray 对 NumPy 数组的调试断言仍要求既有 PYTHONOPTIMIZE=1 运行约定；普通解释器失败与优化解释器复验分别记录，不改消息源码或测试断言。

Docker 外部容器验证及缺少 Jazzy 的专项均 NOT_RUN；根全仓一并收集仍有不同 demo 包同名 test_evidence 的既有约束，改按包边界覆盖。历史性能、GPU、真实部署输入适配、Orin/实车、外部 TCP、策略效果和收敛未在本次补做。精确主包计数、失败复验、完整命令和日志以外部 task-3-report.md 为准，不把 colcon 对 DRL 的 Ran 0 tests 当成包测试通过。
