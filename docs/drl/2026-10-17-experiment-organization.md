# DRL 旧实验结案与入口整理（2026-10-17）

[当前开发入口](../DRL开发入口.md) · [中央台账](../分支整合台账.md) · [完整旧入口档案](archive/2026-10-17-development-entry-history.md)

## 依据与边界

用户于 2026-10-17 确认此前旧实验已经失败，本轮训练接近收尾、效果很好。
效果结论仅归属于用户确认，本任务没有重训、策略重评测或产生新的效果指标。
失败实验结束开发，不以重新验收、ahead 数量或独有提交数重新进入待合入队列；
这不否定已经吸收的算法、工具、技术修复和性能贡献，也不回退当前开发实现。

Task1 源码资格锚点：`3931e2f59e65a462ecbc91b7f8062fbc62e1d340`，已独立审查；44 项限定源码回归通过（记录于仓库外 task-1-report.md）。
本 Task2 仅整理五个文档/JSON，做 UTF-8、链接、来源和分支覆盖静态核验，没有重跑这 44 项测试。
本专题来源为 `hk/drl-experiment-organization-20261017`，最终文档 HEAD 不在自身预写。
实际文档/分支 HEAD、本地落地与收口结果分别以证据根中已生成的 `final-inventory.json`、
`landing.json`、`final-closure.json` 为准，源码资格锚点与实际分支 HEAD 分开记录。

仓库外证据根为 `/home/kai/CodexDownloads/lunar_navigation/drl-organization-20261017`：`before.json` 固定本轮 33 个分支的原 SHA/祖先关系，
`archive-refs.json` 是 root create-only 归档回执，`runtime-observation-proof.json` 及
`observed-runtime/config.json`、`identity.json`、`adoption.json` 记录观察身份。
本文的“是否 dev 祖先”均相对 before.json 的 dev `95d0b88e8ce33ff83c8bf2523e56eb2a4588bfb8`，
不等价于功能效果验收；原 SHA 指本轮 before 快照，中央台账中更早的原始 SHA/原字节保全细节继续保留。

## 当前开发和运行

唯一 dev 及保留物理路径见入口。活动训练 cwd 为 `/mnt/data/WS/lunar-navigation/.worktrees/drl-navigation-recovery-20261016`；
源码为 `/mnt/data/WS/lunar-navigation/.runtime/candidate-geometry-v3/source-domain-busy-20261017/lunar_drl_exploration`，runtime 为 `c8a6f92a56a2613ef23deff72ea34185f3264ce4968d4e8720be1659044f4a50`。
[活动 79 文件清单](active-training-source-20261017.json) 只读记录源码与已有身份，observation-only，不是部署动作。
[开发 79 文件清单](development-source-manifest.json) 仍为 `not_deployed`；Task1 后与活动包仅
`course_two_trial.py` 不同。DomainBusy 处理已承接，开发的 loaded state 生命周期修复继续保留。
[旧 formal 清单](formal-runtime-source-manifest.json) 原字节保留：旧 r2 与活动包仅 parallel 不同，
旧 r2 与开发则为 parallel/trial 两文件不同。原配置、identity、adoption 的 SHA-256 见新活动清单。
未读取模型/回放内容，也不将模型、回放或训练输出收进 Git。

## 33 项完整处置

合计：active_development 1、protected_running_source 1、formal_milestone 1；
closed_failed_experiment 17、closed_technical_source 11、completed_integration_source 2。
后三类合计 30 个归档引用，各指向原 before SHA；原分支没有删除或移动。
活动 dev、受保护运行来源与 formal 里程碑不作为失败实验归档。

|分支身份|本轮原 SHA（before）|是否 dev 祖先|archive ref|实际处置|
|---|---|---|---|---|
|`dev/drl-exploration`|`95d0b88e8ce33ff83c8bf2523e56eb2a4588bfb8`|是|不归入失败实验归档|唯一活动开发线；Task1 源码资格锚点，实际落地见证据根 landing.json|
|`docs/drl-loop-learning-design`|`3d3513cc90ca8e38dfeee2e3598efa18f6205626`|否|`refs/archive/drl-closed/20261017/docs/drl-loop-learning-design`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`docs/drl-region-query-20261013`|`0fa56fd65a0deca292c4edcc930b9dae44c84a68`|否|`refs/archive/drl-closed/20261017/docs/drl-region-query-20261013`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`feat/drl-bounded-potential-20261012`|`ab2c1faa637faeec1e953db8256bd39a7cb7c315`|否|`refs/archive/drl-closed/20261017/feat/drl-bounded-potential-20261012`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`feat/drl-exploration-complete-design`|`a0f190c88f1a500fa71b13d65ff52eeec1cfb467`|否|`refs/archive/drl-closed/20261017/feat/drl-exploration-complete-design`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`feat/drl-exploration-training`|`c6473ae70eb4a0297d4cd975ca88346ea7597d89`|否|`refs/archive/drl-closed/20261017/feat/drl-exploration-training`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`feat/drl-factorized-entropy`|`96778aeb99f6bb1319f45c52738b1361dd477d55`|否|`refs/archive/drl-closed/20261017/feat/drl-factorized-entropy`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`feat/drl-formal-version-20261016`|`ea20a3e9723398ed449906e20477f45da7161cd2`|是|不归入失败实验归档|正式源码里程碑；保留原 SHA，不作为第二条 dev 或活动 runtime|
|`feat/drl-input-action-v5-20261014`|`e2b2b5bf9a65f47c9e3d51c1adb405bfe41da4c3`|否|`refs/archive/drl-closed/20261017/feat/drl-input-action-v5-20261014`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`feat/drl-integration-20261012`|`31a4eacce515d3314e99ecb82f087dddc3cf9572`|是|`refs/archive/drl-closed/20261017/feat/drl-integration-20261012`|已完成整合来源结案；保留已吸收贡献与原来源，不再待整合|
|`feat/drl-multistep`|`32155954f14b69d69ecb06d3b73bc7f7e09d084b`|是|`refs/archive/drl-closed/20261017/feat/drl-multistep`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`feat/drl-outside-specialist-20261013`|`5123340f49a987689af98b97afb7279cd44b97ff`|否|`refs/archive/drl-closed/20261017/feat/drl-outside-specialist-20261013`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`feat/drl-position-aux`|`dcca338bc501d9d40694dc609119826f95824595`|否|`refs/archive/drl-closed/20261017/feat/drl-position-aux`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`feat/drl-position-temperature-20261013`|`d7de50d41e782052a70f73f8b012930082697d0d`|否|`refs/archive/drl-closed/20261017/feat/drl-position-temperature-20261013`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`feat/drl-region-contraction-20261006`|`37b5964c688e895ff870c5dcc35e47b206318fd5`|否|`refs/archive/drl-closed/20261017/feat/drl-region-contraction-20261006`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`feat/drl-region-query-20261013`|`5a0d017cea6a0792280d50999166f212df015a50`|否|`refs/archive/drl-closed/20261017/feat/drl-region-query-20261013`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`feat/drl-region-relation-v2-20261014`|`ef7cefa805fbdba165d78e8b06e4620171c86070`|否|`refs/archive/drl-closed/20261017/feat/drl-region-relation-v2-20261014`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`feat/drl-task-frontier`|`bc8e60d7ce32b0c74ddfc7bb788c36c9417aed7b`|否|`refs/archive/drl-closed/20261017/feat/drl-task-frontier`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`fix/drl-area-budget`|`b70e1019bbddd9bc78d83333bb48b1c572565386`|否|`refs/archive/drl-closed/20261017/fix/drl-area-budget`|历史技术来源结案（修复/性能）；保留已吸收贡献与原来源，不再待整合|
|`fix/drl-arrival-contract-20261015`|`09d316f181a050a5325ef2edf9f8aca65d8858f7`|是|`refs/archive/drl-closed/20261017/fix/drl-arrival-contract-20261015`|历史技术来源结案（修复/性能）；保留已吸收贡献与原来源，不再待整合|
|`fix/drl-checkpoint-memory-20261013`|`4c660a303c864d209e70e0d92c044886625c6531`|否|`refs/archive/drl-closed/20261017/fix/drl-checkpoint-memory-20261013`|历史技术来源结案（修复/性能）；保留已吸收贡献与原来源，不再待整合|
|`fix/drl-coverage99-success`|`fe9cab83f6c7a87d41a62d9d763d5594d94e7930`|是|`refs/archive/drl-closed/20261017/fix/drl-coverage99-success`|历史技术来源结案（修复/性能）；保留已吸收贡献与原来源，不再待整合|
|`fix/drl-evaluation-budget`|`4123c1836d0ea0437b7de1548ba6ccf6b521c1ed`|否|`refs/archive/drl-closed/20261017/fix/drl-evaluation-budget`|历史技术来源结案（修复/性能）；保留已吸收贡献与原来源，不再待整合|
|`fix/drl-full-replay-resume-20261014`|`bae8c0e51e9e173bb50a80d6b38d6c110c667fc4`|否|`refs/archive/drl-closed/20261017/fix/drl-full-replay-resume-20261014`|历史技术来源结案（修复/性能）；保留已吸收贡献与原来源，不再待整合|
|`fix/drl-navigation-recovery-20261016`|`0647b8272c4a9df6bffc36da6f1e9d6a97d82e02`|是|不归入失败实验归档|受保护活动运行来源；cwd/冻结源/进程原位保护，分支 SHA 不等于 runtime|
|`fix/drl-stagnation-budget`|`b79fa5d547284cfea99b158422eb5802576d03fe`|是|`refs/archive/drl-closed/20261017/fix/drl-stagnation-budget`|历史技术来源结案（修复/性能）；保留已吸收贡献与原来源，不再待整合|
|`fix/drl-undiscounted-completion`|`fda3a31e8ddf3412298333b8316681ebc0bab9b5`|是|`refs/archive/drl-closed/20261017/fix/drl-undiscounted-completion`|历史技术来源结案（修复/性能）；保留已吸收贡献与原来源，不再待整合|
|`perf/drl-frontier-performance`|`d9207e00b6a455568a6d04682f5142d746ba1828`|是|`refs/archive/drl-closed/20261017/perf/drl-frontier-performance`|历史技术来源结案（修复/性能）；保留已吸收贡献与原来源，不再待整合|
|`perf/drl-message-aggregation-20261013`|`d2c965a027f0df773bb8ea20bb773ea56817a2d7`|否|`refs/archive/drl-closed/20261017/perf/drl-message-aggregation-20261013`|历史技术来源结案（修复/性能）；保留已吸收贡献与原来源，不再待整合|
|`perf/drl-observation-cache`|`3e98dc27598f77584235ab8323eae3224d72822e`|否|`refs/archive/drl-closed/20261017/perf/drl-observation-cache`|历史技术来源结案（修复/性能）；保留已吸收贡献与原来源，不再待整合|
|`test/drl-coverage-completion`|`96f8ba1f50806f2c640a4de72c5e13b04a91bcb7`|是|`refs/archive/drl-closed/20261017/test/drl-coverage-completion`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`test/drl-loop-learning`|`d5de79c0b157506dc6ea59078372a1415e94d1c9`|否|`refs/archive/drl-closed/20261017/test/drl-loop-learning`|旧失败实验结案（用户确认）；保留已吸收贡献与原来源，不再待整合|
|`chore/drl-integration-20261016`|`d452677ae0c681dc865bac35a4092beab3034f9b`|是|`refs/archive/drl-closed/20261017/chore/drl-integration-20261016`|已完成整合来源结案；保留已吸收贡献与原来源，不再待整合|

## 后续路径与保留代价

代码专题从唯一 dev 派生，在独立工作树实现，完成适用验证与审查后回 dev；成熟模块再按公共接口资格晋级主线。
纯参数实验在同一已提交代码下使用独立配置和输出，不另开算法开发主线。
训练自然结束后，再单独保存模型、最终评测报告及是否部署的决定；本轮不预判胜出或实车资格。

本任务不切换源码或配置，不启动/停止/重启进程，不清理原分支、83 个原工作树、未提交字节或运行产物。
原历史文档仍原位保留，仅完整旧入口逐字另存，避免多个日期段争夺当前入口。
保留全部历史来源和产物会继续占用空间；archive 是恢复定位，不表示已删除或全量备份非 Git 训练数据。

审查、落地、运行保护和备份分别以外部回执记录实际结果，不由本文预报成功：

- 证据根 `final-inventory.json` 记录实际 refs/HEAD，`landing.json` 记录本地落地，`final-closure.json` 记录审查和最终收口。
- 同根 `protection-before-landing.json`、`protection-after.json` 记录落地前后原 refs、工作树身份/dirty 字节、
  旧 r2 与活动源各 79 文件、config/identity 及关键 PID/start_ticks 的实际核验；自然退出如实记录，不为核验而重启。
- `D:/CodexDownloads/lunar_navigation/drl_organization_20261017/complete-backup-verification.json`
  记录独立 bare 恢复库、自包含 bundle 的实际校验与备份边界。

结果以已生成回执为准；源码清单或本文自身不证明落地、运行保护或备份成功。本专题不涉及推送或创建 PR。
