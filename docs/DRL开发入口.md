# 强化学习开发与训练入口

唯一后续开发线是 `dev/drl-exploration`，保留工作树
`/mnt/data/WS/lunar-navigation/lunar-runtime/.worktrees/drl-exploration-redesign`。
本轮 Task1 已审源码资格锚点为 `3931e2f59e65a462ecbc91b7f8062fbc62e1d340`；本专题仍待独立文档审查及 root 本地快进落地，
最终实际 HEAD 以本批次仓库外 `final-inventory` / `landing` 记录为准。

## 当前训练身份（2026-10-17 只读观察）

- 运行 cwd：`/mnt/data/WS/lunar-navigation/.worktrees/drl-navigation-recovery-20261016`。
- 冻结源码包：`/mnt/data/WS/lunar-navigation/.runtime/candidate-geometry-v3/source-domain-busy-20261017/lunar_drl_exploration`。
- runtime：`c8a6f92a56a2613ef23deff72ea34185f3264ce4968d4e8720be1659044f4a50`。
- 原实验根：`/mnt/data/WS/lunar-navigation/.runtime/candidate-geometry-v3/experiment-20261014`。
- 用户确认本轮训练接近收尾、效果良好；本次没有重训或重新评测策略，不新增成功率或部署结论。

保持原 cwd、冻结源、配置、模型和进程，让训练按既有计划自然结束；不切源、不重启。
[活动训练源码清单](drl/active-training-source-20261017.json) 是 observation-only 的 79 文件及身份记录。
[开发源码清单](drl/development-source-manifest.json) 同为 79 文件且 `not_deployed`：
Task1 已承接活动运行包的 DomainBusy 评测失败处理，开发相对活动包仅
`course_two_trial.py` 保留 loaded state 生命周期修复。
[旧 r2 运行清单](drl/formal-runtime-source-manifest.json) 保留原字节和历史 runtime，不覆盖活动观察身份。

从仓库根显式只读核验（脚本只读文件，不启动训练）：

```bash
python3 scripts/drl/verify_formal_source.py --manifest docs/drl/active-training-source-20261017.json --package /mnt/data/WS/lunar-navigation/.runtime/candidate-geometry-v3/source-domain-busy-20261017/lunar_drl_exploration
python3 scripts/drl/verify_formal_source.py --manifest docs/drl/development-source-manifest.json --package ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration
```

## 后续开发与结案

新代码专题从 dev 派生，在独立工作树实现和验证后回 dev；纯参数实验使用同一提交、独立配置与输出目录。
本轮 33 项身份已逐项登记：唯一 dev、受保护活动运行来源、正式源码里程碑各 1；
17 个用户确认失败的旧实验、11 个历史技术修复/性能来源、2 个已完成整合来源以 30 个 archive refs 结案。
原分支和工作树保留，已吸收贡献继续保留；独有提交数不再构成待整合队列。
训练自然结束后，再单独固化模型、最终评测报告以及是否部署的决定。

- [实验整理与 33 项明细](drl/2026-10-17-experiment-organization.md)
- [中央分支整合台账](分支整合台账.md)
- [完整旧入口原文](drl/archive/2026-10-17-development-entry-history.md)
- [前轮开发归并](drl/2026-10-17-development-integration.md)、[正式源码里程碑](drl/2026-10-16-formal-version-consolidation.md)（历史证据）
