# 强化学习工作树整合记录

日期：2026-10-12。用户授权整理强化学习工作树并整合。Git 根为 `/home/kai/WS/lunar-navigation/lunar-runtime`；当前训练仅使用 `.worktrees/drl-task-frontier`，独立整合树为 `.worktrees/drl-integration-20261012` / `feat/drl-integration-20261012`。

## 来源和处理

盘点到 10 个原强化学习工作树，其中 task-frontier 比原 dev 多 32 个提交，region-contraction 比共同基点多 3 个脚本/文档提交；两个源树分别有 80/18 个未提交文件。运行态和 Git 提交状态分别核对。

| 分支 | 原 HEAD | 未提交文件 | 处理 |
|---|---|---:|---|
| `feat/drl-exploration-complete-design` | `a0f190c88f1a` | 0 | 历史连续训练设计已收录并标注旧版本 |
| `dev/drl-exploration` | `32155954f14b` | 0 | 统一开发线；已快进至通过验收的整合结果 |
| `feat/drl-exploration-training` | `c6473ae70eb4` | 0 | 旧版训练器/接口为历史原型，当前图与 checkpoint 格式已替代；历史文档补齐 |
| `feat/drl-factorized-entropy` | `96778aeb99f6` | 0 | 分离熵已有当前实现；补齐缺失历史验证文档；原分支保留 |
| `test/drl-loop-learning` | `d5de79c0b157` | 0 | 旧模型诊断/softsign 数值实验保留原 Git 实现与历史记录，未覆盖当前模型和奖励 |
| `docs/drl-loop-learning-design` | `3d3513cc90ca` | 0 | 历史方案记录，代码版本从属旧循环实验 |
| `feat/drl-position-aux` | `dcca338bc501` | 0 | 位置辅助损失不重新启用；当前配置要求权重 0；保留历史结果与分支 |
| `feat/drl-region-contraction-20261006` | `7e6fd899a780` | 18 | 3 个独有提交已合并；17 个源码/文档改动已保存提交；1 个 JPEG 证据外部归档 |
| `feat/drl-multistep` | `32155954f14b` | 0 | 与原开发线 HEAD 相同；现有开发线已含其成果，保留分支/目录 |
| `feat/drl-task-frontier` | `6d29042db70e` | 80 | 当前 A2/课程实现及 80 个未提交文件已复制提交；原树冻结供正在运行的训练 |

所有旧分支、工作树及未提交文件保留，整合只在新增树上提交；训练目录和运行时绑定不迁移。旧代码没有盲目合并：旧训练原型接口、位置辅助损失、循环试验的模型/Q/奖励约定不能直接覆盖当前任务前沿实现。缺失的 26 份历史文档收录时标注来源分支及 SHA；旧功能源代码仍通过原分支完整可取。当前合并保留最新核心、已经采用的分离熵、当前 A2/课程代码、恢复修复、扩展预算及压力评测工具。

工作复制备份与 SHA 清单放在 `/home/kai/CodexDownloads/lunar_navigation/drl-integration-20261012`：inventory-initial.json、working-copy-manifest.json、两个源码 tar.gz、historical-documents.json、ignored-artifacts-inventory.json。旧 dev/训练工作树含被 Git 忽略的 training-output、install/log 和执行账本，这些仍保留原位，不把“git status 干净”等同于“没有数据”。导出的 JPEG 未提交，原文件保留并另复制 evidence/；验证文档指向该外部证据。

## 构建与验证

新隔离缓存 `/home/kai/.cache/lunar-drl-consolidation/jazzy-20261012` 绑定新增整合树；6 个 Jazzy 包构建成功，约 68 秒。167 个当前 DRL 实现文件与正在训练的 source 树逐字节一致；98 个源树未提交文件 SHA 未变化，训练 source HEAD 仍为 6d29042d。

完整 DRL 包及压力工具首次联合测试为 1395 passed / 1 failed / 2 skipped；唯一失败是绘图工具按训练 venv 运行时没有 Matplotlib。该工具明确要求 system Python，随后按其规定环境独立补验全部绘图测试。包代码与测试没有为此改动，运行中的训练依赖未安装或变更。去重后的全部 1396 项测试已通过，另 2 项跳过；系统绘图日志与首次环境错误均保留。已快进唯一 `dev/drl-exploration` 工作树；开发线和整合树均干净，相关源分支与目录原位保留，Git connectivity 检查通过。构建、包测试属于本机 Jazzy 证据，Humble/Orin/实车未重新验收；源码整合不宣称策略已改善。

## 完成状态

已整合并更新统一入口。旧工作树/分支按历史、实验和运行冻结状态保留；未执行删除、移动、远端推送或公共 integration/pure-planner-orin 合并。当前追加课程二继续使用原 task-frontier 源与缓存，代码内容未改变，进度仍更新。最终源文件 SHA、分支和活进程检查见外部证据目录 postflight.json。
