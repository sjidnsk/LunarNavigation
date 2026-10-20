# 论文资料库

本目录用于管理项目后续多篇论文的研究资料。每篇论文使用独立目录，共享文献库与模板放在 `shared/`。

## 当前论文入口

- [图强化学习探索策略论文工作区](graph-reinforcement-learning-exploration-policy/README.md)
- [实时登记的研究进度总览](graph-reinforcement-learning-exploration-policy/02-research-design/PROGRESS.md)
- [步骤安排与执行章程](graph-reinforcement-learning-exploration-policy/02-research-design/WORKPLAN.md)
- [详细工作清单](graph-reinforcement-learning-exploration-policy/02-research-design/CHECKLIST.md)

进度总览在任务登记后更新，不自动读取训练或评测进程。

## 目录约定

- `shared/references/`：跨论文共用的文献元数据与阅读索引；具体论文的阅读笔记放入对应论文目录。
- `shared/templates/`：可复用的写作、实验记录和投稿模板。
- `graph-reinforcement-learning-exploration-policy/`：第一篇论文的工作区，主题为图强化学习探索策略网络。
  - `01-literature/`：文献检索记录、论文阅读笔记与相关工作素材。
  - `02-research-design/`：研究问题、贡献边界、假设和整体方案。
  - `03-method/`：方法描述、网络结构、状态/动作/奖励定义及算法细节。
  - `04-experiments/`：实验协议、配置和逐次运行记录。大规模原始数据、模型权重、缓存和运行产物应放在仓库外的受控存储中，并在记录中保留路径与身份信息。
  - `05-results/`：分析脚本说明、统计汇总、表格数据及结果解释；结论需区分开发集与独立测试集。
  - `06-manuscript/`：论文正文、章节草稿、摘要及参考文献整理。
  - `07-figures/`：可复现图表的源文件与供稿件使用的导出版本。
  - `08-submission/`：目标期刊/会议要求、投稿清单、投稿信和审稿往来。
  - `99-archive/`：明确废弃或被替代的方案与版本，保留必要来源说明。

## 命名建议

新增论文目录使用简短、稳定、全小写的英文 kebab-case 主题名；不要把年份、投稿轮次或临时状态写进目录名。按项目正式设计基线，第一篇论文资料不得把训练开发评测直接表述为独立论文测试结果；当前设计入口见 [`docs/superpowers/specs/2026-09-12-drl-exploration-redesign.md`](../../docs/superpowers/specs/2026-09-12-drl-exploration-redesign.md)。
