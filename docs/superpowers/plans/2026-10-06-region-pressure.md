# A0 / A2 Pressure Test Implementation Plan

> **For agentic workers:** Use subagent-driven implementation with independent review. Steps use checkbox syntax.

**Goal:** 完成50–300m固定多场景A0/A2压力对照，交付实时看板、原始证据和分档结论。

**Architecture:** 复用上一轮冻结原生评测核心；只新增调度、两模式汇总和动态看板。后台调度保存完成身份和状态，按尺寸控制并发；没有训练或导航更改。

**Tech Stack:** Python标准库、现有NumPy/PyTorch/Jazzy运行时、HTML Canvas、pytest。

**Spec:** `docs/superpowers/specs/2026-10-06-region-pressure.md`

## Global Constraints

- Actor、evaluate.py、contraction.py、原导航和奖励保持前轮身份。
- 五档、每档4张图、40回合；预算256/512/960/1536/3264，同case两臂一致。
- <=100m最多2case并发，>=150m串行；不改变正在运行的正式训练。
- 错误/不完整/有效策略失败分列，不自动替换种子或覆盖目录。

## Review Focus

- 双模式配对必须与旧三模式默认行为兼容。
- 初始化失败不能成为策略覆盖率0样本。
- 恢复必须拒绝源码/Actor/配置身份改变、缺失来源和未完整目录。
- 大图输出与内存成本单列，不把Actor节省等同端到端节省。
- 当前未结束回合和已结束均值在UI明确分开。

### Task 1: 两模式汇总（benchmark agent）

- [x] 回归测试：两臂、任意case、分层、错误/缺失与来源拒绝。
- [x] `compare.py --modes baseline remote --cases ...`，增加分层JSON/CSV。
- [x] 保持三模式默认测试通过；独立审阅。

### Task 2: 实时压力看板（evaluation agent）

- [x] 新建 `pressure_viewer.html/pressure_serve.py`，`/api/overview`轻量读取协议/状态/最新完整步骤。
- [x] 动态case选择、逐场景覆盖和节点回放；汇总结束分母及错误分列。
- [x] 语法与服务检查，浏览器实际显示验证。

### Task 3: 冻结调度与完整实验（parent）

- [x] 测试预算、矩阵、身份和完整终态恢复条件。
- [x] 新建 `pressure.py`，固定协议、独立session、受控并发、日志与负载采样。
- [x] 最终来源提交 `7e6fd899` 固定后启动全部40回合；运行期间保持HEAD和冻结文件不变。
- [ ] 核对每个终态和有效配对。
- [ ] 分档汇总、曲线与压力结论；更新验证文档、UTF-8与diff检查。

### Task 4: 独立数据后处理（并行实现、独立审阅）

- [x] `pressure_plot.py`：中文分档图、20场景覆盖率曲线、配对指标；8项回归。
- [x] `pressure_resources.py`：10秒采样属于本次评测的进程树RSS/PSS；10项回归。
- [x] `pressure_report.py --watch`：60秒按实际进展更新，终态强制重算；10项回归。
- [x] `pressure_diagnostics.py`：有界流式图峰值、80/95/99阈值、零增段和计时；独立审阅问题已修复，34项回归通过。
- [ ] 40回合完成后独立审阅全部结果和身份；仅此后提交辅助代码/验证文档。

原生目标测试已通过111项、跳过1项。以上后处理不启动任何评测或训练；资源采样峰值是10秒采样下界，初期覆盖不完整。最终效率只比较双方成功的场景子集，错误与未完成值不补零。
