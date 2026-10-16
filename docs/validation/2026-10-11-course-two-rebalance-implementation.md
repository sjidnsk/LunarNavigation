# 课程二重设计实现与验收

日期：2026-10-11。工作树 `feat/drl-task-frontier`，基线 `6d29042d`。

当前状态：只执行 C3，取消 C0/C1/C2 对照。按用户在两次黑屏重启后的要求，将验收、评测和训练均降为 12 路，已保留 128 项完成记录并恢复。代码验证包括完整回归 1113 passed、2 skipped；单路线定向 65 passed；16 路资源调度 61 passed；降为 12 路及完整 fork 定向 27 passed，均经过相应独立复审。当前尚无本轮优化器更新，也没有学习改善结论。

## 原始资产

原实验 `/home/kai/CodexDownloads/lunar_navigation/drl-course-two-20261010/managed` 保留。新输出在 `/home/kai/CodexDownloads/lunar_navigation/drl-course-two-redesign-20261011`。修改前 149 个源文件、脚本和测试保存为 `implementation/source-before.tar.gz`，哈希清单同目录。block31 完整源 SHA 为 `cf58f56b1aaf5411645123ffc76b027eaefa4146391cac6ef2e8ae1fc755f6bc`。

## 改动

- C3 以显式课程版本进入，同步保留 v1 旧课、旧实验的接口与默认语义；管理进程拒绝启动 C0/C1/C2。
- 采集旧/新 50/50；学习按动作前覆盖 32/16/8/8 分层。
- C3 全局回合上限 4、批内不重复；成功终止奖励证明与最后 16 步窗口可恢复；保存时保留必要多样性，重载检查。
- 预填经验单独记录，最多 4096 条，优化器更新为零，不积攒更新欠账；C3 直接 fork 完整 block31 后自行补采，无其他组的检查点依赖。
- 新场景包括区内覆盖、区内遮挡、必要离区绕行、短程接近、实际前序形成的收尾与返回。前序只执行真实导航/观测；不写假 known mask，不产生伪行为概率。
- 首批 128 训练需求、64 开发需求先进行原生验收；原对照专用的 64 项已经取消。之后按任务持久化序号生成新种子，新版场景在任何 SAC 转移前执行同样的原生角色验收；收尾/返回额外验证真实完成路线，再重放前序交接。
- 开发评测固定为修正协议 126 项 + 64 项能力场景；先重跑 block31。C3 首段运行 2048 条/512 更新；完整状态先保存再评测，退化不加练，阶段结束等待检查，不自动进入更难传感器阶段。

## 证据

日志位于新输出的 `implementation/`。

- Replay 定向与既有回归：24 passed（`replay-regression.log`）。
- 场景几何：7 passed（`scene-green.log`）。
- 初版课程集成与既有课程/调度：22 passed（`integration-green.log`）。
- 前序与课程集成：9 passed（`prelude-green.log`）。
- 退化门槛：3 passed（`trial-green.log`）。
- 早期全包结果：1079 passed, 2 skipped；期间源文件有编辑，不作为最终冻结版本的完整验证。
- 独立审查修正后的定向验证：43 passed（`review-fix-regression.log`），含真实 collector 保存/恢复、零更新补采、慢 reset 超过旧 watchdog 以及停止回归。
- 最终全包回归：1111 passed, 2 skipped in 347.85s（`final-full-regression.log`）；退出码 0。`git diff --check` 通过。
- 原生运行暴露的 artifact 竞态修复后，全包回归：1113 passed, 2 skipped in 346.11s（`after-artifact-fix-full-regression.log`）。
- 用户取消对照后，控制器、独立 C3 完整 fork、采集恢复、回放及 artifact 等定向回归：65 passed in 17.87s（`single-c3-regression.log`）；独立复审额外运行 controller 测试 4 passed。仅调度范围精简，未再次重复完整测试。

原生探针（本地 Jazzy 仿真闭环，同一原生导航与真实传感器观测链）：

| 场景 | 接管覆盖 | 接管位置 | 前序决策 | 可执行后缀验证 |
|---|---:|---|---:|---|
| cover | 43.8545% | 区内 | 0 | 后续完整策略评测待运行 |
| approach | 0% | 区外 | 0 | 几何参考接近路线有效 |
| finish-small | 91.6940% | 区内 | 20 | 2 次导航后 99.7485% |
| return | 90.4655% | 区外 | 15 | 1 次导航后 99.8189%，返回路程 7.1508 m |

证据为 `probe-*-v2.json`。收尾/返回在验证完成路线后又重新初始化并重放冻结前序，接管条件再次通过。最终修正后的 `probe-return-v3.json` 再次通过，15 个真实目标执行结果均为 0，保存接管 hash 与完成后缀证据。探针绑定其运行时摘要；正式全目录验收绑定最终运行时，不复用这些探针作为全目录合格证。

## 边界

这是课程实现与可解性证据，不是 C3 学习改进证据。尚无新一轮学习结果、完整正式开发评测或 Orin/实体车验证。训练不改变模型结构、奖励、99% 标准、温度或 A2 导航行为。传感器范围 10–100 m、60/90/120/180/360° 的后续课程保留；本轮首先只开放 A1 的 10 m/90°。

## 独立审查与修正

独立审查发现两项 P1：keyword-only sampler 调用错误，及新版前序 reset 时间预算超过旧 collector 180 秒 watchdog。已经修正调用，添加真正 collector → capture → restore 的零更新补采集成测试；按版本区分普通动作、冻结前序（1260 秒 watchdog）和在线原生验收（2460 秒 watchdog）。验收本身有 1200/2400 秒期限，失败会显式停下，不再由恢复机制悄悄跳过慢场景。另补逐目标执行结果、费用、接管状态 hash，并在开发评测记录与验收状态是否一致。hash 不假定物理 ROS 重放逐位一致。

第一轮完整回归：1106 passed, 2 skipped, 1 failed；失败是原测试 reserve() 插桩只接受原签名。默认训练保留无 prefill 关键字的原调用，新增关键字只用于明确的零更新补采。修正后的最终完整回归全部通过。

实际 block31 完整检查点 fork 已验证：learner、replay、schedule、RNG、collector_state 内容校验全部一致；原文件 SHA 未变。仅运行槽位由 8 增至 16，并显式记录新课程身份；未进行优化更新。证据 actual-block31-fork-proof.json。

独立复审结论：两项 P1 与一项 P2 均已解决，完整回归通过后可启动。首次冻结运行时 `e3c8a225fd7e9d6e2cee2680c185ba25d0f669abf84d0927960262eda2db5194`；181 个源文件、脚本和测试归档为 `implementation/source-qualified.tar.gz`，文件级清单及测试结果见 `source-qualified-proof.json`。原 block31 完整状态和旧退化模型的 SHA 均再次核对未变。

## 并行原生启动发现的文件竞态

首次启动在 63/256 原生验收时停止。调用链为 `qualify → write → atomic_json → _scan → lstat`：扫描列出其他进程的临时文件后，该文件被原子替换移走，因而触发 `FileNotFoundError`。该故障未进入训练，不是场景不可解或策略失败。目录 `managed-before-artifact-race-fix` 完整保留首次运行，`implementation/artifact-race-recovery.json` 记录备份和当时状态。

修复只让目录扫描忽略已消失的项；其他错误仍上抛，符号链接、硬链接检查和 shared budget 锁不变。确定性文件竞态先复现失败（`artifact-race-red.log`），再通过包含 8 进程进度写入的 42 项回归（`artifact-race-green.log`）。独立复审额外运行 20 项 artifact 测试通过。非共享锁目录的扫描仍是并发近似计量，不声称严格总量事务。

新版本已经重新 prepare，未修改旧 proof，也未复用旧运行时缓存。后续用户取消对照，尚未启动的准备目录保存为 `managed-before-single-c3`；当前 `managed` 仅包含 C3 运行身份。

## 单路线启动与恢复

当前运行时为 `616e9f9fbdf67792900c8013e7af12bad86cab173d7b5625a2b6af28c498673e`。完整源归档 `implementation/source-single-c3.tar.gz`，SHA `ffbe375f4479ccd0f118f69734a91a09ff95ab4006546e3a0ec98ddaa9164d69`；文件级清单、1113 项完整回归和 65 项范围变更回归记录在 `source-single-c3-proof.json`。

初次单路线控制器 PID 208572，在状态显示 35/192、实际已落盘 36 项时请求停止。确认控制器及当时 8 个验收子进程退出，36 项结果均保留，状态 paused。随后 resume 返回 PID 210589；终端窗口“课程二 · 覆盖阶段与回合均衡训练”已打开。相关证据为 `launch-before-stop.json`、`launch-paused.json` 和恢复后记录。这里验证的是管理流程暂停恢复；真实 collector 的训练状态保存/恢复另有定向集成测试，尚未声称完成真实 SAC 训练块。

恢复后独立核验：112/192 场景验收完成，8 个验收进程存活，面板 PID 210610；36 项缓存记录全部保留、所有完成记录均绑定当前运行时，尚未创建训练 checkpoint。证明保存在 `implementation/launch-resumed.json`。此计数是启动验收快照，后续实时状态以终端及 managed/state.json 为准。

## 资源调整与两次主机重启后的恢复

用户先要求从 8 扩至 16。验收槽位使用 DDS 200–215，评测改为 216–231，避免原 220 基址在第 16 槽超过 232。原 114 项完成记录以逐文件 SHA、原/目标运行时和不变的 request/config 身份显式继承，未修改原 proof。相关回归 61 passed，独立复审 15 passed。随后系统发生重启，第一次恢复核验了 120 项结果。

用户随后报告再次黑屏重启，要求降至 12。核实 owner 不存在、新 boot ID 出现、128 项完成记录可校验，且尚无 C3 训练 checkpoint。此前内核日志存在 `NVRM: nvAssertFailedNoLog ... osapi.c:2075`，没有查到对应 OOM-kill 记录；这些证据不足以独立确定黑屏或重启根因。

当前三份 worker settings 均为 12。仅追加训练并发的配置入口，使实际 C3 arm config 采用 12，而不改已经冻结的根配置/场景身份。27 项回归通过，独立测试确认完整 block31 能直接 fork 为 12 环境并保留学习状态。资源迁移再次逐项保留 128 个文件的原始 hash/proof；不把资源兼容例外泛化到奖励、传感器、场景或策略变更。

当前运行时 `c4d80a1b185b0e6fb7a78e7ccdfb6ad8e9b8841ddfadc7f4fd02f7de8b56fe31`，恢复返回 PID 13668。源归档 `source-single-c3-12workers.tar.gz`，文件级清单在同名前缀的 proof.json；迁移证据为 `resize12-before.json`、`resize12-applied.json` 和 frozen/resource-resize12.json。12 路的初步运行检查与长期稳定性应分开；没有因降并发而宣称 GPU 问题已经修复。

恢复后核验：12 个槽位 0–11 的验收进程均存活，进度从 128 增至 129/192；三份 settings 均为 12，原 128 项文件 hash/proof 保持不变。当前进程树合计 PSS 约 3.06 GiB，系统可用内存约 21.31 GiB，swap 基本空闲。`implementation/resize12-live.json` 保存此快照；这是原生验收阶段的初步资源测量，不能当成后续 SAC 或长期稳定性保证。
