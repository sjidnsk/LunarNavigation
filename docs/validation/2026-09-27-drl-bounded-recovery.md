# DRL 小场景有界恢复训练

## 任务与来源

用户于2026-09-26确认先整理开发分支、限制磁盘占用、恢复有限训练，并进一步指定优先训练小场景。
执行跨越本地日期2026-09-27。实现从 `fix/drl-stagnation-budget@452ca016` 的已有停滞改动继续，
目标仅为 `dev/drl-exploration`；历史模型、实验、其他工作树及远程分支不清理、不推送。

学习目标保持原样：C0 Actor、gamma=1、完成奖励5、99%成功阈值、batch64/microbatch8、
Adam、温度和已有回放。停滞截断仅用于训练采集，正式评估默认关闭；不引入禁回头、黑名单、
强制改选目标或循环惩罚。此前Actor变换试验的未完成对照不作为本轮算法有效性证据。

只读源检查点：
`/home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/drl-exploration-redesign/training-output/drl-coverage99-v1/resume.pt`。
本次重新计算SHA256仍为
`71e2485ea4e53d59f2ca7a7f624c42715c0aba0967994cb459a3b7bd2fa95307`。
实际dry-fork校验通过：源admitted98,212、updates24,296。允许变化明确记录为
stagnation_window_steps 0→128、artifact_root、total_output_max_bytes 20→8 GiB、reward_diagnostics false→true；
没有改变奖励、折扣、学习/回放容量或课程配置身份。

## 实现与存储契约

- 统一根 `/home/kai/CodexDownloads/lunar_navigation/drl-small-recovery-20260926`，
  所有本阶段训练、导出和评测产物共享8 GiB，诊断子预算128 MiB。manifest初始化后额度不可静默提高。
  flock串行化预算检查与写入，检查原子保存的旧文件+临时新文件峰值；路径逃逸、软链接、
  删除manifest后退回独立额度均拒绝。已有历史目录不复制、不纳入自动清理。
- `resume.pt`仍覆盖保存一份；SIGKILL留下的模型临时文件计入总额，不误计为诊断文件。
  失败写入保留旧好检查点，预算错误不被Torch ZIP收尾异常替换。
- metrics滚动8 MiB；reward-summary覆盖1 MiB；reward-cases最多16例、每例128步、16 MiB；
  runtime.log滚动64 MiB。所有这些都计入同一个128 MiB诊断总额。
- 统计只处理新admission，沿用ledger去重，在checkpoint保存固定容量状态。固定覆盖桶、
  family/size桶、奖励四项成本与完成项在线累加。恢复切断旧活动片段，完成、截断、其他终止、
  中断和未完成的>=95%收益分别汇总，记录Actor版本范围；它不是冻结策略价值校准。
- 成功和停滞案例各8槽，不因大量循环刷掉所有成功对照；不保存完整地图、图张量或额外回放副本。
  案例为有偏选择，不能用来估计总体发生率。零新增含有用转移，不直接定义为循环。
- wrapper控制同根只运行一个训练任务，记录PID/start ticks/boot ID、子命令、退出状态，
  向所属进程组转发信号；接近额度时停止日志增长并请求有序停止。原生与Python ROS均关闭外部日志。
  预算是应用写入器的约束，不能拦截任意外部程序绕过API直接写文件；不额外开无界tee/screen日志。
- `--max-transitions`派发前计算已入库+在途额度，保证本次新增不超过指定值。

## 小场景课程与恢复

使用 `config/drl_exploration_recovery.yaml`，真实训练命令明确带
`--probe --probe-extent 60 --probe-budget 512 --max-transitions 2048`。
moon/cave各4个环境，固定60 m，episode预算512，停滞窗口128。
这里probe仅覆盖采集条件，仍真实更新SAC，没有重置warmup或累计进度。
原checkpoint已在原课程后期，所以后续resume也必须携带这些覆盖参数；不带会恢复原大小混合课程。

小场景训练用于提高本轮可完成回合和可解释证据的获取机会，不能据此宣称大场景问题已解决。
首轮重点分析覆盖后期净奖励与成功后续收益，只有得到证据才安排单因素奖励/价值学习试验。

## 本次验证

- 派发额度及真实CPU learner/collector训练—保存—恢复接线：7 passed，5.50 s。
- 固定容量奖励诊断：15 passed；8环境满128窗口时summary约8 KB，附加checkpoint状态约509 KB。
- launcher真实子进程与环境单元验证：39 passed（其中launcher9项），覆盖信号、孙进程、互斥、退出码和容量。
- 存储/fork/checkpoint相关：57 passed，13.97 s；包括实际原子峰值、跨目录与并发额度、旧状态兼容。
- 首轮完整源码包：432 passed / 2 skipped，78.25 s；跳过项为单独启用的原生测试。
- 复用隔离Jazzy缓存构建：6 packages，1.43 s。
- 已安装包真实ROS导航/控制/末转移：2 passed，15.52 s；指定的ROS日志目录无新增文件。
- 独立审查两项重要问题已修：采集者/worker将SIGTERM交由learner处理停止屏障；
  wrapper在所有退出路径清理所属进程组，并保留原始退出码。原生子进程独立session中恢复TERM默认。
  三个真实失败用例先红后绿，直接16项通过；collector/training依赖46项通过。
  独立复审确认无剩余Important/Critical，独立launcher/processes重跑15项通过。
- 包级测试首次误用`/usr/bin/python3`缺Torch，9 failed / 332 passed / 57 skipped；
  切换venv后首次自动选择unittest没有运行测试（exit5），未当作通过。最终命令显式
  用venv启动colcon并指定`--python-testing pytest`：435 passed / 2 skipped，88.07 s。
- 最终6包构建1.65 s，安装后原生2项重跑15.66 s；40个Python模块SHA256均与源码一致，
  terrain native扩展来自同一新缓存。原生验证目录仍无ROS日志文件。

## 整合与实际启动

`046ed473517e09fea4ac03116fb5be16cc33bf57`已从修复分支快进合入干净的
`dev/drl-exploration`，整合后tree完全一致；没有推送、删除分支/工作树或清理历史产物。

北京时间2026-09-27 00:15:02从开发工作树启动一个持久包装器，初始wrapper PID69666、
learner PID69667（PID是当时快照，后续须以launch.json的start ticks和实际进程核对）。
真实启动命令、代码版本和来源保存于统一产物根的launch.json、run/run.json。
初始新resume.pt已在采集启动前保存，包含原累计98,212条/24,296次更新及本轮诊断起点。
在线状态与最终退出结果以run/reward-summary.json、runtime.log和exit.json为准；
`exit.json.status=running`本身不能证明进程仍存活。

启动后现场核对：wrapper与learner仍在运行，8环境均60 m、moon/cave各4；已新增12条，
累计98,224、updates24,300（较源增加4）。本轮12条均有正面积新增，其中1条净奖励为负，
总新增152.12 m²；尚无完整成功/截断回合，因此不能估计完成率。奖励分项核对无不一致。
现场产物2,535,663,579 B（约2.36 GiB），诊断115,441 B，分别低于8 GiB/128 MiB。
初始resume仍是98,212条边界；实时摘要中的新增进度尚待周期或最终屏障保存，不误称已可恢复。

首批learner update wall约17.6–18.1秒，显示吞吐还受学习计算影响；本轮继承历史回放，
暂不能把耗时全部归因于当前小场景导航，也不能仅凭该计时定位某一算子。
以上是启动时快照；本轮随后达到2048新增入库后自行停止并保存，最终结果如下。

## 完成后核验（2026-09-27）

北京时间02:01:40正常结束，exit.json记录child/包装器退出码均为0，无信号、异常或强制停止。
训练累计106.63分钟；实际新增2,048条、更新512次，最终保存100,260条、updates24,808，
摘要与检查点边界一致。现场确认原wrapper/learner已退出；检查点结构、模型身份和2,475个
learner/collector张量的有限性校验通过。Collector最后发布版本24,800，导出评测使用最终learner版本24,808。

完整累计摘要记录10次99%成功（洞穴8、月面2），碰撞、其他终止和环境截断均为0。
全局采集额度停止时8个环境回合尚未完成，其中6个已有>=95%覆盖后片段；这些回合不能算失败，
也不能把10次完成解释为100%成功率。首轮结束时metrics仅保留1,256条transition记录，案例只保留最近8次成功；
全局统计以reward-summary的全部2,048条为准。

| 动作开始时覆盖率 | 动作数 | 零增益动作 | 有增益但负奖励 |
| --- | ---: | ---: | ---: |
| <80% | 865 | 144（16.65%） | 45/721（6.24%） |
| 80–95% | 449 | 222（49.44%） | 46/227（20.26%） |
| 95–99% | 734 | 593（80.79%） | 63/141（44.68%） |

95–99%桶去掉完成奖励后合计-3.086；包含完成奖励则+46.914。
10个成功尾段合计302步、回报+50.186；6个未完成尾段432步、回报-3.272。
这支持后期即时收益偏弱的假设，但尚不能区分奖励目标、价值传播与Actor学习问题。
gamma=1，不存在按步折扣直接衰减；变化中的训练Actor轨迹也不是冻结策略价值校准。

仍有循环的直接证据：洞穴seed87019971607003347第117–189步连续73步零新增，
覆盖停在97.527%；73个实际终点聚为3组，主要两个位置分别出现37和35次。
未达到128步训练截断阈值，所以STAGNATION=0不代表没有循环。
反例是洞穴seed87019971607003344连续134步无新增但移动到许多不同位置，随后第233步
新增3 m²并达99.597%；不能仅凭零新增长度缩短截断窗口。

训练前后Actor已导出到共同根的evaluation/，合计8,054,886 B；导出后总占用
2,492,781,450 B，诊断9,220,282 B，仍低于8 GiB/128 MiB。两个Actor结构一致、参数有变化。
用户随后确认执行[训练前后冻结初筛](2026-09-27-drl-frozen-screen.md)，具体效果以该独立记录为准。

## 验收边界

有界保存、可恢复训练和观测摘要不等于网络已学会解环。训练后关闭截断的4图固定场景对照
已完成（见[冻结初筛](2026-09-27-drl-frozen-screen.md)），完成数2/4→3/4，困难洞穴仍有循环。
大场景泛化、长期学习改善、Humble、Orin、实车：NOT_RUN。
