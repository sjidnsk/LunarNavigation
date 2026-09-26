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

## 验收边界

有界保存、可恢复训练和观测摘要不等于网络已学会解环。训练后关闭截断的固定场景对照、
大场景泛化、长期学习改善、Humble、Orin、实车：尚未验证。
