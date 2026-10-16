# 有界一/八步 soft Retrace 对照设计

## 意图与验收

用户已要求按“多步设计和验证 → 同起点小预算一/八步训练 → 冻结验收”的顺序执行。
目标是模型自身在99%探索任务中避免长期无新增往返。前轮Polyak对照有局部改善，但moon2703仍失败，
不是已通过的修复。本设计继续保持奖励、gamma=1、C0网络、学习率、更新比例、部署动作选择不变；
不加禁回头、黑名单、强制换目标、循环接管或循环惩罚。

这是现有采集/回放/离散SAC训练路径的扩展。默认仍为一步，最多八步是待检验候选，不承诺最佳步数。
当前用户授权涵盖设计、实现、工程验证及后续有界实验；设计中的实现选择和所有结果留档供复核。

## 选择与公式

朴素八步奖励和会混用历史行为策略与当前策略；整回合蒙特卡洛又容易被大场景长度、截尾和方差影响。
采用截断soft Retrace，固定lambda=1，唯一实验变量是learning.return_steps（1或8）。
理论依据：[Retrace原论文](https://arxiv.org/abs/1606.02647)、
[SAC原论文](https://arxiv.org/abs/1812.05905)。论文不为本任务的八步效果提供保证。

记当前Actor为pi，真实采集分布为mu，qbar=min(target1,target2)，alpha为当前温度，
Vbar(s)=sum_a pi(a|o)[qbar(s,a)-alpha log pi(a|o)]。
长度m<=return_steps的连续片段用以下倒推：

```
d_i = gamma * (1 - terminated_i)
c_i = exp(min(0, log pi(a_i|o_i) - log mu_i))      # i >= 1
G_(m-1) = r_(m-1) + d_(m-1) * Vbar(s_m)
G_i = r_i + d_i * [Vbar(s_(i+1)) + c_(i+1) * (G_(i+1)-qbar(s_(i+1),a_(i+1)))]
```

全部目标无梯度。目标网络、当前Actor和alpha在一次learner.update内保持同一版本；不额外加入当前动作熵，
不使用行为策略的熵，不把目标双Q均值替代min。n=1必须走原td_targets路径并保持原数值/更新行为。
同策略时有限Retrace含动作优势控制变量，只要求期望与soft多步目标对齐，不要求每条随机轨迹逐值相同。
gamma=1不满足原论文折扣收缩证明的条件，不能引用该定理宣称本任务神经网络必然收敛。
Critic仍仅更新均匀抽出的64个anchor动作；Actor和alpha仍只用这些anchor，更新频率不变。
真实terminated停止bootstrap；预算、训练停滞truncated结束序列但保留最后next state的bootstrap。
缺后续经验、回放尾部或淘汰形成的间隙使用现有片段，不能补假奖励或跨重置连接。

## 数据与兼容

Transition追加两个可选字段：step_index:int|None（回合内0起步）、behavior_log_prob:float|None。
两个字段同时缺省表示旧经验；出现新字段时必须配对完整，step_index是非负整数，log_mu有限且<=0。
episode_id已有真实回合UUID。collector在同一次torch.multinomial抽样中取实际联合分布的选中动作概率，
log_mu=log(w[action])-log(sum(w))，w为同次multinomial的实际权重，不加epsilon或重新前向。
经同一个STEP/token交给worker；worker把该概率与执行前步号附加到完成Transition。
不使用actor_version重建mu，不改变部署ActorPolicy，不增加随机数抽取。ACK重发保留完全相同的完成记录。

ReplayBuffer新增sample_sequences(batch_size,rng,max_steps)，返回list[tuple[Transition,...]]。
anchor抽样严格复用原sample的随机索引与替换采样分布，不只选“有后续”的anchor。
只沿同episode_id、step_index+1、同scene及相邻next/current观测和privileged状态一致的记录扩展。
旧anchor/缺元数据保守返回单条。actor_version变化不截断，交由真实mu修正。
不复制观察数组，不缓存额外图；序列索引两臂均维护，计入内存额度并随淘汰释放，避免改变某一臂的淘汰条件。
快照保留原有共享与字节上限。目标推理每次最多microbatch_size个状态，不能把八步展开为8倍大图batch。
旧transport缺字段由默认值读取；legacy snapshot仍可完整恢复。

## 配置、恢复与审计

LearningConfig.return_steps默认1，接受整数1..8。旧配置/learner/semantics缺字段规范化为1；
普通resume不能悄悄更改return_steps。新显式--fork-experiment paired-multistep仅允许return_steps、
相同paired_scene_sequence和收紧snapshot预算，其他学习参数、seed和模型身份变化必须拒绝。
先按原语义验证源，再在新fork中明确记录目标语义迁移；原检查点不写入。
所有网络、目标网络、四个Adam、alpha、RNG、回放、schedule、counters必须保留。
paired-multistep初态身份仅排除return_steps，仍比较polyak；不能沿用排除polyak来掩盖额外变量。

仅增加有界标量诊断：实际trace长度、长度>1占比、有效系数质量和旧经验一步占比。
要报告八步实际上使用了多少，不能把大量旧数据fallback的试验称为全量八步。
推理schema/网络输入不变，终态仍可导出原大小Actor。训练numerical failure不覆盖最后安全检查点。

## 实验资源与阶段门槛

共同源为drl-small-recovery-20260926/run/resume.pt，102308转移/25320更新、credit1，
SHA0149887da8ebfedfee8cc536338888c99465bf7b6b9c1e8ba9bc5ec9afd27eba。
保持同根总8GiB、诊断128MiB；不删除已有两臂Polyak证据，不复制完整源。
两臂各新建一个输出目录、顺序运行、各最多2048新增；60m、512决策、8环境，训练停滞128。
tau=.005、lr=1e-5、batch64/micro8、update_ratio=.25，来源完整回放4954条均恢复。
两臂落盘snapshot_max_bytes=512MiB，只影响新的保存快照；中断后不自动续训或重跑。
峰值需包含现有约4.41GiB、A+B+原子临时B、诊断余额和Actor，并在启动前按实际字节复核。
每臂训练上限7200秒；任一失败或不同初态/实际更新量/credit即停止，不能伪称等预算对照。

工程验证通过后才启动训练。两臂冻结joint argmax、关闭训练截断，先各跑
cave/moon×2026092701/2702及moon2703，共10回合。看99%完成、实际无新增往返、共同成功成本及碰撞。
关键图仍失败、原成功退化或无可信改善则关闭本轮；不自动长训/扫4、16或其他参数。
若通过，再用新种子2705/2706×cave/moon共8回合；关键单次改善最多2图每臂复核一次，总上限22回合。
固定种子不能消除ROS异步轨迹波动。未通过不宣称多步无效或已找出唯一根因。
Humble、Orin、实车、大图均NOT_RUN。

## 验证要求

手算三步/八步、n=1回归、pi=mu、截断importance ratio、零系数切断、熵只计算一次；
terminal空候选、truncated保留bootstrap；更新后目标/Adam/alpha有限且microbatch等价。
异步交错回合、同scene不同episode、step间隙、淘汰、重复step、快照恢复、旧数据回退、内存额度。
真实collector概率与被抽中动作对应且不改变RNG轨迹；STEP重发/ACK/恢复不串记录。
普通resume拒绝跨度变化，显式fork只允许计划差异；初态摘要检测polyak/优化器/RNG变化。
完整受影响包测试、隔离Jazzy构建、安装与源码一致和一次有限运行验证后方可给效果结论。
