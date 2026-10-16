> 历史分支记录：`test/drl-loop-learning` @ `d5de79c0b157`。本文运行状态、接口和实验结论是当时快照；当前开发以 [统一入口](../../DRL开发入口.md) 为准。

# 学习式解环首轮对照 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现可追溯的新有界 Actor 候选及解环诊断，完成 C0/新候选各 3000 条转移的工程筛查，判断是否值得扩大实验，而不是承诺本轮解决全部循环。

**Architecture:** 保持离散 SAC、奖励、回放采样、地图/规划/控制不变，仅将 Actor 分数形式作为学习对照变量。数值守卫、冻结 Q 探针、真实覆盖恢复指标、回放审计和冻结策略价值校准分别成模块；现有配对驱动新增独立 profile 与筛查停止门。

**Tech Stack:** Python 3.12、现有 PyTorch/NumPy、pytest、隔离 ROS 2 Jazzy/colcon、既有 CPU collector + 单 GPU learner。

**Spec:** [已确认书面设计](../specs/2026-09-23-drl-loop-learning-design.md)。用户于 2026-09-23 确认设计；本实施计划尚待审阅与执行方式选择。下面未勾选的步骤均未执行。

## Global Constraints

- 不加入运行时禁回头、目标黑名单、循环后强制换目标、随机解困或额外循环惩罚。
- Actor 仍只读取测量信息，真值仅供训练侧 Critic 和评估使用；不改地图、候选合法性、规划、净空路径简化、控制、车辆 launch 或 TCP。
- 保留 C=0 及旧 `centered_tanh_v1`；新候选为 `centered_softsign_v1`，`b_i=C*x_i/(C+abs(x_i))`，`x_i=z_i-mean_valid(z)`，C=10，均值可微。
- gamma=1、completion_bonus=5.0、success_coverage=0.99、联合熵、alpha 上限 1e-4、初始 alpha=5e-5、学习率 1e-5、有效 batch64、microbatch=8、更新比 0.25、预热1024。
- 8 环境、30 倍目标倍率、10 m/90°、0.2 m/s；40～80 m 小地图课程，512 决策回合预算；单 GPU 串行各组。
- 训练种子 2026092301、2026092302、2026092303；每个种子的两组相同参数初值，均新建优化器。第一种子每组 3000 条转移后必须停止。后续各组累计10000条、共60000条，仅在另获用户明确同意后运行。
- 原检查点只读，不覆盖、不兼容迁移、不改 C 后直接 `--resume`；原开发树中 microbatch=8 的两处未提交修改原样保留。
- 所有诊断默认关闭；不改变原命令的实验身份和输出含义。新 profile 的身份校验拒绝旧目录、不同模型形式、不同种子/预算/场景协议混用。
- 构建、模型、日志和实验输出均在独立仓库外目录；不写原 Jazzy cache。不合并、不推送、不创建 PR、不设置 upstream、不删除分支/worktree。
- `GOAL_REACHED` 不是探索完成；新候选效果、120/250 m、Humble、Orin、目标 DDS、实车在未运行前均为 `NOT_RUN`。

## Review Focus

1. 老 C0/旧 C10 文件缺新字段：必须还原旧含义，新 softsign 不得冒充兼容恢复。由 Task 1 的旧文件及混搭检查点测试覆盖。
2. 原始最高分已并列、单动作、空 successor、float32 映射后并列：区分合法对称与新增数值问题，不靠动作破平局绕过。由 Task 1～2 覆盖。
3. 恢复探索刚发生就预算截断、第三点继续零新增、观测到任务外网格：不得算持续解环成功，不能从条件分母中悄悄删除。由 Task 4 与 Task 6 覆盖。
4. collector 重发 DONE、快照只剩尾段、旧快照缺诊断计数：不得重复计数或伪造完整回合/历史。由 Task 5 与 Task 8 覆盖。
5. 筛查断点恢复、用户中断、子进程退出码0但报告错误、磁盘不足：不得自动进入长训或覆盖最后好检查点。由 Task 2、Task 7～8 覆盖。

## 工作区、文件分工及依赖

本计划只在 `docs/drl-loop-learning-design` 写文档。执行时从承载本计划的该分支创建 `test/drl-loop-learning` 及独立工作树；该分支以 `dev/drl-exploration@fe9cab83` 为代码基线。执行前重查祖先和工作区，若开发线已新增代码，先报告差异，不静默合并到实验。

| 文件（均位于 `ros2_ws/src/lunar_drl_exploration/`） | 单一职责 |
| --- | --- |
| `lunar_drl_exploration/config.py`, `model.py` | 分数参数化与身份，保持模型参数张量结构 |
| `lunar_drl_exploration/policy_numerics.py`（新） | 纯张量数值检查、有限错误摘要，不选择动作 |
| `lunar_drl_exploration/loop_probe.py`（新） | 当前样本冻结 Q、原/新 Adam 离线对照 |
| `lunar_drl_exploration/loop_metrics.py`（新） | 真实覆盖增量、停滞/恢复事件，不读取 Q 或改动作 |
| `lunar_drl_exploration/loop_backtrack_case.py`（新） | 固定回退回归场景构造，不含选路策略 |
| `lunar_drl_exploration/replay_audit.py`（新） | 只读经验组成与有界新采集窗口 |
| `lunar_drl_exploration/value_calibration.py`（新） | 冻结随机策略 rollout 与 soft-return 校准 |
| `lunar_drl_exploration/loop_protocol.py`（新） | profile、地图清单、阶段停止门和身份 |
| `lunar_drl_exploration/loop_report.py`（新） | 逐组/逐场景汇总，不运行训练 |
| `sac.py`, `collector.py`, `training.py`, `replay.py`, `checkpoint.py`, `evaluation.py`, `runtime.py`, `paired_experiment.py`（同上包目录） | 只做上述模块的接入与兼容；不整文件重构 |
| `test/test_loop_*.py`, `test/test_policy_numerics.py`, `test/test_value_calibration.py`（新）及相关既有测试 | 各模块测试和端到端故障契约 |

依赖：Task 1 → Task 2 → Task 3；Task 4 → Task 5/6；Task 3/5/6 → Task 7 → Task 8。均可按顺序执行，不依赖多代理才能完成。

## 执行环境与通用验证命令

执行者按 `using-git-worktrees` 及仓库 AGENTS 要求建立独立代码树，不在脏开发树写入。不另装 Torch，不复制原模型。首次实现前记录已知基线：上轮当前代码两组测试 24 passed、1 failed，失败为旧 C10 的最高分并列；执行时重跑并保留输出，不把本计划中的历史结果当作新验证。

后续命令均在代码树根目录执行。测试前先用独立 cache 构建原基线的 native/ROS 依赖；每次涉及安装代码的验证前重建受影响包。

```bash
export DRL_CACHE=/home/kai/.cache/lunar-drl-loop-learning/jazzy
export DRL_PYTHON=/home/kai/.cache/lunar-drl-training/venv/bin/python
scripts/drl/build.sh

drl_python() {
  env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u ROS_PACKAGE_PATH -u PYTHONPATH -u LD_LIBRARY_PATH \
    -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
    bash --noprofile --norc -c '
      set -e
      source /opt/ros/jazzy/setup.bash
      source /home/kai/.cache/lunar-drl-loop-learning/jazzy/install/local_setup.bash
      test "$ROS_DISTRO" = jazzy
      export PYTHONPATH="$PWD/ros2_ws/src/lunar_drl_exploration:$PYTHONPATH"
      export PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
      export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
      export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
      exec /home/kai/.cache/lunar-drl-training/venv/bin/python "$@"
    ' drl-loop-python "$@"
}
```

这只在显式实验/测试命令中优先导入当前源码；正式新 profile 用重建后的独立安装，并记录各模块 `__file__`，防止旧安装混入。无 ROS 的纯函数测试仍使用同一 Python，避免依赖差异。

---

### Task 1: 新参数化、旧模型身份与数值回归

**Files:**

- Modify: `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/config.py`（ModelConfig、canonical_model_config）
- Modify: `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/model.py`（bound_actor_scores、Actor.forward_packed）
- Modify: `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/checkpoint.py`、`runtime.py`、`evaluation.py`（同目录，配置一致性及加载消费点）
- Test: `ros2_ws/src/lunar_drl_exploration/test/test_bounded_actor.py`
- Test: `ros2_ws/src/lunar_drl_exploration/test/test_loop_model_identity.py`（新）

**Interfaces:**

- `ModelConfig.actor_score_transform: str = 'centered_tanh_v1'`；只允许该值及 `centered_softsign_v1`。
- `bound_actor_scores(raw_logits, action_groups, count, bound, transform='centered_tanh_v1') -> Tensor`；四参旧调用不变，C0返回原对象。
- `canonical_model_config(record) -> dict` 为缺失 transform 的旧配置补 `centered_tanh_v1`，不得补其它缺失网络字段。

- [ ] **Step 1 — 写确定性红测试，不依赖测试顺序偶然放大出的分数。**

```python
def test_softsign_recovers_distinct_top_and_nonzero_gradient():
    import torch
    from lunar_drl_exploration.model import bound_actor_scores
    z = torch.tensor([100., 101., -201.], requires_grad=True)
    groups = torch.zeros(3, dtype=torch.long)
    y = bound_actor_scores(z, groups, 1, 10., 'centered_softsign_v1')
    gradient, = torch.autograd.grad(-y.log_softmax(0)[1], z)
    assert y.argmax().item() == 1
    assert y.abs().max() <= 10
    assert torch.isfinite(gradient).all() and gradient.abs().max() > 0

def test_legacy_tanh_saturation_remains_explicit():
    import torch
    from lunar_drl_exploration.model import bound_actor_scores
    z = torch.tensor([100., 101., -201.], requires_grad=True)
    y = bound_actor_scores(z, torch.zeros(3, dtype=torch.long), 1, 10.)
    torch.testing.assert_close(y, torch.tensor([10., 10., -10.]))
    gradient, = torch.autograd.grad(-y.log_softmax(0)[1], z)
    assert torch.count_nonzero(gradient) == 0
```

补充：旧 C0/旧 C10 去掉新字段后，加载和导出概率保持原值；相同张量但新旧 transform 不同的完整恢复被拒绝；篡改 `config.model`、`semantics.model`、`learner.model_config` 任一处均被拒绝，不只比较其中两处。

- [ ] **Step 2 — 运行红测试并保存失败。**

```bash
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_bounded_actor.py ros2_ws/src/lunar_drl_exploration/test/test_loop_model_identity.py -q -p no:cacheprovider
```

预期新增 softsign/identity 测试因缺接口失败；历史 tanh 反例应通过。若出现依赖导入错误，先修验证环境，不能当作红测试证据。

- [ ] **Step 3 — 实现最小参数化，保留旧 tanh 的算术路径。**

```python
if bound == 0:
    return raw_logits
counts = segment_sum(torch.ones_like(raw_logits), action_groups, count)
means = segment_sum(raw_logits, action_groups, count) / counts.clamp_min(1)
centered = raw_logits - means[action_groups]
if transform == 'centered_tanh_v1':
    return bound * torch.tanh(centered / bound)
return centered * (bound / (bound + centered.abs()))
```

softsign 的非有限中间值交由 Task 2 显式报错，不以 nan_to_num 静默改策略。增加配置枚举校验和所有配置消费者；未知 transform 抛 ValueError。原随机放大测试拆成固定种子正常域契约、上述旧饱和再现、新 softsign 压力用例。保留旧形式风险说明，不删除失败历史或改用 raw argmax。

- [ ] **Step 4 — 跑 C0 等价、跨状态独立、共同偏移、排列等变、空/单动作、double gradcheck、极大有限值及导出/恢复回归。**

```bash
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_bounded_actor.py ros2_ws/src/lunar_drl_exploration/test/test_loop_model_identity.py ros2_ws/src/lunar_drl_exploration/test/test_runtime.py ros2_ws/src/lunar_drl_exploration/test/test_evaluation.py ros2_ws/src/lunar_drl_exploration/test/test_replay_checkpoint.py -q -p no:cacheprovider
```

预期新增契约与既有兼容测试通过，旧 tanh 饱和作为明确行为测试通过；并非声称旧 tanh 饱和被修复。

- [ ] **Step 5 — UTF-8 阅读、`git diff --check`，显式暂存上述实际修改文件并提交。**

提交信息：`feat(drl): add versioned centered softsign actor scores`。不得暂存原开发树的两处修改。

### Task 2: 数值守卫覆盖采集、TD 与学习更新

**Files:**

- Create: `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/policy_numerics.py`
- Modify: 同包 `config.py`、`sac.py`、`collector.py`、`training.py`
- Create: `ros2_ws/src/lunar_drl_exploration/test/test_policy_numerics.py`
- Modify: `ros2_ws/src/lunar_drl_exploration/test/test_bounded_actor.py`、`test_collector.py`、`test_replay_checkpoint.py`

**Interfaces:**

- `TrainingConfig.loop_diagnostics: bool = False`，仅诊断开关，不进入经验目标的 `resume_semantics`；新 profile 要求两组同时为 True。
- `policy_health(policy, action_groups, count) -> Tensor[count,3]`，列为非有限值、新增最高 logits 并列、新增最高概率并列；均为按状态的0/1。
- `PolicyNumericsError(RuntimeError)` 携带 JSON 安全的 `summary: dict`；非有限原值以字符串表示，不让报告序列化二次失败。
- `SACLearner(..., check_policy_numerics=False)`，普通训练默认路径保持原行为；新 profile 显式开启。

- [ ] **Step 1 — 写红测试固定合法原始并列和新增数值并列。**

```python
def test_only_new_top_ties_are_errors():
    import torch
    from lunar_drl_exploration.model import PolicyOutput
    from lunar_drl_exploration.policy_numerics import policy_health
    raw = torch.tensor([100., 101., -201., 7., 7.])
    logits = torch.tensor([10., 10., -10., 0., 0.])
    groups = torch.tensor([0, 0, 0, 1, 1])
    from lunar_drl_exploration.model import segment_log_softmax
    logp = segment_log_softmax(logits, groups, 2)
    flags = policy_health(PolicyOutput(logits, logp.exp(), logp, raw), groups, 2)
    assert flags[0, 1:].tolist() == [1, 1]
    assert flags[1].sum() == 0
```

再构造全非有限、一动作、零动作、仅概率舍入并列、损失恰为零的用例；后者只能记录零梯度而不应报错。针对 TD successor 和 collector 采样前分别注入故障。

- [ ] **Step 2 — 跑新增测试，确认因模块/开关缺失失败。**

```bash
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_policy_numerics.py -q -p no:cacheprovider
```

- [ ] **Step 3 — 实现纯张量分组检查，不在每个动作上读取 GPU 标量。**

```python
def new_top_ties(raw, mapped, groups, count):
    import torch
    from .model import segment_sum
    rmax = raw.new_full((count,), -torch.inf).scatter_reduce_(
        0, groups, raw, reduce='amax', include_self=True)
    mmax = mapped.new_full((count,), -torch.inf).scatter_reduce_(
        0, groups, mapped, reduce='amax', include_self=True)
    rn = segment_sum((raw == rmax[groups]).to(raw.dtype), groups, count)
    mn = segment_sum((mapped == mmax[groups]).to(raw.dtype), groups, count)
    return ((rn == 1) & (mn > 1)).to(raw.dtype)
```

`policy_health` 分别对 logits/probs 调用该函数，非有限列覆盖 raw/logits/probs/log_probs。SAC 汇总 TD 与 Actor 两阶段的标志，拼入现有末尾一次 CPU 指标传输；若有问题，在 `learner.update()` 返回前抛错，仍处于 `UpdateBoundary.update()` 内，不发布该模型、不允许保存。失败路径可复制首个坏状态的动作分数/间隔作最多1MiB诊断；正常路径不增加逐动作同步或全图拷贝。collector 为 CPU 检查，在 multinomial 前失败走现有 FATAL/清理流程，不另选动作。

- [ ] **Step 4 — 验证部分更新后异常不写 checkpoint，普通梯度和 microbatch 等价。**

用既有 capture_state/UpdateBoundary fixture 写入初始 checkpoint，注入数值异常，断言磁盘源文件哈希不变、boundary.failed=True、无 PUBLISH；不以 caught exception 后继续训练替代回滚。相同初值运行有效 batch64，比较 microbatch8/16/64 所有参数和指标；延续既有 CUDA `NoScalarDeviceSync` 测试。

```bash
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_policy_numerics.py ros2_ws/src/lunar_drl_exploration/test/test_bounded_actor.py ros2_ws/src/lunar_drl_exploration/test/test_sac.py ros2_ws/src/lunar_drl_exploration/test/test_collector.py ros2_ws/src/lunar_drl_exploration/test/test_replay_checkpoint.py -q -p no:cacheprovider
```

- [ ] **Step 5 — 审阅无动作规则、无额外 CUDA 正常路径同步后，显式提交本任务文件。**

提交信息：`feat(drl): stop invalid loop experiments at safe model boundaries`。

### Task 3: 当前样本的可复现冻结 Q 对照与第一道门

**Files:**

- Create: `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/loop_probe.py`
- Create: `ros2_ws/src/lunar_drl_exploration/test/test_loop_probe.py`
- Read-only inputs: `/home/kai/.cache/lunar-drl-training/loop-cause-20260923-DHIXyD/{sample.bin,networks.pt,audit.json,fit.py}` 和原 `drl-coverage99-v1/resume.pt`

**Interfaces:**

- `fit_case(actor_state, model_config, observation, q, alpha, *, optimizer_state, updates, device) -> dict`：独立复制权重/优化器，返回 `optimizer_mode`（fresh/restored）及 `history`（每项含update、best_probability、soft_objective、greedy_position、q_regret、raw_gradient_max）；记录t=0、最后一步及1/10/50/100/200/300中不超过预算的步；不原位改变传入状态。
- `run_probe(inputs: Path, source_checkpoint: Path, output: Path, *, device='cuda') -> dict`：48状态数值审计，以及索引0/6/22/26 × 三形式 × 两种Adam共24次拟合。
- CLI：`python -m lunar_drl_exploration.loop_probe --inputs PATH --source-checkpoint PATH --output PATH --device cuda`。

- [ ] **Step 1 — 写无 ROS 的小网络/合成 Q 红测试。**

```python
def test_fit_reports_transform_and_learning_separately():
    from lunar_drl_exploration.actor_probe import summarize_fit
    history = [{'soft_objective': 2., 'best_probability': .2},
               {'soft_objective': 3., 'best_probability': .7}]
    row = summarize_fit(1., history)
    assert row['immediate_transform_gain'] == 1.
    assert row['learning_gain'] == 1.
```

该既有函数测试用于保留口径；真正红测试调用新增 `fit_case`：

```python
def test_fit_copies_source_weights_and_accepts_restored_adam():
    import copy
    import torch
    from test_model import observation
    from lunar_drl_exploration.config import ModelConfig
    from lunar_drl_exploration.model import Actor
    from lunar_drl_exploration.loop_probe import fit_case
    cfg = ModelConfig(width=16, heads=2, layers=1)
    actor = Actor(cfg)
    opt = torch.optim.Adam(actor.parameters(), lr=1e-5)
    sum(p.square().sum() for p in actor.parameters()).backward()
    opt.step()
    weights = copy.deepcopy(actor.state_dict())
    moments = copy.deepcopy(opt.state_dict())
    before = copy.deepcopy(moments)
    obs = observation()
    q = torch.linspace(0., 1., len(obs.action_nodes))
    row = fit_case(weights, cfg, obs, q, 1e-4,
                   optimizer_state=moments, updates=2, device='cpu')
    assert row['optimizer_mode'] == 'restored'
    assert [r['update'] for r in row['history']] == [0, 1, 2]
    for key, value in actor.state_dict().items():
        torch.testing.assert_close(weights[key], value, rtol=0, atol=0)
    for key in before['state']:
        for name in ('step', 'exp_avg', 'exp_avg_sq'):
            torch.testing.assert_close(moments['state'][key][name], before['state'][key][name])
```

再测 optimizer_state=None 返回fresh、传入Q不变；teacher偏好回头时报告只称Q跟随，不声称解环。单元测试不运行24次完整网络拟合。

- [ ] **Step 2 — 运行新增接口测试确认失败。**

```bash
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_loop_probe.py -q -p no:cacheprovider
```

- [ ] **Step 3 — 将历史探针中受控拟合核心整理进新模块，去掉硬编码旧源码路径。**

```python
actor = Actor(model_config).to(device).train()
actor.load_state_dict(copy.deepcopy(actor_state), strict=True)
optimizer = torch.optim.Adam(actor.parameters(), lr=1e-5)
if optimizer_state is not None:
    optimizer.load_state_dict(copy.deepcopy(optimizer_state))
for group in optimizer.param_groups:
    group['lr'] = 1e-5
policy = actor.forward_packed(pack_observations([observation], device))
loss = (policy.probs * (alpha * policy.log_probs - q.detach())).sum()
```

在完整循环中重算策略、更新优化器并分开记录变换/学习收益。读取缓存 `audit.json` 的 Q 前，用 `networks.pt` 中同版本双 Critic 重算并校验；源 checkpoint 与缓存的 update=24296、Actor 参数及场景/动作次序须一致，不能只比较 update 数字。流式哈希输入，记录依赖版本、输入大小/mtime/sha256、代码提交和参数。output 不得等于任何输入或位于旧诊断目录。完整 checkpoint 仅作为明确可信的本地文件读取；释放其余回放后再运行拟合，不为取 Adam 长期保留另一份2GiB回放。

- [ ] **Step 4 — 单元测试通过后运行本任务限定的离线实验。**

```bash
drl_python -m lunar_drl_exploration.loop_probe \
  --inputs /home/kai/.cache/lunar-drl-training/loop-cause-20260923-DHIXyD \
  --source-checkpoint /home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/drl-exploration-redesign/training-output/drl-coverage99-v1/resume.pt \
  --output /home/kai/.cache/lunar-drl-loop-learning/experiments/20260923/probe.json \
  --device cuda
```

门槛：新候选固定反例及48状态无新增最高分并列/非有限值，固定反例损失梯度非零；有明确区分的 t=0/学习结果，原Adam与新Adam都保留。若数值不合格，停止在线实验。若数值合格但原Adam下所有跨位置困难样本均未改善 Q 跟随，先报告负结果，请用户决定是否仍花预算开在线对照，不自行换函数或参数。此处的改善仍不是探索成功。

- [ ] **Step 5 — 校验输入哈希/mtime不变，提交代码和测试，不提交输入/拟合模型/JSON。**

提交信息：`test(drl): add checkpoint-matched frozen Q loop probes`。

### Task 4: 真实覆盖恢复与必要回退回归

**Files:**

- Create: `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/loop_metrics.py`
- Create: `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/loop_backtrack_case.py`
- Modify: 同包 `evaluation.py`
- Create: `ros2_ws/src/lunar_drl_exploration/test/test_loop_metrics.py`
- Create: `ros2_ws/src/lunar_drl_exploration/test/test_loop_backtrack_scene.py`

**Interfaces:**

- `reference_delta_cells(before: PrivilegedState, after: PrivilegedState, scene: PrivilegedScene) -> int`，不使用候选估计增益替代实测。
- `LoopRecoveryMetrics(xy, *, tolerance_m=.05, horizon=32)`；`observe(xy, new_reference_cells:int, cell_area_m2:float)`；`finish(*, terminated, truncated, error=None) -> dict`。
- `evaluate(..., loop_diagnostics=False)` 默认仍为旧报告；新 profile 开启后添加 `loop_recovery` 字段。实际位置来自 `env.last_execution.actual_pose`，初始化来自 `env.plant.pose`，缺失则标记 unavailable，不能默默退化为目标点或图节点中心。
- `make_backtrack_case(config) -> tuple[TerrainGrid, Pose, TaskSpec]` 在新fixture模块构造场景；`evaluate_backtrack(config, actor_path, *, budget=128, domain_base=228) -> dict` 在evaluation.py中复用单回合评价循环，以 `reset_scene` 开始，case_id=`backtrack_u_v1`，不加入随机月表/洞穴的完成率分母。

- [ ] **Step 1 — 写红测试：离开两个点但无新增不能算解环。**

```python
def test_third_point_without_gain_is_not_recovery():
    from lunar_drl_exploration.loop_metrics import LoopRecoveryMetrics
    metric = LoopRecoveryMetrics((0., 0.))
    for xy in [(1.,0.), (0.,0.), (1.,0.), (0.,0.), (2.,0.)]:
        metric.observe(xy, 0, .04)
    row = metric.finish(terminated=False, truncated=True)
    assert row['events'][0]['recovered'] is False
    assert row['events'][0]['end_reason'] == 'truncated'
    assert row['recovered_events'] == 0
```

另测：第6步第一次真值新增，recovery_steps=2（自第4步检测）；32步窗口包含恢复这一步及后31步；只观察到1步就截断必须报告window_observed_steps=1及window_complete=False。三点循环连续32步无新增产生停滞事件，但不能标为两点循环。自然有益回退不算错误。

- [ ] **Step 2 — 运行新增测试确认接口缺失失败。**

```bash
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_loop_metrics.py -q -p no:cacheprovider
```

- [ ] **Step 3 — 实现独立纯统计状态机，复用现有几何判断口径。**

```python
def reference_delta_cells(before, after, scene):
    import numpy as np
    if before.scene_id != after.scene_id or after.scene_id != scene.scene_id:
        raise ValueError('reference delta requires one scene')
    if before.observed.shape != after.observed.shape or after.observed.shape != scene.packed_reference.shape:
        raise ValueError('reference bits shape mismatch')
    bits = after.observed & ~before.observed & scene.packed_reference
    return int(np.unpackbits(bits).sum())
```

每次 observe 先推进正在统计的恢复后32步窗口，再检查当前新增。无新增时维护5点坐标及连续零新增计数；ABABA或达到32步零新增均可开启一次停滞事件，已有事件不重复开启。新增网格>0时结束停滞并开始恢复窗口；此后再停滞可开启下一事件，前一窗口仍统计，最多32步后释放。输出检测时刻、零新增起点、恢复时刻、窗口新增面积、实际观测长度、再次停滞和终止原因；所有事件含失败/删失。单回合最多512决策，因此事件/标量序列有界，不存整图。

- [ ] **Step 4 — 增加受控回退场景和计数回归。**

在新模块构造两条较长支路共用底部连接廊的固定场景：

```python
def make_backtrack_case(config):
    import numpy as np
    from .contracts import Pose, TaskSpec
    from .scene import TerrainGrid
    heights = np.full((200, 200), 2., dtype=np.float32)
    heights[40:60, 40:160] = 0.
    heights[40:170, 40:60] = 0.
    heights[40:170, 140:160] = 0.
    terrain = TerrainGrid.from_heights(heights, .2, (-20., -20.), config.platform)
    task = TaskSpec('backtrack_u_v1', 'map',
                    np.array([[-13.,-13.], [13.,-13.], [13.,15.], [-13.,15.]]))
    return terrain, Pose(0., -10., 0.), task
```

图连通测试验证地面支路只有底部连接廊相通，封闭任一支路连接截面即使该支路末端不可达；传感器范围测试验证仅在连接廊不能覆盖两个末端。原生导航测试证明走入一支后返回连接廊可执行。若实际可观测参考使场景无需回退即可达99%，将fixture判为无效并修其几何后重新冻结版本，不能用它声称必要回退通过。

脚本指定路径只用于验证fixture，不算策略成功。正式screen另让各冻结Actor自由选动作，在本fixture中以128决策记录是否自行回退并继续新增，成功、失败或预算不足都保留；不为使其通过而改奖励或屏蔽动作。

```bash
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_loop_metrics.py ros2_ws/src/lunar_drl_exploration/test/test_loop_backtrack_scene.py ros2_ws/src/lunar_drl_exploration/test/test_evaluation.py ros2_ws/src/lunar_drl_exploration/test/test_paired_experiment.py -q -p no:cacheprovider
```

- [ ] **Step 5 — 确认统计器无动作/奖励写入，显式提交。**

提交信息：`feat(drl): measure real exploration recovery without action rules`。

### Task 5: 同时点新采集/内存回放/快照审计

**Files:**

- Create: `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/replay_audit.py`
- Modify: 同包 `replay.py`、`training.py`
- Create: `ros2_ws/src/lunar_drl_exploration/test/test_loop_replay_audit.py`
- Modify: `ros2_ws/src/lunar_drl_exploration/test/test_training_cli.py`

**Interfaces:**

- `ReplayBuffer.iter_transitions() -> Iterator[Transition]`，按保留顺序返回只读引用，不采样、不消耗 RNG。
- `snapshot_contents(payload: bytes) -> tuple[list[Transition], Mapping[str,PrivilegedScene]]` 在 replay.py 中使用现有 envelope/loads_transport 解码及校验，不暴露私有 header 常量给审计模块。
- `summarize_replay(transitions, scenes, *, learner_updates, success_coverage) -> dict`，同时列 reward_new_area 与 reference_delta_cells，禁止将两者混为同一字段。
- `AdmissionAudit(max_rows=1024)`；`observe(transition, scene, *, admission_index)` 只保留标量行；`state_dict()/from_state_dict()` 原语状态进入现有 counters；旧 counters 缺此项时明确从诊断启动时开始计数。

检查点诊断分区固定为 `counters['loop_diagnostics']={schema:'loop_audit_v1', recent:AdmissionAudit.state_dict(), at_checkpoint:{admitted, learner_updates, recent, live, snapshot}}`。recent状态包含rows和total_observed；三个组成摘要均包含transitions、positive_reward_area、positive_reference_gain、episodes、scenes、descriptor_unknown及actor_age_updates。语义保留原schema，只为新profile添加可选诊断键，不能把缺键的旧checkpoint解释为全历史零计数。

- [ ] **Step 1 — 用既有小 replay fixture 写只读/重复消息红测试。**

```python
def test_audit_preserves_replay_and_sampling_rng():
    import numpy as np
    from test_replay_checkpoint import transition, scene
    from lunar_drl_exploration.replay import ReplayBuffer
    from lunar_drl_exploration.replay_audit import summarize_replay
    replay = ReplayBuffer(1000000)
    replay.add(transition(), {'s': scene()})
    before = replay.snapshot(100000)
    rng = np.random.default_rng(3)
    import copy
    rng_before = copy.deepcopy(rng.bit_generator.state)
    row = summarize_replay(replay.iter_transitions(), replay.scenes,
                           learner_updates=10, success_coverage=.99)
    assert row['transitions'] == 1
    assert row['positive_reward_area'] == 1
    assert row['positive_reference_gain'] == 0
    assert replay.snapshot(100000) == before
    assert rng.bit_generator.state == rng_before
    assert replay.sample(1, rng)[0].episode_id == 'episode'
```

在 ControlledEnv 重复 DONE/ACK 测试中断言 audit 仅在 admitted=True 后增一次；旧快照无 family/extent 字段时输出 unknown，不臆造类别。多 episode 交错不能串成长零新增链。

- [ ] **Step 2 — 跑新增测试确认失败。**

```bash
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_loop_replay_audit.py -q -p no:cacheprovider
```

- [ ] **Step 3 — 用场景 descriptor 和现有转移做只读统计。**

```python
def iter_transitions(self):
    for entry in self._entries.values():
        yield entry[0]
```

审计逐条累加 episode/scene 去重、family/extent、正/零新增、真实终止达标转移、retained-order 零新增片段。样本年龄明确使用 `learner_updates - transition.actor_version`，单位是模型更新次数，不伪造采集秒数。因 Transition 无 episode step，不能把保留顺序片段声称为完整回合或证明无丢步；字段显式 `sequence_scope='retained_fragment'`。新采集窗口用 admission_index 标明范围，超过1024行按 FIFO 丢标量；内存审计不从滚动 metrics.jsonl 反推。

在 `training.py:save()` 先 `captured=capture()`，再在同一 ACK barrier 对 recent、live replay 和 `captured.record['replay']` 做审计，写入 `captured.record['counters']` 的诊断分区后保存这一份 captured。避免再调用一次 replay.snapshot 或持有第二份深拷贝。诊断开关关闭时不新增文件/计数；仍只产生原来的 resume.pt、metrics.jsonl、run.json。

- [ ] **Step 4 — 测试快照尾段、恢复缺诊断状态、字节上限和连续两次保存计数不变。**

```bash
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_loop_replay_audit.py ros2_ws/src/lunar_drl_exploration/test/test_replay_checkpoint.py ros2_ws/src/lunar_drl_exploration/test/test_training_cli.py -q -p no:cacheprovider
```

原回放 FIFO、uniform-with-replacement采样、RNG、UpdateSchedule 和字节预算均须保持。新诊断元数据计入原 checkpoint 总输出预算，不绕过资源限制。

- [ ] **Step 5 — 显式提交。**

提交信息：`feat(drl): audit live and saved replay without changing sampling`。

### Task 6: 冻结随机策略的长期价值校准

**Files:**

- Create: `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/value_calibration.py`
- Create: `ros2_ws/src/lunar_drl_exploration/test/test_value_calibration.py`
- Read: 同包 `evaluation.py`、`batch.py`、`ros_env.py`、`sac.py`

**Interfaces:**

- `soft_returns(rewards, log_probs, *, alpha, gamma, complete) -> list[float | None]`；只有真实终止且无采集错误才令 complete=True。
- `calibrate(config, checkpoint: Path, *, cases:list[dict], sampling_seeds:tuple[int,...], budget:int, output:Path, domain_base:int) -> dict`，cases 含 family/extent_m/seed；仅加载冻结 Actor/Q，不创建优化器。
- CLI `python -m lunar_drl_exploration.value_calibration --checkpoint PATH --cases PATH --sampling-seeds 11 29 --budget 512 --output PATH --domain-base 228`；cases JSON 必须由 Task 7 已冻结清单生成。

- [ ] **Step 1 — 写红测试，钉住熵起点和截断空标签。**

```python
def test_soft_return_starts_entropy_at_next_action():
    from lunar_drl_exploration.value_calibration import soft_returns
    assert soft_returns([1.,2.], [-1.,-2.], alpha=.5, gamma=1., complete=True) == [4.,2.]
    assert soft_returns([1.,2.], [-9.,-2.], alpha=.5, gamma=1., complete=True) == [4.,2.]
    assert soft_returns([1.,2.], [-1.,-2.], alpha=.5, gamma=1., complete=False) == [None,None]
```

补充空轨迹、仅一步终止、非有限 reward/logp、长度不同、混合版本轨迹拒绝；模拟预算截断或执行失败必须保留 partial return 和失败原因，完整 Q 标签为空。

- [ ] **Step 2 — 跑新增测试确认失败。**

```bash
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_value_calibration.py -q -p no:cacheprovider
```

- [ ] **Step 3 — 实现回报递推与纯冻结采样。**

```python
def soft_returns(rewards, log_probs, *, alpha, gamma, complete):
    import math
    if len(rewards) != len(log_probs):
        raise ValueError('reward/probability lengths differ')
    if not (math.isfinite(alpha) and alpha >= 0 and 0 < gamma <= 1):
        raise ValueError('invalid soft return coefficients')
    if any(not math.isfinite(float(x)) for x in (*rewards, *log_probs)):
        raise ValueError('nonfinite trajectory scalar')
    if any(x > 0 for x in log_probs):
        raise ValueError('log probability must be nonpositive')
    if not complete:
        return [None] * len(rewards)
    result = [0.] * len(rewards)
    for i in range(len(rewards)-1, -1, -1):
        future = result[i+1] - alpha * log_probs[i+1] if i+1 < len(rewards) else 0.
        result[i] = float(rewards[i] + gamma * future)
    return result
```

冻结当前检查点 Actor、Q1/Q2、alpha，逐步在 inference_mode 中计算动作分布和所选动作 Q，使用专属 CPU generator 的 multinomial；保存所选动作/位置、logp、Q1/Q2、reward分项、真值新增、终止/截断及执行原因。使用 Task 2 守卫与 Task 4 统计。每个 case/rng 都独立 env、finally close，输出失败案例而非丢弃。源模型/温度/权重哈希前后相同；记录 hard return、soft return、Q误差及完整标签分母/截断率。初始已经终止且没有动作的场景没有 Q 校准样本，不能填零误差。

- [ ] **Step 4 — 用假环境验证不更新参数/不调用 backward、真值不进入 Actor、同采样 RNG 可复现、失败无遗留 env。**

```bash
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_value_calibration.py ros2_ws/src/lunar_drl_exploration/test/test_loop_metrics.py -q -p no:cacheprovider
```

报告中明确：仅已终止轨迹的误差有选择偏差；单次 realized soft return 是随机样本，不是该状态精确 Q 真值。不得用旧贪心 hard return 替代。

- [ ] **Step 5 — 显式提交。**

提交信息：`feat(drl): calibrate frozen soft Q against matched policy trajectories`。

### Task 7: 配对 profile、清单、报告与筛查停止门

**Files:**

- Create: `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/loop_protocol.py`
- Create: `ros2_ws/src/lunar_drl_exploration/lunar_drl_exploration/loop_report.py`
- Modify: 同包 `paired_experiment.py`、`evaluation.py`
- Create: `ros2_ws/src/lunar_drl_exploration/test/test_loop_protocol.py`
- Modify: `ros2_ws/src/lunar_drl_exploration/test/test_paired_experiment.py`
- Read/reuse: `scripts/drl/compare_actor.sh`（已有隔离环境入口）

**Interfaces:**

- 新 `--profile loop-learning-v1 --phase screen|extended`；原命令缺 profile 时保持原 legacy 实验。
- `loop_config(seed, arm, output) -> TrainingConfig`：arm 为 raw/softsign10；显式设定所有 Global Constraints 的训练值及 loop_diagnostics=True。
- `loop_stages(seeds, screen=3000, total=10000) -> list[dict]` 保存完整计划；`allowed_stages(plan, phase) -> list[dict]` 控制本次可运行集合。
- `validate_manifest(existing, requested) -> None`，身份包括profile版本、源码树身份、两组模型配置、所有训练常量、场景/诊断清单、预算、初始参数校验和；另记录提交号作溯源。运行时状态与身份分开存储。
- `build_loop_report(state:dict) -> dict`，只汇总已有记录，不触发新阶段或评估。

- [ ] **Step 1 — 写红测试证明 screen 只允许两个3000阶段。**

```python
def test_screen_never_includes_extended_training():
    from lunar_drl_exploration.loop_protocol import loop_stages, allowed_stages
    plan = loop_stages([2026092301,2026092302,2026092303])
    current = allowed_stages(plan, 'screen')
    assert [(r['seed'],r['arm'],r['target']) for r in current] == [
        (2026092301,'raw',3000), (2026092301,'softsign10',3000)]
    assert len(plan) == 8
```

仿既有 Child fixture 运行 driver，断言两子进程后状态为 `awaiting_extended_review`，`--resume --phase screen` 不再启动子进程，也不自动进入 extended。`--phase extended` 没有完整且有效的screen结果时拒绝。不同profile、模型形式、输入hash或heldout清单的resume拒绝；仅文档提交导致HEAD变化而源码树不变时允许恢复，源码树变化时拒绝。旧legacy四阶段测试保持。

- [ ] **Step 2 — 跑新增测试确认失败。**

```bash
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_loop_protocol.py ros2_ws/src/lunar_drl_exploration/test/test_paired_experiment.py -q -p no:cacheprovider
```

- [ ] **Step 3 — 新 profile 复用现有串行子进程、锁、原子写入与有界日志。**

```python
def allowed_stages(plan, phase):
    if phase == 'screen':
        return [stage for stage in plan if stage['phase'] == 'screen']
    if phase == 'extended':
        return list(plan)
    raise ValueError('unknown loop experiment phase')
```

manifest 创建即冻结完整矩阵，resume只筛选未完成且本次允许的阶段；同seed参数初值必须一致。源码身份记录 `git rev-parse HEAD:ros2_ws/src HEAD:scripts/drl HEAD:config` 的三项tree ID并拒绝这些路径的未提交/未跟踪代码。只新增验证文档的提交不会破坏恢复；代码/配置树或协议改变则必须新目录，不自动收养旧checkpoint。逐次运行仍保存真实HEAD与安装包路径。

保持最多一个 GPU 子进程。SIGINT只转发当前子进程，最后落盘状态；异常或子报告有error时停止，不进入下一组。继续沿用原子替换预算，不覆盖既有旧实验目录。

运行前冻结地图清单：开发/回归含旧20个60m案例及诊断场景；留出60m月表/洞穴各10种子2026100101～2026100110。核对已知历史清单和训练命名空间，交集不为空就失败并要求在开跑前整体重定清单，不能按效果替换个别case。

screen 只在两张已使用的诊断地图（月表2026092205、洞穴2026092206，60m）各做一次最多128决策的冻结argmax；同两地图 × 采样种子11/29做最多128决策的随机策略校准；另在Task 4固定回退fixture上做一次128决策Actor评价。每组最多896次评估动作，独立于3000训练转移，不进入回放。新留出集保留到 extended，screen不使用它选模型。所有截断均显式保留。

extended若另获批准，每个最终模型在20个留出场景以512决策做argmax评价，并在上述两个开发场景 × 两采样种子做512决策Q校准；仍不自动运行120/250m。配置与清单进入profile身份，不能与screen的128步结果混算。

- [ ] **Step 4 — 报告保留失败和实测资源，不从熵或TD loss宣布胜负。**

报告按训练seed、family、map seed列出配对结果；相同地图缺一臂时标missing，不删去成“公平共同子集”。完成率分母包含所有已尝试案例，未尝试另列；回归和留出不混算。记录实际转移/超额/更新/耗时/GPU峰值/当前产物大小、数值错误、恢复新增事件及删失、Q校准完整样本分母、三类经验统计。schema标明统计单位是场景/训练种子，不把动作数当独立样本数。尚未运行的阶段显示NOT_RUN。

```bash
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_loop_protocol.py ros2_ws/src/lunar_drl_exploration/test/test_paired_experiment.py ros2_ws/src/lunar_drl_exploration/test/test_evaluation.py -q -p no:cacheprovider
```

- [ ] **Step 5 — 显式提交。**

提交信息：`feat(drl): gate loop-learning paired trials after the screen stage`。

### Task 8: 完整工程验证、有限筛查与文档交付

**Files:**

- Create: `ros2_ws/src/lunar_drl_exploration/test/test_loop_training_integration.py`
- Modify: `docs/DRL开发入口.md`、`docs/操作指令.md`
- Create: `docs/validation/2026-09-23-drl-loop-learning-screen.md`
- Runtime artifacts only: `/home/kai/.cache/lunar-drl-loop-learning/experiments/20260923/`

**Interfaces:**

- 复用 `run_training(..., max_transitions=N, env_factory='worker_fixtures:ControlledEnv')` 做2环境小预算真实优化器/IPC集成测试；正式screen仍8环境/完整网络。
- 复用 `scripts/drl/build.sh`、`compare_actor.sh`，显式设置独立 DRL_CACHE；不更改旧默认输出目录。

- [ ] **Step 1 — 写端到端红测试，优先验证有界恢复和新旧模式隔离。**

先写以下真实IPC集成测试，再加异常注入：

```python
def test_loop_diagnostics_resume_counts_only_admitted_transitions(tmp_path):
    from dataclasses import replace
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.training import run_training
    from lunar_drl_exploration.checkpoint import CheckpointManager
    from lunar_drl_exploration.replay import loads_transport
    cfg = replace(TrainingConfig(), environments=2, warmup=0,
                  output_dir=str(tmp_path), system_reserve_bytes=0,
                  loop_diagnostics=True, save_interval_s=.2)
    first = run_training(cfg, device='cpu', max_transitions=8,
                         env_factory='worker_fixtures:ControlledEnv', probe_budget=3)
    second = run_training(cfg, resume=True, device='cpu', max_transitions=4,
                          env_factory='worker_fixtures:ControlledEnv', probe_budget=3)
    saved = CheckpointManager(tmp_path).load(cfg).record
    audit = loads_transport(saved['counters'])['loop_diagnostics']
    assert audit['recent']['total_observed'] == saved['schedule']['transitions']
    assert audit['at_checkpoint']['admitted'] == saved['schedule']['transitions']
    assert saved['schedule']['unfinished_reservations'] == 0
    assert set(first['episode_ids']).isdisjoint(second['episode_ids'])
```

补测新profile配置下采集/更新/导出/加载；数值异常/模拟OOM/保存空间不足时原checkpoint哈希不变。普通默认配置仍只生成原三种运行文件。所有新守卫和审计走真实IPC，不用只调用mock回调代替。

```bash
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_loop_training_integration.py -q -p no:cacheprovider
```

- [ ] **Step 2 — 修接入问题后跑全包测试、重新构建与包测试。**

```bash
scripts/drl/build.sh --packages-select lunar_drl_exploration
drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test -q -p no:cacheprovider
```

再从清洁子shell加载同一个新Jazzy install，运行：

```bash
colcon --log-base /home/kai/.cache/lunar-drl-loop-learning/jazzy/test-log test \
  --base-paths ros2_ws/src --packages-select lunar_drl_exploration \
  --executor sequential --build-base /home/kai/.cache/lunar-drl-loop-learning/jazzy/build \
  --install-base /home/kai/.cache/lunar-drl-loop-learning/jazzy/install \
  --event-handlers console_direct+
colcon test-result --test-result-base /home/kai/.cache/lunar-drl-loop-learning/jazzy/build --verbose
```

如因现有依赖或测试配置无法执行，记录NOT_RUN及原因，不将pytest替代成colcon通过。含重型规划CTest的包按包串行，不做全仓并行压力测试。

- [ ] **Step 3 — 隔离原生导航、GPU更新与恢复探针。**

先核对 domain 200、216～223、228：实际训练只用216～223，单环境校准/回退评价用228，原生既有测试用200。检查这些domain无其它作业；若有占用，整体换空闲合法范围并记录，不终止他人进程。8环境时base必须不大于225；不能用228作为8环境训练base。

```bash
LUNAR_DRL_RUN_ROS=1 drl_python -m pytest ros2_ws/src/lunar_drl_exploration/test/test_ros_native_execution.py -q -p no:cacheprovider
```

随后按新profile两臂各运行160条新转移的独立smoke目录，只验证工程，不计入正式screen、不降低正式warmup：

```bash
drl_python -c '
from pathlib import Path
from dataclasses import replace
from lunar_drl_exploration.loop_protocol import loop_config
from lunar_drl_exploration.training import run_training
for arm in ("raw", "softsign10"):
    output = Path("/home/kai/.cache/lunar-drl-loop-learning/experiments/20260923/smoke") / arm
    cfg = replace(loop_config(2026092301, arm, output), warmup=64)
    run_training(cfg, device="cuda", max_transitions=160,
                 probe_extent=40, probe_budget=8, domain_base=216)
'
```

另起进程用同样cfg和目录，传 `resume=True,max_transitions=32` 验证恢复；若需要从头重跑，创建有明确新后缀的smoke目录，不删除第一次失败证据。记录源码/安装路径、CUDA驻留、更新、Actor导出/重载及所有ROS子进程清理。smoke配置只使用上述显式probe覆盖，不写入screen目录。

- [ ] **Step 4 — 通过数值门、全包测试和smoke后，启动且仅启动screen。**

```bash
DRL_CACHE=/home/kai/.cache/lunar-drl-loop-learning/jazzy \
  scripts/drl/compare_actor.sh \
  --profile loop-learning-v1 --phase screen \
  --output-dir /home/kai/.cache/lunar-drl-loop-learning/experiments/20260923/paired \
  --seeds 2026092301 2026092302 2026092303 \
  --screen-transitions 3000 --total-transitions 10000 \
  --domain-base 216 --device cuda
```

`--total-transitions` 仅冻结将来的完整矩阵，不能让screen越过两组3000。开始前检查GPU/内存/磁盘和原训练状态，无足够资源就报告，不终止其它作业。执行中展示日志路径与有限进度；首组完成后用实际转移速率估算余时，并说明阶段和异步超额。停止只向本轮精确PID/PGID发有序信号，必要时按现有安全流程核查，不按进程名批量杀死。

若用户中断，用同一命令加 `--resume` 且仍为 `--phase screen` 恢复；任何数值/基础设施失败均先停、保留证据，不悄悄重跑失败组。结束确认 `awaiting_extended_review`、无训练/评估子进程、原检查点哈希未变。不得因为3k结果“不够看”自行扩展。

- [ ] **Step 5 — 写验证记录及两处入口文档，真实填写通过/失败/未运行。**

验证记录必须包括：代码提交与输入hash、全部配置、旧tanh失败的处理方式、各测试/构建命令和结果、24次冻结拟合、三类经验审计、Q校准有效/截断样本、两臂实际admitted/update、逐场景结果、资源/耗时和未完成项。不能把测试通过写成解环成功，不能把静态即时增益写成长期收益。

入口文档给出新profile的启动/同阶段恢复/停止方法及与旧长训的边界；默认C0和旧目录仍不变。按设计说明下一步应优先调查Actor、数据或Critic哪一环；若证据不足就保留“不足”，不自动实现多步TD/经验重采样/熵拆分。

- [ ] **Step 6 — UTF-8完整读取、diff检查、最终独立代码审查与显式提交。**

提交信息：`docs(drl): record loop-learning screen evidence and operating boundaries`。审查方式随用户选择的执行方法确定；本计划编写阶段未委派子代理。完成后不自动合并、推送或扩大训练。

## 需求覆盖与最终交接

| 设计要求 | 对应任务/门槛 |
| --- | --- |
| 保留C0/旧C10、softsign身份及数值风险 | 1、2 |
| 当前样本与原/新Adam分开、t0/学习分开 | 3，失败不进在线 |
| 同初始化、同场景协议、microbatch8、3k停止 | 1、7、8 |
| 真正恢复覆盖、长循环、删失、必要回退 | 4、7 |
| 冻结随机策略soft-Q、截断及终止偏差 | 6、7 |
| 新采集/内存/快照分开，保持采样不变 | 5 |
| 旧20案例为开发集，留出集不用于screen调参 | 7 |
| 有益回退与失败保留，20例留出、多种子后续 | 4、7、8；extended另审阅 |
| 完整测试、隔离Jazzy、原检查点保护、文档 | 1～8 |
| 熵拆分/新Critic目标/经验保留算法 | 明确不在本批实现；按设计第8节依证据再设计 |

计划自查顺序：逐项对照上表 → 检查内容完整 → 核对函数/字段/CLI名称一致 → 核对Review Focus均有测试 → 核对没有将NOT_RUN写成通过。

当前交接点：请用户审阅本实施计划并选择执行方式。推荐本会话顺序执行：模型身份、更新边界、诊断和runner接口耦合较多，由同一实现者串行推进便于保持一致；最后进行一次独立整体审查。也可选择逐任务子代理实现与审查，代价是更多上下文与审查开销。选择后再开始Task 1，不重做已经确认的设计。
