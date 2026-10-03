# 任务内外前沿收益：任务1基础工程与任务2前沿原语验证

日期：2026-10-04。用户已批准设计、实施计划及子代理执行。本记录确认双熵基础工程和任务2实测前沿/消除原语；新的图特征、奖励接线、Critic预测和训练尚未实现或启动。

## 来源与隔离

专项工作树 `/home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/drl-task-frontier`，分支 `feat/drl-task-frontier`。任务起点 `e0cca4e0a789a1d794ec70054904f5dbae716fda`；开发线和与双熵来源的共同祖先为 `32155954f14b69d69ecb06d3b73bc7f7e09d084b`。任务开始专项树干净，HEAD相对开发线为1/0。主工作树的未跟踪 `deployment/config/wheel.yaml` 保留。

按计划仅 cherry-pick `2d56d68ec3d569138acbf62adc96f50d6ae77345`，专项提交为 `81ece811`；未引入后续位置辅助损失提交 `dcca338b`。双熵仍使用该基础的接口和数学语义，`position_aux_weight=0.0` 可以序列化和恢复，非零或非法类型明确拒绝。缺失此字段的旧记录规范化为零，仅为基础配置兼容，不代表允许旧模型/回放混入后续新前沿语义。

只读核验保留的26856基线：

- Actor：`/home/kai/CodexDownloads/lunar_navigation/drl-small-recovery-20260926/long-small-control/origin.pt`；`task_graph_v3`、version=26856；SHA256 `ab2f144f1f201b2abe189e06bcca8ca3c644d936d1d22668c4b8623b2bfc2278`。
- 完整来源：`/home/kai/CodexDownloads/lunar_navigation/drl-small-recovery-20260926/entropy-extended-split/resume.pt`；`bounded_training_v1`、transitions=108452、updates=26856；SHA256 `2a1239f0d24322aea9ed1682e11805d98781a006387e7071fbb7803922639eae`。

旧模型、回放、安装目录及历史工作树均保留。未合并、推送、创建PR或清理工作树。

## 独立构建与环境限制

```bash
DRL_CACHE=/home/kai/.cache/lunar-drl-task-frontier/jazzy DRL_BUILD_JOBS=2 scripts/drl/build.sh --symlink-install
```

首次冷构建完成五包后，DRL失败：`cc1plus: fatal error: native/graph.cpp: No such file or directory`。源码存在，但colcon的symlink开发目录未链接包的 `native/`。控制器确认本轮只修复专用缓存，暂不扩到源码打包修改：

```bash
ln -s /home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/drl-task-frontier/ros2_ws/src/lunar_drl_exploration/native /home/kai/.cache/lunar-drl-task-frontier/jazzy/build/lunar_drl_exploration/native
DRL_CACHE=/home/kai/.cache/lunar-drl-task-frontier/jazzy DRL_BUILD_JOBS=2 scripts/drl/build.sh --symlink-install
```

补救构建：6 packages finished [5.98s]，退出0。没有安装依赖或修改共享缓存。全新cache的symlink构建仍需上述补救，源码打包修复留待后续原生交付任务。

## 测试环境与证据

本轮外部测试启动器 `/home/kai/.cache/lunar-drl-task-frontier/task-1/test.sh` 清除继承的 AMENT/COLCON/CMAKE、ROS_PACKAGE_PATH、PYTHONPATH、LD_LIBRARY_PATH 和 ROS版本变量，加载 `/opt/ros/jazzy/setup.bash` 与专项 `install/local_setup.bash`；设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，prepend本专项源码包，并使用现有 `/home/kai/.cache/lunar-drl-training/venv/bin/python`。

实际导入为 ROS_DISTRO=jazzy、Torch=2.8.0+cu128。config/sac来自专项源码；terrain/graph原生模块来自专项 `jazzy/build/lunar_drl_exploration/*.so`；生成消息来自专项 `jazzy/install/lunar_planning_msgs/`，未混入旧overlay。

新增测试先执行，RED为9 failed in 0.08s，原因均为基础配置缺少 `position_aux_weight` 接口（unexpected keyword argument），不是缺ROS消息。实现字段与校验后GREEN为9 passed in 0.06s。测试覆盖零权重JSON/配置恢复、旧记录默认零、正/负/NaN/Inf/布尔/字符串拒绝及恢复记录拒绝。

聚焦验证：

```bash
bash /home/kai/.cache/lunar-drl-task-frontier/task-1/test.sh -m pytest ros2_ws/src/lunar_drl_exploration/test/test_factorized_entropy.py ros2_ws/src/lunar_drl_exploration/test/test_entropy_fork.py ros2_ws/src/lunar_drl_exploration/test/test_contracts_maps.py ros2_ws/src/lunar_drl_exploration/test/test_frontier_config.py -q
```

结果：71 passed in 15.79s。现有不可变契约fixtures原样运行，未调整旧schema断言。仅源码环境缺少生成消息而导致测试失败属于启动环境错误，不能记为代码失败；本轮正式验证使用完整独立overlay。

完整受影响DRL测试：

```bash
bash /home/kai/.cache/lunar-drl-task-frontier/task-1/test.sh -m pytest ros2_ws/src/lunar_drl_exploration/test -q
```

结果：624 passed, 2 skipped in 128.42s，退出0。两项为 `test_ros_native_execution.py::test_native_cold_start_spin_adjacent_arrival_and_frozen_transition` 的两个参数，需 `LUNAR_DRL_RUN_ROS=1` 显式开启；本任务未运行实际ROS动作闭环。`git diff --check`通过。

外部证据目录 `/home/kai/.cache/lunar-drl-task-frontier/task-1/` 包含red/green、focused/full-suite、imports/source-identity、build-retry和测试启动器；首次冷构建日志 `/home/kai/.cache/lunar-drl-task-frontier-build.log`。构建、日志和模型不入Git。

## 验证边界

这是基础工程通过，不是新前沿收益已完成或模型改善。只运行任务测试（包含既有测试夹具的有界合成更新），没有正式训练或冻结评测。新前沿奖励、真实环境短训练/恢复、月表/洞穴烟测与完整阶段：NOT_RUN。Humble、Orin、DDS部署、传感器和实车：NOT_RUN。

## Task 2：实测前沿与共享消除定义

起点 `26c805d70d4a36c57b40c7ae81664bc3cba9ef59`，实现者完成后由控制器独立审查。新增不可变 `FrontierSet` CSR与 `FrontierSnapshot`，由累计有效 K 和当前实测 intrinsic FREE 生成8邻域内外需求。中心和需求均按同域任务掩码分类，边界属于任务内；当前地图不裁区外，前沿不依赖可达连通性。旧未知接口、见证证书、图布局、奖励及模型schema保持原语义。

`count_removed` 要求旧前沿不再存在、全部冻结旧同域需求已有效观测、旧中心仍在当前 `free_ids`；区外独立计数，新增前沿不冲销旧消除。相同revision返回0，旧revision或epoch/resolution/phase/task/polygon身份不符拒绝。该函数无状态，后续动作消费者必须推进动作前冻结快照；重复调用同一不同revision快照对仍是同一计数，不承担全局去重。

控制器预检发现旧 CoverageHistory 对绝对origin变化清空K，不满足已批准的稳定世界格定义。本任务按修订把 K 存在固定相位的packed-bit世界tile，原点平移时重投影 `mask`/`bounds`；公开 `identity` 仍为当前 `(epoch,resolution,origin)`，新增 `coordinate_identity` 用于面积报告和方向历史连续性。整数origin旧回归改为保留35格，并验证 `RosExplorationEnv._privileged` 投影仍保留35格。非整数相位/epoch/resolution改变清空历史。coverage/frontiers共享整数位移算法（绝对容差1e-10格），相对保留的首次原始origin判断连续性，独立快照按同一容差对齐相位与格ID，FREE/需求ID为有符号世界格打包，中心保留实际米坐标。

### RED / GREEN 与变异验证

- 新模块未实现时，两测试文件首次收集均因 `lunar_drl_exploration.frontiers` 不存在而失败（2 errors，0.21s），符合该任务指定RED入口。
- 原点回归先执行：3 failed、6 passed（0.36s）；失败为旧测试保留35格断言及两个同栅格平移重复面积断言，属于真实行为缺口。
- 首次实现后22 passed（0.36s）；补充ID、实际中心、不可变/非法CSR和机会证书回归后最终聚焦42 passed（0.57s）。
- 实际临时变异：ALL需求改为ANY、移除当前FREE中心门控，各导致1项对应断言失败（其余10项通过）。最初ALL→ANY变异被“旧front仍存在”门控挡住而未失败，因此补充了丢失front条目但旧需求尚未全部解决的冻结快照测试；最终两个变异均被捕获，源码已恢复。
- 工具阶段曾出现一次patch重复目标校验拒绝（无文件写入）及两次只读路径探测错误（不存在的 `src/lunar_drl_exploration` / `snapshot.py`），已纠正；不属于测试失败。

聚焦命令：

```bash
bash /home/kai/.cache/lunar-drl-task-frontier/task-1/test.sh -m pytest ros2_ws/src/lunar_drl_exploration/test/test_frontier_sets.py ros2_ws/src/lunar_drl_exploration/test/test_frontier_removal.py ros2_ws/src/lunar_drl_exploration/test/test_cumulative_coverage.py ros2_ws/src/lunar_drl_exploration/test/test_task_analysis.py -q
```

完整DRL包使用前文同一启动器和包级命令：646 passed、2 skipped in 121.29s，退出0；两项skip仍是 `test_ros_native_execution.py::test_native_cold_start_spin_adjacent_arrival_and_frozen_transition` 的显式ROS闭环参数。没有启动学习任务或评测。源码UTF-8及 `git diff --check` 验证通过。外部日志位于 `/home/kai/.cache/lunar-drl-task-frontier/task-2/`：`red-frontiers.log`、`red-coverage.log`、`green-first.log`、`focused.log`、`mutation-demand-all.log`、`mutation-free-center.log`、`full-suite.log`。

本任务是实测前沿/消除原语及覆盖连续性通过；方向可见性特征、实际奖励接线、Critic虚拟观测、schema升级、训练/评测均属于后续任务。没有更改传感器或原生地图分类规则，没有重建/修改旧overlay与模型。Humble、Orin、DDS、实车仍为NOT_RUN。


### Task 2 独立审查修复 I1

审查复现 resolution=0.2、origin=0.02469135781 加256格时，独立量化相位分别落在舍入边界两侧，错误清空K并拒绝前沿比较。本轮改为相对保留的首次原始origin判断整数位移（绝对容差1e-10格）；复用原坐标身份，工作区携带同一phase/offset。独立前沿快照不再精确比较各自相位；容差判断后对齐必要的整数格ID偏移，覆盖相位跨0/分辨率边界。

先写回归：3 failed in 0.26s，修复后3 passed in 0.22s。增加±1e-16原点案例；临时绕过ID重对齐导致1 failed、2 passed（0.17s），证明独立比较没有只放宽身份而忽略ID。恢复源码后运行test_cumulative_coverage、test_frontier_sets、test_frontier_removal、test_task_analysis、test_graph_decision五文件：67 passed in 2.94s，退出0。日志在原Task 2目录的fix1-red.log、fix1-green.log、fix1-mutation-rebase.log、fix1-focused.log。本轮按控制器范围未重复包级测试；前述646通过是修复前提交的历史证据。无训练/评测；原验证边界继续有效。

## Task 3：分区收益输入与合法转向

起点 `1aeff4e6b9bdcd2efb83962a16429f7978071b4a`。新增统一特征列定义：节点27维，前三列相对XY/任务内标识；任务内收益 `3:11`、任务外收益 `11:19` 各为8方向FREE前沿中心数量除100；真实执行方向历史 `19:27`。每动作7列为相对XY、cos/sin、任务内收益、任务外收益、方向历史。DecisionObservation默认 `task_graph_v4`，批处理、Actor/Q共用MeasuredEncoder同步27/7维。

`directional_frontier_counts` 返回int64[N,8]，只有FREE中心与至少一个冻结同域需求均可见时记1；两个需求仅见一个仍计一个中心，需求可见但中心不可见为0。空间索引按原生fan方形包络保守裁剪，实际range/FOV/遮挡继续使用原生directional_visibility，不引入近似射线或bearing过滤。区外站位可以有任务内收益，任务外通道独立。

DecisionCore将analyzer已冻结FrontierSnapshot交给GraphBuilder，独立build才使用同workspace构造。原有unknown pending接口/见证证书继续负责几何补点与合法原地扫描，和新收益分离。区外前沿不产生任务扫描；任务内FREE前沿为空但跨边界pending仍可见时保留扫描。零收益通行和回退保持既有几何规则。

控制器批准补充 `measured_workspace(...,coverage=None)` 的稳定lattice phase/offset：从snapshot初始origin调用共享helper，coverage分支仍保留原身份。曾复现origin_x=-1e-16经workspace平移丢精度，独立量化把遮挡需求错配到可见阻挡，错误返回8个1。计数器现在只复用workspace的稳定offset，不引入另一套ID量化；±1e-16独立GraphBuilder/可见性回归通过。

### RED / GREEN及最终证据

- 初始新文件执行：8 failed in 0.95s，失败为缺计数接口、旧19维/default v3、区外通道缺失以及跨边界pending误作任务内收益；实现后8 passed in 0.89s。
- 补充lattice RED：1 failed、1 passed、10 deselected in 0.23s；修复后新文件12 passed in 0.95s。最初仅正收益的lattice夹具没有抓住错配，随后加强为BLOCKED需求才得到有效RED。
- 移除中心可见门控和ANY需求改ALL的两次临时变异，各导致对应测试1 failed；均已恢复源码。
- 新文件加原计划六文件：61 passed in 7.14s。覆盖实际历史位19:27、固定人工7列、Actor/Q在动作重排后输出与动作对应、全8方向native观测oracle和±整数滚动offset。
- 一次完整DRL包：661 passed、2 failed、2 skipped in 130.46s。两失败仅为旧fixtures：GraphBuilder猴补漏透传新增frontiers关键字、off-axis pending扫描仍期望正任务内FREE前沿收益。更新这两处fixtures后，完整对应文件加原聚焦集合：80 passed in 7.22s，退出0。生产源码没有因这两项失败再变更；按修改覆盖原则未再次执行全包。两skip仍是显式开启的实际ROS动作闭环参数。

最终聚焦命令：

```bash
bash /home/kai/.cache/lunar-drl-task-frontier/task-1/test.sh -m pytest ros2_ws/src/lunar_drl_exploration/test/test_frontier_features.py ros2_ws/src/lunar_drl_exploration/test/test_current_opportunities.py ros2_ws/src/lunar_drl_exploration/test/test_graph_decision.py ros2_ws/src/lunar_drl_exploration/test/test_valid_actions.py ros2_ws/src/lunar_drl_exploration/test/test_metric_graph.py ros2_ws/src/lunar_drl_exploration/test/test_model.py ros2_ws/src/lunar_drl_exploration/test/test_graph_critic_schema.py ros2_ws/src/lunar_drl_exploration/test/test_task_analysis.py -q
```

完整包命令沿用前述专项启动器与整个test目录。外部证据 `/home/kai/.cache/lunar-drl-task-frontier/task-3/`：red.log、green-first.log、lattice-red.log、green.log、focused-final.log、mutation-center-gate.log、mutation-all-demands.log、full-suite.log、focused-after-package.log、imports.log。显式UTF-8核验、git diff --check通过。导入确认本工作树源码、专项Jazzy native/generated msgs、既有venv；旧overlay与模型未改。

此任务仅为特征/合法动作层通过，等待控制器独立审查。Task4实际奖励/Critic预计消除、Task5发布/恢复schema尚待完成，sac/runtime发布元数据仍属旧v3阶段，当前中间提交不能当作新语义checkpoint发布或训练入口。冷构建修复延后Task8；无正式训练、冻结评测、合并、推送或PR。Humble、Orin、DDS、实际ROS动作闭环、传感器与实车均NOT_RUN。


## Task 4：全动作奖励和Critic终点预计消除

起点 `39f6fa45811ec4eaee4c36382c2444e650b3bc75`。`RewardParts` 新增必填非负整数 `removed_frontiers_in/out`，实际面积、距离、绝对转角继续保留。`make_transition` 无旧面积回退：任务内消除/50，减整段实际距离/转角和每步成本，成功时加原配置完成奖金（默认5）；外部消除只诊断。`step` 在执行前冻结不可变 `core.frontiers`，最终地图ACK及 `core.observe` 后调用同一 `count_removed`；停滞仍按真实面积，成功仍无碰撞且reference coverage≥99%，禁止终止后再次step领奖。

训练专用 `predict_removed` 对终点姿态作一次真值原生观测，只将有限高度且intrinsic非UNKNOWN命中加入已知格，更新有效命中旧中心的当前FREE/BLOCKED分类。虚拟后快照revision+1，只投影旧冻结前沿中心，CSR保留仍未知的同域需求；新前沿不会抵消旧消除，因此本投影无需新增中心。最终仍调用共享消除原语，要求ALL旧同域需求解决、旧front消失、旧中心当前FREE。该投影方式经控制器确认，没有把真值导入部署frontiers模块。

`PrivilegedActionContext.removed_frontiers` 为不可变int64原始计数；`PrivilegedBatch.expected_removed_frontiers` 仅打包时除50，Critic读取该列。Actor 27/7维、内外可见ANY需求计数/100、候选集合沿用Task3；真值差异仅改变Critic计数。终点预测和整段沿途收益有意不同，测试证明两侧各一次观测可消除1，而任一端点单独预测0。

生产修改保持批准的 contracts/ros_env/privileged/batch/model 五文件。必填字段变化同步既有model、replay、SAC、worker、evaluation等测试fixtures，已向控制器说明；没有扩大生产文件范围。

### RED / GREEN及最终证据

- 新两文件实现前 `20 failed in 0.96s`：缺计数合同、旧step无计数字段、predict_removed不存在。首次实现后17通过、3失败来自测试误改TerrainGrid只读数组，改用测试自有copy后解决。
- 首轮六文件62通过、1失败来自旧面积收益单调递减断言。前沿ALL需求语义下东侧需求先解决，会使西侧下一次可消除从5升到8，已用明确前后计数回归替换旧假设。
- 受影响13文件：`178 passed in 14.37s`，退出0。包含任务指定6文件及model、cumulative_coverage、replay_checkpoint、replay_sequences、sac、soft_retrace、evaluation。
- 最终新增区外真实step和下一动作重复revision回归，以及新增面积1/消除0仍重置停滞断言：新两文件加test_ros_env，`54 passed in 1.71s`，退出0。
- 一次完整DRL包：`687 passed, 2 skipped in 133.11s (0:02:13)`，退出0。无失败与警告；两skip为 `test_ros_native_execution.py::test_native_cold_start_spin_adjacent_arrival_and_frozen_transition` 的真实ROS闭环参数。

聚焦与全包均使用 `/home/kai/.cache/lunar-drl-task-frontier/task-1/test.sh -m pytest`。新两文件RED命令及最终全包命令：

```bash
bash /home/kai/.cache/lunar-drl-task-frontier/task-1/test.sh -m pytest ros2_ws/src/lunar_drl_exploration/test/test_frontier_reward.py ros2_ws/src/lunar_drl_exploration/test/test_frontier_privileged.py -q
bash /home/kai/.cache/lunar-drl-task-frontier/task-1/test.sh -m pytest ros2_ws/src/lunar_drl_exploration/test -q
```

原生有效观测oracle、非有限/UNKNOWN、有效BLOCKED需求、中心重分类、负/分数原点及±1e-16、区外站位、只归一化一次、隐藏真值与Actor隔离、整段冻结/ACK/一次完成奖金均有测试。外部日志 `/home/kai/.cache/lunar-drl-task-frontier/task-4/`：red.log、green-first.log、focused-first.log、focused.log、focused-final.log、full-suite.log、imports.log。UTF-8读取及git diff --check通过。

导入确认当前专项源码、专项Jazzy native/generated messages和既有venv。旧overlay/checkpoint/replay未修改。正式训练、冻结评测、真实ROS闭环、Humble、Orin、DDS部署和实车：NOT_RUN。发布及resume元数据仍待Task5，不能用此中间提交混用旧checkpoint。没有合并、推送或PR；等待控制器独立审查。

## Task 5：训练、经验、发布、导出和恢复的统一语义身份

起点 `432554c8e31b8efc590973d81d6aca874c34539f`。`feature_schema.py` 提供 `task_graph_v4`、`candidate_frontier_truth_v2`、`candidate_graph_sac_v3` 与完整 `FRONTIER_SEMANTICS`，记录FREE中心/8邻域同域未知需求、边界归属任务内、中心与ANY同域需求的8方向原生FOV、ALL冻结同域需求有效观测且旧前沿消失/中心仍FREE、整段距离及绝对转角、内外输入各/100、奖励与Critic消除/50、原始整数单位和奖励公式。外层 `bounded_training_v1` 保留；内部身份必须全部匹配。

控制器补充的 `contracts.py` 变更将DecisionObservation默认/检查绑定统一常量；同27维错标签在构造、transport、replay与runtime均拒绝。PrivilegedState/ActionContext具有显式v2标签，transport禁止缺标签静默套新默认；旧面积字段不能解码成原始消除计数。Actor和learner快照、collector初始及PUBLISH、runtime与export共享完整身份验证。PUBLISH先验证/构造replacement再替换；export先验证后原子写文件。旧19维、仅维度相同但奖励/尺度错误、旧replay envelope、checkpoint内部及replay内部不一致均有保留有效目标bytes的回归。

`TrainingState.validate()` 无显式config时也从保存配置重建预期身份，因此save/export不能绕过旧语义拒绝。`LDRLRP3` envelope与 `bounded_replay_v2` 携带语义；保存/加载验证实际payload，并逐条admission/evict，避免额外全量ledger；restore将同一次验证构建的replay直接复用。校验保留Python/Torch RNG，不从旧source初始化或修改旧artifact。真实一个split-SAC update后的五网络、五Adam、双温度、schedule/replay、Python/NumPy/Torch/采样/课程/collector RNG、计数和课程逐项完全一致；导出概率与learner Actor零误差。

### RED / GREEN及最终证据

初始新增identity文件执行 `15 failed, 2 passed in 4.51s`：缺三schema及FRONTIER_SEMANTICS、v3发布、同形错标签未拒绝、无config的save校验接受错误内层语义。实现后初版 `17 passed in 4.68s`；补充内部replay错误奖励/privileged及真实fresh runner边界后，新文件为20项。

```bash
bash /home/kai/.cache/lunar-drl-task-frontier/task-1/test.sh -m pytest -q ros2_ws/src/lunar_drl_exploration/test/test_frontier_identity.py
bash /home/kai/.cache/lunar-drl-task-frontier/task-1/test.sh -m pytest -q ros2_ws/src/lunar_drl_exploration/test
```

一次全包输出 `704 passed, 3 failed, 2 skipped in 133.12s (0:02:13)`；三失败均为fork测试旧错误信息正则要求 `learning config`/`model config`，新无config校验已正确更早拒绝并报 `incompatible full resume semantics: learning/model`。仅更新这两文件的错误信息断言，生产源码未因这三失败更改；随后按受影响范围运行全部指定文件及这两文件：

```bash
bash /home/kai/.cache/lunar-drl-task-frontier/task-1/test.sh -m pytest -q ros2_ws/src/lunar_drl_exploration/test/test_frontier_identity.py ros2_ws/src/lunar_drl_exploration/test/test_replay_checkpoint.py ros2_ws/src/lunar_drl_exploration/test/test_runtime.py ros2_ws/src/lunar_drl_exploration/test/test_training_identity.py ros2_ws/src/lunar_drl_exploration/test/test_collector.py ros2_ws/src/lunar_drl_exploration/test/test_sac.py ros2_ws/src/lunar_drl_exploration/test/test_evaluation.py ros2_ws/src/lunar_drl_exploration/test/test_multistep_fork.py ros2_ws/src/lunar_drl_exploration/test/test_training_fork.py
```

最终 `151 passed in 62.73s (0:01:02)`，退出0，无警告。自审补齐save预算前置检查的replay bytes类型错误后，新identity及replay_checkpoint再覆盖 `37 passed in 7.88s`，退出0。`compileall`、逐文件UTF-8读取与 `git diff --check` 通过。完整包不是全绿声明；上述151与37是最后变更的覆盖证据，两skip仍是未启用的实际ROS动作闭环参数。

外部证据：`/home/kai/.cache/lunar-drl-task-frontier/task-5-{red,green,affected,affected-final,focused-final,package,final,selfreview-final,imports,memory-example}.txt`。导入证据：本专项源码、`ROS_DISTRO=jazzy`、指定venv、专项Jazzy build native与install的lunar_planning_msgs。所有训练/评估调用都是既有ControlledEnv或合成测试fixture，不是正式场景训练或冻结评测；未创建子代理、合并、推送或PR，旧overlay/26856/source checkpoint/replay保留。

### 内存及验证边界

严格payload校验必须短暂持有解码record和不可变数值backing；save-only不保留完整新ledger并在返回时释放。序列化snapshot字节上限不等于RSS/PSS上限：合成1条样本实测serialized为5239字节，完整decode replay ledger计费43778字节（这不是RSS测量）；validate-only返回None。现有ResourceMonitor在训练循环采样，不对save内瞬时allocation单独预留，原子文件总输出预算仍执行。依控制器范围不新增内存预留框架；Task8实际smoke仍须测save/resume峰值。正式训练/冻结评测、真实ROS闭环、Humble、Orin、DDS部署、传感器及实车均NOT_RUN。状态：实现完成，等待控制器独立审查。

## Task 6：精确阶段补完更新与中断信用恢复

起点 `5376eca3dcedf7d0bac92ef7085fce3b723f327a`。正常预算耗尽仅停止新GRANT，继续对已有DONE准入/ACK，并完成所有可支付SAC更新；整除阶段credit=0，任意非整除预算保留不足1余额。终态顺序为最终PUBLISH → 匹配版本PUBLISHED → 新BARRIER快照 → save → STOP → STOPPED。每个最终确认等待有12秒上界，已退出collector的大帧有序交接仍处理。SIGINT/资源stop完成已经开始的同步更新，保留其他信用；先确认完整Actor再STOP，对已完成消息ACK-drain后保存。部分优化器失败仍不得保存或发布污染状态。

CLI/API增加仅resume允许的 `--max-transitions 0`，不产生新GRANT/admission，只偿还旧整数信用；负数和fresh零预算拒绝。任何resume先还旧整数债再派发新转移，fractional信用保留。target0终端与run.json均报告本次采集0、overshoot0。既有schedule Fraction和reservation API已足够，依控制器裁定复用，不新增接口。

有效TDD RED：两阶段真实collector/worker测试在第一阶段只保存1次更新而预期2；三种零预算API恢复与合法CLI resume0被正数校验拒绝。命令：

```bash
bash /home/kai/.cache/lunar-drl-task-frontier/task-1/test.sh -m pytest -q ros2_ws/src/lunar_drl_exploration/test/test_stage_credit_drain.py ros2_ws/src/lunar_drl_exploration/test/test_training_cli.py -k 'exact_two or zero_admission or zero_budget'
```

修复前 `5 failed, 3 passed, 33 deselected in 6.19s`，最小实现后同命令 `8 passed, 33 deselected in 18.43s`。首版测试曾因batch2违反现有batch64配置约束失败，已改为64；直接执行wrapper的权限126也改用bash处理，这些夹具/启动错误不作为功能RED。

覆盖：两个scaled stage分别保存 `(T,U,C)=(12,2,0),(20,4,0)`，最终Actor逐张量等于learner；生产10阶段9216/2048及后续算术由纯schedule测试验证。八个真实worker的DONE通过Event等全部完成后释放，达到8个并发reservation且信用投影≤32；没有把32信用误当32个reservation。credit1/32恢复夹具由4/128条ControlledEnv transition逐次ledger准入并经完整Checkpoint校验构造，零预算分别执行1/32个真实SAC更新；credit1/4保留且后续3条恰好支付一次。

真实SIGINT通过外层spawn父进程和Pipe握手，命中pre-loop、update前、update返回后、schedule记账后、PUBLISH前/后、PUBLISHED后。已开始同步update仍完整执行一次；其余余额保留。恢复核验learner/schedule、回放计数/字节、collector acknowledged与ledger last_admitted、最终Actor及children/IPC线程关闭。实际wrapper SIGTERM启用warmup0，补验信用公式与最终发布版本。缺失PUBLISHED/BARRIER的真实raw-pipe帧测试证明等待有界、无假成功且checkpoint SHA256不变。既有延迟STOPPED/有序EOF继续覆盖；正常save先于STOP，因此STOPPED丢失时保留已确认的terminal checkpoint并报告错误。

优化器失败测试改为真实Adam.step执行一个子步骤后抛错，证明其状态已实际改变；同时检查保存次数只有初始有效一次、文件SHA256未变、无污染PUBLISH、最后有效optimizer/counter保留和子进程关闭。

指定回归命令：

```bash
bash /home/kai/.cache/lunar-drl-task-frontier/task-1/test.sh -m pytest -q ros2_ws/src/lunar_drl_exploration/test/test_stage_credit_drain.py ros2_ws/src/lunar_drl_exploration/test/test_schedule.py ros2_ws/src/lunar_drl_exploration/test/test_training_cli.py ros2_ws/src/lunar_drl_exploration/test/test_bounded_training_signals.py ros2_ws/src/lunar_drl_exploration/test/test_training_fork.py
```

结果 `108 passed in 175.36s`。随后新增fractional实际中断/API非法预算断言，并在自审时收紧每次update前的signal flag检查；完整DRL包使用同一wrapper运行整个`ros2_ws/src/lunar_drl_exploration/test`：`739 passed, 2 skipped in 276.33s`，退出0。两skip为显式启用的真实ROS闭环参数。包级启动后补了上述一行signal检查，最终版本使用同一wrapper单独运行`test_stage_credit_drain.py`：`28 passed in 139.71s`（`task6-final.log`），无警告；不把包级结果冒称该最后一行修改的再次全包测试。

所有外部日志位于 `/home/kai/.cache/lunar-drl-task-frontier/task6-*.log`：red、core-green、signal、focused、package、final、imports。现场导入证据仍为专项源码、新Jazzy graph/terrain native与lunar_planning_msgs、既有依赖venv；未混入旧overlay。阶段评测唯一性属Task7，未声称已验证。正式训练/冻结评测、真实ROS动作、Humble、Orin、DDS、传感器、实车NOT_RUN；未合并、推送或PR。UTF-8逐文件读取、Python源码编译和`git diff --check`通过。状态：实现完成，等待控制器独立审查。
