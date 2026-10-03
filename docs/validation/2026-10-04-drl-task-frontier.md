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
