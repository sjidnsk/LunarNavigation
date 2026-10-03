# 任务内外前沿收益：任务1基础工程验证

日期：2026-10-04。用户已批准设计、实施计划及子代理执行。本记录仅确认双熵依赖和固定零位置辅助权重的基础工程；新的前沿特征、奖励、Critic预测和训练尚未实现或启动。

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
