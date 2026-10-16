> 历史分支记录：`test/drl-loop-learning` @ `d5de79c0b157`。本文运行状态、接口和实验结论是当时快照；当前开发以 [统一入口](../DRL开发入口.md) 为准。

# 学习式解环专项：设计前置检查

日期：2026-09-23。仅文档与现有代码复查；不是算法实现验收。

## 分支与保护范围

- 设计分支：`docs/drl-loop-learning-design`，从 `dev/drl-exploration@fe9cab83` 派生。
- 设计工作树：`/home/kai/WS/lunar-navigation/lunar-runtime/.worktrees/drl-loop-learning-design`。
- 集成主线检查时为 `6868236f`，工作区干净；不在其上写入。
- DRL 开发工作树仍有原来的 `config/drl_exploration.yaml`、`ros2_ws/src/lunar_drl_exploration/test/test_training_cli.py` 两处修改，本轮不修改、不暂存。
- 原 `drl-coverage99-v1/resume.pt` 大小为 2535522730 bytes，mtime 为 `2026-09-22 22:42:33.894333338 +0800`。本轮不加载或写回完整检查点。
- 进程核查未发现 DRL 训练、评估或上一轮诊断进程；本轮不启动训练或 ROS 导航闭环。

## 当前源码测试

从设计工作树根目录，在清除旧 overlay 后仅加载 Jazzy 及既有隔离安装；Python 包搜索路径优先指向本工作树源码。只读复用现有依赖，未安装或修改依赖。

```bash
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u ROS_PACKAGE_PATH -u PYTHONPATH -u LD_LIBRARY_PATH \
  -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION \
  bash --noprofile --norc -c '
    source /opt/ros/jazzy/setup.bash
    source /home/kai/.cache/lunar-drl-redesign/jazzy/install/local_setup.bash
    export PYTHONPATH="$PWD/ros2_ws/src/lunar_drl_exploration:$PYTHONPATH"
    export PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
    /home/kai/.cache/lunar-drl-training/venv/bin/python -m pytest \
      ros2_ws/src/lunar_drl_exploration/test/test_sac.py \
      ros2_ws/src/lunar_drl_exploration/test/test_bounded_actor.py \
      -q -p no:cacheprovider
  '
```

结果：**24 passed、1 failed，10.56 秒，退出码 1**。

失败项：`test_actor_packed_and_per_observation_paths_bound_only_actor_with_same_weights`，`test_bounded_actor.py:74`。同权重、放大输出层后，旧 C10 概率 argmax 为 17，原分数概率 argmax 为 23；再次复现历史的最高分并列问题。

本轮没有修复源码、放宽断言或重跑到通过后隐藏失败。这一失败是[专项设计](../superpowers/specs/2026-09-23-drl-loop-learning-design.md)要求数值就绪门槛的直接依据。它不解释 C0 正式模型的循环，也不证明新增候选已有效。

## 本轮交付与未执行边界

- 已完成：分支/工作区/进程核查、阅读既有实验与源码、现有测试复跑、独立专项设计、文档自查。
- 本轮仅编辑设计、此检查记录和开发入口中的文档索引。
- 新有界分数实现、冻结 Q 新对照、经验统计与 Q 校准新工具：`NOT_RUN`。
- 新方案训练、3000 条筛查、多训练种子评价、120/250 m 评价：`NOT_RUN`。
- 本轮新构建、完整包测试及原生导航闭环：`NOT_RUN`。
- Humble、Orin、目标 DDS、实车：`NOT_RUN`。

待用户审阅书面设计后编写实施计划；本次不合并、不推送、不创建 PR，也不恢复原长训。
