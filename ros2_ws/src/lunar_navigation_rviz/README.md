# RViz 导航与探索面板

Humble / Qt5 RViz 插件。面板仅发布现有 `PureExplorationTask`，不发布速度，不创建额外探索节点，也不自行运行 ROS executor。

`config/p4_topics.yaml` 是正式 P4 话题命名来源。`scripts/simulation/p4_navigation.rviz` 仅保存样式与 `Topic Key` 引用，必须由运行入口调用 `render_rviz_config(template, contract)` 填入真实话题后交给 RViz，不能直接加载源模板。源模板中的导航工具话题为空，渲染器会按 `goal_topic` 填入统一入口。全局参考话题按契约中的全局路线话题加 `_reference` 派生。

1. 使用正式运行入口生成并加载 RViz 配置，确认 Fixed Frame 为 `map`。
2. 点击面板“选取区域”，在地图地面逐点点击；“撤销一点”可回退，“清空区域”会清除草稿并通过现有 CANCEL 结束探索任务，再重新选区。
3. 点击“完成区域”闭合并检查面积，再点击“开始探索”。开始和恢复均需确认会替换当前导航目标。
4. 手动导航仍使用 RViz `2D Goal Pose`，点选位置并拖拽朝向，目标发往契约定义的统一导航目标入口。探索与手动目标交接由既有导航/探索节点处理。
5. 暂停、恢复、结束使用现有命令。关闭面板不会发布 CANCEL；仍在执行的任务继续由后端管理。

区域工具只在 `map` 固定坐标系下将视线投到 z=0 平面，使用同 GUI 线程 Qt 信号同步把顶点交给面板。先点击面板“选取区域”；快捷键 `B` 仅用于切换到区域工具，不会自动新建或清空区域。草图区轮廓使用契约的 `boundary_draft`；实际任务轮廓由探索节点发布，两者用途独立。

面板的 `Task Topic`、`Status Topic`、`Boundary Topic` 会在渲染后的 RViz 配置中持久化，正式入口均由 YAML 契约覆盖。独立添加插件时的 fallback 为 `/Car/T4/exploration/task`、`/Car/T4/exploration/status`、`/Car/T4/exploration/boundary_draft`，这些默认值不替代正式入口契约。状态、覆盖率、目标、等待/失败原因直接来自配置的状态话题；按钮发送后等待后端回执，不把点击当成成功。导航路径/目标可视化不能作为到达证明。

构建测试（先 source ROS Humble 和包含接口包的 overlay）：

```bash
colcon build --packages-up-to lunar_navigation_rviz
QT_QPA_PLATFORM=offscreen colcon test --packages-select lunar_navigation_rviz --event-handlers console_direct+
colcon test-result --verbose
```

`test_boundary_selection` 覆盖坐标系、非有限点、退化、自交、重复点和编辑失效行为；`test_exploration_panel` 覆盖未连接/未选区时不能发送命令。`test_panel_dds` 使用真实 ROS 消息和 test-only 图形宿主，覆盖选区末点同步、命令载荷、确认、连续回执、防重复、话题绑定/重载与关闭不取消。DDS 命令、手动导航交接与 RViz 真机交互须另外验证；离屏测试不代表车辆验收。


“清空区域”也适用于已完成、错误或空闲状态：按钮发送既有 CANCEL，后端在三个探索专用可视化话题 `task_boundary`、`task_map_markers`、`frontiers` 保留明确删除消息，清除任务边界、任务栅格、前沿与候选位姿；重订阅不会回放旧任务图形。它不删除基础地图，不发送车辆速度，不取消用户独立提交的手动导航目标。正在执行的探索目标仍由现有 NavigationClient 按所属 Action 句柄取消。暂停/恢复保持区域及其显示，关闭面板不会自动清空任务。


P4 模板默认关闭 `Exploration task map` 和重复的 `Current exploration goal` Pose 显示。后者的 Pose 协议没有删除操作，因此选中探索目标由现有 `Exploration frontiers` 中的红色候选 Marker 表达，随任务结束一并清理；`current_goal_topic` 接口保留。导航目标 Marker 与导航路径保持原职责。
