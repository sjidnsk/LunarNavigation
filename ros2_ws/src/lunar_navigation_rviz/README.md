# RViz 导航与探索面板

Humble / Qt5 RViz 插件。面板仅发布现有 `PureExplorationTask`，不发布速度，不创建额外探索节点，也不自行运行 ROS executor。

1. 加载 `scripts/simulation/p4_navigation.rviz`，确认 Fixed Frame 为 `map`。
2. 点击面板“选取区域”，在地图地面逐点点击；“撤销一点”可回退，“清空区域”可重选。
3. 点击“完成区域”闭合并检查面积，再点击“开始探索”。开始和恢复均需确认会替换当前导航目标。
4. 手动导航仍使用 RViz `2D Goal Pose`，位置与朝向发往 `/goal_pose`。探索与手动目标交接由既有导航/探索节点处理。
5. 暂停、恢复、结束使用现有命令。关闭面板不会发布 CANCEL；仍在执行的任务继续由后端管理。

区域工具只在 `map` 固定坐标系下将视线投到 z=0 平面，使用同 GUI 线程 Qt 信号同步把顶点交给面板。先点击面板“选取区域”；快捷键 `B` 仅用于切换到区域工具，不会自动新建或清空区域。面板选区轮廓发布 `/lunar_rviz/boundary`；该轮廓表示编辑中的区域，实际任务轮廓仍由探索节点 `/p4_p3_debug/exploration/task_boundary` 发布。

Task Topic / Status Topic 在 RViz 配置中持久化；默认接口为 `/Car/T4/exploration/task` 与 `/Car/T4/exploration/status`，本项目 `p4_navigation.rviz` 明确使用运行入口覆盖后的 `/p4_p3_debug/exploration/task` 与 `/p4_p3_debug/exploration/status`。状态、覆盖率、目标、等待/失败原因直接来自所配置的状态话题。状态尚未接收时不能开始；按钮发送后等待后端回执，不把按钮点击当成成功。当前导航没有供该面板消费的独立状态源，因此导航路径/目标可视化不能作为到达证明。

构建测试（先 source ROS Humble 和包含接口包的 overlay）：

```bash
colcon build --packages-up-to lunar_navigation_rviz
QT_QPA_PLATFORM=offscreen colcon test --packages-select lunar_navigation_rviz --event-handlers console_direct+
colcon test-result --verbose
```

`test_boundary_selection` 覆盖坐标系、非有限点、退化、自交、重复点和编辑失效行为；`test_exploration_panel` 覆盖未连接/未选区时不能发送命令。`test_panel_dds` 使用真实 ROS 消息和 test-only 图形宿主，覆盖选区末点同步、命令载荷、确认、连续回执、防重复与关闭不取消。DDS 命令、手动导航交接与 RViz 真机交互须另外验证；离屏测试不代表车辆验收。
