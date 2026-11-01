# RViz 导航探索操作面板实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans. Steps use checkbox syntax for tracking.

**Goal:** 导航常驻，可随时启停前沿探索，并通过 RViz 选区、选位姿、暂停与恢复。
**Architecture:** 常驻空闲探索节点复用导航 Action。RViz 面板只发任务/目标和显示状态；不发布 cmd_vel，不重做算法。
**Tech Stack:** ROS2 Humble、RViz2/Qt5/C++、现有 Python launch。
**Spec:** docs/superpowers/specs/2026-11-01-rviz-navigation-exploration-panel.md

## Global Constraints

只写本独立工作树与远端P4；P3/Env_X仅可读取使用。控制域10，既有.2m/s/.1rad/s及超时不变。不增加任务实体/权限实体/等待门禁；共享基础地图暂停。每批独立测试与复审。设计文档不上传调试机。

## Review Focus

- 手动目标不能被探索重新抢回。
- 暂停迟到回执不能取消新手动目标。
- 恢复探索时仍有手动任务须明确确认替换。
- 重复点击、空区域与退化多边形不能发送错误任务。
- 关闭界面不擅自结束车辆任务，界面不得发布速度。

## Task 0：静态TF修复（优先、单独提交）

- [ ] 多条latched TF、探索晚加入回归先RED。
- [ ] 只修接收历史深度，与桥接一致，GREEN及现有增量探索回归。
- [ ] 独立复审、提交；隔离部署及状态核对。

## Task 1：运行中启用探索

Files: scripts/simulation/p4_tcp_runtime.py；相关既有Python launch测试。
- [ ] 测试导航默认同时加载frontier但节点保持IDLE，不发START。
- [ ] 修改TCP入口默认策略为frontier（通用launch默认none可保留），显式none仍可用。
- [ ] 测试、独立复审、显式路径提交。

## Task 2：RViz面板与区域选择

Files: 新包 ros2_ws/src/lunar_navigation_rviz/（CMake/package/plugin描述、include/src、test）；scripts/simulation/p4_navigation.rviz。
- [ ] 读取实际PureExplorationTask/Status与导航目标入口约定，确定接口。
- [ ] 小型Qt面板显示状态/覆盖率/目标/具体原因，复用现有显示；使用RViz工具/点击点收集map坐标多边形，可撤销/清空/闭合。
- [ ] START/PAUSE/RESUME/CANCEL复用消息；手动目标使用已有导航goal话题/Action，不发速度。
- [ ] 实现可测试的操作逻辑，验证空区域、重复操作、手动接管/恢复确认和迟到回执；不引入新后台任务实体。
- [ ] Humble构建与测试，独立复审，显式路径提交。

## Task 3：目标交接与统一部署

Files: 上述包/必要现有目标入口的最小改动；P4运行脚本；本地验证文档。
- [ ] 针对实际现有Action替换行为验证暂停探索→手动目标、取消回执→新目标、恢复探索。
- [ ] 只在现有语义无法满足时修正交接，不用固定等待秒数。
- [ ] 合并各批运行安装并启动新RViz；仅重启本会话拥有的P4进程，保留P3与Env_X。
- [ ] 使用隔离DDS/fake导航验证界面命令，再进行P4真实输入显示检查；不未经请求另发无限车辆目标。
- [ ] 更新任务登记/验收证据，说明NOT_RUN及待合入，不自动merge/push。
