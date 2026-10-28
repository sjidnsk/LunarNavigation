# ISAAC-VIPER-MIDTERM-20261028 / showcase-20261029

- 续接原工程任务；用户明确允许合入并放入e572ed3/1a60611三区展示，允许场景简化，传感分辨率/频率与旧平台一致。
- owner integration/pure-planner-orin；专题hk/isaac-viper-showcase-20261029；工作树D:/Codexworktrees/isaac-viper-showcase/Lunar-Navigation；初始基线1a60611ad62b8c53c9a589f50e1bde2bf45b7e14。
- 原工程accf75b由e15c667按19文件差量晋级，未合入34个继承TCP提交；原工程登记与完整历史仍在原专题及D盘coordination/viper-original-task-registration.md，台式共享入口/mnt/data/WS/lunar-navigation/docs/任务登记.md。
- 主线中央登记有其他任务未提交内容，本阶段用本文件记录本子阶段并更新既有台式同任务入口，不覆盖/暂存它。
- 执行者Codex/root；调查只读并行，源码subagent顺序单写入；本地Windows，台式SDK/Humble独立验证。
- 状态：进行中；原2次闭环与部分严格停车FAIL保留；新的三区算法效果/Actor仍NOT_RUN。
- 外部产物D:/CodexDownloads/isaac-viper-showcase-20261029；台式对应analysis-artifacts/isaac-viper-showcase-20261029。
- 最终处置待实际验证和授权合入；不推送/PR/删除，分支/worktree/所有源和新旧产物保留。

## Task 2 本地实现交接

- 执行者 Codex/implement_light_showcase，Windows 单一源码写入者；基线 `9db7ed332986d8f5a8233e341f15e5c237d13ba5`，沿用本任务专题/工作树与 owner。
- builder、统一 1 m 几何、80 tri 岩石代理、实际三角落位、只含相机的三区层、UI/文件视角通道已实现；原 profile 与 Task 1 分段计时保留。
- 本地局部测试和语法验证已执行，实际 PXR/GPU/ROS、误差数值、RTF、控制停车、GUI 由 root 接续；旧失败/旧源保留。
- 处置：代码待本任务显式提交后交 root 实际验证，未合入；不推送、PR、删除或回收工作树。准确提交及命令见 D 盘 coordination/task-2-report.md。
