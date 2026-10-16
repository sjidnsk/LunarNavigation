# Wheel 应用公共架构同步验证

原 HEAD 800b4f11b7a5d0b38e20e25d13c2c7cb79041c36，25 条原始改动以
76ad0a0894ca97a6bfbedbafa16b33f0188a0ee2 单父快照保存，再兼容合并公共
d452677ae0c681dc865bac35a4092beab3034f9b。仅 topic 候选，主 agent 审查后落地。

保留 Orin wheel、STATIC_TF、硬件/控制器配置及轮足平台策略；保持应用 baseline
confidence，不下发开发可选拟合器。共享核心逐源码与 TCP 候选一致，公共
CertifySegment、phase/derivative/local_window、精确 stamp、delta/full 保持；
自定义 intrinsic 优先，默认共享 observations，profile 背景宽度显式传递，
native UNKNOWN bounds 与 persistent 同契约。

Humble CPU：不同的 navigation ROS、controller、exploration ROS 独立 fresh 构建；
planner ROS 因根 config/launch/rviz 外部安装输入不同也独立 fresh 构建。
共享核心/native 依赖闭包精确相同，复用 TCP 新鲜构建，并验证真实绑定。
四个应用包实际 ament share 及 56 个安装资源 SHA 均与本候选对应源码一致。
最终 navigation ROS、controller 定向测试通过；接口 68、应用入口配置 29、
native 221 passed/2 skipped；planner config 定向通过。

纯探索旧测试三处把 BASE 已有 clearance 0.4 误写为 0.2；独立提取 BASE
源码/配置 fresh 编译实际复现后，仅修正精确期望和 ReplaceOnce 源值，
非法值与严格断言保持。动作测试等待 replan count 但错误复用旧 succeeded handle；
仅增加新 goal 数及对应 handle 到达条件，生产代码不变，整组通过。
初始混用 TCP 安装的 legged 配置引起另一项 loader 失败，补齐自身安装后消失。

纯探索场景 16 项仍 8 passed/8 failed：BASE 原测试和 fixture 独立 fresh 编译，
原配置独立 ament share 实际复现相同 8 失败；例如写死 key 3081,2250,0，
实际 BASE 几何生成 2881,2250,0。未修改生产阈值或场景断言，不宣称全绿。
完整原 RED、定位反证、构建及测试结果在仓库外 task-4-wheel-humble，
共享源码证明与安装资源证明分别记录，未用包内 Git 树冒充安装资格。

未变 SAC/课程/模型复用 Task3 已审来源。GPU、训练、Orin、实车、真实外部
服务/TCP/真实 bag NOT_RUN；只执行 Humble 本地 CPU 测试。
原 dirty 工作树与活动运行代码不变；无推送、删除、reset/rebase/amend。
 
补充定点同步复核：仅处理两个既有测试的异步事件等待缺口。
FailureMemoryLimitStillCancelsThirdStuckPlanBeforeStableError 等本轮新 goal 及其 handle；
SuppressedCandidatesAreGrayAndActiveCandidateStaysRed 等第三个目标姿态对应的
PLANNING/NAVIGATION_GOAL_SUBMITTED 状态，单次捕获并确认 optional 存在，
再保留原严格计数及红灰断言。未增加 sleep/超时，未修改生产逻辑。
两用例分别10次通过，完整原 RED 与最终输出在仓库外 sync-tests-final.log 等。
该修正为追加提交，原 merge 候选提交与证据继续保留；不重跑无关核心/native套件。
