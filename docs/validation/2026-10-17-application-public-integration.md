# Legged 应用公共架构同步验证

原 BASE 2d6f6b9913bf6e0d4cb8ec418fae2d0d3e79b738 的31条改动以
6aa006e68280cb756671bed88a8ebf90bee55fcb 单父快照逐字节保全，再兼容合并
d452677ae0c681dc865bac35a4092beab3034f9b。仅topic候选，root审查后实际落地。

保留足式能力/TF/硬件/控制器/默认platform与背景宽度，以及既有baseline
confidence、足式patch支撑、最大terrain路径cost和增量影响半径。
保持main CertifySegment、phase/derivative/local_window、精确stamp、
center_known=false UNKNOWN、delta/full语义；不引入开发可选拟合器。
native完整UNKNOWN bounds与persistent对齐；stationary/support/exporter
显式使用所属profile背景；自定义intrinsic优先，默认入口共享observations。

Humble CPU：独立fresh构建6包（core、navigation ROS、DRL native、
planner ROS、controller、exploration ROS），耗时6分21秒。
实际native/core/ROS库路径和SHA已校验；4包自身ament share的58项安装资源
逐SHA等于本候选根配置/launch/rviz或包内资源，未混用TCP/wheel配置。

core 27组407项、navigation ROS 14组128项、controller 10组225项全部通过；
native 221 passed/2 skipped，两个实际producer另以普通解释器通过；
planner config定向通过；接口68、应用配置与入口51项通过；错误参数入口退出1。
纯探索初次251项250通过：场景16项全部通过，1项测试跨topic同步失败。
该测试等待diagnostics后直接解引用尚未到达的status optional；
诊断版10次中3次明确has_value=false，保留原RED。
改为等待本轮第三个目标姿态对应的PLANNING/NAVIGATION_GOAL_SUBMITTED状态，
单次捕获并确认optional，再保留原计数与红灰断言。
同一fake旧handle提前Finish缺口亦按新goal数量与对应handle到达守卫。
三应用同一两个用例均定点各10次通过，不改生产逻辑、不加sleep/扩大超时。
原配置闭合集合遗漏3个BASE已有文件，仅补明确名字，不通配泛化。

未变SAC/课程/模型复用Task3已审来源；core与TCP/wheel不同，因此未复用其
native资格。2个native ROS opt-in未执行；Orin/实车/GPU/训练/真实TCP/
外部服务/真实bag/真实全局producer均NOT_RUN，不代表实机性能或部署验收。
构建/测试原输出及二进制证据在仓库外task-4-legged-humble；完整说明见stage
task-4-report.md。原dirty树、原分支与活动运行代码不变，无push/删除/reset/rebase/amend。
