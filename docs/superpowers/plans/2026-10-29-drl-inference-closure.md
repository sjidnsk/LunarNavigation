# DRL 导航推理完整反馈闭环计划

任务GIT-PRESERVE-20261029/DRL-INFERENCE-CLOSURE，owner dev/drl-exploration；复用hk/drl-integration-20261029，基线5c21ef1。用户明确要求处理完整DRL→导航→跟踪→反馈闭环，不是训练。

冻结输入复用旧Task4失败窗口的Actor-only block-01（SHA6951491300d144cb9e170859804707eb3613bd0fafd956ae205549eab0120f7d）、原config与0.2m平台文件，不更换模型/目标/奖励。公共MPCC使用eb10ff2对应新source-r2，不把旧45s失败回填为新源结果。12m合成场景、真实观测传感器与有限加速度运动反馈保持原定义，正常推理窗上限90s，两次真实目标到达及下一次策略决策；退出/复位阶段另有显式短时上限。

- [ ] 新增纯验收器与守卫回归RED/GREEN：去重serial+UUID、拒绝取消冒充到达、独立实测误差/停稳、新观测后的下一Actor决策、冻结与退出门槛。
- [ ] 新增独立验证入口，复用既有ActorPolicy/InferenceRuntime/传感器/原生map/task/标准PathMPPI，不先改产品行为。任务通过真实私有Topic START/CANCEL，结果在adapter._finish时捕获，避免同tick下一goal清空身份。
- [ ] 禁止training/sac/replay/collector/ros_env模块；所有Python子入口继承守卫，Optimizer/Critic构造禁止。正常Actor依赖batch/torch模块可导入；仅CPU+inference_mode，不开learner或训练CLI。
- [ ] 先检查ROS域92无其他节点/生产命令，使用私有/lunar_inference/...端点，不碰现有Isaac/训练进程；独立source/build/install/log/cache、只读现有Actor/config/profile。
- [ ] 预声明冻结hash与源码/实际安装身份；执行一轮有界完整推理反馈窗口，覆盖>=2不同目标GOAL_REACHED、>=3Actor决策、实测位置/朝向/停稳、观测推进、当前UUID取消、任务START复位、再次取消、正常退出。
- [ ] 有失败先定位并保留原窗，不放宽目标/物理约束、不重训或盲目重试；必要修复按owner与TDD执行。
- [ ] 独立审查，完整raw与最终grader、权重/config前后hash、所有子进程退出证据保存；仅在满足门槛时声明隔离合成推理闭环通过。真实P3/Orin/实车/策略效果仍NOT_RUN。

本任务不自动push、合并owner或删除资源。对原training ledger不读取、不写入、不声称其前后字节证明。两子任务独立写纯grader与只读验收审计，主线程唯一修改运行验证入口；无共享文件并发写。

## 2026-10-29执行状态

验证器与禁止训练守卫已实现，相关纯测试61 passed。r1/r2失败已定位并仅修复验证环境；r3有界真实ROS推理实际运动，但首目标前0.241m停滞，完整闭环FAIL。精确冻结位姿在空白图上真实acados三组status0、NO_PROGRESS，尚需在线候选证据。详细身份/退出/边界见docs/drl/2026-10-29-inference-closure-validation.md。末段行为修改待用户讨论，owner/main/apps未晋级。
