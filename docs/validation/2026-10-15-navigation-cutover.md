# 自适应导航集成记录

用户已授权修复审核问题，将新导航替换正式强化学习训练的旧导航，并合入三个应用分支；保留旧模式，后续根据测试决定是否回退。不得将本次替换解释为模型、奖励、传感器或训练超参数调整。

## 导航实现

- 沿用应用分支 PrepareCycle / ComputeCycle / ApplyCycle 的异步计算与采纳流程，返回结果经过当前细图复核。
- 受控局部目标端口经过 ROS Instrument 转发，使用原周期 deadline/stop；取消或过期均不能发布 ACTIVE 引用。
- fine 派生立即触发安全复核。自适应导航表示使用独立互斥 callback group/timer 构建；失败重试，过时表示不用于当前细图。
- 完整已知路线优先；MAP_INSUFFICIENT 时不生成可执行的 UNKNOWN 全局路线，而由局部求解器验证到既定目标方向的已知安全执行段，标记 KNOWN_PREFIX_FOUND。到边界进入 WAITING_FOR_MAP，新证据后继续；最终任务目标不变，不选择探索收益或未知岔路。
- 地图等待有独立期限，位姿抖动或引导图变化可以唤醒计算，但不能在细图证据不变时延长期限。安全执行段成功后重置期限。
- 全局直达与端点检查使用共享闭合 supercover，区分已知 BLOCKED 和 UNKNOWN/范围外接触。
- legacy_coarse 保留；它回退全局后端，完整旧导航需使用保留的旧安装。

## 独立审查与验证

新端点接触与位姿抖动两个 Important 审查意见均先完成失败回归，再修复。新增实际 ROS 包装器转发、目标构造内部取消/过期，以及表示构建阻塞期间路径失效测试。核心按包串行 CTest，测试进程使用既有 64 MiB stack 限制；不将缺失该限制导致的 snapshot 测试栈溢出归为算法失败。初次 ROS 配置安装回归发现缺少 *_orin.yaml 安装规则，已补齐。

完整命令、RED/GREEN 输出、consumer 合并前的未提交文件备份、源文件 SHA、冻结场景结果、运行时切换与恢复证据保存在项目顶层 `analysis-artifacts/navigation-cutover-20261015/`，不提交构建、模型或运行日志。

## 保留的限制

- Minor：目标所在 FREE 分量已被已知障碍封闭，但起点分量接触未知时，仍可能保守报告 MAP_INSUFFICIENT。该分类不会把未知当成可执行空间，也不等价于探索完成；闭合阻断证明的优化留待后续。
- ROS controlled-target 的内部取消和期限已增加端到端覆盖；真实控制器停止距离、实车 DDS、Orin 性能和实车均须分别验证，当前不能以本机 Humble 结果替代。

## 审查未判断事项的处理

- consumer 合并后的 terrain、平台与 GetPolicyMap 通过逐分支保留差异、构建和包测试核验。
- 模型、optimizer、完整回放、ClearML 与恢复连续性通过切换前后 checkpoint 字节校验和加载证据核验。
- Orin 与实车本次不部署，标为 NOT_RUN。
