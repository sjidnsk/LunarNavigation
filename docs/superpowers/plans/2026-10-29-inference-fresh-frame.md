# 恢复导航候选：完成后新观测门禁

继续原EXECUTION-CONTRACT任务；确认没有必须先清理的源码来源缺口后恢复导航/DRL候选。基线7432828，owner dev/drl-exploration（推理生命周期），复用hk/drl-integration-20261029及原D盘工作树。单线程实现，不新增状态对象/消息字段。

- [x] 以实际InferenceRuntime类的纯生命周期回归先RED：新响应但same stamp不允许二次决策；stamp前进才允许；取消在无新地图时仍独立停稳完成。
- [x] 使用已有poll_snapshot(minimum_map_stamp_ns)及现有_stamp持久表达严格新帧下界；不改规划/跟踪/容差/代价/训练。
- [x] 更新development87清单中的runtime.py身份，并补齐已提交navigation_backend.py的继承清单遗漏，formal79及正式配置/模型原字节不动。相关本地/真实Humble推理生命周期回归后再固定Actor隔离验证。
- [x] 停滞问题先采现有候选诊断；实际stage仍作为下轮待办，不以放宽容差/超时或增加实体解决；本切片独立记录，不用新鲜度修复宣称停滞已修复。

判据：结果完成后下一Actor决策使用严格前进的观测stamp；新鲜度等待不能阻塞取消/暂停的真实停稳终结。沿用已有协议和字段，部署边界与历史失败分别保留。