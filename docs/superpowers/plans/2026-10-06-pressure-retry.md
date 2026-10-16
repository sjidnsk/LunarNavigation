# 200m 洞穴2 A2 独立串行重测

用户在本轮明确授权重新测试发生导航 watchdog 的回合。目的：保留正式四并发评测的异常证据，用同模型、同图、同预算的串行重测辅助区分可重复问题和并发/时序影响。

## 约束

- 原40回合继续，不增加第五个并发 native，不中断在途回合。
- 仅当原批次40个有效身份终态已落盘、run-state 为 finished/finished_with_errors、active 为空且原 owner 的准确 PID/start 已退出，才启动一次独立重测。
- 保持 HEAD `7e6fd899a78031902c2f47d6449b28f652e464c6`、Actor、配置、导航、奖励、预算、pressure/evaluate/contraction/compare、已登记 parallel/resource 源码不变，冻结期延长到独立重测结束。
- case=`cave200-r2`，mode=`remote`，family=`cave`，extent=200m，seed=`2026100633`，budget=1536，remote 上限32。
- 原 `cave200-r2/remote` 的 infrastructure error 保留。新产物仅写 `retests/cave200-r2-A2-retry-01/`，不覆盖 partial，不写原 run-state/comparison/40回合统计，不自动将重测补为原配对。
- 独立脚本锁与 immutable request 登记 frozen/source/error SHA；失败或主批次中断记录 attention/blocked，不隐式恢复或重复评测。
- 重测使用原 Jazzy wrapper/cache 和原 evaluate；native 子进程绑定新重测 owner 生命周期，关闭只作用于精确拥有进程。
- 120s超时保持原样。重测成功只能说明单次未复现，不能单凭一次差异证明并发导致；初始化输入、宿主负载与随机时序仍需核对。

## 任务

- [x] 分析异常，确认成功前缀37次，失败尝试为第38次；分类为 infrastructure，不纳入成功率分母。
- [x] 子代理实现 pressure_retry.py 与行为测试。
- [x] 独立审阅；只读预检原错误/冻结身份/候选命令。
- [x] 启动一次后台串行重测排队，核验 queued 与原四并发不受影响。
- [x] 更新加尔跟踪，正式40回合后先完成重测，再进行全批最终验收与独立结果审阅。

当前结论边界：单动作120秒墙钟 watchdog 覆盖接受、规划、执行与结果传输；失败动作没有保存完整 tracking/odom/future，因此尚未唯一定位内部阻塞阶段。可用内存和DDS域隔离正常，不能将此次错误直接归因于四并发。

## 启动验收

- 重测 helper SHA：`f4d9abdc29b2cf3697027594dbe76dbdda97e906ff3360630ce2159b255c6c51`。20项 retry 检查由主代理复跑通过；子代理完整 region suite 为184 passed、1 skipped。独立审阅通过，HEAD与冻结核心源码不变。
- 后台队列 PID `808158`、start_ticks `24460565`，主启动终端退出后 PPID 为宿主 `1911`，状态 `queued` 持续刷新，尚无 evaluation 目录。正式批次仍 `running`，启动时27/40终态、4个活动回合。
- 新产物：`retests/cave200-r2-A2-retry-01/{request.json,retry-state.json,launch.json}`；控制器日志为 `retests/cave200-r2-A2-retry-01-controller.log`。原错误摘要SHA保持 `8e85981a9efdb38f7167f84bf10f9a4fcf95f52878771895ccb6ee000886ff96`。
- 加尔的现有 `a0-a2` 跟踪已更新并核验仍ACTIVE；正式40回合结束时若重测仍排队/运行，不结束跟踪。全批结果验收和重测实际结果仍待后台完成。
