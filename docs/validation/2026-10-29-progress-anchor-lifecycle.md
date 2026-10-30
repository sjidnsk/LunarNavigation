# 进展锚点生命周期修复：第一切片

2026-10-29，基线83426f8，hk/drl-integration-20261029。用户授权先修锚点再改采样。

重绑之前将旧锚点到实际位姿的有向投影结转到原有MPPI标量（原_best_progress未参与其计算，现明确命名_translation_credit_m）；新锚点仍是真实位姿。负向位移不截断，原3cm推进/0.01rad角改善判据达成时一并清credit和停滞时钟。参考刷新自身不清时钟、不修改已发布历史。clear、新session、拐点完成的同类复位也清credit。未新增状态实体或接口，阶段屏障、到达/停稳阈值、超时、代价和采样保持。

定向六项先RED，修复后与阶段/终端测试共82通过；十组纯控制器回归359通过。Windows缺geometry_msgs，reference/incremental_path两组收集失败原样保留，待实际Humble包测，不算通过。

r16已记录217条接管快照的13次物理锚点变化，按原真实方向逐段转移、最后一次判据结算得到0.038336136m；原时钟14.843854296s，修正后的最终判据清至0。此重放仅验证记录事件的位移转移与最终判据，不模拟缺失周期，不声称新实时闭环已通过。结果execution-tests/progress-lifecycle-r16-replay.json，源码脚本同级replay-progress-lifecycle.py。

当前仅第一切片本地验证完成待隔离Humble联合构建/包测；后续采样切片尚未实施。完整闭环仍未通过，不晋级owner/main/apps，不训练、不push/PR。阶段尾部语义另行讨论，未改动。

后续联合验证：Humble受影响控制器包524项通过，r17/r18六/十目标严格闭环均PASS；最终证据及部署边界见2026-10-29-progress-reference-sampling.md。
