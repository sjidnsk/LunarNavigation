# NAV1CM 末端平滑朝向与职责边界计划

任务 NAV-1CM-AUTO-20261029/TERMINAL-APPROACH；owner integration/pure-planner-orin；复用 hk/nav-1cm-feasibility-20261026，基线a63afae。
用户已确认 nav1cm + 局部MPCC + 标准Path MPPI：接近终点时平滑调整最终姿态，到点后剩余yaw原地完成；全局/局部/跟踪器不得混责。本次确认作为行为设计依据。

职责：全局路线、地图、净空与标准Path跟踪器算法不改。局部MPCC对moving候选在末段以剩余路线弧长平滑混合行进车体yaw与最终yaw；目标是软代价，仍受曲率/加速度和独立整车扫掠认证。行进对准spin不承载最终yaw。协调器继续按实测XY决定最终姿态阶段，不以弧长或跟踪阶段代替到达。

- [x] 在tests/navigation/one_cm/test_terminal_approach.py建立spin污染、平滑moving目标、倒车/跨pi、0.3rad模式重置、截窗边界回归，先观察失败。
- [x] 仅修改scripts/navigation/one_cm/nav1cm/local_mpcc.py：纯yaw目标生成与受限末段继续行进条件；参数传入实际mode；保留原制动、曲率、净空与候选认证边界。
- [x] 运行纯测试、既有MPCC/terminal/coordinator/标准Path测试。旧terminal参数测试按新平滑语义更新，保留倒车最终yaw不加pi的断言。
- [x] 在新的隔离台式目录运行真实acados及标准Path MPPI短组件验证；不得覆盖旧安装/失败证据，不启动ROS/Actor/训练，不冒充DDS/实车验收。
- [x] 独立审查、UTF8与明确路径diff检查、本地任务切片提交；不自动合并/push或切换运行安装。

混合长度采用局部求解时域可行驶长度N*dt*vmax，不新增与全局职责重叠的终点路线重写。最短角差，倒车仅行进方向加pi。mode门槛只对末段且车体方向位于行进至混合方向的角区间、正向推进投影为正时放行；其他状态沿用原对准或制动。最终到达判据和terminal_candidate保持。

验证注意：大角度/短距离受曲率限制，不保证行进中完成全部yaw；位置到达后必须完成剩余yaw并停稳。单调进度到终点但横向位置未达的参考退化另列风险，禁止误判到达。
