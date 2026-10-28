# 轮式 1 cm 导航三阶段 Implementation Plan

后续用户授权的时间门禁修订与计算优化执行 [软周期计划](2026-10-26-navigation-soft-budget.md)。本文件保留初轮实施及失败证据，原超期制动条款已由软周期设计替代。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 独立验证 5 m × 5 m、1 cm 局部 MPCC 和大范围稀疏任意角全局规划的正确性及性能。

**Architecture:** 地图、参考匹配、局部求解、独立认证、全局搜索及编排以不可变对象交换数据。
各组件仅保存自身状态；benchmark 负责输入生成与证据，不承担算法决策。所有代码先隔离于验证包。

**Tech Stack:** Python 3.10、NumPy / SciPy、CasADi / acados SQP-RTI，必要时独立 C++ 加速核心；ROS 2 Humble / Orin 仅作原生算法运行环境。

**Spec:** `docs/superpowers/specs/2026-10-26-navigation-1cm-feasibility-design.md`

## Global Constraints

- 轮式车体 1.182 × 0.818 m；v ∈ [-0.2, 0.2] m/s，|w| ≤ 0.1 rad/s；|a| ≤ 0.5 m/s²，|alpha| ≤ 0.5 rad/s²；行进 |w| ≤ |v| × 1.0 m⁻¹，静止允许自转。
- 局部窗口以车体中心为中心，5 m × 5 m、500 × 500 格，分辨率 0.01 m；起点完整车体覆盖区为已知自由。
- 不接 P3，不加入起步盲区补全，不发布车辆控制，不改既有生产算法、配置或节点。
- 全局允许未知；局部 allow_unknown 可配置；UNKNOWN 原始状态必须保留，已知障碍目标必须拒绝。
- 地图 epoch、路线 version 和轨迹来源贯穿接口。地图换版使旧认证失效；路线换版重新投影进度。
- 暂定完整局部周期 50 ms、全局周期 1 s。原始样本及 P50/P95/P99/max、超时率、RSS 和失败原因必须保留；未达标不得报通过。
- 本任务不提交、不合并、不推送、不删除；只写独立任务工作树及用户授权的 /home/yanfa/P4_veri。子任务实现串行，主 agent 负责整合和审查。

## Review Focus

- 路线交叉、U 形回折和换版时的进度跳跃：Task 1 用连续弧长及首次出窗测试限制误匹配。
- 车体角点、两节点之间薄障碍及制动尾段：Task 1 用独立整车扫掠验证，不能仅测轨迹中心。
- 求解状态成功但数值非有限或不满足动力学：Task 1 / 2 独立拒绝并保留求解状态。
- 大范围的厘米窄门、未知/图外目标和预算耗尽：Task 3 保留高精度拓扑，区分失败原因。
- 地图更新或全局换版与旧轨迹并存：Task 4 先失效旧认证，重新匹配并闭环验证。

### Task 1: 数据契约、地图、参考匹配和独立认证

**Files:** Create `scripts/navigation/one_cm/nav1cm/{__init__,contracts,maps,reference,dynamics,certification}.py`; test `tests/navigation/one_cm/test_contracts_maps_reference.py`, `test_certification.py`。

**Interfaces:** 定义 frozen `WheelLimits`（上述配置）、`VehicleState(x,y,yaw,v,w)`、整数厘米 `CellRect(x0,y0,x1,y1)`、`WorldSnapshot(epoch,blocked,known_free,resolution_m=0.01)`、`GridSnapshot(epoch,origin_xy,resolution_m,cells,allow_unknown)`、`Route(version,map_epoch,points,goal)`、`ProgressToken(route_version,s)`、`LocalReference(route_version,map_epoch,points,s,progress_s)`、`Trajectory(map_epoch,route_version,times,states,controls,mode)` 及明确枚举原因。数值数组复制为不可改写缓冲区，单位米/秒/弧度。states 列为 x,y,yaw,v,w,s；controls 列为 a,alpha,nu。

**Produces:** `extract_window(world,state,allow_unknown)->GridSnapshot`，`build_distance_field(grid)->DistanceField`，`extract_reference(route,state,grid,previous=None)->(LocalReference,ProgressToken)`，`step(state_vector,control,dt)->ndarray`，`certify(trajectory,grid,limits,current_route_version)->Certification`。

- [x] 写并运行失败测试：快照不会别名可变源；500×500 原始未知不被策略改写；图外为未知；角点碰撞、穿越 1 cm 薄墙、制动尾段、超限/NaN/坏动力学/旧 epoch 均拒绝。
- [x] 实现稀疏原生厘米矩形地图和保守窗口提取；EDT 净空下界及梯度。全局稀疏矩形是首轮可压缩输入表示，不宣称支持任意 1 km 全密集栅格。
- [x] 实现弧长投影：同版限制于旧进度邻域、仅前向连续参考至首次出窗；新版重新投影。测试交叉和回折不能跳段。
- [x] 动力学采用中点离散 unicycle，v/w 状态，a/alpha 控制，nu 推进参考弧长。认证核验完整状态转移、行进/自转模式、速度及加速度，执行全车矩形与格子面积碰撞检查，并补制动尾段；用运动上界进行连续扫掠保守包络。
- [x] 运行 `python -m pytest tests/navigation/one_cm/test_contracts_maps_reference.py tests/navigation/one_cm/test_certification.py -q`，明确记录 RED/GREEN；读 UTF-8 与 `git diff --check`，不得提交。

### Task 2: 局部 MPCC 与独立固定参考验证（阶段一）

**Files:** Create `scripts/navigation/one_cm/nav1cm/{local_mpcc,seeds}.py`，`scripts/navigation/one_cm/benchmark_local.py`；test `tests/navigation/one_cm/test_local_mpcc.py`；task-scoped acados 环境脚本放 `scripts/navigation/one_cm/setup_solver.sh`。

**Consumes:** Task 1 的 frozen 类型、净空场、动力学和 certify；禁止修改其接口而不向主 agent 说明。
**Produces:** `MpccPlanner(limits,build_dir)`，`plan(state,grid,reference)->LocalPlanResult`，其中含候选、明确原因、求解和子阶段耗时。求解不调用 certify，benchmark / coordinator 显式执行独立认证。

- [x] 先写失败测试：直线向前推进、允许倒车、原地转向、完整约束；未知两策略；绕障候选及无解时无可执行轨迹。依赖不可用要真实报错，不以 fallback 宣称 MPCC 成功。
- [x] 构建真实 acados SQP-RTI MPCC，状态 x,y,yaw,v,w,s，控制 a,alpha,nu；路线切线参数化轮廓/滞后误差，s 作为优化变量并奖励进度，控制平滑。初始采用 20 个 0.2 s 节点，4 s 预测；根据窗口/制动和目标剩余长度缩短有效参考，终点减速。
- [x] 轮式模式分 forward / reverse / spin，行进曲率约束不禁用静止自转；以覆盖车体的圆组线性化净空硬约束，最终整车认证仍独立。常规单热启动，失败时最多两个左右旁通初始化，总尝试有界。
- [x] 在 seeds 提供明确标为 brake 的受限停车候选供编排独立认证；不得以瞬时清零实测 v/w 来模拟安全停止，停车候选不能计为 MPCC 成功。
- [x] benchmark 在固定有效参考上跑 open、turn/spin、reverse、corridor、thin-obstacle、unknown 两策略、blocked；计入窗口、净空、参考、种子、求解、认证及总周期。记录冷启动构建与稳态周期，保留所有样本及状态，不能删失败样本。
- [x] 依赖仅安装在 `/home/yanfa/P4_veri/deps`，源码固定 SHA，构建于独立目录。Windows 跑纯 Python 测试；Humble/Orin 跑真实求解测试及不少于 100 个稳态周期，反馈不达标项。

**Execution outcome:** 实现及独立审查通过；Orin59 passed/1skip、Windows50 passed/10skip。900稳态完整周期严格50ms FAIL（34/900超时），性能失败保留，不作部署就绪声明。

### Task 3: 稀疏高精度任意角全局规划（阶段二）

**Files:** Create `scripts/navigation/one_cm/nav1cm/{global_planner,global_fixtures}.py`，`scripts/navigation/one_cm/benchmark_global.py`；test `tests/navigation/one_cm/test_global_planner.py`。

**Consumes:** `WorldSnapshot` / `CellRect` / `Route` / `WheelLimits`。
**Produces:** `AdaptiveGlobalPlanner(limits)`，`plan(world,start_xy,goal_xy,version,budget_s=1.0)->GlobalPlanResult`；返回 `planning_outcome:0,reason_code:PLAN_FOUND,has_reference:true` 才是正式成功，另含 domain/expanded/storage/timing。

- [x] 先写失败测试：图外和未知目标可规划，原始障碍目标拒绝，1 cm 细节未因压缩消失，无路/超时/搜索域耗尽区分，输入地图不变，整条线段碰撞检查而非采样端点。
- [x] 用保留厘米矩形边界的自适应自由空间分解（建议垂直 slab 和自由区间 portals），稀疏图搜索及任意角可见性捷径；障碍配置空间膨胀明确使用半车宽，返回几何参考而非承诺动态可行。禁止按地图边长/0.01 分配二维密集数组。
- [x] 全局地图中 UNKNOWN 可通行，搜索域包含起终点和全部已知障碍再加明确余量；有限域穷尽只能报 DOMAIN_EXHAUSTED，不能证明无界平面无路。闭合障碍内外及有限域无路证据分开。
- [x] 跑原生 1 cm 100 m、500 m、1 km 场景以及细尺度窄门、连续细墙、未知/图外目标；同时独立校验每条路径原始矩形的线段碰撞、长度与结果码，至少 20 次稳态计时/尺度及内存；输出真实失败和预算命中。
- [x] 大范围场景分别至少 100、500、1000 个厘米对齐障碍矩形，报告实际图节点/边及障碍数量；不能仅拉伸同一个空旷场景来证明复杂地图性能。
- [x] 用户追加：障碍数量不代表复杂性。另测闭合外围墙内的 100 m / 500 m / 1 km 多门蛇形通道、岔路/死胡同、环路/瓶颈、82 cm 开门与 81 cm 封门，以及新增 1 cm 墙后改道；每例 20 次完整更新。逐路验证指定门穿越/门序、围墙内域及独立连续几何碰撞，分别记录预期有解/无路/预算耗尽；保存布局和路线供可视化审查，报告强制转弯、岔路、死胡同、瓶颈及净通道宽度。
- [x] 运行对应 pytest、UTF-8 与 diff 检查。准确称作二维自适应原型，不冒称官方 wavestar 或无条件最优。

**Execution outcome:** 全局几何原型及门禁修复审查通过；39项global测试，最新导航范围89通过10跳过。新正式规模60条有效参考；封闭通道160有效参考/20预期封门无路，零1秒超时，1km40门约320ms。动态跟随仍由Task4验证。

### Task 4: 编排、接口闭环、完整计时与交付（阶段三）

**Files:** Create `scripts/navigation/one_cm/nav1cm/{coordinator,global_worker,fixtures}.py`，`scripts/navigation/one_cm/benchmark_navigation.py`，`scripts/navigation/one_cm/README.md`；test `tests/navigation/one_cm/test_coordinator.py`。更新原设计、计划状态和 `docs/任务登记.md`；仓库外更新 REPORT.md。

**Consumes:** Tasks 1–3 的公开类型和接口。
**Produces:** `NavigationCoordinator.tick(world,state,goal,now_s)->CycleResult`，输出认证结果和语义事件；不发布 ROS 或实车命令。

- [x] 先写失败测试：独立全局 1 Hz/局部 20 Hz 调度、图障更新使旧认证失效、路线换版重投影、未知策略改变、目标后来成为障碍、无进展触发重规划，求解失败不改写地图。
- [x] 全局搜索通过独立单 worker 进程执行（最多一个在途任务），局部 tick 不等待全局完成；版本化快照与结果由轻量序列化交接，父进程重新构造不可变 Route。分别记录搜索耗时、队列/交接耗时及局部完整 tick，包含首轮等待和更新时的周期。测试用可注入确定性 runner，正式组合 benchmark 使用真实异步 worker。
- [x] 编排只组合组件，不复制数学；超周期/拒绝时使用新认证的受限制动候选或明确报告无可认证停车轨迹，不能冻结仍在运动的状态或沿用已失效认证。各周期统计具有清楚包含关系；全局 worker CPU 竞争产生的局部延迟必须进入实测。
- [x] Orin 离线闭环运行原生 1 cm 案例：直线、转向、绕障重入、地图新增阻挡后改道、未知两策略、目标图外、目标被占据、路线换版/回折；保存轨迹、原始周期及源码/依赖身份，独立校验真实执行的每个动作。
- [x] 分别报告全局和局部指标、闭环到达/安全停止/无解、完整 50 ms 命中率。失败不转换为成功；若性能不足，基于分阶段计时优化并重跑受影响验证。
- [x] 单次完整相关测试与独立审查，UTF-8 / diff 检查；任务登记写明源码未提交、运行目录保留、ROS 节点/实车 NOT_RUN；不合并、不推送、不回收工作树。


**最终验收处置：** 实现和本轮验证交付完成；全局已测场景通过，整体实时/复杂动态指标未通过。
最终全分支审查Important已定向修复，0Critical/0Important，2个非阻断Minor保留登记。
最终源码Windows110passed11skipped；Orin完整真实114passed1skipped在小修前执行，随后两档真实构建2passed、终态纯分类21passed1skipped定向覆盖。
未重跑长批次或声称最终源码下全部历史轨迹重新执行。源码未提交，独立分支及证据保留，后续优化/集成待另行处理。
