# P3 输入与双 Path 输出实施计划

> For agentic workers: REQUIRED SUB-SKILL: superpowers:subagent-driven-development。按任务串行实施，规格审查后再作质量审查；不提交、合并、推送或删除资源。

**Goal:** 实现用户已确认的 P3 原生分辨率输入、固定厘米局部计算、完整位置朝向目标及两条标准 Path 输出。

**Architecture:** 复用 lunar_incremental_navigation_core 的高程累计与轮式地形判定，提供独立 native 桥接及无 ROS 地图转换层；原 nav1cm 核心继续只处理不可变厘米快照。目标生命周期和末端朝向放在核心/无 ROS 会话层；新增 ROS 包只做消息转换、非阻塞调度和发布，不依赖 DRL、验证 plant 或车辆控制。

**Tech Stack:** C++20、Python、NumPy/SciPy、现有 acados/CasADi、ROS 2 Humble、pytest/CTest。

批准依据：2026-10-27 用户确认 docs/superpowers/specs/2026-10-27-navigation-p3-path-interface-design.md。
复用 NAV-1CM-20261026，hk/nav-1cm-feasibility-20261026，基线439178f；所有构建/测试产物放任务D盘目录或授权远端验证目录。

## Task 1：地图与原生地形桥接

Files: 新建 nav1cm/p3_map.py、nav1cm/terrain_native.py、nav1cm/terrain_bridge.cpp（均在 scripts/navigation/one_cm 下）；新增 tests/navigation/one_cm/test_p3_map.py 及 native 对照测试。

- [x] 先写并运行行为失败测试：0.005/0.01/0.03/0.05/0.1/0.2 m输入、非整厘米原点、GridMap行列布局/滚动起始索引、未知/障碍、同一会话源分辨率变化；输出仍0.01m/500²。
- [x] native桥直接调用既有 PersistentElevationMap / MakePlatformElevationEvaluator，输出未按车体膨胀的 intrinsic 分类；不导入 DRL 包，不复制坡度/高差阈值算法。提供明确ABI和构建入口。
- [x] Python 层读取GridMap真实几何与层，维护历史地图并将分类结果保守转换为 WorldSnapshot 整数厘米区域；先在源分辨率判定地形，障碍与UNKNOWN不因重采样消失。新原点/分辨率输入不拒收；避免每格长期生成独立矩形，合并行程/区域。
- [x] 执行 Python 行为测试和真实 native 对照；确认新接口可供后续任务调用：P3MapStore.apply(message, transform) -> WorldSnapshot，snapshot属性和 source_resolution_m 可查询。依赖缺失时明确失败，不用替代地形算法冒充复用。

Task 1 验证：规格及质量复审均 PASS；KAI 台式当前 ABI2 独立重建，相关 7 文件 180 passed。补齐旋转细 UNKNOWN 条带、跨源新旧观测覆盖、重复地图 epoch 和 8 源 LRU 回归。原失败记录、稳定源码清单和资源复测保留在任务外部 artifacts/p3-interface-20261027。

Example acceptance:
```python
store.apply(grid_map(resolution=.05), identity_transform)
assert store.snapshot.resolution_m == .01
assert extract_window(store.snapshot, state, False).cells.shape == (500, 500)
store.apply(grid_map(resolution=.03), identity_transform)
assert store.source_resolution_m == .03
```

## Task 2：完整目标姿态与末端转向

Files: 新建 nav1cm/pose_goal.py；修改 contracts.py、coordinator.py、async_coordinator.py、local_worker.py、local_mpcc.py 中确有必要的部分；新增 test_pose_goal.py / test_terminal_yaw.py。

- [x] 先写失败测试：同位置不同yaw、跨pi最短角、到位置未到朝向不能ARRIVED、末端转向车体扫掠碰撞、最终朝向已满足时停车、未知/图外目标不拒绝。
- [x] 保持现有仅xy调用兼容；新增显式最终yaw路径。MPCC末段使用目标朝向，位置附近可以生成满足角速度/角加速度并经现有整车认证的转向轨迹；不只改末点四元数。
- [x] 位置/朝向/停稳全部满足才能到达；同xy新yaw亦使旧worker结果/缓冲失效。数值失败不是无效目标证明。
- [x] Windows契约测试及远端真实solver短例验证末段姿态；旧纯xy回归不能退化。

Task 2 验证：规格及质量复审均 PASS，最终 source-goal-r4/8 文件哈希一致。远端相关回归 143 passed/1 skipped，质量审查另独立复核真实前进/倒车 MPCC 终端短闭环 17 passed。保留原 xy 接口；新增停稳后位姿越界、近浮点终点及旧路线重入回归。解析末端转向明确标记 terminal-yaw:bounded，仍逐段认证，不冒充 acados 成功。

Example acceptance:
```python
result = coordinator.tick(world, VehicleState(gx, gy, 0., 0., 0.), (gx, gy), now, goal_yaw=1.)
assert result.status != 'ARRIVED'
# 在后续实际模型推进中核对最终xy/yaw、停稳和每段扫掠，不靠伪造末点。
```

## Task 3：会话层与ROS包

Files: 新建 nav1cm/p3_session.py、nav1cm/ros_conversion.py（均无ROS导入）；新增 scripts/navigation/one_cm/ros2/lunar_nav1cm_planner 包、launch/config及测试。

- [x] 先写目标生命周期与Path转换失败测试；有效新目标覆盖，非法新目标保留原目标，新目标无路保留新目标，旧结果不发布。
- [x] 标准输入：grid_map_msgs/GridMap、nav_msgs/Odometry、/tf与/tf_static（可配置旧转发）、geometry_msgs/PoseStamped目标。允许零/缺省TF时间戳、缺省目标frame按map、非单位四元数归一化；不增加严格同步/固定源分辨率门槛。
- [x] 标准输出仅global_route与local_path（nav_msgs/Path）；全局几何、局部含车体yaw和参考时间t0+ti，保留倒车/原地转向点。空局部路径表示受控停止而非取消。
- [x] 地图处理、solver不阻塞ROS输入回调；保留当前异步全局/局部worker与软周期行为。状态/TF使用最新值；无必需数据等待日志，TF静态/零时间戳不拒绝。
- [x] 新包安装nav1cm核心与native桥，可在清空源码PYTHONPATH的已安装overlay中运行。不依赖verification_plant，不发布T5。

Task 3 验证：规格及质量双审 PASS；Windows 核心 375 passed/116 skipped，KAI 真实 native/solver 核心 490 passed/1 skipped。Humble Release 构建及包测试通过（8 pytest 加 2 CTest 汇总项）；installed-only 消息 smoke、零 stamp TF 更新及线程/worker 退出通过。真实入口 SIGINT 0.815 s、退出码 0，无升级终止。source-p3-r3 保留代码基线；仅 README 后补“允许下游自行重定时”说明，最终 Task4 快照统一收录。

Example acceptance:
```python
session.offer_goal(valid_pose)
assert not session.offer_goal(obstacle_pose)
assert session.goal == valid_pose
paths = converter.paths(route, trajectory, ros_start_ns)
assert paths.local.poses[-1].header.stamp > paths.local.poses[0].header.stamp
```

## Task 4：远端构建、DDS验证与收尾

Files: 增加有界ROS输入/探针脚本和使用文档；更新原登记及实施报告。

- [x] 只读核对远程台式主机、架构、ROS版本/依赖与可用目录；复用授权入口，构建输出分离，不混overlay。
- [x] 用目标机C++/真实solver运行相关完整测试；构建新ROS包和依赖、执行包测试。
- [x] 隔离ROS域内运行独立GridMap/Odometry/TF/PoseStamped生产者及Path消费者；证明真实类型、零TF时间戳、多源分辨率/热变化、局部厘米图、位姿终端、有效/无效目标更换、无路保持、慢计算续用、空路径和进程退出。
- [x] 测试限100m及短运动闭环；只模拟下游运动，不发生产控制。分别记录源码/纯计算/ROS消息/运动证据与NOT_RUN。
- [x] 各任务先规格后质量审查并修复问题；最后综合审查、UTF-8读取、git diff --check。保留原失败及旧证据；按实际状态更新登记/源码清单/报告，不把旧原型证据算作新P3接口通过。

## 验证命令

Windows：从工作树根用 `python -m pytest tests/navigation/one_cm -q`，显式设置PYTHONPATH为任务deps/windows并把pytest临时目录放D盘。
目标ROS环境：使用独立build/install/log，`colcon build --packages-up-to lunar_nav1cm_planner`，`colcon test --packages-select lunar_nav1cm_planner`，再运行有界DDS和运动验证入口。

Task4实测：2026-10-28 KAI x86_64/Humble，r6最终六场全部PASS（五场三进程+一场双进程空Path注入，共17角色，均exit0/无升级/残留[]）；最终完整core537 passed/1 skipped、76.86s；final-r1包Release构建与10项测试汇总通过。保留初版异步接续、r4fix有限来源语义和r3纳秒量化失败证据；不把历史false改写为true。独立消费者由Path几何重定时，真实P3流/当前源码Orin/实车仍NOT_RUN。

后补质量P2：监督器提前exit0异常曾只写error而未否决；现run汇总和离线评分均以监督错误证据否决，保留其他诊断。纯mock红3失败/1正常通过→受影响16项全通过；原59文件SHA不变，更严离线复算6场仍PASS，未重跑ROS/537或改production。已审final-r3单列后补harness/评分器SHA，不冒称原运行SHA；P2经独立窄规格及原QA复审关闭，整体独立复审PASS。final-r4仅收尾Markdown状态及复跑目录命令，运行代码/测试/评分器逐SHA不变。

计划状态：Task1–4及r5b/r6修复均已完成规格与质量双审，监督P2已关闭，整体独立复审PASS；本阶段实现、验证与复审完成，最后review项已按实际结果勾选。见[实施报告](../../reports/2026-10-27-nav1cm-p3-interface-validation.md)。处置为暂停保留（本阶段完成，源码未提交，待用户审阅及后续提交/合入授权）；源码未合入、未推送，专题资源保留待处置，不宣称仓库整合或资源回收完成。
