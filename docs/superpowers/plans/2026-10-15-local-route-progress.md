# Local Route Progress Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复已知全局绕行路线的跨墙投影导致局部折返/循环，并与已冻结的局部旧版进行同场景对比。
**Architecture:** 局部目标估价只接入当前窗口内、由现有细格supercover证实直线连接FREE的指导路段，在检查过的合法连接中取最小“连接距离+剩余弧长”。优先检查后端点，再检查垂足/起点，不声称求出连续可见域的精确最优。不修改地图、不选择探索目标。对于未被细图证实的乐观指导，保留现有前沿等待和平台搜索契约。给目标构造传入原周期deadline/stop，保持旧入口兼容。
**Tech Stack:** C++20, ROS2 Humble, gtest/CTest, Python NPZ固定地形/真实公共控制器/SE2闭环。
**Spec:** 用户本轮明确要求修改局部层并重跑；沿用现有地图/导航/探索职责边界及原始比较指标。

## Global Constraints
- 不修改全局搜索/地图构建/车辆控制器/训练/模型/回放/原安装。
- 全局预算500ms、周期预算和局部窗口64m、速度0.2m/s及目标容差均不变；原始结果和冻结运行库全部保留。
- 只在adaptive-navigation功能工作树及新artifact目录构建运行；不合并、不推送、不提交、不删除工作树。
- 比较中明确全局指导线、首段决策、实际行驶及无路/超时；公里级plan-only不声称已行驶。

## Review Focus
- 墙另一侧的将来路段不能给当前候选虚假低余路；允许绕行暂时远离最终点。
- 可见、合法的真实近路仍可选，不能只锁定单个最近投影或强制跟随所有离散节点。
- 不连通/未知/盲起点的现有等待及平台运动可执行性仍由原契约负责。
- 全局折线穿出再进入局部窗口时，连接检查仍有界；窗口边界和对角格接触不穿障碍。
- 新增构造遵守deadline/stop，超时不给出伪NO_PATH，也不提交路径。

### Task 1: Reproduce then repair target scoring
**Files:** test/reachable_local_goals_test.cpp; src/local/local_goal_region.cpp; include/local_target_selector.hpp.
**Interfaces:** 保留SelectRolling原入口；新增接收const SearchControl&的重载。
- [x] 加墙/近邻回程回归：错误上方候选被拒绝，两个平台局部路径向正确下方出口推进；先看旧实现RED。
- [x] 加开放空间合法近路和预算/取消回归；原前沿/盲区/终点朝向测试保留。
- [x] 用现有grid_supercover做窗口裁剪后连线证明，修复已检查连接的余路估计；运行目标测试GREEN。

### Task 2: Budgeted production wiring
**Files:** planning_session_coordinator.hpp/.cpp; incremental_navigation_node.cpp; coordinator tests.
**Interfaces:** PlanningSessionPorts末尾新增select_target_controlled(fine,window,start,goal,guidance,SearchControl)，原select_target继续兼容。
- [x] 先写构造阶段期限耗尽返回TIMEOUT/取消不提交路径的RED回归。
- [x] 生产ROS传原deadline与stop；旧测试端口不受影响。
- [x] 新隔离Release构建core/ROS，全包串行CTest通过；独立审查并处理实际发现。

### Task 3: Same-fixture comparison
**Files:** scripts/navigation中的局部修复比较脚本/报告，artifact放local-route-progress-20261015。
**Interfaces:** 读取前两轮19个固定NPZ/哈希、冻结新旧记录；只在新私有安装运行修复后的adaptive_known。
- [x] 先跑long_wall/serpentine/spiral小组，确认实际到达及消除折返，再跑其余全组。
- [x] 固定同预算/参数，保存真实全局路线、PathReference、物理轨迹和完整诊断/运行库SHA。
- [x] 汇总三阶段结果和前后路线图；核验模型/原安装及旧运行库未变，git diff --check。

## Completion evidence

- 本机 Humble Release：核心17/17、ROS13/13 CTest条目串行通过。
- 19场景：16/16可达闭环到达、0碰撞，1个不连通对照返回NO_KNOWN_ROUTE，2个公里级场景仅测规划且指导线M通过。
- 上轮仅改全局为14/16到达，原算法为7/16；旧记录与运行库均保留。
- 本轮102个PLAN_FOUND周期，最大完整决策649.94ms；有额外单周期计算开销，不宣称所有场景更快。
- artifact：`/mnt/data/WS/lunar-navigation/analysis-artifacts/local-route-progress-20261015/report.md`，`verification.json`全部279项通过。
- 独立审查的可见后端点Important问题已用公共接口RED→GREEN修复；有限锚点估计不作连续最优或完备性承诺。
- Orin、完整动力学与实车NOT_RUN；正式训练/模型/经验/原安装未修改，未合并/提交/推送。

执行方式：用户已授权在本会话直接修复并比较，沿用本会话执行方式，不增加重复审批。
