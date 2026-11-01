# 导航四项职责修复与本地验证

基线666db9e，hk/p3-pose-wait-20261030/main-worktree，主线通用导航owner。用户指定顺序：旋转边界→完成条件统一→真实制动接转向→局部投影统一。ROOT单写入实施，独立只读审查规划/认证与跟踪边界。无SSH、上传、进程切换、车辆指令、训练、push或合并；tcp2候选未改。

## 1. 旋转边界

1487dca：StandardPathReference对单个稀疏大角也拆为≤π/2，密集采样沿用相同上限，不改变输入角采样的有序方向与圈数。6项半圈微反偏旧实现RED，首轮控制器228通过。

独立复审进一步捕获初始yaw反偏91度/接近180度仍被末端wrap符号误认已执行。补4项旧实现RED，将跨段判断与前视剩余量统一改为相对当前stage起点的有向角进度；起点后方偏差不计作已完成，同向跨内部边界仍无需停稳。仍保留原stage、原参考身份、命令历史；无新增累计器或旋转状态实体。最终控制器236项覆盖稀疏半圈微偏/大反偏、跨±π、多圈、顺逆转、真实反转停稳、最终停稳、滚动参考及终端反馈。

## 2. 完成条件统一

0b4d6c2：coordinator.goal_reached为唯一位姿/停稳谓词，同时用于实测与制动终点。NavigationSession只保存当前已认证ARRIVED的单份证据，绑定world对象、state值、goal和generation；NOT_DUE本身没有新证明，只能在完全相同输入下复用。不同地图、状态、目标、代际立即不能使用该证明。NavigationTask不再重复计算位置/朝向到达，负责执行反馈、任务身份和原有保持条件。Runtime在提交前最新观测改变时仍可否决终态。

这份有界缓存用来区分“本周期不计算”和“先前证明已失效”，不是新任务实体或延时门禁；无ROS接口扩展。候选、grid、limits、digest匹配仍检查。

DRL nav1cm适配器直接交付导航已确认的GOAL_REACHED/CANCELED，删除固定1秒定位接收门槛与旧速度二次否决；terminal_stop_confirmed由现有last_goal_result派生，暂停/取消消费该事实，未新增状态账本。旧导航后端仍沿用原测量停止语义。6项相关RED后，导航会话/末端/任务Runtime/DRL定向78通过。

## 3. 真实制动接转向

ff660e1：seeds.braking_prefix与join_trajectories提取原起步路径的真实制动/接续工具。局部MPCC起步与terminal_candidate复用，删除内联重复积分/拼接。差速车平移制动可保留合法同向角速度，角速度反向、超限或制动距离不足时先物理制动；从预测终点重新计算转向，不要求下一帧测量先成为数学零值。

前缀从原始v/w开始，不改写测量；前缀时长计入本次时域。曲率模型仍用兼容曲率的真实制动。轨迹身份/状态边界检查、完整连续动力学/碰撞扫掠和制动尾段认证保持。

修正新测试地图窗口参数后，10项旧行为RED；关联128通过。完整回归时将5项原本要求“候选只能是brake”的旧测试更新为已授权的“制动后接转向”，加强零速边界、目标转角与认证验证，未删测试。另新增前缀撞障仍拒绝的回归。纯角速度反向也可在同一候选内完成制动接续。

## 4. 局部投影统一

MppiPathFollower._reference_projection一次返回有序局部距离、原segment索引及完整segment上的fraction；父类沿用其切线，标准Path沿用同一次关联插值车体yaw。删除标准子类对整条stage的全局argmin和batch×time×全段数二次搜索；不可变车体yaw数组按path身份缓存，方向补偿仍按执行方向处理。

3项近邻返程/倒车/跨π截窗旧实现RED。增加缓存复用、只读、滚动替换和临时path恢复回归；原局部窗口裁剪、端点fallback、投影tie顺序测试继续通过。独立审查未发现生产stage上重复XY索引问题：有序参考构造在重复XY处分段，旋转单点仍用最终yaw。未测量目标机耗时，不能量化性能收益。

## 最终验证

- 本机导航、认证、会话、任务Runtime与DRL适配：715 passed、126 skipped、6 deselected，17.17秒。
- 标准Path、进展字段生命周期、执行合同、MPPI、几何缓存、Path配对/入口：236 passed，11.04秒。
- 上述为两组本地验证，共951项通过，不等于ROS整系统通过。
- 6项排除原因：NAV1CM_GEOMETRY_LIB与NAV1CM_DISTANCE_FIELD_LIB未配置；为test_clearance中的2个native/Python一致性、1个native旋转制动、2个native角点净空，以及test_follower_map的1个native支持一致性。126项按各自真实依赖条件跳过，包括原生、ROS、真实求解器场景，未作为通过计数。
- 控制器本机验证仅对lunar_planning_msgs.msg.MotionReference提供类型占位，实际算法类/地图/优化/认证照常执行；不是ROS通信测试。
- DRL完整Runtime本机缺lunar_drl_terrain_native，使用实际tick方法的隔离回归检查终态消费，Action仅传输替身；没有伪装完整地形/Actor运行。
- 扩大检查过程中，stdin入口与Windows spawn不兼容且第一次D盘basetemp父目录未建，导致工具环境错误；最终改用标准python -m pytest入口并预建D盘父目录解决。旧终端测试预期同步后，上述范围无剩余算法失败。
- UTF-8读取、Python编译、git diff --check通过。最后只删除抽取工具后遗留的未使用import并更新说明，未修改算法参数。

最新Humble构建、ROS消息/节点联调、真实acados组件、目标机持续带噪定位闭环及TCP运动/长距离绕障全部NOT_RUN。仍需恢复连接后由tcp统一组合tcp2候选、构建、部署与复测；不能声称现场走停或绕障已通过。

## 产物与收尾

源码/测试/计划/本记录及任务登记按切片本地提交待合入，长期线不移动。其他审查项（圆包络、探索预算、参数生效、TCP备用入口、双向加减速）未夹带处理。

本轮后期pytest临时产物在D:/CodexDownloads/lunar_navigation/nav-four-fixes-20261031/pytest-nav-final1，测试结束后可按需要清理，不提交。前期pytest默认生成的小型fixture见C:/Users/77634/AppData/Local/Temp/pytest-of-77634/pytest-100，用于本机测试，测试已结束，可清理；本轮未删除。未在C盘下载或构建依赖。
