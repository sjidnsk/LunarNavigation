# Orin 足式复用轮式跟踪器分析

2026-09-22，初始源码审查后已完成第一阶段：复用路径执行框架，增加足式能力映射与
低速 vx／omega 部署入口，并通过本地导航闭环。用户确认真实足式驱动尚未接入；
横移、步态及真实车辆控制验证仍未完成。

## 已调整配置

两个足式导航入口显式设置 `navigation.clearance_weight: 2.0`，与轮式默认权重一致。
平台自己的净空距离仍使用足式 0.3 m，轮式为 0.2 m，机身包络仍分别计算。
`legged_orin.yaml` 设置 `start_blind_zone_margin_m: 3.0`；通用 `legged.yaml` 不变。
3 m 是包络外额外余量，仍经过起点补图的支撑平面、出口及障碍检查，并非放开所有未知区域。
传感器配置继续为水平视场 360°、距离 10 m。

## 可以复用与必须适配的部分

| 部分 | 当前实现及足式适配判断 |
| --- | --- |
| 输入与反馈 | `incremental_reference` 模式使用 PathReference 的 session/revision、失效通知和 TrackingStatus，可继续沿用导航闭环；普通 Path 模式不能替代这套版本反馈契约。 |
| 执行管理 | PathExecutor 的换段、刹停、进度、偏离检测、停滞检测与终点处理框架可复用；内部对齐、曲率限制与控制律需要按平台拆分。 |
| 位姿与保护 | 可复用 odometry/TF 输入及超时停止。当前停车判据已使用 hypot(vx, vy)，但实际站稳和底盘失能不由它认证。 |
| 配置加载 | 原 node.py 硬读轮式 maximum_spin_rate_radps、maximum_acceleration_mps2、maximum_braking_deceleration_mps2、maximum_curvature_per_m、maximum_lateral_acceleration_mps2。足式 YAML 使用 maximum_yaw_rate_radps、maximum_linear_acceleration_mps2，且缺少后三个轮式字段；原实现直接加载会失败。现在 platform_policy.py 按 platform_type 分别映射并验证限幅；足式制动使用专用控制配置，待实测确认。 |
| 控制输出 | TrackingCommand 只有 vx、omega，发布 Twist 也只填 linear.x/angular.z；无法执行保持机身朝向的横移。 |
| 控制律 | 追踪沿路径切线选前进／后退，并通过 Pure Pursuit 的曲率约束耦合速度和角速度。可以让足式转向后前进，但不能直接利用足式独立横移。 |
| 路径语义 | incremental_path.py 提取 x、y、yaw，未使用 z；PathReference 没有落脚、步态、抬腿、跨沟执行指令。地图判可行不会自动产生跨障动作。 |
| 实机接口 | 需确认足式底盘命令的坐标系、单位、使能／行走模式、命令超时与零速行为，以及故障反馈。现有代码无法证明这些与 Car_Cmd_Vel 完全相同。 |

源码入口：`ros2_ws/src/lunar_pure_wheeled_controller/python/lunar_pure_wheeled_controller/`
下的 `node.py`、`execution.py`、`tracking.py`、`limits.py`、`incremental_path.py`，以及
`ros2_ws/src/lunar_planning_msgs/msg/PathReference.msg`。

## 建议适配顺序

1. 平面基础版：保留 vx＋omega 跟踪方式，增加足式能力映射与底盘接口适配，独立配置速度、
   加减速、停车阈值、跟踪容差。前提是底盘支持对应速度模式、原地转向，并能自行保持步态。
   转向会改变矩形机身的扫掠范围，需验证路径转角处旋转空间；不能假定原计划的横移可直接替换。
2. 横移版：复用版本与状态框架，把平移控制改为机身坐标下 vx、vy，独立控制 omega；
   增加横向限速／加减速、实际速度反馈及保持朝向的横移测试。中间路径朝向如何约束机身，
   必须与规划端统一，不能只增加 linear.y 发布字段。
3. 台阶／跨沟：只有确认底层能够自主执行相应地形运动，或补齐动作／步态接口和能力验证后，
   才能使用足式地图更宽的台阶、间隙阈值做实机闭环验收。

控制器继续只负责跟踪并保持单一车辆命令发布者；规划、探索、重规划仍由现有导航模块负责。
接入后至少验证取消与换段、参考失效、里程计／TF 超时、故障停车、横移、拐角和终点朝向，
并用目标机实测制动与跟踪误差确定配置。第一阶段已实现并验证本地闭环；横移与跨障执行仍未实现。Humble、原生 Orin、真实底盘验证均 NOT_RUN。

## 第一阶段落地与容差修正

入口为 `pure_legged_controller.launch.py` 与 `scripts/orin/start_legged_controller.sh`，
加载 `orin_legged_controller.yaml`，复用现有节点名称。足式模式必须提供能力文件且使用
incremental_reference；能力缺字段、非有限值、非正值及平台不匹配均拒绝启动。
通用轮式入口与控制律保留；低速和关闭倒车是专用 YAML 默认值，直接运行通用节点需显式加载。

完整闭环首次发现：局部规划默认允许约 10° 朝向误差，导航只允许 0.05 rad。跟踪器完成后，
规划器把偏差约 0.067 rad 的当前位置继续判为满足目标，反复产生已完成的静止参考，Action 不结束。
现在真实足式规划端使用导航 yaw 容差的一半（部署为 0.025 rad），跟踪端使用 0.02 rad，
总预算 0.045 rad 小于导航 0.05 rad；未放宽导航验收。轮式规划端不变。
离散 spin primitive 仍约束足式可达朝向；不能满足严格窗口时可以返回 NO_PATH，
并未增加连续旋转认证或承诺任意角度均可达。跟踪端容差须小于导航预算剩余部分。

本地实测两次 GOAL_REACHED 与一次运动中取消 CANCELED，见
[第一阶段验证记录](validation/2026-09-22-orin-legged-tracking.md)。
