# P4局部MPCC直接控制实施计划

用户明确：全局规划→局部路径规划→cmd_vel；P3定位反馈闭环，不依赖四轮姿态/轮速。定位保持/Car/T3/localization/odometry；不接/car/odom、不新建坐标配准；底盘零速摆正不变，不实施反转策略。反馈中使用P3车体位姿与其body twist，底盘轮级运动学仍由已有车辆接口承担。

基线7a80583，公共执行算法owner integration/pure-planner-orin，P4接入owner app/orin-unreal-simulation。复用登记专题hk/tcp-combined-deploy-20261031；源码每批唯一写入者，逐批独立复审，ROOT唯一216部署者。仅P4可写，P3和Env_X/InterFace/legacy目录只读，0.5m/s与1rad/s平台上限和原加减速不变，共享基础地图暂停。无push/长期线移动。

## 第一批：直接执行核心

复用execution.action_prefix/execute_action、CycleResult.action/action_certificate以及现有need_action消费端区分。增加最小执行选择（mpcc_cmd_vel与兼容原path_mppi），未启用控制时不得发布速度，其他应用默认行为不擅改。直接模式只由MPCC计算轨迹/控制，执行层把当前已认证短动作转换为Twist的线/角速度，不做第二次路径优化，不直接取未来任意节点速度，不用模型预测状态假装实测反馈。

NavigationSession直接模式才need_action=True，默认原路径模式不变。取消/完成/等待/失败的速度处理明确定义并用P3实际反馈完成判定；取消分支也必须正确生成或读取认证制动动作。来源有限、整段/真实短动作证书、当前地图与真实初态/能力检查保留；无另加源龄门禁、计时器、任务实体。runtime发布速度沿既有generation/取消/pose-valid检查，失去反馈明确零指令。planner/node/task运行直接执行时不得等待已移除MPPI的外部TrackingStatus，不伪造外部实际停稳；优先用既有_feedback_required=False及P3 pose检查，保留Action/路径可视化与必要身份接口兼容。

回归RED→GREEN：位移和旋转动作到正确cmd_vel、反向合法及零速、.05与模型.2周期截断、初态/地图/版本/动力学/碰撞拒绝、取消/缺反馈/旧代际不发运动、无外部跟踪状态仍按P3实际反馈完成、不模拟exec_action推进在线状态。先纯函数/运行时测试，再真实Humble节点。

## 第二批：P4启动与车体反馈边界

launch增加最小backend选择：enable_controller=true且mpcc_cmd_vel时planner拥有command_topic且不创建MPPI节点；兼容path_mppi不破坏。P4主和有界入口均显式选mpcc_cmd_vel，同一TCP profile、P3地图/定位、现有目标Action与RViz路径不变。命令仍先至P4私有cmd_vel再经已有relay发送/car/cmd_vel；/car/set_mode仍由现有relay负责，只有一个速度owner。relay不读取四轮字段；必要的传输接管/停稳只使用车体vx/vy/wz，不参与P3导航位置估计。不得关闭P3内部传感器融合。

回归拓扑：直接模式无MPPI节点，cmd_vel发布者唯一；两入口定位仍P3、平台能力同源、预检不接管；body-only遥测不需要wheel字段；未启用控制不输出。现有下游Action入口不变，保留tcp2独立代码成果。

## 第三批：复审与216隔离验证、部署

冻结每批提交，ROOT核对P4生产文件基线后隔离Humble构建受影响包、执行包测/实际ROS直控命令到达与取消检查（独立域无车辆接管）。真机切换先停旧P4并用车体新观测确认停稳，逐项备份必要源码/配置，不修改其他目录。保持原接口零速摆正、既有车体速度上限。最后一轮有界实际P3闭环复测，记录actual path/command/body pose，不以离线状态推进冒充实测。若未自然到达，明确记录当前阻断，保留可恢复运行入口；不把源码/包测通过写成车辆验收。

## 现场反馈后的执行接口修订

2026-10-31 用户明确删除计算期间正常P3更新导致零速的规则，并要求明确MPCC到cmd_vel的连续输出。a475259现场18秒196条车辆指令中19条零速、最长连续同向2.50秒、角速中位0.00571rad/s；车辆接口3秒转向准备被打断，不能以非零转发数当运动通过。

批次A：每控制周期冻结P3采样初态；普通新定位下一周期使用，不因状态不相等输出零。终态仍需最新真实推进观测。目标/取消抢占、真正定位中断与无效、地图碰撞仍有效。

批次B：MPCC依旧唯一优化器。首个配置的shooting区间末速是速度参考，命令按上一条成功发布值和真实控制时间限加减速趋近参考，禁止累加MPCC加速度形成隐式积分器。上一条命令不是实测、不写回session。实际发出值从冻结P3初态建立受限响应、持令和制动轨迹，再独立认证；原candidate仅证明参考。真正制动前缀不得无条件跳过。复用horizon_dt_s，无新外部调参开关/轮级反馈/阶段实体。首区间目标若仍不足必须记录，不能把提高取样跨度称为根治或盲目提高限制。

回归：两种正常定位竞态不清零；延迟近零twist下命令连续超过3秒、slew由最后发布值推导、命令与实测分离、真实发出值证明、碰撞/取消/终态/反向与制动过渡；独立复审后Humble构建与P4实际验证。仅P4写入，Env_X和P3不改，零速摆正及原准备逻辑不改。
