# 导航与控制防护职责精简

基线aaa448f，任务PROGRESS-OWNER / GUARD-SIMPLIFICATION，owner integration/pure-planner-orin，复用hk/p3-pose-wait-20261030/main-worktree，ROOT单写入。用户确认删除小进展硬拒绝、不另建导航进展累计、统一等待/重算/失败并收敛P4转发职责。本轮仅本地操作，不连接216、不改P3/Env_X、不覆盖tcp2候选。

## 修改

- MPCC删除虚拟弧长增量小于5mm的NO_PROGRESS硬拒绝；进展仍由原优化目标鼓励，速度、加速度、方向、曲率、净距、参考制动边界与后续独立认证保持。零/小进展不等于已到达。
- 导航删除_motion/_motion_s/_stalled与stall_timeout_s及相应NO_PROGRESS触发；实际位置向量仍供规划使用，路线游标与最终位姿/停稳到达判断保留。周期重规划、地图改变、目标改变和参考失效处理保留。构造函数已移除无用stall_timeout_s，仓内无外部传参，相关旧测试改为验证周期重规划。
- 在既有motion_configuration模块以无状态input_disposition唯一分类：定位/地图暂不可用→等待，发布证书过期/认证期间换图→重算，其余不安全或非法输入→失败；已锁存硬故障优先，不能因等待或retryable标记洗掉。WAITING_FOR_MAP复用既有等待与新参考认证通路，地图恢复不续跑旧几何。_standard_pose_wait沿用内部方法名但负责现有输入等待，reason区分定位与地图；不新增状态枚举/实体/时钟。
- P4适配删除TrackingStatus订阅及reference_accepted/执行状态/状态年龄二次门控。只按同域控制器命令、TCP车辆反馈、既有租约和断流保护转发；保留有限性、低速界、零速park和退出释放。命令是否可执行由跟踪器承担。

## 验证与边界

新增回归先RED：11项失败，分别为3组小进展、7类输入处置、1项平台独立活性。最终本机194通过1跳过，含候选能力、完整认证、异步执行、反馈身份/到达/故障、实际Runtime、周期重规划、P4传输及时序。跳过项为既有非本机验证场景，不能算通过。修改Python语法和git diff --check通过。

最新ROS消息/控制器完整包构建测试、216部署、车辆起步/走停/绕障及到达均NOT_RUN；不沿用旧提交的Humble或车辆结果。没有修改既有运动/认证阈值，没有以增加超时实现本轮精简。仅本地提交待合入，不push、不移动长期分支。

## 恢复部署

tcp2最后读取状态为idle/interrupted，其overlay基线仍提及state(0,1)定位等待特例；该描述已被aaa448f的统一反馈接收活性替代，不能直接覆盖最新runtime。恢复时核对平滑交付、最新source/install和运行进程，再进行保留双方差异的组合；特别保留本轮coordinator删除重复停滞逻辑及P4传输变化。用户授权最终部署/复测由tcp单一入口负责；本轮未发送远端命令。
