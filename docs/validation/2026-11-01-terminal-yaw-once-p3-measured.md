# 终点单向交接与P3实测状态

用户明确要求：全局净空权重1；位置完成后不因位置漂移重开位置阶段，继续完成朝向；不外推P3位姿；不用测试，直接替换P4正式运行。

实现：复用coordinator已有_position_completed，仅目标身份变化/取消清除；goal_reached在该目标位置阶段完成后只再要求实测朝向/停稳。无yaw目标保留位置完成合同。runtime移除CommandPrediction对象、命令历史写入、predict/correct接线和估计状态日志；保留P3原始位姿/速度、源戳推进和实测新观察，MPCC未来候选预测不变。旧command_spin_prepare_s关键字保留兼容启动但不用于当前位姿外推。

旧P4核对：P4_legacy_20261030/debug/p3_joint/controller.yaml实际覆盖10cm位置、0.05rad航向、0.1rad/s；保存源码与旧已安装execution/tracking/node逐字一致。FINAL_ALIGN的位置丢失会GOAL_POSITION_LOST制动；本次明确采用用户新要求，不复用该回退。旧停稳0.01m/s与0.03rad/s。新版既有5度朝向容差未改。

修改前新阶段回归已复现12cm漂移后清除位置标记。随后用户明确取消测试，本轮不运行pytest、ROS闭环或实车测试，仅语法解析、diff核对及部署后进程/参数读取；不能声明回归通过。之前版本的测试不覆盖本轮新行为。

owner integration/pure-planner-orin，专题hk/terminal-yaw-once-weight1-20261101，基线9aa27e6；应用同步与P4替换沿本会话授权，不push，P3/Env_X文件不改。
