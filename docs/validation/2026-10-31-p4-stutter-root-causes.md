# P4顿挫原因复查（只读诊断）

任务继续P4-DIRECT-MPCC-CONTROL-20261031；运行源码21efca0，专题hk/tcp-combined-deploy-20261031；本轮不修改运行源码、参数、进程或目标，不修改P3/Env_X/legacy。只读采集与隔离复现不是新控制验收。

## 40秒现场证据

P4/artifacts/stutter-analysis-21efca0-20261031.jsonl与stutter-analysis-summary-20261031.json：最终cmd376条、零速76条；规划器私有cmd374条、零速74条；实际轮指令199条中62条全零（仅诊断，非闭环输入）。最长连续零段4.836秒，对应35次前进候选LOCAL_CERTIFICATION_OUT_OF_BOUNDS及73次BUFFER_EMPTY。31–34秒窗口15条轮指令全部轮速零，证明确实停车，不只是RViz显示跳变。

控制接收间隔中位0.0835秒、最大0.6396秒；私有cmd四个间隔尖峰为0.6353、0.6157、0.6025、0.6613秒。P4转发端额外两次零速分别在t=0.7793、36.8107，距上一私有命令约0.619、0.646秒；此前私有命令均非零，之后约18毫秒又收到新命令。对照p4_tcp_runtime.py的0.6秒command_ready门槛与heartbeat.park，证据指向该重复指令时限触发停车/auto重入。车辆接口自身已有1秒cmd_timeout和5Hz检查；原地旋转准备3秒，零速/park退出准备，短零也会放大为物理停顿。

P3接收间隔中位1.144秒、最大2.273秒；源数据年龄中位2.388秒。本窗口未出现ODOMETRY_WAITING，不能把这次主要停车归为定位失效；低频延迟定位仍会令RViz位姿显示跳动。实际轮指令与车体遥测已证明另有真实停顿。LOCAL_RESULT worker CPU中位约7.72毫秒，不能把整个0.6秒尖峰都归咎于acados求解。尖峰的地图/认证/调度耗时还未逐段profile。

采集对应generation2目标(10.720040,-21.832531,-2.199339)。之后用户在RViz另选generation3，统计未混入新目标。

## 空旷窗口真实acados对照

脚本与结果仅在P4/artifacts/replay-stutter-free-window-r2-20261031.py、stutter-free-window-replay-result-r2-20261031.json。复用上述采集t约30.49秒P3初态、当时目标、当前TCP .5/.5/1能力、5m窗口；将障碍设为空旷，复用已有生成库且generate=False/build=False，不覆盖求解器、不发布车辆指令。

连续三次真实求解均status0、CANDIDATE；原候选认证BRAKING_OUT_OF_BOUNDS，ReferenceTrajectoryBuffer追加停车尾后变成OUT_OF_BOUNDS；同一候选当前.05秒动作+1.2秒持令+停车尾认证全为VALID。例一：源末端v=0.36958m/s，y=-18.05782；追加停车后y=-18.19272，车体连同净空越过窗口下边界y=-18.98030。不是现实障碍，问题在滚动参考的远端停车要求与当前有限执行权限混用。

该实验复现与现场一致的拒绝类别并证明机制冲突；没有保存现场35条原始拒绝候选，不能写成35条逐条重放通过。

## 转向重锚MODE确定性反例

复用现有braking_prefix/join_trajectories/TrajectoryBuffer/ReferenceTrajectoryBuffer/certify，空旷网格、差速能力。原spin来源从真实v=0.0002m/s单调制动至0后旋转，证书VALID。新测量v=0.0001m/s、相同pose/yaw/w，重锚先桥接至旧来源的0.0002，再重播旧制动前缀至0，生成速度序列[0.0001,0.0002,0,0]，认证MODE。新测量为-0.0001时同样产生[-0.0001,0.0002,0,0]并MODE。通过正式ReferenceTrajectoryBuffer.prepare_source（几何投影phase0）同样复现，不只是强行指定相位。

说明重锚在重复执行旧制动入口，认证拒绝的是重锚自己生成的非单调制动；不是需要放宽MODE或抬高微小速度阈值。现场generation2起步期间确有LOCAL_CERTIFICATION_MODE/BUFFER_CERTIFICATION_MODE，但本40秒窗口以远端越界为主，不能混写计数。

## 应处理的职责

1. 先修参考与执行认证边界：参考本身动力学/碰撞检查保留；当前真实速度命令的响应/持令/停车检查保留。审查滚动参考远端必须停车是否仍是实际执行需求，不能用远端假想停车否决当前已证明安全的短动作。若整条参考仍承诺可随时执行到末端，则MPCC必须显式提供一致的可停车末端可行域，不能在后处理中附加一个经常不可行的尾段才拒绝。
2. 重锚保留物理来源与正确相位，但真实制动入口只能按当前测量重建一次，不能先恢复旧残余速度再重播旧制动；复用已有工具，不增加另一套阶段机。
3. 去除或统一P4重复的0.6秒断指令停车职责，由现有车辆接口负责真正断流停车；恢复auto/park责任也要一起核对，不能只删一行使底盘watchdog停车后P4误以为仍armed。不要仅改成另一个任意阈值。
4. 再profile必要的当前地图/实际命令认证开销，不把所有顿挫归为定位延迟或求解器，不通过调大限速/放宽碰撞解决。

本轮结论是诊断证据；以上修复尚未实施。自然到达/绕障/Orin未新增验收。
