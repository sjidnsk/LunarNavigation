# 起步制动衔接与长距离复测

任务 LONG-OBSTACLE-20261030 / INITIAL-DIRECTION，owner integration/pure-planner-orin，复用 hk/p3-pose-wait-20261030 与 main-worktree，基线 0dd0f3b，ROOT 单写入。远端仅修改 P4，P3/Env_X 只读。目标仍为用户指定 (-16.5,0)。

## 原因与修复

attempt-4 在起步转向时反复 BRAKE，未出现真正平移。导航既有停稳条件为 |v|<=0.01 m/s，局部 spin 要求近零线速度；P3 实际残值 -0.0045547 至 +0.0076269 m/s 导致模式无法连续衔接。定位持续推进与正式全局 PLAN_FOUND 仍存在，不能归因于固定源龄门禁。

局部规划从真实状态生成物理减速前缀，并从推导的制动终点继续求解；差速模型保留角速度连续性，不将测量归零。独立认证由实际状态/控制推导出单调、不反向、无进展信用的制动前缀，检查所有段的速度、加速度、动力学、碰撞扫掠及完整制动尾。到零速节点才按主模式认证，不新增消息或身份实体，不放宽数值能力/超时/安全检查。初始方向提示通过已有参数传递现有角向停稳条件，提示结束后双向运动仍允许。

## 已执行验证

目标机 related-final.log：100 项通过；Humble 规划器包 60 项、标准 Path 控制器包 627 项，均 0 错误/失败/跳过。全构建目录汇总另含既有 incremental_navigation_node_test 失败，不把全树报告为全绿。

真实 acados 使用 attempt-4 捕获的两组实际状态求解，首状态保留原值，两组均输出 spin + braking-entry，完整独立认证 VALID。这是自由地图组件验证，不能证明现场绕障。证据仅存 P4/artifacts/initial-braking-entry-20261030 与仓库外 D 盘。

复测前 7 个源码/测试文件与本地哈希一致，其中 5 个运行源码安装哈希一致，回执 pre-motion-receipt.json。attempt-5 未等到控制器参数服务，未发目标、未取得租约，停稳和启动组退出确认；attempt-6 仅将测试脚本参数服务发现等待从 10s 改 45s，不改变产品运行时条件。

## 当前处置

源码已验证并本地提交 d7ac6bb，待合入；长期分支不移动、不推送。attempt-6 有界车辆复测进行中，起步/走停/实际绕障/到达仍未通过，必须依据真实轨迹与结果单独更新。测试上限 1800s、距起点 18m，线速 0.1m/s、角速 0.195077rad/s；测试中取消及停车沿原脚本执行。只有 first_translation_command.v>0 才记录前进起步通过，不能以单纯旋转替代。


## attempt-6 现场结果与后续阻塞

固定目标 (-16.5,0)，本次实际起点 (-4.67488,0.73951)，距目标约 11.85m。发目标后 12.768s 以 EXECUTION_FEEDBACK_TIMEOUT 结束；9 个非零指令均为转向，无 first_translation_command，无 INITIAL_START_COMPLETED，因此前进起步、走停解决、实际绕障、到达均未通过。P3 最大位移约 0.00535m，TCP 最大位移约 0.00876m，不能当作有意平移。停车已实测确认，launch退出0，测试租约释放。

定位接收最大断档9.3059s，地图9.7539s，超过现有无更新预算；这不是固定源龄停车。事件相对时间42.208s开始WAITING_FOR_POSE；等待期间参考版本推进至894。46.802s定位源恢复推进，46.839s跟踪器切到旧版本793的state6/accepted=false地图失效反馈，后续72条均是这一旧身份。793未被实际接受且低于最新已观测894，任务合同正确拒收；新等待心跳停止约2s后，48.793s出现EXECUTION_FEEDBACK_TIMEOUT。控制器_tick_standard_path在定位恢复后先检查地图，_standard_unavailable使用旧executor reference发故障，绕过了采用最新offered reference的等待路径。需要统一可恢复输入等待与真实执行失败的反馈归属，不延长超时、不移除地图/碰撞检查，也不能恢复旧轨迹。使用当前代码隔离重放788accepted→793未接受→894等待→793旧失败，复现旧失败拒收及相同超时；未增加接口实体或修改此处产品行为。

全局三组路线正式PLAN_FOUND，额外检查均不穿已捕获障碍格，路线绕到y=-5.3187m，但没有实际行驶经过障碍，不能报告绕障通过。现场原始产物P4/run/tcp-long-obstacle-20261030/attempt-{5,6}-initial-forward，D盘execution-tests同名副本及derived-summary.json，安装/隔离重放证据位于initial-braking-entry目录。代码d7ac6bb及格式记录659ce63已提交待合入；本次没有继续盲目重试。

用户反馈RViz点目标不动后只读图检查：普通rviz在域10发布/goal_pose，无订阅者；p4_planning_rviz在域57发布/p4_p3_debug/goal，导航器有订阅。但P4控制器输出/Car/T5/Car_Cmd_Vel在域57无订阅者，TCP接口在域10收/car/cmd_vel。测试转发进程已退出，/car/external_control_lease无发布者。现有常驻P4调试节点未被本轮停止或改动；未自动把两个RViz的目标接到车控。
