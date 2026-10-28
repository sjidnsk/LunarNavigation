# NAV-1CM-AUTO-20261029 验证记录

## Task1 共用任务层与可选 Action

公共 owner 为 integration/pure-planner-orin，专题 hk/nav-1cm-feasibility-20261026；本阶段基线211c7c41cdb14ad4eae0bb86650ee137cc40c42e。实现源码提交9a752d55，验收补强7a9e278f，SIGINT回归0f87c86b，修复后的已安装源码663c39bb9f9d2d0f3186ae802370835c4175306e。已提交待主线程独立审查、待合入，未移动owner。

- 纯任务状态机按UUID/代际处理替换、无效目标、取消与实测到达；Runtime为唯一session/coordinator写入线程。人工PoseStamped与Action经过同一序列队列，无效新请求保留旧目标。
- 到达要求当前目标xy/yaw及配置停稳容差、0.3秒保持和后续里程计测量。取消退休旧发布/在途身份，使用现有能力约束及当前地图认证制动，实测停稳后结束。有效替换不要求先停稳。
- 默认planner_node保持原接口；可选navigation_task_node复用NavigateToPose。有效替换结果为业务CANCELED/GOAL_REPLACED、ROS ABORTED；显式取消完成才是ROS CANCELED。
- 真实SIGINT验收发现pending任务列表清空早于rclpy GetResult回调发送，已新增RED回归并增加1秒有界排空窗口；不再丢失SHUTDOWN结果。

### 当前验证

Windows已有D:/conda_envs/lunar-explorer/python.exe：初始任务8 failed、Runtime3 failed、缺Action入口1 failed；实现后扩展纯任务/Runtime13 passed。默认Anaconda缺scipy的首次收集错误保留。

获准Ubuntu22.04/Humble x86_64、ROS_DOMAIN_ID=92、ROS_LOCALHOST_ONLY=1：

- 独立colcon构建lunar_nav1cm_planner成功。
- 完整one_cm套件649 passed、1 skipped，真实acados及三个native库启用。唯一skip是依赖缺失测试发现acados已经安装；运行输出中的两条QP求解失败由原套件覆盖，不是pytest失败。
- 修复前完整ROS包套件14 passed；修复后包CTest串行5/5通过，内含新增SIGINT回归，总计15个pytest用例。核心源码从7a9e278f到663c39bb未变，不重复运行已通过的核心套件。
- 实际Action客户端覆盖有效接受、反馈、无效目标保留旧任务、同位姿替换、迟到旧UUID取消、人工替换、无yaw、朝向等待、运动不成功、实测停稳后取消、到达保持及关闭。
- 默认真实native/MPCC节点smoke生成局部Path，正式全局日志同时满足planning_outcome=0、reason_code=PLAN_FOUND、has_reference=true。该smoke的里程计为脚本发布，只证明任务传输/状态机及真实planner接入，不能作为MPPI/独立plant运动闭环证据。
- 真实入口在活动Action期间SIGINT：客户端收到ROS ABORTED/outcome INTERNAL_ERROR/reason SHUTDOWN，exit_code=0。正常smoke所有worker自然退出；冷启动SIGINT按既有超时机制终止一个local worker，surviving_pids均为空。
- 38个安装Python文件与源码逐文件SHA256一致；实际导入均来自本任务install。native三个SHA及core动态链接路径见安装回执。

首次Humble合集因ROS安装的scripts实体包遮蔽仓库namespace而收集失败；关闭自动插件仍未解决。最终纯one_cm仅从PYTHONPATH剔除/opt/ros/humble/local/lib/python3.10/dist-packages，保留native/acados依赖；ROS包套件恢复原路径。所有测试从仓库根目录以python3 -m pytest执行。旧pytest不识别仓库pythonpath配置的warning被保留，未改测试/源码掩盖环境问题。

本地证据根：D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/exploration-integration-20261029/task1。远端证据根：/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-p3-interface-20261027/exploration-integration-20261029/task1。完整命令见final-r3-run.sh、sigint-fix-run.sh；回执见remote-receipts/full-pure-r3.txt、ctest-final.txt、native-smoke.json、sigint-result.json、installed-identity.json，失败日志保留。

NOT_RUN：Jazzy、Orin、实车、生产DDS/真实P3、真实MPPI+独立plant闭环、前沿/DRL接线及任何正式训练。任务范围内没有启动控制器、连接真实T5、改变训练或合并/推送/PR/upstream/删除。专题与本任务构建资源保留待审查/待授权处置。


### Task1 审查修复 round1：I1 取消/到达终态原子顺序

审查指出旧Runtime在输入锁内检查序列、解锁后修改任务终态；取消可以插入两者之间，使旧观测仍成功并永久忽略取消。0a78e337基线确定性RED为2 failed/4 passed，分别复现observe入口重入与真实输入线程在观测交接点入队；原始输出保存在task1/fix-round1/red-runtime.txt。

修复提交9f8da10e0825ef647b2cb63900e41cee4dd4c6cd：先生成不改变active/保持计时/轮次的不可变TaskObservation，再在同一个Condition临界区验输入序列、提交任务状态并写内部事件队列。临界区不执行外部回调；event_sink锁外交付，因此默认RLock重入不能绕过终态提交，sink也能等待另一输入线程。早于提交的取消使旧提案作废并在下tick进入真实制动/停稳取消；晚于终态提交的取消按旧取消处理，顺序明确。Runtime仍为session/task唯一写入者。

本轮当前结果：Windows纯任务+新旧Runtime 22 passed；Humble纯任务10 passed、Runtime12 passed，独立包构建通过，CTest 5/5（18用例）通过；默认真实native Action smoke及活动Action SIGINT通过。正常smoke自然退出无残留；冷启动SIGINT受控terminate一个local worker（PID757097），两类surviving_pids均为空，进程exit0。38个安装Python文件SHA匹配本轮源码，3个native SHA与前版相同。原649项数值/地图suite未重复，之前结果保留为历史，不声称本轮重新执行。pytest配置警告M1保持deferred，本轮未修改全仓配置或依赖。

本轮产物均位于原task1证据根的fix-round1子目录（独立build/install/log/acados-cache）。完整命令run.sh，日志remote-receipts/pure-task.txt、runtime.txt、ctest.txt、native-smoke.json、sigint-result.json、installed-identity.json。当前仅完成I1修复及实现者验证，仍待原审查者对I1和新增breakage进行scoped复审；Task2尚未解锁。NOT_RUN和脚本Odometry测量边界不变。
