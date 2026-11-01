# 2厘米位置目标：首轮验证与末段方向冲突

状态：2cm收敛及用户选定的末段方向修复已实现，r20六目标完整闭环通过。r21追加十目标运行按用户要求中断，不再追加验证。基线为9ef6f3e，工作分支hk/drl-integration-20261029，长期线未移动。

## 位置收敛与公共验收分离

原r17/r18共16次自然到达均在进入10cm范围后停止平移：首个FINAL_ALIGN参考中心与最终位置相同。用户确认保留10cm公共验收上限，内部按2cm收敛。

本轮由导航协调器统一计算有效位置目标，供同步/异步末段切换、制动终点与任务完成使用；通过既有ExecutionReference位置容差传给标准Path跟踪器。不增加消息或身份实体，不改变全局路径目标、控制代价、进展门槛或训练。

## 验证结果

- 定向位置目标回归先RED后GREEN；隔离Humble规划器/控制器包最终571项测试通过，0失败/错误/跳过。
- 原生几何组件定向测试35通过、3跳过、1项源配置检查未运行；Windows全量纯导航测试中的6项缺原生库失败在原生组件环境补验，不宣称Windows全量通过。
- r19固定Actor推理运行预算600秒，要求6个自然到达，实际4个自然到达、5次决策；第5个未完成。完整闭环判定FAIL，取消/复位完整验收未完成。
- 第1个自然到达位置误差1.935cm；前4个均满足2cm质量目标。第5个最终位置误差2.528cm，速度为0；无碰撞，冻结Actor/配置/平台输入不变。

## 第5目标的原因证据

目标为(3.5, 1.899999976, 0.785398163)，停滞姿态为(3.474854625, 1.897390057, 2.596025261)。全局路径仍有效；实际执行中出现围绕当前位置的微小旋转参考，位置没有继续收敛。

以该实际姿态、目标及相同车辆限制，在独立空闲地图和全新warm start下进行真实acados组件复现：

- 局部模式选择reverse，倒车对应车体行进朝向3.245rad。
- 末段软朝向却要求车头接近0.793rad。
- 该软朝向下，按倒车速度行驶在目标方向的投影为-0.772，即反而远离目标。
- nominal/left/right三组求解状态均为0，但三组候选均被NO_PROGRESS拒绝。

这是受控组件复现，不是原地图及原warm start的精确重放。当前证据确认模式选择和最终姿态插值存在方向不相容；不能据此声称已解释所有实时缓冲更新细节。

用户随后明确选择“按最终姿态选择相容的末段前进/倒车方向”，已据此实施：局部MPCC在末段按最终车体朝向在路线上的投影选择方向，模式选择和旋转对准共用同一决策；需要换向时仍先制动。正交时保留原方向偏好。全局规划、跟踪器及认证职责不变，无新消息或身份实体。

## 方向修复后的结果

- 新增真实停滞姿态、相容方向及换向安全用例，旧实现4失败，修复后通过；追加正交、跨π与距离边界。方向及推理启动合同27项通过。独立只读审查无阻断。
- 本机导航模块628通过、126跳过、6失败；失败均为缺少NAV1CM_GEOMETRY_LIB或NAV1CM_DISTANCE_FIELD_LIB，分别为test_native_and_python_clearance_and_braking_agree的两个margin、test_euclidean_margin_accepts_full_rotation_and_braking的native分支、test_corner_distance_checks_both_sides_of_euclidean_margin的两个native margin及test_native_and_reference_support_agree。新隔离原生环境覆盖这些用例的定向运行57通过、5跳过、1项未上传源配置检查不运行；Humble规划器/控制器包571项通过。
- 新隔离源码49项SHA核对一致；实际安装local_mpcc.py与本地SHA均为1179dd44d633aa8f3d1544b592291922f47164b725d0c18c484a092842bd7b40。原r19目录与原安装未覆盖。
- r19实际停滞姿态的受控原生组件探针从三组NO_PROGRESS变为spin对准候选；候选尚未认证，不能替代完整闭环证据。
- r20完整验收器返回ok=true、errors=[]、completed_goals=6、decisions=8，包含2cm质量门槛、取消停稳、复位及再次取消和新帧检查。第2/3/5目标位置误差分别约1.996/1.966/1.992cm。目标序列由Actor在线选择，与r19不完全相同，不能称为同一序列精确重放。
- r21原拟追加10目标；用户要求“不用验证了”后已向准确父进程发中断，父进程及plant退出，记录KeyboardInterrupt、1个自然到达/2次决策。处置USER_STOPPED，未完成完整取消复位验收，不算PASS，也不归为算法失败。不继续下载/独立复算/可视化更新或启动测试。

## 证据位置与边界

仓库外本地证据：D:/CodexDownloads/lunar_navigation/git-preservation-20261029/execution-tests/position-target-r19/，包含verdict、summary、events、declaration、plant-summary与stalled-pose-component-probe.json。包测试日志为同级position-target-build-r1b.log。

台式隔离运行目录：/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-drl-inference-closure-20261029/run-r19-position-2cm/。

方向修复源为台式同一T目录下source-r4/position-target-r2，安装为execution-position-target-r2/install；r20/r21目录分别为R下run-r20-terminal-direction及run-r21-terminal-direction-long（T/R前缀沿上述两条绝对路径）。原始运行证据保留台式。停止请求后，自动审批拒绝了排队的r20下载/独立复算准备命令，未执行该命令。

本轮为合成平面场景、Humble/真实acados与MPPI组件、固定Actor推理；无训练。Isaac、Orin、实车、真实坡面均NOT_RUN。r20完整验收通过不外推这些层级，未晋级主线或应用线。本机导航回归由pytest产生的小型临时文件位于C:/Users/77634/AppData/Local/Temp/pytest-of-77634/pytest-72/，用途为测试fixture，测试结束后可手动删除；后续定向测试临时目录已显式转到D盘。
