# nav1cm参数生效一致性

NAV-STARTUP-PARAMETERS-20261031；基线f2342e9，复用hk/p3-pose-wait-20261030/main-worktree，ROOT单写入。仅nav1cm规划器及任务Action入口，未扩展为全仓库参数热更新。

## 原因与修复

wheel_config、WheelLimits同名覆盖、规划周期/预算、时域、目标/停稳合同、地图策略、话题/QoS和执行反馈参数都只在构造期建立运行对象。旧参数服务却接受运行时修改：真实Humble SetParameters与混合原子修改返回成功，而factory接收到的运行能力不变。

先保留既有启动覆盖与能力解析，按原下限收敛minimum_clearance_m及clearance_radius_m并原子回写，随后在资源factory调用前以现有ParameterDescriptor冻结本节点应用参数。新描述符显式保留原类型；action_name声明时即只读。描述说明restart required，运行set明确失败，不新增管理器、热更新回调、参数实体或运动安全门禁。数值默认、能力上限、导航完成条件和定位判活不变。

ROS自身use_sim_time保留原生更新路径，专项实测True/False两次设置都成功。startup_parameters日志记录最终声明值，Action名另行记录；日志不是运行健康或导航成功回执。参数分组、改与不改的影响已写入包README“参数生效边界”。Humble参数描述符及只读语义核对官方源码：https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/node.py 。

## 真实Humble与本机证据

台式KAI/x86_64、Ubuntu ROS2 Humble，独立根目录：
/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-terminal-approach-20261029/startup-parameters-20261031/

仅上传必要源码、测试、wheel.yaml及测试依赖p3_input_timing.py；不上传模型、设计文档或运行数据。final manifest-green-r3.json逐项核对141文件SHA256。只读复用既有SDK环境和execution-state-r2消息overlay；本轮core和planner在本目录build/install重新构建，未修改原安装。ROS_DOMAIN_ID=92、ROS_LOCALHOST_ONLY=1，测试订阅/Path/Action使用私有端点，无车辆命令发布器。

最终结果：

- 核心依赖包Release构建通过（其BUILD_TESTING=OFF，未声称core测试通过）；受影响lunar_nav1cm_planner包BUILD_TESTING=ON构建通过。
- 包内10组CTest全部通过；xunit实际60项pytest、0失败/错误/跳过。colcon汇总70包含10组包装结果，不能写成70个独立pytest。
- 显式预载本轮install中的node.py/task_node.py，逐字节与本轮source核对一致，再跑5项真实rclpy参数专项全部通过。与上面60项有重叠，不叠加成65项覆盖。
- 专项覆盖启动能力/周期/topic/Action覆盖，低净空请求收敛后的回读，所有本节点应用参数只读与set拒绝、混合原子修改无部分生效、跨节点SetParameters服务返回失败，以及use_sim_time独立成功更新。
- 本机配置/启动/执行合同5文件91通过、2跳过、10未选；首次扩大选择额外触发3项未配置NAV1CM_GEOMETRY_LIB的既有环境失败，未改产品来绕过。最后选择边界显式排除与本任务无关的native/优化器/欧氏净空角点组。

red-r4.log在旧产品代码上3行为失败、2通过，证明假成功及只读缺失。此前red/r2/r3日志保留，属于ROS环境nounset、测试半车宽前置值和Humble测试API纠正，不能计为产品RED。初次green构建通过，但真实测试揭示新描述符缺type导致启动拒绝，已显式继承type修复；另6项包测试因漏传p3_input_timing.py失败，补必要工具后全部通过。所有失败证据保留，不覆盖；未删检查或放宽断言。

最终原始证据build-green-r3.log、installed-r3.log、manifest-green-r3.json、verification-summary.json及早期日志保留远端根目录；本地副本与运行脚本位于D:/CodexDownloads/lunar_navigation/startup-parameters-20261031/。本轮只验证参数接口、启动/关闭、Action/消息和调度回归；规划worker在节点测试中隔离，不能当作真实acados导航闭环。TCP/P3真实输入运动、Orin/实车、探索/DRL闭环均NOT_RUN。

## 处置

独立只读复审未发现新增阻断；建议补的use_sim_time成功更新已纳入最终Humble重跑。源码/测试/文档及登记验证后本地提交待合入，不push/merge、不切换远端运行安装，不触碰TCP机其他目录。F07探索等待归因与F12地图统一仍待讨论，本轮未实施。
