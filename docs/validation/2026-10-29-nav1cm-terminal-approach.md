# NAV1CM 末端平滑朝向：职责、修复与验证

任务 NAV-1CM-AUTO-20261029/TERMINAL-APPROACH；owner integration/pure-planner-orin；专题hk/nav-1cm-feasibility-20261026；修改前a63afae。本次用户确认nav1cm+局部MPCC+标准Path MPPI，接近终点平滑调整姿态，到点后原地完成剩余yaw，不能混淆各层职责。

## 行为与职责

产品改动仅scripts/navigation/one_cm/nav1cm/local_mpcc.py。

- 全局AdaptiveGlobalPlanner继续提供可通行几何路线，未增加终点朝向策略或修改地图。
- 局部MPCC在真实终点属于参考时，以剩余弧长和当前局部时域N×dt×vmax生成smoothstep朝向软目标，沿最短角差逐点混合行进车体yaw和最终车体yaw。默认20×0.2×0.2对应0.8m混合长度；它是局部软目标范围，不是硬性到达阈值。曲率、加速度、净空及独立整车扫掠认证保持原值。
- 行进对准spin全时域只对准行进方向，已开始末端转向且有残余角速度时保持当前yaw受限制动，不在最后节点混入最终yaw。moving向最终姿态平滑趋近；倒车只给行进车体方向加pi，不给最终yaw加pi。
- 模式保持只放行位于末段转向角区间、沿所选前/倒车方向仍有正推进投影的状态。0.05rad局部阶段滞回吸收轻微跟踪超调，不放宽最终到达角度容差；错误侧和非正推进仍保持原对准/制动边界。
- 到点后的阶段切换仍由coordinator实测欧氏位置容差决定，沿用terminal_candidate制动和有界原地转向，直到位置、朝向和实测停稳满足。没有让标准PathFollower或MPPI替规划器补目标，跟踪器产品文件未改。

大角度与短剩余距离可能无法在行进中完成全部yaw；局部运动约束限制能完成的部分，剩余姿态到点后完成。弧长投影已到头但横向位置仍未达的既有安全制动边界未扩展修复；不能把s到头或follower阶段当GOAL_REACHED。

## 当前验证

- 基线：21通过12真实求解器跳过。
- 新末段行为RED：6失败3通过；实现后GREEN。残余w回归RED1后修复；独立审查微超调(.85对.84375rad)回归RED2后加入局部滞回。
- 最终Windows定向：83通过13跳过，3.51s，覆盖MPCC、最终yaw、末段目标、同步/异步协调器。Windows完整one_cm回归曾583通过122跳过6失败；6失败均因未设置NAV1CM_GEOMETRY_LIB或NAV1CM_DISTANCE_FIELD_LIB，不能记成完整全绿。
- 台式x86_64真实acados最终定向：45通过3跳过，24.56s。三跳过为依赖缺失分支反测(实际已安装)和两项未启用native geometry对照。
- 其中三场真实MPCC→标准Path MPPI组件闭环全部通过：原45秒临界状态、正向接近、倒车接近。测试每场最多45s模拟时间，断言位置≤0.1m、yaw≤0.05rad、实测线/角速度≤0.01；并验证行进中v/w均非零、原地末端转向被采用、候选经过独立certify、跟踪无failure。独立三场日志3通过12.31s；最终组合日志含同三场。
- 独立代码设计审查发现并修正微超调模式切换问题；限定复审通过。审查没有替代实际数值测试。
- 本地待提交local_mpcc.py与台式source-r2 SHA256同为6f9c6a64c5d9cac76d444c56f989b5737ae91b22386bc469977a0e484c0adefe。

此组件测试使用完全已知空图和给定几何路线，真实运行MPCC、标准PathFollower/MPPI及独立候选认证；不运行全局进程/ROS节点/Action/DDS，不是原场景精确回放，也不是生产导航已GOAL_REACHED的声明。原44次全局成功和原45秒失败记录保持，不回填本次源码。

## 隔离与失败证据

本地产物：D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/terminal-approach-20261029。台式：/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-terminal-approach-20261029。source-r1/source-r2、各manifest、测试脚本、real-final.log与closed-r7.log保留。原安装、旧acados缓存、Actor、训练源与账本均未修改。

初次台式测试暴露pytest缺typing_extensions、ROS scripts包遮蔽源码命名空间、NumPy用户包ABI冲突；后续组合首次缺真实ROS消息类型，均在求解或组件执行前失败。旧失败日志保留。最终runner显式绑定本次source-r2命名空间，仅使用既有消息类型和隔离test-deps副本，不安装或修改共享依赖、不加载ROS节点。LF脚本替代Windows管道CRLF造成的shell失败；该失败不作为算法结果。

本轮早期Windows测试使用工具默认小型临时目录C:/Users/77634/AppData/Local/Temp/pytest-of-77634/pytest-71，用于pytest临时fixture，可在相关测试结束后删除；本轮未执行删除。后续定向临时目录已显式设为上述D盘artifacts/pytest-local-final。

## 处置

实现、回归和限定组件验证已完成，本地专题提交待owner合入；最终提交号以本记录所属提交为准。无push、PR、分支/worktree删除、训练恢复或默认入口切换。Jazzy运行、完整Humble ROS/DDS链、Orin、实车、障碍场景闭环及原45秒端到端复验均NOT_RUN。后续整合按公共导航→DRL→相关应用线逐批适配并验证，不把本次组件通过外推平台资格。
