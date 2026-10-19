# MPPI轮式控制器设计

用户已明确授权实现并评估MPPI。任务MPPI-20261018，owner dev/drl-exploration，基线f7db40fe。保留局部A*与PathReference，新增可切换的mppi后端，默认pure_pursuit不变。前后速度上限均0.2m/s。

MPPI核心：NumPy向量化差速/可原地转向运动模型，热启动控制序列、时间相关高斯扰动、softmax代价加权更新。预测中显式限制线/角加速度，含响应延迟。障碍与未知为硬不可行，地形代价、路径偏离、沿路进展、航向与平滑为软代价。加权控制须重新预测与认证，不直接发未经检查的加权结果。无解时输出停车并按停稳反馈请求重规划。

地图使用同一导航器GetPolicyMap服务的已膨胀fine states和costs；独立增量缓存校验epoch/base revision/几何。未知默认不可行，仅允许服务显式返回的起点unknown连接，不能覆盖blocked，运动只能离开起点连接。查询新鲜度与reference revision门槛失效停车。地图用局部稠密裁剪，碰撞认证对采样间运动作细分并包含额外margin与停车过程；不再次膨胀车体。

控制接收、session/revision/INVALIDATED与TrackingStatus保持兼容；仅incremental_reference支持mppi。最后到达仍需公共0.10m/0.05rad及实测停稳。采用内部更紧控制目标。保留对照后端，不修改正式服务或模型。

评估先核心算法反例与现有controller回归，再Humble ROS构建/节点测试，随后相同保存参考/地图与初态的闭环A/B（墙边、角点、窄通道、正倒车、死路和地图更新），记录成功、碰撞、时间、偏差、控制耗时及实际轨迹。再做有限场景真实局部规划端到端对照；不以合成测试代替200场策略验收，Orin/实车NOT_RUN。

实际默认参数为32条采样、40步×0.05秒、1次加权更新，温度0.15。连续运动认证使用每段恒定twist圆弧的切线三角形包络，并逐场景检查运输延迟与状态/计算等待；加权轨迹优先，无法认证时才选已认证候选。停车预测保留最长延迟前缀之后的完整刹停尾，并检查末端实测模型速度归零。该认证限定于实现的离散动力学/延迟情景，不代表真实平台的形式化安全保证。

ROS节点设置40ms计算预算，超时丢弃运动结果并停车；源里程计年龄最多50ms，并要求map→odom动态TF与里程计源时间戳一致。现阶段不支持以任意旧/静态TF替代该配对。地图有效期500ms。命令历史只在实际发布Twist时记录。平台超出时间模型时不能直接放宽时效阈值，需要重新验证响应模型。

启用：在隔离且已重建新增GetPolicyMap接口的Humble overlay中，启动原controller launch并设置`controller_backend:=mppi`、`policy_map_service:=实际原生地图服务`；mode必须incremental_reference。当前默认服务`/Car/T4/mapping/get_policy_map`，需与部署命名空间核对。新增诊断String主题`/Car/T4/control/mppi_diagnostics`记录计算时间/超时/选择方式。默认仍pure_pursuit，正式评测未切换。

最终配对结论：离线静态16场MPPI16/16完成、0碰撞，基线14/16、2碰撞；ROS固定参考6场MPPI1/6完成、0碰撞，基线5/6、1碰撞；原生地图/A*月表和洞穴各一次导航动作双方2/2成功。ROS末端收敛/固定参考跟踪仍失败，不能由离线结果推导上线可用。当前保留实验分支，不切换默认、不做正式部署。完整报告与实际轨迹位于上述任务artifacts目录。
