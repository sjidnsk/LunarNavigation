# 统一75cm精确碰撞距离

任务TCP-EXACT-COLLISION-20261103，基线ba88bde3，公共owner integration/pure-planner-orin，当前TCP专题承接，ROOT单写。

## 改动与职责

GridSnapshot附带来源原始障碍矩形并纳入摘要；允许UNKNOWN的生产路径精确查询矩形距离及解析梯度，不再经过障碍栅格半对角线与查询偏移重复扣减。无来源几何的纯栅格及禁止UNKNOWN兼容路径保留原行为，不能混称全模式已替换。全局线段工具移动到既有rectangle_distance模块，认证复用，接触仍拒绝；圆半径仍75cm，MPCC额外3mm删除。

连续认证检查圆中心的整条运动，纯旋转不因角点位移额外扩大圆；实际中心漂移后的位姿/平移轨迹仍检查。弯曲积分以二阶导数上界覆盖弦偏差（分段误差上界不超过10微米），不将其当配置净空。此检查保证模型轨迹，不保证未观测的打滑/漂移，P3延迟及预测误差仍是现场限制。

MPCC此前只有硬圆约束；现在复用已有global_soft_clearance_m/global_clearance_weight给局部软偏好，不新增硬门槛；模型指纹v5包含软参数，避免旧生成物误复用。软代价不能替代独立连续认证，线性化切弯仍须拒绝。

## 验证

新增临界距离/移动窗口/三层一致/直行与旋转/漂移与碰撞/摘要变化回归，RED后GREEN。本机完整导航1308通过109跳过；新增软代价测试后定向53通过1跳过。原生测试初次加载旧ABI失败，已按当前源码独立重建ABI3后全量通过，未覆盖旧库。

Humble隔离141通过；首轮打包漏benchmark_global.py导致29失败，补齐后141全通过。真实acados N80 dt0.2、75cm、soft0.5/weight1，在三帧P3地图重建的同一临界位置恢复求解，距离0.758252m，原地对齐候选通过连续认证。人工向西直线参考会进入障碍，前进候选仍被拒绝，不能将spin候选成功说成目标到达。新模型构建约13.97s，离线三轮求解时延见P4/tmp/exact-collision-20261103/real-result.json；未发目标/速度。

尚未正式部署，未做车辆闭环验收。tcp2地图统一组合验证继续本任务，不改P3/Env_X、不合并长期线。

## 产物

本机D:/CodexDownloads/lunar_navigation/exact-collision-20261103，远端/home/yanfa/P4/tmp/exact-collision-20261103。早期测试默认在C:/Users/77634/AppData/Local/Temp/pytest-of-77634/pytest-245及pytest-246生成可删除测试临时文件，测试结束后可清理；后续明确使用D盘basetemp，未自动删除。

## tcp2组合及性能复审（进行中）

接收tcp2最终来源bd5b893775c6e9039f4fa44c4d1f716561ca626c相对98fb6081的最终差异，三方应用保留本会话域57、相机/RViz、75cm精确碰撞、局部软代价；不覆盖正式运行。只删除其明确退役的两个legacy补图源码文件。组合本机1569通过29跳过，Humble新规划包构建成功，串行planner13/13、core21/21、ROS16/16通过；私有域174 DDS20→5→20cm通过。legacy长距离性能仍沿tcp2保留未通过项，不混作活动nav1cm验收。

真实64m/20cm三帧组合重建发现新的运行性能缺陷：P3MapStore.apply超过80秒CPU100%，SIGINT traceback定位_next_facts→update_evidence→accepted_spans逐项全扫描classes/candidates及_cut重复对象构造。已通知tcp2仅在其独立专题修复并交付，本会话负责真实数据留216测量；没有停用共享事实或用小地图掩盖问题。组合实图性能尚未验收，正式P4未切换。

独立精确碰撞批次使用原地图管线构建同一冻结几何，并用实际全局目标重规划：PLAN_FOUND（0.436s），三轮均一次spin求解并通过连续认证。80步纯圆心固定旋转曾重复检查160次相同圆，现在仅复用一次几何证据，状态/动力学逐步验证不减；新增漂移及动力学错误回归。再将MPCC各预测节点精确距离/梯度批量计算。三次样本中位：规划43.74→30.84ms，认证47.85→14.82ms；只是同主机小样本离线对照，不是车辆实时保证。原始数据留/home/yanfa/P4/tmp/exact-collision-20261103/exact-route-before-static.json、exact-route-before-batch.json、exact-route-result.json。曲线切弯及实际中心漂移仍必须过连续认证，未做真实车辆新目标测试。
