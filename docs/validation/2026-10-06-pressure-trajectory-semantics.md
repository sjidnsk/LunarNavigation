# 压力评测可视化的站位连线语义

用户发现蓝线呈直线且穿过障碍。核查确认：每个点来自初始化扫描后的实际姿态或动作结束的 `last_execution.actual_pose`，绘图直接连接相邻点；中途行驶与避障折线没有保存。

- 记录：`scripts/drl/region_contraction/evaluate.py:382,427`。
- 绘图：`scripts/drl/region_contraction/pressure_viewer.html` 的 `frame.trajectory` / `lineTo`。
- 当前产物没有逐 tick/odom、PathReference 坐标或 rosbag；NPZ 保存的是图观测。
- `ros_messages.py` 的有限姿态缓存仅在内存，未序列化。PathReference 回调保留状态与点数，未保留路径坐标。

累计实际路程另有来源：原环境 `plant.py:47` 每次物理积分累计 `abs(v) * dt`，动作记录保存前后累计差。这是平面运动弧长，计入倒车、转弯和加减速后的速度，不能解释为蓝色端点连线长度或三维地表里程。

碰撞由原环境 `plant.py:32` 根据含平台包络的真值 navigation M 检查物理运动跨格/角点，与绘图连线无关。连线穿过障碍不证明实际碰撞；缺少逐步位姿也不能重建已完成回合的避障路线。规划路径即使另行记录，也不等于实走轨迹。

本次只调整未纳入策略冻结身份的显示文件：图例改为“决策站位连线（非行驶轨迹）”，蓝线改虚线，并明确记录缺失边界。冻结 Actor、评测核心、导航、奖励、批次协议与 HEAD 均保持原身份；压力评测继续。

验证：JavaScript `node --check` 通过；内置浏览器实际刷新后核对图例与画面。显示文件 SHA256：`0dd376eb0b8aa0f53c034efdfe7af33ad4226194a6a19add8a9d942a52a0c03d`。

![修正后的站位连线图例](/home/kai/CodexDownloads/lunar_navigation/drl-integration-20261012/evidence/pressure-trajectory-clarification-20261006.jpg)

这份记录只验证显示语义修正，不是全部 40 回合完成或 A2 效果提升的验收证明。
