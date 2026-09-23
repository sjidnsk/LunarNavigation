# 足式 5 cm 配置与地形评估复核

## 范围与配置

用户要求只调整分辨率、复核地形评估，保持碰撞模型不变。
P3 原始文件不修改。P4 新增两份独立 P3 配置：

- `config/p3_hardware237_5cm.yaml`：实时传感器模式。
- `config/p3_hardware237_bag_5cm.yaml`：录包模式，保留原有时间与标定配置。

来源为 2026-09-23 只读抓取的 P3 配置，各文件头记录原始 SHA256。
逐键比较确认仅 `map_resolution: 0.05`、`map_window: 16.0` 两项有变化。
320×320=102400 格，低于 `max_query_cells: 250000`。
新配置需要显式传给 P3 的 `scripts/control.py start --profile`；原有
`start_dog.sh` / `replay_dog.sh` 不会自动选用它们。不得声称既有启动默认已切换。
必须使用新的输出目录，不复用 0.2 m 的高程数据库。

实时模式入口示例（传感器恢复且旧进程已正常停止后）：

```bash
cd /home/yanfa/P3/lidar_visual_fusion
export ROS_DOMAIN_ID=69 ROS_LOCALHOST_ONLY=0
export P3_VISUAL_TIMING_TOPIC=/fusion/lidar_metrics
export CYCLONEDDS_URI=file:///home/yanfa/P3/lidar_visual_fusion/config/cyclonedds_hardware.xml
source scripts/env.sh
/usr/bin/python3 scripts/control.py start --network \
  --profile /home/yanfa/P4/config/p3_hardware237_5cm.yaml \
  --output /home/yanfa/P4/deployment-evidence/p3-live-5cm
```

录包模式仍需沿用已验证的 dog 回放时间对齐链路，不能简单用普通 bag 播放
替代该链路。远端恢复后，需核对 replay_dog 的 profile 覆盖入口，再运行回放。

## 地形评估复核

保留现有生产算法、碰撞外接圆、能力和阈值，没有修改轮式配置。

| 项目 | 0.2 m | 0.05 m | 处理 |
|---|---|---|---|
| 背景窗口覆盖 | 1×1 m | 0.25×0.25 m | 保留 5×5，作为局部地形测量 |
| 背景窗口采样中心跨度 | 0.8 m | 0.2 m | 坡度计算已除以真实分辨率 |
| 中央残差/完整观测区 | 0.6×0.6 m | 0.15×0.15 m | 保留 3×3，不插值补足未知 |
| 最大坡度 | 30° | 30° | 不变 |
| 最大台阶 | 0.5 m | 0.5 m | 不变 |
| 支持带 | 0.0125 m | 0.0125 m | 米制阈值，不按格距缩放 |
| 支持邻接 | 相邻观测格 | 相邻观测格 | 不是完整足底落脚面积认证 |
| 硬膨胀半径 | 约 0.378 m | 约 0.378 m | 不变 |

5×5 是测量窗口，不是整机碰撞包络。完整机身周围仍由现有 FineCellEvaluator
遍历外接圆区域内的地形判定。当前证据不足以决定扩大背景窗口，因此本轮
只复核、不更改窗口或放宽支持要求。5 cm 下每格观测可能减少，坡度对高度
噪声更敏感；理想几何测试通过不代表真实点云下 UNKNOWN 比例或噪声表现通过。
P4 的 1 m 探索粗图保持现状，不代表已支持窄洞探索。

## 验证

- 本机 Jazzy，Release 核心包构建通过；18/18 CTest 目标通过。
- 增加 0.05/0.1/0.2 m 对照：29°/31°坡度、±0.49/0.51 m 台阶、
  单格缺测、孤立高点、±20 mm 高程区间。
- 支撑台阶对照新增 0.05 m。
- 完整派生地图+局部规划测试采用相同 6.2×6.2 m 物理场景、相同起终点、
  相同机身包络，分别验证可跨台阶、超限断崖、0.8 m 未知沟隙。
- 使用 P3 源码副本读取两份新配置，均成功生成 0.05 m、320×320 窗口。
- 证据：`/home/kai/CodexDownloads/lunar_navigation/legged-hires-20260923/`。
- Orin SSH 多次超时/No route to host：新配置远端部署、Humble 构建、
  5 cm 真 bag 回放、DDS 消息与性能、实车验证均 NOT_RUN。

下一步验证应记录真实消息分辨率/尺寸、P3 发布耗时/丢帧、P4 接收/派生耗时、
FREE/BLOCKED/UNKNOWN 比例，以及同一录包时刻的 0.2 m/0.05 m 地形对照。
不得由当前测试推出噪点已消除或导航闭环已通过。
