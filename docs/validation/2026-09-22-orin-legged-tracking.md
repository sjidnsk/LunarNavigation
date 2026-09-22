# Orin 足式低速前进／转向闭环验证

## 范围与实现

分支 `app/orin-legged-vehicle`，本次用户明确先完成低速前进／转向，再扩展横移，
并确认真实足式驱动尚未接入。复用现有 PathExecutor、PathReference、TrackingStatus，
不改变消息 schema，不启动第二套规划或控制链路。

- 新增按平台区分的能力映射，足式使用 yaw rate 与 linear acceleration 字段；轮式保留原映射。
  无效、缺失、非有限或非正能力值及平台不匹配拒绝启动。足式要求明确能力文件和版本化参考。
- `orin_legged_controller.yaml`：前进 0.1 m/s、角速度 0.15 rad/s、默认禁止倒车，零横向输出。
  制动参数来自初始控制配置，尚无实机测量；零命令不等于真实机体已经站稳。
- 增加独立 launch／Orin 启动脚本；导航入口仍不自动启动控制器。
- 保留前序用户配置：净空权重 2.0、起点盲区额外余量 3 m、360°／10 m。

## 首次闭环失败与修正

完整链路首次停在约 (0.519, 0.052, 1.504 rad)，没有返回导航成功。
`tracking-probe-trace2/trace.json` 显示同一 session 持续发布到 revision 265 的静止最终路径，
跟踪器均报告 COMPLETED。根因是足式局部规划器默认终点 yaw 容差约 0.1745 rad，
比导航要求 0.05 rad 宽。当前位置误差约 0.067 rad 被规划器接受，却不满足导航完成条件。

修正：RealSessionPorts 向足式规划器传入导航 yaw 容差的一半，即当前 0.025 rad；
跟踪配置改为 0.02 rad，合计 0.045 rad，保留 0.005 rad 余量。轮式规划器构造不变。
新增实际规划端回归，防止静止朝向在导航预算外仍被判可行。
离散 spin primitive 仍生效，不可达的严格朝向允许返回 NO_PATH，不能把该测试理解为
任意角度都能成功纠正。

审查修正了两处探针证据问题：COMPLETED 必须匹配 session UUID 和 revision；
取消停车必须收到取消后新发布的零 Twist，不能使用理想模型自己的命令超时停车代替。
早期追踪脚本遇到 UUID 序列化问题后，已将子进程清理放到记录写入之前，相关孤留的本次
验证进程已停止。最终运行正常退出，未留控制器进程。

## 可复核证据

产物根目录：`/home/kai/CodexDownloads/lunar_navigation/orin-legged-20260922/`。
仅使用本机 Jazzy 的独立 ros-build／ros-install；没有混用 Humble 构建产物。

| 验证 | 结果与证据 |
| --- | --- |
| 构建 | `legged-controller-build3.log`，导航与控制器两包成功 |
| 包内回归 | `legged-controller-final-tests.log`；导航 86 项、控制器 162 项通过，无失败；包含轮式回归、足式能力限幅、节点模式限制、ROS topic 理想闭环、版本失效、里程计／TF 超时停车 |
| 配置／接口 | `legged-controller-contracts.log`，43 项通过 |
| 外部接口检查 | `legged-controller-interfaces.log`，pure planner external interfaces: OK；使用本分支实际的 config/external_interfaces.yaml 及检查器 |
| 脚本／补丁 | bash -n、git diff --check 通过，中文文档按 UTF-8 读取检查 |
| 独立审查 | 配置、限幅、测试及终点容差修正复审，无阻断项；审查者未独立运行测试 |

完整端到端探针为 `tests/launch/legged_tracking_probe.py`：真实导航 Action → PathReference →
真实跟踪器 → Twist → 理想平面速度积分 → Odometry → TrackingStatus → Action 结果。
localhost 域 187，所有探针接口位于 `/legged_tracking_probe/`，平坦合成高程图，
无真实车控消费者。`tracking-probe-fixed/result.json` 与 `trace.json` 记录：

| 目标 | 结果 | 最终位置／朝向 |
| --- | --- | --- |
| (0.6, 0), yaw=π/2 | GOAL_REACHED，outcome=0，revision=2 | (0.518676, 0.052414), yaw=1.534226 |
| (0.6, 0.6), yaw=0 | GOAL_REACHED，outcome=0，revision=2（新 session） | (0.546851, 0.518814), yaw=-0.005630 |
| 行驶中取消第三目标 | CANCELED，并收到新的零速命令 | 已停止理想模型 |

两次到达均在 0.1 m／0.05 rad 内；1083 个状态样本、6 个参考消息，最大输出
0.1 m/s 与 0.15 rad/s。另一个确定步长的 ROS topic 单测验证直角路径停转及最终对齐。
复现命令见部署文档。探针使用现有安装 overlay，运行前须完成相应构建。

## 未验证与后续

- 真实足式驱动未接入：命令坐标系、使能／步态、watchdog、故障状态、零速站稳行为待确认。
- Humble、原生 Orin、真实 P3／DDS 网络、真实运动／制动／步态稳定性均 NOT_RUN。
- 没有横移控制、落脚规划或台阶／跨沟动作执行；地图能力上限不等于本次闭环实测能力。
- 旋转时的机身扫掠与真实障碍空间尚需目标机验证；合成平面模型不覆盖这些问题。
- 本次针对受影响包和接口验证；前序地形记录中的全仓历史失败及探索偶发断言仍保留，
  不宣称全仓回归全部通过。无远程部署、推送或合并。
