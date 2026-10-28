# NAV1CM 与标准 Path 跟踪器集成收尾（2026-10-29）

任务 `NAV-1CM-20261026`，关联 `FOLLOWER-SWEPT-MAP-20261024`。执行者为 Codex 主 agent；Windows 独立专题开发，用户授权的 kai@10.63.250.129 Ubuntu 22.04.5 / ROS 2 Humble / x86_64 隔离验证。最终综合审查发现的两项 Important 已在唯一协调修复轮关闭；同一审查者最终 scoped 复审为 0 Critical / 0 Important / 0 新 Minor，允许落地。

## 代码与分支

| 角色 | 已验收并落入远程 owner 的代码记录 | 实际生产安装来源 |
| --- | --- | --- |
| 主集成线 integration/pure-planner-orin | f33eb35cc2b3e8d8d05610d56ef0aacc67e5edbb | 4940a6e0c2fb819b724e29e69d0cc711c6a3e0e3 / final-fix-main-r1 |
| 轮式应用线 app/orin-real-vehicle | 37d3c39eb103fe0aa9cfff290ddd337158ec98dc | 6905df2beef9db24bd06cfc1bdb1c6b7846b53c0 / final-fix-app-r7 |

表中验收记录相对对应生产安装只增加任务登记。本文及入口文档随后以收尾提交补入；具体文档提交见本文件 Git 历史和外部落地回执。收尾另清理主线一个测试行末的 CR，AST 不变，不改变产品文件或既有测试断言。

远程 owner 工作树为 `/mnt/data/WS/lunar-navigation/lunar-runtime/.worktrees/pure-planner-orin` 与 `orin-real-vehicle`。本地轮式 owner 指针同步；Windows 原 integration 工作树仍在 439178f2，含其他任务的未提交修改，未切换、覆盖或移动。新导航完整来源保存在本地 NAV 专题和已合入的远程主线；原 Windows 主工作区后续同步须先由其写入者保存其他任务。

原 follower 专题 83d6d10a41cfa743e4f10d796632472a8e3092a0 已进入主线祖先；轮式应用按模块适配公共来源，保留其自有 terrain core、P4 TF 与严格能力参数，未把整条主线覆盖到应用。三个专题分支及工作树均保留，状态为已合入、待授权回收；无 origin 推送、PR、upstream 设置或删除。既有 return/keyframe 等历史实验依原台账暂停，不因独有提交整支合入。

## 当前接口和职责

| 接口 | 约定 |
| --- | --- |
| 地图输入 | /Car/T3/mapping/grid_map，grid_map_msgs/GridMap，使用 elevation 和源图实际分辨率；地形处理输出轮式可通行状态 |
| 定位输入 | /Car/T3/localization/odometry，nav_msgs/Odometry；结合 TF 得到规划坐标系下车辆状态 |
| TF | 使用实际变换；支持静态、零时间戳及链式变换；轮式应用保留 /P4/input/map_to_odom 来源 |
| 目标输入 | /Car/T4/navigation/goal，geometry_msgs/PoseStamped，位置和朝向均为终端条件；通过碰撞检查可原地转向 |
| 全局输出 | /Car/T4/planning/global_route，nav_msgs/Path，用于观察整体绕行 |
| 局部输出 | /Car/T4/planning/local_path，nav_msgs/Path，用于跟踪；跟踪器不依赖旧 PathReference/地图服务/ACK 合同 |

全局允许未知，局部 allow_unknown 可配置、默认关闭。源图按其真实分辨率解释，局部始终 5 m × 5 m、500 × 500、1 cm；细分重采样不会创造上游不存在的物理精度。本轮未添加起步盲区补全。有限 intrinsic UNKNOWN 更新会撤销对应旧 FREE，保留 BLOCKED、真实 NaN 历史和跨源权限规则。

最新有效目标替换旧目标；无效新目标说明原因并保留旧目标。已接收目标后来被障碍阻断时保留该新目标，不退回历史目标。局部 Path 可携带计划时间；跟踪器按几何顺序和自身动力学重定时。规划软周期超 50 ms 不单独触发制动；实际碰撞、无可行制动等约束仍独立执行。

## 统一净空和入口

用户最新要求优先：MAIN 与 APP 的 wheel.yaml / wheel_orin.yaml 导航 minimum_clearance_m 均为 **0.2 m**。目标验证、全局包络、局部优化及扫掠、制动和跟踪器使用同一有效净空。实际车体仍为 1.182 m × 0.818 m；minimum_underbody_clearance 是另一项地形离地参数，未改成导航净空。APP 保留最大角速度 0.1 rad/s、终点航向容差 0.02 rad、nominal response 0.15 s。零净空原型与 0.4 m 参数化反例继续作为测试。

`bash scripts/orin/start_navigation.sh wheel false` 默认选择 nav1cm_wheel.launch.py，默认仅规划。APP 硬件源码 wrapper 同样选择新导航，未执行真实硬件 wrapper。显式旧入口、足式入口保持可用。启用跟踪器需显式 start_controller:=true；本次实际测试 command_topic 均使用私有主题，不连接车辆 T5。

## 验证证据和含义

| 范围 | 结果 | 边界 |
| --- | --- | --- |
| MAIN 最终安装三包 | 467 项检查通过；52 个安装 Python 文件 SHA 匹配 | Humble x86_64 |
| MAIN 完整 one_cm 源码测试 | 639 PASS / 1 条件跳过，约 71.63 s | 两条既有 QP status 3 输出保留，不声称每次求解成功 |
| APP 最终受影响三包 | 923 项检查通过；54 个安装 Python 文件 SHA 匹配 | 应用自有 core/native，无混用 MAIN core |
| APP 定向与 scalar 对照 | 48 项通过，4 个 GTest 对照通过 | 含有限 UNKNOWN、全依赖 halo、异常点、阈值及回退 |
| 统一 0.2 m 的短距离组合 | MAIN 45/48 s、APP 45/75 s 四场通过，12 角色正常退出 | 源图 5 cm / 局部 1 cm；独立车体扫掠净空下界分别 0.479828 / 0.481088 m；资格绑定原安装身份 |
| APP 最终 1 cm 输入闭环 | 完整 500² 二维缓坡、20 s reverse 通过，实测位移 0.704324 m、终点误差 0.095676 m、停稳；340 非空局部 Path；38 次完整地图 | 3 角色 exit 0，无残留；无障碍，因此不新增障碍净空验收 |

实际 planner 成功记录满足 planning_outcome=0、reason_code=PLAN_FOUND、has_reference=true。空 Path 注入仅为跟踪器 smoke，不作为正式规划成功。私有图、安装身份、异常独立否决、逐次 T5 publisher 否决及退出清理均有原始记录。

APP 原 5 m / 1 cm 输入 20.003 s 超时已闭合：最终原生二维缓坡约 0.095 s，微扰约 0.101 s；组合 worker 已测最慢孤立异常点约 1.799 s，保留真实 received_at，未放宽原 3 s 新鲜度。关闭实测 follower 0.0366 s / planner 0.0372 s。数值为这些实际样例的观测，不是任意地形、任意尺寸或 Orin 硬实时上限。三态 FREE 充分证书仅供 wheel bridge，完整 scalar 字段与原阈值保留，未证区域回退；APP 的数学与浮点域边界见其 docs/validation/2026-10-29-app-terrain-batch-proof.md。

## 剩余边界和协作

Orin、实车、真实 P3 流/rosbag、生产 DDS/车辆控制、真实 RViz 操作、公里级长时及冻结 DRL/足式重新验收均 NOT_RUN。MAIN 两套旧 C++ 测试只有相关语法/生命周期检查，未重新完整运行；APP 受影响完整 C++ 已执行。旧 QP status 3 属已记录非阻断数值诊断，不能解释为所有求解尝试成功。源码默认切换不等于设备部署或已启动车辆。

后续公共问题以 integration 为 owner，先专题验证再适配 APP；APP terrain 证明、依赖 ABI 与 P4 TF 不得直接被主线版本替换。同一任务一次只有一个源码写入者和一个实际 ROS 运行槽。当前三个专题及依赖、安装、失败证据均保留待授权回收，不清理其他任务目录。

本地外部证据根：D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/integration-20261028。关键文件：final-whole-branch-fix-report.md（SHA256 3316749690d1c7a80b76a7924d1ecff66888698e991683d49bd95b4fb01b8aa9）、final-scoped-re-review.md（SHA256 fd06e9d8d4116062b4cd99feab50eebdc6dd6aa1f833c0488a317c3016ec1f85）、owner-code-landing-receipt.json、最终文档落地回执、progress.md 的 22 项裁定清单。原 .4 失败、20 s 超时、r1..r8 及早期校验器失败均保留，不改写为通过。
