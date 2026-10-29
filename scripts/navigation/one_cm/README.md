# 原生 1 cm 轮式导航核心与 P3 Path 适配

本目录提供 1.182 × 0.818 m 轮式平台的全局几何参考、移动局部窗口 MPCC、整车轨迹认证及可选 ROS 适配。算法与离线基准不发布 ROS；旧隔离验证包使用 `/nav1cm_verification/*`，新 `lunar_nav1cm_planner` 接收标准 P3 GridMap/Odometry/TF/PoseStamped 并输出两条标准 Path，入口见[适配包说明](ros2/lunar_nav1cm_planner/README.md)。本目录不发布车辆命令，不补起步未知区，不证明实车部署就绪。速度、角速度和加速度边界来自 `WheelLimits`；所有地图保留原始 UNKNOWN。2026-10-26 的 Orin/隔离证据保留原范围，当前 P3 接口证据另行记录。

有界三进程 DDS 及独立 Path 运动验证见 [p3_path_validation](p3_path_validation/README.md)。该验证消费者仅由标准 Path 几何驱动并自行重定时，不导入生产认证或 verification_plant；重新检查自己的速度、加速度、制动和车体扫掠，不继承原路径时序的认证。

## 公共接口与职责

| 模块 | 接口与职责 |
| --- | --- |
| contracts | 不可变 WorldSnapshot、GridSnapshot、Route、LocalReference、Trajectory 与认证身份 |
| maps | `extract_window(world,state,allow_unknown)`：车体中心 5 × 5 m、500²、0.01 m；`build_distance_field(grid)` |
| field_native / distance_field_native.cpp | 显式可选的精确欧氏距离变换、保守净空和同定义梯度；只加速地图数学，不参与规划或认证 |
| global_planner | `AdaptiveGlobalPlanner.plan(...)`：全局允许未知、原始障碍目标拒绝；只 `0 + PLAN_FOUND + has_reference + route` 为规划成功 |
| global_worker | `GlobalWorker.submit(dict)` / `poll()` / `busy` / `close()`：spawn 单进程、至多一个在途；普通 dict/list 跨进程，父端 `restore_result` 重建不可变 Route |
| reference | `extract_reference(route,state,grid,previous)`：连续弧长关联，首次出窗截断；路线换版重新投影 |
| local_mpcc | `MpccPlanner(limits,build_dir).plan(...)`：真实 acados SQP-RTI，仅产候选，不认证 |
| certification | `certify(...,backend=...)` 与 `certificate.matches(...)`：矩形连续扫掠、运动约束、停车尾段及完整来源绑定 |
| seeds | `brake_candidate(...)`：受限制动，不能计为 MPCC 成功 |
| execution | `action_prefix(candidate,measured,.05)`：截取真实时间分段，停车后才补零控制；`execute_action(action,measured)` 逐段调用公开 `step` |
| coordinator | `NavigationCoordinator.tick(world,state,goal,now_s) -> CycleResult`：调度、版本、失败门禁与事件，不复制动力学数学 |
| fixtures / benchmark_navigation | 仅造输入 / 运行与记录证据，不参与算法行为 |

`CycleResult` 同时保留完整 `candidate` / `certificate` 和固定 50 ms 的 `action` / `action_certificate`。每个动作均在 tick 内按当前窗口重新认证并检查实测初态前五维（绝对容差 1e-6）。图更新使旧认证失效；路线来源 epoch 可旧于当前窗口，但候选与证书须绑定当前窗口。50 ms 是软性能预算，晚到的有效候选和动作继续执行；原候选或动作无效时重新生成并认证 brake。若连 brake 都失败，返回 `NO_CERTIFIED_ACTION`，离线实例失败终止，绝不把运动状态瞬时置零。

## 调度与版本

旧同步入口默认局部 20 Hz、全局 1 Hz、全局搜索预算 1 s。全局进程忙时合并待办原因为最新世界和目标，不累计请求；局部 poll 不等待 future。目标或地图已变化时拒收陈旧结果。首次等待也输出当前认证制动，计入原始周期。

同图周期重规划保留已接受参考的连续性，记录 `ROUTE_CONTINUITY_RETAINED`；仍完整记录新全局搜索结果。地图更新、目标变化或无进展触发的新路线会接受并重新投影。无进展检查包括位置、航向和速度变化，制动与转向不会仅因路线 s 未增长就立即视为卡死。数值失败不写地图、不把未知或 QP 失败升级为物理障碍。

地图模块的 `DistanceFieldCache` 只缓存一项几何，key 为完整可通行位图、shape、分辨率和未知策略。位图相同时可共享不可变距离与梯度数组，同时将场重绑定当前 epoch/origin；位图变化就重建。同步窗口中心等于本次实测车体中心；异步入口为未来接续初态中心，原始实测反馈与预测目标时间分别记录，实际执行段另行独立审计。窗口不量化或冻结。`SDF_SOURCE` 记录当前完整 grid digest 和是否复用；向 MPCC 提交的场身份和 digest 仍须匹配当前窗口。认证模块用 `prepare_certification(grid,backend=...)` 为同一 tick 构建一次危险格索引，完整候选、动作前缀及必要制动分别调用 `certify(...,context=...)`，仍独立完成扫掠和停车尾段检查。上下文不能跨完整图摘要或 backend 复用。

## 运行与证据

纯单元测试从仓库根目录运行 `python -m pytest tests/navigation/one_cm`。Windows 缺 acados/native 时对应真实求解器测试跳过，不能据此宣称真实组合通过。

在已准备的 Orin 环境，设置 `PYTHONPATH` 为源码根及 `scripts/navigation/one_cm`，设置 acados 的 `ACADOS_SOURCE_DIR`、`LD_LIBRARY_PATH`，并将 `OMP_NUM_THREADS`、`OPENBLAS_NUM_THREADS`、`MKL_NUM_THREADS` 设为 1。运行示例：

```bash
/home/yanfa/P4_veri/deps/solver-venv/bin/python scripts/navigation/one_cm/benchmark_navigation.py \
  --build-dir /home/yanfa/P4_veri/build/humble/mpcc \
  --geometry-lib /home/yanfa/P4_veri/build/humble/geometry/libnav1cm_geometry.so \
  --output-dir /home/yanfa/P4_veri/artifacts/navigation/UNIQUE-RUN
```

输出目录必须不存在。`--case` 可重复指定场景，`--pace` 将循环尽量对齐墙钟 50 ms；默认模拟每步固定 50 ms，墙钟为实际计算时间，初次等 worker 时有界让出 5 ms。每个慢周期均记录，动作由当前认证有效性决定。该同步离散模型没有真实实时调度器、求解期间的反馈推进或执行器延迟模型，不能由此推断异步连续实车执行通过。

逐场景 JSONL 保留初态/后态、完整候选、实际 action 全部分段控制、证书内容和 digest、地图/路线身份、每个事件、全局结果与 PID、关联误差、全部阶段计时和 RSS。单独 `*-global-tail.json` 保留场景结束时仍在途的全局结果，其排空时间不计入局部周期。SVG 包含全部已接受全局路线与完整实际执行路径；灰色为各地图 epoch 的障碍叠图。

`tick.timings_ms.total` 以动作选择完成快照为终点，包含地图、SDF、参考、seed/solve/candidate、认证索引准备、完整轨迹认证、实际前缀认证/身份门禁、IPC/调度与必要制动。已取消原 49/50 ms 时间制动门禁。基准另外测量 `tick_call_wall_ms`（包含函数返回开销）；`strict_50ms_pass` 只表示所有样本是否均在 50 ms 内，不能替代动作安全或到达验收。`timing_policy=soft_period_certified_action`，`late_certified_motion` 计数外层超过 50 ms 且独立审计有效的非制动动作，属于允许的软周期观测。

`field_cache_lookup` 是整个 cache.get 的时间，未命中时 `field_build` 等于这一时间；两者不可相加。`sdf` 包含 cache.get 和来源事件构造。`certification_prepare` 单列索引准备；`certification_identity`、`action_gate` 与必要的 `brake` 随后各自计时。

`planner_*` 是 `planner_wall` 内部子阶段，不可相加重复统计。基准的额外独立复核及公开 `step` 模型推进另记 `offline_audit_step_ms`，JSON/SVG 文件写入不属于算法周期。首次等待、冷启动后的首轮求解、所有失败和超时均纳入汇总，不挑选快样本。RSS 包含基准保留的完整证据对象，不等同纯规划器常驻内存。

目标到达按实际 xy 距离 ≤ 0.1 m 且 |v|、|w| ≤ 0.01 判定。`SAFE_STOP_NOT_ARRIVED` 仅说明经认证动作后停稳但未到达，仍是导航未成功，不证明全局无解或数值失败具有物理障碍原因。步数上限是 `STEP_LIMIT`，不能宣称安全停止。`SAFE_STOP_GOAL_REJECTED` 有当前地图拒绝目标证据。

默认全局净空半径 0.90 m；较紧通道观察单列 0.409 m。全局几何参考不保证矩形动态可跟随，1 km/40 km 蛇形几何实验不代表实际闭环跑完。正式性能和各场景结论以对应 `summary.json`、完整原始 JSONL 和 Task 4 报告为准，不能以单次路径图替代。

隔离 ROS 包已在 Jetson AGX Orin 的 Humble 主机构建、包测试并运行同机 DDS 三进程闭环；其余生产接口、跨机 DDS、Jazzy、rosbag、真实传感器/执行器、车辆与 P3 均为 NOT_RUN。各层证据分别记录，不能由同机消息闭环推断实车就绪。

## 可选 Humble 三进程隔离验证

`ros2/lunar_nav1cm_verification` 是独立 `ament_python` 包，fixture/navigator/probe 分别为独立 rclpy 进程。fixture 等真实 MPCC worker 预热与 probe READY 后才广播同机 `perf_counter` 原点并启动 50ms 车辆时钟；导航 20Hz timer 非阻塞 tick/poll，单 pending 仅限制完整 `TimedPlan` 发布。发布后必须同时收到该序列ACCEPTED ACK及实际feedback确认sequence，并等pending清空才可再发，支持跨topic乱序。完整计划发布频率和车辆/反馈/核心周期分别统计。worker 默认真实注入250ms，默认只测 straight 1.4m，40秒车辆时间上限；启动预热另限180秒。probe以首个有效fixture feedback原点独立计算运行期限，READY不能替代起钟证据。

线协议为 `std_msgs/String` 内 v1 JSON，含 session、producer PID、发送单调时钟与独立 ROS stamp；反馈实际 sample/id/vector/sequence 与未来对齐接续时间分开保留。world/goal 使用 transient-local，其余可靠 DDS；所有 topic 均在 `/nav1cm_verification/*`。单调时间不从 ROS wall stamp 推算。fixture 迟到跳槽单次发布并记录 skipped_slots，不补发历史反馈。三个角色均逐条流式保存完整原消息，回调不累计历史；退出后才离线审计实际车体连续扫掠、限值和停车尾段。

Windows 纯契约回归（工作树根）：

```powershell
$env:PYTHONPATH='D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/deps/windows'
python -m pytest tests/navigation/one_cm scripts/navigation/one_cm/ros2/lunar_nav1cm_verification/test -q
```

Humble 命令由主 agent 在目标主机执行。以下从**干净 bash 环境**开始，仅 source Humble 与本实验 install；`SOURCE_ROOT` 是保留仓库相对结构的同步根，`CORE_SRC` 指向其中算法目录，独立构建通道不接 production overlay：

```bash
source /opt/ros/humble/setup.bash
export SOURCE_ROOT=/home/yanfa/P4_veri/source/one_cm
export CORE_SRC="$SOURCE_ROOT/scripts/navigation/one_cm"
export RUN=/home/yanfa/P4_veri/humble/nav1cm_async_ros
export SDK=/home/yanfa/P4_veri/deps/acados-v040
export PYTHONPATH="$SOURCE_ROOT:$CORE_SRC:/usr/lib/python3/dist-packages:/home/yanfa/P4_veri/deps/solver-python:$SDK/interfaces/acados_template:${PYTHONPATH:-}"
export ACADOS_SOURCE_DIR="$SDK"
export LD_LIBRARY_PATH="$SDK/lib:${LD_LIBRARY_PATH:-}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
export ROS_HOME="$RUN/log/ros-home" ROS_LOG_DIR="$RUN/log/ros-logs"
mkdir -p "$RUN"
colcon --log-base "$RUN/log/build" build \
  --base-paths "$CORE_SRC/ros2/lunar_nav1cm_verification" \
  --build-base "$RUN/build" --install-base "$RUN/install" \
  --packages-select lunar_nav1cm_verification \
  --event-handlers console_direct+
source "$RUN/install/local_setup.bash"
colcon --log-base "$RUN/log/test" test \
  --base-paths "$CORE_SRC/ros2/lunar_nav1cm_verification" \
  --build-base "$RUN/build" --install-base "$RUN/install" \
  --packages-select lunar_nav1cm_verification \
  --event-handlers console_direct+
colcon test-result --test-result-base "$RUN/build" --verbose

# output-dir 必须尚不存在；每次运行使用唯一名字。
python3 -m lunar_nav1cm_verification.launcher \
  --output-dir /home/yanfa/P4_veri/artifacts/navigation-async-ros/straight250-UNIQUE \
  --build-dir /home/yanfa/P4_veri/build/humble/mpcc-async-ros \
  --geometry-lib /home/yanfa/P4_veri/build/humble/geometry/libnav1cm_geometry.so \
  --distance-field-lib /home/yanfa/P4_veri/build/humble/distance-field/libnav1cm_distance_field.so \
  --delay-ms 250 --timeout-s 40 --startup-timeout-s 180
```

`nav1cm_run` 是同一有界启动/探针总入口；退出码0必须同时满足 ARRIVED 停稳、integrity、非空完整执行审计、worker忙期间真实反馈与非零位移、三个不同PID的DDS接收见证、probe完整收齐反馈/计划/ACK、正常子进程及worker关闭。先核对sample0至最终反馈每个实际推进区间的segments连续唯一、时间/向量连接，再调用动作认证；末反馈停车认证不能弥补缺段。离线probe按channel/producer/session/ID对整个信封规范化SHA256比对，检查发送/接收身份唯一和单调时钟，拒绝同ID不同控制或source向量。缺数据或认证错误返回1。单独 `nav1cm_probe` 仅作为在线消息记录角色，其进程退出码只表示记录器正常关闭；验收以总入口退出码和 `probe-summary.json` 为准。

保存 `fixture/navigator/probe.jsonl`、三个 console.log、每角色 status.json、launcher.json、execution-audit.json 与 probe-summary.json。原始流含完整 source/native SHA、冷启动与worker PID、完整控制计划、实际分段轨迹、反馈、ACK、每 tick 事件与分阶段计时。源码与native身份由每角色独立记录。freshness拒收分列，身份/预测/接续错误仍fatal；关闭时仍在途请求单列，不能借尾部未完成请求宣称忙时连续性。

到达或上限后 launcher 只对自己创建并保存的三个进程句柄发SIGINT，宽限8秒，随后各句柄terminate，再2秒后必要时kill；导航自身先通过core有界关闭两组worker，记录 surviving_pids。外部中断可对该 launcher 发送SIGINT（前台Ctrl-C）；不扫描系统进程、不使用pkill。2026-10-27 主 agent 已完成 Humble 构建及 4 项包测试，真实 straight250ms 三进程到达停稳、动作审计、消息收齐与关闭检查通过。运行在 P4_veri/artifacts/navigation/async-humble-straight250-20261027-final；执行回调 P95 37.51ms、max64.19ms。目标 20Hz，实际反馈19.80Hz、未跳槽，完整实际时间段112条均审计有效，不宣称硬实时。Jazzy、跨机 DDS、rosbag、实车、传感器/执行器和生产接入未验证。

## 经用户授权的独立高速度档

`--speed-profile fast` 设置 vmax=0.5 m/s、wmax=0.5 rad/s；原档默认仍为 0.2/0.1。车体、a/alpha=0.5、曲率上限 1.0、窗口 5 m/500²/1 cm 与认证精度均不变。`MpccPlanner(..., n=20, dt=.1)` 使用 2 s 时域，默认构造仍为 20×0.2=4 s。OCP、初始化、重积分和候选时间统一使用实例 dt；模型名包含 N/dt/limits 指纹。

构造参数 `build_dir` 保持兼容，但语义为可共享构建根目录，保存为实例 `build_root`；实际实例 `build_dir = build_root / model_fingerprint`。JSON 为 `<root>/<fingerprint>/<mode>.json`，生成源码/构建文件为 `<root>/<fingerprint>/<mode>/`，不同 N/dt/limits 共享同一根目录也互不覆盖。基准额外在根目录下分 baseline/fast，再由 planner 按指纹隔离。

高档采用 2 s 时域的窗口估算：0.5×2 + 0.5²/(2×0.5) + 三圆外包约 0.848 = 2.098 m < 窗口半宽 2.5 m；实际轨迹仍必须通过完整认证，该估算不代替转向/障碍证明。实际执行周期仍为 .05 s。高档增障及目标变障提前至第 40 步（2 s），原档保留第 100 步，保证更新发生在足够车身与制动余量外。每例记录实际 v/w 峰值与时域，提速不能作为性能或安全验收通过的替代。

## 可选原生距离场

默认保留 SciPy 参考实现。`NativeDistanceField(path)` 显式加载单独动态库，检查 ABI 并记录绝对路径和 SHA256；`build_distance_field(grid,backend=...)`、`DistanceFieldCache(backend=...)` 和 coordinator 的 `field_backend` 可注入该后端。基准使用 `--distance-field-lib /绝对路径/libnav1cm_distance_field.so`，汇总记录 `distance_field_backend`、库路径和 SHA256。未提供参数时运行 SciPy，不隐式加载或安装库。

原生实现采用 O(HW) 精确可分离平方距离变换，随后执行与 SciPy 路径相同的半格对角线减项、窗口边界裁限和梯度定义。局部原始格、保守净空语义和认证精度不变。构建与逐点对照示例（从源码根目录运行，输出放验证目录）：

```bash
mkdir -p /home/yanfa/P4_veri/build/humble/distance-field
g++ -O3 -std=c++17 -shared -fPIC \
  scripts/navigation/one_cm/nav1cm/distance_field_native.cpp \
  -o /home/yanfa/P4_veri/build/humble/distance-field/libnav1cm_distance_field.so
NAV1CM_DISTANCE_FIELD_LIB=/home/yanfa/P4_veri/build/humble/distance-field/libnav1cm_distance_field.so \
  python -m pytest tests/navigation/one_cm/test_native_distance_field.py -q
```

对照覆盖不同尺寸、500²、随机稠密/稀疏、细墙、全部自由/占据与未知两策略；真实后端对照必须显式启用库。源码静态检查或跳过该项不能代替 Orin 数值与性能结果。

## 当前异步核心与验证层

AsyncNavigationCoordinator 默认执行 20 Hz、局部请求最高 10 Hz、全局 1 Hz，LocalWorker 单在途。旧离线/隔离 JSON 验证入口保留 `TrajectoryBuffer` 的严格 elapsed 控制裁剪和 verification_plant 未来接续协议；该历史验证层不属于新 P3 适配包的运行依赖。

当前 P3 标准 Path 模式使用 `ReferenceTrajectoryBuffer`：通过当前图认证的新 worker 结果才安装为固定 canonical source，普通 tick 只按实测位置/朝向单调消费源的物理进度，从真实反馈派生受限接续并重认证。bridge、派生轨迹及新的参考时间不回写 source、不重置源余量；bridge 和制动尾段花费同一份有限弧长/转角预算，提前留一个反应周期及实际刹停余量。暂停不消费几何，source 不采用墙钟 TTL。worker 忙时继续发布当前图下安全的旧参考，不因超过 50 ms 单独清空；来源耗尽或安全条件不满足仍受限停车。下游可用 Path 几何自行重定时，并负责自己的运动限制与扫掠检查。

2026-10-26 原阶段完整回归：Windows252passed/48skipped；Orin原生/真实acados与native库299passed/1skipped。Orin测试末尾有两条acados QP status3诊断，测试本身全部通过，不表示每次求解尝试都成功。全仓Windows测试另有70项范围外依赖收集错误，未宣称全仓通过。完整证据、原始失败与身份见任务外部artifacts/navigation/async-execution-20261026/REPORT.md；源码未提交、合并或推送。该记录不证明当前 P3 / fixed-source 版本在 Orin 上通过。

## 最终证据门禁（2026-10-27）

两个异步入口现在通过无 ROS 的 `nav1cm.verification_evidence` 保真保存每个 TICK 的
`global_result`（含 Route、NumPy 转换）、`current_goal`、`current_epoch` 和活动
`route_version`。失败、陈旧和周期重算结果同样保留；不会根据 ROUTE_ACCEPTED 或到达补造
`planning_outcome/reason_code/has_reference`。正式证明要求三字段为 0/PLAN_FOUND/true，且此前
GLOBAL_SUBMITTED 对应的 goal/epoch/version、当前调用输入、结果内 route 和同 tick 的
ROUTE_ACCEPTED 身份一致。ROUTE_CONTINUITY_RETAINED 的 computed_version 单独核对，不能
计作新接受路线。ROS probe 还必须收到与 navigator 相同的含结果完整信封（规范化 SHA256），
`formal_global_probe_delivery` 与 `formal_global_evidence.pass` 均进入严格总验收。

原生 `cases[case].pass` 和 ROS 顶层 `pass` 是总验收；`navigation_arrived` 和 `motion_outcome`
独立描述末态。`formal_global_evidence.status` 为 VALID、NOT_RECORDED 或
NO_VALID_ACCEPTED_PROOF。旧异步 raw 没记录三字段时为 NOT_RECORDED，总验收 false；旧 summary
和原执行身份不改写。上文 R2 的实测到达、审计和 DDS 结果保留其当时边界，不能据此宣称新正式
全局证据门禁通过。

`execution_coverage(rows, audit=None, *, initial_anchor=None)` 是原生/ROS 共用检查。
不给 audit 时只查段链结构；正式 `session_outcome` 始终给实际审计列表，逐正时长段匹配唯一
`(start_s, end_s, source)` 且 valid=true 的动作审计。时间缺口、重复、向量断开、漏尾段或仅
TERMINAL_STATE 均不能通过。`structure_complete` 区分结构与动作审计失败；对应状态为
FAILED_EXECUTION_COVERAGE / FAILED_EXECUTION_AUDIT，原有不相关执行失败仍保留。
只有完整覆盖与审计成立，`all_executed_actions_audited` 和 `continuity_pass` 才可为 true。

首帧优先真实 sample_s=0 的 READY/FEEDBACK。新 logical 模式显式记录初态 READY；历史确实
未发初态时，调用者必须显式提供配置锚点，禁止猜末态或插入伪造历史消息：

```python
from benchmark_async_navigation import reanalyze_session
anchor = {
    'sample_s': 0.,
    'vector': [start.x, start.y, start.yaw, start.v, start.w, 0.],
    'source': 'configured scenario.start; not emitted feedback',
}
result = reanalyze_session(
    identity_bytes, raw_jsonl_bytes, audit_bytes, original_summary_bytes, case,
    goal=explicit_goal, analysis_sha256=analysis_source_sha, initial_anchor=anchor)
# reanalyze_session 无文件 I/O。调用者 exclusive 创建新分析文件，绝不写回原 summary。
with new_path.open('x', encoding='utf-8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2)
```

返回值保留 `identity`、完整 `original_summary`、`original_status` 及原四项输入 bytes 的
`input_sha256`；新分析单列 `analysis_sha256`、`analysis_initial_anchor`、显式目标来源与
`execution_rerun=false`。analysis SHA 应覆盖 benchmark 与 verification_evidence 两个实际
分析源。新分析里 `execution_coverage.initial_anchor_source` 标明实际首帧或配置锚点来源。
只有结构/动作覆盖的历史重新判定不等于重新执行长场景。

最终证据修复后的短原生与Humble250ms均严格pass=true：正式全局三字段与请求/接受身份一致、完整动作覆盖及逐段审计通过，ROS探针收到对应全局结果原始消息，退出通过。旧四场raw重分析仍到达/完整覆盖/动作审计通过，但正式全局字段为NOT_RECORDED，因此严格新pass=false；不能从到达补造旧全局结果。源与报告快照保存在同任务D盘artifacts/navigation/async-execution-20261026。
