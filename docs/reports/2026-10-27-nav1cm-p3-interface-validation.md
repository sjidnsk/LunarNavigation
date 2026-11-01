# P3 输入与标准 Path 接口实施、验证记录

任务 NAV-1CM-20261026；接口实施始于 2026-10-27，最终运行与收尾跨至 2026-10-28（Asia/Shanghai）。本阶段实现、验证和全部独立复审已完成：Task1–4 规格与质量均 PASS，监督门禁P2经窄规格及原QA复审关闭，整体独立复审 PASS。处置为暂停保留，本阶段完成，源码未提交，待用户审阅及后续提交/合入授权。

## 结论及边界

已实现原生 GridMap 分辨率输入、固定 1 cm / 500×500 局部计算、完整位置/yaw 目标及两条标准 `nav_msgs/Path`。最终生产 r6 在 KAI x86_64 / Ubuntu 22.04.5 / ROS 2 Humble 上，原始六个有界场景全部门禁通过；生产r6及原运行验证器版本的完整核心回归 **537 passed / 1 skipped，76.86 s**，新隔离 Release 包构建及包测试汇总 **10 tests，0 errors/failures/skipped**。之后质量审查发现监督器可漏判提前退出，已加强运行/离线失败门禁并增加纯测试，受影响 **16 passed**；对原完整raw更严评分仍六场PASS，没有重新运行DDS或完整core。P2已在独立窄规格及原QA复审后关闭，整体独立复审亦PASS。

六场为五场三进程验证加一场双进程标准 Path 注入 smoke，共 17 个外部角色进程，全部退出 0、无升级终止、进程组残留 `[]`。实际 planner 的内部 worker 另外由关闭回执证明无强制终止、无存活 worker。所有 DDS 场景地图为 6 m 或 8 m，不跑 1 km 运动，不启动生产车辆控制。

输入为生成高程；下游为独立数学模型，只读收到的 Path 几何并自行重定时。它没有读取内部候选、controls 或 Goal 来控制运动，不传送到路径点，不把平均速度当末速，不导入生产 certify 或 verification_plant。因此本次证明的是合成标准消息接入和有界模型跟踪，不是实车验收、原参考时序精确执行或原时序认证的继承。

| 验证层 | 本次状态 |
| --- | --- |
| 源码、无 ROS 会话、真实 native / acados / CasADi | PASS；生产r6及原运行验证器完整回归537/1；后补监督门禁受影响纯测试16项另测 |
| Humble x86_64、本机隔离域 DDS、实际 nav_msgs / grid_map_msgs | PASS；五场实际 planner 与独立消费者 |
| 独立短运动、限速/加速度/曲率、矩形 SAT、停车 | PASS；平图前进/倒车与末端转向，含独立碰角校准 |
| 真实上游 P3 数据流、真实 rosbag、跨机 DDS | NOT_RUN |
| 当前 r6 源码在 Orin / aarch64、当前源码 Jazzy | NOT_RUN |
| 实车、执行器、轮地物理、起步盲区补全、1 km 运动 | NOT_RUN |

2026-10-26 的 Orin 299/1、旧异步隔离 JSON/plant 结果、旧正式证据重分析均保留原身份与范围，不算作当前 P3/fixed-source 版本在 Orin 上通过；见原任务外部 `artifacts/navigation/async-execution-20261026/REPORT.md`。

## 最终实现及责任

| 层 | 实现与责任 |
| --- | --- |
| 地图 | `P3MapStore` 在消息真实几何/分辨率下调用既有 PersistentElevationMap 与轮式地形 evaluator；独立 native 桥接复用算法；再保守转为厘米快照，UNKNOWN 不因重采样消失 |
| 目标 | 完整 `(x,y,yaw)`，有效新目标替换；障碍内新目标拒绝并保留旧目标；有效但无路仍保留；代际隔离旧结果；位置/yaw/停稳同时成立才到达 |
| 运行 | ROS 回调只交接输入；地图线程与 tick 线程分离；真实 LocalWorker / GlobalWorker 单在途；不依赖 DRL、验证 plant 或车辆命令 |
| 标准 Path 参考缓冲 | `ReferenceTrajectoryBuffer` 固定 canonical source，按实测位置/yaw 单调消费物理进度；认证通过的新 worker 结果才安装 source，普通 tick 只 update derived |
| 有限余量 | bridge、源控制和停车尾段共同花费固定弧长/转角预算，留反应周期及实际制动余量；derived/bridge 不回写来源、不充值；暂停不消费几何，不设墙钟 TTL |
| 旧严格执行入口 | 原 `TrajectoryBuffer` 保留 strict elapsed 控制裁剪；不把它作为标准 Path 下游必须精确跟时序的要求 |
| 消息转换 | 保留倒车车体 yaw 与同 xy 转向点；local 首点 stamp=t0，其后 `max(t0+round(ti×1e9), previous+1ns)`；正常可表示时间不变，纳秒碰撞最小顺序化，不改轨迹或 digest |

默认轮式限制：车体 1.182×0.818 m，双向 0.2 m/s，角速度 1.0 rad/s，线/角加速度 0.5，行进曲率 1.0；执行 20 Hz、local 请求最高 10 Hz、global 1 Hz。能力来源为安装 `wheel.yaml → WheelLimits → 显式同名 ROS 参数`。接受零/缺省 TF stamp、空目标 frame 和可归一化四元数，不增加固定源分辨率或严格同步门槛。

输入及输出名称、类型、构建和启动说明见[规划包说明](../../scripts/navigation/one_cm/ros2/lunar_nav1cm_planner/README.md)；有界验证实现见[验证入口](../../scripts/navigation/one_cm/p3_path_validation/README.md)。

## 最终六场原始证据

远端唯一写入根：`/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-p3-interface-20261027`，下文记为 ROOT。SSH 入口 `kai-10-63-250-129`，实测主机名 KAI；Python 3.10，acados v0.4.0 / CasADi 3.6.6。ROS_DOMAIN_ID=91、ROS_LOCALHOST_ONLY=1、PYTHONNOUSERSITE=1；build/install/log 分开，SDK 与依赖没有重装。

实际运行：`ROOT/log/task4-all-r4/summary.json`，总 `pass_=true`。完整本地副本为 [suite summary](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-final-evidence-r1/task4-all-r4/summary.json)，[完整归档](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-final-dds-evidence-r1.tar.gz)。

监督器补强后，使用新评分器对这批完整raw独立复算：[更严离线评分](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-final-offline-rescore-r3/summary.json)，`pass_=true`、`execution_rerun=false`。六场均无 `supervisor-error.txt`，新门禁 `supervisor_integrity=true`；59个原输入文件评分前后SHA不变，原summary/archive未覆盖。新分析分别保留原运行 `harness-source-sha256.json` 和后补评分器SHA，不能把新harness说成本次17角色的原运行代码。

| case | 模式/运行秒 | local / 非空 global 消息 | 新接受正式结果 | 验证结果 |
| --- | --- | --- | --- | --- |
| contract | 三进程 / 28.994 | 876 / 518 | 2 | PASS：0.2→0.05→0.03→0.01→0.005 m 各源实际 MAP_APPLIED，LOCAL_GRID=0.01/500²；非恒等零 stamp static TF 与同零 stamp 更新；缺输入空 Path；frame、有限单位四元数及严格参考时间 |
| legacy_tf | 三进程 / 10.001 | 500 / 137 | 2 | PASS：旧 `/P4/input/map_to_odom` reliable/volatile 的零 stamp TF 及更新 |
| lifecycle | 三进程 / 26.991 | 899 / 322 | 3 | PASS：C 后同发布者连续有效 A→障碍内 B，A 不重发且保留；完整墙环内新有效目标产生 `1/DOMAIN_EXHAUSTED/false`；期间不发布旧目标路线，开门后不重发目标仍恢复 |
| forward_spin | 三进程 / 23.997 | 705 / 198 | 4 | PASS：0.6 m 前进、同 xy 新 yaw=0.6；终端停稳保持≥0.3 s；第15请求延迟250 ms时已有 buffer、DDS非空续发且实际运动；运动中 SIGINT 后新鲜度制动 |
| reverse | 三进程 / 21.000 | 644 / 277 | 2 | PASS：0.6 m 倒车、有符号负末速/车体 yaw、终端保持；再次运动后停发受限制动 |
| empty_smoke | 双进程注入 / 7.000 | 118 / 0 | 不适用 | PASS：独立标准 Path 注入，运动中收空 Path 后连续受限制动；不冒称实际 planner 产生此空 Path |

每场目录为 `ROOT/log/task4-all-r4/<case>/`；对应本地副本在上述 suite 目录同名子目录。保存 `fixture.jsonl`、`consumer.jsonl`、`planner.jsonl`（empty 无 planner）、`state.csv`、各 console、压缩原高程/几何 `.npz`、`summary.json`；前进另有 `delay.jsonl`。进程 PID/PPID/PGID、Linux start ticks、ROS 域、wall/monotonic clock、安装路径、生产 Python/native SHA、限制及退出回执均在原始流中。地图证据是输入高程及几何/发送记录，不声称保存全部 DDS wire bytes。

DDS graph 记录实际标准类型与发布者。正式全局成功必须同时满足原始 `0/PLAN_FOUND/true`、请求目标等于当前目标、提交 generation/revision 等于当前值、同 tick `ROUTE_ACCEPTED` 的版本/epoch/终点对应；周期 `ROUTE_CONTINUITY_RETAINED` 不计新接受。新旧 Path 用 publisher 相同的 header stamp 区分产生在新目标发送前后，避免把排队中的旧消息冒充新输出。Path 本身无 revision，精确 A→B→A 身份隔离仍由纯 session 测试证明。

## 独立运动与时间边界

消费者每小步≤0.02 s，限幅加速度后分别更新末速和中点位姿；先真刹停再 spin。行进检查 `|w|≤|v|×1.0`，spin 单独允许，所有小步检查速度/线角加速度及整车矩形 SAT。SAT 以解析 fixture 障碍和细分采样间运动上界独立覆盖旋转中段；单测包括端点安全但中段车角碰撞，未调用生产 certify。运动案例为短平图直线与转向，不能扩大为真实障碍运动或车辆动力学验收。

| 真实状态指标 | 结果 |
| --- | --- |
| 前进首目标停稳保持 | 6.472–6.782 s；x=0.551751 m，位置误差0.048249 m，yaw/v/w=0 |
| 倒车首目标停稳保持 | 6.507–6.808 s；x=-0.547475 m，位置误差0.052525 m，yaw/v/w=0；实际负速度有记录 |
| 末端转向停稳保持 | 12.229–12.539 s；yaw=0.595002 rad，误差0.004998 rad；转向阶段实际转角0.594317 rad、平移0 m |
| 延迟续用 | 实测 sleep0.250378 s；区间收到5条非空 local Path，实际位移0.047564 m，且已接受 buffer 与 local_busy 续用事件对应 |
| 运动限制观测 | max|v|=0.2 m/s、max|w|=0.543024 rad/s、max|a|=max|alpha|=0.5；逐步曲率及 SAT 门禁通过 |
| planner SIGINT 停发后 | 前进/倒车时刻速度分别+0.2/-0.2 m/s，随后位移0.111502/0.111519 m并停稳；含0.35 s路径新鲜度反应，未瞬时置零 |
| 空 Path smoke | 接收附近采样v=0.194817 m/s，其后剩余制动0.037958 m / 0.391635 s；完整状态与每步限幅记录保留 |

指标完整计算结果见 [motion metrics](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-final-motion-metrics-r1.json)，可直接审阅 [forward CSV](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-final-evidence-r1/task4-all-r4/forward_spin/state.csv)、[reverse CSV](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-final-evidence-r1/task4-all-r4/reverse/state.csv)。时间字段合同另行验证；重定时消费者负责自己的制动/扫掠，不继承原参考时序的认证。

## 测试、安装与来源链

Task1/2/3 及后续 r5b/r6 生产修复均已经规格与质量双审。Task4 独立 motion/证据判定经历红绿：最初7项、转向停止/Path检查到9项、当前目标关联/实际无路原因/DDS类型到12项，Windows定向12 passed；这12项已纳入生产r6的 KAI 完整回归。之后新增4个纯mock监督门禁测试，先3 failed/1 passed，再与原12项合跑16 passed（0.07 s）；没有把后补测试算成已执行的537项全套。

- [生产r6完整核心日志](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-all-core-final-r1.log)：监督门禁补强前537 passed / 1 skipped，76.86 s，退出0。范围为当时 `tests/navigation/one_cm` 加规划包 `test/`；真实 native 与 solver 显式启用。唯一 skip 为 acados 已安装时的缺依赖提示测试。末尾保留两条 `SQP_RTI ... QP solver ... status 3` 诊断，不能由套件通过推断每次求解成功。补强后仅跑[受影响16项纯测试](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-supervision-green-r3.log)和原raw离线复核。
- [最终包结果](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-pkg-result-final-r1.log)：Release 构建及10项汇总通过；包含8个 pytest 加2个 CTest 汇总。现有 colcon mixin 重名警告保留，没有构建或测试失败。
- 最终安装路径：`ROOT/install/p3-pkg-final-r1/lunar_nav1cm_planner`；隔离 build=`ROOT/build/p3-pkg-final-r1`。构建源为 `ROOT/source-p3-r6`；[安装身份回执](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-installed-identity-final-r1.json)证明最终安装33个生产Python（29个core、4个adapter）与r6源码清单逐SHA一致；其中5场实际planner各自记录的29个core Python及3份native `.so` 与最终安装一致，empty_smoke没有planner。README一致；清除源码 PYTHONPATH 后从仓库外导入，无 verification_plant 导入。
- 最终生产基线 r6 archive SHA256=`418bbb915abce6b3cb0c250219cf50a6351d47ce2bc3860e23b9be0739f82d0e`；转换文件 SHA=`483e336f1ea5a87677cb9404320e03a37b9e9d64e36a314c043931c448913736`。固定 source 的 r5b SHA=`6d61434aa79defa5670eb70fa330767b1bfce9cc084f549329419738ab7a287d`；r6 只改转换及其测试，没有改参考缓冲/运动核心。
- 运行/评分代码的已审冻结为 `ROOT/source-p3-final-r3`，其原archive/manifest保留；最后文档收尾交付为 `ROOT/source-p3-final-r4`。新 [外部 manifest](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-source-p3-final-r4-manifest.json)保留所有文件SHA、相对r3仅Markdown变化的逐项差异及archive SHA，避免报告自包含自身hash。final-r4没有改变production、harness、评分器或测试；此前相对原r6的评分器等后补变化仍分别保留身份，不能称原r6的180文件全部不变。生产33个Python及3个native源/二进制仍与实际五场planner一致，未重装或重建native/SDK。具体原运行与后补评分器SHA分别保存于更严离线分析，不混作新ROS执行证明。

## 原失败及修复事实

所有旧日志、源码快照、失败 summary 与真实成本原样保留；新输出使用新label，无覆盖、清理或把历史false改成true。

| 阶段 | 真实问题与处置 |
| --- | --- |
| 初始 Task4 r1/r2 | 静止收到 `.6m` 目标却因异步结果按 elapsed 删首段加速、保留制动尾段，MODE/LIMIT 安全拒绝导致起步失败；解析最小反例原认证VALID、age=.1后v=[0,.05,-.05]变MODE。另有harness在自然退出收尾过早SIGINT导致-2，已用自然退出宽限修复；不能将该场总报PASS |
| r4fix1/2/3 演进 | 速度桥接改善新结果接续；旧buffer仍会LIMIT/MODE。每tick重新安装派生轨迹的版本因固定来源/有限余量审查未通过；15秒/弯道probe揭示参考可反复充值，不作为最终算法。源与raw保留于 `task3-handover-dds-r4fix1/2`、`spec-review-r4fix3-p2` 和对应archive |
| r5b | canonical source 与 derived 分离、物理单调进度、bridge+尾段扣固定几何预算；通过有限来源、暂停及弯道probe与双审。旧严格elapsed分支保留；不是简单放松认证或初态置零 |
| Task4 r3 / r5b | 完整六场中四场全PASS；contract/legacy仅path_contract失败，原始相邻pose stamp相等。总suite=false保留于 [r3证据](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-all-r3-evidence.tar.gz)；不是TF、分辨率或运动失败 |
| r6 / Task4 r4 | 只修整数纳秒量化顺序，首点t0，后续最小+1ns；保留全部同xy/yaw点与轨迹digest。定向测试、安装真消息及双审后完整六场全PASS。最小连续时间 `[0,.2,.2000000001]` 不再输出重复stamp |
| 后补监督门禁P2 | QA用真实empty raw合法前5秒加纯mock进程复现：consumer5s提前exit0、fixture7s自然结束，supervisor写error却仍return0/suite true。修复为捕获异常及既有error证据独立否决case/suite，其他分析保留；原QA反例现return1/suite false，正常收尾正例仍PASS，离线analyze也拒绝空/既有error文件。原完整六场没有该异常，更严离线复核仍全部PASS；P2已由独立窄规格及原QA复审关闭，整体复审PASS，原失败证据保留 |

初轮归档见 [r1/r2完整证据](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-first-evidence-r1.tar.gz)，量化反例见 [最小脚本](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-ns-minimal-r1.py)。无路原因曾被harness硬编码为未观察到的 `NO_PATH_ENCLOSED`；已改为核对实际 `1/DOMAIN_EXHAUSTED/false`，没有改写planner原因或旧summary。原阶段、各演进和最终记录分列，不挑选单个成功场景替代总门禁。

监督漏洞的[修复前复现](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-supervision-qa-red-r3/result.log)、[修复后相同QA场景](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-supervision-qa-green-r3/result.log)与[评分脚本](D:/CodexDownloads/lunar_navigation/nav-1cm-feasibility-20261026/artifacts/p3-interface-20261027/task4-offline-rescore-r3.py)均另存；没有ROS或真实子进程，未覆盖原QA脚本或原6场summary。

## 可复跑命令

构建先 source Humble、真实SDK及 core underlay；源码位于 `SOURCE`。每次复跑使用不存在的新RUN目录，保存新log，不覆盖本报告原记录。安装/运行参数和真实话题见规划包README。

```bash
set -e
export ROOT=/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-p3-interface-20261027
export SOURCE="$ROOT/source-p3-final-r4"
export RUN="$ROOT/recheck-<unique-label>" # 替换为本次全新路径
test ! -e "$RUN"
mkdir -p "$RUN/harness"
export NAV1CM_SOURCE_ROOT="$SOURCE"
source "$ROOT/deps/env_desktop_humble.sh"
source "$ROOT/install/core-humble-r1/setup.bash"
colcon --log-base "$RUN/log-build" build \
  --base-paths "$SOURCE/scripts/navigation/one_cm/ros2/lunar_nav1cm_planner" \
  --packages-select lunar_nav1cm_planner --build-base "$RUN/build" --install-base "$RUN/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=ON
colcon --log-base "$RUN/log-test" test \
  --base-paths "$SOURCE/scripts/navigation/one_cm/ros2/lunar_nav1cm_planner" \
  --packages-select lunar_nav1cm_planner --build-base "$RUN/build" --install-base "$RUN/install"
colcon test-result --test-result-base "$RUN/build" --verbose
source "$RUN/install/setup.bash"
export NAV1CM_BUILD_DIR="$RUN/mpcc"
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1
# 清除env helper加入的源码路径；harness目录自身也不暴露源nav1cm。
export PYTHONPATH="$(python3 -c 'import os; from pathlib import Path; s=Path(os.environ["SOURCE"]).resolve(); print(":".join(p for p in os.environ["PYTHONPATH"].split(":") if p and not Path(p).resolve().is_relative_to(s)))')"
cp "$SOURCE/scripts/navigation/one_cm/p3_path_validation/"*.py "$RUN/harness/"
cd "$RUN"
python3 "$RUN/harness/run.py" --output "$RUN/dds" --build-dir "$NAV1CM_BUILD_DIR" \
  --cases contract legacy_tf lifecycle forward_spin reverse empty_smoke
```

上述域91仅是本次隔离选用值，复跑前确认该域空闲。算法全套使用另一个独立纯core shell，从源码根执行，不能直接沿用前段ROS/install的Python环境；下面的core bootstrap清除ROS Python namespace并保留真实SDK，再补纯包测试的源码入口。算法全套本次已执行一次；收尾仅以同环境 `--collect-only` 验证可收集，不重复537项执行：

```bash
set -e
export ROOT=/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-p3-interface-20261027
export SOURCE="$ROOT/source-p3-final-r4"
export RUN="$ROOT/recheck-core-<unique-label>" # 替换为本次全新路径
test ! -e "$RUN"
mkdir -p "$RUN"
export NAV1CM_SOURCE_ROOT="$SOURCE"
source "$ROOT/deps/env_desktop_core.sh"
export PYTHONPATH="$SOURCE/scripts/navigation/one_cm/ros2/lunar_nav1cm_planner:$PYTHONPATH"
export NAV1CM_REAL_SOLVER=1 NAV1CM_BUILD_DIR="$RUN/mpcc"
export INSTALL="$ROOT/install/p3-pkg-final-r1/lunar_nav1cm_planner"
export NAV1CM_TERRAIN_LIB="$INSTALL/lib/libnav1cm_terrain.so"
export NAV1CM_GEOMETRY_LIB="$INSTALL/lib/libnav1cm_geometry.so"
export NAV1CM_DISTANCE_FIELD_LIB="$INSTALL/lib/libnav1cm_distance_field.so"
export LD_LIBRARY_PATH="$ROOT/install/core-humble-r1/lunar_incremental_navigation_core/lib:$LD_LIBRARY_PATH"
cd "$SOURCE"
python3 -m pytest tests/navigation/one_cm scripts/navigation/one_cm/ros2/lunar_nav1cm_planner/test \
  -q --basetemp="$RUN/pytest-core"
```

Windows源码、依赖、导出和临时产物均放任务 D: 根；本次没有新增 C: 大文件。任务分支 `hk/nav-1cm-feasibility-20261026`、基线 `439178f2f09399470696b43e1f47f089c44f41cb`，归属 `integration/pure-planner-orin`。处置为**暂停保留（本阶段完成，源码未提交，待用户审阅及后续提交/合入授权）**。源码未提交、未合入、未推送，未创建PR、设置upstream或删除资源；专题工作树、旧快照、两机依赖/构建和全部证据保留待处置，不宣称已完成仓库级整合或资源回收。最终外部closure receipt记录本阶段Task1–4及整体独立复审PASS，并区分已审r3与仅文档收尾r4。
