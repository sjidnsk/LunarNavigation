# 公共滚动控制器专题验证（2026-10-16）

## 范围与来源

候选分支 `chore/rolling-controller-integration-20261016`，基线
`490333080a25c78b2298dc26a94c7e2b827ef207`，owner 为 `integration/pure-planner-orin`。
未移动长期分支、未推送、未触碰原应用 dirty、`.runtime` 或训练。

- `execution.py`、`limits.py` 固定提取自应用提交
  `800b4f11b7a5d0b38e20e25d13c2c7cb79041c36`，含 `6b9c1645` 到达修正；保留原主线到达回归。
  I1 修复后 execution.py 在该来源上增加初始对齐航向锚点，已不再与来源逐字一致。
- `node.py` 仅采用 `b5059eb8edd77dd0b50a5fcc3ed7ae31e0e72c91` 的滚动参数和执行器构造，
  加同一固定应用源的 `note_command` 发布记录；不引入应用硬件/平台策略或改变 TF 策略。
- 成熟执行器依赖横向速度停稳判断，故最小提取同应用源 `tracking.py` 的末尾默认字段
  `lateral_mps=0.0`，及 `frame.py` 的读取、有限性检查、传递三处；兼容已有五参数调用。
- `test_execution.py` 保留固定应用源全部行为回归，新增横向运动/非法反馈及两模式最终位置丢失回归。
  `test_node.py` 提炼同源 timer.cancel 稳定修正，新增 6 个模式/开关组合、停止命令历史、横向停稳边界。
  `test_tracking.py` 与固定应用源逐字相同，未改动。
- `tests/test_isolation_contract.py` 和 `tests/test_incremental_navigation_interface_contract.py`
  只取 Task 2 `7acdfb5cda8d47e2d1c68d198d2a3fcbc0984b1f` 已验证的三项过时断言修正：
  独立节点 32 m 与 YAML 64 m 覆盖、两个不同地图发布器、已有 wheel/legged Orin 配置白名单。
  未引入可信度专属断言或为测试更改生产值。

`PathExecutor` 构造器默认 `rolling_updates=False`。节点 `enable_rolling_updates=True`
仅在 `incremental_reference` 生效；显式 false、`motion_reference`、`incremental_path` 保留旧切换行为。
TRACKING 热替换保留连续执行，方向改变、过大偏差、曲率不可行仍制动。
本轮行为验证覆盖 TRACKING 停滞、FINAL_ALIGN 等价修订停滞，以及 I1 新增的初始 ALIGNING
等价修订停滞、真实航向改善、实质新目标、停稳后失败与 clear；验证范围见下节。
主线在非 TRACKING 反馈后 hold/freeze 并丢弃待处理 rolling，只是降低触发机会；
PathExecutor 必须独立维持本地超时契约，不能依赖上游 hold。

## 初次提交 52b6dfa5 的验证记录

环境为宿主 `/opt/ros/humble`，`ROS_DOMAIN_ID=226`、`ROS_LOCALHOST_ONLY=1`。
仅隔离测试节点，无车辆、仿真服务或真实命令输入。构建/日志在仓库外
`/home/kai/CodexDownloads/lunar_navigation/branch-integration-20261016/task-5-humble`。
只读消息依赖 `/mnt/data/WS/lunar-navigation/.runtime/region-relation-v2/native-humble/install/lunar_planning_msgs`，
其中 `PathReference.msg`、`TrackingStatus.msg`、`MotionReference.msg` 已与当前源码逐字比较。
没有使用可信度 candidate 库或旧控制器 install。

|检查|结果|日志（上述产物目录内）|
|---|---|---|
|修改前 controller 全包 pytest|154 通过|`logs/controller-baseline.log`|
|来源测试先行 RED|22 失败、154 通过，缺 rolling API/命令函数|`logs/controller-red.log`|
|节点新增回归 RED|8 失败、37 通过，开关/命令历史缺失及横向运动错误成功|`logs/node-red.log`|
|controller 最终 pytest|184 通过|`logs/controller-green-final.log`|
|controller Humble configure/build/install|通过|`logs/{configure,build,install}-controller.log`|
|controller CTest 串行|6 组、184 用例，零失败/错误/跳过|`logs/ctest-controller.log`、`controller/test_results/`|
|受影响静态契约|68 通过；修正前 3 失败/65 通过|`logs/contracts-final.log`、`logs/contracts-initial.log`|
|扩大 root 离线 pytest|230 通过、2 个 BASE 同样失败|`logs/pytest-root-offline.log`|
|固定 BASE 的上述两项|2 失败|`logs/baseline-known-failures.log`|
|根目录全部 pytest 尝试|收集冲突，未执行整库全量|`logs/pytest-repository.log`|
|固定源与主线保留文件 SHA256、覆盖设计原文前缀|通过|`source-manifest.json`|
|UTF-8、git diff --check|通过|专题提交前检查|

第一次应用后新增最终位置测试错误要求 BRAKING 阶段直接输出 failure_reason，故为 2 失败/182 通过，
见 `logs/controller-green.log`。按原主线既有契约修正测试为先 BRAKING、下一周期 FAILED，
不修改生产行为；随后最终全包和 CTest 均通过。

## 精确执行入口

完整可重放命令存于 stage 的 `task-5-run-pytest.sh`、`task-5-run-baseline.sh` 和 `task-5-build.sh`。
`STAGE=/home/kai/CodexDownloads/lunar_navigation/branch-integration-20261016`：

```bash
bash "$STAGE/task-5-run-pytest.sh" ros2_ws/src/lunar_pure_wheeled_controller/test
bash "$STAGE/task-5-build.sh"
bash "$STAGE/task-5-run-pytest.sh" tests/test_external_interface_contract.py tests/test_isolation_contract.py tests/test_incremental_navigation_message_contract.py tests/test_incremental_navigation_interface_contract.py tests/test_launch_contract.py
bash "$STAGE/task-5-run-pytest.sh" tests --ignore=tests/launch
bash "$STAGE/task-5-run-baseline.sh" tests/test_operator_scripts.py::test_start_all_passes_grid_v1_mode_to_the_real_planner_invocation tests/exploration/test_task7_implementation_contract.py::test_rank_and_failure_ownership_contract
bash "$STAGE/task-5-run-pytest.sh" --import-mode=importlib
```

runner 在仓库根执行 `python3 -m pytest -q -p no:cacheprovider`，显式绑定 Humble、
消息 Python `local/lib/python3.10/dist-packages`、消息 `lib` 和 `AMENT_PREFIX_PATH`。
独立 CMake 构建 controller，`Release`、`BUILD_TESTING=ON`，`-j2` 构建、`ctest -j1`。

## 限制与后续

扩大离线检查的两个失败未越界修改：

1. `tests/test_operator_scripts.py::test_start_all_passes_grid_v1_mode_to_the_real_planner_invocation`：
   fake ros2 捕获未包含期望 planner 调用。源码 `scripts/start_all.sh` 已显式传入 grid 参数，
   但四个后台启动配合 `wait -n`/退出清理存在捕获先后竞争的可能；本次未对运行脚本下最终根因结论。
2. `tests/exploration/test_task7_implementation_contract.py::test_rank_and_failure_ownership_contract`：
   固定断言仅接受 `kExecutionReplansExhausted`，当前 BASE 实现已增加 `kNavigationNoPath` 与
   `kNavigationTimeout`，属于现有测试与实现契约不一致。后续探索模块验收单独核对。

根目录全量命令在 Humble `launch_testing` 的 `path.pyimport()` 中遇到同名 `test_evidence`：
`ros2_ws/src/lunar_incremental_controller_demo/test/test_evidence.py` 与
`ros2_ws/src/lunar_integrated_exploration_demo/test/test_evidence.py`，`--import-mode=importlib` 仍不能绕过插件。
因此不声称整库测试通过；上述明确测试集结果独立有效。

主线 `incremental_navigation_node.cpp`、`planning_session_coordinator.cpp`、`local_goal_region.cpp`
及消息接口与 BASE 保持逐字一致，最终到达/纠正次数/超时和异步安全前缀未被旧来源覆盖。
覆盖定义只保存已确认设计；返航七快照及最新可达目标快照准确列为待完成适配，见分支整合台账。

`NOT_RUN`：Jazzy、Orin、真实 DDS 输入/rosbag、RViz、TCP 动力学、车辆闭环、训练、返航 480 请求矩阵、
新增覆盖指标、导航 core/ROS 整包重构建与测试、全项目部署资格验收。没有以其他专题历史结果代替本轮验证。
本报告为实现者自审记录，不能替代主任务安排的独立审查；候选尚未落地长期主线。


## I1 修复回合 1（2026-10-16）

修复基线 52b6dfa5a81aa03508c812635de35a356eb32d06。独立审查发现初始 ALIGNING
在每次同会话 revision 后重置最佳角误差，固定姿态被反复误记为改善。
只设置一次引用的原探针对照会超时，持续 revision 则保持 ALIGNING；本轮先复现此 RED。

初始对齐现在保留“有效目标航向、最佳实测航向”的锚点。等价几何、重采样/尾部延长、
小于等于既有 alignment_tolerance_rad 的切线变化，都在当前目标下比较实测航向改善；
引用本身不能伪造进展。真实角误差改善超过既有 0.01 rad 门槛可续期；有效目标变化
超过既有对齐容差时才重建基准。新引用仍进入 BRAKING，只有测得停稳后才继续对齐。
新会话、显式 clear、进入 TRACKING 时清理对齐锚点；FINAL_ALIGN 和故障锁存原逻辑保留。
没有禁用热更新或修改生产 timeout/限速/容差。

|本轮检查|实际结果|日志（task-5-humble/ 内）|
|---|---|---|
|原审查探针 RED|exit 1；单引用超时，连续 revision 仍 ALIGNING|logs/fix-round1-alignment-probe-red.log|
|新增行为用例 RED|6 failed、1 passed、69 deselected|logs/fix-round1-alignment-tests-red.log|
|新增行为用例 GREEN|7 passed、69 deselected|logs/fix-round1-alignment-tests-green.log|
|原审查探针 GREEN|exit 0；两种输入均 FAILED/ALIGNMENT_STALLED，预算 0.35s|logs/fix-round1-alignment-probe-green.log|
|controller 全包 + 五个相关契约|259 passed（191 controller + 68 contract）|logs/fix-round1-controller-contracts.log|
|Humble configure/build/install、串行 CTest|6/6 组通过；191 用例，零失败/错误/跳过|logs/fix-round1-*-controller.log、fix-round1-xunit-counts.json|
|全仓 pytest 尝试|exit 1；两个 test_evidence.py 的既有收集冲突，未执行全量|logs/fix-round1-pytest-repository.log|

新增 7 例覆盖三种连续修订停滞、真实航向改善后再次停滞、实质新切线先停稳且预算有限、
超时先 BRAKING 再 FAILED/锁存/clear、新会话预算。既有 TRACKING 热更新、FINAL_ALIGN、
最终位置丢失和节点模式边界均包含在 191 项 controller 回归中。

使用前述同一 Humble runner 和只读消息依赖。精确命令、作者自审和报告副本在
/home/kai/CodexDownloads/lunar_navigation/branch-integration-20261016/task-5-report.md
的“I1 修复回合 1”节；独立审查仍交主 agent，不以作者自审替代。
本轮未重跑扩大 root 离线集或 BASE 两项失败；前述原始记录仍属初次提交。
原 source-manifest.json 也是初次提交证据，不证明修复后的 execution.py 与应用源一致。
Jazzy、Orin、车辆/真实服务、GPU、训练、core/ROS 整包、返航矩阵和新增覆盖指标仍 NOT_RUN。


## I1 修复回合 2（2026-10-17）

修复基线 63b87c5408ab137cb4a31e5374934f0f6ac1caf3。回合 1 的微小切线抖动测试仅覆盖
固定 yaw=-1，未覆盖正、负倒车选择阈值；独立复审发现 ±2.4 rad 附近每次 revision
重新选择正反向，几何只改变 0.04 rad，内部 desired 却翻转约 pi，仍重置对齐预算。
因此回合 1 是部分修复，其通过集合不证明该边界已解决。

本轮同一对齐 episode 另保留“几何切线、已选方向”状态；当前切线相对保留基准的角差
小于等于既有 alignment_tolerance_rad 时沿用方向。超过该容差才按原策略重选方向，
更新几何基准；没有改 reverse_heading_threshold_rad、allow_reverse、timeout 或热更新开关。
方向状态与实测航向进展锚点在同会话 ALIGNING/BRAKING 修订中保留，
新会话、clear、进入 TRACKING 时清理。新引用仍先 BRAKING 并等待实测停稳。

|本轮检查|实际结果|task-5-humble/ 内日志或证据|
|---|---|---|
|正负阈值原复审探针 RED|2 对照通过、2 持续修订失败；exit 1|logs/fix-round2-reverse-threshold-probe-red.log|
|新增 7 例有效 RED|4 failed、3 passed、76 deselected|logs/fix-round2-direction-tests-red-final.log|
|新增 7 例 GREEN|7 passed、76 deselected|logs/fix-round2-direction-tests-green.log|
|未改阈值探针 GREEN|4/4 通过，均 FAILED/ALIGNMENT_STALLED、预算 0.35s|logs/fix-round2-reverse-threshold-probe-green.log|
|未改原初始对齐探针 GREEN|2/2 通过|logs/fix-round2-original-alignment-probe-green.log|
|controller 全包 + 五个契约|266 passed（198 controller + 68 contract）|logs/fix-round2-controller-contracts.log|
|Humble configure/build/install、串行 CTest|6/6 组通过；198 用例、零失败/错误/跳过|logs/fix-round2-build-ctest.log、fix-round2-xunit-counts.json|

新增 7 例包括正负阈值 × 首个切线正负符号的四种连续修订停滞（每反馈前两次修订），
实质切线变化后先停稳再重新选择倒车方向且预算仍有限，以及 clear/新会话重选方向两例。
有效 RED 的 3 passed 是已有正确行为的保护，不冒称它们均曾失败。
回合 1 的 7 例（含真实 yaw 改善）、原 TRACKING/FINAL_ALIGN、故障锁存和节点边界均在完整套重跑。
首次新增测试曾把新会话的反向旋转要求提前一帧，忽略既有命令斜率限制；只调整测试，
在改生产前重新获得上述有效 RED。完整记录见 stage/task-5-report.md 的回合 2 节。

本轮未重跑全仓 collection、扩大 root 离线集及两项 BASE 失败；既有全仓收集冲突和
历史扩大离线限制继续保留。ROS 环境、只读消息 overlay 与隔离参数沿用前述配置。
上游 hold/freeze 仍仅缓解触发机会，本地 executor/API 独立维持停滞超时。
Jazzy、Orin、车辆/真实服务、GPU、训练、真实 DDS 时序、core/ROS 整包、返航矩阵仍 NOT_RUN。
独立 scoped 复审由 root 执行，作者自审不代替独立复审。
