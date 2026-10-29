# 有界 P3 / 标准 Path 验证

此目录是验证工具，不是规划器部署依赖。Linux ROS 2 Humble 中由一个 supervisor 依次启动三个独立 rclpy 进程：fixture 发 GridMap/TF/Goal，实际已安装的 `PlannerNode` 规划，consumer 只收两条 Path 并发布独立模型的 Odometry。没有 T5、ACK、私有控制或目标到模型的控制捷径。`empty_smoke` 明确为双进程标准 Path 注入测试，不冒充实际规划器输出。

消费者只适用于本入口的短直线、倒车和末端转向场景。它用 Path 几何重新安排速度，通过至多 20 ms 的小步、末速与中点位姿积分实现连续运动。重定时后的限速、曲率、加速度、制动和矩形 SAT 扫掠由消费者独立检查；原 Path 时间字段另行核对，不声称精确执行原时序或继承原认证。

先按[规划包说明](../ros2/lunar_nav1cm_planner/README.md)构建真实依赖和包。隔离 source Humble、core 与 planner 三个正确 overlay，配置真实 SDK；`PYTHONNOUSERSITE=1` 避免混入用户 NumPy。运行的 `nav1cm.__file__` 必须位于安装目录，不能依靠源码 PYTHONPATH。将本目录单独复制到仓库外 `HARNESS`，其上级不能包含源 `nav1cm/`：

```bash
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1
export ROS_HOME="$RUN/ros-home" ROS_LOG_DIR="$RUN/ros-logs"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export NAV1CM_BUILD_DIR="$RUN/mpcc"
cd "$RUN"
python3 "$HARNESS/run.py" --output "$RUN/dds-fresh" \
  --build-dir "$NAV1CM_BUILD_DIR" \
  --cases contract legacy_tf lifecycle forward_spin reverse empty_smoke
```

`RUN`、`HARNESS`、build/install/log 应为明确的仓库外路径。Windows 开发与导出默认使用 `D:/CodexDownloads`；本次授权远端仅写 `/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-p3-interface-20261027`。`--output` 必须不存在；工具不清理或覆盖旧证据。完整运行约两分钟，单场约 7–29 s；supervisor 对每个进程有期限，只向本次创建的确切 PID/进程组发信号，任何升级终止或残留进程都判失败。

监督异常是独立否决门禁。即使角色进程提前退出码为0、其他运动分析通过，也必须使case/suite失败、入口返回非0，并保留其余诊断和 `supervisor-error.txt`。离线 `analyze` 同样检查已有监督错误证据，空错误文件也不能被当作通过；`checks.supervisor_integrity` 与 `supervisor_error` 保真记录该项。

场景：

| case | 输入和门禁 |
| --- | --- |
| contract | 6 m 平图；源 0.2→0.05→0.03→0.01→0.005 m 各保持 5 s；实际 MAP_APPLIED、LOCAL_GRID=0.01/500²；标准 static TF 零 stamp 非恒等及更新；缺输入空 Path；Path frame、单位四元数和局部严格递增参考时间 |
| legacy_tf | 同样零 stamp 更新，经旧 `/P4/input/map_to_odom` reliable/volatile 入口 |
| lifecycle | 8 m 完整已知墙环；先 C，再同发布者连续有效 A→障碍内 B；A 不重发；新有效无路目标在环内，开门后不重发目标仍恢复全局参考 |
| forward_spin | 0.6 m 前进、同 xy 新 yaw 0.6 rad；终端位置/朝向/停稳持续至少 0.3 s；第 15 个真实 local 请求内部延迟 250 ms，核对已有 buffer、DDS 非空续发及真实运动；运动中 SIGINT 规划器并按路径新鲜度刹停 |
| reverse | 0.6 m 倒车；有符号负末速和实际车体 yaw；停稳终端；再次运动后停发受限停车 |
| empty_smoke | 标准 Path 注入后改为空 Path，核对实际运动中的连续制动；单独标注证据类型 |

生产 planner 不增加延迟参数或状态话题。内部 factory 仅观测原始 tick/events，单请求 sleep 不改算法和输出；正式全局成功必须来自原始 `0/PLAN_FOUND/true` 三字段。Path 不含目标 revision，不能单凭 DDS Path 证明精确 A→B→A 身份隔离；该项由纯 session 测试覆盖。

每场保存原始 `fixture/planner/consumer.jsonl`、完整两条 Path、压缩原高程输入 `.npz`、逐步 `state.csv`、console、安装路径/进程启动 ticks/时钟/限制、退出结果及 `summary.json`。DDS graph 的实际类型和发布者、已安装生产 Python/native SHA 一并记录。源码 SHA 单列于 `harness-source-sha256.json`。正式成功要求原始三字段、当前目标/代际及对应 ROUTE_ACCEPTED 的请求版本和 epoch 一致；周期连续性保留单独记录，不能计为新接受。封闭墙环的实际无路原因保真记录为 `1/DOMAIN_EXHAUSTED/false`。回看时以 summary 的全部门禁为准；到达、solver status 0 或可视化单独均不足以通过。

独立校准从仓库根执行：

```powershell
python -m pytest tests/navigation/one_cm/test_path_validation.py tests/navigation/one_cm/test_path_supervision.py -q `
  --basetemp=D:/CodexDownloads/lunar_navigation/nav1cm-path-test-tmp
```

已知边界：fixture 是生成高程，不是真实 P3 采集；短运动是独立数学模型，不是车辆驱动。跨机 DDS、当前源码 Orin、真实 rosbag、实车、起步盲区和长距离 1 km 均不由本入口证明。
