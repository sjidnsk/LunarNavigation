# 起步与行进模式偏好及回退交付

任务：SPIN-ANGLE-30-20261102。owner：integration/pure-planner-orin。
专题：hk/spin-angle-30-20261102；基线 b1575f9b。

## 行为与职责

- 起步15°、普通停车后30°是优先模式选择角；正在行进不因超过30°强制停车。终点完成、残余角速度及真实制动规则保持原职责。
- 行进候选三组既有初始化均失败后，若仍有需对准的角度，复用既有spin求解器，最多再尝试三组；有平移速度时先拼接既有真实制动前缀。
- 固定初始圆碰撞不可行、无剩余对准角，不进行无效spin尝试；不更改75cm碰撞半径，不清除地图障碍。
- 返回的仍是未认证候选；调用方复用既有认证。没有引入控制器或额外任务实体。父级认证拒绝后的模式选择本批未扩展。
- 复审发现协调器仍用0.08rad判起步已对齐。10°/14.9°的正反向回归先复现4失败，随后与规划器共用INITIAL_ALIGNMENT_ANGLE_RAD，防止起步要求无法退休。

## 验证边界

- Windows本机：tests/navigation/one_cm，1284 passed、96 skipped；含模式偏好、正反向起步退休、失败回退、真实初态保持、认证与地图不变。日志：D:/CodexDownloads/lunar_navigation/global-planning-perf-20261101/spin-mode-fallback-handoff.log。
- TCP ARM64/Humble隔离目录：/home/yanfa/P4/tmp/spin-mode-fallback-20261102。上一版local_mpcc真实acados定向测试68 passed、3 skipped；35项生成模板警告，没有改依赖。
- 真实acados组件额外对照：注入行进solver失败，10°静止/0.1m/s移动初态，spin求解与完整认证通过；actual-platform-results.json。该组件使用0.3m/s、0.15rad/s，不代表现场配置或真实几何堵塞场景。
- 最终协调器共享15°补充：仅Windows回归通过；Humble重新安装与车辆到达验证NOT_RUN，由统一部署入口接收后执行。

## 现场与交接

在统一部署通知到达前，上一版local_mpcc已替换以下三个正式文件并重启P4：

1. /home/yanfa/P4/source/scripts/navigation/one_cm/nav1cm/local_mpcc.py
2. /home/yanfa/P4/build/current/input/scripts/navigation/one_cm/nav1cm/local_mpcc.py
3. /home/yanfa/P4/build/current/install/lunar_nav1cm_planner/local/lib/python3.10/dist-packages/nav1cm/local_mpcc.py

三处部署SHA256：cbf08a367386d5e0e72efef9dd29bb057b467d975d9d0b0144cbc2686473df9c。
回执：上述隔离目录的deployed.json、running-verified.json；备份在before/。
当时运行目录/home/yanfa/P4/run/p4-20261102-184016-66606，operator66606，导航66659，RViz66660；地图与定位有输入，未下发目标。
配置校验未变，现场为0.3/0.3m/s、0.1rad/s、坡度20°；与旧会话30°/0.15rad/s有差异，未覆盖其他任务配置。

收到tcp统一部署分工后，停止全部后续远端写入/重启。最终交付含local_mpcc.py与async_coordinator.py两文件，必须作为一组同步；当前现场只有上一版local_mpcc，并非本专题最终完整版本。统一部署会话按用户最新要求决定平台限速，本专题不抢写配置。

处置：专题验证后提交待owner合入；现场为部分旧版已部署，最终补充待统一部署。未push、未合并长期线、未清除地图、未改P3/Env_X。专题工作树及隔离证据保留待授权回收。
