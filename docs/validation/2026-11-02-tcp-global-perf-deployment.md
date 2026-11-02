# TCP全局性能优化正式同步

用户于2026-11-02授权同步192.168.100.216，追加软净空50cm，并授权合入主线。先前自动审批拒绝主线合并后，收到明确授权才重新执行；未绕过审批。公共优化bf7c0029已快进主线，TCP接收合并0069e6be。

本批补齐全局worker预热、后台平滑、同快照精确证明复用、搜索/处理/平滑共享1s、分阶段日志。TCP启动配置允许既有global_soft_clearance_m参数，正式tcp_p3_feedback.yaml设0.5。全局权重1、硬碰撞半径0.75m、地图分辨率和全局15s周期均未改。

远端原runtime.py与TCP分支差异在P3反馈观测高水位处理。本次保存原件后仅叠加GLOBAL_RESULT计时字段，未用主线整文件覆盖该差异；因此远端不能标为与某Git提交整树相同。三份算法文件SHA与公共优化逐字节一致，runtime另有适配后SHA，全部见verified.json/applied.json。保留差异后，正式安装runtime相关回归通过。

部署只修改P4内10个明确文件（source六份、build/current/input四份），原件在P4/tmp/global-perf-sync-20261102/before及before-install；重建build/current中的lunar_nav1cm_planner成功。未上传设计文件、模型或数据，不改P3/Env_X。
用户已在现场更新坡度25度、前后0.3m/s、角速度0.15rad/s。保留README、source/config/wheel.yaml、source/config/tcp_wheel_speed.yaml、config/vehicle.yaml及环境文件，部署前后SHA一致；组合配置加载断言通过。没有把这些现场平台修改回填到公共默认能力文件，后续平台配置归属按原修改任务处理。

验证：Windows one_cm1245通过95条件跳过；TCP配置/相关全局114通过。正式ARM64 Humble构建成功，安装导入路径及四模块SHA匹配部署清单；相关算法/ROS135通过1条件跳过。新增50cm传递测试补齐隔离测试所需p4_topics.yaml后通过（第一次仅缺测试夹具，不是启动故障）。本批未重新进行车辆到达验收。

启动核对：开始部署时未发现P4运行。用户要求先启动时，检测到另一入口已于12:00启动TCP/P3/P4/RViz；本会话没有重复启动、重启或发目标。P4进程18568、导航入口18621、RViz18622存活，进程启动晚于本次代码更新时间。运行目录/home/yanfa/P4/run/p4-20261102-120025-18568，launch-arguments确认软净空0.5，实际test-wheel确认25度/.3/.3/.15。日志显示用户目标正在执行，MPCC认证VALID、实际指令非零；两次GLOBAL_RESULT为PLAN_FOUND/0/has_reference=true，worker_compute_ms约228/255、smoothing_ms约32.5/31.8、total_budget_s=1。本轮未证明任务自然到达，不将持续发令当作闭环成功。

本机证据D:/CodexDownloads/lunar_navigation/global-planning-perf-20261101/tcp-deploy-20261102；远端仅/home/yanfa/P4/tmp/global-perf-sync-20261102。后续收尾仅本地提交与分支记录，不打断正在运行的任务。无push/PR，临时工作树及回滚副本保留，待后续授权回收。
