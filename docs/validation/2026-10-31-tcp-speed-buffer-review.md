# TCP平台速度同源与参考余量门禁收敛（2026-10-31）

任务TCP-SPEED-BRAKE-REVIEW-20261031，基线cc1a261，最终45137ec；复用hk/tcp-combined-deploy-20261031，ROOT唯一部署入口。应用速度归app/orin-unreal-simulation，公共缓冲/控制器修复待晋级integration/pure-planner-orin；不移动长期线、不push。共享基础地图暂停。

## 速度配置与真实生效链

新增config/tcp_wheel_speed.yaml只覆盖前进0.5m/s、倒退0.5m/s、角速度1.0rad/s；公共wheel.yaml不改，加速/制动/角加速度仍0.5。main和有界检查均load_tcp_platform_profile，生成同一最终profile交给planner/认证/MPPI/relay，原先有界入口写死0.05/0.1已退出实际运行路径。preflight-only仍在acquire前退出，无速度/模式/租约发布。

标准Path跟踪器原先min(TrackingPolicy旧默认0.2,平台cap)会阻止平台提速；已修为三速度未显式覆盖时直接取平台cap，不保留另一个硬编码角速度默认。现有显式用户降档参数仍可降低运行速度，超出cap会按平台能力执行，不构成第二个平台能力源。最终解析值回写ROS参数，与_policy及executor一致；非标准旧后端行为不变。

目标机实读底盘原max_linear_speed=0.2、spin_max_linear_speed=0.1，独立于导航profile。因此仅修改上层不够。旧接口按park退出后复用原二进制，以P4内生成的vehicle-params.yaml启动：max_linear_speed=0.5；按既有实读wheel_base0.815、track_width0.622将spin_max_linear_speed设为0.5126180351879945，对应原地旋转角速度1.0。轮半径0.12、最大轮速10、spin_prep_time3、steer_step_deg5、cmd_timeout1等保持原值。P3、legacy项目源码及配置均只读。

实际GetParameters回读：planner linear_speed_mps=.5/angular_speed_rps=1，controller max_linear/max_reverse=.5/max_angular=1且加减速=.5；两者都读取新run/runtime/test-wheel.yaml。底盘回读max_linear=.5且spin轮速除以实读几何半径=1。记录effective-limits.json。第一次规划器查询错误包含未声明platform_config返回空结果，改用其真实wheel_config参数后读取成功，不是能力未生效。

## BRAKE原因、必要性与阈值

8609360旧完整日志82个BRAKE周期互斥分解：12等首全局参考；27新spin被BUFFER_BRAKING_RESERVE拒绝后旧buffer为空；37无可接管候选（35个local_busy，含3个stale返回）；4新旧完整候选MODE；1仅旧MODE；1反馈重锚不连续。BUFFER_EMPTY是前因失败后的结果，不是64个独立算法故障。

实际源码/空闲地图复现：v=.002/w=.05的braking+spin、源w仅1e-6而实测w=.02的前进参考，原source及重锚raw完整认证都VALID，却因来源剩余D/A小于停车余量被拒绝。这层判断没有查询真实障碍物，且冗余_stopping_geometry对差速模型错误沿用了曲率耦合。因此删除_prepare预检及_finite逐轴预算裁剪，不靠降低reaction、放宽速度/碰撞/反馈阈值解决。

保留固定canonical source、单调phase floor、仅原有限控制后缀、实测重锚、来源耗尽/反馈不连续和当前图完整认证。_finite保留有限raw并调用原brake_candidate显式追加制动尾，尾进入实际Path；source不会被derived/bridge递归替换。bridge及制动尾可使派生终点超过canonical几何终点，不能宣称绝不超来源末端或已到达最终目标。局部规划/导航最终目标验收职责不变。

## 验证与部署

本机速度相关99通过，缓冲171通过3跳过，控制器解析纯测90通过；各组有重叠，不相加。独立复审逐批通过，关闭角速度仍被硬编码1.0限制的初稿问题。216隔离速度99通过、缓冲147通过2跳过；最终Humble两包构建通过，JUnit planner11组67项、controller27组665项，最终零失败零跳过，含真实ROS节点三速策略/执行器/参数读回、平台角上限1.2与显式降档/截断场景。

保留失败证据：隔离初次漏传依赖/旧测试夹具及build环境导致失败，产品源码逐文件与冻结提交核对一致，补齐当前提交测试后仅复跑受影响项；配置CTest又暴露依赖repo-root导入的问题，45137ec修测试独立目录导入，目标CTest重跑通过。未为过时停滞字段测试恢复重复机制。

旧operator42688退出时即时measured_stopped=false，未继续部署；随后新遥测确认vx约-1.82e-9、vy约-9.07e-11、各轮速均小于8.3e-8，measured_stopped=true后才切换。原即时失败结果与补充post-stop-evidence均保留。旧车辆6521正常退出；新operator52228、vehicle52191。六必要文件逐项备份同步，其他目录不写；backup P4/artifacts/pre-tcp-speed-buffer-20261031，receipt P4/artifacts/tcp-speed-buffer-deploy-receipt-45137ec.json。隔离overlay P4/artifacts/tcp-speed-buffer-install-20261031；运行目录P4/run/tcp-interactive-45137ec-20261031，P4即/home/yanfa/P4。P3保持原运行，域10。

## 现场边界

同一目标重新规划35秒：551参考含516 FOLLOW、30 BRAKE、5 INVALIDATED；225指令64零速，线速峰值0.001318、角速峰值1.0。主要转向，不能声称平移实际达到0.5或导航顺畅。703遥测最大间隔0.618秒，仅一次auto、无park。P3原始odom47条，最大接收间隔1.68秒，原始位置距目标约8.244→8.073米（最小8.038），未自然到达。

完整日志快照83个BRAKE周期，无BUFFER_BRAKING_RESERVE、ACTION_REJECTED；仍有BUFFER_EMPTY48、LOCAL_CERTIFICATION_OUT_OF_BOUNDS13、FEEDBACK_DISCONTINUITY11与LOCAL同类7、BUFFER_CERTIFICATION_MODE9与LOCAL同类3、BUFFER_SOURCE_EXHAUSTED2。事件可同周期重叠，快照时间长于35秒，不与参考分母混用。提升速度后需继续核对轨迹与局部窗口尺度、重锚误差及完整MODE真实分支；本轮不凭计数放宽这些阈值。

真实长距离/绕障/自然到达验收未完成。原始证据在新run的motion-after.jsonl/navigation.log/effective-limits.json；本地ignored artifacts/motion-after-45137ec.jsonl、navigation-45137ec.log、tcp-speed-buffer-live-analysis-45137ec.json。P4/RViz保留可操作。
