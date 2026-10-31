# NAV1CM全局参考整形：实现与TCP交付（2026-10-30）

本会话实现、回归与候选交付已完成；生产部署、实时RViz和车辆验收由用户授权的TCP会话单入口接管，尚未由本会话执行。不要把离线曲线验证写成现场闭环通过。

## 身份与职责

- 任务NAV1CM-REFERENCE-SMOOTHING-20261030；公共owner integration/pure-planner-orin。主树D:/Codexworktrees/nav1cm-reference-smoothing-20261030/Lunar-Navigation，分支hk/nav1cm-reference-smoothing-20261030，基线7dfa1990bc4e5671d14c0367fffc5f7262e03215。
- 产品链：180d644几何、b91ed61接入、340463d长直线、32ab25f预算、859d406同图证明、834112a批量等价优化、2338d7b包测试夹具。几何子树最终d3082efc0d45ea3779de3058cbbbff632e0b79e6；两树均保留，不push、不移动长期线或删除。
- 全局搜索仍输出原始路线；纯几何模块输出Bezier采样参考和连续凸包证据；同一个self.route供extract_reference/MPCC和global_reference显示使用，raw_route独立保存。MPCC、动态约束、整车/制动认证、控制器产品代码未修改。
- 当前图重验是整形前提。新图同时复核折线和曲线证据；失败退休整条参考并重算，避免同版本换形后误接旧局部结果，代价是一轮重规划/制动等待。周期等价比较原始路线，保持参考进度。

## 实现边界

相切二次Bezier：裁切不超过相邻边1/4和0.75m；曲线叶控制多边形长度<=0.05m，原直线保留精确端点。曲线用向外32eps扩张的DeCasteljau凸包叶盒证明，采样折线另做闭区间连续碰撞检查；触边拒绝。

25ms是协作式墙钟预算，不是硬实时保证。深度16、总细分16384、4096新增参考点及64几何项×1024障碍分块均有保护。未证明raw且超时不能当安全回退；coordinator仅在同图raw清障之后复用证明。预算/几何失败回退原路线，不降低净距、动态约束、输入时效或执行合同。

新增ROS只读启动参数smooth_global_reference，默认True；不宣称公共launch已透传同名launch argument。global_reference_topic默认global_route_topic+'_reference'，解析后的raw/reference/local话题必须不同。等待/取消清空三条显示。参数、新消息发布和同图原路可信入口均有RED/GREEN回归。

## 验证

| 层级 | 当前证据 |
| --- | --- |
| Windows主线程定向 | 最终133 passed，1 skipped；无真实ROS/acados推断。 |
| 独立最终审查 | 164 passed，1 skipped；P1当前图复核及P2预算问题关闭，最终批量切片PASS。范围与上项重叠，不相加。 |
| 几何切片 | 43几何+39全局=82 passed；包含750m、短边/大坐标、连续曲线专属碰撞、raw未证明耗尽、recursive/scalar oracle与真实P3四点案例。 |
| 216隔离ROS接入 | 58 passed：三条消息/清空、只读参数、话题别名、MPCC实际输入一致和失效制动。批量数学切片之前的接入回归。 |
| Humble ARM候选构建 | 三个native库和Python包在artifacts下独立构建/安装成功；未写生产build/install。 |
| 隔离包检查 | 接入及预算版本10组CTest、46项0失败/错误；原日志isolated-ctest-r2.log。最终批量切片未重复该包检查，结合下面目标机数值验证交TCP做最终组合验证。 |
| 216真实P3快照 | 19障碍、用户前次目标[-14.40797233581543,-1.9836714267730713]，raw4点；批量版默认25ms连续12/12 SMOOTHED，68点、64proof、全部clear，6.31–10.69ms。没有ROS目标或车辆指令。 |
| 现场RViz/控制执行/到达 | 本会话NOT_RUN，按用户统一交TCP。实车/Jazzy/DRL均NOT_RUN。 |

性能修复前，真实P3案例8/8预算回退；0.5s只用于隔离取证，未改产品25ms。发现126次逐节点递归和逐边验证开销后，改为按层批量DeCasteljau、整数参数键恢复原叶顺序及分块连续验证。12次目标机实测源SHA88e03a784756332329bef4f3a60ff0a36468d524fd1592771426d18b3b002953（CRLF），与最终core b6de828a691542e7f258dcd0faf6509f74ad23bb755e438609beaa8291627b9d（LF）仅换行差异，已逐字节归一核对。

早期包失败保留：三Path夹具未更新；system Python用户NumPy1.25与SciPy版本警告干扰子进程协议；隔离安装路径误设site-packages导致部分用例载入旧包。已修夹具，用生产.venv NumPy1.24.4/SciPy1.10.1和正确isolated-install/local/lib/python3.10/dist-packages重验10组通过。几何agent较早Windows全core记录673 passed/126 skipped/6 native env失败保留，不宣称全仓库全绿。

## 单写入与远端身份

用户授权TCP统一最终/P4/source、ros2-humble/install/build更新、进程和车辆测试后，本会话仅本地和隔离候选；TCP运动期间停止远端SDK/CTest/性能测试。平滑源码未部署生产。

原22570/22572及旧RViz会话由外部退出；按用户控制器启用要求曾用最新域10入口恢复operator57429（planner57451/controller57453），P3单向57→10。TCP接管后确认该operator停稳，measured_stopped=true、forwarded_nonzero=0、navigation退出0。当前新车辆复测身份以TCP会话为准，本会话不再重启、停进程或发目标。

所有本轮远端写入均为/P4/artifacts/reference-smoothing-20261030和/P4/run/reference-smoothing-live-20261030及其日志；没有写P3/Env_X、没有改p4_tcp_runtime.py。最后候选性能回执p3-snapshot-batch-verification.json之后未再远端高CPU验证；生产源码更改均由TCP另行完成。

## 精确交付

- 本地包D:/Codexworktrees/nav1cm-reference-smoothing-20261030/Lunar-Navigation/artifacts/nav1cm-smoothing-handoff-2338d7b-r2.tar；SHA256 6c9e3c8fc1840174d63caa3db2c20e5e5ed6171dddbcca4c0fa6f6aa5aebfa9e，215040字节。
- 展开目录artifacts/handoff含README.md、manifest.json、payload-sha256.json、files/、validation-tests/、smoothing-current-production.patch、p3-smoothed-reference.rviz。manifest列10个生产源码/包测试文件，含基线和候选SHA256及6个Python安装路径。包不会自动执行部署。
- runtime基线450cfd20273c4e0bce89da73b214c39ba7a7710c0f89dd55ecde63de2a108c35，overlayd1432d0b0079f2a6a9a034c7b93d1ab83d664537b74e9b5693c096c65c01bac1；保留TASK_STATE、INITIAL_START_COMPLETED、LOCAL_PLAN_STATUS与定位活性语义，三个反馈/任务投递方法AST相同。
- overlay还保留initial_travel_direction只读及TCP平台限速断言。严禁用通用分支整文件覆盖新人工目标TaskSnapshot、navigation_task.py、standard_path_follower.py、p4_tcp_runtime.py、wheel.yaml或tcp_p3_feedback.yaml。
- RViz候选：灰色raw默认隐藏，青色/p4_p3_debug/global_route_reference，黄色/p4_p3_debug/local_path，/goal_pose入口不变；域10只使用现有已桥接的P3派生导航图/odom/TF。可选P3 elevation cloud默认关闭，不改跨域桥。
- TCP以b96277d及其后续源码为最终集成基线；若生产SHA变化，须重做精确patch合并，不能直接覆盖。完整命令/证据路径见包内README。

处置：本地已提交待合入，候选已交付TCP；最终生产组合验证、部署、RViz与车辆验收待TCP完成。主/几何工作树及ignored artifacts/SDD记录保留待授权回收，未删除。

存储例外：几何agent首次全套测试的小型临时产物位于C:/Users/77634/AppData/Local/Temp/pytest-of-77634/pytest-99，用于该次pytest；测试结束后可清理，建议任务验收后由用户按文件逐个清理，本会话未批量删除。其后所有本地构建/测试产物均显式D盘ignored artifacts。

交付r2已跟进TCP最新定位等待state(0,1)心跳与TASK_STATE字段精简；本地runtime overlay AST核对保留最新三个反馈/投递方法。p3_map_timeout_s=0不在本任务payload中，保留TCP配置。旧远端runtime候选只供历史测试，不能拿来覆盖最新生产；当前精确payload以本地r2为准。
