# P3 预检与运行时时间合同对齐

同任务 P3-POSE-WAIT-20261030 / INPUT-TIMING-ALIGN，公共owner integration/pure-planner-orin，专题 hk/p3-pose-wait-20261030，基线7dfa199，main-worktree，ROOT单写入。用户要求区分源数据延迟与接收断档，并指出帧间隔2–3秒；恢复条件本轮不改，方案待讨论。

定位预检读取实际planner/controller参数并核对一致，按odometry_timeout_s同时检查源龄和接收龄。direct_p3地图沿运行时合同只按p3_map_timeout_s检查有效接收龄，0保留已有静态图策略；不再用5.5秒源龄拒绝地图。没有新增消息、修改产品规划/跟踪/认证算法、放宽运行时预算或改写源戳。预检不能替代地图有效解析、几何认证和停车检查。

p3_input_timing复用现有观察字典，常数空间分别累计到达源延迟、已闭合接收间隔的count/min/max/mean，另报当前源龄、未闭合接收断档和源戳停滞年龄。check_tcp_navigation为可复用P4有界检查入口，预检使用运行时参数，保留2秒及源戳推进条件；--preflight-only不获取租约、不发目标或车辆指令。显式TCP profile的无控制观察入口采用相同合同；普通观察模式保留原预算并标记observation_only。地图订阅已对齐运行时可靠/保留QoS。

验证：初始8失败/38通过；实现后本机101项通过、3脚本AST检查通过；216 Humble关联127项通过，含实际planner时间判定及地图接收门禁的一致性回归。Windows扩展配置测试9项因缺scipy失败，对应测试在目标机127项内通过；未安装依赖或伪装为本机通过。目标机首个命令漏传测试文件，0项执行，补齐后正式127通过。

首次实际只读预检：定位68条，到达源延迟1.976–3.629秒、均值2.581秒；接收间隔最大2.929秒、均值0.788秒。窗口内未收到地图，地图统计count=0不能解释为延迟0；预检未通过。control_authority_acquired=false、0非零、TCP位移0、遥测停稳、launch退出0。证据P4/run/p3-input-timing-align-20261030及P4/artifacts/p3-input-timing-align-20261030。

发现旧检查地图QoS为volatile，随后源码改为与运行时一致的reliable/transient_local，准备r2复验时SSH被关闭、拒绝连接后超时。用户确认机器正在重启，因此停止重试。最新QoS改动仅静态核对通过，最新远端同步/运行状态未确认，不宣称r2或完整导航通过。P3/Env_X未改，机器恢复后再核对源SHA及只读预检。处置：本地已提交待复测/合入，长期线未移动，资源保留，不push、不删除。
