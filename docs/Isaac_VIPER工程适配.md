# Isaac VIPER 工程适配

本工程归属 `app/orin-unreal-simulation`，任务 `ISAAC-VIPER-MIDTERM-20261028 / engineering`，专题分支 `hk/isaac-viper-engineering-20261028`。它以项目轮式几何构建 4WS 动态骨架，引用 VIPER 外观，并接入项目既有控制器和真实 SDK 传感器。真值里程计用于隔离跟踪 bench；不代表 SLAM、完整规划/导航、Orin 或实车验收。

## 来源、配置与资源

- VIPER 来源：[Alex Maietta 的 lunar-surface-mobility-assets](https://github.com/alexmaietta/lunar-surface-mobility-assets)，固定 `v0.1.1`，作者 Alex Maietta，CC BY 4.0。这是社区复现模型。
- 原模型、原月球地形、旧展示场景保持只读。`material-substitution.json` 保留缺失原 grey normal 与明确采用 dark-normal 替代的记录；不宣称原始材质完整无缺。
- 几何/运动权威来源为 `config/wheel_orin.yaml`；仿真假设和传感器配置为 `config/isaac_viper.yaml`。总质量 50 kg、双目 0.14 m 是用户确认的待实机标定值。
- 规范骨架为 9 刚体、8 关节。底盘 X 向前、Y 向左、Z 向上；四轮顺序 fl/fr/rl/rr。零转向名义包络 `1.182 × 0.818 × 1.29996 m`，转向扫掠宽度另验，不能把名义 footprint 当全角度扫掠。
- 前置杆、双目和 RTX LiDAR 位于 local +X。深度点云是 RTX 深度反投影，不能称为双目匹配输出；LiDAR 原始/有效点云分开，IMU 使用原生物理传感器。
- 本地外部产物根 `D:/CodexDownloads/isaac-viper-midterm-20261028/engineering`；台式根 `E=/mnt/data/WS/lunar-navigation/analysis-artifacts/isaac-viper-midterm-20261028/engineering`。源码仓库只保存脚本、测试与文档。

## 身份与验证边界

| 层级 | 证据与实际边界 |
| --- | --- |
| 本机 Python | 最终 startup 修复后相关测试 172 passed / 11 skipped；新增3组19场景与既有8项 PXR 检查依赖真实 SDK，本机跳过不当通过；Task4原165/8为历史记录 |
| Windows 全仓 pytest | 真实执行收集失败：5 skipped / 71 collection errors；缺 rclpy、fcntl、ROS launch/包等，详见 coordination/task-4-full-pytest.log |
| Humble 构建/包测试 | 复用 source `2d9a8ce1620d951f110c36ab13e59d3bb478df16` 的独立构建：lunar_planning_msgs、lunar_pure_wheeled_controller，211 tests、0 errors/failures/skips；本任务未改这两个包，不重复构建 |
| SDK PXR | 最终493f206隔离源码CPU实跑11测试组全通过（旧8项+新增3组19场景）；base validator新增场景RED为13fail/0error/0skip，原v5 SHA前后不变；未加载Kit/ROS。Task2生成与旧13mutation保留历史 |
| Kit 物理/RTX、Humble DDS | Task3 六方向输入和轮组/根响应有实测，12 topics 有真实样本；严格停车矩阵部分失败，见下文 |
| 项目控制器短路径 | 两次均未通过：run1终端yaw误差.384357rad；run2仅bench no_progress_timeout_s=300，终端yaw误差.408577rad；源码和 Python 测试不能代替实测 |
| Jazzy、Orin、实车、真实标定、SLAM、完整自主探索、PLAN_FOUND | NOT_RUN；本次跟踪不提供规划成功证明 |

Kit 当前运行源码是 `750bc3bfecb7075e3b36ebd14c29021811151427`，胶囊 SHA256 `a8504b26c38d098022ef96af7a3739c333c3275df34d83af7fa44f53ae338de9`，与最终文档/验证器提交身份分开。Task4 对四个早期文件只补 EOF 换行，另两指定文件已存在换行；行为不变但字节 hash 不同，不宣称最终分支 HEAD 整体已经在 Kit 执行。

冻结输入场景 `E/viper-engineering-v5.usda` 的 SHA256 为 `c4354150205f8c2cf85cf669aec822bfc22afb703a3420d0e2e8ff5a5654847e`。唯一运行目录 `E/task3-gui-v14-matrix`，其 `runtime.usda` 继承源场景并恢复源物理材质定义/绑定；没有为凑停车结果改摩擦、轮径、根姿态或地形。当前 Kit PID 724575、精确 GUI XID `0x5e00009`、DISPLAY `:1`；重启后 PID/XID 必须重新核对。

## 启动和恢复

现存 GUI 已由 Task3 保留，Task4 复用，不启动第二个 Kit。不向其他 Kit、正式训练或真实车辆域发命令。

Kit 启动使用 SDK 自带 cp312，不能 source 系统 Humble cp310 overlay。新版恢复使用已核对的隔离源码目录（仅在现存实例明确关闭后重新建立会话）；此新版Kit/ROS集成尚未实跑：

```bash
E=/mnt/data/WS/lunar-navigation/analysis-artifacts/isaac-viper-midterm-20261028/engineering
cd "$E/final-contract-source"
export DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority
export XDG_RUNTIME_DIR=/run/user/1000 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
export WARP_CACHE_PATH="$E/task3-cache/warp" XDG_CACHE_HOME="$E/task3-cache"
export ROS_DISTRO=humble ROS_DOMAIN_ID=117 ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/home/kai/isaacsim-6.0.1/exts/isaacsim.ros2.core/humble/lib:${LD_LIBRARY_PATH:-}
/home/kai/isaacsim-6.0.1/python.sh -m tools.isaac_viper.run_scene \
  --stage ../viper-engineering-v5.usda --output-dir ../NEW_UNIQUE_RUN \
  --duration 0 --keep-open
```

`E/final-contract-source`逐项复用已接受的8个模块及3份配置，并替换本次493f206的`run_scene.py`/`validate_stage.py`和新测试；精确文件SHA见`E/final-contract-cpu-receipt.json`，不等于完整493f206仓库已部署。现存GUI的历史启动cwd仍是`E/task3-source`、源码750bc3bf，原argv保留于`status.json`/启动log；旧命令不得当作带新startup门的恢复入口。

输出目录必须是新的明确路径，不能覆盖既有证据。运行精确环境与 argv 以 `status.json`/启动 log 为准。Pause/Resume 可继续；Stop 会锁存会话失效，不能再 Play 复用旧 tensor view，需关闭该实例并按原命令建立新会话。GUI 预览只调整 `/World/EngineeringView`，不为构图传送 actor。

系统 Humble cp310 的控制器在独立终端启动；既有 overlay 在 `E/humble-install/setup.bash`：

```bash
source /opt/ros/humble/setup.bash
source "$E/humble-install/setup.bash"
export ROS_DOMAIN_ID=117 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
mkdir -p "$E/ros-log"
export ROS_LOG_DIR="$E/ros-log"
ros2 run lunar_pure_wheeled_controller lunar_pure_wheeled_controller_node.py --ros-args \
  -p input_mode:=incremental_path -p use_sim_time:=false \
  -p platform_config:=/mnt/data/WS/lunar-navigation/lunar-runtime/.worktrees/orin-unreal-simulation/config/wheel_orin.yaml \
  -p map_frame:=map -p odom_frame:=odom -p base_frame:=base_link \
  -p path_topic:=/Car/T4/planning/local_path \
  -p odometry_topic:=/Car/T3/localization/odometry \
  -p command_topic:=/Car/T5/Car_Cmd_Vel -p tf_topic:=/tf \
  -p control_rate_hz:=20.0 -p odometry_timeout_s:=10.0 -p tf_timeout_s:=0.5 \
  -p max_linear_mps:=0.2 -p max_angular_radps:=0.5 \
  -p goal_position_tolerance_m:=0.2 -p goal_yaw_tolerance_rad:=0.2
```

这是 **wall-scheduled controller bench / use_sim_time=false**。低 RTF 约 .055、最大 render 7.0808 墙钟秒，故本次明确覆盖 odom freshness 为 10 秒；正式源码/YAML 默认值仍为 .5 秒。控制器在 render 期间会用缓存反馈重算，不能宣称每条命令都基于新物理采样。dt 相关 slew、无进展 15 秒按墙钟工作，不能据此证明仿真时间加速度约束。TF .5 秒、驱动命令 watchdog .5 秒均保持原值。

`wheel_orin.yaml` 已核对 hash `9e54ba40f15b41d1a93d5c526e1abc76696bb6784d066294929818ad8f2de3be`。命令实际是原项目 controller 回调重算，不使用 Twist relay。Path 模式不发布 TrackingStatus，不能伪造内部 COMPLETED；公开验收只使用独立反馈判据。

## 短路径验证器

`tools/isaac_viper/verify_ros.py` 可单文件运行，不要求向 Kit 上传或覆盖生产源码。当前同步身份为 source commit `fad8994f758855e4bc37f81ba739ce384a9fec68`、SHA256 `fd45ff464993c88c568f77d84ee10699859acdef02f83e7af9aa6aa11859ca51`，文件 `E/task4-verify_ros-r1-i1-i2.py`；主线程已核对单文件同步和主机Python3.10编译PASS，原始回执为 `E/task4-verifier-r1-sync-receipt.json`。旧0b99、74f7文件与两次真实run保留，当前header及r1校验的ROS集成仍NOT_RUN。运行前核对 domain117 只有本任务 Kit、没有旧 fixture/命令 probe；启动后只允许一个项目 controller 发布 cmd。

```bash
python3 "$E/task4-verify_ros-r1-i1-i2.py" \
  --telemetry "$E/task3-gui-v14-matrix/telemetry.jsonl" \
  --output-dir "$E/NEW_UNIQUE_CONTROLLER_RUN" \
  --fixed-identity-tf --odometry-timeout 10 --wall-timeout 240
```

验证器先订阅真实 odom/clock，检查帧、有限值、归一化四元数、初始新样本停稳与 controller graph，然后沿当前朝向发布一次 `.8 m` 直线路径，目标再偏航 `.35 rad`，RELIABLE/TRANSIENT_LOCAL depth1。它自己订阅 Path 捕获清除/替换，不发送 Twist，也不在退出时清 Path。240 墙钟秒是单次 run 上限；失败不自动续发/重试。run1使用默认 no_progress_timeout_s=15；后续主线程裁定仅run2覆盖300秒（约15sim秒/实测RTF），单独记录启动参数和成本，不改正式默认或安全watchdog。run2仍从真实当前位置生成新的相对.8m/.35rad路径，不能当相同起点的A/B比较。

`--fixed-identity-tf` 只适用当前 simulation truth world 与 odom 一致的固定 map→odom，墙钟 20 Hz 发布同一 identity；Kit 原边也必须为相同 identity。它不是持续重发旧地图修正。任何非 identity 变换立即记录异常，不继续覆盖。

输出 `events.jsonl` 保存 ROS 回调接收时刻、原始仿真 stamp 与当前 Kit 逐物理步的根/关节、真实 rx_wall/rx_sequence/targets，以及 render 时序。遥测从本 run 开始时 EOF 读取；不借用旧矩阵物理样本。`result.json` 保存判据、终端误差、最大横向偏差、实际位移/yaw、速度峰值、物理关节峰值、真实反馈最大间隙、超时/拒收数和实际 wall/sim 耗时。

通过要求同时满足：实际前向位移至少 .5m，有非零线/角命令及实测关节响应；最终 odom 与 root 保持位置误差≤.2m、yaw≤.2rad；最终连续尾窗至少 1sim 秒且 1wall 秒、至少 20 条连续零 cmd、最大命令间隙<.5wall 秒。每个新 odom/root 速度≤.01m/s、yaw≤.03rad/s，八关节速度≤`.01/当前轮半径` rad/s。该关节界限是本 bench 的额外保守停稳门，不修改执行器门限。

尾窗至少 3 个递增且相互匹配的 odom/root 原始 stamp，并跨过真实 render；重复缓存不累积仿真停稳时长。cmd 时刻 odom 接收 age≤10秒、TF≤.5秒，全 run 新物理反馈最大间隙必须<10秒。退出前滑出容差、清空/替换 Path、拒收/acceptedFalse、断流或仅 timeout 零目标不能冒充成功。失败保留全部原始证据；控制器中止后的零包单独作为收尾，不纳入到达尾窗。

只读复算：`python3 tools/isaac_viper/verify_ros.py --analyze /absolute/path/events.jsonl`。默认输出 stdout JSON，成功 exit0，未通过 exit2；不要用 exit2 文件替换原记录。

## 已知真实限制与保留资源

Task3 六方向输入响应均有动态证据，持续输入阶段拒收/误 timeout 为 0；IMU100、双目15、LiDAR10 是仿真 Hz，不能写成墙钟频率。DDS 大图像接收约7–8.4sim Hz，与传感器 native15sim Hz 分别记录。

Task3 `.4sim秒` 严格逐步 `.01m/s/.01rad/s` 尾窗：stop1/2/3 通过，stop4/5/timeout 未通过。long-no-command 固定2sim尾窗也非全通过，存在真实漂移和轮速尖峰。源日志和诊断保留，不宣称全部停车矩阵通过，也未确定只有摇摆或接触噪声。真实 `.5秒` watchdog 是在下一可执行物理步按原接收 age 判超时，不是渲染阻塞期间硬实时停车保证。

最终真实预览必须独立保存全车与前置 sensor 构图，并由主线程下载实际查看；旧 `viper-hero-r2.png` 和旧 final 目录不代表工程版。主线程已实际查看E/task4-preview/hero-window.png并确认全车未裁切、前置杆/RTX LiDAR/双目外壳可见，EngineeringView保持当前暂停视角；最终材料见下文。

工作树、当前 GUI/运行胶囊、模型/材质来源、Humble overlay、构建目录和全部失败/成功产物保留，状态“已提交待合入 / 运行依赖保留待回收”；不合并、不推送、不创建 PR，不擅自删除或批量清理。

早期小型 pytest YAML 夹具曾由工具默认生成在 C 盘：`C:/Users/77634/AppData/Local/Temp/pytest-of-77634/pytest-58`、`C:/Users/77634/AppData/Local/Temp/pytest-of-77634/pytest-59`、`C:/Users/77634/AppData/Local/Temp/pytest-of-77634/pytest-60`。用途为临时 YAML fixtures，审查结束后可由用户手动逐项清理；本任务没有删除它们。后续测试 basetemp、日志和导出均在 D 盘任务根。

补充历史工具产物：一次sandbox pytest尝试在 `C:/Users/77634/AppData/Local/Packages/sandbox.{cbd5d30f-f1db-4ef8-bafa-f068f0cc672c}/AC/Temp/pytest-of-77634` 创建临时夹具目录失败；是否残留及可删除性不可确认，本任务未删除。早期SDK默认日志 `/home/kai/isaacsim-6.0.1/kit/logs/Kit/Isaac-Sim Python/6.0/kit_20261029_002959.log` 和共享 `/home/kai/.cache/warp/1.13.0` 也保留。后续已用任务独立 `/mnt/data` 日志/cache路径；不自动清理SDK共享cache或历史log。

### Task4 run1 原始结果

`E/task4-controller-run1/{events.jsonl,result.json,diagnosis.json}`、`E/task4-controller-run1-launch.json` 与两份stdout保留。verifier运行240.0437wall秒，含进程收尾240.6085wall秒；有效路径后238.9577wall/13.05sim（RTF约.05461）。真实前向+.608843m，yaw变化-.034357rad；终端位置误差.191417m、yaw误差.384357rad，故未完成。4780条cmd、2101非零、1306个新odom/root，实际最大cmd间隙.108554wall秒、反馈间隙1.46411秒，拒收/timeout样本均0。

最后观测投影progress提升>.01m在Path后90.065255wall秒（progress.624562m），持续零cmd开始于105.057728秒，相差14.992473秒；与默认15wall无进展机制强相关，但本模式没有公开内部status，不能把相关性写成实收TRACKING_STALLED。源码链：TrackingPolicy默认15→node遍历fields声明并读取参数→TrackingPolicy(**policy_values)→PathExecutor；普通Path progress提升>.01才重置，否则按wall dt累积。零前5.66sim内Drive waiting_for_stop2.05sim、tracking3.37sim，另有转向/制动。

result中的controller_command_gap等generic尾窗reason表示“满足目标与停稳条件的最终尾窗不足”；yaw持续超差使尾窗不断重置，末尾1cmd/0physical不代表整次只有这些数据，更不代表全run20Hz命令真实断流。最终失败结论同时依赖公开误差和原始反馈。

本次ROS默认空日志另保留在 `/home/kai/.ros/log/python3_738682_1793218709743.log`、`/home/kai/.ros/log/python3_738712_1793218709940.log`、`/home/kai/.ros/log/python3_745077_1793219139014.log`，以及 `/home/kai/.ros/log/python3_745049_1793219138960.log`（run2 verifier），核对时均0字节；未清理。后续控制器恢复命令显式设置任务ROS_LOG_DIR，stdout/回执已在E。

### Task4 run2 与离线诊断

获准run2仅增加启动参数 `-p no_progress_timeout_s:=300.0`，独立输出目录 `E/task4-controller-run2`，launcher/stdout同前缀，未第三次GPU运行。verifier240.0491wall秒、含收尾240.6715wall秒；239.8270wall/13.29sim。真实前向+.659146m、yaw变化-.058577rad；终点位置误差.141251m、yaw误差.408577rad，仍未完成。4797cmd/1463非零，1330新odom/root，actual最大cmd间隙.098134wall秒、反馈间隙1.471323秒，拒收/timeout均0。

全程最小位置误差.108742m，未有样本进入内部.1m目标带；纯angular cmd为0。持久零cmd从Path后73.177058wall秒开始；全run cmd处odom最大接收age1.470843秒、直接TF最大age.105932秒，均在本bench门内，不能解释为freshness断流。

两run的 `offline-replay.json` 使用未改、已构建原控制器PathExecutor，按observer回调时序近似tick：全部4780/4797条预测cmd与实收cmd最大分量差均≤.005，最大偏差各.0001613/.0025875。run1在105.057728秒回放得到TRACKING_STALLED（no_progress15.000044秒，k=.8211<1）；run2在73.177058秒先BRAKING（k=-.8094），79.226995秒回放CURVATURE_INFEASIBLE：k=-2.50728超能力1、航向差.062969≤.08、位置差.110175>.1。该结果是高度一致的离线重建，不能称为live内部状态实收；未改算法/能力/goal凑通过。

### fixture时基修复身份

run1/run2实际使用的0b99cb3/c86d35文件，Path及fixture固定TF header来自node墙钟；Kit原TF/header来自sim clock，因此这两次run存在同域混合时基事实。现controller只依direct TF接收age、Path不按header过期，故仍能做本次跟踪观测；这不证明tf2消费者时间契约通过。

运行后最小修复提交 `74f7da5633a4ee183972085d1115c988eabba4df`，该历史版本verifier SHA256 `f46681bf0845de65e3de41e04173ee391aa53057a8354c4191f8ab12d25e8b92`：Path/固定TF转写已观测latest /clock；render停顿只重复真实simstamp，不凭墙钟伪造新采样。6个新增RED后GREEN，全相关130passed/8skip。该新header行为只有本地纯Python验证，ROS/Kit实跑NOT_RUN；父已单文件同步新74f7文件并做主机cp310语法编译，未再启动ROS/GPU或改写旧run结果。当前主示例已改用后续r1文件名与新输出占位；两次历史精确argv以各launch回执为准。

### 最终真实工程预览与恢复

当前唯一Kit724575/XID0x5e00009保留在Pause153.47sim，真实根姿态未传送或摆拍；原始v5输入hash仍c4354150205f8c2cf85cf669aec822bfc22afb703a3420d0e2e8ff5a5654847e。两个Task4 controller进程组738681/745048均已SIGINT正常结束，domain117只剩原Kit与launcher。没有Stop latch；Play可恢复Pause，但要重新启动controller并有新鲜Twist。若点击Stop则必须按原命令重建运行会话。

`E/task4-preview/display-receipt.json` 保存暂停事件、冻结root pose、无新physics时长、进程/场景hash及恢复提示。Stage ProjectVIPER选择、Frame与3格wheel dolly只调整EngineeringView；早期框全地形的失败PNG保留，不当合格材料。后续selected/hero真实PNG由主线程下载实际查看，黑→可见/Frame延迟机制没有被当作已确定原因。

最终材料在同一`task4-preview`独立目录，不覆盖旧final/或viper-hero-r2.png：

- `hero-window.png`：真实精确窗口原图3840×2096，SHA256 `22b03b403c1ebad355c878a9dd1fd89614f6ba82554a0d2209af415b1285ef86`，父实际view确认全车与前置安装可见。
- `hero-viewport.png`：从原图ROI `(x900,y420,w1000,h850)` 逐像素提取，全车含少量地面，1000×850；文件SHA256 `6ddbb5e8e91127233588085b396c3a085318b004f68ee38a9374a58502d54a33`。
- `front-sensors-viewport.png`：原图ROI `(x1400,y500,w320,h550)`，320×550的前置sensor近景；SHA256 `ad82ed250413fc163d86015c644ae3e126233a8a4753f5964510ac3436d90246`。
- `viewport-roi-receipt.json`：记录原图、区域、文件和像素hash；是原始渲染像素选区，没有AI重绘、代理数据或追加物理试验。原XWD/PNG及各相机操作回执保留。

最终处置：代码/文档已提交待合入，短路径两次验收仍FAIL，source module实现/代码测试与物理策略效果分别记录。Task4限定复审已Approved；全分支独审完成并提出startup I1和入口M1，本波实现后仍待限定核验；不将展示图当作到达成功，也不将新header纯测试当集成通过。

主线程已把两张ROI取回并实际view：全车完整（远侧轮自然被车体遮挡，未被ROI裁切），前置杆、顶置RTX LiDAR和横向双目housing清楚；前sensor近景也合格。交付副本：`D:/CodexDownloads/isaac-viper-midterm-20261028/engineering/viper-engineering-hero.png`、`D:/CodexDownloads/isaac-viper-midterm-20261028/engineering/viper-engineering-front-sensors.png`。保持现有paused GUI/EngineeringView，不再调视角。

### Task4 独审修复 round1

I1修复按接收时间维护全记录严格递增stamp高水位；重复包不刷新最后真实新采样wall，倒退为硬FAIL且不回退高水位。间隙包含路径开始到首个新样本（若已观测路径前新样本，则取最后该样本作为freshness锚点）、新样本之间和末个新样本到最后cmd/odom/physics观测边界；仍严格要求<10秒。237条缓存重发后恢复的回归报告11.8秒并FAIL，短重复报告.8秒且PASS。

I2修复直接map→odom七个分量先全部finite再校验identity；健康tf事件保存`transform_xyz_xyzw`。非法原分量以repr字符串留在严格JSON可保存的error事件，回调记录后中止，不生成健康tf刷新；离线分析对已有raw字段也复验。旧记录缺raw时不能事后认证该有限值契约，未假称两次旧run发生这些漏洞，也未更改其FAIL/SHA/成本。

本轮仅本机纯Python：I1最终聚焦34passed，I2最终聚焦64passed，全相关165passed/8SDKskip。无ROS/SDK/GPU/GUI动作，header与本轮校验的ROS集成继续NOT_RUN；Task4限定复审已Approved，修复版单文件已由主线程同步并核对cp310编译PASS，原74f7已同步身份仅属上一版。报告追加于coordination/task-4-report.md，复审范围为I1/I2修复diff。

r1修复版当前源码SHA256为`fd45ff464993c88c568f77d84ee10699859acdef02f83e7af9aa6aa11859ca51`，已同步新文件`E/task4-verify_ros-r1-i1-i2.py`，源提交fad8994、主机cp310编译PASS及ROS集成NOT_RUN见`E/task4-verifier-r1-sync-receipt.json`；当前主恢复命令使用该文件；不覆盖0b99或74f7历史文件，输出目录仍必须新建。本轮ROS集成未执行。


### 最终 startup stage/profile 合同门

全分支I1修复使用既有只读 `validate_stage`，在 `SimulationApp` 完成SDK/PXR初始化后、打开用户stage及创建 `RealSensors`/`DriveController`/`RosBridge`/物理Play之前重开实际输入USD。输出目录中的 `stage-profile-contract.json` 保存输入场景SHA、实际加载profile的semantic SHA、测量/错误和验收边界；读取异常或不匹配均保存失败诊断并终止，不能启动命令接收。指纹仅为证据身份，匹配结论来自实际PXR几何/物理/相机属性，不要求改写旧v5。

原几何、质量、关节、外参与基线检查继续生效；新增检查双目实际焦距、水平/垂直孔径、perspective和零孔径偏移，IMU/LiDAR与车体对齐的朝向、千克单位以及实际chassis COM与运行端消费的metadata。分辨率由CameraSensor显式设置，支持的RTX频率由tick_rate设置；IMU当前固定100Hz物理采样，RTX频率不得超过15Hz渲染调度，未知sensor implementation拒绝。有效观测范围仍由ROS桥过滤原始数据；配置范围不代表底层传感器超范围能力保证。

本机172passed/11SDKskip；冻结v5真实CPU/PXR已完成：原profile通过，轮径/轴距/轮距/外参/基线/焦距/双孔径8个合法profile变更全部拒绝；实际光学与frame/metadata/unit共10个session/匿名层故障全部拒绝。源码493f206在`E/final-contract-source`隔离快照运行，11测试组0fail/0error/0skip，回执`E/final-contract-cpu-receipt.json`和RED/GREEN日志、`E/final-fix-cpu-{red,green}`逐案JSON保留；冻结SHA前后均c4354150...。新版runtime的Kit/ROS集成仍NOT_RUN；现存GUI继续运行旧750bc3bf/task3-source，不能把新源码CPU检查写成该实例已升级；主恢复命令已切到含新门的final-contract-source，只有本实例明确关闭后才新建会话。最终波限定复审尚待，不预记通过。


### 2026-10-29 当前入口：原 A/B/C 场景与真实新导航同视窗

用户最终选择 `original-source-v1`。当前运行场景应直接保留原 A/B/C 地形、岩石、材质、
碰撞和区域相机；历史 `uniform1m-rock80-v1`、`uniform0p5m-rock320-v2` 只作历史对照。
构建器 API 为兼容历史仍缺省 `quality='light'`，当前入口必须显式传 `--quality original`。
`light` 新版为 0.5m / 8,000,000 三角 / 每岩石320三角，不能再写成1m/80。
当前 viewport 来自回执的 `viewport_size`（1280×720），双目仍是640×480@15Hz。

已冻结原展示根层 `/mnt/data/WS/lunar-navigation/analysis-artifacts/isaac-viper-showcase-20261029/original-v1/viper-showcase.usda`，
SHA256 `e0573e650232211b6ef0c0281fe644028855a95f823eda7f9db9d51a9e5eb70e`。
原场景车辆初始在C，切A/B相机不会移动车辆。A真实导航必须使用独立生成的新初始位姿层与新运行会话，
不能把旧C车辆加A镜头算作A执行。以下命令都是新入口示例；SDK/ROS实跑结果由每次独立回执确认。

```bash
cd "$REPO"
# SDK CPU/PXR；AOUT 必须尚不存在。原 stage / metadata 不写。
"$ISAAC_ROOT/python.sh" -m tools.isaac_viper.navigation_scene \
  --source-stage "$S/original-v1/viper-showcase.usda" \
  --profile config/isaac_viper.yaml --output-dir "$AOUT"
```

生成器只在 A `[235,335,-315,-215]` 内搜索16×16m近场与4m候选起终点，按真实启用碰撞
Mesh、Rock实例世界变换裁剪三角/栅格，0.2m绝对高程取每格最高支撑；未知保留NaN，不抹岩石。
`navigation-scene.json` 保存源层/NPZ/USD哈希、格内高程包络误差界、碰撞来源、
wheel_orin包络外接圆+.5m余量与保守坡度/起伏检查、实际三角落位和 `NOT_RUN`。
候选直线仅用于选择起终点，从不作为规划消息发布；正式路径来自新导航 Action。
本地图明确是A已知图 `known_map_from_same_scene_collision_geometry`，禁止注入C或Actor。
静态几何核对不能代替运行时PhysX接触/落稳与碰撞验收。

```bash
# Kit cp312 独立进程，不 source 系统 Humble/host overlay；KLOG 必须尚不存在。
ROS_DOMAIN_ID=117 ROS_LOCALHOST_ONLY=1 \
  "$ISAAC_ROOT/python.sh" -m tools.isaac_viper.run_scene \
  --stage "$AOUT/viper-navigation.usda" --profile config/isaac_viper.yaml \
  --output-dir "$KLOG" --duration 0 --keep-open \
  --navigation-overlay-file "$OVERLAY"
```

`OVERLAY` 为显式新会话JSON路径。SDK `omni.ui.scene.SceneView/Curve` 仅绘制viewport叠加，
全局蓝、局部黄、真实odom轨迹青；原始坐标保存在JSON，显示统一抬高.12m避免路面遮挡。
没有路径USD Mesh进入RTX/碰撞。每app帧poll，因此Pause可继续更新；Stop仍结束旧控制会话。
近景相机按实际起终点/初始高程生成，不改车辆运行位姿；保留Vehicle/All/A/B/C按钮。

新导航必须由独立、同源码身份的Humble cp310完整闭包构建，旧仅controller的install不足以启动它。
启动正式导航和唯一项目控制器的参数参照 `config/incremental_navigation_interfaces.yaml`、
`config/wheel_orin.yaml` 与本任务构建回执；导航使用 `adaptive_known` 和真实T3输入，控制器为唯一T5发布者。
不要运行旧会手造local_path的 `verify_ros` fixture，也不运行合成terrain/运动emulator。

```bash
# 单独 host shell：source /opt/ros/humble/setup.bash 及本次独立新导航 install/setup.bash。
# 不与 Kit cp312 混 Python overlay。HLOG 与 OVERLAY 必须尚不存在。
ROS_DOMAIN_ID=117 ROS_LOCALHOST_ONLY=1 python3 -m tools.isaac_viper.navigation_showcase \
  --scene-json "$AOUT/navigation-scene.json" --output-dir "$HLOG" \
  --overlay-file "$OVERLAY" --send-goal --wall-timeout 180 --sim-timeout 30
```

缺省不加 `--send-goal` 时只发布A图、监听原消息与真实轨迹；加开关后先核验clock、两次可靠持久图、
递增odom/同stamp直接TF与起点位置，至多发一次真实 `NavigateToPose`。
`--freshness-wall` 缺省2s仅是助手发目标前的接收健康门，不改变正式控制器门限。
地图header为odom，规范layout与反转双轴编码，绝对elevation/NaN保留。
真实轨迹topic `/Car/T4/showcase/actual_trace` 使用原stamp，拒绝倒退和非法pose/TF；
源odom、TF、diagnostics、PathReference、Action feedback/result保留在 `events.jsonl`。
`result.json` 中 `planning_outcome=0/has_reference=true` 只能从同Action session、同segment/
traversability revision 的原 `PLAN_FOUND` 与非空ACTIVE reference派生，不能当成原诊断字段。
Action `outcome=0/reason_code=GOAL_REACHED`另判；图上有线、accepted或ROS SUCCEEDED均不代替到达/严格停车验收。
助手有wall/sim界和有界取消；取消回执、真实尾窗/唯一控制器停车仍由root核对。

本轮本机相关纯Python测试197 passed、17 SDK skipped（另5 subtests passed）；3个CLI help已执行。
实际PXR地图生成/派生USD门、Kit SceneView显示、Humble导航/控制闭环、目标到达及停车均 `NOT_RUN`，
由root在独立新运行补记。C Actor、完整三区导航、Jazzy、Orin、实车也未由本轮证明。
原两次工程短路径失败与原/轻档全部历史产物保留，不借其结果替代新导航验收。
