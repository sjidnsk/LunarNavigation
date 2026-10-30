# 终端参考接管、阶段完成与跟踪反馈合同

任务：GIT-PRESERVE-20261029/DRL-INFERENCE-CLOSURE/EXECUTION-CONTRACT。公共owner为integration/pure-planner-orin；实现复用hk/drl-integration-20261029，基线70ef6f3。长期线尚未移动，不推送、不训练、不改原安装。

## 职责和权限

- 全局规划生成绕障路线，不决定局部旋转阶段是否完成。
- 局部规划生成满足当前地图及运动能力的Path，并由任务所有者发布FOLLOW、BRAKE、HOLD、FINAL_ALIGN或INVALIDATED执行授权。
- 标准Path MPPI按有序参考执行并独立认证；普通FOLLOW不能吞旧屏障，显式终端授权才可退休旧平移阶段。停止/首次终端接管有实际停车与认证门槛，同session故障不能被新Path洗掉。
- Action最终目标在接受后不可在原session内修改；新目标使用新session。局部滚动Path末yaw只表示本次规划时域末端，不能充当最终目标身份。
- Path仍是唯一几何来源；新PathExecutionReference只传身份、意图及完成阈值，用时间戳和完整Path SHA256配对。收到/配对成功均不等于实际接受。

## 完成与反馈

终端位置、yaw和线/角停稳阈值由同一任务owner传至跟踪器。导航最终成功仍要求自身新鲜位姿、独立里程计停稳保持0.3秒，以及同session新鲜且已实际接受的终端WAITING/COMPLETED反馈。旧session、未知revision、摘要/intent错误、过期或回退反馈不能确认到达。

标准Path模式新增真实TrackingStatus回传，关联本地参考revision/digest/intent。跟踪失败先进入ERROR_STOPPING并取消规划，独立里程计确认停车后才Action INTERNAL_ERROR，并保留原始reason；实ROS故障注入已验证TRACKING_STALLED到ABORTED，不靠伪造成功状态。

Path/元信息允许双向乱序，有界缓存配对；传输临时停车仅在有效FOLLOW实际接受后解除，不清手动停止或真实故障。本轮变更ROS消息定义，使用方须在同一ROS发行版/overlay成套重建消息、规划器与控制器，不能混用旧生成接口。当前时间戳门禁要求ROS时间连续；/clock回退需先停止并重启合同进程，本轮不宣称跨时钟epoch自动恢复。

## 当前验证证据

源码/纯测试：广泛导航相关回归635 passed、126 skipped、6 deselected；跳过/筛除不作为ROS或native验收。协议及组合入口最新38 passed。最终控制器相关193 passed；隔离Humble的msgs/planner/controller实际构建和包测试545 passed，无skip；包含真实DDS/Action故障注入和标准Path双向配对测试。最终进展基准修复已重新构建、测试及真实闭环复核。

用户明确授权本轮源码上传到台式source-r4及新execution-r4目录隔离构建/运行。没有上传模型、原配置或数据；台式既有Actor/config/profile只读复用。

固定Actor SHA256：6951491300d144cb9e170859804707eb3613bd0fafd956ae205549eab0120f7d。
config SHA256：8d159d7e55fcb42aa23aef21900595199d8371f809543b3b83ef2ef1618deefb。
platform SHA256：23b93ebaf83466eb51991881cab045f14a80e871811b08f6dd39a5a768d91fe6。

r6：合法FOLLOW接受后仍保留EXECUTION_PAIR_WAIT临时停车；已有针对回归并修复，原失败保留。

r7：90秒，1个自然到达、2次真实Actor决策，完整闭环FAIL。首目标GOAL_REACHED：位置误差0.099520468m、yaw误差0.046134531rad、实测v/w均0，并有身份绑定的新鲜终端反馈。第二目标最终位置误差0.098642501m、yaw误差约1.997rad，超时后受控停车。80至89秒跟踪状态103帧BRAKING/未接受、77帧FINAL_ALIGN/已接受，真实角速度在约-0.025与0之间反复。原因是4秒截断终端旋转Path的末yaw变化触发重接管；此证据不能归因MPPI代价。

r7四个直接子进程exit0，无强制退出、残留worker或训练尝试，冻结输入hash未变。未完成第二次自然到达、后续决策及正常路径取消/复位门槛；失败清理不能替代这些验收。未扩大90秒预算，也未放宽容差。


r8：修复时域尾yaw误判及20Hz尾点堆积后，首目标再次自然到达（位置0.098879585m、yaw 0.015727766rad、实测停稳）。第二目标仍因TRACKING_STALLED失败，但这次故障经真实TrackingStatus回到导航，独立停稳后Action ABORTED/INTERNAL_ERROR，原reason保留；不再假装持续执行。完整闭环仍FAIL，原始结果保留。

r8进一步定位进展计量：第二session首次接受FINAL_ALIGN后1.5414秒内，xy完全固定，实际同向旋转0.57422162rad，当前固定stage朝向误差1.56631476降至0.99209314rad，尚未跨pi或到首阶段末端即报停滞。显式退休令path=None，接管函数因此判compatible，漏掉新阶段进展锚绑定；旧FOLLOW的近零best_angle与旧停滞计时被拿来比较新spin误差。正确修复是实际接管新几何阶段时重绑定方向/角度比较锚，不清累计停滞时钟、故障或命令历史；后续独立实测角进展才能按原规则清计时。连续同向滚动不重绑，静止刷新不能延期。此为阶段合同修复，不修改优化代价或timeout。

r8四进程exit0、无强制退出/残留/训练尝试、冻结输入未变，1713条参考和1707条身份绑定已接受状态落盘。额外execution-paths.jsonl保留每个Path摘要及有序yaw，可重放核对阶段。它没有完成正常路径的后续取消/复位，故不计完整闭环通过。

## r9：当前固定场景闭环通过

最终源码冻结包execution-final3-source.zip SHA256 f9f1cb39422f8b0fbd1cb735e78542ffcb6756b790b022b537a3670f2a5732c4；26个明确源码文件逐项SHA256上传核对，完整清单在仓库外execution-tests/execution-final3-source-manifest.json。台式source-r4与独立execution-r4成套重建，545 tests、0 errors/failures/skipped；控制器相关193通过，最后增量独立源码审查通过。

r9不调整Actor、原配置、容差、优化代价或90秒上限。2个不同目标在84.390398秒内自然GOAL_REACHED，分别位置误差0.099691505m/0.099471870m、yaw误差0.019542062rad/0.036322380rad，两个终态实测v/w均0。收到完成反馈后真实Actor继续决策，总计4次；第三个已接受任务取消成功，新任务复位得到不同session且新鲜反馈，再次取消成功。最终state=CANCELED、stopped=true、inflight=false，verdict ok=true/errors=[]。

1674条参考及1698条身份绑定已接受状态落盘（一个参考可有多个状态）；两个自然到达session都有完成前1秒内的新鲜终端已接受反馈。四直接子进程exit0，无强制退出/残留worker/训练尝试，原Actor/config/profile hash未变。证据本地execution-tests/run-r9及台式同名目录。

这只是本固定场景的完整DRL→导航→跟踪→反馈闭环通过，不等于执行架构已彻底解耦，也不等于已满足所有接近终点的平滑转姿态情况。用户随后要求转向本质分析：停止继续逐症状改动，保留本轮验证切片；稳定机动身份、完成对象、进展域及停车/超时生命周期的剩余结构缺口见[本质原因分析](2026-10-29-navigation-execution-root-cause.md)。不因单条通过轨迹自动合入长期线。
## 来源与边界

本地证据D:/CodexDownloads/lunar_navigation/git-preservation-20261029/execution-tests；台式源码/build位于analysis-artifacts/nav1cm-terminal-approach-20261029/{source-r4,execution-r4}；闭环原始证据位于analysis-artifacts/nav1cm-drl-inference-closure-20261029/run-r6及run-r7。失败档案保留，不覆盖旧结果。

本轮是真实ROS Humble消息、Action、acados/native规划与标准Path MPPI，运动与观测由隔离合成plant闭合。真实高程/打滑、Isaac在线、Orin、实车、正式训练均NOT_RUN。模块代码验收、策略效果和平台部署分别记录；r9通过仅赋予本固定场景运行证据；后续架构评审、公共修复拆分和长期线晋级仍待审查。