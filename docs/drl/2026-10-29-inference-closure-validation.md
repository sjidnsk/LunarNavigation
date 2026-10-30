# 固定 Actor 推理闭环验证记录（2026-10-29）

任务：GIT-PRESERVE-20261029/DRL-INFERENCE-CLOSURE。owner为dev/drl-exploration，执行分支hk/drl-integration-20261029。用户已授权脚本上传和隔离推理验证；禁止训练与原安装修改。当前闭环验收 **FAIL**，尚不可晋级owner、主线或应用线。

## 冻结输入与职责

Actor-only block-01 SHA256：6951491300d144cb9e170859804707eb3613bd0fafd956ae205549eab0120f7d。
原配置SHA256：8d159d7e55fcb42aa23aef21900595199d8371f809543b3b83ef2ef1618deefb。
平台SHA256：23b93ebaf83466eb51991881cab045f14a80e871811b08f6dd39a5a768d91fe6。
MPCC源码为公共导航eb10ff2对应source-r2；标准Path MPPI跟踪器，私有命名空间/lunar_inference/nav1cm_closure、ROS_DOMAIN_ID=92、localhost。12m合成平地、有限视野观测及有限加速度运动反馈，非实车或真实高程场景。

DRL仅选探索目标并消费新观测/导航结果；nav1cm全局规划负责可行路线；局部MPCC负责局部运动和末段姿态参考；标准Path MPPI负责执行已发布路径。当前建议不得把末段规划修复移入DRL或跟踪器。

## 原始窗口

全部远程证据保留于台式机/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-drl-inference-closure-20261029/；每轮使用独立脚本/输出目录，未覆盖失败窗口。

- run-r1：启动前命名空间检查失败。旧training命名空间构造器不适用于私有inference；验证器新增独立端点检查和启动前完整命令构造，产品启动器不变。
- run-r2：90秒，Actor产生一个目标，Action接受，全局PLAN_FOUND且has_reference=true，但局部worker因unknown encoding: future_fstrings初始化失败，未运动。原因是新sitecustomize遮蔽依赖目录旧钩子，未执行编码器注册。在线验收另因NumPy float类型与落盘JSON类型不同，错误拒绝有效目标身份。
- run-r3：恢复编码器注册、统一事件JSON规范化后，本地相关61项测试通过；远程codec lookup及acados_template导入通过，再执行90秒。Actor/配置/平台前后hash一致。车辆实际运动、地图revision从1到10，但首目标前停滞，无自然到达和下一次Actor决策，最终verdict.json为ok=false。取消/复位成功合同尚未验收；失败清理不能冒充正常取消验收。

r3目标为(0.10000000000000142,1.9000000000000021,3.9269908169872414)。最终停位(-0.011113089409385851,1.6860044490367556,2.0879774035927263)，v=w=0，距目标0.2411228203m；Action反馈WAITING_FOR_PATH。四个直接子进程均exit0，forced_exit=false、workers_remaining=[]、guard_attempts=[]；这只证明本轮退出与守卫证据，不证明闭环成功。冻结输入未变，不读取或修改原训练账本。

r3新钩子SHA256：cacfdfd6ff04f6c71be0773524b0b724b20a61b5eedcf5c268079870b2499a8b。
r3验证入口SHA256：a08926bc84dfeeb669874a8bbe2e3aaffe9bae6b2b3ed7273809e1826a3d8624。

## 冻结位姿诊断与讨论门

另以r3精确停位和停位→目标两点Route，在独立全空白500x500厘米网格运行一次真实acados诊断，debug-frozen-r3.log保留。select_mode=forward，行进目标yaw=1.0918826735。nominal/left/right三组均solver status=0，但均被NO_PROGRESS拒绝，candidate=null。这是无障碍静态复现，**尚未采集原在线局部图及每轮候选拒收轨迹**，不能据此断言在线停滞只有单一原因。

已向用户提出待讨论方案：局部MPCC按剩余距离与曲率限制提前最终朝向偏转；无进展时先恢复行进方向再接近；位置到达后原地补齐最终姿态。另一选项是保持现有行为，先补在线候选诊断。遵守“公共导航平移/终点朝向阶段冲突先讨论再实施”：尚未修改产品行为，不通过延长时间、放宽容差、换Actor或人工指定目标凑通过。

## 验证边界与下一步

纯Python回归61 passed；台式Humble真实ROS、native全局规划、acados和标准Path MPPI已实际运行，但完整推理反馈闭环FAIL。真实P3输入、坡地/打滑、Orin、实车、训练及策略效果验收均NOT_RUN。

待用户讨论末段策略后，补充原在线候选诊断与失败回归，再按已确认职责实施最小修复；定向验证后重新执行有界完整推理闭环。dev/drl-exploration与主线指针保持原值；工作树和失败artifact保留待继续。
