# 定位持续更新版本 TCP 导航测试

用户授权启动车辆控制测试。代码/安装身份1af35fa，owner公共integration/pure-planner-orin、TCP应用适配；复用P3-POSE-WAIT-20261030、hk/p3-pose-wait-20261030，ROOT唯一写入。仅P4运行产物，无P3/Env_X修改。

两轮使用P3真实地图/定位、既有UE TCP车辆接口，指令限速0.05m/s、0.1rad/s，UNKNOWN=true、4.5s源戳推进间隔/6s地图接收/140步/200ms完整建模包络。原稳定2s与源戳推进预检保留，参数实际回读并通过；没有恢复固定源龄拒绝。

| 运行 | 正式规划证据 | 结果 |
|---|---|---|
| tcp-pose-continuity-nav-20261030 | 68条planning_outcome=0、PLAN_FOUND、has_reference=true且generation一致 | 前方0.5m目标接受，75.107s内120条非零转发，TCP最大位移0.43985m；P3终点距离误差0.05590m。未自然到达，取消确认CANCELED_STOPPED、遥测停稳。 |
| 同名-r2 | 45条同条件正式规划证据 | 新Action身份、同一空间目标，从剩余约5.6cm处补充末段测试，上限120s；67.973s自然GOAL_REACHED，status4/outcome0；62条非零转发、TCP位移0.04448m，P3终点距离误差0.009349m，遥测停稳。 |

不能把两轮拼成一次连续0.5m自然到达。第二轮只更换测试时限及固定同一目标，不改算法、限速或认证；脚本仅P4/artifacts/p3-pose-continuity-20261030/same-goal-120s.py，语法检查通过。9.35mm是P3位姿下的目标距离，不是独立地面真值定位精度。

首轮定位到达延迟1.787–3.227s、最大接收间隔2.268s；地图到达延迟3.853–4.812s、最大间隔3.116s。第二轮分别为定位1.653–3.151s/最大2.013s、地图3.747–5.077s/最大3.695s。未因正常源延迟阻断预检或跟踪。

仍有推进慢、WAITING和STALE_PATH，以及少量认证发布过期/地图切换延后，需另行分析，不能宣称性能或所有故障恢复已通过。命令通过脚本边界检查；遥测vx绝对峰值首轮0.13667m/s、次轮0.06295m/s，不能把指令限速当实测速度上限，也不凭该记录断言原因。

两轮结束均park、释放外部租约、launch返回0，本轮命名空间进程退出，6665/6668仍ESTAB；不遗留无人监督控制。启动/跟踪两轮通过，到达仅第二轮通过，实测停稳两轮通过；长距离绕障、定位真中断注入、其他场景和实车NOT_RUN。

远端P4/run/tcp-pose-continuity-nav-20261030及-r2保留result/events/launch；本地D:/CodexDownloads/lunar_navigation/git-preservation-20261029/execution-tests/同名目录保存副本，r2有derived-summary.json。源13项/安装4项哈希再次匹配1af35fa。记录已提交待合入，不移动长期线、不push、不删除。
