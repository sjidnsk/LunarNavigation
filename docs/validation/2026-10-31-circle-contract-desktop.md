# 圆包络去重：台式真实acados隔离验证

源码e98572b，基线模型来自38b3677。用户于本轮授权连接远程台式验证。连接kai-10-63-250-129，主机KAI/x86_64；不是TCP调试机yanfa-216。

## 目录与范围

台式唯一新增验证根：/mnt/data/WS/lunar-navigation/analysis-artifacts/nav1cm-terminal-approach-20261029/circle-contract-20261031/。

- source/：本轮Python源码、测试及一个旧baseline_local_mpcc.py。
- generated/{baseline,single-circle}/：真实新旧模型生成C代码/库，按模型指纹隔离。
- pytest-generated/：组件pytest的独立模型缓存。
- validation.log、pytest-r2.log、pytest-r3.log、model-results.json：原始过程与结果。
- manifest.json、manifest-r3.json：初始79项及补齐后的80项源码SHA256。

本地原始结果与脚本：D:/CodexDownloads/lunar_navigation/circle-contract-20261031/remote/。不提交产物。仅上传必要Python源码/测试/验证脚本，不上传模型、项目配置或数据；config/wheel.yaml在隔离目录只读复用台式旧source-r4配置。原安装、旧模型缓存和其他项目不修改；不启ROS导航节点、不发车辆指令、不运行Actor或训练。

## 环境与真实模型对照

复用既有env_desktop_humble.sh，仅作为环境入口；缓存、日志与输出重定向到本轮根目录。Python3.10.12、CasADi3.6.6、既有acados-v040真实SDK。新旧代码共用同一SDK、车体、0.2m净空、0.2m/s线速上限、1rad/s角速上限、N20/dt0.2；未调参。

|运动模型|实现|圆数/参数数|前进|倒车|旋转|
|---|---|---|---|---|---|
|differential_drive|基线|3/22|status0，VALID|status0，VALID|status0，VALID|
|differential_drive|单圆|1/14|status0，VALID|status0，VALID|status0，VALID|
|curvature_limited|基线|3/22|status0，VALID|status0，VALID|status0，VALID|
|curvature_limited|单圆|1/14|status0，VALID|status0，VALID|status0，VALID|

12组均真实生成/编译并执行AcadosOcpSolver；返回未认证MPCC候选后，再用本轮独立连续矩形/制动认证得到VALID。检查候选模式、真实行进位移或转角，避免仅以SDK返回0认定成功。新旧轨迹有浮点迭代差异，没有要求逐位相同。每组只单次场景求解，构建约2.8-3.0秒，求解墙钟约4-6.5毫秒；这些不是性能基准，不能宣称稳定提速。

新模型指纹04070066c9e4/differential_drive、b1acd2084d69/curvature_limited；基线7c1e23dce4d4、31286e0017be，目录没有复用旧布局。

## 组件回归与过程问题

最终pytest-r3：60 passed、3 skipped、9 deselected，12.52秒。四个文件：test_circle_contract、test_local_mpcc、test_solver_timing、test_clearance。包含19项装配/公式替身回归，以及真实SDK求解、正净空运动、零净空兼容、独立认证和求解计时工具回归。不是60个真实求解场景；独立12组才是明确新旧真实模型对照。

3项跳过：SDK实际已安装时的缺依赖反测1项，以及未启用native geometry的2项对照。9项未选择：依赖原生几何的对照/欧氏净空角点相关选择组及平台双配置文件检查。未假定以前6项native缺失已经补验。

首次pytest collection因SDK依赖中同名scripts包遮蔽仓库namespace失败；r2启动器显式将scripts namespace绑定到本轮源码。随后8项失败是漏传benchmark_solver_timing.py，补传该必要源码后r3全部通过；没有修改产品算法或测试断言以取得通过。原失败日志保留，不覆盖。日志尾的两个QP status3来自预期不可行障碍场景测试，不属于上表12组，整体pytest exit0。

初始79项及补齐后80项源码哈希在远程逐项核对通过。一次内联SSH/Powershell哈希命令引号解析失败，改用verify_source.py实际完成核验；不是跳过验证。

## 最新资格与处置

本轮已覆盖：台式x86_64真实CasADi/acados生成、编译、求解，新旧单圆/三圆与两种运动模型的组件对照，Python独立轨迹认证。上轮本机734+236验证仍保留其独立边界。

未执行：最新ROS Humble包重建与消息/Action联调、TCP仿真运动、真实P3地图与定位延迟闭环、Orin及实车、DRL闭环/训练。因此本轮不能宣称导航到达/长期任务/绕障通过，也不以规划PLAN_FOUND作为本次证据（本轮未跑全局规划）。

本地源码已提交e98572b，本记录本地提交待合入；长期线不移动、不push，不部署。远程目录保留为可复核来源，不自动清理或重复启动。
