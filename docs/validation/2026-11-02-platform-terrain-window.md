# 平台尺度地形窗口验证与TCP候选部署

## 范围与算法

owner `integration/pure-planner-orin`，专题 `hk/platform-terrain-window-20261102`，基线 `b1575f9b`。坡度窗口半径 `ceil(L/(2r))`，L取平台平面长边；台阶/局部高差初期尺度L/3。轮式L=1.182m，足式L=0.68m；足式实际目标5cm源地图。复用各平台原生物理阈值，不改变碰撞包络。

坡度用完整L窗口内相邻高程梯度的横纵滑动中位数，局部高差扣除同一稳健背景坡度。忽略未知样本，不填补未知中心；存在足够非共线数值支持时可计算地形。没有覆盖率门槛、置信度障碍、停车或软代价。缺少数值支持仍标UNKNOWN，由现有allow_unknown策略决定通行。不是把工程覆盖率解释为统计概率。

唯一工具在core，新导航原生桥、探索分类和FineCellEvaluator/PolicyMapExporter复用。旧3x3测量和三参数ABI保留显式兼容；DRL旧训练入口显式保留旧工厂，未训练。默认Derive按raw新窗口，旧输送标量证据用DeriveProjected显式接入。增量影响半径包含平台方窗外接圆、碰撞及软净空支持。

## 当前验证

- Windows相关Python测试：1250通过、108跳过（缓存优化前）；不是Humble/实车验收。
- ARM64/Humble隔离：新窗口20项通过；探索9项、PolicyMapExporter2项通过；原生桥/P3/缓存80项通过。
- 输入halo读取回归：32x32输出、轮式5cm，读取从94288次/单格最多86次降至3136次/单格一次。
- 核心增量回归：更新距中心0.6m的一列，旧halo与全量不一致已RED复现；扩大至真实窗口依赖后，25项非异常核心测试通过（包含逐格增量/全量状态及成本对比）。3个历史标量传输断言已收敛到显式Projected旧工厂，并未删除原合同测试。
- 2项异常输入核心测试在本次隔离构建中中止；替回旧factory的对照也中止，禁用尾调用优化仍中止。此边界未解决，不能声称整包全绿。保留exception-baseline.log和final-core.log，不用“源码已通过”替代这些结果。

## 缓存候选性能

ARM64 TCP机，合成高程原生分类（非路径搜索/非实车闭环），两个帧样本：

| 场景 | 原新窗口 | 缓存候选 |
| --- | ---: | ---: |
| wheel 16m / 5cm | 1056–1080ms | 712–725ms |
| wheel 64m / 5cm | 18599–18913ms | 11920–12186ms |

逐格分类计数一致。64m5cm依然过慢，不能宣称性能优化完成。第二帧仅高程中心+1mm，但现有源输入仍承担整幅测量成本。当前TCP实际为20cm源地图；后续性能子任务独立比较该负载。已保留优化前后JSON，不覆盖证据。

## 用户授权的正式候选

2026-11-02用户明确要求先替换正式P4观察，再并行优化。部署18个明确源码/库/头文件/README文件，原子替换并保存SHA/备份；只写/home/yanfa/P4，不修改P3或Env_X。正式运行不引用隔离tmp库；core/ROS/nav桥都从原正式安装路径加载。

部署回执 `/home/yanfa/P4/build/rollback-platform-window-20261102/receipt.json`。启动运行目录 `/home/yanfa/P4/run/p4-20261102-161603-45590`，交付时operator45590、navigation45809、map45807、RViz45719。保留20度坡度、前后0.3m/s、角0.15rad/s、软净空0.5m、允许UNKNOWN。原生平台尺寸构造smoke通过。

当时只有1帧保留地图及静态TF，无P3 odometry，处于ODOMETRY_WAITING。未发目标、未重启P3或车辆接口；车辆闭环NOT_RUN。旧operator执行零速及退出流程，但缺少定位反馈，measured_stopped=false，未声称实测停稳。

## 处置

当前已部署用户观察候选；性能R3子任务在独立目录进行，尚未验证/未部署。两项异常输入回归待定位。未合入长期线、未push、未删工作树/分支。测试/部署证据仅代表对应环境，足式Orin和轮式实车NOT_RUN。

## R3性能子任务（已验证，未再次部署）

只改新工具cpp/statistics/test三文件：内部halo相对坐标直读，滑动中位数复用移出节点和再平衡节点。保持窗口、阈值、UNKNOWN、判定公式及接口。限定独立只读复审未发现阻塞；复杂度说明仍包含局部高差每格O(local²)，O(N log R)只描述梯度滑窗。

独立Release构建，核心21项及native80项通过（70.38秒）；新增15000步中位数朴素排序对照。两尺寸×3分辨率×未知/负索引/INT64边缘合计151956格，每格2bool+3double全部位级一致，3950856字节SHA256 `3d04c5855514cf236251b901cd0c51298f915686f793936976e7058e43b5fd33`。

| 场景 | 当前正式缓存候选 | R3 |
| --- | ---: | ---: |
| wheel 64m / 20cm（当前TCP分辨率） | 297–310ms | 213–228ms |
| wheel 16m / 5cm | 712–725ms | 362–381ms |
| wheel 64m / 5cm | 11920–12186ms | 6292–6503ms |
| legged 16m / 5cm | 旧原始724–746ms | 281–297ms |

样本量每项2帧，不代表统计置信区间。64m5cm仍未解决实时大图负载。证据本机 `D:/CodexDownloads/lunar_navigation/platform-terrain-perf-r3/evidence/summary.json`，远端 `/home/yanfa/P4/tmp/platform-terrain-perf-r3/`。R3仅本地专题保存/隔离验证，未再次覆盖用户当前正式观察版本；正式候选身份以部署receipt的文件SHA为准。

## R3与tcp (3)最终正式切换

用户追加明确要求加速版正式替换、最大角速度0.1rad/s，并一起接收tcp (3)修复752ffaa7。公共专题cherry-pick为57e89419，TCP部署专题接收公共平台窗口6673df5f和位姿修复；仅任务登记追加冲突保留双方记录。部署前受影响runtime/command_prediction/execution_feedback组合80项通过。

旧operator45590正常停车退出，result.measured_stopped=true。R3三个工具/测试源和正式core库原子替换，另替换source及安装的runtime.py，不覆盖其他包或P4配置。正式core SHA256 `de4b7419c5edb678c38ac17255bf7af21ddfdf1403a07a364363909804a89976`；runtime SHA256 `565184779ed96d83ed6f389e286961c18a9c72b5bcdac83657e9497b5760b867`。新库无tmp依赖；正式环境NativeTerrain smoke通过。

2026-11-02最终运行 `/home/yanfa/P4/run/p4-20261102-163819-50250`，operator50250/navigation50323/map50321/RViz50304。实读前后0.3m/s、最大角0.1rad/s、坡度20度、soft0.5m、allow_unknown=true。地图49帧/定位117帧且推进，已观察用户新目标和非零指令，不代表到达验收；未代发目标。

角速度仅修改P4的source/config/tcp_wheel_speed.yaml、config/vehicle.yaml及README。原生接口配置spin_max_linear_speed按现有wheel_base0.815/track_width0.622换算为0.051261803518799456m/s；Env_X现行进程未重启，因此该文件供其下次带参数启动，未宣称原生运行参数已重载。当前P4生成/转发命令自身按0.1限幅。未修改或重启P3/Env_X。

回滚及部署回执 `/home/yanfa/P4/build/rollback-platform-window-perf-r3-20261102/receipt.json`，角配置备份另见同目录angular-config-receipt.json。源码已本地专题保存，未自动移动长期线或push；2项异常输入隔离回归未解决，到达/实车验收NOT_RUN。
