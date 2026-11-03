# TCP成果逐批整合结案

2026-11-03用户授权公共算法→公共交互→TCP专用逐批独立复审并合并，再同步轮式/足式应用线；不重启正式P4。每批和最终跨线复审均完成，最终无未关闭P1/P2。

## 合入锚点

下列是结案文档前实际已落地的代码/验收锚点，最终HEAD可再包含本结案文档；JSON记录完整SHA与配置逐项核对。所有三应用均包含公共53ab85da，历史分叉和平台专用源码保留。

|长期线|已落地锚点|本轮内容|
|---|---|---|
|integration/pure-planner-orin|53ab85da|公共地图/碰撞/全局局部规划/持续发布，RViz与探索清理，平台事实消费者修复|
|app/orin-unreal-simulation|42154a0c|接已部署来源02b7ec45与公共主线，保留TCP .1/.1、30cm起伏、预测/长前瞻与构建选择|
|app/orin-real-vehicle|fa1a70cf|接公共成果，9项平台配置blob保持，保留非TCP入口和ROS参数文件|
|app/orin-legged-vehicle|046e7cb1|接公共成果，12项平台配置blob保持，保留5cm与足式能力；补内部窗口参数/P3环境接线|

## 验证与复审

- 公共算法实施1599pass/156skip/1deselect，独立复审另182pass；真实native地图补验103pass。
- 公共C++ 148文件与正式同源，Humble core21组、ROS16组适用项；关闭legacy debug单例明确排除，不宣称全包全绿。DDS图查询一例首次失败原参数重跑7/7通过。
- 公共交互静态143pass/4skip/1deselect；同源42文件Humble RViz3组/活动探索3组全部通过，独立复审无阻塞。
- TCP Windows187pass/9skip；ARM Linux入口112pass。修复仅在假runtime JSON完整写入前就绪的测试竞争，不改产品超时。
- 轮式实施/独审各73pass；补ARM Linux入口、静态TF、原生增量20pass；合公共平台修后launch98pass。
- 足式新MSVC DLL125pass/独审111pass；ARM新so与oracle构建成功，适用入口/native/TF/窗口151pass、1依赖Git历史的单例因archive无.git明确排除，本机已验；合并launch108pass。
- 最终复审抓到并关闭真实P1：地图节点原先按wheel加载legged配置，尚未消费事实就抛错。复用既有PlatformProfileFor及selector，真实ROS旧代码RED，新库并应用实际足式配置GREEN3/3。还恢复足式四入口对已有P3域环境的复用；未新增门禁/导航实体/阈值。
- 旧3x3 UNKNOWN测试夹具不再符合平台尺寸窗口；改用真实新高差，要求远邻FREE→BLOCKED、确有变化且所有格点增量/全量相等，未改变产品分类迎合旧断言。

最初隔离包漏夹具/资源、旧Python模块或旧so被先加载的失败均保留；最终显式选择候选Python源码和新so并重跑。日志与独立审查报告在D:/CodexDownloads/lunar_navigation/tcp-common-promotion-validation；远端必要源码/测试仅P4/tmp/branch-sync-20261103。原正式active/previous构建保持，未改P3或Env_X文件。

## 边界和资源

本次仅本地Git整合、同源Humble/ARM隔离验证。轮式和足式设备安装/完整应用重建及真实P3导航、探索到达闭环仍NOT_RUN；没有用隔离测试宣称实车通过。不push、不设upstream、不创建PR、不改开发线、不删分支或工作树。正式P4源码/安装未替换、未重启、不发送现场目标。

任务专题hk/tcp-common-promotion-20261103、hk/tcp-app-sync-20261103、hk/wheel-common-sync-20261103、hk/legged-common-sync-20261103均已由对应owner接收；临时工作树与验证产物保留待授权回收，原protected dirty目录保持。
