# 足式应用公共成果同步

owner app/orin-legged-vehicle；原40d31d26，公共主线53ab85da，算法合成b0196964、环境修0d080910、窗口差分回归ffb92083、平台消费者实际合并75389f22。仅Git同步，未部署足式Orin或重启正式P4。

保留足式12项平台配置blob（尺寸、速度、5cm/P3配置、UNKNOWN允许均不变），保留非TCP入口、planner_params_file、platform_selector及既有足式native入口。公共共享地图/碰撞/规划/持续发布/交互已接收。

原legged接口只传slope/step/gap，无法同时带平台尺寸与clearance；新增最小内部legged窗口入口并保留旧ABI。新MSVC DLL实际native125pass，独立复审111pass；ARM新so/oracle编译成功，适用入口/新native/静态TF/窗口增量一致性151pass、1历史Git查询单例因源码archive无.git排除（该项本机已验）。gap既有core只验证非负，clearance既有足式分类未使用，本轮不冒称增加了这些越障判定语义。

最终复审发现并关闭两类启动问题：公共map节点按wheel加载足式配置由efded1fc修复；足式四入口未加载既有P3域环境由0d080910最小复用p3_environment，显式ROS域/DDS覆盖仍优先。真实Humble地图节点旧代码RED、新库+足式应用实际配置d048bf1845ce5f8700d9360485ef188227c73eabfe3e6bbd0ebb3ec137c7580e GREEN3/3。实际合并launch回归108pass，最终独立复审无P1/P2。

旧hardware selector断言按当前公共薄入口更新；旧3x3 UNKNOWN夹具改为真实0.4m新观测高差，远邻FREE→BLOCKED且所有格点增量/全量相等。无产品阈值变更。

远端证据P4/tmp/branch-sync-20261103/{legged-native,legged-linux-selected-source.log,platform-map-fix}；测试仅隔离域和P4/tmp，正式安装未改。最初漏资源/缺.git、继承旧Python包/旧so污染的失败日志均保留，最终明确选择候选Python源码及新native库后复跑通过。

完整足式ROS安装打包/足式Orin设备部署、真实P3/探索/导航到达闭环仍NOT_RUN；不push、不删旧专题和工作树。
