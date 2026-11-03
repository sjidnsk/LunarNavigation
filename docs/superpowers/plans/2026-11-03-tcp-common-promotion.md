# TCP成熟成果逐批晋级与应用同步

授权：用户2026-11-03要求按公共算法、公共交互、TCP专用逐批整理合并，每批独立复审，再同步轮式和足式；暂不重启正式P4。

基线：integration/pure-planner-orin=b1575f9b；来源hk/tcp-terrain-window-deploy-20261102=02b7ec45（正式payload7fcc7362）。
归属：公共批owner主线；TCP批owner app/orin-unreal-simulation。当前执行ROOT/Windows，独立WT tcp-common-promotion-20261103。

约束：不push、不删分支/工作树、不改远端正式环境、不重启/发目标；保留应用自身能力参数与接口，不改开发线。原始dirty工作树不动。

1. 公共算法：按清单合成来源最终delta，包含ROS事实生产/消费完整依赖、持续发布与单调P3源戳（用户已撤销同戳修正）；不带TCP轮式30cm起伏/限速覆盖。公共topics只加入base_map_facts_topic，显示名称下一批。
2. 公共交互：RViz清区域、最新odometry/相机布局与显示话题，保留模块接口；独立复审与定向回归。
3. TCP应用：先与来源分支及主线做三方合并，保留TCP参数、P3预测启用、长前瞻与构建选择工具；不以部署替代Git整合。
4. 轮式与足式：分别从应用HEAD建专题合并公共主线，核对平台配置未被TCP覆盖；代码测试与实车NOT_RUN分开。
5. 每批记录来源、测试、独立复审和合入SHA；最后总体复审并验证四长期线包含公共成果，登记资源保留待回收。

清单：见同名JSON。采用最终树delta避免重放已经被替代的同戳和配置中间版本；来源SHA+逐文件清单提供追溯。
