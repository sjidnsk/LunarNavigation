# P4可回退统一正式构建

任务TCP-FORMAL-UNIFIED-20261103；owner app/orin-unreal-simulation；ROOT/Windows，复用hk/tcp-terrain-window-deploy-20261102及原工作树，基线0939f1e5。用户授权将未部署碰撞、地图、RViz和已落地持续发布统一为可回退正式构建；P3/Env_X只读；不删除旧构建/源码、不push/移动长期线。

1. 从当前提交导出必要源码、配置和测试，创建/P4/build/formal-20261103-unified；不依赖调试安装。构建活动依赖闭包10包，只用Humble、P4基础依赖及Env_X接口。
2. 启动读取P4_SOURCE_DIR；唯一formal-active指针选择其环境、源码、配置和安装。旧环境/源码封存为baseline回退目标，formal-previous保留上次选择。稳定根启动/环境入口只作引导。
3. 切换工具复用P4启动锁；只在无存活P4 operator时选择已验证构建。校验路径和环境摘要，原子换链，支持--previous；不停止或创建导航任务。部署约束不进入导航链。
4. RED/GREEN验证启动接线和切换/回退。目标新构建顺序跑core/地图/nav1cm/RViz/增量探索测试、真实acados规划/认证与DDS及实图性能；旧legacy失败独立记录。
5. 独立整体复审；生成源码/安装/依赖/配置清单和资格后整体切换。演练回退并切回新正式，检查活动包均同源；不代发现场目标。保存提交和待owner合入/现场NOT_RUN边界。

Ruling：复用已批准算法，不启用训练或更换控制后端；现有0.1/0.1限速、75cm半径、50cm软净空、20度坡度、30cm局部高差、1.6m前瞻及15秒/2秒全局预算保持。最初打包误写不存在的ros2_ws/tests，未导入任何包；按实际tests/scenarios夹具目录重新打包，空构建仅有日志，无正式切换。

Ruling：active/previous使用单一config/p4_active_build.json原子更新，而非两条非原子符号链接；环境从active路径读取environment.sh。稳定启动先持既有启动锁再读取选择，切换工具同锁排他，避免选择期间启动混版。
