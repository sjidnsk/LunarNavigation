# TCP默认统一域57与P4继承终端

任务TCP-DOMAIN-INHERIT-20261103，owner app/orin-unreal-simulation。用户明确授权P3仅两个入口最小修改，其他P3源码/配置及Env_X/.bashrc不得修改。实现fc76c756，集成029ead08；实现专题hk/tcp-domain-inherit-20261103已提交、工作树clean待回收，不移动长期线或push。

## 实际改动

P3严格只有：
- /home/yanfa/P3/lidar_visual_fusion/scripts/live_sensor_relay.py：采集源域由终端ROS_DOMAIN_ID读取，缺省57；建图目标域仍57。同域时不创建与输入同名的雷达/IMU publisher；图像仍做原正常化、输出/fusion/left/right。录制和inbox处理保留，forwarded含处理/录制计数，不应再解读为全部DDS实际发送数。原时戳与关闭流程不变。
- /home/yanfa/P3/lidar_visual_fusion/scripts/sim_imu/start_live_sim_imu.sh：移除强制10，继承已有值/缺省57；轮速目标57、录制及可选IMU原功能保持。

P4：p4_tcp_runtime.py统一用control_domain读取终端域，空/缺省57，显式0/10保留；同域input_bridge在创建任何端点前返回，跨域保留原57→目标域四话题单向桥。main与导航/RViz子进程环境一致。正式config/p4_environment.sh由强制10改为ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-57}"。启动指令.md移除三个固定10设置，说明新终端默认57、整组退出再启动。

.bashrc原本57未修改；车辆与两种采集程序源码未修改。P3 start_live.sh仍固定57，本次不宣称整个P3任意域可配。同域raw雷达/IMU本来直接可被建图adapter订阅，因此relay不再控制其DDS送达；其原入队录制/计数仍在。当前profile左右raw话题与/fusion不同；不宣称任意自定义profile均保持预处理。

## 验证与生效边界

项目Python D:/conda_envs/lunar-explorer/python.exe相关90passed/5skip；skip为Windows缺bash等平台边界。P3候选AST/同域仅左右publisher、跨域四类publisher检查通过；helper未设置/57/10/0四个Shell继承表达式通过，两个shell脚本bash -n通过，源代码AST通过。独立只读复审P3两文件和P4提交均无阻塞。

Humble只读check-only 3秒：control_domain57/p3_source_domain57；map3、odom3、tf3、tf_static6，forwarded_nonzero0、children为空，退出0。未启动导航、RViz或车辆；该验证不覆盖NavObserver实际显示或完整统一域闭环。P3 source_manifest.py只读检查通过，无需改第三个P3构建文件。

文件已备份并落地，但没有重启现有任务。旧车辆/采集/P4进程仍可能在10，旧P3 relay仍执行已加载代码；必须按更新后的启动指令完整退出后，在新终端依次启动车辆、采集、P3、P4才切换到默认57。P3重启新地图，P4重新选目标。实际统一域全链路/车辆验收NOT_RUN。

备份及回执/home/yanfa/P4/build/rollback-domain-inherit-20261103/receipt.json；候选与最小patch在/P4/tmp/domain-inherit-20261103。本机副本D:/CodexDownloads/lunar_navigation/domain-inherit-evidence-20261103，含两源文件、patch、环境、启动指令与receipt。

P3 relay SHA256 de58fd43c6add485c9f4460c92e80eff53910881ecd034686e536e70f151ba14；helper 493bb4a6210c53e84d51c9b1b2aab7e27544318b86cf1c1fd1d3fff6d8544ddb；P4 runtime 7e66f8759fea27f4b51be3522728a591ad0acce2468ee4a267447a7691643987；P4环境6e9229487619da144e907b63067a3192e69b9939280d5b85aca7cb6ffbd47eb9。

处置：已提交并部署文件，待用户整组重启启用；未回收专题与备份，未push/长期线合并，未修改无关目录或P3其他代码。
