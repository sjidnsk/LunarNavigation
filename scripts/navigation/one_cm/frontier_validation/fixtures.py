"""首次实跑前冻结的合成测量/物理合同；失败后不延长场景。"""
CASES={"large_source_preflight":15.,"frontier_progress":75.,"blocked_wait_reselect":60.,"task_cancel_stop":30.}
DECLARATION={
 "domain_m":[-5.,-5.,5.,5.],"source_extent_m":10.,"source_resolution_m":.1,
 "preflight_source_extent_m":5.,"preflight_source_resolution_m":.01,
 "coarse_resolution_m":.2,"local_computation_m":[5.,5.,.01],"clearance_m":.2,
 "body_m":[1.182,.818],"nominal_response_s":.15,"limits":[.2,1.,.5,.5,1.],
 "initial_measured_rectangle_m":[-4.,-4.,3.8,4.],
 "initial_support":"显式历史合成扫描，仅该矩形内有限elevation=0，外部NaN；无盲区补全",
 "progress_scan":"仅首次实际Action GOAL_REACHED后，在实测位姿半径3.2m内新增360度无噪声平地测量；历史测量保留",
 "blocked":"首次前沿goal后、t>=5s且goal距实际车体中心>=1.6m，在goal周围0.6m方块增加高度2m实际障碍；另初始未知狭缝x=[.9,1.3],y=[-1.3,1.3]保留可绕行已知路线；不强制选目标",
 "cancel":"t>=10s实际v或w>.03时发任务CANCEL；未触发则t20s取消但门禁失败；所有case在结束前8s发清理CANCEL并测停",
 "plant":"仅真实Twist驱动；线/角加速度slew<=.5，<=.01s中点积分；不读Path控制、不导入生产nav1cm；独立OBB/AABB净空",
 "sensor":"合成无噪声测量，不声称真实P3传感器或视线遮挡；新增障碍测量为脚本事件，未在控制器/策略注入真值碰撞oracle",
 "timing":"odom约50Hz，map2Hz，零stamp TF约10Hz；各case固定时长；清理最多8s，不延长运动验收",
 "acceptance":"共同身份/私有graph/正式规划/实际运动/运动与.2净空/无异常/有界清理；progress真实xyyawstop持续.3s后新观测与下一前沿goal；blocked真实wait->cancel-stop->distinctgoal；external运动中cancel-stop且无新goal",
 "not_run":["100m行驶","100m全源1cm地图","真实P3","生产DDS","Orin","实车","DRL推理","训练"]}
