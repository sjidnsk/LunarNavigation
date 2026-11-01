"""首次实跑前冻结的合成测量/物理合同；失败后不延长场景。"""
CASES={"large_source_preflight":15.,"frontier_progress":75.,"blocked_wait_reselect":60.,"task_cancel_stop":30.,"blocked_obstacle_update":60.}
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


def measurement_mask(x,y,case,version="v1"):
    """v2仅平移独立合成未知要素；原v1原件保留。"""
    if version not in ("v1","v2"):raise ValueError("invalid fixture version")
    mask=(x>=-4)&(x<=3.8)&(y>=-4)&(y<=4)
    if case=="blocked_wait_reselect":
        lo,hi=(.9,1.3) if version=="v1" else (1.6,2.0)
        mask &= ~((x>=lo)&(x<=hi)&(y>=-1.3)&(y<=1.3))
    return mask

def declaration(version):
    out=dict(DECLARATION,fixture_version=version)
    if version=="v2":
        out["blocked"]=out["blocked"].replace("x=[.9,1.3]","x=[1.6,2.0]")
        out["fixture_ruling"]="ROOT task3-fixture-support-ruling.md；仅未知缝平移；旧失败/sourceSHA保留"
    return out

OBSTACLE_UPDATE_DECLARATION={
 "case":"blocked_obstacle_update","duration_s":60.,"geometry":"qualified progress initial source without extra unknown slit; SHA70d26980e9c32fd604f8578f5bac3895f79860c49a08be2c9625e8b1f707889f; identical body/.2 clearance/.15 response/limits/50Hz",
 "trigger":"first logically GOAL_ACCEPTED CURRENT UUID with active status and measured body-goal>=1.6m; immediate safe event, no t>=5 restriction; otherwise wait next accepted qualified target",
 "obstacle":"same .6m square centered accepted goal, elevation2m; actual source and independent physical truth both change",
 "acceptance":"event->changed source->changed actual map->matching UUID WAIT->CANCELED_STOPPED measured before result->new logically accepted UUID with XY>.1m; all existing motion/formal/physics/identity/graph/cleanup gates retained",
 "no_event":"FAIL; single fixed run only; do not manipulate frontend or tune scene to pass"}
