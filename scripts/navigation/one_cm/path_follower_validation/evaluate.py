"""由原始轨迹、真实 DDS 和来源记录派生指标，不重写原始行。"""
import json
import math
from pathlib import Path
from evidence import legacy,TOPICS,CMD,goal
from grade import grade
prior=legacy('evaluate')
def rows(folder,role):
    return prior.load_rows(folder,role)
def graph_valid(g,case):
    expected={TOPICS['grid']:('grid_map_msgs/msg/GridMap','nav1cm_joint_plant'),
              TOPICS['odom']:('nav_msgs/msg/Odometry','nav1cm_joint_plant'),
              TOPICS['local']:('nav_msgs/msg/Path','nav1cm_joint_plant' if case=='empty_smoke' else 'lunar_nav1cm_planner'),
              CMD:('geometry_msgs/msg/Twist','lunar_pure_wheeled_controller'),
              '/tf_static':('tf2_msgs/msg/TFMessage','nav1cm_joint_plant')}
    if case!='empty_smoke':
        expected[TOPICS['global_route']]=('nav_msgs/msg/Path','lunar_nav1cm_planner')
        expected[TOPICS['goal']]=('geometry_msgs/msg/PoseStamped','nav1cm_joint_plant')
    if g.get('publishers',{}).get('/Car/T5/Car_Cmd_Vel'):return False
    for topic,(kind,name) in expected.items():
        if g.get('types',{}).get(topic)!=[kind]:return False
        if not any(p['node_name']==name and p['topic_type']==kind for p in g.get('publishers',{}).get(topic,[])):return False
    return any(p['node_name']=='lunar_pure_wheeled_controller' for p in g.get('subscriptions',{}).get(TOPICS['local'],[])) and any(
        p['node_name']=='nav1cm_joint_plant' for p in g.get('subscriptions',{}).get(CMD,[]))
def held(s,target,lo=-math.inf,hi=math.inf):
    return prior.held_terminal([dict(r,elapsed_s=r['t']) for r in s],target,lo,hi)
def analyze(folder,case,processes,error,stop):
    plant=rows(folder,'plant');follower=rows(folder,'follower');planner=rows(folder,'planner')
    states=[r for r in plant if r['kind']=='STATE']
    local=[r for r in plant if r['kind']=='PATH' and r['stream']=='local']
    full=[r for r in local if r['message']['poses']]
    formal=[r for r in planner if r['kind']=='TICK' and prior.formal_acceptance(r)]
    close=[r for r in planner if r['kind']=='CLOSE']
    f_close=[r for r in follower if r['kind']=='CLOSE']
    f_install=[r for r in follower if r['kind']=='INSTALL']
    e=dict(states=states,graph_ok=any(graph_valid(r,case) for r in plant if r['kind']=='GRAPH'),
        formal=bool(formal) or case=='empty_smoke',
        installed=bool(f_install) and f_install[0]['executor']=='StandardPathFollower' and '/install/' in f_install[0]['module'] and
            (case=='empty_smoke' or any(r['kind']=='INSTALL' for r in planner)),
        workers_clean=bool(f_close) and f_close[-1]['map_worker_alive'] is False and
            (case=='empty_smoke' or bool(close) and not close[-1]['remaining_children'] and not close[-1]['result']['threads_alive'] and
             all(not w['surviving_pids'] and not w['terminated_pids'] for w in close[-1]['result']['workers'].values() if w)),
        paths=len(full),terminal=held(states,goal(case)),processes=processes,supervisor_error=error)
    result=grade(e)
    result['checks']['path_contract']=bool(full) and all(prior.path_valid(r['message'],local=True) for r in local)
    result['checks']['independent_plant']=any(r['kind']=='EXIT' and not r['forbidden'] for r in plant)
    result['checks']['zero_stamp_tf']=any(r['kind']=='TF' and r['stamp']==0 for r in plant)
    if case=='reverse':
        result['checks']['negative_velocity']=bool(states) and min(r['v'] for r in states)<-.05
    if case=='forward_spin':
        delay=rows(folder,'delay')
        result['checks']['delay_motion']=False
        if len(delay)==2:
            a,b=[r['monotonic_s'] for r in delay]
            during=[r for r in states if a<=r['monotonic_s']<=b]
            result['checks']['delay_motion']=(b-a>=.25 and len(during)>1 and
                math.hypot(during[-1]['x']-during[0]['x'],during[-1]['y']-during[0]['y'])>.001 and
                sum(a<=r['monotonic_s']<=b for r in full)>=2)
        if stop:
            before=min(states,key=lambda r:abs(r['monotonic_s']-stop))
            after=[r for r in states if r['monotonic_s']>=stop]
            result['checks']['planner_stop_moving']=abs(before['v'])>.03
            result['checks']['stale_stop']=bool(after) and abs(after[-1]['v'])<=.01 and abs(after[-1]['w'])<=.01
    if case=='obstacle':
        result['checks']['obstacle_traversed']=bool(states) and max(r['x'] for r in states)>1.5 and max(r['y'] for r in states)>.95
    if case=='mapchange':
        result['checks']['one_goal']=sum(r['kind']=='GOAL' for r in plant)==1
        applied=[json.loads(r['detail'])['source_resolution_m'] for r in planner if r['kind']=='NOTICE' and r['notice']=='MAP_APPLIED']
        result['checks']['hot_resolution']=any(abs(r-.03)<1e-9 for r in applied) and any(abs(r-.05)<1e-9 for r in applied)
    if case=='empty_smoke':
        segment=[r for r in states if 6.6<r['t']<7.8]
        result['checks']['empty_stopped']=bool(segment) and all(abs(r['v'])<=.01 and abs(r['w'])<=.01 for r in segment)
    result['pass_']=all(result['checks'].values())
    result.update(case=case,processes=processes,formal_count=len(formal),local_paths=len(local),nonempty_paths=len(full),
        state_rows=len(states),final_state=states[-1] if states else None,
        maxima={k:max((abs(r[k]) for r in states),default=None) for k in ('v','w','a','alpha')},
        displacement=max((math.hypot(r['x'],r['y']) for r in states),default=0.))
    return prior.apply_supervision_gate(result,folder,error)
