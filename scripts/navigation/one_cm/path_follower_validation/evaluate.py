"""由原始轨迹、真实 DDS 和来源记录派生指标，不重写原始行。"""
import json
import math
from pathlib import Path
from .evidence import legacy,TOPICS,CMD,goal,obstacles
from .grade import grade
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
def analyze(folder,case,processes,error,stop,profile=None):
    plant=rows(folder,'plant');follower=rows(folder,'follower');planner=rows(folder,'planner')
    actors=dict(plant=plant,follower=follower,planner=planner)
    graphs=[r for actor in actors.values() for r in actor if r['kind']=='GRAPH']
    runtime_errors=[dict(role=role,notice=r['notice'],detail=r.get('detail'))
        for role,actor in actors.items() for r in actor
        if r['kind']=='NOTICE' and r.get('notice')=='TICK_ERROR']
    production_command_graphs=sum(bool(r.get('publishers',{}).get('/Car/T5/Car_Cmd_Vel')) for r in graphs)
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
    # 任一次捕获异常或任一已观测图出现实车命令发布者，均独立否决。
    result['checks']['runtime_exceptions_absent']=not runtime_errors
    result['checks']['production_command_absent']=bool(graphs) and production_command_graphs==0
    entities=[entry for actor in (plant,follower,planner) for r in actor if r['kind']=='ENTITY_QOS' for entry in r['entities']]
    result['checks']['dds_qos']=any(qos_valid(r,entities,case) for r in plant if r['kind']=='GRAPH')
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
    if profile:
        from .profiles import within_bounds,held as profile_held
        result['checks']['profile_bounds']=bool(states) and all(within_bounds(r,profile['expected']) for r in states)
        result['checks']['profile_terminal']=profile_held(states,goal(case),profile['expected'])
        effective=f_install[0]['parameters'] if f_install else {}
        resources=[r for r in planner if r['kind']=='RESOURCES']
        actual_planner=({**resources[0]['settings'],**resources[0]['limits']} if resources else {})
        result['checks']['profile_parameters']=all(effective.get(k)==v for k,v in profile['follower'].items()) and (
            case=='empty_smoke' or all(actual_planner.get(k)==v for k,v in profile['planner'].items()))
        if 'minimum_clearance_m' in profile['expected']:
            from .clearance import measure
            required=profile['expected']['minimum_clearance_m']
            clearance=measure(states,lambda t:obstacles(case,t),required)
            result['clearance_evidence']=clearance
            result['checks']['profile_clearance']=clearance['pass']
            result['checks']['profile_clearance_parameters']=(effective.get('minimum_clearance_m')==required and
                actual_planner.get('minimum_clearance_m')==required and
                actual_planner.get('clearance_radius_m',0.)>=math.hypot(1.182,.818)/2+required)
        result['declared_profile']=profile
    result['pass_']=all(result['checks'].values())
    result.update(runtime_exceptions=runtime_errors,graph_snapshot_count=len(graphs),
        production_command_graph_count=production_command_graphs,
        case=case,processes=processes,formal_count=len(formal),local_paths=len(local),nonempty_paths=len(full),
        state_rows=len(states),final_state=states[-1] if states else None,
        maxima={k:max((abs(r[k]) for r in states),default=None) for k in ('v','w','a','alpha')},
        displacement=max((math.hypot(r['x'],r['y']) for r in states),default=0.))
    return prior.apply_supervision_gate(result,folder,error)

def qos_valid(graph,entities,case):
    # 历史与depth在DDS发现端点可能未知；深度必须由同一真实运行实体补证。
    pairs=[(TOPICS['local'],'publishers','nav1cm_joint_plant' if case=='empty_smoke' else 'lunar_nav1cm_planner','VOLATILE',1),
           (TOPICS['local'],'subscriptions','lunar_pure_wheeled_controller','VOLATILE',1),
           ('/tf_static','publishers','nav1cm_joint_plant','TRANSIENT_LOCAL',None),
           ('/tf_static','subscriptions','lunar_pure_wheeled_controller','TRANSIENT_LOCAL',None)]
    if case!='empty_smoke':
        pairs.extend([(TOPICS['global_route'],'publishers','lunar_nav1cm_planner','VOLATILE',1),
                      ('/tf_static','subscriptions','lunar_nav1cm_planner','TRANSIENT_LOCAL',None)])
    for topic,direction,node,durability,depth in pairs:
        own=[r for r in entities if r['topic']==topic and r['direction']==direction and r['node_name']==node]
        discovered=[r for r in graph.get(direction,{}).get(topic,[]) if r['node_name']==node]
        if not own or not discovered:return False
        for row in own:
            q=row.get('qos',{})
            if q.get('reliability')!='RELIABLE' or q.get('durability')!=durability:return False
            if depth is not None and (q.get('depth')!=depth or q.get('history')!='KEEP_LAST'):return False
        for row in discovered:
            q=row.get('qos',{})
            if q.get('reliability')!='RELIABLE' or q.get('durability')!=durability:return False
            if depth is not None:
                if q.get('history') not in ('KEEP_LAST','UNKNOWN'):return False
                if q.get('depth') is not None and q['depth']!=depth:return False
    return True
