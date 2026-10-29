"""只读原始证据门禁；实际到达与策略完成分开。"""
import math


def obstacle_sequence_valid(sequence):
    if not sequence or any(not sequence.get(k) for k in ("event","source","map","wait","terminal","next")):return False
    event,source,map_update,wait,terminal,nxt=(sequence[k] for k in ("event","source","map","wait","terminal","next"))
    times=[r['monotonic_s'] for r in (event,source,map_update,wait,terminal,nxt)]
    return (all(math.isfinite(t) for t in times) and times==sorted(times) and
      event['accepted_at']<=times[0] and event['active'] and event['distance_m']>=1.6 and
      source['changed'] and map_update['changed'] and wait['uuid']==event['uuid']==terminal['uuid'] and
      wait['reason'] in ('WAITING_FOR_PATH','WAITING_FOR_MAP') and terminal['status']==5 and terminal['outcome']==5 and
      terminal['reason']=='CANCELED_STOPPED' and terminal['stopped_before'] and nxt['logical_accepted'] and
      nxt['uuid']!=event['uuid'] and nxt['generation']>event['generation'] and event['binding_source']=='TASK_ACCEPTED' and nxt['binding_source']=='TASK_ACCEPTED' and math.hypot(nxt['pose'][0]-event['pose'][0],nxt['pose'][1]-event['pose'][1])>.1)

def grade(e):
    required=("graph","formal","motion","bounds","clearance","cleanup","installed","no_exceptions","source_received","sensor_timing","authoritative_identity")
    checks={k:bool(e["common"].get(k)) for k in required}
    reached=e.get("reached",[])
    checks["actual_goal_reached"]=bool(reached) and all(
        r.get("uuid") and all(math.isfinite(r[k]) for k in ("xy_error","yaw_error","v","w","hold_s")) and
        r.get('sample_age_s',math.inf)<=.15 and r["xy_error"]<=.1+1e-9 and abs(r["yaw_error"])<=.05+1e-9 and abs(r["v"])<=.01+1e-9 and
        abs(r["w"])<=.01+1e-9 and r["hold_s"]>=.3-1e-9 and r["status"]==4 and r["outcome"]==0 for r in reached)
    if e["case"]=="frontier_progress":
        checks.update(new_observations=e.get("observed_growth",0)>0,next_frontend_goal=e.get("distinct_goals",0)>=2)
    elif e["case"]=="blocked_wait_reselect":
        checks.pop("actual_goal_reached")
        checks.update(path_or_map_wait=e.get("waits",0)>0,actual_cancel_stop=e.get("canceled_stopped",0)>0,
                      reselected=e.get("distinct_goals",0)>=2)
    elif e["case"]=="blocked_obstacle_update":
        checks.pop("actual_goal_reached")
        checks["actual_obstacle_update_sequence"]=obstacle_sequence_valid(e.get("obstacle_sequence"))
    elif e["case"]=="task_cancel_stop":
        checks.pop("actual_goal_reached")
        checks.update(cancel_while_moving=bool(e.get("cancel_while_moving")),
                      actual_cancel_stop=e.get("canceled_stopped",0)>0,no_new_goal=bool(e.get("no_goals_after_cancel")))
    elif e["case"]=="large_source_preflight":
        checks.pop("actual_goal_reached");checks.pop("motion");checks.pop("formal")
    else:raise ValueError("unknown fixture")
    if e.get('tf_mode')=='static_only_latched':checks['static_only_latched']=bool(e.get('static_tf_contract'))
    return dict(pass_=all(checks.values()),checks=checks)
