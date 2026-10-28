"""只读原始证据门禁；实际到达与策略完成分开。"""
import math

def grade(e):
    required=("graph","formal","motion","bounds","clearance","cleanup","installed","no_exceptions","source_received","sensor_timing")
    checks={k:bool(e["common"].get(k)) for k in required}
    reached=e.get("reached",[])
    checks["actual_goal_reached"]=bool(reached) and all(
        r.get("uuid") and all(math.isfinite(r[k]) for k in ("xy_error","yaw_error","v","w","hold_s")) and
        r["xy_error"]<=.1+1e-9 and abs(r["yaw_error"])<=.05+1e-9 and abs(r["v"])<=.01+1e-9 and
        abs(r["w"])<=.01+1e-9 and r["hold_s"]>=.3-1e-9 and r["status"]==4 and r["outcome"]==0 for r in reached)
    if e["case"]=="frontier_progress":
        checks.update(new_observations=e.get("observed_growth",0)>0,next_frontend_goal=e.get("distinct_goals",0)>=2)
    elif e["case"]=="blocked_wait_reselect":
        checks.pop("actual_goal_reached")
        checks.update(path_or_map_wait=e.get("waits",0)>0,actual_cancel_stop=e.get("canceled_stopped",0)>0,
                      reselected=e.get("distinct_goals",0)>=2)
    elif e["case"]=="task_cancel_stop":
        checks.pop("actual_goal_reached")
        checks.update(cancel_while_moving=bool(e.get("cancel_while_moving")),
                      actual_cancel_stop=e.get("canceled_stopped",0)>0,no_new_goal=bool(e.get("no_goals_after_cancel")))
    elif e["case"]=="large_source_preflight":
        checks.pop("actual_goal_reached");checks.pop("motion");checks.pop("formal")
    else:raise ValueError("unknown fixture")
    return dict(pass_=all(checks.values()),checks=checks)
