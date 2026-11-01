"""只离线回放产品CommandPrediction候选；复用既有因果评分与标签。"""
import argparse
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
from scripts.analysis.compare_delayed_pose_estimation import read_log,evaluate,metrics,wrap


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    core=Path(__file__).resolve().parents[1]/'navigation/one_cm';sys.path.insert(0,str(core))
    from nav1cm.contracts import VehicleState,WheelLimits
    path=core/'ros2/lunar_nav1cm_planner/lunar_nav1cm_planner/command_prediction.py'
    spec=importlib.util.spec_from_file_location('candidate_observed_prediction',path);mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    estimates,commands,observations=read_log(args.log);obs={o.stamp:o for o in observations}
    parameters=dict(linear_noise=.03,angular_noise=.03,position_std=.005,yaw_std=.01)
    scored=evaluate(estimates,observations,commands,parameters);queries={}
    for row in scored:queries.setdefault(row['known_until'],[]).append(row)
    p=mod.CommandPrediction(spin_prepare_s=3.);limits=WheelLimits();i=0;origin=estimates[0]['t'];durations=[];dispute=None
    for event in estimates:
        now=event['t']
        while i<len(commands) and commands[i][0]<=now:
            p.issue(commands[i][0]-origin,commands[i][1]);i+=1
        p.correct(VehicleState(*obs[event['stamp']].state),int(event['stamp']*1e9),now-origin,int(now*1e9))
        start=time.perf_counter();forecast=p.predict(now-origin,limits);durations.append((time.perf_counter()-start)*1e6)
        for row in queries.get(now,[]):
            prediction=copy.deepcopy(p).predict(row['t']-origin,limits);state=np.array([prediction.x,prediction.y,prediction.yaw,prediction.v,prediction.w]);truth=np.asarray(row['truth']);goal=row['goal']
            pd=None if goal is None else float(np.linalg.norm(state[:2]-goal));td=None if goal is None else float(np.linalg.norm(truth[:2]-goal))
            row['errors']['observed_motion']=dict(position_m=float(np.linalg.norm(state[:2]-truth[:2])),yaw_rad=abs(wrap(state[2]-truth[2])),state=state.tolist(),predicted_goal_m=pd,early_target=bool(pd is not None and pd<=.02 and td>.02),premature_position=bool(pd is not None and pd<=.02 and td>.1))
        if abs(now-1793507038.0219862)<.001:
            left=max((o for o in observations if o.stamp<=now),key=lambda o:o.stamp);right=min((o for o in observations if o.stamp>=now),key=lambda o:o.stamp)
            fraction=(now-left.stamp)/(right.stamp-left.stamp);truth=left.state+fraction*(right.state-left.state);xy=np.array([forecast.x,forecast.y]);goal=np.array([-11.277111053466797,-5.087968349456787])
            dispute=dict(t=now,old_prediction_goal_cm=float(np.linalg.norm(event['online'][:2]-goal)*100),candidate_goal_cm=float(np.linalg.norm(xy-goal)*100),later_p3_interpolated_goal_cm=float(np.linalg.norm(truth[:2]-goal)*100),candidate_position_error_cm=float(np.linalg.norm(xy-truth[:2])*100))
    split=(estimates[0]['t']+estimates[-1]['t'])/2;groups=dict(training=[r for r in scored if r['label_received']<split],held_out=[r for r in scored if r['t']>=split])
    result=dict(groups={},disputed_sample=dispute,predict_median_us=float(np.median(durations)),predict_p95_us=float(np.percentile(durations,95)),limitations=['Logged visibility approximates reception','P3 reference only','Original commands frozen, not counterfactual closed loop','Command times use publish log not exact predictor.issue monotonic time'])
    for name,rows in groups.items():
        m=metrics(rows)
        for method in m:
            m[method]['position_max_cm']=max(r['errors'][method]['position_m']*100 for r in rows)
            m[method]['yaw_max_deg']=max(r['errors'][method]['yaw_rad']*180/math.pi for r in rows)
        result['groups'][name]=m
    (args.output/'results.json').write_text(json.dumps(result,indent=2),encoding='utf-8');(args.output/'scored-samples.json').write_text(json.dumps(scored,indent=2),encoding='utf-8')
    print(json.dumps(dict(groups={k:{n:v for n,v in m.items() if n in ('online','observed_motion','two_pose_cv')} for k,m in result['groups'].items()},disputed_sample=dispute,predict_p95_us=result['predict_p95_us']),indent=2))


if __name__=='__main__':main()