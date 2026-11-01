"""仅离线：P3源时刻标签与当时可见的历史预测比较，不接入ROS或控制。"""
from __future__ import annotations
import argparse
from bisect import bisect_right
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import time
import numpy as np


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def transition(state, dt):
    """恒速unicycle精确积分及Jacobian，速度定义在受控车体点。"""
    x = np.array(state, dtype=float, copy=True)
    theta, v, w = x[2:]
    c, s = math.cos(theta), math.sin(theta)
    f = np.eye(5)
    if abs(w) < 1e-6:
        a = c*dt-.5*s*w*dt**2-c*w*w*dt**3/6
        b = s*dt+.5*c*w*dt**2-s*w*w*dt**3/6
        da = -s*dt-.5*c*w*dt**2+s*w*w*dt**3/6
        db = c*dt-.5*s*w*dt**2-c*w*w*dt**3/6
        aw, bw = -.5*s*dt**2-c*w*dt**3/3, .5*c*dt**2-s*w*dt**3/3
    else:
        end = theta+w*dt
        se, ce = math.sin(end), math.cos(end)
        a, b = (se-s)/w, (c-ce)/w
        da, db = (ce-c)/w, (se-s)/w
        aw = (dt*ce*w-(se-s))/(w*w)
        bw = (dt*se*w-(c-ce))/(w*w)
    x[0] += v*a
    x[1] += v*b
    x[2] = wrap(theta+w*dt)
    f[0,2], f[1,2] = v*da, v*db
    f[0,3], f[1,3] = a, b
    f[0,4], f[1,4], f[2,4] = v*aw, v*bw, dt
    return x, f


@dataclass
class Observation:
    stamp: float
    received: float
    state: np.ndarray


class DelayedEKF:
    """源时刻后验与当前预测副本分开。单一有序P3流，无需回滚已融合的其他流。"""
    def __init__(self, linear_noise=.1, angular_noise=.1, position_std=.005, yaw_std=.01):
        self.qv, self.qw = linear_noise**2, angular_noise**2
        self.r = np.diag([position_std**2, position_std**2, yaw_std**2])
        self.state = self.covariance = self.stamp = None

    def propagate(self, state, covariance, dt):
        x, f = transition(state, dt)
        mid = state[2]+state[4]*dt/2
        g = np.array([math.cos(mid),math.sin(mid),0.,0.,0.])
        ev, ew, ey = np.eye(5)[3], np.eye(5)[4], np.eye(5)[2]
        q = self.qv*(np.outer(g,g)*dt**3/3 + (np.outer(g,ev)+np.outer(ev,g))*dt**2/2 + np.outer(ev,ev)*dt)
        q += self.qw*(np.outer(ey,ey)*dt**3/3 + (np.outer(ey,ew)+np.outer(ew,ey))*dt**2/2 + np.outer(ew,ew)*dt)
        # 少量独立平面随机游走允许P3横向位置变化，不拟合未知外参。
        q[:2,:2] += np.eye(2)*.002**2*dt
        p = f@covariance@f.T+q
        return x, (p+p.T)/2

    def correct(self, observation):
        if self.stamp is not None and observation.stamp <= self.stamp:
            return False
        if self.stamp is None:
            # 不把可能为0的P3 twist当作已验证速度；由后续pose序列估计。
            self.state = np.r_[observation.state[:3],0.,0.]
            self.covariance = np.diag([self.r[0,0],self.r[1,1],self.r[2,2],.1**2,.1**2])
        else:
            x,p = self.propagate(self.state,self.covariance,observation.stamp-self.stamp)
            innovation = observation.state[:3]-x[:3]
            innovation[2] = wrap(innovation[2])
            k = np.linalg.solve(p[:3,:3]+self.r,p[:3,:]).T
            self.state = x+k@innovation
            self.state[2] = wrap(self.state[2])
            kh = np.eye(5); kh[:,:3] -= k
            self.covariance = kh@p@kh.T+k@self.r@k.T
            self.covariance = (self.covariance+self.covariance.T)/2
        self.stamp = observation.stamp
        return True

    def predict(self, target):
        if self.stamp is None or target < self.stamp:
            return None
        return self.propagate(self.state,self.covariance,target-self.stamp)[0]


def read_log(path):
    estimates, commands, observations = [], [], {}
    pattern = re.compile(r'\[(\d+\.\d+)\].*?\]: ([A-Z_]+): (.*)')
    for line in Path(path).open(encoding='utf-8',errors='replace'):
        m = pattern.search(line)
        if not m:
            continue
        try:
            data = json.loads(m[3])
        except ValueError:
            continue
        t = float(m[1])
        if m[2]=='MPCC_COMMAND':
            identity=data.get('source_identity') or {}
            commands.append((t,data.get('actual_published_vw',[data.get('v',0.),data.get('w',0.)]),identity.get('goal')))
        if m[2]!='COMMAND_STATE_ESTIMATE' or data.get('source_stamp_ns') is None or data.get('estimated_state') is None:
            continue
        state = np.asarray(data['measured_state'],float)
        stamp = data['source_stamp_ns']/1e9
        if stamp not in observations:
            observations[stamp] = Observation(stamp,t,state)
        elif not np.array_equal(observations[stamp].state,state):
            raise ValueError('same source stamp has inconsistent state')
        estimates.append(dict(t=t,stamp=stamp,online=np.asarray(data['estimated_state'],float)))
    if not estimates:
        raise ValueError('no valid estimates')
    obs = sorted(observations.values(),key=lambda o:o.received)
    if any(b.stamp<=a.stamp for a,b in zip(obs,obs[1:])):
        raise ValueError('prototype assumes advancing single P3 stream')
    return estimates,commands,obs



def prediction_states(event,last,previous,ekf,target):
    dt=last.stamp-previous.stamp
    dtheta=wrap(last.state[2]-previous.state[2]);mid=previous.state[2]+dtheta/2
    delta=last.state[:2]-previous.state[:2]
    signed=float(delta@[math.cos(mid),math.sin(mid)])/dt
    diffstate=np.r_[last.state[:3],signed,dtheta/dt]
    return dict(online=transition(event['online'],target-event['t'])[0],
                hold=last.state.copy(),p3_twist_cv=transition(last.state,target-last.stamp)[0],
                two_pose_cv=transition(diffstate,target-last.stamp)[0],delayed_ekf=ekf.predict(target))


def interpolated_evaluation(estimates,observations,commands,parameters):
    # 辅助指标：在每条日志预测的精确时刻，以P3源点插值评分，不外推标签。
    stamps=[o.stamp for o in observations];j=0;f=DelayedEKF(**parameters);rows=[]
    for event in estimates:
        now=event['t'];i=bisect_right(stamps,now)-1
        if i<0 or i+1>=len(observations):continue
        left,right=observations[i:i+2];gap=right.stamp-left.stamp
        if gap>4 or right.received<=now:continue
        while j<len(observations) and observations[j].received<=now:
            f.correct(observations[j]);j+=1
        if j<2:continue
        fraction=(now-left.stamp)/gap
        truth=left.state.copy()+fraction*(right.state-left.state)
        truth[2]=wrap(left.state[2]+fraction*wrap(right.state[2]-left.state[2]))
        predictions=prediction_states(event,observations[j-1],observations[j-2],f,now)
        goal=next((c[2] for c in reversed(commands) if c[0]<=now and c[2] is not None),None)
        errors={}
        for name,state in predictions.items():
            predicted_d=None if goal is None else float(np.linalg.norm(state[:2]-goal))
            truth_d=None if goal is None else float(np.linalg.norm(truth[:2]-goal))
            errors[name]=dict(position_m=float(np.linalg.norm(state[:2]-truth[:2])),
                             yaw_rad=abs(wrap(state[2]-truth[2])),
                             early_target=bool(predicted_d is not None and predicted_d<=.02 and truth_d>.02),
                             premature_position=bool(predicted_d is not None and predicted_d<=.02 and truth_d>.1))
        rows.append(dict(t=now,label_received=right.received,source_gap_s=gap,errors=errors))
    return rows


def correction_jumps(estimates,observations,parameters,split):
    f=DelayedEKF(**parameters);jumps=[];durations=[];j=0;by_stamp={}
    for event in estimates:
        while j<len(observations) and observations[j].received<=event['t']:
            o=observations[j];before=f.predict(o.received)
            t=time.perf_counter();f.correct(o);durations.append((time.perf_counter()-t)*1e6)
            after=f.predict(o.received)
            if before is not None and o.received>=split:
                jump=(float(np.linalg.norm(after[:2]-before[:2])),abs(wrap(after[2]-before[2])))
                jumps.append(jump);by_stamp[o.stamp]=jump
            j+=1
        t=time.perf_counter();f.predict(event['t']);durations.append((time.perf_counter()-t)*1e6)
    online=[];paired_ekf=[]
    for before,after in zip(estimates,estimates[1:]):
        dt=after['t']-before['t']
        if after['t']<split or before['stamp']==after['stamp'] or dt>.2:continue
        projected=transition(before['online'],dt)[0]
        if after['stamp'] not in by_stamp:continue
        online.append((float(np.linalg.norm(after['online'][:2]-projected[:2])),abs(wrap(after['online'][2]-projected[2]))))
        paired_ekf.append(by_stamp[after['stamp']])
    def report(a):
        return dict(n=len(a),position_p95_cm=float(np.percentile([x[0]*100 for x in a],95)),
                    yaw_p95_deg=float(np.percentile([x[1]*180/math.pi for x in a],95)))
    return dict(online_approx=report(online),delayed_ekf_matched=report(paired_ekf),delayed_ekf_all=report(jumps),
                ekf_operation_median_us=float(np.median(durations)),ekf_operation_p95_us=float(np.percentile(durations,95)),
                caveat='Online jump approximated across adjacent log timestamps; EKF before/after at exact visibility time. Desktop timing only.')


def evaluate(estimates, observations, commands, parameters):
    """未来P3源时刻是评分标签；各方法仅用最近在线预测时刻已可见的数据。"""
    times = [r['t'] for r in estimates]
    command_times = [r[0] for r in commands]
    ekf = DelayedEKF(**parameters)
    j = 0
    rows=[]
    for truth in observations:
        i = bisect_right(times,truth.stamp)-1
        if i<0:
            continue
        event=estimates[i]; known=event['t']; alignment=truth.stamp-known
        if alignment>.1 or truth.received<=truth.stamp:
            continue
        while j<len(observations) and observations[j].received<=known:
            ekf.correct(observations[j]);j+=1
        if j<2:
            continue
        last,previous=observations[j-1],observations[j-2]
        assert last.received<=known<truth.received and last.stamp<truth.stamp
        predictions = prediction_states(event,last,previous,ekf,truth.stamp)
        ci=bisect_right(command_times,known)-1
        goal = next((commands[k][2] for k in range(ci,-1,-1) if commands[k][2] is not None),None)
        errors={}
        for name,state in predictions.items():
            position=float(np.linalg.norm(state[:2]-truth.state[:2]));yaw=abs(wrap(state[2]-truth.state[2]))
            predicted_d=None if goal is None else float(np.linalg.norm(state[:2]-goal))
            truth_d=None if goal is None else float(np.linalg.norm(truth.state[:2]-goal))
            errors[name]=dict(position_m=position,yaw_rad=yaw,state=state.tolist(),predicted_goal_m=predicted_d,
                              early_target=bool(predicted_d is not None and predicted_d<=.02 and truth_d>.02),
                              premature_position=bool(predicted_d is not None and predicted_d<=.02 and truth_d>.1))
        rows.append(dict(t=truth.stamp,known_until=known,latest_visible_source=last.stamp,
                         latest_visible_received=last.received,label_received=truth.received,
                         alignment_s=alignment,observation_age_s=truth.stamp-last.stamp,
                         source_gap_s=last.stamp-previous.stamp,truth=truth.state.tolist(),goal=goal,errors=errors))
    return rows


def metrics(rows):
    answer={}
    for name in rows[0]['errors']:
        p=np.array([r['errors'][name]['position_m'] for r in rows]);y=np.array([r['errors'][name]['yaw_rad'] for r in rows])
        answer[name]=dict(n=len(p),position_rmse_cm=float(np.sqrt(np.mean(p*p))*100),position_p95_cm=float(np.percentile(p,95)*100),
                          yaw_rmse_deg=float(np.sqrt(np.mean(y*y))*180/math.pi),yaw_p95_deg=float(np.percentile(y,95)*180/math.pi),
                          early_target_samples=sum(r['errors'][name]['early_target'] for r in rows),
                          premature_position_samples=sum(r['errors'][name]['premature_position'] for r in rows))
    return answer



def write_plot(output, rows, split, summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    selected=[r for r in rows if r['t']>=split]
    methods=[('online','Current predictor','#c43c39'),('p3_twist_cv','P3 velocity + CV','#2474ae'),('delayed_ekf','Delayed pose EKF','#278346')]
    fig,axes=plt.subplots(2,2,figsize=(12,7))
    for key,label,color in methods:
        t=[r['t']-split for r in selected]
        axes[0,0].plot(t,[r['errors'][key]['position_m']*100 for r in selected],'.-',label=label,color=color,alpha=.8)
        axes[0,1].plot(t,[r['errors'][key]['yaw_rad']*180/math.pi for r in selected],'.-',label=label,color=color,alpha=.8)
    axes[0,0].set_ylabel('Position error (cm)');axes[0,1].set_ylabel('Heading error (deg)')
    for ax in axes[0]:ax.set_xlabel('Seconds into held-out half');ax.grid(alpha=.2);ax.legend(fontsize=8)
    for ax,metric,label in [(axes[1,0],'position_rmse_cm','Position RMSE (cm)'),(axes[1,1],'yaw_rmse_deg','Heading RMSE (deg)')]:
        values=[summary['held_out'][k][metric] for k,_,_ in methods]
        ax.bar([label for _,label,_ in methods],values,color=[c for _,_,c in methods])
        for i,v in enumerate(values):ax.text(i,v,f'{v:.2f}',ha='center',va='bottom')
        ax.set_ylabel(label);ax.set_ylim(0,max(values)*1.2);ax.grid(axis='y',alpha=.2)
    fig.suptitle('Causal offline comparison: later P3 source poses as reference (not ground truth)')
    fig.tight_layout();fig.savefig(output/'comparison.png',dpi=160);plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    started=time.perf_counter();estimates,commands,obs=read_log(args.log)
    split=(estimates[0]['t']+estimates[-1]['t'])/2
    # 测量噪声是预先声明的实验假设；仅前半段评分选择过程噪声。
    trials=[]
    for qv in (.03,.1,.3):
        for qw in (.03,.1,.3):
            parameters=dict(linear_noise=qv,angular_noise=qw,position_std=.005,yaw_std=.01)
            rows=evaluate(estimates,obs,commands,parameters)
            train=[r for r in rows if r['label_received']<split]
            m=metrics(train)['delayed_ekf']
            score=m['position_rmse_cm']/10+m['yaw_rmse_deg']/5
            trials.append(dict(parameters=parameters,score=score,training=m))
    chosen=min(trials,key=lambda r:r['score'])
    rows=evaluate(estimates,obs,commands,chosen['parameters'])
    train=[r for r in rows if r['label_received']<split]
    test=[r for r in rows if r['t']>=split]
    sensitivity=[]
    for scale in (.5,2.):
        params=dict(chosen['parameters']);params['position_std']*=scale;params['yaw_std']*=scale
        candidate=evaluate(estimates,obs,commands,params)
        sensitivity.append(dict(measurement_std_scale=scale,metrics=metrics([r for r in candidate if r['t']>=split])))
    strict=[r for r in test if r['alignment_s']<=.05]
    auxiliary=interpolated_evaluation(estimates,obs,commands,chosen['parameters'])
    auxiliary_test=[r for r in auxiliary if r['t']>=split]
    eligible=[o for o in obs if o.stamp>=split]
    coverage=dict(holdout_source_points=len(eligible),scored=len(test),
                  excluded_alignment_gt_100ms=sum(o.stamp-estimates[bisect_right([e['t'] for e in estimates],o.stamp)-1]['t']>.1 for o in eligible))
    groups={'steady':[],'transition':[]}
    command_times=[c[0] for c in commands]
    for row in test:
        lo=bisect_right(command_times,row['t']-4);hi=bisect_right(command_times,row['t'])
        modes={(1 if c[1][1]>.005 else -1 if c[1][1]<-.005 else 0,abs(c[1][0])>.005) for c in commands[lo:hi]}
        groups['transition' if len(modes)>1 else 'steady'].append(row)
    summary=dict(log_sha256=hashlib.sha256(args.log.read_bytes()).hexdigest(),source_commit='12b05e9',
                 split_wall_s=split,estimate_count=len(estimates),unique_observations=len(obs),
                 source_gap_median_s=float(np.median(np.diff([o.stamp for o in obs]))),
                 receive_delay_median_s=float(np.median([o.received-o.stamp for o in obs])),
                 chosen=chosen,training=metrics(train),held_out=metrics(test),alignment_le_50ms=metrics(strict),
                 coverage=coverage,groups={k:metrics(v) for k,v in groups.items()},
                 interpolated_auxiliary=metrics(auxiliary_test),
                 interpolated_gap_le_2s=metrics([r for r in auxiliary_test if r['source_gap_s']<=2]),
                 correction_jumps=correction_jumps(estimates,obs,chosen['parameters'],split),
                 holdout_alignment_max_s=max(r['alignment_s'] for r in test),
                 holdout_alignment_median_s=float(np.median([r['alignment_s'] for r in test])),
                 measurement_sensitivity=sensitivity,elapsed_s=time.perf_counter()-started,
                 limitations=['P3 reference, not independent ground truth','First log visibility approximates reception',
                              'Online baseline advanced <=100ms using its logged v/w','Frozen original commands, not counterfactual closed loop',
                              'One goal and one run; late half mostly terminal manoeuvres','No command-driven response calibration in CV EKF'])
    (args.output/'results.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    (args.output/'scored-samples.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
    (args.output/'parameter-trials.json').write_text(json.dumps(trials,indent=2),encoding='utf-8')
    write_plot(args.output,rows,split,summary)
    print(json.dumps({k:summary[k] for k in ('held_out','coverage','interpolated_auxiliary','correction_jumps','elapsed_s')},indent=2))


if __name__=='__main__':
    main()