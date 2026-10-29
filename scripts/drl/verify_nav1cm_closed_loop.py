#!/usr/bin/env python3
"""固定Actor的真实ROS推理反馈闭环；不创建训练环境、Replay或优化器。"""
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import verify_nav1cm_backend as base
from inference_only_guard import install
from inference_closure_grade import grade


def descendants(pid):
    result={};pending=[pid]
    while pending:
        current=pending.pop()
        try:
            stat=Path('/proc/'+str(current)+'/stat').read_text().split()
            result[current]=stat[21]
            pending.extend(int(x) for x in Path('/proc/'+str(current)+'/task/'+str(current)+'/children').read_text().split())
        except (FileNotFoundError,ProcessLookupError,PermissionError):pass
    return result


def run(args,d):
    if os.environ.get('ROS_DOMAIN_ID')!='92' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise ValueError('isolated domain92/localhost required')
    import rclpy
    from rclpy.parameter import Parameter
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import Point32
    from lunar_pure_exploration_msgs.msg import PureExplorationTask
    from lunar_drl_exploration.inference_node import attach_node
    from lunar_drl_exploration.navigation_backend import owned_nav1cm_specs
    from lunar_drl_exploration.processes import ros_arguments
    from lunar_drl_exploration.ros_messages import velocity_is_stopped
    art=args.artifact;ns=d['namespace'];events=[];children=[];logs=[];owned={};runtime=None;node=None
    accepted=set();error=None;summary={};task_pub=None;python_expected={}
    python_expected[os.getpid()]=Path('/proc/self/stat').read_text().split()[21]
    event_stream=(art/'events.jsonl').open('x',buffering=1)
    trace=(art/'runtime.jsonl').open('x',buffering=1)
    base.write_json(art/'inference-config.json',d['inference_config'])
    def emit(kind,**values):
        event=dict(kind=kind,wall=time.monotonic(),**values);events.append(event)
        event_stream.write(json.dumps(event,default=lambda v:vars(v))+'\n')
    def spawn(name,command):
        log=(art/(name+'.log')).open('x');logs.append(log)
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append((name,child));emit('process',name=name,pid=child.pid,command=command)
    def inspect_accept():
        a=runtime.adapter;state=a.execution_state();session=state['session_id'];serial=a._goal_serial
        if state['accepted'] and (serial,session) not in accepted:
            accepted.add((serial,session));emit('accepted',serial=serial,session_id=session,goal=list(a.sent_goal))
    def spin_until(predicate,seconds,phase):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            for name,child in children:
                if child.poll() is not None:raise RuntimeError(name+' exited during '+phase)
                owned.update(descendants(child.pid))
            for pid,start in owned.items():
                try:
                    if 'python' in os.readlink('/proc/'+str(pid)+'/exe').lower():python_expected[pid]=start
                except (FileNotFoundError,ProcessLookupError,PermissionError):pass
            rclpy.spin_once(node,timeout_sec=.05)
            if runtime is not None:
                inspect_accept()
                if runtime.last_error:raise RuntimeError(runtime.last_error)
            if predicate():return
        raise RuntimeError(phase+' timeout')
    def publish(command,task_id):
        msg=PureExplorationTask();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
        msg.task_id=task_id;msg.command=command
        msg.boundary.points=[Point32(x=float(x),y=float(y),z=0.) for x,y in d['scene']['task_polygon']]
        task_pub.publish(msg);emit('task_published',command=command,task_id=task_id)
    def cancel_current(task_id):
        a=runtime.adapter;inspect_accept();state=a.execution_state()
        if not state['accepted']:raise RuntimeError('cancel requires accepted current Action')
        emit('cancel',serial=a._goal_serial,session_id=state['session_id'])
        publish(PureExplorationTask.CANCEL,task_id)
        spin_until(lambda:runtime.state=='CANCELED' and not a.inflight and velocity_is_stopped(a.velocity),15.,'topic cancel/stop')
    try:
        rclpy.init()
        pre=rclpy.create_node('drl_closure_preflight')
        try:
            end=time.monotonic()+1.5
            while time.monotonic()<end:rclpy.spin_once(pre,timeout_sec=.1)
            others=[(n,s) for n,s in pre.get_node_names_and_namespaces() if n!='drl_closure_preflight']
            if others:raise RuntimeError('domain92 occupied: '+repr(others))
        finally:pre.destroy_node()
        spawn('plant',[sys.executable,str(Path(__file__).resolve()),'--plant','--artifact',str(art)])
        specs=owned_nav1cm_specs(ns,platform_config=d['platform'],build_dir=str(art/'planner-cache'),use_sim_time=False)
        for i,(package,entry,params) in enumerate(specs):
            command=[str(Path(get_package_prefix(package))/'lib'/package/entry),*ros_arguments(ns,params)]
            command[command.index('--log-level')+1]='info';spawn('node'+str(i),command)
        values=dict(actor_path=d['actor'],actor_config_path=str(art/'inference-config.json'),navigation_backend='nav1cm',
            platform_config=d['platform'],task_topic=ns+'/task',status_topic=ns+'/status',action_name=ns+'/navigate',
            policy_map_service=ns+'/get_map',odometry_topic=ns+'/odom',tf_topic=ns+'/tf',tf_static_topic=ns+'/tf_static',global_route_topic=ns+'/route')
        node=rclpy.create_node('drl_closure_inference',namespace=ns,parameter_overrides=[Parameter(k,value=v) for k,v in values.items()])
        runtime=attach_node(node);a=runtime.adapter
        original_begin=a.begin_goal;original_finish=a._finish
        def begin(goal):
            serial=original_begin(goal);emit('goal',serial=serial,goal=list(goal));return serial
        def finish(result):
            goal=list(a.sent_goal);serial=a._goal_serial
            status=None
            if a._result_future is not None and a._result_future.done():status=int(a._result_future.result().status)
            inspect_accept();value=original_finish(result);execution=a.last_goal_execution
            pose=a.pose;xy=math.hypot(pose.x-goal[0],pose.y-goal[1]) if pose else math.inf
            yaw=abs(math.atan2(math.sin(pose.yaw-goal[2]),math.cos(pose.yaw-goal[2]))) if pose else math.inf
            emit('finish',serial=serial,session_id=execution['session_id'],accepted=execution['accepted'],goal=goal,
                outcome=int(result.outcome),reason_code=str(result.reason_code),action_status=status,
                position_error_m=xy,yaw_error_rad=yaw,velocity=list(a.velocity),pose_age_s=time.monotonic()-a.pose_received_monotonic,
                stamp_ns=int(a.processed_stamp_ns),revision=int(a.snapshot.revision) if a.snapshot else 0)
            return value
        a.begin_goal=begin;a._finish=finish
        original_policy=runtime.policy
        class ObservedPolicy:
            actor=original_policy.actor
            def __call__(self,observation):
                if self.actor.training:raise RuntimeError('Actor is not eval')
                action=original_policy(observation)
                index=1+sum(e['kind']=='decision' for e in events)
                emit('decision',index=index,stamp_ns=int(a.processed_stamp_ns),revision=int(a.snapshot.revision),
                    selected_goal=observation.goals[action].tolist(),schema=observation.schema,
                    completed_before=sum(e['kind']=='finish' and e['outcome']==0 for e in events))
                base.write_json(art/('observation-'+str(index)+'.json'),vars(observation))
                return action
        runtime.policy=ObservedPolicy()
        task_pub=node.create_publisher(PureExplorationTask,ns+'/task',10)
        spin_until(lambda:a.ready and a.pose is not None and task_pub.get_subscription_count()>0,30.,'input readiness')
        pubs=node.get_publishers_info_by_topic(ns+'/cmd_vel')
        if len(pubs)!=1 or node.get_publishers_info_by_topic('/Car/T5/Car_Cmd_Vel'):
            raise RuntimeError('unexpected command publisher graph')
        base.write_json(art/'graph.json',dict(topics=node.get_topic_names_and_types(),command_publishers=[dict(node_name=p.node_name,namespace=p.node_namespace) for p in pubs]))
        task_id='inference-closure';publish(PureExplorationTask.START,task_id)
        end=time.monotonic()+d['duration_s'];closed=False
        while time.monotonic()<end:
            spin_until(lambda:True,.2,'inference tick')
            trace.write(json.dumps(dict(wall=time.monotonic(),status=runtime.status(),serial=a._goal_serial,
                execution=a.execution_state(),pose=vars(a.pose) if a.pose else None,velocity=a.velocity,
                revision=a.snapshot.revision if a.snapshot else None,stamp_ns=a.processed_stamp_ns),default=lambda v:vars(v))+'\n')
            completed={(e['serial'],e['session_id']) for e in events if e['kind']=='finish' and e['outcome']==0 and e['reason_code']=='GOAL_REACHED'}
            decisions=[e for e in events if e['kind']=='decision']
            if len(completed)>=args.minimum_completed and decisions and decisions[-1]['completed_before']>=args.minimum_completed and a.execution_state()['accepted']:
                closed=True;break
        if not closed:raise RuntimeError('natural completion/next Actor decision budget exceeded')
        cancel_current(task_id);cancel_stamp=int(a.processed_stamp_ns);cancel_serial=a._goal_serial
        reset_id='inference-closure-reset';publish(PureExplorationTask.START,reset_id)
        spin_until(lambda:runtime.status()['task_id']==reset_id and a._goal_serial>cancel_serial and a.execution_state()['accepted'] and int(a.processed_stamp_ns)>cancel_stamp,10.,'new task feedback reset')
        emit('reset',task_id=reset_id,serial=a._goal_serial,session_id=a.execution_state()['session_id'],stamp_ns=int(a.processed_stamp_ns),accepted=True)
        cancel_current(reset_id)
        summary.update(state=runtime.state,stopped=velocity_is_stopped(a.velocity),inflight=a.inflight)
    except BaseException as exc:
        error=repr(exc);summary['error']=error
        if runtime is not None:
            runtime.cancel()
            try:spin_until(lambda:not runtime.adapter.inflight and velocity_is_stopped(runtime.adapter.velocity),10.,'failure cleanup')
            except BaseException as cleanup:summary['cleanup_error']=repr(cleanup)
    finally:
        cleanup_failures=[]
        for name,operation in [('adapter',lambda:runtime.adapter.close() if runtime is not None else None),
                               ('node',lambda:node.destroy_node() if node is not None else None),
                               ('rclpy',lambda:rclpy.shutdown() if rclpy.ok() else None)]:
            try:operation()
            except BaseException as exc:cleanup_failures.append(name+': '+repr(exc))
        if cleanup_failures:summary['cleanup_failures']=cleanup_failures;error=error or repr(cleanup_failures)
        exits={};forced=False
        for name,child in reversed(children):
            owned.update(descendants(child.pid))
            try:
                if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=8.)
                except subprocess.TimeoutExpired:
                    forced=True;os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=5.)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=2.)
                exits[name]=child.returncode
            except BaseException as exc:
                forced=True;exits[name]=child.poll();summary.setdefault('exit_errors',[]).append(name+': '+repr(exc))
        for log in logs:log.close()
        remaining=[]
        for pid,start in owned.items():
            try:
                if Path('/proc/'+str(pid)+'/stat').read_text().split()[21]==start:remaining.append(pid)
            except FileNotFoundError:pass
        attempts=[];starts=[]
        for f in (art/'guard').glob('*.jsonl'):
            for line in f.read_text().splitlines():
                row=json.loads(line)
                if row['kind']=='startup':starts.append(row)
                elif row['kind']!='constructor_guard':attempts.append(row)
        frozen={key:base.digest(d[key]) for key in ('actor','config','platform')}
        summary.update(child_exits=exits,forced_exit=forced,workers_remaining=remaining,guard_attempts=attempts,
            guard_startups=starts,guard_expected_processes=[dict(pid=p,starttime=t) for p,t in python_expected.items()],frozen_after=frozen,frozen_unchanged=all(frozen[k]==d[k+'_sha256'] for k in frozen))
        event_stream.close();trace.close()
        base.write_json(art/'summary.json',summary)
        verdict=grade(events,summary,minimum_completed=args.minimum_completed)
        if error:verdict['ok']=False;verdict.setdefault('errors',[]).append(error)
        base.write_json(art/'verdict.json',verdict)
    print(json.dumps(verdict))
    return 0 if verdict['ok'] else 2


def main():
    p=base.parser();p.set_defaults(duration=90.)
    p.add_argument('--minimum-completed',type=int,default=2)
    args=p.parse_args()
    install(args.artifact/'guard')
    if args.plant:return base.plant_worker(args.artifact)
    if args.backend!='nav1cm' or args.minimum_completed<2:raise ValueError('nav1cm and >=2 completed goals required')
    d=base.declaration(args);d['namespace']='/lunar_inference/nav1cm_closure'
    d['closure_script_sha256']=base.digest(__file__);d['grade_sha256']=base.digest(Path(__file__).with_name('inference_closure_grade.py'))
    d['guard_sha256']=base.digest(Path(__file__).with_name('inference_only_guard.py'));d['minimum_completed']=args.minimum_completed
    d['criteria'].update(arrival_claim=True,next_actor_decision=True,reset_via_topic=True)
    if d['actor_sha256']!='6951491300d144cb9e170859804707eb3613bd0fafd956ae205549eab0120f7d':raise ValueError('fixed regression Actor identity mismatch')
    if args.declare_only:
        base.write_json(args.artifact/'declaration.json',d);print(json.dumps(dict(declaration=str(args.artifact/'declaration.json'))));return 0
    if not args.run:raise ValueError('declare-only before run')
    if json.loads((args.artifact/'declaration.json').read_text())!=d:raise ValueError('frozen declaration drift')
    return run(args,d)


if __name__=='__main__':raise SystemExit(main())
