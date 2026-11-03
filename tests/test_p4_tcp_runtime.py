"""P4域继承接线与车辆租约回归；不依赖ROS。"""
from types import SimpleNamespace as S
import pytest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
from scripts.simulation.p4_tcp_runtime import CONTROL_DOMAIN, P3_SOURCE_DOMAIN, input_bridge, launch_settings


def test_control_launch_uses_existing_wheel_limits_and_private_adapter_input():
    assert (CONTROL_DOMAIN,P3_SOURCE_DOMAIN)==(57,57)
    args=launch_settings(ROOT,'/p4/cache','/p4/test-wheel.yaml',dict(command_hold_s=1.2,odometry_timeout_s=4.5,p3_map_timeout_s=6.,mppi_time_steps=140,mppi_compute_envelope_s=.2))
    assert args['command_topic']=='/Car/T4/control/cmd_vel'
    assert args['goal_topic']=='/Car/T4/navigation/goal'
    assert args['enable_controller']=='true' and args['exploration_strategy']=='frontier'
    assert args['initial_travel_direction']=='forward'
    assert args['platform_config']=='/p4/test-wheel.yaml'


def test_input_bridge_preserves_messages_stamps_and_latched_tf():
    subscriptions=[];published=[]
    def publisher(cls,topic,qos):
        return S(publish=lambda msg:published.append((topic,msg,qos)))
    def subscription(cls,topic,callback,qos):
        subscriptions.append((topic,callback,qos));return object()
    source=S(create_subscription=subscription);target=S(create_publisher=publisher)
    classes=dict(map=object,odom=object,tf=object)
    handles=input_bridge(source,target,classes,dict(map='latched-map',odom='odom',tf='tf',tf_static='latched-static'),target_domain=10)
    assert len(subscriptions)==4 and len(handles)==8
    for topic,callback,qos in subscriptions:
        msg=S(header=S(stamp=123));callback(msg)
        assert published[-1]==(topic,msg,qos)
    assert subscriptions[-1][0]=='/tf_static' and subscriptions[-1][2]=='latched-static'


def test_tf_output_accepts_reliable_rviz_without_changing_source_subscription():
    subscriptions=[];publishers=[]
    source=S(create_subscription=lambda cls,topic,callback,qos:subscriptions.append((topic,qos)))
    target=S(create_publisher=lambda cls,topic,qos:(publishers.append((topic,qos)) or S(publish=lambda msg:None)))
    input_bridge(source,target,dict(map=object,odom=object,tf=object),dict(map=1,odom=2,tf='best_effort',tf_output='reliable',tf_static=4),target_domain=10)
    assert ('/tf','best_effort') in subscriptions and ('/tf','reliable') in publishers


def test_p4_profile_has_no_map_time_expiry_and_preflight_uses_same_policy():
    from pathlib import Path
    import yaml
    from scripts.simulation.p3_input_timing import record_sample,runtime_input_health
    profile=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/tcp_p3_feedback.yaml').read_text(encoding='utf-8'))
    assert profile['p3_map_timeout_s']==0.
    args=launch_settings(ROOT,'/p4/cache','/p4/wheel.yaml',profile)
    assert float(args['p3_map_timeout_s'])==0.
    sample=record_sample(None,95_000_000_000,100_000_000_000,10.)
    assert runtime_input_health('map',sample,1100_000_000_000,1010.,odometry_timeout_s=4.5,p3_map_timeout_s=profile['p3_map_timeout_s'])['ready']



def profile():
    return {'capability':{'maximum_forward_speed_mps':.2,'maximum_reverse_speed_mps':.2,'maximum_spin_rate_radps':1.}}


def message(v=0.,w=0.):return S(linear=S(x=v),angular=S(z=w))


def telemetry(**changes):
    values=dict(vx=0.,vy=0.,wz=0.)
    values.update(changes);return S(**values)


def relay_fixture():
    from scripts.simulation import p4_tcp_runtime as m
    events=[];clock=[0.]
    relay=m.TcpCommandRelay(m.low_speed_profile(profile()),
        publish_velocity=lambda msg:events.append(('velocity',msg.linear.x,msg.angular.z)),
        publish_mode=lambda mode:events.append(('mode',mode)),
        publish_lease=lambda active:events.append(('lease',active)),
        make_zero=message,clock=lambda:clock[0])
    relay.update_connected(S(data=True))
    return relay,events,clock


def test_low_speed_profile_is_shared_capped_and_does_not_mutate_source():
    from scripts.simulation.p4_tcp_runtime import low_speed_profile
    original=profile();run=low_speed_profile(original);test=low_speed_profile(original,linear_mps=.05,angular_rps=.1)
    assert run['capability']['maximum_forward_speed_mps']==.1
    assert test['capability']['maximum_forward_speed_mps']==.05
    assert original['capability']['maximum_forward_speed_mps']==.2
    original['capability']['maximum_reverse_speed_mps']=.03
    assert low_speed_profile(original)['capability']['maximum_reverse_speed_mps']==.03


def test_private_launch_uses_same_controller_budgets_and_initial_direction():
    budgets=dict(command_hold_s=1.2,odometry_timeout_s=4.5,p3_map_timeout_s=0.,mppi_time_steps=140,mppi_compute_envelope_s=.2)
    args=launch_settings(ROOT,'/p4/cache','/p4/wheel',budgets,prefix='/check',goal_topic='/check/goal')
    assert args['command_topic']=='/check/control/cmd_vel' and args['action_name']=='/check/navigation/navigate_to_pose'
    assert args['initial_travel_direction']=='forward'
    assert float(args['p3_map_timeout_s'])==0. and args['goal_topic']=='/check/goal'


def test_relay_preflight_publishes_nothing_and_execution_needs_no_task_status():
    r,events,clock=relay_fixture();r.update_telemetry(telemetry())
    assert not r.receive(message(.02)) and not events
    r.acquire();events.clear()
    assert r.receive(message(.02))
    assert events==[('mode','auto'),('velocity',.02,0.)]
    assert r.receive(message(.03)) and r.forwarded_nonzero==2
    assert events.count(('mode','auto'))==2


@pytest.mark.parametrize('bad',[message(.11),message(-.11),message(0.,.3),message(float('nan')),message(0.,float('inf'))])
def test_invalid_command_parks_and_is_never_forwarded(bad):
    r,events,clock=relay_fixture();r.acquire();r.update_telemetry(telemetry());events.clear()
    with pytest.raises(ValueError):r.receive(bad)
    assert events==[('velocity',0.,0.),('mode','park')]
    assert r.forwarded_nonzero==0


@pytest.mark.parametrize('gap',[.601,2.,600.])
def test_command_gap_heartbeat_only_renews_lease_and_next_command_recovers(gap):
    r,events,clock=relay_fixture();r.acquire()
    events.clear();clock[0]=gap
    assert r.heartbeat()
    assert events==[('lease',True)]
    r.update_telemetry(telemetry());assert r.receive(message(.02))
    clock[0]+=gap;events.clear()
    assert r.heartbeat() and r.armed
    assert events==[('lease',True)]
    # 底盘 watchdog 已自行 park，relay 无模式反馈；下一条有效输入重申 auto。
    events.clear();assert r.receive(message(.03))
    assert events==[('mode','auto'),('velocity',.03,0.)]


def test_zero_command_and_cleanup_cannot_reenter_auto():
    r,events,clock=relay_fixture();r.acquire();r.update_telemetry(telemetry());r.receive(message(.02))
    assert not r.receive(message()) and events[-1]==('velocity',0.,0.)
    r.begin_stop();before=r.forwarded_nonzero;events.clear()
    assert not r.receive(message(.02));r.heartbeat()
    assert r.forwarded_nonzero==before and ('mode','auto') not in events
    r.release();events.clear();assert not r.receive(message(.02)) and not events


@pytest.mark.parametrize('changes,age,stopped',[({},0.,True),({},.5,False),({},.51,False),({'vy':.02},0.,False),({'wz':.06},0.,False)])
def test_measured_stop_uses_shared_real_telemetry(changes,age,stopped):
    r,events,clock=relay_fixture();r.update_telemetry(telemetry(**changes));clock[0]=age
    assert r.measured_stopped()==stopped


def test_bounded_runner_routes_command_through_shared_adapter_without_reference_gate():
    import ast,collections,math,time
    from pathlib import Path
    path=Path(__file__).resolve().parents[1]/'scripts/simulation/check_tcp_navigation.py'
    tree=ast.parse(path.read_text(encoding='utf-8'))
    callback=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='command_input')
    module=ast.fix_missing_locations(ast.Module(body=[callback],type_ignores=[]))
    seen=[]
    scope=dict(relay=S(armed=False,receive=lambda msg:seen.append(msg)),counts=collections.Counter(),
               event=lambda *a,**k:None,time=time,math=math,armed=False,nonzero=0,forwarded_nonzero=0,
               last_command=None,leased=True,handle=None,reference_accepted=False)
    exec(compile(module,str(path),'exec'),scope)
    command=message(.02);scope['command_input'](command)
    assert seen==[command]



@pytest.mark.parametrize('changes',[{'vy':float('nan')},{'wz':float('nan')}])
def test_invalid_telemetry_is_not_measured_stop(changes):
    r,events,clock=relay_fixture();r.update_telemetry(telemetry(**changes))
    assert not r.measured_stopped()


def test_reverse_limit_and_environment_come_from_shared_configuration(monkeypatch):
    monkeypatch.delenv('ROS_DOMAIN_ID',raising=False)
    monkeypatch.delenv('NAV1CM_BUILD_DIR',raising=False)
    from scripts.simulation.p4_tcp_runtime import TcpCommandRelay,low_speed_profile,runtime_environment
    source=profile();source['capability']['maximum_reverse_speed_mps']=.03
    events=[]
    r=TcpCommandRelay(low_speed_profile(source),publish_velocity=lambda m:events.append(m),
        publish_mode=lambda m:None,publish_lease=lambda b:None,make_zero=message,clock=lambda:0.)
    r.update_connected(S(data=True));r.acquire();r.update_telemetry(telemetry())
    assert r.receive(message(-.02))
    with pytest.raises(ValueError):r.receive(message(-.04))
    env=runtime_environment('/home/yanfa/P4','/home/yanfa/P4/run/test')
    assert env['ROS_DOMAIN_ID']=='57' and env['ROS_LOG_DIR'].replace(chr(92),'/').endswith('test/ros-log')
    assert env['NAV1CM_BUILD_DIR'].replace(chr(92),'/').endswith('.cache/nav1cm')



@pytest.mark.parametrize('bad',[message(float('nan')),message(.2)])
def test_cleanup_discards_invalid_queued_commands_and_releases_lease(bad):
    r,events,clock=relay_fixture();r.acquire();r.update_telemetry(telemetry())
    r.begin_stop()
    assert not r.receive(bad)
    r.release()
    assert events[-1]==('lease',False) and not r.leased
    assert not r.receive(bad)


def test_observer_stale_goal_global_watermark_and_frame_transition():
    from scripts.simulation.p4_tcp_runtime import NavigationObserver
    starts=[];paths=[];goals=[]
    def pose(): return S(header=None,pose=None)
    def path(): return S(header=None,poses=[])
    def odom(t,frame='map'):
        return S(header=S(frame_id=frame,stamp=S(sec=t,nanosec=0)),pose=S(pose=S(position=S(x=1.,y=2.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))))
    o=NavigationObserver(pose,path,starts.append,goals.append,paths.append,4.5)
    o.odometry(odom(10),0.);o.goal(S(header=odom(10).header,pose=None),5.)
    assert starts[-1] is None and paths[-1].poses==[]
    o.odometry(odom(9,'odom'),5.1);assert starts[-1] is None
    o.odometry(odom(11,'odom'),5.2);assert starts[-1].header.frame_id=='odom'
    o.odometry(odom(12,'map'),5.3);assert len(paths[-1].poses)==1 and paths[-1].header.frame_id=='map'
    o.poses=o.poses*2000;o.odometry(odom(13),5.4);assert len(paths[-1].poses)==2000
    o.goal(S(header=odom(13).header,pose=None),5.5);assert starts[-1].header.stamp.sec==13


def display_odom(stamp):
    return S(header=S(frame_id='map',stamp=S(sec=stamp,nanosec=0)),pose=S(pose=S(position=S(x=0.,y=0.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))))


def test_display_mailbox_coalesces_without_blocking_publish_or_old_path_after_goal():
    from scripts.simulation.p4_tcp_runtime import DisplayWorker
    import threading
    entered=threading.Event();release=threading.Event();done=threading.Event();events=[]
    class Observer:
        def odometry(self,msg,when):
            value=msg.header.stamp.sec
            if value==1: entered.set();release.wait(2)
            events.append(('path',value))
            if value==3:done.set()
        def goal(self,msg,when): events.append(('goal',msg))
    worker=DisplayWorker(Observer());worker.start();worker.offer_odom(display_odom(1),0.)
    assert entered.wait(1)
    worker.offer_odom(display_odom(2),1.);worker.offer_goal('old goal',2.);worker.offer_goal('new goal',3.);worker.offer_odom(display_odom(3),4.)
    release.set();assert done.wait(1);worker.stop()
    assert ('goal','old goal') not in events
    assert events[-3:]==[('path',2),('goal','new goal'),('path',3)]


def test_observer_is_enabled_only_for_active_rviz():
    import ast
    from pathlib import Path
    source=(Path(__file__).resolve().parents[1]/'scripts/simulation/p4_tcp_runtime.py').read_text(encoding='utf-8')
    tree=ast.parse(source)
    condition=next(n.test for n in ast.walk(tree) if isinstance(n,ast.If) and any(isinstance(x,ast.Assign) and any(isinstance(v,ast.Call) and isinstance(v.func,ast.Name) and v.func.id=='attach_navigation_observer' for v in ast.walk(x)) for x in n.body))
    expr=compile(ast.Expression(condition),'<condition>','eval')
    assert eval(expr,{},dict(options=S(rviz_config='config',check_only=False)))
    assert not eval(expr,{},dict(options=S(rviz_config=None,check_only=False)))
    assert not eval(expr,{},dict(options=S(rviz_config='config',check_only=True)))


def test_tcp_budget_and_brake_hold_cover_nominal_poll():
    from pathlib import Path
    import yaml
    from scripts.simulation.p4_tcp_runtime import TRANSPORT_TIMEOUT_S
    config=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/tcp_p3_feedback.yaml').read_text(encoding='utf-8'))
    assert config['mppi_compute_envelope_s']==.55 and config['mppi_time_steps']==140
    assert config['p3_map_timeout_s']==0. and config['odometry_timeout_s']==4.5
    assert TRANSPORT_TIMEOUT_S==.5


def test_marker_delete_is_explicit_and_topics_use_marker_protocol():
    from scripts.simulation.p4_tcp_runtime import attach_navigation_observer
    import threading
    events=[]; ready=threading.Event()
    class Marker:
        DELETE=2; ADD=0; ARROW=0
        def __init__(self):
            self.scale=S(x=0.,y=0.,z=0.); self.color=S(r=0.,g=0.,b=0.,a=0.)
    class Node:
        def create_publisher(self,cls,topic,qos):
            def publish(msg):
                events.append((topic,msg))
                if topic.endswith('goal_marker'):ready.set()
            return S(publish=publish)
        def create_subscription(self,cls,topic,callback,qos): return callback
    worker,handles=attach_navigation_observer(Node(),Node(),dict(marker=Marker,path=lambda:S(header=None,poses=[]),pose=lambda:S(header=None,pose=None),odom=object),dict(display=1,odom=2,goal=3),4.5)
    header=S(frame_id='map',stamp=S(sec=1,nanosec=0))
    position=S(x=1.,y=2.,z=0.);orientation=S(x=0.,y=0.,z=0.,w=1.)
    worker.offer_odom(S(header=header,pose=S(pose=S(position=position,orientation=orientation))),0.)
    worker.offer_goal(S(header=header,pose=S()),1.)
    assert ready.wait(1); ready.clear()
    worker.offer_goal(S(header=header,pose=S()),6.)
    assert ready.wait(1);worker.stop()
    deletes=[m for topic,m in events if topic.endswith('/start_marker') and m.action==Marker.DELETE]
    assert len(deletes)==1 and deletes[0].header.frame_id=='map'
    assert [m.action for topic,m in events if topic.endswith('/goal_marker')]==[Marker.ADD,Marker.ADD]
    assert events[-1][1].header.frame_id=='map' and events[-1][1].poses==[]


def test_display_worker_limits_batches_and_stop_wakes_without_period_wait():
    from scripts.simulation.p4_tcp_runtime import DisplayWorker
    import threading,time
    seen=[]; first=threading.Event(); second=threading.Event()
    class Observer:
        def odometry(self,msg,when):
            seen.append((msg.header.stamp.sec,time.monotonic()))
            (first if len(seen)==1 else second).set()
        def goal(self,msg,when): pass
    worker=DisplayWorker(Observer());worker.start();worker.offer_odom(display_odom(1))
    assert first.wait(1)
    for n in range(2,51):worker.offer_odom(display_odom(n))
    assert second.wait(1)
    assert [x[0] for x in seen]==[1,50]
    assert seen[1][1]-seen[0][1]>=.19
    worker.offer_odom(display_odom(51))
    before=time.monotonic();worker.stop();assert time.monotonic()-before<.15
    assert [x[0] for x in seen]==[1,50]


def test_deferred_path_flush_coalesces_goal_and_odom():
    from scripts.simulation.p4_tcp_runtime import NavigationObserver
    paths=[]
    def pose():return S(header=None,pose=None)
    def path():return S(header=None,poses=[])
    header=lambda t:S(frame_id='map',stamp=S(sec=t,nanosec=0))
    p=S(position=S(x=0.,y=0.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))
    o=NavigationObserver(pose,path,lambda _:None,lambda _:None,paths.append,4.5,defer_path=True)
    o.goal(S(header=header(1),pose=p),0.)
    o.odometry(S(header=header(2),pose=S(pose=p)),.1)
    assert not paths
    o.flush();assert len(paths)==1 and len(paths[0].poses)==1
    o.flush();assert len(paths)==1


@pytest.mark.parametrize('prior,first_stamp,first_frame,expected',[ (True,3,'map',3),(False,3,'map',3),(True,1,'other',2),(False,0,'other',3)])
def test_worker_goal_freezes_nearest_legal_before_or_first_after(prior,first_stamp,first_frame,expected):
    from scripts.simulation.p4_tcp_runtime import NavigationObserver,DisplayWorker
    import threading
    starts=[];barrier=threading.Event();release=threading.Event();done=threading.Event()
    def pose():return S(header=None,pose=None)
    def odom(t,frame='map'):
        return S(header=S(frame_id=frame,stamp=S(sec=t,nanosec=0)),pose=S(pose=S(position=S(x=0.,y=0.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))))
    o=NavigationObserver(pose,lambda:S(header=None,poses=[]),lambda p:starts.append(p),lambda _:None,lambda _:done.set(),4.5,defer_path=True)
    class Block:
        def odometry(self,msg,when):o.odometry(msg,when)
        def goal(self,msg,when):
            if msg=='block':barrier.set();release.wait(2)
            else:o.goal(msg,when)
        def flush(self):o.flush()
    w=DisplayWorker(Block(),timeout=4.5);w.start()
    if prior:
        w.offer_odom(odom(2),0.);import time;time.sleep(.25)
    w.offer_goal('block',.1);assert barrier.wait(1)
    w.offer_odom(odom(first_stamp,first_frame),.2)
    w.offer_goal(S(header=odom(1).header,pose=None),.3)
    w.offer_odom(odom(4 if first_stamp==3 else 3),.4);w.offer_odom(odom(5),.5)
    release.set();assert done.wait(2);w.stop()
    assert [p for p in starts if p is not None][-1].header.stamp.sec==expected


def test_worker_keeps_first_advance_after_already_processed_waiting_goal():
    from scripts.simulation.p4_tcp_runtime import NavigationObserver,DisplayWorker
    import threading
    starts=[];goal_seen=threading.Event();done=threading.Event()
    def odom(t):
        return S(header=S(frame_id='map',stamp=S(sec=t,nanosec=0)),pose=S(pose=S(position=S(x=0.,y=0.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))))
    o=NavigationObserver(lambda:S(header=None,pose=None),lambda:S(header=None,poses=[]),starts.append,lambda _:goal_seen.set(),lambda p:done.set() if p.poses else None,4.5,defer_path=True)
    w=DisplayWorker(o,4.5);w.start();w.offer_goal(S(header=odom(1).header,pose=None),0.)
    assert goal_seen.wait(1)
    w.offer_odom(odom(2),.1);w.offer_odom(odom(3),.2)
    assert done.wait(1);w.stop()
    assert [p for p in starts if p is not None][0].header.stamp.sec==2


@pytest.mark.parametrize('late_stamp,late_frame',[(10,'map'),(9,'other')])
def test_duplicate_or_older_odom_cannot_refresh_goal_start(late_stamp,late_frame):
    from scripts.simulation.p4_tcp_runtime import NavigationObserver,DisplayWorker
    import threading
    entered=threading.Event();release=threading.Event();done=threading.Event();starts=[]
    def odom(t,frame='map'):
        return S(header=S(frame_id=frame,stamp=S(sec=t,nanosec=0)),pose=S(pose=S(position=S(x=0.,y=0.,z=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))))
    o=NavigationObserver(lambda:S(header=None,pose=None),lambda:S(header=None,poses=[]),starts.append,lambda _:None,lambda _:done.set(),4.5,defer_path=True)
    class Block:
        def goal(self,msg,when):
            if msg=='block':entered.set();release.wait(2)
            else:o.goal(msg,when)
        def odometry(self,msg,when):o.odometry(msg,when)
        def flush(self):o.flush()
    w=DisplayWorker(Block(),4.5);w.start();w.offer_goal('block',-1.)
    assert entered.wait(1)
    w.offer_odom(odom(10),0.);w.offer_odom(odom(late_stamp,late_frame),.15)
    w.offer_goal(S(header=odom(10).header,pose=None),4.60)
    release.set();assert done.wait(1);assert starts[-1] is None
    done.clear();w.offer_odom(odom(11),4.7);assert done.wait(1);w.stop()
    assert starts[-1].header.stamp.sec==11


def test_connected_relay_keeps_running_through_telemetry_gap():
    r, events, clock = relay_fixture()
    r.update_connected(S(data=True)); r.acquire(); r.update_telemetry(telemetry())
    assert r.receive(message(.02))
    clock[0] = 1.1
    assert r.receive(message(.02)) and r.heartbeat() and r.armed
    r.update_connected(S(data=False))
    assert not r.armed and not r.receive(message(.02))


def test_new_arm_requires_fresh_telemetry_and_connection():
    r, events, clock = relay_fixture(); r.update_connected(S(data=False)); r.acquire(); r.update_telemetry(telemetry())
    assert not r.receive(message(.02))
    r.update_connected(S(data=True)); clock[0] = 1.01
    assert not r.receive(message(.02))


def test_tcp_production_profile_overrides_only_speed_and_feeds_launch_and_relay():
    from pathlib import Path
    import yaml
    from copy import deepcopy
    from scripts.simulation.p4_tcp_runtime import tcp_platform_profile, TcpCommandRelay
    root=Path(__file__).resolve().parents[1]
    base=yaml.safe_load((root/'config/wheel.yaml').read_text(encoding='utf-8'))
    original=deepcopy(base)
    override=yaml.safe_load((root/'config/tcp_wheel_speed.yaml').read_text(encoding='utf-8'))
    merged=tcp_platform_profile(base,override)
    assert profile_command_limits_for_test(merged)==(.1,.1,.1)
    assert base==original
    for key in ('maximum_acceleration_mps2','maximum_braking_deceleration_mps2','maximum_yaw_acceleration_radps2'):
        assert merged['capability'][key]==base['capability'][key]==.5
    assert launch_settings(ROOT,'/p4/cache','/p4/final.yaml',dict(command_hold_s=1.2,odometry_timeout_s=4.5,p3_map_timeout_s=0.,mppi_time_steps=140,mppi_compute_envelope_s=.2))['platform_config']=='/p4/final.yaml'
    relay=TcpCommandRelay(merged,publish_velocity=lambda x:None,publish_mode=lambda x:None,publish_lease=lambda x:None,make_zero=message)
    assert relay.limits==(.1,.1,.1)


def profile_command_limits_for_test(profile):
    from scripts.simulation.p4_tcp_runtime import profile_command_limits
    return profile_command_limits(profile)


def test_tcp_speed_override_rejects_non_speed_or_invalid_values():
    from scripts.simulation.p4_tcp_runtime import tcp_platform_profile
    valid={'maximum_forward_speed_mps':.5,'maximum_reverse_speed_mps':.5,'maximum_spin_rate_radps':1.}
    for bad in ({**valid,'maximum_acceleration_mps2':5.}, {**valid,'maximum_forward_speed_mps':float('nan')}, {'maximum_forward_speed_mps':.5}):
        with pytest.raises(ValueError): tcp_platform_profile(profile(),bad)


def test_both_entries_use_shared_profile_and_preflight_precedes_acquire():
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    main=(root/'scripts/simulation/p4_tcp_runtime.py').read_text(encoding='utf-8')
    check=(root/'scripts/simulation/check_tcp_navigation.py').read_text(encoding='utf-8')
    assert "profile=load_tcp_platform_profile(source_root)" in main
    assert "relay=TcpCommandRelay(profile," in main and "config.write_text(yaml.safe_dump(profile" in main
    assert "profile=load_tcp_platform_profile(SOURCE)" in check
    assert check.index("if options.preflight_only:raise PreflightOnlyComplete()") < check.index("relay.acquire()")


def test_both_tcp_entries_load_same_configured_limits():
    from pathlib import Path
    from scripts.simulation.p4_tcp_runtime import load_tcp_platform_profile
    root=Path(__file__).resolve().parents[1]
    assert profile_command_limits_for_test(load_tcp_platform_profile(root))==(.1,.1,.1)


def test_direct_settings_and_body_only_telemetry():
    args=launch_settings(ROOT,'/cache','/tcp.yaml',dict(command_hold_s=1.2,odometry_timeout_s=4.5,p3_map_timeout_s=0.,mppi_time_steps=140,mppi_compute_envelope_s=.2))
    assert args['control_backend']=='mpcc_cmd_vel'
    assert float(args['command_hold_s'])==1.2
    r,events,clock=relay_fixture()
    r.update_telemetry(S(vx=0.,vy=0.,wz=0.))
    assert r.measured_stopped()
    r.acquire()
    assert r.receive(message(.02))


def test_bounded_direct_entry_has_no_mppi_service_or_wheel_feedback():
    import ast
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    text=(root/'scripts/simulation/check_tcp_navigation.py').read_text(encoding='utf-8')
    tree=ast.parse(text)
    assert not any(isinstance(n,ast.Attribute) and n.attr.startswith(('wheel_speed','steer')) for n in ast.walk(tree))
    assert "params('lunar_pure_wheeled_controller'" not in text
    assert "['control_backend']=='mpcc_cmd_vel'" in text
    assert text.index('if options.preflight_only:raise PreflightOnlyComplete()')<text.index('relay.acquire()')
    relay_tree=ast.parse((root/'scripts/simulation/p4_tcp_runtime.py').read_text(encoding='utf-8'))
    relay=next(n for n in relay_tree.body if isinstance(n,ast.ClassDef) and n.name=='TcpCommandRelay')
    assert not any(isinstance(n,ast.Attribute) and n.attr.startswith(('wheel_speed','steer')) for n in ast.walk(relay))


def test_tcp_hold_contract_is_single_profile_source():
    from pathlib import Path
    import yaml
    profile=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/tcp_p3_feedback.yaml').read_text(encoding='utf-8'))
    assert profile['command_hold_s']==1.2
    assert float(launch_settings(ROOT,'/cache','/tcp.yaml',profile)['command_hold_s'])==1.2


def test_stock_interface_optional_connection_diagnostic():
    r,events,clock=relay_fixture()
    r._connected=None  # 原生接口没有扩展诊断topic。
    r.acquire();r.update_telemetry(S(vx=0.,vy=0.,wz=0.))
    assert r.receive(message(.02)) and r.heartbeat()
    clock[0]=2.
    assert r.receive(message(.02)) and r.heartbeat()  # 已接管不增加遥测龄门禁。
    r.update_connected(S(data=False))
    assert not r.armed and not r.receive(message(.02))
    assert not r.heartbeat()


def test_stock_interface_still_requires_fresh_initial_body_and_command():
    r,events,clock=relay_fixture();r._connected=None
    r.acquire();r.update_telemetry(S(vx=0.,vy=0.,wz=0.));clock[0]=1.01
    assert not r.receive(message(.02))
    r.update_telemetry(S(vx=0.,vy=0.,wz=0.))
    assert r.receive(message(.02))
    clock[0]+=.601
    assert r.heartbeat() and r.armed


def test_zero_before_first_command_does_not_enter_auto_and_initial_telemetry_required():
    r,events,clock=relay_fixture();r.acquire();events.clear()
    assert not r.receive(message())
    assert events==[('velocity',0.,0.)]
    events.clear();assert not r.receive(message(.02))
    assert ('mode','auto') not in events


def test_global_period_uses_shared_tcp_profile_in_both_entries():
    from pathlib import Path
    import yaml
    settings = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/tcp_p3_feedback.yaml').read_text(encoding='utf-8'))
    assert settings['global_period_s'] == 15.
    for prefix in ('/p4_p3_debug', '/bounded'):
        args = launch_settings(ROOT, '/p4/cache', '/p4/wheel.yaml', settings, prefix=prefix)
        assert float(args['global_period_s']) == 15.
    settings['global_period_s'] = 27.
    assert float(launch_settings(ROOT, '/p4/cache', '/p4/wheel.yaml', settings)['global_period_s']) == 27.


def test_tcp_spin_response_delay_from_shared_feedback_config():
    from pathlib import Path
    import yaml
    root=Path(__file__).resolve().parents[1]
    budget=yaml.safe_load((root/'config/tcp_p3_feedback.yaml').read_text(encoding='utf-8'))
    assert budget['command_spin_prepare_s'] == 3.
    assert launch_settings(root,root/'cache',root/'profile.yaml',budget)['command_spin_prepare_s'] == '3.0'


def test_exploration_strategy_is_explicit_and_preserves_settings():
    budgets = dict(command_hold_s=1.2, global_period_s=15.)
    default = launch_settings(ROOT, '/cache', '/tcp.yaml', budgets)
    assert default['exploration_strategy'] == 'frontier'
    disabled = launch_settings(ROOT, '/cache', '/tcp.yaml', budgets, exploration_strategy='none')
    assert disabled == dict(default, exploration_strategy='none')
    for invalid in ('drl', '', None):
        with pytest.raises(ValueError, match='exploration_strategy'):
            launch_settings(ROOT, '/cache', '/tcp.yaml', budgets, exploration_strategy=invalid)


def test_main_exploration_cli_choices_and_forwarding(monkeypatch, capsys):
    import ast
    import inspect
    from scripts.simulation import p4_tcp_runtime as runtime
    tree = ast.parse(inspect.getsource(runtime.main))
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)]
    option = next(n for n in calls if n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == '--exploration-strategy')
    keywords = {k.arg: ast.literal_eval(k.value) for k in option.keywords}
    assert keywords['choices'] == ('none', 'frontier')
    assert keywords['default'] == 'frontier'
    launch = next(n for n in calls if isinstance(n.func, ast.Name) and n.func.id == 'launch_settings')
    strategy = next(k.value for k in launch.keywords if k.arg == 'exploration_strategy')
    assert ast.unparse(strategy) == 'options.exploration_strategy'
    monkeypatch.setattr('sys.argv', ['runtime', '--artifact', '/unused', '--exploration-strategy', 'drl'])
    with pytest.raises(SystemExit) as error:
        runtime.main()
    assert error.value.code == 2
    assert 'invalid choice' in capsys.readouterr().err


def test_topic_contract_keeps_p3_and_private_layers():
    for prefix,namespace in ((None,'/Car/T4'),('/check','/check')):
        args=launch_settings(ROOT,'/cache','/profile',dict(command_hold_s=1.2,mppi_compute_envelope_s=.55),prefix=prefix)
        for key,suffix in dict(goal_topic='/navigation/goal',global_route_topic='/planning/global_route',tracking_status_topic='/control/tracking_status',exploration_map_topic='/mapping/exploration_map',task_topic='/exploration/task').items():
            assert args[key]==namespace+suffix
        assert args['map_topic']=='/Car/T3/mapping/grid_map'
        assert args['odometry_topic']=='/Car/T3/localization/odometry'
        assert args['map_qos_reliability']=='reliable' and args['map_qos_durability']=='transient_local'
        assert float(args['mppi_compute_envelope_s'])==.55


def test_observer_actual_subscriptions_use_injected_contract():
    from scripts.simulation.p4_tcp_runtime import attach_navigation_observer
    topics=[];published=[]
    contract=dict(launch=dict(odometry_topic='/input/pose',goal_topic='/operator/goal'),visualization=dict(start_marker='/view/start',goal_marker='/view/goal',actual_path='/view/path'))
    node=S(create_publisher=lambda cls,topic,qos:(published.append(topic) or S(publish=lambda msg:None)),create_subscription=lambda cls,topic,callback,qos:topics.append(topic))
    worker,handles=attach_navigation_observer(node,node,dict(marker=object,path=object,pose=object,odom=object),dict(display=1,odom=2,goal=3),4.5,contract=contract)
    try:
        assert topics==['/input/pose','/operator/goal']
        assert published==['/view/start','/view/goal','/view/path']
    finally:worker.stop()


def test_runtime_relay_registers_contract_endpoints():
    import ast,inspect
    from scripts.simulation import p4_tcp_runtime as runtime
    tree=ast.parse(inspect.getsource(runtime.main))
    block=next(n.body for n in ast.walk(tree) if isinstance(n,ast.If) and any(isinstance(s,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='vel' for t in s.targets) for s in n.body))
    start=next(i for i,s in enumerate(block) if isinstance(s,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='vel' for t in s.targets))
    end=next(i for i,s in enumerate(block) if isinstance(s,ast.Expr) and isinstance(s.value,ast.Call) and isinstance(s.value.func,ast.Attribute) and s.value.func.attr=='acquire')
    topics=[]
    node=S(create_publisher=lambda cls,topic,qos:(topics.append(('pub',topic)) or S(publish=lambda m:None)),create_subscription=lambda cls,topic,callback,qos:topics.append(('sub',topic)))
    contract=dict(vehicle=dict(command_topic='/native/command',lease_topic='/native/lease',mode_topic='/native/mode',telemetry_topic='/native/body',connected_topic='/native/connection'))
    env=dict(PREFIX=runtime.PREFIX,target=node,contract=contract,vehicle=contract['vehicle'],args=dict(command_topic='/test/control/cmd_vel'),Twist=message,Bool=S,String=S,VehicleTelemetry=S,qos=dict(odom=1,map=2),profile=profile(),handles=[],TcpCommandRelay=runtime.TcpCommandRelay)
    exec(compile(ast.Module(body=block[start:end],type_ignores=[]),'<relay-wiring>','exec'),env)
    assert topics==[('pub','/native/command'),('pub','/native/lease'),('pub','/native/mode'),('sub','/native/body'),('sub','/native/connection'),('sub','/test/control/cmd_vel')]


def test_runtime_rviz_command_uses_rendered_contract(tmp_path):
    import ast,inspect,yaml
    from scripts.simulation import p4_tcp_runtime as runtime
    tree=ast.parse(inspect.getsource(runtime.main))
    block=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and any(isinstance(c,ast.Attribute) and c.attr=='append' and isinstance(c.value,ast.Name) and c.value.id=='commands' for c in ast.walk(n)) and isinstance(n.test,ast.Compare) and isinstance(n.test.left,ast.Attribute) and n.test.left.attr=='rviz_config')
    template=tmp_path/'template.rviz';template.write_text(yaml.safe_dump({'Visualization Manager':{'Tools':[{'Class':'rviz_default_plugins/SetGoal','Topic Key':'goal_topic'}]}}))
    contract={'launch':{'goal_topic':'/operator/navigation/goal','global_route_topic':'/route'},'visualization':{}}
    env=dict(out=tmp_path,options=S(rviz_config=template),yaml=yaml,render_rviz_config=runtime.render_rviz_config,contract=contract,commands=[])
    exec(compile(ast.Module(body=[block],type_ignores=[]),'<rviz-runtime>','exec'),env)
    generated=yaml.safe_load((tmp_path/'p4_navigation.rviz').read_text())
    assert generated['Visualization Manager']['Tools'][0]['Topic']=='/operator/navigation/goal'
    assert env['commands'][0][1][2]==str(tmp_path/'p4_navigation.rviz')


def test_bounded_checker_registers_returned_topics_and_action():
    import ast
    tree=ast.parse((ROOT/'scripts/simulation/check_tcp_navigation.py').read_text(encoding='utf-8'))
    nodes=[]
    for node in tree.body:
        if isinstance(node,ast.For) and isinstance(node.target,ast.Tuple) and [x.id for x in node.target.elts]==['topic','cls','kind']:nodes.append(node)
        elif isinstance(node,ast.Expr) and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Attribute) and node.value.func.attr=='append' and isinstance(node.value.func.value,ast.Name) and node.value.func.value.id=='subs':nodes.append(node)
        elif isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='client' for t in node.targets):nodes.append(node)
    topics=[];actions=[]
    node=S(create_subscription=lambda cls,topic,callback,qos:topics.append(topic))
    args=dict(odometry_topic='/Car/T3/localization/odometry',map_topic='/Car/T3/mapping/grid_map',exploration_map_topic='/check/mapping/exploration_map',diagnostics_topic='/check/control/diagnostics',tracking_status_topic='/check/control/tracking_status',local_path_topic='/check/planning/local_path',global_route_topic='/check/planning/global_route',command_topic='/check/control/cmd_vel',action_name='/check/navigation/navigate_to_pose')
    env=dict(args=args,vehicle=dict(telemetry_topic='/car/telemetry',connected_topic='/car/ue_connected'),control_node=node,subs=[],q=1,map_qos=2,callback=lambda *a:None,command_input=lambda m:None,relay=S(update_connected=lambda m:None),ActionClient=lambda node,cls,topic:actions.append(topic))
    env.update({k:object for k in ('Odometry','GridMap','OccupancyGrid','String','TrackingStatus','RosPath','VehicleTelemetry','Bool','Twist','NavigateToPose')})
    exec(compile(ast.Module(body=nodes,type_ignores=[]),'<bounded-wiring>','exec'),env)
    assert topics==['/Car/T3/localization/odometry','/Car/T3/mapping/grid_map','/check/mapping/exploration_map','/check/control/diagnostics','/check/control/tracking_status','/check/planning/local_path','/check/planning/global_route','/car/telemetry','/car/ue_connected','/check/control/cmd_vel']
    assert actions==['/check/navigation/navigate_to_pose']


def test_preloaded_contract_is_used_without_rereading_and_preserves_namespace(monkeypatch):
    from scripts.simulation import p4_topics
    contract=p4_topics.load_topic_contract(ROOT)
    original=dict(contract['launch'])
    monkeypatch.setattr(p4_topics,'load_topic_contract',lambda *a:pytest.fail('unexpected second config read'))
    args=launch_settings(ROOT,'/cache','/profile',dict(command_hold_s=1.2),prefix='/isolated',contract=contract)
    assert args['command_topic']=='/isolated/control/cmd_vel'
    assert args['goal_topic']=='/isolated/navigation/goal'
    assert contract['launch']==original


def test_runtime_counter_subscription_uses_configured_static_tf_qos():
    import ast,inspect
    from scripts.simulation import p4_tcp_runtime as runtime
    tree=ast.parse(inspect.getsource(runtime.main))
    loop=next(n for n in ast.walk(tree) if isinstance(n,ast.For) and isinstance(n.target,ast.Tuple) and [v.id for v in n.target.elts]==['topic','cls'])
    registered=[]
    env=dict(topics=dict(map_topic='/map',odometry_topic='/odom',tf_topic='/frames/live',tf_static_topic='/frames/static'),GridMap=type('GridMap',(),{}),Odometry=type('Odometry',(),{}),TFMessage=type('TFMessage',(),{}),target=S(create_subscription=lambda cls,topic,cb,qos:registered.append((topic,qos))),source_counts={},handles=[],qos=dict(map='map',odom='odom',tf='dynamic',tf_static='latched'))
    exec(compile(ast.Module(body=[loop],type_ignores=[]),'<counter-qos>','exec'),env)
    assert registered[-1]==('/frames/static','latched')


def test_checker_loads_launch_from_active_overlay(tmp_path):
    import ast
    launch=tmp_path/'launch';launch.mkdir();(launch/'nav1cm_exploration.launch.py').write_text('current overlay')
    tree=ast.parse((ROOT/'scripts/simulation/check_tcp_navigation.py').read_text(encoding='utf-8'))
    node=next(n for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='launch_source' for t in n.targets))
    env=dict(ROOT=tmp_path/'wrong-install',Path=Path,get_package_share_directory=lambda package:str(tmp_path))
    exec(compile(ast.Module(body=[node],type_ignores=[]),'<overlay-launch>','exec'),env)
    assert env['launch_source']=='current overlay'


def test_tcp_soft_clearance_override_reaches_existing_launch_interface():
    import yaml
    profile=yaml.safe_load((ROOT/'config/tcp_p3_feedback.yaml').read_text(encoding='utf-8'))
    profile['global_soft_clearance_m']=.5
    args=launch_settings(ROOT,'/cache','/wheel.yaml',profile)
    assert float(args['global_soft_clearance_m'])==.5
    assert args['platform_config']=='/wheel.yaml'


def test_tcp_uses_sixteen_second_local_horizon_and_final_published_velocity():
    from pathlib import Path
    from scripts.simulation.p4_topics import load_topic_contract
    from scripts.simulation.p4_tcp_runtime import launch_settings
    root=Path(__file__).resolve().parents[1]
    args=launch_settings(root,'/unused/build','/unused/profile.yaml',{'command_hold_s':1.2})
    assert args['horizon_steps']=='80' and float(args['horizon_dt_s'])==.2
    assert args['command_prediction_enabled']=='true'
    assert args['prediction_command_topic']==load_topic_contract(root)['vehicle']['command_topic']
    assert float(args['command_hold_s'])==1.2


@pytest.mark.parametrize('inherited,expected',[(None,'57'),('','57'),('0','0'),('10','10'),('57','57')])
def test_runtime_children_inherit_terminal_domain(monkeypatch,inherited,expected):
    from scripts.simulation.p4_tcp_runtime import runtime_environment
    if inherited is None:monkeypatch.delenv('ROS_DOMAIN_ID',raising=False)
    else:monkeypatch.setenv('ROS_DOMAIN_ID',inherited)
    assert runtime_environment('/home/yanfa/P4','/home/yanfa/P4/run/test')['ROS_DOMAIN_ID']==expected


@pytest.mark.parametrize('domain',[0,10,57])
def test_same_domain_creates_no_bridge_endpoints(domain):
    def forbidden(*args,**kwargs):
        pytest.fail('同域不应创建同话题转发端点')
    source=S(create_subscription=forbidden)
    target=S(create_publisher=forbidden)
    assert input_bridge(source,target,{}, {},source_domain=domain,target_domain=domain)==[]


@pytest.mark.parametrize('inherited,expected',[(None,57),('0',0),('10',10),('57',57)])
def test_main_contexts_and_bridge_use_inherited_domain(monkeypatch,inherited,expected):
    import ast
    from scripts.simulation import p4_tcp_runtime as runtime
    if inherited is None:monkeypatch.delenv('ROS_DOMAIN_ID',raising=False)
    else:monkeypatch.setenv('ROS_DOMAIN_ID',inherited)
    tree=ast.parse(Path(runtime.__file__).read_text(encoding='utf-8'))
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
    start=next(i for i,n in enumerate(main.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='selected_domain' for t in n.targets))
    end=next(i for i,n in enumerate(main.body[start:],start) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='handles' for t in n.targets))
    domains=[];publishers=[];subscriptions=[]
    def node(*args,**kwargs):
        return S(create_publisher=lambda cls,topic,qos:(publishers.append(topic) or S(publish=lambda msg:None)),
                 create_subscription=lambda cls,topic,cb,qos:subscriptions.append(topic))
    scope=dict(vars(runtime),rclpy=S(Context=lambda:object(),init=lambda **kw:domains.append(kw['domain_id']),create_node=node),
               SingleThreadedExecutor=lambda **kw:S(add_node=lambda node:None),
               QoSProfile=lambda **kw:kw,ReliabilityPolicy=S(RELIABLE=1,BEST_EFFORT=2),
               DurabilityPolicy=S(TRANSIENT_LOCAL=1),GridMap=object,Odometry=object,TFMessage=object,
               contract=runtime.load_topic_contract(ROOT))
    exec(compile(ast.Module(body=main.body[start:end+1],type_ignores=[]),'<main-domain-wiring>','exec'),scope)
    assert domains==[57,expected]
    assert len(publishers)==len(subscriptions)==(0 if expected==57 else 4)



def test_runtime_source_uses_selected_build_and_rejects_escape(tmp_path):
    from scripts.simulation.p4_tcp_runtime import runtime_source
    root=tmp_path/'P4';legacy=root/'source';legacy.mkdir(parents=True)
    selected=root/'build/formal-test/source';selected.mkdir(parents=True)
    assert runtime_source(root,{})==legacy
    assert runtime_source(root,{'P4_SOURCE_DIR':str(selected)})==selected
    import pytest
    with pytest.raises(ValueError,match='P4'):
        runtime_source(root,{'P4_SOURCE_DIR':str(tmp_path/'other')})



def test_selected_build_uses_its_own_solver_cache(tmp_path,monkeypatch):
    from scripts.simulation.p4_tcp_runtime import runtime_build_dir,runtime_environment
    root=tmp_path/'P4';cache=root/'build/formal-test/solver-cache'
    monkeypatch.setenv('NAV1CM_BUILD_DIR',str(cache))
    assert runtime_build_dir(root)==cache
    assert runtime_environment(root,root/'run/test')['NAV1CM_BUILD_DIR']==str(cache)
