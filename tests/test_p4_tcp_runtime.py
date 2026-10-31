"""P4域10接线与车辆租约回归；不依赖ROS。"""
from types import SimpleNamespace as S
import pytest
from scripts.simulation.p4_tcp_runtime import CONTROL_DOMAIN, P3_SOURCE_DOMAIN, input_bridge, launch_settings, command_ready


def test_control_launch_uses_existing_wheel_limits_and_private_adapter_input():
    assert (CONTROL_DOMAIN,P3_SOURCE_DOMAIN)==(10,57)
    args=launch_settings('/p4/source','/p4/cache','/p4/test-wheel.yaml',dict(odometry_timeout_s=4.5,p3_map_timeout_s=6.,mppi_time_steps=140,mppi_compute_envelope_s=.2))
    assert args['command_topic']=='/p4_p3_debug/cmd_vel'
    assert args['goal_topic']=='/goal_pose'
    assert args['enable_controller']=='true' and args['exploration_strategy']=='none'
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
    handles=input_bridge(source,target,classes,dict(map='latched-map',odom='odom',tf='tf',tf_static='latched-static'))
    assert len(subscriptions)==4 and len(handles)==8
    for topic,callback,qos in subscriptions:
        msg=S(header=S(stamp=123));callback(msg)
        assert published[-1]==(topic,msg,qos)
    assert subscriptions[-1][0]=='/tf_static' and subscriptions[-1][2]=='latched-static'


@pytest.mark.parametrize('command_age,telemetry_age,expected',[
    (.1,.1,True),(.55,.1,True),(.6,.1,True),(.601,.1,False),(.1,.51,False),(-.1,.1,False)])
def test_relay_rejects_transport_gaps(command_age,telemetry_age,expected):
    assert command_ready(command_age,telemetry_age)==expected


def test_tf_output_accepts_reliable_rviz_without_changing_source_subscription():
    subscriptions=[];publishers=[]
    source=S(create_subscription=lambda cls,topic,callback,qos:subscriptions.append((topic,qos)))
    target=S(create_publisher=lambda cls,topic,qos:(publishers.append((topic,qos)) or S(publish=lambda msg:None)))
    input_bridge(source,target,dict(map=object,odom=object,tf=object),dict(map=1,odom=2,tf='best_effort',tf_output='reliable',tf_static=4))
    assert ('/tf','best_effort') in subscriptions and ('/tf','reliable') in publishers


def test_p4_profile_has_no_map_time_expiry_and_preflight_uses_same_policy():
    from pathlib import Path
    import yaml
    from scripts.simulation.p3_input_timing import record_sample,runtime_input_health
    profile=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/tcp_p3_feedback.yaml').read_text(encoding='utf-8'))
    assert profile['p3_map_timeout_s']==0.
    args=launch_settings('/p4/source','/p4/cache','/p4/wheel.yaml',profile)
    assert float(args['p3_map_timeout_s'])==0.
    sample=record_sample(None,95_000_000_000,100_000_000_000,10.)
    assert runtime_input_health('map',sample,1100_000_000_000,1010.,odometry_timeout_s=4.5,p3_map_timeout_s=profile['p3_map_timeout_s'])['ready']



def profile():
    return {'capability':{'maximum_forward_speed_mps':.2,'maximum_reverse_speed_mps':.2,'maximum_spin_rate_radps':1.}}


def message(v=0.,w=0.):return S(linear=S(x=v),angular=S(z=w))


def telemetry(**changes):
    values=dict(vx=0.,vy=0.,wheel_speed_fl=0.,wheel_speed_fr=0.,wheel_speed_rl=0.,wheel_speed_rr=0.)
    values.update(changes);return S(**values)


def relay_fixture():
    from scripts.simulation import p4_tcp_runtime as m
    events=[];clock=[0.]
    relay=m.TcpCommandRelay(m.low_speed_profile(profile()),
        publish_velocity=lambda msg:events.append(('velocity',msg.linear.x,msg.angular.z)),
        publish_mode=lambda mode:events.append(('mode',mode)),
        publish_lease=lambda active:events.append(('lease',active)),
        make_zero=message,clock=lambda:clock[0])
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
    budgets=dict(odometry_timeout_s=4.5,p3_map_timeout_s=0.,mppi_time_steps=140,mppi_compute_envelope_s=.2)
    args=launch_settings('/p4/source','/p4/cache','/p4/wheel',budgets,prefix='/check',goal_topic='/check/goal')
    assert args['command_topic']=='/check/cmd_vel' and args['action_name']=='/check/navigate_to_pose'
    assert args['initial_travel_direction']=='forward'
    assert float(args['p3_map_timeout_s'])==0. and args['goal_topic']=='/check/goal'


def test_relay_preflight_publishes_nothing_and_execution_needs_no_task_status():
    r,events,clock=relay_fixture();r.update_telemetry(telemetry())
    assert not r.receive(message(.02)) and not events
    r.acquire();events.clear()
    assert r.receive(message(.02))
    assert events==[('mode','auto'),('velocity',.02,0.)]
    assert r.receive(message(.03)) and r.forwarded_nonzero==2
    assert events.count(('mode','auto'))==1


@pytest.mark.parametrize('bad',[message(.11),message(-.11),message(0.,.3),message(float('nan')),message(0.,float('inf'))])
def test_invalid_command_parks_and_is_never_forwarded(bad):
    r,events,clock=relay_fixture();r.acquire();r.update_telemetry(telemetry());events.clear()
    with pytest.raises(ValueError):r.receive(bad)
    assert events==[('velocity',0.,0.),('mode','park')]
    assert r.forwarded_nonzero==0


@pytest.mark.parametrize('kind',['command','telemetry'])
def test_transport_gap_parks_using_the_same_existing_budget(kind):
    r,events,clock=relay_fixture();r.acquire();r.update_telemetry(telemetry());r.receive(message(.02))
    clock[0]=.61 if kind=='command' else .51
    if kind=='command':r.update_telemetry(telemetry())
    events.clear();assert not r.heartbeat()
    assert ('velocity',0.,0.) in events and ('mode','park') in events
    assert not r.armed


def test_zero_command_and_cleanup_cannot_reenter_auto():
    r,events,clock=relay_fixture();r.acquire();r.update_telemetry(telemetry());r.receive(message(.02))
    assert not r.receive(message()) and events[-1]==('velocity',0.,0.)
    r.begin_stop();before=r.forwarded_nonzero;events.clear()
    assert not r.receive(message(.02));r.heartbeat()
    assert r.forwarded_nonzero==before and ('mode','auto') not in events
    r.release();events.clear();assert not r.receive(message(.02)) and not events


@pytest.mark.parametrize('changes,age,stopped',[({},0.,True),({},.5,False),({},.51,False),({'vy':.02},0.,False),({'wheel_speed_rr':.06},0.,False)])
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



@pytest.mark.parametrize('changes',[{'vy':float('nan')},{'wheel_speed_rr':float('nan')}])
def test_invalid_telemetry_is_not_measured_stop(changes):
    r,events,clock=relay_fixture();r.update_telemetry(telemetry(**changes))
    assert not r.measured_stopped()


def test_reverse_limit_and_environment_come_from_shared_configuration():
    from scripts.simulation.p4_tcp_runtime import TcpCommandRelay,low_speed_profile,runtime_environment
    source=profile();source['capability']['maximum_reverse_speed_mps']=.03
    events=[]
    r=TcpCommandRelay(low_speed_profile(source),publish_velocity=lambda m:events.append(m),
        publish_mode=lambda m:None,publish_lease=lambda b:None,make_zero=message,clock=lambda:0.)
    r.acquire();r.update_telemetry(telemetry())
    assert r.receive(message(-.02))
    with pytest.raises(ValueError):r.receive(message(-.04))
    env=runtime_environment('/home/yanfa/P4','/home/yanfa/P4/run/test')
    assert env['ROS_DOMAIN_ID']=='10' and env['ROS_LOG_DIR'].replace(chr(92),'/').endswith('test/ros-log')
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


def test_display_mailbox_coalesces_without_blocking_publish_or_old_path_after_goal():
    from scripts.simulation.p4_tcp_runtime import DisplayWorker
    import threading
    entered=threading.Event();release=threading.Event();events=[]
    class Observer:
        def odometry(self,msg,when):
            if msg=='blocked': entered.set();release.wait(2)
            events.append(('path',msg))
        def goal(self,msg,when): events.append(('goal',msg))
    worker=DisplayWorker(Observer());worker.start();worker.offer_odom('blocked',0.)
    assert entered.wait(1)
    worker.offer_odom('old',1.);worker.offer_goal('old goal',2.);worker.offer_goal('new goal',3.);worker.offer_odom('new',4.)
    release.set();worker.stop()
    assert ('goal','old goal') not in events and ('path','old') not in events
    assert events[-2:]==[('goal','new goal'),('path','new')]


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
    from scripts.simulation.p4_tcp_runtime import COMMAND_TIMEOUT_S,TRANSPORT_TIMEOUT_S
    config=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/tcp_p3_feedback.yaml').read_text(encoding='utf-8'))
    assert config['mppi_compute_envelope_s']==.55 and config['mppi_time_steps']==140
    assert config['p3_map_timeout_s']==0. and config['odometry_timeout_s']==4.5
    assert COMMAND_TIMEOUT_S==.6 and TRANSPORT_TIMEOUT_S==.5
    assert (3+11+1)*.05 >= COMMAND_TIMEOUT_S+.1


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
