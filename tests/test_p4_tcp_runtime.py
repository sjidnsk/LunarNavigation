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
    (.1,.1,True),(.51,.1,False),(.1,.51,False),(-.1,.1,False)])
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
