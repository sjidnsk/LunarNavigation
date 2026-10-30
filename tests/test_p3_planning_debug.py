"""无控制联调入口必须固定隔离输出与真实P3输入。"""
from scripts.simulation.debug_p3_planning import launch_arguments, goal_candidates


def test_readonly_composition_cannot_enable_controller_or_vehicle_output():
    args=launch_arguments('/p4/source','/p4/cache')
    assert args['enable_controller']=='false'
    assert args['exploration_strategy']=='frontier'
    assert args['map_topic']=='/Car/T3/mapping/grid_map'
    assert args['odometry_topic']=='/Car/T3/localization/odometry'
    assert args['map_qos_durability']=='transient_local'
    assert args['command_topic']=='/p4_p3_debug/disabled_cmd_vel'
    for key in ('action_name','local_path_topic','global_route_topic','task_topic','status_topic'):
        assert args[key].startswith('/p4_p3_debug/')


def test_goal_candidates_use_only_known_free_cells_and_never_current_pose():
    from types import SimpleNamespace as S
    grid=S(info=S(width=4,height=1,resolution=1.,origin=S(position=S(x=0.,y=0.))),data=[0,-1,100,0])
    assert goal_candidates(grid,.5,.5)==[(3.5,.5)]


def test_pending_goal_is_resolved_cancelled_and_terminal_before_cleanup_success():
    from types import SimpleNamespace as S
    from scripts.simulation.debug_p3_planning import cleanup_goal
    clock=[0.];events=[]
    class Future:
        def __init__(self,ready,value):self.ready=ready;self.value=value
        def done(self):return clock[0]>=self.ready
        def result(self):return self.value
    terminal=Future(.3,S(status=5))
    handle=S(accepted=True,goal_id=S(uuid=[1]*16),get_result_async=lambda:terminal,
        cancel_goal_async=lambda:(events.append('cancel') or Future(.2,S(goals_canceling=[S(goal_id=S(uuid=[1]*16))]))))
    pending=Future(.1,handle)
    result=cleanup_goal(pending,None,lambda:clock.__setitem__(0,clock[0]+.05),timeout_s=1.,now=lambda:clock[0])
    assert result['ok'] and result['terminal'] and result['cancel_accepted'] and events==['cancel']
    assert clock[0]>=.3


def test_unresolved_goal_cannot_be_reported_cancelled():
    from scripts.simulation.debug_p3_planning import cleanup_goal
    from types import SimpleNamespace as S
    clock=[0.];pending=S(done=lambda:False)
    result=cleanup_goal(pending,None,lambda:clock.__setitem__(0,clock[0]+.05),timeout_s=.2,now=lambda:clock[0])
    assert not result['ok'] and not result['terminal']


def test_stay_failure_refreshes_final_health_and_never_retains_probe_success():
    from scripts.simulation.debug_p3_planning import observation_report
    result=observation_report({'ok':True,'initial_probe_ok':True},
        {'odom':{'ready':False,'reason':'SOURCE_FROZEN'}},tf=(0,0,0,0,0,0,1),tf_changed=False,stay_fault='SOURCE_FROZEN')
    assert not result['ok'] and result['initial_probe_ok']
    assert result['input_health']['odom']['reason']=='SOURCE_FROZEN'
    assert result['stay_fault']=='SOURCE_FROZEN'


def test_goal_already_terminal_can_arrive_after_cancel_response():
    from types import SimpleNamespace as S
    from scripts.simulation.debug_p3_planning import cleanup_goal
    clock=[0.]
    class Future:
        def __init__(self,ready,value):self.ready=ready;self.value=value
        def done(self):return clock[0]>=self.ready
        def result(self):return self.value
    handle=S(accepted=True,goal_id=S(uuid=[1]*16),get_result_async=lambda:Future(.3,S(status=6)),
        cancel_goal_async=lambda:Future(.1,S(goals_canceling=[])))
    result=cleanup_goal(None,handle,lambda:clock.__setitem__(0,clock[0]+.05),timeout_s=1.,now=lambda:clock[0])
    assert result['ok'] and result['terminal'] and not result['cancel_accepted']


def test_formal_plan_does_not_reuse_old_cancelled_request_evidence(tmp_path):
    import json
    from scripts.simulation.debug_p3_planning import formal_plan
    path=tmp_path/'launch.log'
    record={'planning_outcome':0,'reason_code':'PLAN_FOUND','has_reference':True,'requested_goal':[1.,2.],'accepted_generation':1,'current_accepted_generation':1}
    path.write_text('GLOBAL_RESULT: '+json.dumps(record)+'\n')
    offset=path.stat().st_size
    assert formal_plan(path,(1.,2.),after_offset=offset) is None
    with path.open('a') as stream:stream.write('GLOBAL_RESULT: '+json.dumps(dict(record,accepted_generation=2,current_accepted_generation=2))+'\n')
    assert formal_plan(path,(1.,2.),after_offset=offset)['accepted_generation']==2


def test_unknown_action_status_is_not_terminal_cleanup_proof():
    from types import SimpleNamespace as S
    from scripts.simulation.debug_p3_planning import cleanup_goal
    done=S(done=lambda:True,result=lambda:S(status=0))
    handle=S(accepted=True,get_result_async=lambda:done)
    result=cleanup_goal(None,handle,lambda:None)
    assert not result['ok'] and not result['terminal']
