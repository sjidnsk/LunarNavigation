"""短路径判据：手工时间线验证新物理样本、滑移与断流不能假成功。"""
import copy
import pytest


def good_run():
    events = [dict(event='path', wall=0., poses=[[0., 0., 0.], [.8, 0., .35]]),
              dict(event='graph', wall=0., command_publishers=['lunar_pure_wheeled_controller'], command_subscribers=['viper_engineering_bridge']),
              dict(event='cmd', wall=.1, values=[.2, 0., 0., 0., 0., .1])]
    events.append(dict(event='physics', time=-.05, state_publish_done_wall=.2,
                       root_pose_xyzw=[.01, 0., 0., 0., 0., 0., 1.], root_world_velocity=[.1,0.,0.,0.,0.,.1],
                       dof_positions=[.1]*8, dof_velocities=[1.]*8, targets={'reason':'tracking'}, rx_sequence=1))
    for i in range(31):
        wall = 1. + i * .05
        events.extend([dict(event='tf', wall=wall, stamp=i*.05),
                       dict(event='clock', wall=wall, stamp=i*.05),
                       dict(event='odom', wall=wall, stamp=i*.05, pose=[.75, 0., .30], velocity=[0., 0., 0.]),
                       dict(event='physics', time=i*.05, state_publish_done_wall=wall,
                            root_pose_xyzw=[.75, 0., 0., 0., 0., .14943813247359922, .9887710779360422],
                            root_world_velocity=[0.]*6, dof_positions=[0.]*8, dof_velocities=[0.]*8,
                            targets={'reason':'tracking'}, rx_sequence=i+1),
                       dict(event='cmd', wall=wall+.001, values=[0.]*6)])
    events.append(dict(event='render_timing', time=.5, render_start_wall=1.4, render_done_wall=1.45))
    return events


def assess(events):
    from tools.isaac_viper.verify_ros import evaluate
    return evaluate(events)


def test_real_motion_goal_and_fresh_stopped_tail_pass():
    result = assess(good_run())
    assert result['passed']
    assert result['final_position_error_m'] == pytest.approx(.05)
    assert result['final_yaw_error_rad'] == pytest.approx(.05)
    assert result['stop_window']['distinct_physics_samples'] == 31


@pytest.mark.parametrize('event', ['odom', 'physics', 'clock'])
def test_repeated_cached_sample_cannot_accumulate_stop_time(event):
    events = good_run()
    for e in events:
        if e['event'] == event:
            e['time' if event == 'physics' else 'stamp'] = 0.
    assert not assess(events)['passed']


def test_sliding_out_after_earlier_good_window_fails():
    events = good_run()
    for e in events[-5:]:
        if e['event'] == 'odom':
            e['pose'][0] = 1.1
        if e['event'] == 'physics':
            e['root_pose_xyzw'][0] = 1.1
    assert not assess(events)['passed']


@pytest.mark.parametrize('mutation', ['empty_path', 'missing_tf', 'cmd_gap', 'moving_root', 'moving_joint', 'timeout', 'second_publisher', 'no_turn', 'nan'])
def test_invalid_completion_evidence_fails_closed(mutation):
    events = copy.deepcopy(good_run())
    if mutation == 'empty_path':
        events.append(dict(event='path', wall=3., poses=[]))
    elif mutation == 'missing_tf':
        events = [e for e in events if e['event'] != 'tf']
    elif mutation == 'cmd_gap':
        events = [e for e in events if not (e['event'] == 'cmd' and 1.3 < e['wall'] < 2.4)]
    elif mutation in ('moving_root', 'moving_joint', 'timeout'):
        for e in events:
            if e['event'] == 'physics':
                if mutation == 'moving_root': e['root_world_velocity'][0] = .02
                if mutation == 'moving_joint': e['dof_velocities'][0] = .5
                if mutation == 'timeout': e['targets']['reason'] = 'command_timeout'
    elif mutation == 'second_publisher':
        events[1]['command_publishers'].append('probe')
    elif mutation == 'no_turn':
        events[2]['values'][5] = 0.
    elif mutation == 'nan':
        next(e for e in events if e['event'] == 'odom')['pose'][0] = float('nan')
    assert not assess(events)['passed']


@pytest.mark.parametrize('mutation', ['missing_render', 'root_no_motion', 'stamp_mismatch', 'command_stream_ended', 'stale_odom'])
def test_independent_physics_and_live_window_are_required(mutation):
    events = good_run()
    if mutation == 'missing_render':
        events = [e for e in events if e['event'] != 'render_timing']
    elif mutation == 'root_no_motion':
        for e in events:
            if e['event'] == 'physics': e['root_pose_xyzw'][0] = 0.
    elif mutation == 'stamp_mismatch':
        for e in events:
            if e['event'] == 'physics': e['time'] += 100.
    elif mutation == 'command_stream_ended':
        events = [e for e in events if e['event'] != 'cmd' or e['wall'] < 1.8]
    elif mutation == 'stale_odom':
        for e in events:
            if e['event'] == 'cmd': e['wall'] += 11.
    assert not assess(events)['passed']


def test_no_measured_joint_response_cannot_be_motion_evidence():
    events = good_run()
    for e in events:
        if e['event'] == 'physics': e['dof_velocities'] = [0.]*8
    assert not assess(events)['passed']


@pytest.mark.parametrize('event', ['rejected_twist', 'twist'])
def test_real_receipt_schema_without_wall_detects_rejected_input(event):
    events = good_run()
    events.append(dict(event=event, rx_wall=.3, rx_sequence=2, components=[0.]*6, accepted=False))
    result = assess(events)
    assert not result['passed']
    assert 'runtime_rejected_twist' in result['reasons']


def test_feedback_gap_over_run_limit_is_failure_even_with_fresh_final_tail():
    events = good_run()
    for e in events:
        if e.get('wall',0.) >= 1.: e['wall'] += 11.
        if e['event'] == 'physics' and e['state_publish_done_wall'] >= 1.:
            e['state_publish_done_wall'] += 11.
        if e['event'] == 'render_timing':
            e['render_start_wall'] += 11.
            e['render_done_wall'] += 11.
    events.append(dict(event='odom', wall=.2, stamp=-.05, pose=[.01,0.,0.], velocity=[.1,0.,.1]))
    result = assess(events)
    assert not result['passed']
    assert 'feedback_gap_exceeded' in result['reasons']


def test_fixture_headers_use_observed_sim_clock_and_do_not_advance_while_render_frozen():
    from tools.isaac_viper.verify_ros import sampled_stamp
    assert [sampled_stamp(t) for t in [112.2,112.2,112.21]] == [(112,200000000),(112,200000000),(112,210000000)]
    assert sampled_stamp(149.9999999996) == (150,0)


@pytest.mark.parametrize('clock', [float('nan'),float('inf'),-1.,True,None])
def test_fixture_headers_reject_missing_or_invalid_clock(clock):
    from tools.isaac_viper.verify_ros import sampled_stamp
    with pytest.raises(ValueError): sampled_stamp(clock)


def cached_prefix_run(delay, repeats):
    events = good_run()
    for e in events:
        if e.get('wall',0.) >= 1.: e['wall'] += delay
        if e['event'] == 'physics' and e['state_publish_done_wall'] >= 1.:
            e['state_publish_done_wall'] += delay
        if e['event'] == 'render_timing':
            e['render_start_wall'] += delay
            e['render_done_wall'] += delay
    for i in range(repeats):
        events.append(dict(event='odom',wall=.2+i*.05,stamp=-.05,pose=[.01,0.,0.],velocity=[.1,0.,.1]))
    return events


def test_repeated_old_stamp_cannot_hide_long_new_sample_gap_after_recovery():
    result = assess(cached_prefix_run(11.,237))
    assert not result['passed']
    assert 'feedback_gap_exceeded' in result['reasons']
    assert result['max_feedback_gap_wall_s'] == pytest.approx(11.8)


def test_short_duplicate_stamp_burst_does_not_reject_healthy_run():
    result = assess(cached_prefix_run(0.,16))
    assert result['passed']
    assert result['max_feedback_gap_wall_s'] == pytest.approx(.8)


def test_stamp_reversal_before_final_good_tail_still_fails():
    events = cached_prefix_run(0.,1)
    events.append(dict(event='odom',wall=.25,stamp=-.06,pose=[.01,0.,0.],velocity=[.1,0.,.1]))
    result = assess(events)
    assert not result['passed']
    assert any('odom_stamp_reversed' in reason for reason in result['reasons'])


@pytest.mark.parametrize('boundary', ['start','end'])
def test_new_sample_gap_includes_active_run_boundaries(boundary):
    if boundary == 'start':
        events = cached_prefix_run(11.,0)
        expected = 12.
    else:
        events = good_run()
        events.append(dict(event='odom',wall=13.,stamp=1.5,pose=[.75,0.,.30],velocity=[0.,0.,0.]))
        expected = 10.5
    result = assess(events)
    assert not result['passed']
    assert 'feedback_gap_exceeded' in result['reasons']
    assert result['max_feedback_gap_wall_s'] == pytest.approx(expected)


def test_identity_tf_keeps_raw_components_in_healthy_event():
    from tools.isaac_viper.verify_ros import identity_tf_event
    event = identity_tf_event([0.,0.,0.,0.,0.,0.,1.],2.,1.5)
    assert event['event'] == 'tf'
    assert event['transform_xyz_xyzw'] == [0.,0.,0.,0.,0.,0.,1.]
    assert (event['wall'],event['stamp']) == (2.,1.5)


@pytest.mark.parametrize('component',range(7))
@pytest.mark.parametrize('bad',[float('nan'),float('inf'),-float('inf')])
def test_nonfinite_tf_component_is_serializable_error_and_cannot_refresh_freshness(component,bad):
    import json
    from tools.isaac_viper.verify_ros import identity_tf_event
    values = [0.,0.,0.,0.,0.,0.,1.]
    values[component] = bad
    event = identity_tf_event(values,2.,1.5)
    assert event['event'] == 'error'
    assert event['transform_xyz_xyzw'][component] == repr(bad)
    assert 'nonfinite' in event['message']
    json.dumps(event,allow_nan=False)
    result = assess(good_run()+[event])
    assert not result['passed']
    assert 'recorded_bench_error' in result['reasons']


@pytest.mark.parametrize('component',range(7))
def test_finite_nonidentity_tf_is_rejected_and_raw_values_preserved(component):
    from tools.isaac_viper.verify_ros import identity_tf_event
    values = [0.,0.,0.,0.,0.,0.,1.]
    values[component] += .01
    event = identity_tf_event(values,2.,1.5)
    assert event['event'] == 'error'
    assert event['transform_xyz_xyzw'] == values


def test_offline_analysis_rejects_nonfinite_raw_values_in_healthy_labelled_tf():
    events = good_run()
    event = next(e for e in events if e['event'] == 'tf')
    event['transform_xyz_xyzw'] = [0.,0.,0.,0.,0.,0.,float('nan')]
    assert not assess(events)['passed']
