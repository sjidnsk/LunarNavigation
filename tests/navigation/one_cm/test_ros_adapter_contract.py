"""ROS 外部传输契约；无需 rclpy，真实动力学/厘米地图仍来自 core。"""
from pathlib import Path
import importlib
import json
import sys
import pytest

ROOT = Path(__file__).resolve().parents[3] / 'scripts/navigation/one_cm'
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'ros2/lunar_nav1cm_verification'))


def contract():
    # 显式断言缺失实现，使 RED 指向缺少功能而不是 collection error。
    assert importlib.util.find_spec('lunar_nav1cm_verification') is not None, 'ROS transport contract missing'
    return importlib.import_module('lunar_nav1cm_verification.contract')


def test_sparse_world_keeps_integer_cm_and_native_window():
    c = contract()
    from nav1cm.maps import extract_window
    from nav1cm.contracts import VehicleState
    world = c.world_from_wire({'epoch': 7, 'resolution_m': .01,
                              'blocked': [[90, -10, 100, 10]], 'known_free': [[-300, -300, 400, 300]]})
    grid = extract_window(world, VehicleState(0, 0, 0, 0, 0), False)
    assert grid.cells.shape == (500, 500)
    assert grid.resolution_m == .01 and world.blocked[0].x0 == 90
    assert grid.cell_at_world(.95, 0) == 1
    with pytest.raises(ValueError):
        c.world_from_wire({'epoch': 1, 'resolution_m': .02, 'blocked': [], 'known_free': []})
    with pytest.raises(ValueError):
        c.world_from_wire({'epoch': 1, 'resolution_m': .01, 'blocked': [[1.5, 0, 2, 1]], 'known_free': []})


def test_version_session_namespace_and_monotonic_are_enforced():
    c = contract()
    msg = c.envelope('s1', 'fixture', 'FEEDBACK', {'sample_s': .05}, 15., 999, pid=123)
    decoded = c.decode(c.encode(msg), 's1')
    assert decoded['sent_monotonic_s'] == 15. and decoded['ros_stamp_ns'] == 999
    assert decoded['producer_pid'] == 123
    for mutation in ({'version': 2}, {'version': True}, {'session': 'other'}, {'sent_monotonic_s': float('nan')}):
        with pytest.raises(ValueError):
            c.decode(json.dumps(dict(msg, **mutation)), 's1')
    assert c.topic('plan') == '/nav1cm_verification/plan'
    with pytest.raises(ValueError):
        c.topic('/Car/T5/Car_Cmd_Vel')


def test_plan_roundtrip_keeps_actual_feedback_and_future_target_separate():
    c = contract()
    from nav1cm.contracts import VehicleState, WheelLimits
    from nav1cm.verification_plant import PlantModel, prepare_handover, plan_from_candidate
    from nav1cm.seeds import brake_candidate
    from types import SimpleNamespace
    limits = c.verification_limits()
    assert (limits.linear_speed_mps, limits.angular_speed_rps, limits.linear_accel_mps2) == (.5, .5, .5)
    model = PlantModel(VehicleState(0, 0, 0, 0, 0), limits)
    model.advance(.05); feedback = model.feedback()
    future = prepare_handover(feedback, .06, limits)
    candidate = brake_candidate(future['state'], SimpleNamespace(epoch=1), SimpleNamespace(route_version=1, progress_s=0), limits)
    plan = plan_from_candidate(candidate, 1, feedback['sample_s'], future['prediction_target_s'], feedback=feedback)
    restored = c.plan_from_wire(c.plan_to_wire(plan))
    assert restored.source_feedback_s == .05
    assert restored.valid_start_s == pytest.approx(.2)
    assert restored.source_feedback_id == 1 and restored.source_vector == (0., 0., 0., 0., 0., 0.)
    assert model.accept(restored, .07) == 'ACCEPTED'


def test_late_clock_skips_slots_without_replaying_feedback():
    c = contract()
    assert c.clock_slot(.049, 0) is None
    assert c.clock_slot(.051, 0) == (1, .05, 0)
    assert c.clock_slot(.261, 1) == (5, .25, 3)


def test_pending_only_gates_publication_and_ack_needs_measured_confirmation():
    c = contract()
    gate = c.PublicationGate()
    feedback = {'sequence': 0, 'pending': None}
    assert gate.can_publish(feedback)
    gate.sent(1)
    gate.ack(1, 'ACCEPTED')
    assert not gate.can_publish(feedback)
    assert not gate.can_publish({'sequence': 1, 'pending': {'sequence': 1}})
    assert gate.can_publish({'sequence': 1, 'pending': None})
    gate.sent(2); gate.ack(2, 'LATE_DELIVERY')
    assert gate.can_publish({'sequence': 1, 'pending': None})


def test_feedback_before_ack_cannot_open_publication_gate():
    c = contract()
    gate = c.PublicationGate()
    gate.sent(7)
    feedback = {'sequence': 7, 'pending': None}
    assert not gate.can_publish(feedback)
    gate.ack(6, 'ACCEPTED')
    assert not gate.can_publish(feedback)
    gate.ack(7, 'ACCEPTED')
    assert gate.can_publish(feedback)


def test_freshness_is_retained_but_does_not_stop_fixture():
    c = contract()
    from nav1cm.contracts import VehicleState
    from nav1cm.verification_plant import PlantModel, TimedPlan
    model = PlantModel(VehicleState(0, 0, 0, 0, 0), c.verification_limits())
    actual = model.feedback()
    plan = TimedPlan(1, 0., .3, (0., .05), ((0., 0., 0.),), 1, 1, 'forward', 'brake',
                     (0., 0., 0., 0., 0.), actual['feedback_id'], 0, tuple(actual['vector']))
    assert model.accept(plan, .16) == 'SOURCE_EXPIRED'
    assert c.fatal_protocol_reason(model.protocol_failures) is None
    model.advance(.05)
    assert model.feedback()['sample_s'] == .05
    assert model.protocol_failures[0]['reason'] == 'SOURCE_EXPIRED'
    for reason in ('SOURCE_IDENTITY_MISMATCH', 'PREDICTION_MISMATCH', 'HANDOVER_MISMATCH'):
        assert c.fatal_protocol_reason(model.protocol_failures + [{'reason': reason}]) == reason


def test_probe_deadline_changes_only_on_actual_feedback_origin():
    c = contract()
    assert hasattr(c, 'SessionDeadline'), 'session clock policy missing'
    clock = c.SessionDeadline(100., 10., 40.)
    assert clock.expired(109.9) is None
    assert clock.expired(110.) == 'STARTUP_TIMEOUT'
    clock.observe_origin(102.)
    assert clock.expired(111.) is None
    assert clock.expired(141.9) is None
    assert clock.expired(142.) == 'WALL_TIMEOUT'


def make_callback_node(monkeypatch, tmp_path, role):
    """只替换ROS传输壳；实际fixture callback/PlantModel/协议策略照常执行。"""
    from types import ModuleType, SimpleNamespace
    runtime = importlib.import_module('lunar_nav1cm_verification.runtime')
    clock = {'now': 100.}
    class String:
        def __init__(self, data): self.data = data
    class Node:
        def __init__(self, *args, **kwargs): pass
        def create_publisher(self, *args): return SimpleNamespace(publish=lambda message: None)
        def create_subscription(self, *args): return object()
        def create_timer(self, *args): return SimpleNamespace(cancel=lambda: None)
        def get_clock(self): return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=123))
        def destroy_node(self): pass
    modules = {name: ModuleType(name) for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'std_msgs', 'std_msgs.msg')}
    modules['rclpy.node'].Node = Node
    modules['rclpy.qos'].QoSProfile = lambda **kwargs: SimpleNamespace(**kwargs)
    modules['rclpy.qos'].ReliabilityPolicy = SimpleNamespace(RELIABLE=1)
    modules['rclpy.qos'].DurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL=1, VOLATILE=2)
    modules['std_msgs.msg'].String = String
    for name, module in modules.items(): monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(runtime, 'perf_counter', lambda: clock['now'])
    monkeypatch.setattr(runtime, 'source_identity', lambda args: {})
    args = SimpleNamespace(role=role, session='callback-test', output_dir=str(tmp_path), startup_timeout_s=10., timeout_s=40.)
    return runtime.make_node(args), clock, String


def test_fixture_timer_keeps_freshness_evidence_and_advances_then_stops_fatal(monkeypatch, tmp_path):
    contract()
    node, clock, _ = make_callback_node(monkeypatch, tmp_path, 'fixture')
    try:
        node.ready_roles.update(('navigator', 'probe'))
        node.on_timer()
        node.model.protocol_failures.append({'reason': 'SOURCE_EXPIRED', 'sequence': 1, 'sample_s': 0.})
        clock['now'] = 100.05
        node.on_timer()
        assert not node.done and node.model.sample_s == pytest.approx(.05)
        assert node.model._feedback_id == 2
        assert node.model.protocol_failures[0]['reason'] == 'SOURCE_EXPIRED'
        node.model.protocol_failures.append({'reason': 'HANDOVER_MISMATCH', 'sequence': 2, 'sample_s': .05})
        clock['now'] = 100.1
        node.on_timer()
        assert node.done and node.finish_reason == 'HANDOVER_MISMATCH'
    finally:
        node.close()


@pytest.mark.parametrize('observed_start', [False, True])
def test_probe_callback_uses_actual_clock_not_ready_for_runtime_deadline(monkeypatch, tmp_path, observed_start):
    c = contract()
    node, clock, String = make_callback_node(monkeypatch, tmp_path, 'probe')
    try:
        clock['now'] = 109.
        node.on_timer()  # 自己READY不能替代fixture实际起钟证据。
        if observed_start:
            message = c.envelope('callback-test', 'fixture', 'FEEDBACK', {'origin_monotonic_s': 102.}, 108., 1, pid=11)
            node.receive('feedback', String(c.encode(message)))
        clock['now'] = 111.
        node.on_timer()
        assert node.done is (not observed_start)
        if observed_start:
            clock['now'] = 142.
            node.on_timer()
            assert node.done and node.finish_reason == 'WALL_TIMEOUT'
        else:
            assert node.finish_reason == 'STARTUP_TIMEOUT'
    finally:
        node.close()


def execution_samples():
    return [dict(kind='FEEDBACK', sample_s=0., vector=[0., 0., 0., .2, 0., 0.], segments=[]),
            dict(kind='FEEDBACK', sample_s=.15, vector=[.03, 0., 0., .2, 0., .03], segments=[
                dict(start_s=0., end_s=.05, before=[0., 0., 0., .2, 0., 0.], after=[.01, 0., 0., .2, 0., .01], control=[0., 0., .2]),
                dict(start_s=.05, end_s=.1, before=[.01, 0., 0., .2, 0., .01], after=[.02, 0., 0., .2, 0., .02], control=[0., 0., .2]),
                dict(start_s=.1, end_s=.15, before=[.02, 0., 0., .2, 0., .02], after=[.03, 0., 0., .2, 0., .03], control=[0., 0., .2])])]


@pytest.mark.parametrize('mutation', ['clear', 'drop_middle', 'duplicate', 'broken_vector', 'reversed', 'sample_gap'])
def test_execution_coverage_rejects_missing_duplicate_or_disconnected_segments(mutation):
    contract()
    offline = importlib.import_module('lunar_nav1cm_verification.offline')
    assert hasattr(offline, 'execution_coverage'), 'complete execution coverage missing'
    rows = execution_samples()
    assert offline.execution_coverage(rows)['complete']
    if mutation == 'clear': rows[1]['segments'] = []
    elif mutation == 'drop_middle': rows[1]['segments'].pop(1)
    elif mutation == 'duplicate': rows[1]['segments'].insert(0, rows[1]['segments'][0])
    elif mutation == 'broken_vector': rows[1]['segments'][1]['before'][0] = .011
    elif mutation == 'reversed': rows[1]['segments'].reverse()
    else: rows[1]['sample_s'] = .2
    assert not offline.execution_coverage(rows)['complete']


@pytest.mark.parametrize('mutation', ['controls', 'source_vector', 'missing_field', 'missing_on_both', 'duplicate', 'duplicate_sent', 'future_send', 'reverse_clock'])
def test_probe_complete_message_hash_uniqueness_and_clock_checks(mutation):
    import copy
    c = contract()
    offline = importlib.import_module('lunar_nav1cm_verification.offline')
    streams = {role: [{'direction': 'identity', 'pid': pid}] for role, pid in [('fixture', 11), ('navigator', 22), ('probe', 33)]}
    from nav1cm.verification_plant import TimedPlan
    plan = c.plan_to_wire(TimedPlan(1, 0., .1, (0., .1), ((0., 0., 0.),), 1, 1, 'forward', 'brake',
                                  (0., 0., 0., 0., 0.), 1, 0, (0., 0., 0., 0., 0., 0.)))
    for role, channel, kind, payload, pid in [('fixture', 'feedback', 'FEEDBACK', {'feedback_id': 1}, 11),
          ('navigator', 'plan', 'PLAN', plan, 22), ('fixture', 'ack', 'PLAN_ACK', {'sequence': 1, 'plan': plan}, 11)]:
        msg = c.envelope('s', role, kind, payload, 10., 7, pid=pid)
        streams[role].append(dict(direction='send', channel=channel, message=msg, receiver_pid=pid, received_monotonic_s=10.))
        streams['probe'].append(dict(direction='receive', channel=channel, message=copy.deepcopy(msg), receiver_pid=33, received_monotonic_s=10.01))
    assert offline.transport_evidence(streams)['complete_probe_delivery']
    if mutation == 'controls': streams['probe'][2]['message']['payload']['controls'][0][0] = .3
    elif mutation == 'source_vector': streams['probe'][2]['message']['payload']['source_vector'][0] = .3
    elif mutation == 'missing_field': del streams['probe'][2]['message']['payload']['source_plan_sequence']
    elif mutation == 'missing_on_both':
        for rows in streams.values():
            for row in rows:
                if row.get('channel') == 'plan': row['message']['payload'].pop('source_vector', None)
                elif row.get('channel') == 'ack': row['message']['payload']['plan'].pop('source_vector', None)
    elif mutation == 'duplicate': streams['probe'].append(copy.deepcopy(streams['probe'][2]))
    elif mutation == 'duplicate_sent': streams['navigator'].append(copy.deepcopy(streams['navigator'][1]))
    elif mutation == 'future_send': streams['probe'][2]['message']['sent_monotonic_s'] = 11.
    else: streams['probe'][2]['received_monotonic_s'] = 9.
    assert not offline.transport_evidence(streams)['complete_probe_delivery']


def test_launcher_rejects_non_humble_and_production_domain():
    c = contract()
    with pytest.raises(ValueError): c.validate_environment({'ROS_DISTRO': 'jazzy', 'ROS_DOMAIN_ID': '87', 'ROS_LOCALHOST_ONLY': '1'})
    with pytest.raises(ValueError): c.validate_environment({'ROS_DISTRO': 'humble', 'ROS_DOMAIN_ID': '0', 'ROS_LOCALHOST_ONLY': '1'})
    c.validate_environment({'ROS_DISTRO': 'humble', 'ROS_DOMAIN_ID': '87', 'ROS_LOCALHOST_ONLY': '1'})


def test_empty_raw_evidence_cannot_pass_dds_or_execution_audit():
    contract()
    assert importlib.util.find_spec('lunar_nav1cm_verification.offline') is not None, 'offline auditor missing'
    offline = importlib.import_module('lunar_nav1cm_verification.offline')
    result = offline.transport_evidence({'fixture': [], 'navigator': [], 'probe': []})
    assert result['dds_three_process_pass'] is False
    assert result['complete_probe_delivery'] is False


def test_dds_claim_requires_canonical_pid_and_complete_probe_receipt():
    c = contract()
    offline = importlib.import_module('lunar_nav1cm_verification.offline')
    streams = {role: [{'direction': 'identity', 'pid': pid}] for role, pid in [('fixture', 11), ('navigator', 22), ('probe', 33)]}
    def message(role, kind, payload, pid):
        return c.envelope('s', role, kind, payload, 10., 7, pid=pid)
    from nav1cm.verification_plant import TimedPlan
    plan = c.plan_to_wire(TimedPlan(1, 0., .1, (0., .1), ((0., 0., 0.),), 1, 1, 'forward', 'brake',
                                  (0., 0., 0., 0., 0.), 1, 0, (0., 0., 0., 0., 0., 0.)))
    data = [('fixture', 'feedback', 'FEEDBACK', {'feedback_id': 1}, 11),
            ('navigator', 'plan', 'PLAN', plan, 22),
            ('fixture', 'ack', 'PLAN_ACK', {'sequence': 1, 'plan': plan}, 11)]
    for role, channel, kind, payload, pid in data:
        msg = message(role, kind, payload, pid)
        streams[role].append(dict(direction='send', channel=channel, message=msg, receiver_pid=pid, received_monotonic_s=10.))
        streams['probe'].append(dict(direction='receive', channel=channel, message=msg, receiver_pid=33, received_monotonic_s=10.01))
    for receiver, sender, pid, receiver_pid in [('fixture', 'navigator', 22, 11), ('fixture', 'probe', 33, 11), ('navigator', 'fixture', 11, 22)]:
        streams[receiver].append(dict(direction='receive', channel='diagnostic', message=message(sender, 'READY', {}, pid), receiver_pid=receiver_pid, received_monotonic_s=10.01))
    assert offline.transport_evidence(streams)['dds_three_process_pass']
    assert offline.transport_evidence(streams)['complete_probe_delivery']
    streams['probe'][1]['message']['producer_pid'] = 999
    assert not offline.transport_evidence(streams)['dds_three_process_pass']


@pytest.mark.parametrize('payload', [{'x': float('nan')}, {'a': [float('inf')]}])
def test_nested_nonfinite_payload_cannot_enter_transport(payload):
    c = contract()
    with pytest.raises(ValueError):
        c.envelope('s', 'fixture', 'WORLD', payload, 1., 1)
