"""三个独立 rclpy 进程；回调逐条落盘，不在内存保留轨迹/事件历史。"""
from nav1cm.verification_evidence import tick_global_evidence
import argparse
from dataclasses import asdict
import hashlib
import math
import os
from pathlib import Path
import signal
from time import perf_counter

from .contract import (CHANNELS, PublicationGate, SessionDeadline, clock_slot, decode, encode, envelope, fatal_protocol_reason,
                       plan_from_wire, plan_to_wire, topic, validate_environment,
                       verification_limits, world_from_wire, world_to_wire)


def parser(*, require_session=True):
    p = argparse.ArgumentParser()
    p.add_argument('--role', choices=('fixture', 'navigator', 'probe'))
    p.add_argument('--session', required=require_session)
    p.add_argument('--output-dir', required=True)
    p.add_argument('--build-dir', required=True)
    p.add_argument('--geometry-lib', required=True)
    p.add_argument('--distance-field-lib', required=True)
    p.add_argument('--timeout-s', type=float, default=40.)
    p.add_argument('--startup-timeout-s', type=float, default=180.)
    p.add_argument('--delay-ms', type=float, default=250.)
    return p


def source_identity(args):
    import nav1cm
    core_dir = Path(nav1cm.__file__).resolve().parent
    source_paths = list(core_dir.glob('*.py')) + list(Path(__file__).parent.glob('*.py')) + [core_dir.parent / 'benchmark_async_navigation.py']
    return dict(source_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths},
                native_sha256={str(Path(p).resolve()): hashlib.sha256(Path(p).read_bytes()).hexdigest()
                               for p in (args.geometry_lib, args.distance_field_lib)},
                limits=asdict(verification_limits()), planner_options=dict(n=20, dt=.1),
                delay_ms=args.delay_ms, domain_id=os.environ['ROS_DOMAIN_ID'])


def make_node(args):
    # 延迟导入让纯转换测试和离线审计无需安装 ROS。
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String

    class VerificationNode(Node):
        def __init__(self):
            super().__init__('nav1cm_' + args.role, namespace='/nav1cm_verification')
            self.args = args
            self.done = False
            self.exit_code = 0
            self.finish_reason = None
            self.started_wall = perf_counter()
            self.deadline = SessionDeadline(self.started_wall, args.startup_timeout_s, args.timeout_s)
            self.origin = None
            self.stream = (Path(args.output_dir) / (args.role + '.jsonl')).open('x', encoding='utf-8')
            self.publishers_by_channel = {}
            self.subscriptions_by_channel = []
            self.ready_roles = set()
            self.feedback = self.world = self.goal = None
            self.last_target = -1.
            self.sequence = 0
            self.slot = 0
            self.gate = PublicationGate()
            self.core = self.worker = self.model = None
            self.cold = None
            self.target_rejected = False
            self.last_ready = -1.
            self.received_count = 0
            self.sent_count = 0
            self.identity = source_identity(args)
            self.write('identity', identity=self.identity, pid=os.getpid(), role=args.role)
            for channel in CHANNELS:
                qos = QoSProfile(depth=1 if channel in ('world', 'goal') else 50,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL if channel in ('world', 'goal') else DurabilityPolicy.VOLATILE)
                self.publishers_by_channel[channel] = self.create_publisher(String, topic(channel), qos)
                # probe 收全量；导航/fixture只订阅实际需要的话题，避免自身日志复制。
                interests = {'fixture': ('plan', 'diagnostic'),
                             'navigator': ('world', 'goal', 'feedback', 'ack'),
                             'probe': CHANNELS}
                if channel in interests[args.role]:
                    self.subscriptions_by_channel.append(self.create_subscription(
                        String, topic(channel), lambda msg, ch=channel: self.receive(ch, msg), qos))
            if args.role == 'fixture':
                from nav1cm.fixtures import scenarios
                from nav1cm.verification_plant import PlantModel
                self.scenario = next(s for s in scenarios() if s.name == 'straight')
                self.model = PlantModel(self.scenario.start, verification_limits())
            elif args.role == 'navigator':
                from nav1cm.local_worker import LocalWorker
                from nav1cm.field_native import NativeDistanceField
                from nav1cm.geometry_native import NativeGeometry
                from nav1cm.async_coordinator import AsyncNavigationCoordinator
                from nav1cm.verification_plant import WorkerDelay
                field = NativeDistanceField(args.distance_field_lib)
                backend = NativeGeometry(args.geometry_lib)
                self.worker = LocalWorker(verification_limits(), args.build_dir,
                                          planner_options=dict(n=20, dt=.1), field_backend=field,
                                          request_wrapper=WorkerDelay(args.delay_ms / 1000.))
                try:
                    self.core = AsyncNavigationCoordinator(local_worker=self.worker, limits=verification_limits(),
                                                           backend=backend, field_backend=field,
                                                           planning_period_s=.1, execution_period_s=.05,
                                                           allow_unknown=False, clearance_radius_m=.9)
                except Exception:
                    self.worker.close()
                    raise
            self.timer = self.create_timer(.05, self.on_timer)

        def write(self, direction, **row):
            self.stream.write(encode(dict(direction=direction, receiver_pid=os.getpid(),
                                          received_monotonic_s=perf_counter(), **row)) + '\n')
            self.stream.flush()

        def send(self, channel, kind, payload):
            msg = envelope(args.session, args.role, kind, payload, perf_counter(), self.get_clock().now().nanoseconds)
            self.write('send', channel=channel, message=msg)
            self.publishers_by_channel[channel].publish(String(data=encode(msg)))
            self.sent_count += 1

        def receive(self, channel, raw):
            # 异会话只记录丢弃，不允许进入本会话模型。
            try:
                msg = decode(raw.data, args.session)
            except (ValueError, TypeError, KeyError) as exc:
                self.write('rejected_wire', channel=channel, reason=str(exc), raw=raw.data)
                return
            self.write('receive', channel=channel, message=msg)
            self.received_count += 1
            payload, kind = msg['payload'], msg['kind']
            allowed = {'world': 'fixture', 'goal': 'fixture', 'feedback': 'fixture', 'ack': 'fixture', 'plan': 'navigator'}
            if channel in allowed and msg['producer'] != allowed[channel]:
                self.fail('PRODUCER_MISMATCH')
                return
            if args.role == 'probe':
                if channel == 'feedback':
                    self.deadline.observe_origin(payload['origin_monotonic_s'])
                    self.origin = payload['origin_monotonic_s']
                return
            if args.role == 'fixture':
                if kind == 'READY':
                    self.ready_roles.add(msg['producer'])
                elif kind == 'TARGET_REJECTED':
                    self.model.reject_target()
                    self.target_rejected = True
                elif channel == 'plan':
                    if self.origin is None:
                        self.fail('PLAN_BEFORE_CLOCK')
                        return
                    plan = plan_from_wire(payload)
                    received = perf_counter()
                    reason = self.model.accept(plan, received - self.origin)
                    self.send('ack', 'PLAN_ACK', dict(sequence=plan.sequence, reason=reason,
                                                     received_wall_s=received, plan=payload))
                    if reason in ('SOURCE_IDENTITY_MISMATCH', 'PREDICTION_MISMATCH', 'HANDOVER_MISMATCH', 'STALE_SOURCE', 'INVALID_PLAN'):
                        self.fail(reason)
            elif channel == 'world':
                self.world = world_from_wire(payload)
            elif channel == 'goal':
                if payload['limits'] != asdict(verification_limits()):
                    self.fail('LIMITS_MISMATCH')
                self.goal = tuple(payload['xy'])
            elif channel == 'feedback':
                if self.feedback is not None and payload['sample_s'] <= self.feedback['sample_s']:
                    self.fail('FEEDBACK_TIME_REVERSED')
                    return
                self.feedback = payload
                self.origin = payload['origin_monotonic_s']
            elif channel == 'ack':
                self.gate.ack(payload['sequence'], payload['reason'])

        def fail(self, reason):
            self.exit_code = 1
            self.finish(reason)

        def finish(self, reason):
            if self.done:
                return
            self.finish_reason = reason
            self.send('diagnostic', 'FINISHED', dict(reason=reason, sent_count=self.sent_count,
                                                    received_count=self.received_count,
                                                    tail_inflight=bool(self.worker and self.worker.busy)))
            (Path(args.output_dir) / (args.role + '-status.json')).write_text(
                encode(dict(reason=reason, pid=os.getpid(), exit_code=self.exit_code)), encoding='utf-8')
            self.done = True

        def on_timer(self):
            if self.done:
                return
            now = perf_counter()
            if self.origin is None and self.deadline.expired(now) == 'STARTUP_TIMEOUT':
                self.fail('STARTUP_TIMEOUT')
                return
            if args.role == 'probe':
                reason = self.deadline.expired(now)
                if reason is not None:
                    self.fail(reason)
                    return
                if now - self.last_ready >= .2:
                    self.send('diagnostic', 'READY', dict(role='probe'))
                    self.last_ready = now
                return
            if args.role == 'fixture':
                self.fixture_timer(now)
            else:
                self.navigator_timer(now)

        def fixture_timer(self, now):
            if self.origin is None:
                if not {'navigator', 'probe'}.issubset(self.ready_roles):
                    return
                self.send('world', 'WORLD', world_to_wire(self.scenario.world))
                self.send('goal', 'GOAL', dict(xy=self.scenario.goal, allow_unknown=False,
                                               clearance_radius_m=.9, limits=asdict(verification_limits())))
                self.origin = perf_counter()
                self.publish_feedback(0., 0)
                return
            elapsed = now - self.origin
            due = clock_slot(elapsed, self.slot)
            if due is None:
                return
            self.slot, target, skipped = due
            # 单次推进真实经过的槽；不循环发布过去的反馈，也不复制规划状态。
            self.model.advance(target - self.model.sample_s)
            self.publish_feedback(max(0., now - self.origin - target), skipped)
            fatal_reason = fatal_protocol_reason(self.model.protocol_failures)
            if fatal_reason is not None:
                self.fail(fatal_reason)
            elif math.hypot(self.model.vector[0] - self.scenario.goal[0], self.model.vector[1] - self.scenario.goal[1]) <= .1 and abs(self.model.vector[3]) <= .01 and abs(self.model.vector[4]) <= .01:
                self.finish('ARRIVED')
            elif elapsed >= args.timeout_s:
                self.fail('WALL_TIMEOUT')

        def publish_feedback(self, lateness, skipped):
            # feedback() 的唯一调用点；每个 id 对应一条真实发布消息。
            feedback = self.model.feedback()
            self.send('feedback', 'FEEDBACK', dict(**feedback, segments=self.model.last_segments,
                      origin_monotonic_s=self.origin, published_wall_s=perf_counter(),
                      scheduled_slot=self.slot, skipped_slots=skipped, lateness_s=lateness))

        def navigator_timer(self, now):
            if not self.worker.ready:
                ready = self.worker.poll_ready()
                if ready:
                    self.cold = dict(**ready, preclock_wall_ms=(perf_counter() - self.started_wall) * 1000)
                if self.worker.failed:
                    self.fail('WORKER_STARTUP_FAILED')
                return
            if self.origin is None:
                if now - self.last_ready >= .2:
                    self.send('diagnostic', 'READY', dict(role='navigator', cold=self.cold, identity=self.identity))
                    self.last_ready = now
                return
            if self.world is None or self.goal is None or self.feedback is None:
                return
            from nav1cm.verification_plant import prepare_handover, plan_from_candidate
            feedback = self.feedback
            prepared = prepare_handover(feedback, perf_counter() - self.origin,
                                        verification_limits(), self.last_target)
            if prepared is None:
                return
            target = prepared['prediction_target_s']
            began = perf_counter()
            # 始终调用核心；pending/ACK绝不能跳过tick或worker poll。
            result = self.core.tick(self.world, prepared['state'], self.goal, target)
            self.last_target = target
            self.send('diagnostic', 'TICK', dict(sample_s=feedback['sample_s'],
                      source_feedback_id=feedback['feedback_id'], source_feedback_sequence=feedback['sequence'],
                      prediction_target_s=target, predicted_state=asdict(prepared['state']),
                      tick_start_wall_s=began, tick_finish_wall_s=perf_counter(), status=result.status,
                      events=result.events, timings_ms=result.timings_ms, local_busy=self.worker.busy,
                      buffer_remaining_s=self.core.buffer.remaining_s(target),
                      **tick_global_evidence(result,self.goal,self.world.epoch,self.core.route)))
            if result.action is None:
                self.fail('FAILED_NO_CERTIFIED_ACTION')
                return
            if result.status in ('GOAL_IN_OBSTACLE', 'GOAL_CLEARANCE'):
                if not self.target_rejected:
                    self.send('diagnostic', 'TARGET_REJECTED', dict(status=result.status))
                    self.target_rejected = True
                return
            if self.target_rejected:
                return
            if self.gate.can_publish(feedback):
                self.sequence += 1
                plan = plan_from_candidate(result.candidate, self.sequence,
                                           feedback['sample_s'], target, feedback=feedback)
                self.send('plan', 'PLAN', plan_to_wire(plan))
                self.gate.sent(self.sequence)
            else:
                self.send('diagnostic', 'PLAN_PUBLICATION_SKIPPED', dict(target_s=target, reason='AWAITING_ACK_OR_PENDING'))

        def close(self):
            close_result = None
            if self.core:
                close_result = self.core.close()
            self.write('closed', role=args.role, finish_reason=self.finish_reason, close_result=close_result)
            self.stream.close()
            self.destroy_node()

    return VerificationNode()


def main(argv=None, role=None):
    args = parser().parse_args(argv)
    args.role = role or args.role
    if args.role is None:
        raise ValueError('--role required')
    validate_environment(os.environ)
    if (not math.isfinite(args.delay_ms) or args.delay_ms < 0 or
            not 0 < args.timeout_s <= 300 or not 0 < args.startup_timeout_s <= 300):
        raise ValueError('bounded finite timing arguments required')
    import rclpy
    # 自己处理信号，确保先关闭core的已知worker，再关闭ROS context。
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stopping = False
    def stop(signum, frame):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        node = make_node(args)
        while not stopping and not node.done:
            rclpy.spin_once(node, timeout_sec=.1)
    except Exception as exc:
        if node is not None:
            node.send('diagnostic', 'ERROR', dict(type=type(exc).__name__, detail=str(exc)))
            node.fail('EXCEPTION')
        raise
    finally:
        if node is not None:
            if not node.done:
                node.finish('SIGNAL_STOP')
            node.timer.cancel()
            # 有界排空可靠DDS发送，终端反馈也须被独立probe收到。
            drain_until = perf_counter() + .2
            while perf_counter() < drain_until:
                rclpy.spin_once(node, timeout_sec=.02)
            node.close()
        rclpy.shutdown()
    return node.exit_code


def fixture_main():
    return main(role='fixture')


def navigator_main():
    return main(role='navigator')


def probe_main():
    return main(role='probe')


if __name__ == '__main__':
    raise SystemExit(main())
