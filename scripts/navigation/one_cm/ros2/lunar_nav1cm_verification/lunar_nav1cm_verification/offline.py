"""进程退出后才载入全量证据和认证实际执行；绝不回灌或过滤车辆动作。"""
from dataclasses import fields
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from .contract import decode, encode, plan_from_wire, verification_limits
from nav1cm.verification_plant import TimedPlan
from nav1cm.verification_evidence import execution_coverage


def transport_evidence(streams):
    pids = {}
    pairs = set()
    received = {'feedback': {}, 'plan': {}, 'ack': {}}
    expected = {'feedback': {}, 'plan': {}, 'ack': {}}
    errors = []
    formal_sent, formal_received = {}, {}
    sessions = set()
    for role, rows in streams.items():
        identities = [r['pid'] for r in rows if r['direction'] == 'identity']
        if len(identities) == 1:
            pids[role] = identities[0]
    for role, rows in streams.items():
        last_log_s = -float('inf')
        last_sent_by_channel = {}
        last_received_by_writer = {}
        for row in rows:
            msg = row.get('message')
            if not msg:
                if row['direction'] == 'rejected_wire':
                    errors.append(dict(receiver=role, reason=row['reason']))
                continue
            logged_s = row.get('received_monotonic_s')
            if logged_s is None or not math.isfinite(logged_s) or logged_s < last_log_s:
                errors.append(dict(receiver=role, reason='LOG_CLOCK_NONMONOTONIC'))
            else:
                last_log_s = logged_s
            sessions.add(msg['session'])
            try:
                decode(encode(msg), msg['session'])
            except (ValueError, TypeError, KeyError):
                errors.append(dict(receiver=role, reason='INVALID_ENVELOPE'))
            if logged_s is None or msg['sent_monotonic_s'] > logged_s + 1e-9:
                errors.append(dict(receiver=role, reason='SEND_AFTER_LOG_OR_RECEIPT'))
            if msg['producer_pid'] != pids.get(msg['producer']):
                errors.append(dict(receiver=role, reason='PRODUCER_PID_MISMATCH'))
            if row['receiver_pid'] != pids.get(role):
                errors.append(dict(receiver=role, reason='RECEIVER_PID_MISMATCH'))
            if row['channel'] in ('plan', 'ack'):
                try:
                    plan_payload = msg['payload'] if row['channel'] == 'plan' else msg['payload']['plan']
                    if set(plan_payload) != {field.name for field in fields(TimedPlan)}:
                        raise ValueError('incomplete TimedPlan payload')
                    plan_from_wire(plan_payload)
                except (ValueError, TypeError, KeyError):
                    errors.append(dict(receiver=role, reason='INVALID_OR_INCOMPLETE_TIMED_PLAN', channel=row['channel']))
            if msg.get('kind') == 'TICK' and msg['payload'].get('global_result') is not None:
                identity = (msg['producer'], msg['session'], msg['sent_monotonic_s'])
                digest = hashlib.sha256(json.dumps(msg, sort_keys=True, allow_nan=False, separators=(',', ':')).encode('utf-8')).hexdigest()
                target = formal_sent if row['direction'] == 'send' and role == 'navigator' else (formal_received if row['direction'] == 'receive' and role == 'probe' else None)
                if target is not None:
                    if identity in target:
                        errors.append(dict(receiver=role, reason='DUPLICATE_FORMAL_TICK'))
                    target[identity] = digest
            if row['direction'] == 'receive':
                writer = (msg['producer'], row['channel'])
                if msg['sent_monotonic_s'] < last_received_by_writer.get(writer, -float('inf')):
                    errors.append(dict(receiver=role, reason='RECEIVED_WRITER_CLOCK_NONMONOTONIC', channel=row['channel']))
                last_received_by_writer[writer] = msg['sent_monotonic_s']
                pairs.add((role, msg['producer']))
                if row['receiver_pid'] == msg['producer_pid'] and role != msg['producer']:
                    errors.append(dict(receiver=role, reason='SAME_PID_CROSS_ROLE'))
                if role == 'probe' and row['channel'] in received:
                    key = 'feedback_id' if row['channel'] == 'feedback' else 'sequence'
                    identity = (msg['producer'], msg['session'], msg['payload'][key])
                    digest = hashlib.sha256(json.dumps(msg, sort_keys=True, allow_nan=False, separators=(',', ':')).encode('utf-8')).hexdigest()
                    if identity in received[row['channel']]:
                        errors.append(dict(receiver=role, reason='DUPLICATE_RECEIVED_ID', channel=row['channel']))
                    received[row['channel']][identity] = digest
            elif row['direction'] == 'send' and row['channel'] in expected:
                key = 'feedback_id' if row['channel'] == 'feedback' else 'sequence'
                identity = (msg['producer'], msg['session'], msg['payload'][key])
                digest = hashlib.sha256(json.dumps(msg, sort_keys=True, allow_nan=False, separators=(',', ':')).encode('utf-8')).hexdigest()
                if identity in expected[row['channel']]:
                    errors.append(dict(receiver=role, reason='DUPLICATE_SENT_ID', channel=row['channel']))
                expected[row['channel']][identity] = digest
            channel = row['channel']
            if row['direction'] == 'send':
                if msg['sent_monotonic_s'] < last_sent_by_channel.get(channel, -float('inf')):
                    errors.append(dict(receiver=role, reason='SEND_CLOCK_NONMONOTONIC', channel=channel))
                last_sent_by_channel[channel] = msg['sent_monotonic_s']
    required = {('probe', 'fixture'), ('probe', 'navigator'), ('fixture', 'navigator'),
                ('fixture', 'probe'), ('navigator', 'fixture')}
    if len(sessions) > 1:
        errors.append(dict(reason='SESSION_MISMATCH'))
    missing = {ch: [list(key) for key in expected[ch] if key not in received[ch]] for ch in expected}
    mismatched = {ch: [list(key) for key in expected[ch] if key in received[ch] and expected[ch][key] != received[ch][key]] for ch in expected}
    extra = {ch: [list(key) for key in received[ch] if key not in expected[ch]] for ch in expected}
    delivery = all(expected[ch] and not missing[ch] and not mismatched[ch] and not extra[ch] for ch in expected) and not errors
    return dict(process_pids=pids, received_role_pairs=sorted([list(x) for x in pairs]),
                dds_three_process_pass=len(pids) == 3 and len(set(pids.values())) == 3 and required <= pairs and not errors,
                complete_probe_delivery=bool(delivery), wire_errors=errors,
                formal_global_probe_delivery=bool(formal_sent) and formal_sent == formal_received and not errors,
                formal_global_probe_sent=len(formal_sent),formal_global_probe_received=len(formal_received),
                missing_probe_messages=missing, mismatched_probe_messages=mismatched, extra_probe_messages=extra,
                message_hash_algorithm='SHA256(canonical complete v1 envelope, sorted JSON keys)')


def analyze(output_dir, geometry_lib):
    from nav1cm.fixtures import scenarios
    from nav1cm.geometry_native import NativeGeometry
    from benchmark_async_navigation import audit_feedback, session_outcome, cadence
    out = Path(output_dir)
    streams = {role: [json.loads(line) for line in (out / (role + '.jsonl')).read_text(encoding='utf-8').splitlines() if line]
               for role in ('fixture', 'navigator', 'probe')}
    transport = transport_evidence(streams)
    # 独立probe收到的数据另作交叉见证；真正动作审计以fixture完整落盘为准。
    rows = []
    for role in ('fixture', 'navigator'):
        for raw in streams[role]:
            if raw['direction'] != 'send':
                continue
            message = raw['message']
            payload = dict(message['payload'])
            kind = message['kind']
            if kind == 'PLAN':
                rows.append(dict(kind='PLAN_SENT', plan=payload, sent_wall_s=message['sent_monotonic_s']))
            else:
                rows.append(dict(kind=kind, **payload))
    samples = [r for r in rows if r['kind'] == 'FEEDBACK']
    ticks = [r for r in rows if r['kind'] == 'TICK']
    intervals = [(e['worker_start_wall_s'], e['worker_finish_wall_s']) for t in ticks for e in t['events']
                 if e['kind'] == 'LOCAL_RESULT' and e.get('worker_start_wall_s') is not None]
    busy = []
    for begin, end in intervals:
        inside = [s for s in samples if begin <= s['published_wall_s'] <= end]
        busy.append(dict(start_wall_s=begin, end_wall_s=end, feedback_count=len(inside),
                         displacement_m=float(np.linalg.norm(np.array(inside[-1]['vector'][:2]) - inside[0]['vector'][:2])) if len(inside) >= 2 else 0.))
    scenario = next(s for s in scenarios() if s.name == 'straight')
    coverage = execution_coverage(rows)
    audit = audit_feedback(rows, scenario, verification_limits(), NativeGeometry(geometry_lib)) if coverage['complete'] else []
    (out / 'execution-audit.json').write_text(json.dumps(audit, indent=2), encoding='utf-8')
    tail = any(r['kind'] == 'FINISHED' and r.get('tail_inflight', False) for r in rows)
    terminal = json.loads((out / 'fixture-status.json').read_text(encoding='utf-8'))
    outcome = session_outcome(rows, audit, scenario.goal, busy, tail, terminal['reason'])
    timing = [t['timings_ms']['total'] for t in ticks]
    summary = dict(**outcome, **transport, worker_busy_evidence=busy, ticks=len(ticks), feedback_count=len(samples),
                   terminal=terminal, final_feedback=samples[-1] if samples else None,
                   tick_ms=dict(zip(('p50', 'p95', 'p99', 'max'), map(float, np.percentile(timing, [50, 95, 99, 100])))) if timing else {},
                   schedules=dict(plant_sample=cadence([s['sample_s'] for s in samples]),
                                  feedback_wall=cadence([s['published_wall_s'] for s in samples]),
                                  core_wall=cadence([t['tick_start_wall_s'] for t in ticks]),
                                  core_target=cadence([t['prediction_target_s'] for t in ticks]),
                                  plan_publication_wall=cadence([r['sent_wall_s'] for r in rows if r['kind'] == 'PLAN_SENT'])),
                   skipped_plant_slots=sum(s['skipped_slots'] for s in samples),
                   plant_lateness_max_ms=max((s['lateness_s'] * 1000 for s in samples), default=None),
                   raw_sha256={role: hashlib.sha256((out / (role + '.jsonl')).read_bytes()).hexdigest() for role in streams},
                   identity={role: next((r['identity'] for r in streams[role] if r['direction'] == 'identity'), None) for role in streams},
                   not_run=['physical sensors/actuators', 'production nodes/P3', 'real vehicle'],
                   publication_contract='20Hz core timer and independent plant timer; lower rate complete TimedPlan publication measured separately')
    closed = {role: [r for r in streams[role] if r['direction'] == 'closed'] for role in streams}
    close_result = closed['navigator'][-1].get('close_result') if closed['navigator'] else None
    worker_closed = bool(close_result) and all(not (close_result.get(pool) or {}).get('surviving_pids', []) for pool in ('local', 'global'))
    launcher = json.loads((out / 'launcher.json').read_text(encoding='utf-8'))
    summary['shutdown_pass'] = bool(all(len(v) == 1 for v in closed.values()) and worker_closed and
                                    all(p['returncode'] == 0 for p in launcher['children'].values()))
    summary['worker_close_result'] = close_result
    summary['launcher'] = launcher
    summary['pass'] = bool(summary['status'] == 'ARRIVED' and summary['navigation_arrived'] and summary['integrity_pass'] and
                           summary['all_executed_actions_audited'] and summary['continuity_pass'] and
                           summary['dds_three_process_pass'] and summary['complete_probe_delivery'] and
                           summary['shutdown_pass'] and terminal['exit_code'] == 0 and
                           summary['formal_global_evidence']['pass'] and summary['formal_global_probe_delivery'])
    (out / 'probe-summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    return summary
