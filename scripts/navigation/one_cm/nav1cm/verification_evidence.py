"""无 ROS 的只读验证证据：保真序列化、正式全局身份及实际执行覆盖。"""
from dataclasses import fields, is_dataclass
import math
import numpy as np


def serialize_global_result(value):
    """只转 JSON 类型；不补齐、不推断正式成功字段。"""
    if is_dataclass(value):
        return {f.name: serialize_global_result(getattr(value, f.name)) for f in fields(value)}
    if isinstance(value, np.ndarray):
        return serialize_global_result(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: serialize_global_result(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [serialize_global_result(item) for item in value]
    return value


def tick_global_evidence(result, goal, epoch, route):
    """绑定本次 tick 调用输入及调用后的实际活动路线。"""
    return dict(global_result=serialize_global_result(result.global_result),
                current_goal=list(goal), current_epoch=epoch,
                route_version=route.version if route is not None else None)


def formal_global_evidence(rows):
    """只把有实际提交、三字段及同 tick 接受身份的结果计为正式成功。"""
    requests = {}
    results = []
    accepted = 0
    invalid_acceptances = 0
    for index, row in enumerate(rows):
        if row.get('kind') != 'TICK':
            continue
        events = row.get('events', [])
        raw = row.get('global_result')
        accepts = [e for e in events if e.get('kind') == 'ROUTE_ACCEPTED']
        if raw is not None:
            version = raw.get('requested_version')
            route = raw.get('route') or {}
            triple = (type(raw.get('planning_outcome')) is int and raw['planning_outcome'] == 0
                      and raw.get('reason_code') == 'PLAN_FOUND' and raw.get('has_reference') is True)
            identity = (raw.get('requested_goal'), raw.get('requested_epoch'))
            current = (row.get('current_goal'), row.get('current_epoch'))
            bound = (version is not None and identity[0] is not None and identity[1] is not None
                     and identity == current and requests.get(version) == identity
                     and route.get('version') == version and route.get('map_epoch') == identity[1]
                     and route.get('goal') == identity[0])
            stale = any(e.get('kind') == 'STALE_GLOBAL_RESULT' for e in events)
            retained = [e for e in events if e.get('kind') == 'ROUTE_CONTINUITY_RETAINED']
            matched = (len(accepts) == 1 and accepts[0].get('version') == version
                       and accepts[0].get('map_epoch') == identity[1]
                       and accepts[0].get('points') == route.get('points')
                       and row.get('route_version') == version)
            if stale:
                disposition = 'STALE'
            elif not triple:
                disposition = 'NOT_FORMAL_SUCCESS'
            elif not bound:
                disposition = 'IDENTITY_MISMATCH'
            elif matched and not retained:
                disposition = 'ACCEPTED'
                accepted += 1
            elif (not accepts and len(retained) == 1 and retained[0].get('computed_version') == version
                  and retained[0].get('version') == row.get('route_version')
                  and row.get('route_version') is not None):
                disposition = 'RETAINED'
            else:
                disposition = 'IDENTITY_MISMATCH'
            results.append(dict(row_index=index, requested_version=version, disposition=disposition,
                                formal_triple=triple, identity_matches=bound))
            if accepts and disposition != 'ACCEPTED':
                invalid_acceptances += len(accepts)
        elif accepts:
            invalid_acceptances += len(accepts)
        # poll precedes submit inside core.tick; a same-tick submit cannot witness its result.
        for event in events:
            if event.get('kind') == 'GLOBAL_SUBMITTED':
                requests[event['version']] = (row.get('current_goal'), row.get('current_epoch'))
    passed = accepted > 0 and invalid_acceptances == 0
    return dict(status='VALID' if passed else ('NOT_RECORDED' if not results else 'NO_VALID_ACCEPTED_PROOF'),
                **{'pass': passed}, accepted_successes=accepted, invalid_acceptances=invalid_acceptances,
                recorded_results=len(results), results=results)


def execution_coverage(rows, audit=None, *, initial_anchor=None):
    """每个正时段须连贯且有对应审计；显式配置锚点永不伪装发布反馈。"""
    samples = [row for row in rows if row.get('kind') in ('READY', 'FEEDBACK') and 'vector' in row]
    errors = []
    checked = 0
    anchor_source = None
    if samples and samples[0].get('sample_s') == 0.:
        anchor_source = samples[0]['kind']
    elif initial_anchor is not None:
        if not initial_anchor.get('source'):
            raise ValueError('initial_anchor requires explicit source')
        samples = [dict(initial_anchor, kind='CONFIGURED_ANCHOR', segments=[])] + samples
        anchor_source = initial_anchor['source']
    if len(samples) < 2:
        return dict(complete=False, structure_complete=False, errors=[dict(reason='INSUFFICIENT_FEEDBACK')],
                    checked_segments=0, initial_anchor_source=anchor_source)
    def finite(value):
        return isinstance(value, (int, float)) and math.isfinite(value)
    def vector_equal(left, right):
        try:
            left, right = np.asarray(left, dtype=float), np.asarray(right, dtype=float)
            return left.shape == right.shape == (6,) and np.isfinite(left).all() and np.isfinite(right).all() and np.allclose(left, right, atol=1e-6, rtol=0)
        except (TypeError, ValueError):
            return False
    initial = samples[0]
    if (not finite(initial.get('sample_s')) or abs(initial['sample_s']) > 1e-9
            or initial.get('segments') or not vector_equal(initial.get('vector'), initial.get('vector'))):
        errors.append(dict(reason='MISSING_OR_INVALID_INITIAL_SAMPLE'))
    segments_to_audit = []
    for index, (previous, current) in enumerate(zip(samples, samples[1:]), start=1):
        cursor = previous.get('sample_s')
        vector = previous.get('vector')
        segments = current.get('segments', [])
        if not segments or not finite(cursor) or not finite(current.get('sample_s')) or current['sample_s'] <= cursor:
            errors.append(dict(sample_index=index, reason='MISSING_SEGMENTS_OR_TIME_REVERSED'))
            continue
        for offset, segment in enumerate(segments):
            checked += 1
            start, end = segment.get('start_s'), segment.get('end_s')
            if (not finite(start) or not finite(end) or not finite(cursor) or abs(start - cursor) > 1e-9
                    or end <= start or end > current['sample_s'] + 1e-9):
                errors.append(dict(sample_index=index, segment_index=offset, reason='SEGMENT_TIME_GAP_OVERLAP_OR_ORDER'))
            if not vector_equal(vector, segment.get('before')):
                errors.append(dict(sample_index=index, segment_index=offset, reason='SEGMENT_VECTOR_DISCONNECTED'))
            cursor, vector = end, segment.get('after')
            segments_to_audit.append((index, offset, segment))
        if not finite(cursor) or abs(cursor - current['sample_s']) > 1e-9 or not vector_equal(vector, current.get('vector')):
            errors.append(dict(sample_index=index, reason='FEEDBACK_INTERVAL_NOT_COVERED'))
    structure_complete = not errors and checked > 0
    if audit is not None:
        # 终点与地图事件的零时长认证不属于动作段，不能填补缺失审计。
        entries = {}
        for entry in audit:
            start, end = entry.get('start_s'), entry.get('end_s')
            if finite(start) and finite(end) and end > start:
                entries.setdefault((start, end, entry.get('source')), []).append(entry)
        for index, offset, segment in segments_to_audit:
            matches = entries.get((segment.get('start_s'), segment.get('end_s'), segment.get('source')), [])
            if len(matches) != 1 or matches[0].get('valid') is not True:
                errors.append(dict(sample_index=index, segment_index=offset, reason='MISSING_DUPLICATE_OR_INVALID_ACTION_AUDIT'))
    return dict(complete=not errors and checked > 0, structure_complete=structure_complete,
                errors=errors, checked_segments=checked, initial_anchor_source=anchor_source,
                initial_sample_s=initial.get('sample_s'), final_sample_s=samples[-1].get('sample_s'),
                action_audit_checked=audit is not None)
