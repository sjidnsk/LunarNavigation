"""P3 无控制联调的纯时序观察；预算不属于车辆执行配置。

保留源戳、ROS 接收戳、单调接收时间；健康查询不修改状态。源戳高水位
仅在有效且严格推进时更新。时钟回退锁存到显式以 previous=None 重建观察。
"""
import math


def _finite_number(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


def _stamp(value):
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def record_sample(previous, source_ns, received_ros_ns, received_mono_s):
    """返回新的观察字典，不改 previous，不替换任何实际输入戳。

    previous 为本函数上次返回值或 None。无效/未来戳不推进高水位。
    重复或回退源戳仍记录实际接收时间，但不刷新 last_advanced_mono_s。
    """
    previous = previous or {}
    sample = dict(source_ns=source_ns, received_ros_ns=received_ros_ns,
                  received_mono_s=received_mono_s,
                  high_water_source_ns=previous.get('high_water_source_ns'),
                  last_advanced_mono_s=previous.get('last_advanced_mono_s'),
                  high_water_ros_ns=previous.get('high_water_ros_ns'),
                  high_water_mono_s=previous.get('high_water_mono_s'),
                  fault=None)
    if previous.get('fault') == 'CLOCK_ROLLBACK':
        sample['fault'] = 'CLOCK_ROLLBACK'
        return sample
    if not (_stamp(source_ns) and _stamp(received_ros_ns)
            and _finite_number(received_mono_s) and received_mono_s >= 0):
        sample['fault'] = 'INVALID_TIMESTAMP'
        return sample
    old_ros = previous.get('high_water_ros_ns')
    old_mono = previous.get('high_water_mono_s')
    if ((_stamp(old_ros) and received_ros_ns < old_ros)
            or (_finite_number(old_mono) and received_mono_s < old_mono)):
        sample['fault'] = 'CLOCK_ROLLBACK'
        return sample
    sample['high_water_ros_ns'] = received_ros_ns
    sample['high_water_mono_s'] = received_mono_s
    if source_ns > received_ros_ns:
        sample['fault'] = 'FUTURE_TIMESTAMP'
        return sample
    high_water = sample['high_water_source_ns']
    if high_water is None or source_ns > high_water:
        sample['high_water_source_ns'] = source_ns
        sample['last_advanced_mono_s'] = received_mono_s
    elif source_ns < high_water:
        sample['fault'] = 'SOURCE_ROLLBACK'
    return sample


def timing_health(sample, ros_now_ns, mono_now_s, *, max_source_age_s,
                  receive_timeout_s, progress_timeout_s):
    """只读检查真实年龄。所有预算必须有限且严格为正，否则 ValueError。

    返回 ready/reason 与三个秒数；无法计算的年龄为 None。优先级为缺失、
    输入异常、时钟回退、接收断流、源冻结、源过旧、READY。正向 ROS 跳变
    仍按真实源年龄判断；不假定 ROS 时钟与单调钟的推进速度相同。
    """
    for value in (max_source_age_s, receive_timeout_s, progress_timeout_s):
        if not _finite_number(value) or value <= 0:
            raise ValueError('observation budgets must be finite and positive')
    result = dict(ready=False, reason='MISSING_SAMPLE', source_age_s=None,
                  receive_age_s=None, progress_age_s=None)
    if sample is None:
        return result
    source = sample.get('source_ns')
    receipt_ros = sample.get('received_ros_ns')
    receipt_mono = sample.get('received_mono_s')
    advance = sample.get('last_advanced_mono_s')
    if not (_stamp(ros_now_ns) and _finite_number(mono_now_s) and mono_now_s >= 0
            and _stamp(source) and _stamp(receipt_ros)
            and _finite_number(receipt_mono) and receipt_mono >= 0):
        result['reason'] = 'INVALID_TIMESTAMP'
        return result
    result['source_age_s'] = (ros_now_ns - source) / 1e9
    result['receive_age_s'] = mono_now_s - receipt_mono
    if _finite_number(advance):
        result['progress_age_s'] = mono_now_s - advance
    fault = sample.get('fault')
    if fault:
        result['reason'] = fault
    elif ros_now_ns < receipt_ros or mono_now_s < receipt_mono or (
            advance is not None and mono_now_s < advance):
        result['reason'] = 'CLOCK_ROLLBACK'
    elif source > ros_now_ns or source > receipt_ros:
        result['reason'] = 'FUTURE_TIMESTAMP'
    elif result['receive_age_s'] > receive_timeout_s:
        result['reason'] = 'RECEIVE_TIMEOUT'
    elif advance is None:
        result['reason'] = 'INVALID_TIMESTAMP'
    elif result['progress_age_s'] > progress_timeout_s:
        result['reason'] = 'SOURCE_FROZEN'
    elif result['source_age_s'] > max_source_age_s:
        result['reason'] = 'SOURCE_TOO_OLD'
    else:
        result.update(ready=True, reason='READY')
    return result

