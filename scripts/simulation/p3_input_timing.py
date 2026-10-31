"""P3 时序观察与执行预检；记录延迟和断档，不改写输入戳。

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
    # 常数空间统计：到达时的源延迟与相邻接收间隔分别累计。
    sample['statistics'] = {key: dict(value) for key,value in previous.get('statistics', {}).items()}
    def observe(key,value):
        item=sample['statistics'].setdefault(key,dict(count=0,min=value,max=value,total=0.))
        item.update(count=item['count']+1,min=min(item['min'],value),
                    max=max(item['max'],value),total=item['total']+value)
    if (_stamp(source_ns) and _stamp(received_ros_ns) and source_ns<=received_ros_ns):
        observe('source_delay_at_receive_s',(received_ros_ns-source_ns)/1e9)
    old_receipt=previous.get('received_mono_s')
    if (_finite_number(old_receipt) and _finite_number(received_mono_s)
            and 0<=old_receipt<=received_mono_s):
        observe('receive_gap_s',received_mono_s-old_receipt)
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


def _timing_values(sample, ros_now_ns, mono_now_s):
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

    else:
        result.update(ready=True, reason='READY')
    return result


def timing_health(sample, ros_now_ns, mono_now_s, *, max_source_age_s,
                  receive_timeout_s, progress_timeout_s):
    """通用观察模式保留原三个严格正预算；不作为地图运行合同。"""
    for value in (max_source_age_s, receive_timeout_s, progress_timeout_s):
        if not _finite_number(value) or value <= 0:
            raise ValueError('observation budgets must be finite and positive')
    result=_timing_values(sample,ros_now_ns,mono_now_s)
    if not result['ready']:return result
    if result['receive_age_s']>receive_timeout_s:reason='RECEIVE_TIMEOUT'
    elif result['progress_age_s'] is None:reason='INVALID_TIMESTAMP'
    elif result['progress_age_s']>progress_timeout_s:reason='SOURCE_FROZEN'
    elif result['source_age_s']>max_source_age_s:reason='SOURCE_TOO_OLD'
    else:return result
    return dict(result,ready=False,reason=reason)


def runtime_input_health(kind,sample,ros_now_ns,mono_now_s,*,odometry_timeout_s,p3_map_timeout_s):
    """匹配标准 Path 输入时间合同，不替代地图处理/几何/停车认证。

    定位检查源戳推进接收龄；direct_p3 地图只检查接收龄，0保留静态图策略。
    源延迟和戳推进年龄一直如实返回，不额外缩短配置预算。
    """
    if kind not in ('odom','map'):raise ValueError('unknown input kind')
    if (not _finite_number(odometry_timeout_s) or odometry_timeout_s<=0
            or not _finite_number(p3_map_timeout_s) or p3_map_timeout_s<0):
        raise ValueError('invalid execution input budget')
    result=_timing_values(sample,ros_now_ns,mono_now_s)
    if not result['ready']:return result
    budget=odometry_timeout_s if kind=='odom' else p3_map_timeout_s
    if budget>0 and result['receive_age_s']>budget:
        return dict(result,ready=False,reason='RECEIVE_TIMEOUT')
    if kind=='odom' and (result['progress_age_s'] is None or result['progress_age_s']>budget):
        return dict(result,ready=False,reason='SOURCE_FROZEN')
    return result


def timing_statistics(sample,ros_now_ns,mono_now_s):
    """闭合帧间隔与当前仍在增长的接收断档分别报告；查询不刷新状态。"""
    result={}
    for key in ('source_delay_at_receive_s','receive_gap_s'):
        value=(sample or {}).get('statistics',{}).get(key)
        result[key]=(dict(count=value['count'],min=value['min'],max=value['max'],
                          mean=value['total']/value['count']) if value else
                     dict(count=0,min=None,max=None,mean=None))
    ages=_timing_values(sample,ros_now_ns,mono_now_s)
    result.update(current_source_age_s=ages['source_age_s'],
                  current_receive_gap_s=ages['receive_age_s'],
                  current_source_stall_s=ages['progress_age_s'])
    return result
