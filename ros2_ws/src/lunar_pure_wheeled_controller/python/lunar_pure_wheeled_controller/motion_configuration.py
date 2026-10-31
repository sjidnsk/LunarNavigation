"""同一平台文件的运动模型选择；旧执行器保持既有能力合同。"""
MODELS=('differential_drive','curvature_limited')


def resolve_motion_model(capability,requested,*,standard_path=False,explicit=False):
    if not standard_path:
        if explicit and requested!='curvature_limited':
            raise ValueError('differential_drive requires standard Path MPPI')
        return 'curvature_limited'
    value=requested if explicit else capability.get('motion_model','curvature_limited')
    if not isinstance(value,str) or value not in MODELS:
        raise ValueError('unknown motion_model: '+str(value))
    return value


def p3_map_receipt_expired(received_at,timeout_s,now_s):
    """仅 direct_p3 支持 0 禁用接收年龄门禁；缺少有效接收仍不可用。"""
    return received_at is None or (timeout_s>0 and now_s-received_at>timeout_s)


def input_disposition(reason,*,hard_failure=False):
    """输入等待、候选重算、真实失败的唯一分类；硬故障不可被等待覆盖。"""
    if hard_failure:return 'FAIL'
    if reason in ('missing_or_stale_feedback','stale_source_feedback',
                  'stale_feedback_after_optimization','stale_feedback_during_recertification',
                  'stale_feedback_after_certification'):
        return 'WAITING_FOR_POSE'
    if reason in ('missing_or_stale_map','stale_map_after_optimization','stale_map_after_certification'):
        return 'WAITING_FOR_MAP'
    if reason in ('map_changed_during_certification','publication_certificate_expired'):
        return 'RECOMPUTE'
    return 'FAIL'
