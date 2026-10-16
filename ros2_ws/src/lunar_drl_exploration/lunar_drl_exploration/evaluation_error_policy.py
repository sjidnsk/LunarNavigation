"""Explicit opt-in policy: operational case errors count as failed evaluations."""
import math
from pathlib import Path


def enabled(root):
    from .course_two_control import read
    path=Path(root)/'evaluation-settings.json'
    policy=read(path).get('error_policy','stop') if path.exists() else 'stop'
    if policy not in ('stop','count_failure'):raise ValueError('unknown evaluation error policy')
    return policy=='count_failure'


def valid_coverage(value):
    return type(value) in (int,float) and math.isfinite(value) and 0<=value<=1


def counted_failure(row):
    """Preserve errors and measured coverage; never infer completion from an error."""
    if not isinstance(row.get('error'),str) or not row['error']:
        raise ValueError('counted failure requires an error record')
    result=dict(row,completed=False,evaluation_failure_kind='infrastructure',
                truncation_reason='INFRASTRUCTURE_ERROR')
    coverage=row.get('final_coverage')
    if not valid_coverage(coverage):
        trajectory=row.get('trajectory',[])
        coverage=trajectory[-1][3] if trajectory and len(trajectory[-1])>=4 else row.get('initial_coverage')
        result['coverage_source']='last_observed' if valid_coverage(coverage) else 'unavailable'
    result['final_coverage']=coverage if valid_coverage(coverage) else None
    result.setdefault('initial_completed',False)
    result.setdefault('collisions',None)
    return result
