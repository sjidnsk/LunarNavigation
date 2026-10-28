"""验证参数来自运行前冻结的外部JSON，独立于生产模块和plant。"""
import json,math
from pathlib import Path

def load(path):
    if not path:return None
    data=json.loads(Path(path).read_text(encoding='utf-8'))
    assert set(data)=={'planner','follower','expected','durations'}
    base={'max_v','max_w','max_a','max_alpha','curvature','xy','yaw'}
    assert set(data['expected']) in (base,base|{'minimum_clearance_m'})
    assert all(math.isfinite(data['expected'][k]) and data['expected'][k]>0 for k in base)
    if 'minimum_clearance_m' in data['expected']:
        value=data['expected']['minimum_clearance_m'];assert math.isfinite(value) and value>=0
    assert all(math.isfinite(v) and 0<v<=120 for v in data['durations'].values())
    return data

def within_bounds(row,expected):
    return all(math.isfinite(row[k]) and abs(row[k])<=expected[n]+1e-6
               for k,n in [('v','max_v'),('w','max_w'),('a','max_a'),('alpha','max_alpha')]) and (
        abs(row['v'])<1e-8 or abs(row['w'])<=abs(row['v'])*expected['curvature']+1e-7)

def held(rows,target,expected):
    start=None
    for row in rows:
        valid=(math.hypot(row['x']-target[0],row['y']-target[1])<=expected['xy'] and
               abs(math.atan2(math.sin(row['yaw']-target[2]),math.cos(row['yaw']-target[2])))<=expected['yaw'] and
               abs(row['v'])<=.01 and abs(row['w'])<=.01)
        start=row['t'] if valid and start is None else start if valid else None
        if start is not None and row['t']-start>=.3:return True
    return False
