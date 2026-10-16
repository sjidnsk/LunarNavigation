"""Pure, deterministic settings for the approved three-scene sensor extension."""
import numpy as np

PROFILE='distance_sensor_10_30_v1'
FOVS=(60.,90.,120.,180.,360.)
RANGE_BANDS=((10.,20.),(20.,30.))


def sensor_spec(seed,ordinal):
    # Each task has its own persisted ordinal; scheduling order cannot change it.
    if ordinal%5==0:return dict(range_m=10.,fov_deg=90.)
    varied=ordinal-ordinal//5-1
    rng=np.random.default_rng(np.random.SeedSequence([seed,402]))
    return dict(range_m=float(rng.uniform(*RANGE_BANDS[varied%len(RANGE_BANDS)])),fov_deg=FOVS[varied%5])


def semantics():
    return dict(scene='three_ordinary_distance_sensor_10_30_v1',scene_profile=PROFILE,
        outside_distance_m=[10.,50.],distance_measure='euclidean_polygon_boundary',
        context_margin_m=62.,sensor_scope='immutable_scenario',range_m=[10.,30.],
        range_bands_m=[list(b) for b in RANGE_BANDS],fov_deg=list(FOVS),
        original_sensor_fraction=.2,replay_scope='current_scene_profile_v1',
        stages=['mixed'],max_blind_gap_m=None)


def evaluation_gate(rows,baseline):
    """Only actions can demonstrate learned success; preserve original scenes."""
    from collections import Counter
    import math
    import statistics
    from .course_two_simple import TASKS
    identity=lambda items:{(r['case_id'],r['kind'],r.get('capability'),r.get('scene_profile'),
        r.get('evaluation_role')) for r in items}
    expected={(t,role):n for t in TASKS for role,n in [('regression',5),('adaptation',15)]}
    if (len(rows)!=60 or len(baseline)!=60 or len(identity(rows))!=60 or identity(rows)!=identity(baseline)
            or any(r['kind']!='simple' or r.get('scene_profile')!=PROFILE for r in rows+baseline)
            or Counter((r.get('capability'),r.get('evaluation_role')) for r in rows)!=expected):
        raise ValueError('distance/sensor evaluation identity/count mismatch')
    previous={r['case_id']:r for r in baseline}
    success=lambda r:bool(r['completed']) and not r['collisions'] and not r.get('error')
    initial=lambda r:r.get('initial_completed') is True
    active=lambda r:not initial(r) and not r.get('error')
    retained=dict(data=not any(r.get('error') for r in rows+baseline),
        initialization=all(type(r.get('initial_completed')) is bool and
            initial(r)==initial(previous[r['case_id']]) for r in rows),
        collisions=not any(r['collisions'] for r in rows))
    development={};checks={}
    for task in TASKS:
        group=[r for r in rows if r['capability']==task]
        policy=[r for r in group if active(r)]
        count=sum(map(success,policy));before=sum(success(previous[r['case_id']])
            for r in group if active(previous[r['case_id']]))
        ratios=[r['decisions']/previous[r['case_id']]['decisions'] for r in policy
            if success(r) and success(previous[r['case_id']]) and active(previous[r['case_id']])
            and previous[r['case_id']]['decisions']>0]
        ratio=statistics.median(ratios) if ratios else None
        original=[r for r in policy if r['evaluation_role']=='regression']
        new=[r for r in policy if r['evaluation_role']=='adaptation']
        old_count=sum(map(success,original))
        old_before=sum(success(previous[r['case_id']]) for r in group
            if r['evaluation_role']=='regression' and active(previous[r['case_id']]))
        development[task]=dict(success=count,baseline=before,total=len(policy),planned=20,
            initial_completed=sum(map(initial,group)),median_decision_ratio=ratio,
            regression_success=old_count,regression_baseline=old_before,regression_total=len(original),
            adaptation_success=sum(map(success,new)),adaptation_total=len(new))
        retained[task]=count>=before-1
        retained[task+'_regression']=old_count>=old_before
        retained[task+'_decisions']=ratio is None or ratio<=1.2
        checks[task+'_adaptation']=bool(new) and sum(map(success,new))>=math.ceil(.9*len(new))
    checks.update(retained)
    checks['development']=all(g['total']>0 and g['success']>=math.ceil(.9*g['total'])
        for g in development.values())
    return dict(passed=all(checks.values()),stop=not all(retained.values()),checks=checks,
        development=development,next_action='end_block_and_review')
