"""独立门禁，只使用原始状态与独立来源证据。"""
import math
def grade(e):
    s=e.get('states',[])
    checks={k:bool(e.get(k)) for k in ('graph_ok','formal','installed','workers_clean','terminal')}
    checks.update(
        motion=bool(s) and max(math.hypot(r['x']-s[0]['x'],r['y']-s[0]['y']) for r in s)>.3,
        collision=bool(s) and all(r['clear'] for r in s),
        bounds=bool(s) and all(all(math.isfinite(r[k]) for k in ('x','y','yaw','v','w','a','alpha')) and
            abs(r['v'])<=.200001 and abs(r['w'])<=1.000001 and abs(r['a'])<=.500001 and abs(r['alpha'])<=.500001 and
            (abs(r['v'])<1e-8 or abs(r['w'])<=abs(r['v'])+1e-7) for r in s),
        rolling=e.get('paths',0)>=30,
        processes=bool(e.get('processes')) and all(p['returncode']==0 and not p['premature'] and
            not p['remaining'] and not p['escalated'] for p in e['processes']),
        supervisor=e.get('supervisor_error') is None)
    return dict(pass_=all(checks.values()),checks=checks)
