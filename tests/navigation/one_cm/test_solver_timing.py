import numpy as np
import pytest
from scripts.navigation.one_cm.nav1cm import local_mpcc as mpcc
from tests.navigation.one_cm.test_local_mpcc import scene
from scripts.navigation.one_cm.nav1cm.contracts import WheelLimits

class FakeSolver:
    def __init__(self, fail=False, status=0): self.fail=fail; self.status=status
    def reset(self): pass
    def set(self,*args): pass
    def solve(self): return self.status
    def get(self,k,field): return np.array([.1,0.,.1])
    def get_stats(self,field):
        if self.fail: raise RuntimeError('stats unavailable')
        return np.array([1,2]) if field=='qp_iter' else 1.

def planner(fail=False,status=0):
    p=mpcc.MpccPlanner.__new__(mpcc.MpccPlanner)
    p.N=20; p.dt=.2; p.limits=WheelLimits(); p.offsets=np.array([-.394,0,.394]);p.radius=.57
    p._warm=None;p._solvers={'forward':FakeSolver(fail,status)}
    p._candidate_reason=lambda *args:'CANDIDATE'
    return p

@pytest.mark.parametrize('status',[0,1])
def test_disjoint_stage_timings_and_stats(status):
    state,grid,ref=scene(); r=planner(status=status).plan(state,grid,ref)
    stages=('reset','input_set','solve_call','read_validate','diagnostics')
    for row in r.attempts:
        for stage in stages:
            assert row[stage+'_ms']>=0 and row[stage+'_cpu_ms']>=0
        assert row['solve_ms']==sum(row[k+'_ms'] for k in stages[:3])
        assert row['acados_stats']['qp_iter']==[1,2]
    for stage in stages:
        assert r.timings_ms[stage]==sum(row[stage+'_ms'] for row in r.attempts)
    assert r.timings_ms['plan_total']>=r.timings_ms['solve']
    assert r.timings_ms['plan_total_cpu']>=r.timings_ms['solve_cpu']

def test_stats_failure_preserves_candidate():
    state,grid,ref=scene()
    good=planner().plan(state,grid,ref); bad=planner(True).plan(state,grid,ref)
    assert good.candidate.digest()==bad.candidate.digest()
    row=bad.attempts[0]
    assert row['acados_stats']=={}
    assert set(row['acados_stats_unavailable'])==set(mpcc.ACADOS_STATS_FIELDS)

def test_stage_clocks_are_disjoint(monkeypatch):
    clock={'wall':0.,'cpu':0.}
    monkeypatch.setattr(mpcc,'perf_counter',lambda:clock['wall'])
    monkeypatch.setattr(mpcc,'process_time',lambda:clock['cpu'])
    def advance(w,c):clock['wall']+=w;clock['cpu']+=c
    class ClockSolver(FakeSolver):
        def reset(self):advance(.002,.001)
        def set(self,*args):advance(.003,.002)
        def solve(self):advance(.005,.004);return 0
        def get(self,*args):advance(.007,.006);return super().get(*args)
        def get_stats(self,field):advance(.011,.01);return super().get_stats(field)
    p=planner();p._solvers['forward']=ClockSolver()
    state,grid,ref=scene();r=p.plan(state,grid,ref);row=r.attempts[0]
    assert row['reset_ms']==pytest.approx(2)
    assert row['input_set_ms']==pytest.approx((2+21*2+20)*3)
    assert row['solve_call_ms']==pytest.approx(5)
    assert row['read_validate_ms']==pytest.approx(20*7)
    assert row['diagnostics_ms']==pytest.approx(7*11)
    assert row['solve_call_cpu_ms']==pytest.approx(4)
    assert r.timings_ms['plan_total']==pytest.approx(sum(row[k+'_ms'] for k in ('reset','input_set','solve_call','read_validate','diagnostics')))


def test_snapshot_exact_reconstruction_and_reject_digest():
    from dataclasses import asdict
    from scripts.navigation.one_cm.benchmark_solver_timing import restore_frame
    from scripts.navigation.one_cm.nav1cm.contracts import CellRect, WorldSnapshot
    state,grid,ref=scene()
    frame={'world':asdict(WorldSnapshot(1,(),(CellRect(-400,-400,400,400),))),
           'state':asdict(state),'route':{'version':ref.route_version,'map_epoch':1,
           'points':[[0,0],[2,0]],'goal':[2,0]},'reference':asdict(ref),
           'allow_unknown':False,'grid_digest':grid.digest()}
    rebuilt=restore_frame(frame)
    assert rebuilt[1].digest()==grid.digest()
    assert np.array_equal(rebuilt[3].points,ref.points)
    frame['grid_digest']='wrong'
    with pytest.raises(ValueError,match='grid digest'):restore_frame(frame)

def test_replay_retains_all_samples_and_initialization(tmp_path,monkeypatch):
    import json
    from dataclasses import asdict
    from scripts.navigation.one_cm import benchmark_solver_timing as bench
    from scripts.navigation.one_cm.nav1cm.contracts import CellRect,WorldSnapshot
    state,grid,ref=scene()
    frame={'cycle':1,'world':asdict(WorldSnapshot(1,(),(CellRect(-400,-400,400,400),))),
        'state':asdict(state),'route':{'version':ref.route_version,'map_epoch':1,'points':[[0,0],[2,0]],'goal':[2,0]},
        'reference':asdict(ref),'allow_unknown':False,'grid_digest':grid.digest(),'recorded_timings_ms':{'planner_solve':136.}}
    data={'frames':[frame],'limits':asdict(WheelLimits()),'horizon_n':20,'horizon_dt_s':.2}
    snapshot=tmp_path/'snapshot.json'
    snapshot.write_text(json.dumps(data,default=lambda a:a.tolist()),encoding='utf-8')
    def fake_planner(*args,**kwargs):
        p=planner();p.build_dir=tmp_path/'build';p.build_ms=0.;return p
    monkeypatch.setattr(bench,'MpccPlanner',fake_planner)
    summary=bench.run(snapshot,tmp_path/'build',tmp_path/'out',repeats=2)
    rows=[json.loads(r) for r in (tmp_path/'out'/'samples.jsonl').read_text().splitlines()]
    assert len(rows)==6==summary['sample_count']
    assert sum(r['first_call'] for r in rows)==2
    assert rows[-1]['input_kind']=='generated-open-moving'
    assert (tmp_path/'out'/'input-snapshot.json').read_bytes()==snapshot.read_bytes()
    assert summary['snapshot_sha256']==bench.sha(snapshot)
    assert summary['cold_initialization']['wall_ms']>=0

def test_summary_includes_first_call_peak():
    from scripts.navigation.one_cm.benchmark_solver_timing import summarize_frames
    frames=[{'cycle':1977,'recorded_timings_ms':{'planner_solve':136.}}]
    rows=[{'frame_index':0,'first_call':first,'timings_ms':{'solve':value}}
          for first,value in ((True,140.),(False,2.),(False,3.))]
    row=summarize_frames(frames,rows)[0]
    assert row['recorded_solve_peak_reached'] is True
    assert row['solve_ms']['max']==140.
    assert row['groups']['first_call']['recorded_solve_peak_reached'] is True
    assert row['groups']['steady']['recorded_solve_peak_reached'] is False
    assert row['groups']['all']['count']==3

@pytest.mark.parametrize('change,expected',[(None,'RESTORED'),('grid','GRID_DIGEST_MISMATCH'),('route','ROUTE_VERSION_MISMATCH'),('mode','MODE_MISMATCH')])
def test_previous_candidate_warm_identity(change,expected):
    from scripts.navigation.one_cm.benchmark_solver_timing import restore_previous_warm
    from dataclasses import replace
    state,grid,ref=scene();p=planner();previous=p.plan(state,grid,ref).candidate
    frame={'grid_digest':grid.digest(),'reference':{'route_version':ref.route_version}}
    if change=='grid':frame['grid_digest']='other'
    if change=='route':frame['reference']['route_version']+=1
    if change=='mode':previous=replace(previous,mode='spin')
    p._warm=None
    detail=restore_previous_warm(p,frame,previous,state,grid,ref)
    assert detail['reason']==expected
    assert detail['restored']==(change is None)
    assert (p._warm is not None)==(change is None)


def test_reanalysis_preserves_runtime_source_identity(tmp_path):
    import json
    from scripts.navigation.one_cm.benchmark_solver_timing import reanalyze
    original={'source_sha256':{'nav1cm/local_mpcc.py':'original-runtime'},'library_sha256':{'solver.so':'original-lib'},'frames':[]}
    files={'summary.json':original,'replay-inputs.json':[{'cycle':1,'recorded_timings_ms':{'planner_solve':136.}}]}
    for name,data in files.items():(tmp_path/name).write_text(json.dumps(data),encoding='utf-8')
    (tmp_path/'samples.jsonl').write_text(json.dumps({'frame_index':0,'first_call':True,'timings_ms':{'solve':140.}})+'\n',encoding='utf-8')
    updated=reanalyze(tmp_path/'summary.json',tmp_path/'replay-inputs.json',tmp_path/'samples.jsonl')
    assert updated['source_sha256']==original['source_sha256']
    assert updated['library_sha256']==original['library_sha256']
    assert updated['frames'][0]['recorded_solve_peak_reached'] is True
    assert updated['analysis_metadata']['execution_rerun'] is False
    assert json.loads((tmp_path/'summary.json').read_text())==original
