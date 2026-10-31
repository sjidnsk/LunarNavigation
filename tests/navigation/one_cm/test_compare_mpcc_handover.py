from types import SimpleNamespace
import numpy as np
import pytest
from scripts.analysis.compare_mpcc_handover import simulate, summarize, ScheduledLocal
from scripts.navigation.one_cm.nav1cm.contracts import Trajectory, WheelLimits
from scripts.navigation.one_cm.nav1cm.seeds import initial_seed


class ProbePlanner:
    dt=.2
    limits=WheelLimits(motion_model='differential_drive')
    def __init__(self,limits=None,build_dir=None,n=20,dt=.2):
        self.requests=[]
        if limits is not None:self.limits=limits
        self.dt=dt
    def plan(self,state,grid,reference,**kwargs):
        self.requests.append((state,grid.epoch,reference.route_version))
        initial=[state.x,state.y,state.yaw,state.v,state.w,reference.progress_s]
        delta=reference.points[1]-reference.points[0]
        yaw=float(np.arctan2(delta[1],delta[0]));mode='spin' if abs(yaw-state.yaw)>.3 else 'forward'
        xs,us=initial_seed(initial,mode,self.limits,yaw,n=20,dt=self.dt)
        candidate=Trajectory(grid.epoch,reference.route_version,np.arange(21)*self.dt,xs,us,mode,'test:probe')
        return SimpleNamespace(candidate=candidate,reason='TEST_PROBE',attempts=(),timings_ms={})


@pytest.mark.parametrize('scenario',['straight','curve','spin','refresh'])
def test_real_coordinator_buffer_and_command_proof_roll_forward(scenario):
    p=ProbePlanner()
    result=simulate(p,scenario,duration=.5,refresh_period=.1)
    rows=result['rows']
    assert len(rows)==10
    assert any(r['accepted'] for r in rows)
    assert any(max(abs(v) for v in r['issued'])>0 for r in rows)
    assert any(r['handover_target_jump'] is not None for r in rows)
    assert all('command_reject_reason' in r['diagnostics'] for r in rows)
    assert rows[-1]['actual_state']!=rows[0]['actual_state']
    if scenario=='refresh':
        assert max(r['epoch'] for r in rows)>1
        assert max(r['route_version'] for r in rows)>1


def test_simulated_feedback_delay_is_explicit_and_preserves_actual_history():
    result=simulate(ProbePlanner(),'straight',duration=.4,feedback_delay=2.5)
    assert all(r['feedback_state']['x']==0 for r in result['rows'])
    assert result['rows'][-1]['actual_state']['x']>0


def test_delayed_results_do_not_fake_candidate_acceptance():
    result=simulate(ProbePlanner(),'straight',duration=.3,result_delay=2.5)
    assert not any(r['accepted'] for r in result['rows'])
    assert all(r['issued']==[0.,0.] for r in result['rows'])
    assert result['summary']['zero_insertions']==0
    assert result['summary']['nearzero_recovery_proxies']==0
    assert result['summary']['command_rejections']


def test_cli_freezes_limits_hold_and_exports_json(monkeypatch,tmp_path):
    import json
    from scripts.analysis import compare_mpcc_handover as module
    limits=tmp_path/'limits.json';limits.write_text(json.dumps(vars(ProbePlanner.limits)),encoding='utf-8')
    output=tmp_path/'result.json'
    builds=[]
    class BuildProbe(ProbePlanner):
        def __init__(self,limits,build_dir,**kwargs):
            builds.append(build_dir);super().__init__(limits,build_dir,**kwargs)
    monkeypatch.setattr(module,'load_planner',lambda path,name:(BuildProbe,dict(path=path,sha256='test-only')))
    hold=[];original=module.certified_velocity_command
    def command(*args,**kwargs):
        hold.append(kwargs['command_hold_s']);return original(*args,**kwargs)
    monkeypatch.setattr(module,'certified_velocity_command',command)
    module.main(['--baseline-module','old.py','--warm-module','new.py','--limits-json',str(limits),
        '--build-dir',str(tmp_path/'build'),'--output',str(output),'--duration','.3','--scenarios','straight'])
    result=json.loads(output.read_text(encoding='utf-8'))
    assert builds==[tmp_path/'build',tmp_path/'build']
    assert result['evidence']=='MODEL_SIMULATION'
    assert result['command_hold_s']==1.2
    assert set(hold)=={1.2}
    assert result['limits']==vars(ProbePlanner.limits)
    assert result['limits_file']['sha256']
    assert result['shared']['coordinator']['sha256']
    assert result['arms']['baseline']['runs'][0]['rows']
    assert 'nearzero_recovery_proxies' in result['arms']['warm']['runs'][0]['summary']
