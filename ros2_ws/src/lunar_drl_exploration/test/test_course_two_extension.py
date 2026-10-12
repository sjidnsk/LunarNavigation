"""Budget extension preserves partial work, frozen blocks and failed gates."""
import json
from pathlib import Path
from types import SimpleNamespace
import pytest

@pytest.mark.parametrize('interrupt_at',[None,2])
def test_four_extended_blocks_continue_and_resume_exact_budgets(tmp_path,monkeypatch,interrupt_at):
    from lunar_drl_exploration import course_two_trial as trial,course_two_control as ctl
    import lunar_drl_exploration.evaluation as evaluation
    d=tmp_path/'arms/C3';run=d/'training/run';run.mkdir(parents=True)
    initial=dict(transitions=257497,updates=64000,credit='0')
    ctl.write(d/'prefill.pt',dict(schedule=initial))
    ctl.write(run/'resume.pt',dict(schedule=initial))
    ctl.write(tmp_path/'cases.json',[dict(case_id='case')])
    monkeypatch.setattr(trial,'checkpoint',lambda p:SimpleNamespace(record=ctl.read(p)))
    monkeypatch.setattr(evaluation,'export_actor',lambda p,out:Path(out).write_bytes(Path(p).read_bytes()))
    monkeypatch.setattr(trial,'gate',lambda rows,baseline:dict(passed=False,stop=True))
    class Fake:
        root=tmp_path
        identity=dict(continuation_blocks=4,block_admissions=4096,pilot_admissions=16384,pilot_updates=4096)
        def __init__(self):self.stopping=False;self.state={'results':{}};self.budgets=[];self.interrupt=interrupt_at;self.labels=[]
        def initialize_arm(self,arm):return SimpleNamespace(output_dir=str(run))
        def save(self,**values):self.state.update(values)
        def spawn(self,args,label,**env):
            budget=int(args[args.index('--max-transitions')+1]);self.budgets.append(budget)
            v=ctl.read(run/'resume.pt')
            interrupted=len(self.budgets)==self.interrupt
            admitted=budget//2 if interrupted else budget
            v['schedule']['transitions']+=admitted;v['schedule']['updates']+=admitted//4
            ctl.write(run/'resume.pt',v)
            return not interrupted
        def screen(self,actor,cases,label):self.labels.append(label);return []
    f=Fake();done=trial.Trial.extend(f,[])
    if interrupt_at:
        assert not done and len(f.state['results'])==1
        preserved=(d/'block01.pt').read_bytes()
        f.interrupt=None;assert trial.Trial.extend(f,[])
        assert (d/'block01.pt').read_bytes()==preserved
        assert f.budgets==[4096,4096,2048,4096,4096]
    else:
        assert done and f.budgets==[4096]*4
    assert f.labels==[f'C3-block{i:02}' for i in range(1,5)]
    assert len(f.state['results'])==4
    final=ctl.read(run/'resume.pt')['schedule']
    assert final['transitions']==257497+16384 and final['updates']==64000+4096
    assert all(not g['passed'] and g['stop'] for g in f.state['results'].values())
