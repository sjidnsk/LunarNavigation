"""Budget extension preserves partial work, frozen blocks and failed gates."""
import json
from pathlib import Path
from types import SimpleNamespace
import pytest

@pytest.mark.parametrize('interrupt_at',[None,2])
@pytest.mark.parametrize('block_size',[4096,8192])
def test_four_extended_blocks_continue_and_resume_exact_budgets(tmp_path,monkeypatch,interrupt_at,block_size):
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
        identity=dict(continuation_blocks=4,block_admissions=block_size,pilot_admissions=block_size*4,pilot_updates=block_size)
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
        assert f.budgets==[block_size,block_size,block_size//2,block_size,block_size]
    else:
        assert done and f.budgets==[block_size]*4
    assert f.labels==[f'C3-block{i:02}' for i in range(1,5)]
    assert len(f.state['results'])==4
    final=ctl.read(run/'resume.pt')['schedule']
    assert final['transitions']==257497+block_size*4 and final['updates']==64000+block_size
    assert all(not g['passed'] and g['stop'] for g in f.state['results'].values())
