import numpy as np
import pytest
from lunar_drl_exploration.approach_potential import potential, distance_to_region, shaping_reward

SPEC={'bound':2.,'scale_m':50.,'mode':'bounded_quadratic_v1'}
POLY=np.array([[0,0],[10,0],[10,10],[0,10]],float)

def test_distance_inside_edge_corner():
    assert distance_to_region((5,5),POLY)==0
    assert distance_to_region((0,5),POLY)==0
    assert distance_to_region((-3,5),POLY)==3
    assert distance_to_region((-3,-4),POLY)==5

def test_bound_far_inside_and_roundtrip():
    assert potential(0,SPEC)==0
    assert potential(50,SPEC)==-1
    assert potential(100,SPEC)==pytest.approx(-1.6)
    assert -2<=potential(1e200,SPEC)<=0
    assert shaping_reward(50,40,SPEC)==pytest.approx(.21951219512)
    assert sum(shaping_reward(a,b,SPEC) for a,b in [(50,40),(40,10),(10,60),(60,50)])==pytest.approx(0)
    assert shaping_reward(0,0,SPEC)==0

def test_termination_correction_and_truncation_bootstrap():
    assert shaping_reward(50,40,SPEC,terminated=True)==1
    assert shaping_reward(50,40,SPEC,terminated=False)==pytest.approx(.21951219512)
    assert sum(shaping_reward(a,b,SPEC,terminated=t) for a,b,t in [(50,40,False),(40,60,True)])==pytest.approx(1)

@pytest.mark.parametrize('d',[-1,float('nan'),float('inf')])
def test_invalid_distance(d):
    with pytest.raises(ValueError):potential(d,SPEC)

def test_actual_observation_transition_and_budget():
    from types import SimpleNamespace
    from lunar_drl_exploration.ros_env import make_transition
    before=SimpleNamespace(polygon=(POLY+np.array([50.,-5.]))/10.)
    after=SimpleNamespace(polygon=(POLY+np.array([40.,-5.]))/10.)
    kw=dict(initial=(0,0,0),final=(0,0,0),removed_frontiers_in=0,removed_frontiers_out=0,
            completed=False,budget_hit=True,episode_id='e',actor_version=0,approach_reward=SPEC)
    t=make_transition(before,0,after,None,None,**kw)
    assert t.truncated and not t.terminated
    assert t.parts.approach_reward==pytest.approx(.21951219512)
    assert t.reward==pytest.approx(t.parts.approach_reward-.001)
    kw.update(completed=True,completion_bonus=20.)
    t=make_transition(before,0,after,None,None,**kw)
    assert t.terminated and not t.truncated
    assert t.parts.approach_reward==1
    assert t.reward==pytest.approx(20.999)

def test_config_reward_resume_identity():
    from dataclasses import replace
    from lunar_drl_exploration.config import TrainingConfig,resume_semantics,config_record,training_config_from_record
    cfg=TrainingConfig();new=replace(cfg,approach_reward=SPEC)
    assert 'approach_reward' not in resume_semantics(cfg)
    assert resume_semantics(new)['approach_reward']['bound']==2
    assert training_config_from_record(config_record(new))==new

def test_shaped_diagnostics_roundtrip():
    from test_reward_diagnostics import diagnostics,transition,observe
    d=diagnostics();t=transition();base=t.reward
    t.parts.approach_reward=.36;t.reward+=.36
    observe(d,t)
    assert d.summary()['totals']['reward_mismatch_count']==0
    assert d.summary()['totals']['reward_components']['approach']==pytest.approx(.36)
    restored=diagnostics(state=d.state_dict())
    assert restored.summary()['totals']['reward_sum']==pytest.approx(base+.36)

from test_ordered_extent_fork import ordered_checkpoint

def test_baseline_fork_excludes_source_rewards_and_preserves_learner(ordered_checkpoint,tmp_path,monkeypatch):
    from dataclasses import replace
    from test_course_two_training import completed_source
    from lunar_drl_exploration import course_two_trial as trial,course_two_control as ctl
    from lunar_drl_exploration.training import state_checksum
    from lunar_drl_exploration.replay import ReplayBuffer
    import shutil
    _,source,original=completed_source(ordered_checkpoint,tmp_path)
    old=tmp_path/'old';(old/'frozen').mkdir(parents=True)
    shutil.copyfile(source,old/'frozen/source-resume.pt')
    (old/'frozen/block31.pt').write_bytes(b'actor')
    (old/'frozen/diagnostic_scenes.py').write_text('# retained')
    monkeypatch.setattr(trial,'OLD_ROOT',old);monkeypatch.setattr(trial,'SOURCE_SHA',ctl.sha(source))
    target=tmp_path/'new';trial.prepare(target,bounded_potential=True)
    controller=trial.Trial(target);cfg=controller.initialize_arm('C3')
    new=trial.checkpoint(target/'arms/C3/training/run/resume.pt')
    assert state_checksum(new.record['learner'])==state_checksum(original.record['learner'])
    assert new.record['schedule']==original.record['schedule']
    assert cfg.approach_reward==SPEC
    assert len(ReplayBuffer.from_snapshot(new.record['replay'],cfg.replay_max_bytes))==0
    assert ctl.sha(old/'frozen/source-resume.pt')==ctl.sha(source)
    assert new.record['fork']['reward_replay_reset']
    with pytest.raises(ValueError):new.validate(replace(cfg,approach_reward=None))
