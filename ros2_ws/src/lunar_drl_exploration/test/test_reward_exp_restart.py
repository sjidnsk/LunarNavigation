"""Exponential shaping, scaled exploration and fresh-run reward identity."""
from dataclasses import replace
import math
from types import SimpleNamespace

import numpy as np
import pytest

from lunar_drl_exploration.approach_potential import potential, shaping_reward
from lunar_drl_exploration.config import TrainingConfig, config_record, resume_semantics, training_config_from_record
from lunar_drl_exploration.region_relation_run import make_config
from lunar_drl_exploration.ros_env import make_transition
from lunar_drl_exploration.reward_diagnostics import RewardDiagnostics

SPEC = dict(mode='bounded_exponential_v1', bound=2., scale_m=50.)


def test_exponential_signal_remains_useful_at_boundary_and_bounded_far_away():
    assert potential(0., SPEC) == 0.
    assert shaping_reward(1., 0., SPEC) == pytest.approx(.0396026533864894)
    assert shaping_reward(50., 40., SPEC) == pytest.approx(.1628990458915586)
    assert potential(1e200, SPEC) == -2.
    assert potential(1e-12, SPEC) == pytest.approx(-4e-14, rel=1e-12, abs=0.)


def test_exponential_roundtrip_terminal_and_discounted_telescoping():
    route = [50., 60., 20., 1., 50.]
    assert sum(shaping_reward(a, b, SPEC) for a, b in zip(route, route[1:])) == pytest.approx(0.)
    assert shaping_reward(50., 40., SPEC, terminated=True) == pytest.approx(-potential(50., SPEC))
    assert shaping_reward(50., 40., SPEC, terminated=False) != pytest.approx(-potential(50., SPEC))
    gamma = .97
    rewards = [shaping_reward(a, b, SPEC, gamma=gamma) for a, b in zip(route, route[1:])]
    assert sum(gamma**i*r for i, r in enumerate(rewards)) == pytest.approx(gamma**4*potential(50., SPEC)-potential(50., SPEC))


def test_fresh_config_freezes_new_reward_and_rejects_ordinary_old_reward_resume(tmp_path):
    c = make_config(tmp_path, 'a'*64)
    assert c.approach_reward == SPEC
    assert c.exploration_reward_scale == .5
    assert c.completion_bonus == 20.
    assert training_config_from_record(config_record(c)) == c
    assert resume_semantics(c) != resume_semantics(replace(c, exploration_reward_scale=1.))
    assert resume_semantics(c) != resume_semantics(replace(c, approach_reward=dict(SPEC, mode='bounded_quadratic_v1')))
    assert 'exploration_reward_scale' not in resume_semantics(TrainingConfig())


@pytest.mark.parametrize('scale', [0., -1., float('nan'), float('inf'), True])
def test_invalid_exploration_scale_rejected(scale):
    with pytest.raises(ValueError):
        TrainingConfig(exploration_reward_scale=scale)


@pytest.mark.parametrize('completed,exhausted,budget', [(False,False,False),(False,False,True),(True,False,False),(False,True,False)])
def test_actual_transition_and_diagnostics_share_scaled_reward(completed, exhausted, budget):
    polygon = np.array([[0.,0.],[10.,0.],[10.,10.],[0.,10.]])
    before = SimpleNamespace(polygon=(polygon+[10.,-5.])/10.)
    after = SimpleNamespace(polygon=(polygon+[1.,-5.])/10.)
    t = make_transition(before,0,after,None,None,initial=(0.,0.,0.),final=(3.,9.,math.pi),
        removed_frontiers_in=50,removed_frontiers_out=500,completed=completed,exhausted=exhausted,
        budget_hit=budget,episode_id='fresh',actor_version=0,completion_bonus=20.,
        approach_reward=SPEC,exploration_reward_scale=.1)
    expected = .1-.018-.005-.001+(20. if completed else 0.)+shaping_reward(10.,1.,SPEC,terminated=completed or exhausted)
    assert t.reward == pytest.approx(expected)
    d = RewardDiagnostics(1,completion_bonus=20.,exploration_reward_scale=.1)
    d.observe(0,t,before_coverage=.9,after_coverage=.995 if completed else .95,family='moon',extent_m=50.)
    totals = d.summary()['totals']
    assert totals['reward_components']['exploration'] == pytest.approx(.1)
    assert totals['reward_components']['completion'] == (20. if completed else 0.)
    assert totals['reward_mismatch_count'] == 0
    restored = RewardDiagnostics(1,completion_bonus=20.,exploration_reward_scale=.1,state=d.state_dict())
    assert restored.summary()['totals']['reward_sum'] == pytest.approx(expected)
    with pytest.raises(ValueError):
        RewardDiagnostics(1,completion_bonus=20.,state=d.state_dict())


def test_scaled_reward_success_index_and_single_admission_copy():
    from lunar_drl_exploration.replay import ReplayBuffer
    from test_course_two_replay import add_episode
    original = ReplayBuffer()
    t = add_episode(original,'c2_A1_finish','scaled',count=1,covered=95,complete=True)[0]
    t = replace(t,parts=replace(t.parts,removed_frontiers_in=100),reward=20.199)
    replay = ReplayBuffer()
    index = replay.enable_course_two_sampling(exploration_reward_scale=.1)
    replay.add(t,original.scenes)
    assert len(index.success_indices) == 1
    with pytest.raises(ValueError):
        replay.enable_course_two_sampling(exploration_reward_scale=1.)


def test_full_checkpoint_rejects_changed_reward_and_accepts_new_objective():
    import torch
    from lunar_drl_exploration.config import ModelConfig
    from lunar_drl_exploration.sac import SACLearner
    from lunar_drl_exploration.replay import ReplayBuffer
    from lunar_drl_exploration.schedule import UpdateSchedule
    from lunar_drl_exploration.checkpoint import UpdateBoundary
    from test_replay_checkpoint import capture_state, transition, scene
    c=TrainingConfig(model=ModelConfig(width=16,heads=2,layers=1),
        approach_reward=SPEC,exploration_reward_scale=.1)
    torch.manual_seed(123)
    learner=SACLearner(c.model,c.learning)
    replay=ReplayBuffer()
    replay.add(replace(transition(),reward=.009),{'s':scene()})
    schedule=UpdateSchedule(c.warmup,c.update_ratio,c.max_update_credit)
    schedule.collected()
    state=capture_state(learner,c,replay,schedule,UpdateBoundary())
    state.validate(c)
    for incompatible in (replace(c,exploration_reward_scale=1.),
            replace(c,approach_reward=dict(SPEC,mode='bounded_quadratic_v1'))):
        with pytest.raises(ValueError,match='incompatible full resume semantics'):
            state.validate(incompatible)
