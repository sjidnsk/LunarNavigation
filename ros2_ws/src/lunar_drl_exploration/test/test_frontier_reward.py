"""Whole-action counts are rewards; square metres remain diagnostics only."""
import math
from dataclasses import replace
import numpy as np
import pytest
from lunar_drl_exploration.contracts import RewardParts
from lunar_drl_exploration.ros_env import make_transition


def transition(*, inside=3, outside=7, area=100., completed=False):
    from test_model import observation, state
    obs = observation(); priv = state(obs=obs)
    return make_transition(obs, 0, obs, priv, priv, initial=(0., 0., 0.),
        final=(area, 10., math.pi), removed_frontiers_in=inside,
        removed_frontiers_out=outside, completed=completed, budget_hit=False,
        episode_id='frontiers', actor_version=0, completion_bonus=5.)


def test_reward_uses_integer_inside_removals_and_whole_action_costs():
    t = transition()
    assert t.reward == pytest.approx(3/50 - .02 - .005 - .001)
    assert t.parts == RewardParts(100., 10., math.pi, 3, 7)
    assert transition(area=0.).reward == t.reward
    assert transition(outside=700).reward == t.reward


def test_outside_only_has_no_positive_exploration_term_and_success_adds_five():
    t = transition(inside=0)
    assert t.reward == pytest.approx(-.026)
    assert transition(inside=0, completed=True).reward == pytest.approx(t.reward+5.)
    assert transition(completed=True).terminated


@pytest.mark.parametrize('bad', [-1, 1.5, True, np.nan])
@pytest.mark.parametrize('field', ['removed_frontiers_in', 'removed_frontiers_out'])
def test_reward_parts_reject_non_counts(field, bad):
    fields = dict(new_area_m2=0., distance_m=0., turn_rad=0.,
                  removed_frontiers_in=0, removed_frontiers_out=0)
    fields[field] = bad
    with pytest.raises(ValueError, match='nonnegative integer'):
        RewardParts(**fields)


def test_step_freezes_before_execution_counts_after_ack_and_bonus_only_once(monkeypatch):
    from test_ros_env import stagnation_env
    from test_task_analysis import snapshot
    from lunar_drl_exploration.frontiers import count_removed
    env, transport = stagnation_env(monkeypatch)
    old = env.core.frontiers
    calls = []
    real_snapshot = env._snapshot
    def execute(goal):
        calls.append('execute')
        env.plant.distance_m += 10.
        env.plant.turn_rad += math.pi
        known = np.ones((12, 12), np.uint8); known[5, 9] = 0
        transport['snapshot'] = replace(snapshot(np.pad(np.ones((6,6),np.uint8),3),
            b=known, pose=env.plant.pose), revision=2)
        # Asynchronous map consumption can advance frontiers during navigation.
        env.core.observe(transport['snapshot'])
        from lunar_drl_exploration.ros_messages import NavigationResult
        return NavigationResult(0, 'GOAL_REACHED', 0)
    def ack():
        calls.append('final_ack')
        return real_snapshot()
    monkeypatch.setattr(env, '_execute_goal', execute)
    monkeypatch.setattr(env, '_snapshot', ack)
    t = env.step(0)
    expected = count_removed(old, env.core.frontiers, domain='inside')
    assert expected > 0 and t.parts.removed_frontiers_in == expected
    assert calls == ['execute', 'final_ack']
    assert t.parts.new_area_m2 == 107.
    assert t.reward == pytest.approx(expected/50 - .02 - .005 - .001 + 5.)
    assert t.terminated and env.completed
    with pytest.raises(RuntimeError, match='requires reset'):
        env.step(0)


def test_step_outside_removals_are_diagnostic_and_next_action_cannot_reaward(monkeypatch):
    from test_ros_env import stagnation_env
    from test_task_analysis import snapshot, task
    from lunar_drl_exploration.frontiers import count_removed
    env, transport = stagnation_env(monkeypatch)
    env.reset_scene(env.terrain, env.plant.pose, task(0, 0, 6, 6))
    before = env.core.frontiers
    observed = np.ones((12,12), np.uint8)
    observed[:6,:6] = 0; observed[3:6,3:6] = 1
    transport['snapshot'] = replace(snapshot(np.pad(np.ones((6,6),np.uint8),3),
        b=observed, pose=env.plant.pose), revision=2)
    first = env.step(0)
    assert first.parts.removed_frontiers_out == count_removed(before, env.core.frontiers, domain='outside') > 0
    assert first.parts.removed_frontiers_in == 0 and first.parts.new_area_m2 == 0
    assert first.reward == -.001
    second = env.step(0)
    assert second.parts.removed_frontiers_in == second.parts.removed_frontiers_out == 0
    assert second.reward == -.001
