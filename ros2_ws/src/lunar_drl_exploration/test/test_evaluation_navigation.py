import json
import pytest


def test_explicit_follower_configuration_reaches_controller_parameters(tmp_path):
    from lunar_drl_exploration.evaluation_navigation import load_evaluation_navigation
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    settings = dict(controller_backend='mppi_path_follower', simulation_step_sync=True,
                    mppi_nominal_response_time_s=0.)
    (tmp_path/'evaluation-navigation.json').write_text(json.dumps(settings))
    options = load_evaluation_navigation(tmp_path)
    env = RosExplorationEnv(TrainingConfig(), 0, **options)
    params = env._controller_parameters()
    assert params['controller_backend'] == 'mppi_path_follower'
    assert params['mppi_simulation_step_sync'] is True
    assert params['mppi_nominal_response_time_s'] == 0.
    assert params['mppi_step_ack_topic'] == env.namespace+'/mppi_step_ack'


def test_legacy_evaluation_and_simulation_defaults_are_preserved(tmp_path):
    from lunar_drl_exploration.evaluation_navigation import load_evaluation_navigation
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    env = RosExplorationEnv(TrainingConfig(), 0, **load_evaluation_navigation(tmp_path))
    assert env._controller_parameters() == {}


def test_mppi_manifest_without_explicit_wiring_cannot_silently_use_legacy(tmp_path):
    from lunar_drl_exploration.evaluation_navigation import load_evaluation_navigation
    (tmp_path/'comparison-protocol.json').write_text(json.dumps(dict(controller_backend='mppi_path_follower')))
    with pytest.raises(ValueError, match='explicit'):
        load_evaluation_navigation(tmp_path)


@pytest.mark.parametrize('change', [dict(controller_backend='typo'),
    dict(simulation_step_sync='true'), dict(mppi_nominal_response_time_s=-.1),
    dict(mppi_nominal_response_time_s=float('nan')), dict(unknown=1),
    dict(controller_backend='pure_pursuit')])
def test_invalid_evaluation_navigation_is_rejected(tmp_path, change):
    from lunar_drl_exploration.evaluation_navigation import load_evaluation_navigation
    settings = dict(controller_backend='mppi_path_follower', simulation_step_sync=True,
                    mppi_nominal_response_time_s=0.)
    settings.update(change)
    (tmp_path/'evaluation-navigation.json').write_text(json.dumps(settings))
    with pytest.raises(ValueError):
        load_evaluation_navigation(tmp_path)


def test_backend_selection_does_not_require_simulation_lockstep(tmp_path):
    from lunar_drl_exploration.evaluation_navigation import load_evaluation_navigation
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    settings=dict(controller_backend='mppi_path_follower', simulation_step_sync=False,
                  mppi_nominal_response_time_s=.15)
    (tmp_path/'evaluation-navigation.json').write_text(json.dumps(settings))
    env=RosExplorationEnv(TrainingConfig(), 0, **load_evaluation_navigation(tmp_path))
    assert env._controller_parameters()['controller_backend']=='mppi_path_follower'
    assert env._controller_parameters()['mppi_nominal_response_time_s']==.15
    assert 'mppi_simulation_step_sync' not in env._controller_parameters()


def test_compute_diagnostics_are_passive_and_malformed_messages_do_not_abort():
    from types import SimpleNamespace
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.ros_env import RosExplorationEnv
    env=RosExplorationEnv(TrainingConfig(),0,controller_backend='mppi_path_follower')
    for data in ['invalid','[]',json.dumps(dict(event='tick'))]:
        env._on_mppi_diagnostics(SimpleNamespace(data=data))
    event=dict(event='solve_result',compute_ms=41.,deadline_missed=True)
    env._on_mppi_diagnostics(SimpleNamespace(data=json.dumps(event)))
    assert env._mppi_compute_ms==[41.]
    assert env._mppi_deadline_events==[event]
