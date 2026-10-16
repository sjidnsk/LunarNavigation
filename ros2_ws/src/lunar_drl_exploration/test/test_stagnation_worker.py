import multiprocessing as mp

from lunar_drl_exploration.config import TrainingConfig, config_record
from lunar_drl_exploration.replay import loads_transport
from lunar_drl_exploration.worker import worker_main
from test_collector import receive


def test_training_worker_admits_truncation_before_requesting_reset():
    parent, child = mp.get_context('spawn').Pipe()
    process = mp.get_context('spawn').Process(target=worker_main,
        args=(child, config_record(TrainingConfig()), 0),
        kwargs={'env_factory': 'worker_fixtures:TrainingModeEnv'})
    process.start()
    child.close()
    try:
        receive(parent, 'HELLO')
        parent.send(dict(kind='RESET', spec=dict(seed=1, family='moon', extent=40, episode_budget=1)))
        receive(parent, 'SCENE')
        receive(parent, 'READY')
        parent.send(dict(kind='STEP', token=1, action=0, actor_version=7))
        result = receive(parent, 'DONE')
        transition = loads_transport(result['payload'])
        assert transition.truncated and not transition.terminated
        assert result['reset'] and transition.next_observation is not None
        assert not parent.poll(.2), 'reset must wait for replay admission ACK'
        parent.send(dict(kind='ACK', token=1))
        receive(parent, 'NEED_RESET')
        parent.send(dict(kind='STOP'))
        receive(parent, 'CLOSED')
        process.join(3)
        assert process.exitcode == 0
    finally:
        if process.is_alive():
            process.terminate()
        process.join(3)
        parent.close()
