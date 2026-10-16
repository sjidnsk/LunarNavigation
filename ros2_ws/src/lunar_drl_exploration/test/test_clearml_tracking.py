import json
from types import SimpleNamespace
import pytest
from lunar_drl_exploration.clearml_tracking import ClearMLTracking


class FakeTask:
    calls = []
    @classmethod
    def get_tasks(cls, **kwargs):
        return []
    @classmethod
    def init(cls, **kwargs):
        cls.calls.append(kwargs)
        return SimpleNamespace(id=kwargs['reuse_last_task_id'] or 'one-task',
            connect=lambda *a, **k: None, close=lambda: None,
            mark_completed=lambda *a,**k:None,mark_failed=lambda **k:None,mark_stopped=lambda **k:None,
            get_logger=lambda: SimpleNamespace(report_scalar=lambda **k: None))


def test_fresh_reward_run_has_separate_durable_task_name(tmp_path):
    protocol = {'clearml': {'task_name': 'wheeled-region-reward-exp-fresh-20261014'}}
    FakeTask.calls.clear()
    tracker = ClearMLTracking.open(tmp_path, enabled=True, task_class=FakeTask, protocol=protocol)
    tracker.close()
    assert FakeTask.calls[-1]['task_name'] == protocol['clearml']['task_name']
    with pytest.raises(ValueError, match='foreign'):
        ClearMLTracking.open(tmp_path, enabled=True, task_class=FakeTask)
    tracker = ClearMLTracking.open(tmp_path, enabled=True, task_class=FakeTask, protocol=protocol)
    tracker.close()
    assert FakeTask.calls[-1]['continue_last_task'] is True


def test_disabled_has_no_sdk_or_files(tmp_path):
    tracker = ClearMLTracking.open(tmp_path / 'absent')
    tracker.report_metrics({'value': 1}, iteration=1)
    assert not (tmp_path / 'absent').exists()


def test_one_task_and_durable_resume(tmp_path):
    FakeTask.calls.clear()
    tracker = ClearMLTracking.open(tmp_path, enabled=True, task_class=FakeTask, config={'seed':20261014})
    with pytest.raises(BlockingIOError):
        ClearMLTracking.open(tmp_path, enabled=True, task_class=FakeTask)
    tracker.close()
    resumed = ClearMLTracking.open(tmp_path, enabled=True, task_class=FakeTask)
    resumed.close()
    assert FakeTask.calls[0]['reuse_last_task_id'] is False
    assert FakeTask.calls[1]['reuse_last_task_id'] == 'one-task'
    assert FakeTask.calls[1]['continue_last_task'] is True
    assert all(c['auto_connect_frameworks'] is False and c['output_uri'] is False for c in FakeTask.calls)


def test_uncertain_creation_fails_closed(tmp_path):
    class Failing(FakeTask):
        @staticmethod
        def init(**kwargs):
            raise RuntimeError('interrupted')
    with pytest.raises(RuntimeError, match='interrupted'):
        ClearMLTracking.open(tmp_path, enabled=True, task_class=Failing)
    assert json.loads((tmp_path/'clearml-task.json').read_text())['task_id'] is None
    with pytest.raises(RuntimeError, match='unresolved'):
        ClearMLTracking.open(tmp_path, enabled=True, task_class=FakeTask)


def test_remote_task_prevents_duplicate_creation(tmp_path):
    class Existing(FakeTask):
        @classmethod
        def get_tasks(cls, **kwargs):
            return [SimpleNamespace(id='existing')]
    with pytest.raises(RuntimeError, match='already exists'):
        ClearMLTracking.open(tmp_path, enabled=True, task_class=Existing)
    assert not (tmp_path/'clearml-task.json').exists()


@pytest.mark.parametrize('outcome,expected',[('completed','completed'),('stopped','stopped'),
    ('paused','stopped'),('failed','failed'),('needs_attention','failed')])
def test_close_preserves_requested_remote_outcome(outcome,expected):
    class Task:
        def __init__(self):self.calls=[];self.status='running'
        def mark_completed(self):self.calls.append(('completed',{}));self.status='completed'
        def mark_failed(self,**kwargs):self.calls.append(('failed',kwargs));self.status='failed'
        def mark_stopped(self,**kwargs):self.calls.append(('stopped',kwargs));self.status='stopped'
        def close(self):self.mark_completed()
    task=Task();original=task.mark_completed
    tracker=ClearMLTracking(task)
    tracker.close(outcome=outcome,message='reason')
    assert task.status==expected
    assert task.calls[-1][0]==expected
    assert task.mark_completed==original
    if expected=='failed':assert task.calls[-1][1]['ignore_errors'] is False
    if expected!='completed':assert task.calls[-1][1]['force'] is True
    tracker.close(outcome=outcome)
    assert tracker.task is None


def test_default_close_is_stopped_and_invalid_outcome_refused():
    events=[]
    task=SimpleNamespace(mark_completed=lambda:events.append('completed'),
        mark_stopped=lambda **k:events.append('stopped'),close=lambda:None)
    tracker=ClearMLTracking(task)
    with pytest.raises(ValueError):tracker.close(outcome='unknown')
    assert not events
    tracker.close()
    assert events==['stopped']
