"""Optional, explicit tracking for the one approved experiment (no import side effects)."""
import fcntl
import json
import math
import os
import re
from pathlib import Path

PROJECT = '训练项目'
TASK_NAME = 'wheeled-region_relation_v2-from-scratch-seed-20261014'


class ClearMLTracking:
    """Hold a local lease; persist creation intent before contacting ClearML.

    A crash between remote creation and local id persistence fails closed. Recover
    by explicitly restoring the remote task id into the marker after read-only
    inspection; never silently issue a second Task.init creation.
    """
    def __init__(self, task=None, lease=None):
        self.task = task
        self._lease = lease

    @classmethod
    def open(cls, directory, *, enabled=False, config=None, protocol=None,
             identity=None, task_class=None):
        if not enabled:
            return cls()
        if task_class is None:
            from clearml import Task
            task_class = Task
        root = Path(directory)
        task_name = (protocol or {}).get('clearml', {}).get('task_name', TASK_NAME)
        if not isinstance(task_name, str) or not task_name.strip() or len(task_name)>256:
            raise ValueError('invalid ClearML task name')
        root.mkdir(parents=True, exist_ok=True)
        lease = os.open(root / 'clearml.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            marker = root / 'clearml-task.json'
            if marker.is_symlink():
                raise ValueError('ClearML marker cannot be a symlink')
            state = json.loads(marker.read_text()) if marker.exists() else None
            if state is not None:
                if state.get('project_name') != PROJECT or state.get('task_name') != task_name:
                    raise ValueError('foreign ClearML task identity')
                if not state.get('task_id'):
                    raise RuntimeError('unresolved ClearML creation intent; inspect remote task before recovery')
            else:
                existing = task_class.get_tasks(project_name=PROJECT, task_name="^" + re.escape(task_name) + "$")
                if existing:
                    raise RuntimeError("remote experiment task already exists; restore its durable marker explicitly")
                state = dict(project_name=PROJECT, task_name=task_name, task_id=None)
                _persist(marker, state)
            task = task_class.init(project_name=PROJECT, task_name=task_name,
                reuse_last_task_id=state['task_id'] or False,
                continue_last_task=bool(state['task_id']),
                auto_connect_frameworks=False, auto_connect_arg_parser=False,
                auto_connect_streams=False, auto_resource_monitoring=False,
                output_uri=False)
            if state['task_id'] and task.id != state['task_id']:
                raise RuntimeError('ClearML returned a different task id')
            if not task.id:
                raise RuntimeError('ClearML task has no durable id')
            state['task_id'] = task.id
            _persist(marker, state)
            for name, data in (('RunConfig', config), ('Protocol', protocol), ('Identity', identity)):
                if data is not None:
                    task.connect(dict(data), name=name, ignore_remote_overrides=True)
            return cls(task, lease)
        except BaseException:
            os.close(lease)
            raise

    def report_metrics(self, metrics, *, iteration, title='training'):
        if self.task is None:
            return
        logger = self.task.get_logger()
        for name, value in metrics.items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                if not math.isfinite(value):
                    raise ValueError('nonfinite ClearML metric: ' + name)
                logger.report_scalar(title=title, series=name, value=float(value), iteration=int(iteration))

    def upload_final_actor(self, path):
        """Explicit selected Actor only; no automatic checkpoint/replay uploads."""
        if self.task is not None:
            self.task.upload_artifact(name='final_selected_actor', artifact_object=str(Path(path)))

    def close(self, *, outcome='stopped', message=None):
        """Close SDK resources then explicitly preserve the controller outcome.

        The installed SDK close chooses its status from exit hooks, independent
        of an earlier mark_failed/mark_stopped call. Apply the forced final
        status after cleanup through documented public methods.
        """
        aliases={'complete':'completed','paused':'stopped','needs_attention':'failed'}
        outcome=aliases.get(outcome,outcome)
        if outcome not in ('completed','stopped','failed'):
            raise ValueError('unknown ClearML close outcome')
        try:
            if self.task is not None:
                task=self.task
                try:
                    task.close()
                finally:
                    if outcome=='failed':
                        task.mark_failed(ignore_errors=False,force=True,
                            status_reason='training_run_failed',status_message=message)
                    elif outcome=='stopped':
                        task.mark_stopped(force=True,status_message=message)
                self.task=None
        finally:
            if self._lease is not None:
                os.close(self._lease)
                self._lease = None


def _persist(path, state):
    temporary = path.with_suffix('.tmp')
    fd = os.open(temporary, os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'w', encoding='utf-8') as stream:
        json.dump(state, stream, ensure_ascii=False, sort_keys=True)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)
