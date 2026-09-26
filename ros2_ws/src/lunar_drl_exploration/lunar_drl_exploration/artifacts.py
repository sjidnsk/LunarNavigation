"""Cooperative, process-locked output budgets shared by one experiment stage.

Every regular file below the root is counted, including old files and temporary
replacements. Only ``.pt`` model/checkpoint files are outside the diagnostic
sub-budget. Writers must use this module; arbitrary external writes are observed
at the next operation, not intercepted. Limits are immutable once initialized.
"""
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import shutil
import stat
import tempfile


MANIFEST = '.artifact-budget.json'
LOCK = '.artifact-budget.lock'
DIAGNOSTIC_MAX_BYTES = 128 * 1024**2


class ResourceLimitError(RuntimeError):
    def __init__(self, reason, observed, limit):
        self.reason, self.observed, self.limit = reason, observed, limit
        super().__init__(f'{reason}: observed={observed}, limit={limit}')


def _safe_path(path):
    path = Path(path)
    if '..' in path.parts:
        raise ValueError('artifact path cannot contain parent traversal')
    path = Path(os.path.abspath(path))
    for parent in (*reversed(path.parents), path):
        if parent.is_symlink():
            raise ValueError(f'artifact symlink is not allowed: {parent}')
    return path


def _scan(root):
    total = diagnostic = 0
    for directory, dirs, files in os.walk(root, followlinks=False):
        for name in (*dirs, *files):
            path = Path(directory) / name
            info = path.lstat()
            if stat.S_ISLNK(info.st_mode):
                raise ValueError(f'artifact symlink is not allowed: {path}')
            if stat.S_ISDIR(info.st_mode):
                continue
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise ValueError(f'artifact must be a regular, unlinked file: {path}')
            total += info.st_size
            if path.suffix != '.pt':
                diagnostic += info.st_size
    return total, diagnostic


def artifact_bytes(path):
    return _scan(_safe_path(path))[0]


def _find_root(path):
    path = _safe_path(path)
    found = [parent for parent in (path, *path.parents)
             if any((parent / name).exists() or (parent / name).is_symlink()
                    for name in (MANIFEST, LOCK))]
    if len(found) > 1:
        raise ValueError('nested artifact roots are not allowed')
    return found[0] if found else None


@contextmanager
def _root_lock(root):
    _safe_path(root)
    descriptor = os.open(root / LOCK, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
    try:
        fcntl.flock(descriptor, fcntl.LOCK_EX)
        _safe_path(root / LOCK)
        yield
    finally:
        fcntl.flock(descriptor, fcntl.LOCK_UN)
        os.close(descriptor)


def _manifest(root):
    path = _safe_path(root / MANIFEST)
    record = json.loads(path.read_text(encoding='utf-8'))
    if (record.get('schema') != 'drl_artifact_budget_v1' or
            record.get('root') != str(root) or
            any(type(record.get(key)) is not int or record[key] <= 0
                for key in ('total_max_bytes', 'diagnostic_max_bytes'))):
        raise ValueError('invalid artifact budget manifest')
    return record


def initialize_artifact_root(root, total_max_bytes,
                             diagnostic_max_bytes=DIAGNOSTIC_MAX_BYTES):
    """Create an empty stage once, or verify exactly the existing limits."""
    if any(type(value) is not int or value <= 0
           for value in (total_max_bytes, diagnostic_max_bytes)):
        raise ValueError('positive integer artifact budgets required')
    root = _safe_path(root)
    ancestor = _find_root(root)
    if ancestor is not None and ancestor != root:
        raise ValueError('nested artifact roots are not allowed')
    root.mkdir(parents=True, exist_ok=True)
    with _root_lock(root):
        if (root / MANIFEST).exists():
            record = _manifest(root)
            if (record['total_max_bytes'] != total_max_bytes or
                    record['diagnostic_max_bytes'] != diagnostic_max_bytes):
                raise ValueError('artifact root budgets are immutable')
            _scan(root)
            return root
        if any(path.name != LOCK for path in root.iterdir()):
            raise ValueError('new artifact root must be empty; refusing budget reset')
        record = dict(schema='drl_artifact_budget_v1', root=str(root),
            total_max_bytes=total_max_bytes, diagnostic_max_bytes=diagnostic_max_bytes)
        payload = (json.dumps(record, sort_keys=True) + '\n').encode()
        _check(len(payload), len(payload), record, 'peak')
        with (root / MANIFEST).open('xb') as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    return root


def _check(total, diagnostic, record, phase):
    if total > record['total_max_bytes']:
        raise ResourceLimitError(f'artifact {phase} bytes', total, record['total_max_bytes'])
    if diagnostic > record['diagnostic_max_bytes']:
        raise ResourceLimitError(f'diagnostic {phase} bytes', diagnostic, record['diagnostic_max_bytes'])


@contextmanager
def _budget(path, local_total_max_bytes):
    path = _safe_path(path)
    if path.name in (MANIFEST, LOCK):
        raise ValueError('artifact budget control names are reserved')
    root = _find_root(path.parent)
    if root is None:
        path.parent.mkdir(parents=True, exist_ok=True)
        yield path, dict(total_max_bytes=local_total_max_bytes,
            diagnostic_max_bytes=local_total_max_bytes), artifact_bytes(path.parent), 0, False
        return
    with _root_lock(root):
        path = _safe_path(path)
        path.relative_to(root)
        path.parent.mkdir(parents=True, exist_ok=True)
        record = _manifest(root)
        total, diagnostic = _scan(root)
        _check(total, diagnostic, record, 'existing')
        yield path, record, total, diagnostic, True


def artifact_usage(path):
    """Read current shared consumption under the same lock as every writer."""
    root = _find_root(path)
    if root is None:
        raise ValueError('no artifact budget manifest found')
    with _root_lock(root):
        record = _manifest(root)
        total, diagnostic = _scan(root)
        return dict(root=str(root), total_bytes=total, diagnostic_bytes=diagnostic,
            total_max_bytes=record['total_max_bytes'],
            diagnostic_max_bytes=record['diagnostic_max_bytes'])


class BudgetWriter:
    """Check old plus temporary high-water size before each physical write."""
    def __init__(self, stream, existing, limit, free, *, diagnostic_existing=0,
                 diagnostic_limit=None, diagnostic=False, max_bytes=None):
        self.stream, self.existing, self.limit, self.free = stream, existing, limit, free
        self.diagnostic_existing = diagnostic_existing
        self.diagnostic_limit = diagnostic_limit
        self.diagnostic = diagnostic
        self.max_bytes = max_bytes
        self.extent = 0
        self.failure = None

    def write(self, data):
        if self.failure is not None:
            raise self.failure
        extent = max(self.extent, self.stream.tell() + len(data))
        checks = [('artifact peak bytes', self.existing + extent, self.limit),
                  ('disk temporary bytes', extent, self.free)]
        if self.max_bytes is not None:
            checks.append(('artifact file bytes', extent, self.max_bytes))
        if self.diagnostic and self.diagnostic_limit is not None:
            checks.append(('diagnostic peak bytes', self.diagnostic_existing + extent,
                           self.diagnostic_limit))
        for reason, observed, limit in checks:
            if observed > limit:
                self.failure = ResourceLimitError(reason, observed, limit)
                raise self.failure
        result = self.stream.write(data)
        self.extent = extent
        return result

    def __getattr__(self, name):
        return getattr(self.stream, name)


def atomic_write(path, write, *, max_bytes=None, diagnostic=True,
                 local_total_max_bytes=20 * 1024**3):
    """Replace a file only when its entire old+new peak fits the shared root."""
    if not diagnostic and Path(path).suffix != '.pt':
        raise ValueError('only .pt model files may bypass the diagnostic sub-budget')
    with _budget(path, local_total_max_bytes) as (path, limits, total, diag, shared):
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w+b', dir=path.parent,
                    prefix='.' + path.name + '.',
                    suffix='.tmp.pt' if path.suffix == '.pt' else '.tmp', delete=False) as stream:
                temporary = Path(stream.name)
                writer = BudgetWriter(stream, total, limits['total_max_bytes'],
                    shutil.disk_usage(path.parent).free, diagnostic_existing=diag,
                    diagnostic_limit=limits['diagnostic_max_bytes'],
                    diagnostic=shared and path.suffix != '.pt', max_bytes=max_bytes)
                try:
                    write(writer)
                    if writer.failure is not None:
                        raise writer.failure
                except BaseException:
                    if writer.failure is not None:
                        raise writer.failure
                    raise
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
            descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()


def atomic_json(path, value, max_bytes=1024**2):
    payload = (json.dumps(value, ensure_ascii=False, allow_nan=False,
                          separators=(',', ':')) + '\n').encode('utf-8')
    atomic_write(path, lambda stream: stream.write(payload), max_bytes=max_bytes)


def append_bounded(path, data, max_bytes, *, whole_lines=False,
                   local_total_max_bytes=20 * 1024**3):
    """Append/compact one rolling file in-place while holding the root lock."""
    if not isinstance(data, bytes) or max_bytes <= 0:
        raise ValueError('bounded append requires bytes and a positive cap')
    if whole_lines and (len(data) > max_bytes or not data.endswith(b'\n')):
        raise ValueError('whole records must fit the byte budget and end with a newline')
    with _budget(path, local_total_max_bytes) as (path, limits, total, diag, shared):
        size = path.stat().st_size if path.exists() else 0
        if size + len(data) > max_bytes:
            keep = max(0, max_bytes - len(data))
            prefix = b''
            if keep and size:
                with path.open('rb') as stream:
                    offset = max(0, size - keep)
                    if whole_lines and offset:
                        stream.seek(offset - 1)
                        if stream.read(1) != b'\n':
                            stream.readline()
                    else:
                        stream.seek(offset)
                    prefix = stream.read(keep)
            output = prefix + data[-max_bytes:]
            mode = 'wb'
            increase = len(output) - size
        else:
            output, mode, increase = data, 'ab', len(data)
        _check(total + increase, diag + increase if shared else 0, limits, 'peak')
        with path.open(mode) as stream:
            stream.write(output)
