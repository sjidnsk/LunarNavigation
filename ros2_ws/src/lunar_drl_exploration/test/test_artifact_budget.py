"""Shared output limits include replacement peaks across independent runs."""
import json
import multiprocessing
from pathlib import Path

import pytest

from lunar_drl_exploration.artifacts import (
    ResourceLimitError, append_bounded, atomic_json, atomic_write,
    artifact_usage, initialize_artifact_root,
)


def test_two_runs_share_total_budget_and_keep_previous_checkpoint(tmp_path):
    root = tmp_path / 'stage'
    initialize_artifact_root(root, 4096, diagnostic_max_bytes=2048)
    first, second = root / 'one' / 'resume.pt', root / 'two' / 'resume.pt'
    atomic_write(first, lambda stream: stream.write(b'a' * 1500), diagnostic=False)
    atomic_write(second, lambda stream: stream.write(b'b' * 1500), diagnostic=False)
    with pytest.raises(ResourceLimitError, match='artifact peak bytes'):
        atomic_write(first, lambda stream: stream.write(b'c' * 1500), diagnostic=False)
    assert first.read_bytes() == b'a' * 1500
    assert second.read_bytes() == b'b' * 1500
    assert not list(root.rglob('*.tmp'))


def test_diagnostic_limit_is_shared_and_model_bytes_are_separate(tmp_path):
    root = tmp_path / 'stage'
    initialize_artifact_root(root, 10000, diagnostic_max_bytes=1000)
    atomic_write(root / 'resume.pt', lambda stream: stream.write(b'a' * 4000), diagnostic=False)
    atomic_json(root / 'one' / 'summary.json', {'data': 'x' * 350}, max_bytes=500)
    with pytest.raises(ResourceLimitError, match='diagnostic peak bytes'):
        atomic_json(root / 'two' / 'summary.json', {'data': 'y' * 700}, max_bytes=900)
    assert not (root / 'two' / 'summary.json').exists()


def test_root_cannot_be_reinitialized_with_larger_or_smaller_limits(tmp_path):
    root = tmp_path / 'stage'
    initialize_artifact_root(root, 10000, diagnostic_max_bytes=2000)
    initialize_artifact_root(root, 10000, diagnostic_max_bytes=2000)
    for total, diagnostic in ((20000, 2000), (10000, 3000), (9000, 2000)):
        with pytest.raises(ValueError, match='immutable'):
            initialize_artifact_root(root, total, diagnostic_max_bytes=diagnostic)
    with pytest.raises(ValueError, match='nested'):
        initialize_artifact_root(root / 'nested', 10000, diagnostic_max_bytes=2000)


def test_nonempty_root_without_manifest_is_not_silently_adopted(tmp_path):
    (tmp_path / 'old.log').write_bytes(b'old')
    with pytest.raises(ValueError, match='empty'):
        initialize_artifact_root(tmp_path, 10000)


@pytest.mark.parametrize('location', ['parent', 'target', 'unrelated'])
def test_symlinks_cannot_escape_accounting(tmp_path, location):
    root, outside = tmp_path / 'stage', tmp_path / 'outside'
    outside.mkdir()
    initialize_artifact_root(root, 10000)
    if location == 'parent':
        (root / 'alias').symlink_to(outside, target_is_directory=True)
        target = root / 'alias' / 'out.json'
    elif location == 'target':
        (outside / 'old.json').write_text('preserve')
        target = root / 'out.json'
        target.symlink_to(outside / 'old.json')
    else:
        (root / 'alias').symlink_to(outside, target_is_directory=True)
        target = root / 'out.json'
    with pytest.raises(ValueError, match='symlink'):
        atomic_json(target, {'test': True}, max_bytes=100)
    assert not (outside / 'out.json').exists()


def test_bounded_append_rotates_under_root_budget(tmp_path):
    initialize_artifact_root(tmp_path, 2000, diagnostic_max_bytes=1500)
    path = tmp_path / 'run' / 'stdout.log'
    for index in range(100):
        append_bounded(path, f'{index:03d}\n'.encode(), 40, whole_lines=True)
    assert path.stat().st_size == 40
    assert path.read_text().splitlines() == [f'{index:03d}' for index in range(90, 100)]


def test_atomic_json_file_limit_keeps_previous_file(tmp_path):
    initialize_artifact_root(tmp_path, 10000)
    path = tmp_path / 'summary.json'
    atomic_json(path, {'before': 1}, max_bytes=100)
    with pytest.raises(ResourceLimitError, match='artifact file bytes'):
        atomic_json(path, {'after': 'x' * 200}, max_bytes=100)
    assert json.loads(path.read_text()) == {'before': 1}


def test_metrics_and_evaluation_discover_ancestor_budget(tmp_path):
    from lunar_drl_exploration.evaluation import _atomic_file
    from lunar_drl_exploration.metrics import MetricsWriter
    initialize_artifact_root(tmp_path, 1800, diagnostic_max_bytes=1000)
    metrics = MetricsWriter(tmp_path / 'one' / 'metrics.jsonl', max_bytes=900)
    metrics.append({'data': 'x' * 500})
    with pytest.raises(ResourceLimitError, match='diagnostic peak bytes'):
        _atomic_file(tmp_path / 'two' / 'report.json', lambda stream: stream.write(b'y' * 500))
    assert not (tmp_path / 'two' / 'report.json').exists()


def test_checkpoint_explicit_root_rejects_output_outside_root(tmp_path):
    from lunar_drl_exploration.checkpoint import CheckpointManager
    with pytest.raises(ValueError, match='outside'):
        CheckpointManager(tmp_path / 'outside', artifact_root=tmp_path / 'stage', total_max_bytes=10000)


def _concurrent_writer(path, ready, results):
    ready.wait()
    try:
        atomic_write(path, lambda stream: stream.write(b'x' * 1200), diagnostic=False)
        results.put('saved')
    except ResourceLimitError:
        results.put('limited')


def test_process_writers_cannot_both_spend_the_same_free_bytes(tmp_path):
    initialize_artifact_root(tmp_path, 2200, diagnostic_max_bytes=1000)
    context = multiprocessing.get_context('spawn')
    ready, results = context.Event(), context.Queue()
    processes = [context.Process(target=_concurrent_writer,
        args=(tmp_path / str(index) / 'resume.pt', ready, results)) for index in range(2)]
    try:
        for process in processes:
            process.start()
        ready.set()
        assert sorted(results.get(timeout=20) for _ in processes) == ['limited', 'saved']
        for process in processes:
            process.join(20)
            assert process.exitcode == 0
        assert artifact_usage(tmp_path)['total_bytes'] <= 2200
    finally:
        for process in processes:
            if process.is_alive():
                process.terminate()
                process.join()


def test_interrupted_model_temporary_stays_in_total_not_diagnostic_budget(tmp_path):
    initialize_artifact_root(tmp_path, 5000, diagnostic_max_bytes=1000)
    seen = []
    def capture_temp(stream):
        stream.write(b'x' * 1500)
        seen.append(Path(stream.name).name)
    atomic_write(tmp_path / 'resume.pt', capture_temp, diagnostic=False)
    assert seen[0].endswith('.pt')
    # Simulate a previous writer killed before its temporary file was removed.
    orphan = tmp_path / seen[0]
    orphan.write_bytes(b'x' * 1500)
    atomic_write(tmp_path / 'resume.pt', lambda stream: stream.write(b'y' * 1500), diagnostic=False)
    assert artifact_usage(tmp_path)['diagnostic_bytes'] < 1000
    assert orphan.exists()


def test_parent_traversal_cannot_leave_stage(tmp_path):
    initialize_artifact_root(tmp_path, 10000)
    with pytest.raises(ValueError, match='traversal'):
        atomic_json(tmp_path / '..' / 'escape.json', {'bad': True})


def test_deleted_manifest_cannot_reset_existing_stage_budget(tmp_path):
    from lunar_drl_exploration.artifacts import MANIFEST
    initialize_artifact_root(tmp_path, 10000)
    atomic_json(tmp_path / 'existing.json', {'keep': 1})
    (tmp_path / MANIFEST).unlink()
    with pytest.raises(ValueError, match='empty'):
        initialize_artifact_root(tmp_path, 20000)
    with pytest.raises((ValueError, FileNotFoundError)):
        atomic_json(tmp_path / 'new.json', {'bad': 1})


@pytest.mark.parametrize('name', ['.artifact-budget.json', '.artifact-budget.lock'])
def test_generic_writers_cannot_replace_budget_control_files(tmp_path, name):
    initialize_artifact_root(tmp_path, 10000)
    with pytest.raises(ValueError, match='reserved'):
        atomic_json(tmp_path / name, {'total_max_bytes': 999999})
    with pytest.raises(ValueError, match='reserved'):
        append_bounded(tmp_path / name, b'bad', 1000)


def test_caught_budget_exception_cannot_replace_previous_good_artifact(tmp_path):
    initialize_artifact_root(tmp_path, 10000)
    target = tmp_path / 'summary.json'
    atomic_json(target, {'good': True}, max_bytes=100)
    def suppress_error(stream):
        stream.write(b'partial')
        try:
            stream.write(b'x' * 200)
        except ResourceLimitError:
            pass
    with pytest.raises(ResourceLimitError):
        atomic_write(target, suppress_error, max_bytes=100)
    assert json.loads(target.read_text()) == {'good': True}


def test_atomic_progress_write_survives_disappearing_sibling_temporary(tmp_path, monkeypatch):
    from lunar_drl_exploration import artifacts
    temporary = tmp_path / '.other-progress.json.pending.tmp'
    temporary.write_text('pending')
    atomic_json(tmp_path / 'completed.json', {'keep': True})
    original_walk = artifacts.os.walk
    removed = []

    def concurrent_replace(root, *args, **kwargs):
        for directory, directories, files in original_walk(root, *args, **kwargs):
            if temporary.name in files:
                # Another process finishes atomic replacement after directory listing.
                temporary.unlink()
                removed.append(temporary)
            yield directory, directories, files

    monkeypatch.setattr(artifacts.os, 'walk', concurrent_replace)
    atomic_json(tmp_path / 'progress.json', {'phase': 'passed'})
    assert removed == [temporary]
    assert json.loads((tmp_path / 'completed.json').read_text()) == {'keep': True}
    assert json.loads((tmp_path / 'progress.json').read_text()) == {'phase': 'passed'}


def _parallel_progress_writer(path, ready):
    ready.wait()
    for sequence in range(60):
        atomic_json(path, {'sequence': sequence, 'worker': path.name})


def test_eight_parallel_progress_writers_preserve_all_completed_records(tmp_path):
    context = multiprocessing.get_context('spawn')
    ready = context.Event()
    processes = [context.Process(target=_parallel_progress_writer,
        args=(tmp_path / f'worker-{index}.json', ready)) for index in range(8)]
    try:
        for process in processes:
            process.start()
        ready.set()
        for process in processes:
            process.join(30)
            assert process.exitcode == 0
        for index in range(8):
            name = f'worker-{index}.json'
            assert json.loads((tmp_path / name).read_text()) == {'sequence': 59, 'worker': name}
        assert not list(tmp_path.glob('*.tmp'))
    finally:
        for process in processes:
            if process.is_alive():
                process.terminate()
                process.join(5)
