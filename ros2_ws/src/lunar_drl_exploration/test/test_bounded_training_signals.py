"""Real process-group signals must preserve the learner's admission barrier."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest


ROOT = Path(__file__).resolve().parents[4]


def test_wrapper_sigterm_saves_new_admissions_and_closes_collector_workers(tmp_path):
    pytest.importorskip('torch')
    from lunar_drl_exploration.artifacts import initialize_artifact_root
    artifact_root = tmp_path / 'artifacts'
    initialize_artifact_root(artifact_root, total_max_bytes=8 * 1024**3)
    fixture = tmp_path / 'training_fixture.py'
    fixture.write_text('''
from dataclasses import replace
import multiprocessing as mp
from pathlib import Path
import sys

if __name__ == '__main__':
    from lunar_drl_exploration.artifacts import atomic_json
    from lunar_drl_exploration.config import TrainingConfig
    from lunar_drl_exploration.training import run_training
    from lunar_drl_exploration.checkpoint import CheckpointManager
    root = Path(sys.argv[1])
    config = replace(TrainingConfig(), environments=2, warmup=0,
        system_reserve_bytes=0, save_interval_s=3600.,
        artifact_root=str(root), output_dir=str(root / 'training'),
        total_output_max_bytes=8 * 1024**3)
    result = run_training(config, device='cpu', max_transitions=10000,
        env_factory='worker_fixtures:ControlledEnv', probe_extent=40, probe_budget=1000)
    saved = CheckpointManager(config.output_dir).load(config).record
    atomic_json(root / 'fixture-result.json', dict(result=result,
        saved_transitions=saved['schedule']['transitions'],
        unfinished=saved['schedule']['unfinished_reservations'],
        credit=saved['schedule']['credit'],
        updates=saved['schedule']['updates'],
        learner_updates=saved['learner']['updates'],
        actor_version=saved['collector_state']['actor_version'],
        active_children=[p.pid for p in mp.active_children()]))
''', encoding='utf-8')
    python_paths = [str(ROOT / 'ros2_ws/src/lunar_drl_exploration'),
                    str(Path(__file__).parent), os.environ.get('PYTHONPATH', '')]
    environment = dict(os.environ, PYTHONPATH=os.pathsep.join(python_paths))
    process = subprocess.Popen([sys.executable, str(ROOT / 'scripts/drl/run_bounded.py'),
        '--artifact-root', str(artifact_root), '--stop-grace-seconds', '30', '--',
        sys.executable, str(fixture), str(artifact_root)], env=environment,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        metrics_path = artifact_root / 'training/metrics.jsonl'
        run_path = artifact_root / 'training/run.json'
        deadline = time.monotonic() + 30
        transitions = []
        while time.monotonic() < deadline:
            assert process.poll() is None, (
                (artifact_root / 'runtime.log').read_text(errors='replace'), process.communicate())
            if metrics_path.exists():
                # Ignore only an incomplete final line while the actual writer
                # appends; complete records independently prove admissions.
                records = []
                for line in metrics_path.read_text().splitlines():
                    try:
                        records.append(json.loads(line))
                    except json.JSONDecodeError:
                        continue
                transitions = [row for row in records if row.get('event') == 'transition']
                if len(transitions) >= 16 and {row['env'] for row in transitions} == {0, 1}:
                    break
            time.sleep(.03)
        else:
            pytest.fail('real collector did not admit transitions before signal deadline')
        ownership = json.loads(run_path.read_text())['owned_pids']
        collector_pid = ownership[1]
        ownership += [int(pid) for pid in Path(
            f'/proc/{collector_pid}/task/{collector_pid}/children').read_text().split()]
        assert len(ownership) >= 4  # learner, collector, and two environment workers
        process.send_signal(signal.SIGTERM)
        _, stderr = process.communicate(timeout=45)
        log = (artifact_root / 'runtime.log').read_text(errors='replace')
        status = json.loads((artifact_root / 'exit.json').read_text())
        assert status['child_returncode'] == 0, (status, stderr.decode(), log)
        assert process.returncode == 143
        assert status['reason'] == 'signal' and status['signal'] == signal.SIGTERM
        assert status['forced_stop_stage'] == 0
        evidence = json.loads((artifact_root / 'fixture-result.json').read_text())
        assert evidence['saved_transitions'] >= len(transitions) >= 16
        assert evidence['saved_transitions'] == evidence['result']['transitions']
        assert evidence['result']['new_transitions'] == evidence['saved_transitions']
        assert evidence['unfinished'] == 0
        from fractions import Fraction
        assert evidence['updates'] == evidence['learner_updates'] == evidence['actor_version']
        assert Fraction(evidence['credit']) == Fraction(evidence['saved_transitions'], 4) - evidence['updates']
        assert evidence['active_children'] == []
        final = json.loads(run_path.read_text())['observations']['final']
        assert final['owned_children_closed']
        assert not [pid for pid in ownership if Path(f'/proc/{pid}').exists()]
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            process.communicate(timeout=45)
