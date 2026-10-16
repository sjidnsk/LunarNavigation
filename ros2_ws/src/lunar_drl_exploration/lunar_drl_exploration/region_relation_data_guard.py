"""Physical heldout terrain exclusion before training policy access.

Only frozen case identities/hashes are read, never heldout policy outcomes.
One environment owns one append-only audit journal. No training is launched here.
"""
import json
import math
import os
from pathlib import Path
import re
import time

from .course_two_control import sha, runtime_digest
from .region_relation_run import physical_terrain_sha256


def _read_record(path):
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError) as error:
        raise ValueError('unreadable frozen data guard record: ' + str(path)) from error
    if not isinstance(value, dict):
        raise ValueError('frozen data guard record must be an object: ' + str(path))
    return value


def _short_text(value, size=64):
    return str(value).encode('utf-8')[:size].decode('utf-8', errors='ignore')


class TrainingTerrainGuard:
    """Validate frozen provenance once; reject physical heldout collisions per reset.

    Pass ``runtime`` only after the owner verifies the current source runtime.
    Otherwise the constructor computes the current runtime digest itself.
    ``config_sha`` may bind the caller's already-loaded training config; the
    on-disk root config hash is always independently checked.
    """
    def __init__(self, root, config_sha=None, *, runtime=None):
        self.root = Path(root).resolve()
        identity = _read_record(self.root / 'identity.json')
        if identity.get('schema') != 'region_relation_run_identity_v1':
            raise ValueError('invalid frozen training run identity schema')
        if identity.get('qualified') is not True:
            raise ValueError('qualified frozen heldout terrain identity required')
        actual_config = sha(self.root / 'config.json')
        if (identity.get('config_sha') != actual_config or
                config_sha is not None and config_sha != actual_config):
            raise ValueError('training terrain guard config identity changed')
        actual_runtime = runtime_digest() if runtime is None else runtime
        if not actual_runtime or identity.get('runtime') != actual_runtime:
            raise ValueError('training terrain guard runtime identity changed')
        self.run_id = identity.get('run_id')
        if not isinstance(self.run_id, str) or not self.run_id or len(self.run_id.encode()) > 128:
            raise ValueError('invalid frozen training run_id')
        if not isinstance(actual_runtime, str) or len(actual_runtime.encode()) > 128:
            raise ValueError('invalid frozen runtime identity')
        self.runtime, self.config_sha = actual_runtime, actual_config
        physical = identity.get('physical_terrain_sha256_by_case')
        manifest_shas = identity.get('manifests')
        if not isinstance(physical, dict) or not isinstance(manifest_shas, dict):
            raise ValueError('frozen physical terrain hashes and manifests required')
        heldout = {}
        for split in ('validation', 'test'):
            path = self.root / (split + '.json')
            if manifest_shas.get(split) != sha(path):
                raise ValueError('heldout manifest identity changed: ' + split)
            manifest = _read_record(path)
            cases = manifest.get('cases')
            if manifest.get('split') != split or not isinstance(cases, list) or not cases:
                raise ValueError('nonempty frozen heldout manifest required: ' + split)
            ids = set()
            for case in cases:
                cid = case.get('case_id') if isinstance(case, dict) else None
                if not isinstance(cid, str) or not cid or cid in ids:
                    raise ValueError('invalid or duplicate frozen heldout case identity')
                ids.add(cid)
                value = physical.get(cid)
                if not isinstance(value, str) or not re.fullmatch(r'[0-9a-f]{64}', value):
                    raise ValueError('missing/invalid heldout physical terrain hash: ' + cid)
                heldout.setdefault(value, []).append(cid)
        self.heldout_hashes = frozenset(heldout)
        self._heldout_cases = heldout

    def check(self, terrain, descriptor, episode_id, env_id):
        """Reject a heldout grid, else durably journal and return its physical SHA.

        Call exactly once after an episode's terrain is fully constructed and
        before its first policy call. Descriptor split/seed labels never affect
        exclusion: the generated physical grid is hashed independently.
        """
        label = str(env_id)
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', label):
            raise ValueError('safe nonempty env_id required for training terrain audit')
        if not isinstance(episode_id, str) or not episode_id or len(episode_id.encode()) > 128:
            raise ValueError('bounded nonempty episode_id required for training terrain audit')
        terrain_sha = physical_terrain_sha256(terrain)
        if terrain_sha in self.heldout_hashes:
            raise ValueError('heldout physical terrain cannot enter training: ' + terrain_sha +
                             ' (' + ', '.join(self._heldout_cases[terrain_sha]) + ')')
        compact = {}
        descriptor = descriptor or {}
        for key in ('seed', 'family', 'curriculum_task', 'mother_graph_id',
                    'generator_version', 'scenario_id'):
            if key not in descriptor:
                continue
            value = descriptor[key]
            if key == 'seed' and hasattr(value, 'item'):
                value = value.item()
            if value is None or type(value) in (bool, int):
                compact[key] = value
            elif type(value) is float and math.isfinite(value):
                compact[key] = value
            else:
                compact[key] = _short_text(value)
        record = dict(schema='training_terrain_instance_v1', run_id=self.run_id,
            config_sha=self.config_sha, runtime=self.runtime, env_id=label,
            episode_id=episode_id, terrain_sha256=terrain_sha,
            admitted_unix=time.time(), descriptor=compact)
        def encode():
            return (json.dumps(record, sort_keys=True, ensure_ascii=False,
                               separators=(',', ':')) + '\n').encode('utf-8')
        line = encode()
        # Keep complete physical/provenance identifiers; only optional labels
        # may be dropped to retain the 1 KiB per-instance journal bound.
        for key in reversed(tuple(compact)):
            if len(line) <= 1024:
                break
            if key == 'seed':
                continue
            del compact[key]
            line = encode()
        if len(line) > 1024:
            raise ValueError('training terrain audit identifiers exceed 1 KiB record bound')
        folder = self.root / 'training-terrain-audit'
        folder.mkdir(exist_ok=True)
        journal = folder / ('env-' + label + '.jsonl')
        fd = os.open(journal, os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o600)
        try:
            if os.write(fd, line) != len(line):
                raise OSError('incomplete training terrain audit append')
            os.fsync(fd)
        finally:
            os.close(fd)
        return terrain_sha
