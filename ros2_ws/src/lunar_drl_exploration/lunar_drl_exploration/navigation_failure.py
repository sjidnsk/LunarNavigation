"""Failure-only evidence from real execution and the immutable policy map.

This module neither repairs observations nor changes navigation/replay outcomes.
"""
from dataclasses import asdict, is_dataclass
import json
import math
from pathlib import Path
import time
import uuid

import numpy as np


def _finite(value):
    if is_dataclass(value) and not isinstance(value, type):
        return _finite(asdict(value))
    if isinstance(value, np.ndarray):
        return _finite(value.tolist())
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, dict):
        return {str(k): _finite(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite(v) for v in value]
    return value


def write_navigation_failure(artifact_root, env_id, context, *, snapshot=None):
    folder = Path(artifact_root) / 'navigation-failures'
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f'env-{env_id}-{time.time_ns()}-{uuid.uuid4().hex[:8]}.json'
    record = dict(schema='navigation_failure_v1', created_unix=time.time(),
                  env_id=env_id, context=context, snapshot=None)
    if snapshot is not None:
        keys = sorted(snapshot.tiles)
        arrays = {'tile_keys': np.asarray(keys, dtype=np.int64).reshape(-1, 2),
                  'start_connections': snapshot.start_connections}
        for i, key in enumerate(keys):
            tile = snapshot.tiles[key]
            for field in ('states', 'intrinsic_states', 'observed', 'costs', 'elevation_m'):
                arrays[f'tile_{i}_{field}'] = getattr(tile, field)
        np.savez_compressed(path.with_suffix('.npz'), **arrays)
        record['snapshot'] = dict(epoch=snapshot.epoch, revision=snapshot.revision,
            resolution_m=snapshot.resolution_m, origin=snapshot.origin,
            pose=asdict(snapshot.pose), local_bounds=snapshot.local_bounds,
            profile_hash=snapshot.profile_hash,
            start_connection_status=snapshot.start_connection_status,
            goal_position_tolerance_m=snapshot.goal_position_tolerance_m,
            goal_yaw_tolerance_rad=snapshot.goal_yaw_tolerance_rad,
            arrays=path.with_suffix('.npz').name)
    path.write_text(json.dumps(_finite(record), ensure_ascii=False, indent=2,
                               allow_nan=False) + '\n', encoding='utf-8')
    return path
