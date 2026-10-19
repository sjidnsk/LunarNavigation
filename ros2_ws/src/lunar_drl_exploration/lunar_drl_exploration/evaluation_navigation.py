"""Explicit, identity-bound navigation wiring for isolated evaluations."""
import json
import math
from pathlib import Path


def validate_evaluation_navigation(settings):
    expected={'controller_backend','simulation_step_sync','mppi_nominal_response_time_s'}
    if not isinstance(settings,dict) or set(settings)!=expected:
        raise ValueError('evaluation navigation requires explicit backend, synchronization and response')
    backend=settings['controller_backend'];sync=settings['simulation_step_sync']
    response=settings['mppi_nominal_response_time_s']
    if backend not in ('pure_pursuit','mppi','mppi_path_follower') or type(sync) is not bool:
        raise ValueError('invalid evaluation backend or synchronization')
    if response is not None and (type(response) not in (int,float)
            or not math.isfinite(response) or response<0):
        raise ValueError('invalid MPPI nominal response time')
    if sync and backend=='pure_pursuit':
        raise ValueError('MPPI simulation synchronization requires an MPPI backend')
    return dict(settings)


def load_evaluation_navigation(root):
    root=Path(root);path=root/'evaluation-navigation.json'
    if path.exists():
        return validate_evaluation_navigation(json.loads(path.read_text(encoding='utf-8')))
    protocol=root/'comparison-protocol.json'
    if protocol.exists() and json.loads(protocol.read_text()).get('controller_backend','pure_pursuit')!='pure_pursuit':
        raise ValueError('MPPI comparison requires explicit evaluation-navigation.json')
    return dict(controller_backend='pure_pursuit',simulation_step_sync=False,
                mppi_nominal_response_time_s=None)
