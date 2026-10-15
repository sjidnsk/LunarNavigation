"""Explicit continuation across an approved change of arrival tolerances."""
from .config import config_record, resume_semantics, training_config_from_record


def migrate_arrival_contract(state, config):
    """Retain every learner/replay/collector/RNG value; alter only metadata.

    Historical transitions keep their measured outcomes. They are not relabeled
    to the new acceptance band. Callers must preserve the old checkpoint and
    record runtime/config attribution before adopting this migrated state.
    """
    from .checkpoint import TrainingState
    original = training_config_from_record(state.record['config'])
    state.validate(original)
    before, after = config_record(original), config_record(config)
    allowed = {'goal_position_tolerance_m', 'goal_yaw_tolerance_rad'}
    changed = {key for key in before.keys() | after.keys()
               if before.get(key) != after.get(key)}
    if not changed or not changed <= allowed:
        raise ValueError('migration permits only arrival tolerances')
    record = dict(state.record, config=after, semantics=resume_semantics(config))
    record['arrival_contract_migrations'] = [
        *state.record.get('arrival_contract_migrations', []),
        {'version': 1, 'before': {key: before[key] for key in sorted(allowed)},
         'after': {key: after[key] for key in sorted(allowed)},
         'historical_replay': 'retained_without_relabeling'}]
    migrated = TrainingState(record)
    migrated.validate(config)
    return migrated
