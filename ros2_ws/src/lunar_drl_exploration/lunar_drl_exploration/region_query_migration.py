"""Retired additive architecture entry points; historical code stays in old revisions."""


def _retired():
    raise ValueError('region query migration is retired; create region_relation_v2 from scratch or use old code to inspect old experiments')


def migrate_learner_region_query(record):
    _retired()


def fork_region_query(state, source, config):
    _retired()
