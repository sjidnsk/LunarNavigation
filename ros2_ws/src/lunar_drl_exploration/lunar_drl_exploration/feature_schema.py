"""Measured task_graph_v4 node and action column definitions."""
NODE_FEATURE_DIM = 27
ACTION_FEATURE_DIM = 7
IN_GAIN_SLICE = slice(3, 11)
OUT_GAIN_SLICE = slice(11, 19)
HISTORY_SLICE = slice(19, 27)
IN_GAIN_SCALE = 100.0
OUT_GAIN_SCALE = 100.0

# These identify meanings, including units, rather than just tensor dimensions.
OBSERVATION_SCHEMA = 'task_graph_v4'
PRIVILEGED_SCHEMA = 'candidate_frontier_truth_v2'
LEARNER_SCHEMA = 'candidate_graph_sac_v3'
FRONTIER_SEMANTICS = {
    'frontier_definition': 'effective_known_free_center_with_unknown_8_neighbor_same_domain_v1',
    'domain_membership': 'center_and_demand_same_polygon_domain_boundary_inside_v1',
    'directional_fov': 'native_classified_center_tip_prefix_center_and_any_same_domain_demand_8_headings_v1',
    'removal': 'frozen_all_same_domain_demands_effectively_observed_old_frontier_absent_free_center_v1',
    'action_accumulation': 'whole_action_distance_and_absolute_yaw_variation_including_pending_turns_v1',
    'normalization': dict(node_inside_count=100., node_outside_count=100.,
                          reward_inside_count=50., critic_expected_removed_count=50.),
    'privileged_units': 'raw_nonnegative_integer_removed_frontiers_v1',
    'critic_feature': 'expected_removed_frontiers/50',
    'reward_formula': 'removed_frontiers_in/50-.02*distance/10-.005*absolute_turn/pi-.001+completion_bonus*coverage_success_v3',
}


def frontier_semantics():
    """Owned primitive identity suitable for Torch weights-only artifacts and IPC."""
    import copy
    return copy.deepcopy(FRONTIER_SEMANTICS)


def require_frontier_semantics(record):
    if record.get('frontier_semantics') != FRONTIER_SEMANTICS:
        raise ValueError('incompatible frontier semantics (definition, units or reward formula)')


def require_observation_identity(observation):
    if observation.schema == 'task_graph_v5':
        from .candidate_observation import CandidateObservation
        if not isinstance(observation, CandidateObservation):
            raise ValueError('task_graph_v5 requires a CandidateObservation contract')
        return
    if (observation.schema != OBSERVATION_SCHEMA or observation.features.ndim != 2 or
            observation.features.shape[1] != NODE_FEATURE_DIM):
        raise ValueError(f'expected {OBSERVATION_SCHEMA} observation schema with {NODE_FEATURE_DIM} columns')
