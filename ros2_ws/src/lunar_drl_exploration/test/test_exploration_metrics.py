import pytest
from lunar_drl_exploration.exploration_metrics import ExplorationTrace


def test_thresholds_use_confirmed_samples_and_real_odometer():
    trace = ExplorationTrace(.1, 512)
    trace.sample(7., 10., .9)
    trace.sample(12., 18., .95)
    trace.sample(17., 26., .99)
    result = trace.summary(100.)
    assert result['thresholds']['95']['distance_m'] == 12.
    assert result['thresholds']['99']['simulation_s'] == 26.
    assert result['tail_95_to_99'] == {'distance_m': 5., 'simulation_s': 8.}
    assert result['new_area_per_meter_m2'] == pytest.approx(89./17.)


def test_no_gain_metrics_keep_motion_and_turning_time():
    trace = ExplorationTrace(0., 512)
    trace.sample(2., 10., .1)
    trace.sample(8., 20., .1)
    trace.sample(8., 30., .1)
    trace.sample(10., 40., .2)
    result = trace.summary(100.)
    assert result['max_no_gain_distance_m'] == 6.
    assert result['max_no_gain_simulation_s'] == 20.


def test_failure_never_fabricates_threshold_or_free_completion():
    trace = ExplorationTrace(0., 512)
    trace.sample(10., 30., .8)
    trace.decision(1, .8)
    result = trace.summary(100.)
    assert result['thresholds']['99'] is None
    assert result['tail_95_to_99'] is None
    # AUC holds the last observed coverage after a terminal failure; failure
    # remains in safety/completion denominators, and the rule is explicit.
    assert result['decision_auc_512'] == pytest.approx(.8*511/512)


def test_invalid_or_nonmonotonic_samples_fail_visibly():
    trace = ExplorationTrace(.1, 512)
    trace.sample(2., 10., .2)
    for sample in [(1., 11., .3), (3., 9., .3), (3., 11., .1), (3., 11., float('nan'))]:
        with pytest.raises(ValueError): trace.sample(*sample)


def test_no_initial_area_credit_or_division_by_zero():
    trace = ExplorationTrace(.95, 512)
    result = trace.summary(100.)
    assert result['thresholds']['95']['distance_m'] == 0.
    assert result['new_area_per_meter_m2'] is None
    assert result['thresholds']['99'] is None


def test_physical_auc_is_left_continuous_and_uses_common_budget():
    from lunar_drl_exploration.exploration_metrics import curve_auc
    assert curve_auc([[0,0,.1],[10,20,.9]], 0, 20) == pytest.approx(.5)
    assert curve_auc([[0,0,.1],[10,20,.9]], 1, 40) == pytest.approx(.5)
