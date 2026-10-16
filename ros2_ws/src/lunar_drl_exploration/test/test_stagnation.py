"""A rollout budget may stop local repetition, but must preserve useful transit."""
import importlib
import pytest


def monitor(window=6, positions=2, tolerance=.25):
    name='lunar_drl_exploration.stagnation'
    assert importlib.util.find_spec(name) is not None, 'stagnation monitor is not implemented'
    return importlib.import_module(name).StagnationMonitor(window,positions,tolerance)


def test_repeated_actual_positions_trigger_only_after_a_full_zero_gain_window():
    m=monitor()
    # Both sites straddle an arbitrary .25 m cell boundary; physical tolerance
    # must merge the jitter without consulting requested goals or graph IDs.
    poses=[(.249,0.),(2.249,0.),(.251,0.),(2.251,0.),(.248,0.),(2.248,0.)]
    assert [m.observe(p,0.) for p in poses]==[False]*5+[True]
    assert m.zero_gain_steps==6 and m.position_count==2 and m.triggered


def test_any_real_gain_and_reset_clear_the_previous_stagnation_window():
    m=monitor()
    for _ in range(5): assert not m.observe((0.,0.),0.)
    assert not m.observe((0.,0.),.00000001)
    assert m.zero_gain_steps==0 and m.buffered_steps==0 and not m.triggered
    for _ in range(5): assert not m.observe((0.,0.),0.)
    assert m.observe((0.,0.),0.)
    m.reset()
    assert m.zero_gain_steps==0 and m.buffered_steps==0 and not m.triggered
    assert not m.observe((0.,0.),0.)


def test_long_zero_gain_return_is_not_a_loop_and_old_transit_ages_out():
    m=monitor(window=12,positions=4)
    for x in list(range(100))+list(range(98,-1,-1)):
        assert not m.observe((float(x),0.),0.)
        assert m.buffered_steps<=12
    # Earlier motion cannot suppress detection forever after the robot stops.
    results=[m.observe((float(i%2),0.),0.) for i in range(12)]
    assert results[-1] and m.position_count==2 and m.buffered_steps==12


def test_close_neighbor_chain_does_not_collapse_long_motion_into_one_position():
    m=monitor(window=32,positions=4,tolerance=.25)
    for i in range(64):
        assert not m.observe((i*.2,0.),0.)


@pytest.mark.parametrize('pose,area', [((float('nan'),0.),0.),((0.,float('inf')),0.),((0.,0.),float('nan'))])
def test_nonfinite_input_cannot_be_treated_as_no_progress(pose,area):
    m=monitor()
    with pytest.raises(ValueError,match='finite'): m.observe(pose,area)
