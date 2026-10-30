"""末段方向必须与最终车体姿态相容，同时保留制动换向。"""
import numpy as np
import pytest

from scripts.navigation.one_cm.nav1cm.contracts import LocalReference, VehicleState
from scripts.navigation.one_cm.nav1cm.local_mpcc import select_mode, travel_target_yaw


def ref(length=.025280458267038904, heading=.1034228759864693, yaw=.7853981633974483):
    return LocalReference(1, 1, [[0., 0.], [length*np.cos(heading), length*np.sin(heading)]],
                          [0., length], terminal_yaw=yaw)


def test_r19_stalled_pose_aligns_forward_before_terminal_approach():
    state = VehicleState(0., 0., 2.5960252611724064, 0., 0.)
    reference = ref()
    assert select_mode(state, reference) == 'spin'
    assert travel_target_yaw(state, reference, 'spin', .8) == pytest.approx(.1034228759864693)
    aligned = VehicleState(0., 0., .1034228759864693, 0., 0.)
    assert select_mode(aligned, reference) == 'forward'


@pytest.mark.parametrize('yaw, expected, target', [(0., 'forward', 0.), (np.pi, 'reverse', np.pi)])
def test_terminal_pose_selects_compatible_direction(yaw, expected, target):
    reference = ref(heading=0., yaw=yaw)
    aligned = VehicleState(0., 0., target, 0., 0.)
    assert select_mode(aligned, reference) == expected
    other = VehicleState(0., 0., np.pi-target, 0., 0.)
    assert select_mode(other, reference) == 'spin'
    assert np.cos(travel_target_yaw(other, reference, 'spin', .8)-target) > .99


@pytest.mark.parametrize('yaw,v', [(0., -.02), (np.pi, .02)])
def test_incompatible_translation_brakes_before_direction_change(yaw, v):
    state = VehicleState(0., 0., 0., v, 0.)
    assert select_mode(state, ref(heading=0., yaw=yaw)) == 'brake'


def test_distant_route_keeps_current_heading_direction_preference():
    state = VehicleState(0., 0., np.pi, 0., 0.)
    assert select_mode(state, ref(length=2., heading=0., yaw=0.)) == 'reverse'


def test_remaining_angular_motion_still_requires_spin_phase():
    assert select_mode(VehicleState(0., 0., 0., 0., .02), ref(heading=0., yaw=0.)) == 'spin'


@pytest.mark.parametrize('yaw', [np.pi/2, -np.pi/2])
@pytest.mark.parametrize('body, expected', [(0., 'forward'), (np.pi, 'reverse')])
def test_perpendicular_final_pose_preserves_existing_direction(yaw, body, expected):
    assert select_mode(VehicleState(0., 0., body, 0., 0.), ref(heading=0., yaw=yaw)) == expected


@pytest.mark.parametrize('length, expected', [(.8, 'spin'), (.800001, 'reverse')])
def test_terminal_direction_applies_only_inside_approach_distance(length, expected):
    assert select_mode(VehicleState(0., 0., np.pi, 0., 0.),
                       ref(length=length, heading=0., yaw=0.)) == expected


def test_direction_alignment_wraps_across_pi_by_shortest_turn():
    state = VehicleState(0., 0., np.pi-.02, 0., 0.)
    reference = ref(heading=-np.pi+.02, yaw=-np.pi+.1)
    assert select_mode(state, reference) == 'forward'
    assert travel_target_yaw(state, reference, 'forward', .8)-state.yaw == pytest.approx(.04)
