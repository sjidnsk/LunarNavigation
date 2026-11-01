import math
from dataclasses import replace
import pytest
from lunar_pure_wheeled_controller.execution import PathExecutor
from lunar_pure_wheeled_controller.path_geometry import Polyline
from lunar_pure_wheeled_controller.tracking import TrackingPolicy, TrackingState


def test_long_segment_midpoint_is_on_path():
    line = Polyline(((0, 0, 0), (3, 0, 0)))
    assert line.project(1.5, 0).error == 0
    assert line.point_at(2.0) == pytest.approx((2.0, 0.0))
    executor = PathExecutor(TrackingPolicy())
    executor.set_path(((0, 0, 0), (3, 0, 0)))
    assert (
        executor.update(TrackingState(1.5, 0, 0), 0.05).command.failure_reason is None
    )


def test_large_initial_heading_error_rotates_before_translation():
    executor = PathExecutor(TrackingPolicy())
    executor.set_path(((0, 0, -1.85), (3, 0, 0)))
    result = executor.update(TrackingState(0, 0, -1.85), 0.05)
    assert result.phase == "ALIGNING"
    assert result.command.linear_x_mps == 0
    assert result.command.angular_z_radps > 0


def test_reverse_direction_is_latched_and_curvature_is_bounded():
    executor = PathExecutor(TrackingPolicy())
    executor.set_path(((0, 0, 0), (-3, 0, 0)))
    result = executor.update(TrackingState(0, 0, 0), 0.05)
    assert result.direction == -1
    assert result.command.linear_x_mps < 0
    assert abs(result.command.linear_x_mps) <= 0.2


def test_completion_waits_for_measured_stop():
    executor = PathExecutor(TrackingPolicy())
    executor.set_path(((0, 0, 0), (1, 0, 0)))
    result = executor.update(TrackingState(1, 0, 0, linear_mps=0.2), 0.05)
    assert not result.command.complete
    result = executor.update(TrackingState(1, 0, 0), 0.05)
    assert result.command.complete


def test_final_yaw_control_leaves_margin_inside_public_acceptance():
    executor = PathExecutor(replace(TrackingPolicy(), goal_yaw_tolerance_rad=0.05))
    executor.set_path(((0., 0., 0.), (0., 0., 0.)), final=True)
    result = executor.update(TrackingState(0., 0., -0.049), 0.05)
    assert not result.command.complete
    assert result.command.angular_z_radps > 0.
    assert executor.update(TrackingState(0., 0., -0.02), 0.05).command.complete


def test_final_position_control_leaves_margin_inside_public_acceptance():
    executor = PathExecutor(replace(TrackingPolicy(), goal_position_tolerance_m=0.10))
    executor.set_path(((0., 0., 0.), (1., 0., 0.)), final=True)
    result = executor.update(TrackingState(0.925, 0., 0.), 0.05)
    assert not result.command.complete
    assert result.command.linear_x_mps > 0.
    assert executor.update(TrackingState(0.96, 0., 0.), 0.05).command.complete


@pytest.mark.parametrize('x,yaw,reason', [
    (1.11, 0., 'GOAL_POSITION_LOST'),
    (1., 0.0505, 'GOAL_YAW_LOST'),
])
def test_completed_reference_rechecks_new_stopped_pose(x, yaw, reason):
    executor = PathExecutor(replace(TrackingPolicy(), goal_position_tolerance_m=0.10,
                                   goal_yaw_tolerance_rad=0.05))
    executor.set_path(((0., 0., 0.), (1., 0., 0.)), identity=('goal', 1), final=True)
    assert executor.update(TrackingState(1., 0., 0.), 0.05).command.complete
    result = executor.update(TrackingState(x, 0., yaw), 0.05)
    assert not result.command.complete
    assert result.command.failure_reason == reason
    assert result.command.linear_x_mps == result.command.angular_z_radps == 0.


def test_old_revision_and_duplicate_do_not_reset_execution():
    executor = PathExecutor(TrackingPolicy())
    path = ((0, 0, 0), (3, 0, 0))
    assert executor.set_path(path, identity=("a", 2))
    executor.update(TrackingState(1, 0, 0), 0.05)
    assert not executor.set_path(path, identity=("a", 1))
    assert not executor.set_path(path, identity=("a", 2))
    assert executor.progress_m == pytest.approx(1.0)


def test_corner_target_does_not_cut_across_right_angle():
    executor = PathExecutor(TrackingPolicy())
    executor.set_path(((0, 0, 0), (2, 0, math.pi / 2), (2, 2, math.pi / 2)))
    result = executor.update(TrackingState(1.7, 0, 0), 0.05)
    assert result.command.angular_z_radps == 0
    result = executor.update(TrackingState(2, 0, 0), 0.05)
    assert result.command.linear_x_mps == 0
    assert result.command.angular_z_radps > 0


@pytest.mark.parametrize(
    "field,value",
    [
        ("max_reverse_mps", -1.0),
        ("max_linear_accel_mps2", float("nan")),
        ("max_curvature_per_m", 0.0),
        ("no_progress_timeout_s", -1.0),
        ("allow_reverse", 1),
    ],
)
def test_invalid_execution_limits_rejected(field, value):
    with pytest.raises(ValueError):
        PathExecutor(replace(TrackingPolicy(), **{field: value}))


def test_final_alignment_stall_requests_failure_only_after_stop():
    executor = PathExecutor(replace(TrackingPolicy(), no_progress_timeout_s=0.3))
    executor.set_path(((0, 0, 0), (0, 0, 1.0)))
    for _ in range(10):
        result = executor.update(TrackingState(0, 0, 0), 0.1)
    assert result.phase == "FAILED"
    assert result.command.failure_reason == "ALIGNMENT_STALLED"


def test_deviation_waits_for_measured_stop_before_failure():
    executor = PathExecutor(TrackingPolicy())
    executor.set_path(((0, 0, 0), (3, 0, 0)))
    assert executor.update(TrackingState(1, 2, 0, 0.2), 0.05).phase == "BRAKING"
    assert executor.update(TrackingState(1, 2, 0), 0.05).phase == "FAILED"


@pytest.mark.parametrize("curvature", [-1.0, -0.1, 0.0, 0.1, 1.0])
@pytest.mark.parametrize("yaw_rate", [-0.2, -0.01, 0.0, 0.01, 0.2])
def test_joint_speed_limits_preserve_curvature_and_slew(curvature, yaw_rate):
    from lunar_pure_wheeled_controller.limits import tracking_speed

    p = TrackingPolicy()
    speed = tracking_speed(curvature, 0.2, 0.1, yaw_rate, 0.05, p)
    if speed is not None:
        assert abs(speed - 0.1) <= 0.025 + 1e-9
        assert abs(speed * curvature - yaw_rate) <= 0.025 + 1e-9
        assert 0 <= speed <= 0.2


def test_non_final_reference_still_executes_its_terminal_heading():
    executor=PathExecutor(TrackingPolicy())
    executor.set_path(((0,0,0),(0,0,1.)),final=False)
    result=executor.update(TrackingState(0,0,0),.05)
    assert result.phase == 'FINAL_ALIGN'
    assert result.command.angular_z_radps > 0.


@pytest.mark.parametrize("final", [True, False])
@pytest.mark.parametrize("yaw_rate", [0.0, 0.1])
def test_final_alignment_losing_endpoint_position_brakes_then_fails(final, yaw_rate):
    executor = PathExecutor(TrackingPolicy())
    executor.set_path(((0, 0, 0), (1, 0, 1)), final=final)
    assert executor.update(TrackingState(1, 0, 0), 0.05).phase == "FINAL_ALIGN"
    # An along-path localization correction has zero cross-track error but
    # invalidates the endpoint acceptance used to enter final alignment.
    result = executor.update(TrackingState(0.5, 0, 1, 0, yaw_rate), 0.05)
    assert result.phase == "BRAKING"
    assert not result.command.complete
    assert result.command.linear_x_mps == 0
    assert result.command.angular_z_radps == 0
    result = executor.update(TrackingState(0.5, 0, 1), 0.05)
    assert result.phase == "FAILED"
    assert result.command.failure_reason == "GOAL_POSITION_LOST"


def test_rolling_update_keeps_tracking_and_rebases_progress():
    executor = PathExecutor(TrackingPolicy(), rolling_updates=True)
    executor.set_path(((0, 0, 0), (5, 0, 0)), identity=('a', 1), final=False)
    executor.update(TrackingState(0, 0, 0), .05)
    executor.update(TrackingState(.5, 0, 0, linear_mps=.1), .05)
    executor.set_path(((.4, 0, 0), (6, 0, 0)), identity=('a', 2), final=False)
    result = executor.update(TrackingState(.6, 0, 0, linear_mps=.1), .05)
    assert result.phase == 'TRACKING'
    assert result.command.linear_x_mps > 0
    assert result.progress_m == pytest.approx(.2)


@pytest.mark.parametrize('jitter', [0., .001])
def test_rolling_revisions_do_not_hide_a_stalled_vehicle(jitter):
    executor = PathExecutor(replace(TrackingPolicy(), no_progress_timeout_s=.3), rolling_updates=True)
    for i in range(12):
        executor.set_path(((0, 0, 0), (5, 0, 0)), identity=('a', i+1), final=False)
        result = executor.update(TrackingState(jitter * (i % 2), 0, 0), .05)
    assert result.phase == 'FAILED'
    assert result.command.failure_reason == 'TRACKING_STALLED'


def test_rolling_nonfinal_endpoint_does_not_align_but_final_does():
    for final in [False, True]:
        executor = PathExecutor(TrackingPolicy(), rolling_updates=True)
        executor.set_path(((0, 0, 0), (1, 0, 1.)), identity=('a', 1), final=final)
        result = executor.update(TrackingState(1, 0, 0), .05)
        assert result.phase == ('FINAL_ALIGN' if final else 'COMPLETED')


def test_rolling_direction_change_requires_stop():
    executor = PathExecutor(TrackingPolicy(), rolling_updates=True)
    executor.set_path(((0, 0, 0), (5, 0, 0)), identity=('a', 1), final=False)
    executor.update(TrackingState(0, 0, 0), .05)
    executor.set_path(((.1, 0, 0), (-5, 0, 0)), identity=('a', 2), final=False)
    result = executor.update(TrackingState(.1, 0, 0, linear_mps=.1), .05)
    assert result.phase == 'BRAKING'
    assert result.command.linear_x_mps == 0


def test_rolling_projection_does_not_jump_to_later_loop():
    executor = PathExecutor(TrackingPolicy(), rolling_updates=True)
    executor.set_path(((0, 0, 0), (5, 0, 0)), identity=('a', 1), final=False)
    executor.update(TrackingState(0, 0, 0), .05)
    executor.set_path(((0, 0, 0), (2, 0, 0), (2, 2, 0), (0, 2, 0), (0, .05, 0), (3, .05, 0)), identity=('a', 2), final=False)
    result = executor.update(TrackingState(.2, .05, 0, linear_mps=.1), .05)
    assert result.progress_m < 1.


def test_rolling_update_preserves_large_deviation_failure():
    executor = PathExecutor(TrackingPolicy(), rolling_updates=True)
    executor.set_path(((0,0,0),(5,0,0)), identity=('a',1), final=False)
    executor.update(TrackingState(0,0,0),.05)
    executor.set_path(((0,3,0),(5,3,0)), identity=('a',2), final=False)
    assert executor.update(TrackingState(.1,0,0,linear_mps=.1),.05).phase == 'BRAKING'


def test_rolling_stall_uses_vehicle_motion_not_rebased_path_arc():
    executor = PathExecutor(replace(TrackingPolicy(), no_progress_timeout_s=.3), rolling_updates=True)
    for i in range(12):
        executor.set_path(((0, 0, 0), (5, 0, 0)), identity=('a', i+1), final=False)
        result=executor.update(TrackingState(.5, 0, 0), .05)
    assert result.phase == 'FAILED'
    assert result.command.failure_reason == 'TRACKING_STALLED'


def test_rolling_failed_executor_reports_failure_for_latest_adopted_revision():
    executor=PathExecutor(TrackingPolicy(),rolling_updates=True)
    executor.set_path(((0,0,0),(5,0,0)),identity=('a',1),final=False)
    executor.update(TrackingState(0,2,0),.05)
    assert executor.failure=='PATH_DEVIATION'
    assert executor.set_path(((0,2,0),(5,2,0)),identity=('a',2),final=False)
    assert executor.identity==('a',2)
    result=executor.update(TrackingState(0,2,0),.05)
    assert result.phase=='FAILED'
    assert result.command.failure_reason=='PATH_DEVIATION'
    executor.clear()
    assert executor.set_path(((0,2,0),(5,2,0)),identity=('a',3),final=False)
    assert executor.update(TrackingState(0,2,0),.05).phase=='TRACKING'


def test_rolling_reference_arriving_during_final_align_keeps_alignment():
    executor=PathExecutor(replace(TrackingPolicy(),no_progress_timeout_s=.3),rolling_updates=True)
    executor.set_path(((0,0,0),(1,0,1.)),identity=('a',1),final=True)
    assert executor.update(TrackingState(1,0,0),.05).phase=='FINAL_ALIGN'
    for i in range(2,12):
        executor.set_path(((.8,0,0),(1,0,1.)),identity=('a',i),final=True)
        result=executor.update(TrackingState(1,0,0),.05)
        assert result.phase in ('FINAL_ALIGN','BRAKING','FAILED')
    assert result.command.failure_reason=='ALIGNMENT_STALLED'
    assert executor.identity==('a',11)


@pytest.mark.parametrize('direction', [1, -1])
def test_rolling_small_heading_change_slews_without_stopping(direction):
    executor = PathExecutor(TrackingPolicy(), rolling_updates=True)
    executor.set_path(((0, 0, 0), (direction * 5, 0, 0)), identity=('a', 1), final=False)
    executor.update(TrackingState(0, 0, 0), .05)
    for i in range(20):
        executor.update(TrackingState(direction * i * .02, 0, 0, linear_mps=direction * .2), .05)
    executor.set_path(((direction * .5, 0, 0), (direction * 5, .9, .2)),
                      identity=('a', 2), final=False)
    result = executor.update(TrackingState(direction * .5, 0, 0, linear_mps=direction * .2), .05)
    assert result.phase == 'TRACKING'
    # Small reference changes retain motion while approaching the new
    # error-based speed target at the configured braking rate.
    assert result.command.linear_x_mps == pytest.approx(direction * .175)
    assert result.command.angular_z_radps == pytest.approx(direction * .025)


@pytest.mark.parametrize('curvature,yaw_rate', [(.8, 0), (-.8, 0), (0, .1), (.8, -.1)])
def test_rolling_curvature_transition_respects_all_motion_limits(curvature, yaw_rate):
    from lunar_pure_wheeled_controller.limits import rolling_tracking_command
    p = TrackingPolicy()
    speed, yaw = rolling_tracking_command(curvature, .2, .2, yaw_rate, .05, p)
    assert .2 - p.max_linear_decel_mps2 * .05 <= speed <= .2
    assert abs(yaw - yaw_rate) <= p.max_angular_accel_radps2 * .05 + 1e-9
    assert abs(yaw) <= p.max_angular_radps
    assert abs(yaw / speed) <= p.max_curvature_per_m
    assert abs(yaw * speed) <= p.max_lateral_accel_mps2
    assert abs(yaw - speed * curvature) < abs(yaw_rate - speed * curvature)


def test_rolling_transition_does_not_relax_infeasible_deceleration():
    from lunar_pure_wheeled_controller.limits import rolling_tracking_command
    assert rolling_tracking_command(.8, 0, .2, 0, .05, TrackingPolicy()) is None


def test_rolling_replacement_still_brakes_when_required_curvature_exceeds_limit():
    executor = PathExecutor(TrackingPolicy(), rolling_updates=True)
    executor.set_path(((0, 0, 0), (5, 0, 0)), identity=('a', 1), final=False)
    executor.update(TrackingState(0, 0, 0), .05)
    executor.set_path(((.5, 0, 0), (5, 1.5, .3)), identity=('a', 2), final=False)
    result = executor.update(TrackingState(.5, 0, 0, linear_mps=.2), .05)
    assert result.phase == 'BRAKING'
    assert result.command.linear_x_mps == 0


@pytest.mark.parametrize("lateral", [0.2, float("nan")])
def test_endpoint_does_not_complete_with_lateral_motion_or_invalid_feedback(lateral):
    executor = PathExecutor(TrackingPolicy(), rolling_updates=True)
    executor.set_path(((0., 0., 0.), (1., 0., 0.)), identity=("goal", 1))
    result = executor.update(TrackingState(1., 0., 0., lateral_mps=lateral), .05)
    assert not result.command.complete
    assert result.command.linear_x_mps == result.command.angular_z_radps == 0.


@pytest.mark.parametrize("rolling", [False, True])
def test_final_position_loss_is_retained_in_rolling_and_legacy_modes(rolling):
    executor = PathExecutor(TrackingPolicy(), rolling_updates=rolling)
    executor.set_path(((0., 0., 0.), (1., 0., 1.)), identity=("goal", 1), final=True)
    assert executor.update(TrackingState(1., 0., 0.), .05).phase == "FINAL_ALIGN"
    result = executor.update(TrackingState(.5, 0., 0.), .05)
    assert result.phase == "BRAKING"
    assert not result.command.complete
    result = executor.update(TrackingState(.5, 0., 0.), .05)
    assert result.command.failure_reason == "GOAL_POSITION_LOST"
    assert not result.command.complete


@pytest.mark.parametrize("geometry", ["identical", "resampled_tail", "tangent_jitter"])
def test_rolling_initial_alignment_revisions_cannot_hide_stall(geometry):
    executor = PathExecutor(
        replace(TrackingPolicy(), no_progress_timeout_s=0.3), rolling_updates=True
    )
    for tick in range(20):
        if geometry == "resampled_tail":
            points = ((0, 0, 0), (2, 0, 0), (5 + tick, 0, 0))
        elif geometry == "tangent_jitter":
            points = ((0, 0, 0), (5, 0.1 * (-1) ** tick, 0))
        else:
            points = ((0, 0, 0), (5, 0, 0))
        executor.set_path(points, identity=("alignment", tick + 1), final=False)
        result = executor.update(TrackingState(0, 0, -1), 0.05)
    assert result.phase == "FAILED"
    assert result.command.failure_reason == "ALIGNMENT_STALLED"
    assert result.command.linear_x_mps == result.command.angular_z_radps == 0
    assert executor.identity == ("alignment", 20)


def test_rolling_initial_alignment_actual_yaw_progress_renews_budget():
    executor = PathExecutor(
        replace(TrackingPolicy(), no_progress_timeout_s=0.3), rolling_updates=True
    )
    for tick in range(18):
        executor.set_path(
            ((0, 0, 0), (5, 0, 0)), identity=("alignment", tick + 1), final=False
        )
        yaw = -1 + max(0, tick - 5) * 0.04
        result = executor.update(TrackingState(0, 0, yaw), 0.05)
        assert result.phase == "ALIGNING"
        assert result.command.failure_reason is None
        assert result.command.angular_z_radps > 0
    # 实测航向改善可以续期，但随后固定姿态仍应超时。
    for tick in range(18, 30):
        executor.set_path(
            ((0, 0, 0), (5, 0, 0)), identity=("alignment", tick + 1), final=False
        )
        result = executor.update(TrackingState(0, 0, yaw), 0.05)
    assert result.phase == "FAILED"
    assert result.command.failure_reason == "ALIGNMENT_STALLED"


def test_rolling_initial_alignment_new_tangent_brakes_then_gets_own_budget():
    executor = PathExecutor(
        replace(TrackingPolicy(), no_progress_timeout_s=0.3), rolling_updates=True
    )
    old_path = ((0, 0, 0), (5, 0, 0))
    new_path = ((0, 0, 0), (4, 3, 0))
    executor.set_path(old_path, identity=("alignment", 1), final=False)
    for _ in range(6):
        assert executor.update(TrackingState(0, 0, -0.5), 0.05).phase == "ALIGNING"
    executor.set_path(new_path, identity=("alignment", 2), final=False)
    result = executor.update(TrackingState(0, 0, -0.5, angular_radps=0.1), 0.05)
    assert result.phase == "BRAKING"
    assert result.command.linear_x_mps == result.command.angular_z_radps == 0
    # 新目标比原目标离当前航向更远，不能误用旧角误差而立即超时。
    for tick in range(3, 9):
        executor.set_path(new_path, identity=("alignment", tick), final=False)
        result = executor.update(TrackingState(0, 0, -0.5), 0.05)
        assert result.phase == "ALIGNING"
        assert result.command.angular_z_radps > 0
    for tick in range(9, 20):
        executor.set_path(new_path, identity=("alignment", tick), final=False)
        result = executor.update(TrackingState(0, 0, -0.5), 0.05)
    assert result.phase == "FAILED"
    assert result.command.failure_reason == "ALIGNMENT_STALLED"


def test_rolling_initial_alignment_timeout_brakes_latches_and_clear_restarts():
    executor = PathExecutor(
        replace(TrackingPolicy(), no_progress_timeout_s=0.3), rolling_updates=True
    )
    points = ((0, 0, 0), (5, 0, 0))
    executor.set_path(points, identity=("alignment", 1), final=False)
    for _ in range(6):
        executor.update(TrackingState(0, 0, -1), 0.05)
    executor.set_path(points, identity=("alignment", 2), final=False)
    assert executor.update(TrackingState(0, 0, -1), 0.05).phase == "ALIGNING"
    result = executor.update(TrackingState(0, 0, -1, angular_radps=0.1), 0.05)
    assert result.phase == "BRAKING"
    assert result.command.failure_reason is None
    assert result.command.linear_x_mps == result.command.angular_z_radps == 0
    executor.set_path(points, identity=("alignment", 3), final=False)
    result = executor.update(TrackingState(0, 0, -0.5, angular_radps=0.1), 0.05)
    assert result.phase == "BRAKING"
    assert result.command.failure_reason is None
    result = executor.update(TrackingState(0, 0, -0.5), 0.05)
    assert result.phase == "FAILED"
    assert result.command.failure_reason == "ALIGNMENT_STALLED"
    executor.clear()
    executor.set_path(points, identity=("alignment", 4), final=False)
    result = executor.update(TrackingState(0, 0, -0.5, angular_radps=0.1), 0.05)
    assert result.phase == "BRAKING"
    assert result.command.linear_x_mps == result.command.angular_z_radps == 0
    assert executor.update(TrackingState(0, 0, -0.5), 0.05).phase == "ALIGNING"


def test_new_alignment_session_gets_a_fresh_budget():
    executor = PathExecutor(
        replace(TrackingPolicy(), no_progress_timeout_s=0.3), rolling_updates=True
    )
    points = ((0, 0, 0), (5, 0, 0))
    executor.set_path(points, identity=("old", 1), final=False)
    for _ in range(6):
        executor.update(TrackingState(0, 0, -1), 0.05)
    executor.set_path(points, identity=("new", 1), final=False)
    for _ in range(6):
        result = executor.update(TrackingState(0, 0, -1), 0.05)
        assert result.phase == "ALIGNING"
    for _ in range(3):
        result = executor.update(TrackingState(0, 0, -1), 0.05)
    assert result.command.failure_reason == "ALIGNMENT_STALLED"


@pytest.mark.parametrize("yaw_sign", [-1, 1])
@pytest.mark.parametrize("first_tangent_sign", [-1, 1])
def test_rolling_alignment_threshold_jitter_latches_direction_and_times_out(
    yaw_sign, first_tangent_sign
):
    policy = replace(TrackingPolicy(), no_progress_timeout_s=0.3)
    executor = PathExecutor(policy, rolling_updates=True)
    yaw = yaw_sign * policy.reverse_heading_threshold_rad
    chosen_direction = None
    for tick in range(20):
        tangent = first_tangent_sign * 0.02 * (-1) ** tick
        points = ((0, 0, 0), (5 * math.cos(tangent), 5 * math.sin(tangent), 0))
        # 一个反馈周期内可以先收到多个 revision，方向状态也不能丢失。
        for revision in (2 * tick + 1, 2 * tick + 2):
            executor.set_path(points, identity=("threshold", revision), final=False)
        result = executor.update(TrackingState(0, 0, yaw), 0.05)
        if chosen_direction is None:
            chosen_direction = result.direction
        assert result.direction == chosen_direction
    assert result.phase == "FAILED"
    assert result.command.failure_reason == "ALIGNMENT_STALLED"
    assert result.command.linear_x_mps == result.command.angular_z_radps == 0
    assert executor.identity == ("threshold", 40)


def test_rolling_alignment_material_tangent_change_can_reselect_direction():
    executor = PathExecutor(
        replace(TrackingPolicy(), no_progress_timeout_s=0.3), rolling_updates=True
    )
    executor.set_path(((0, 0, 0), (5, 0, 0)), identity=("alignment", 1), final=False)
    for _ in range(6):
        result = executor.update(TrackingState(0, 0, -0.5), 0.05)
    assert result.direction == 1
    points = ((0, 0, 0), (5 * math.cos(2), 5 * math.sin(2), 0))
    executor.set_path(points, identity=("alignment", 2), final=False)
    result = executor.update(TrackingState(0, 0, -0.5, angular_radps=0.1), 0.05)
    assert result.phase == "BRAKING"
    assert result.direction == -1
    assert result.command.linear_x_mps == result.command.angular_z_radps == 0
    for revision in range(3, 9):
        executor.set_path(points, identity=("alignment", revision), final=False)
        result = executor.update(TrackingState(0, 0, -0.5), 0.05)
        assert result.phase == "ALIGNING"
        assert result.direction == -1
        assert result.command.angular_z_radps < 0
    for revision in range(9, 20):
        executor.set_path(points, identity=("alignment", revision), final=False)
        result = executor.update(TrackingState(0, 0, -0.5), 0.05)
    assert result.phase == "FAILED"
    assert result.command.failure_reason == "ALIGNMENT_STALLED"


@pytest.mark.parametrize("restart", ["clear", "new_session"])
def test_alignment_direction_is_reselected_after_episode_restart(restart):
    executor = PathExecutor(
        replace(TrackingPolicy(), no_progress_timeout_s=0.3), rolling_updates=True
    )
    yaw = executor.policy.reverse_heading_threshold_rad
    points = ((0, 0, 0), (5 * math.cos(0.02), 5 * math.sin(0.02), 0))
    executor.set_path(points, identity=("old", 1), final=False)
    assert executor.update(TrackingState(0, 0, yaw), 0.05).direction == 1
    if restart == "clear":
        executor.clear()
        identity = ("old", 2)
    else:
        identity = ("new", 1)
    points = ((0, 0, 0), (5 * math.cos(-0.02), 5 * math.sin(-0.02), 0))
    executor.set_path(points, identity=identity, final=False)
    result = executor.update(TrackingState(0, 0, yaw), 0.05)
    assert result.phase == "ALIGNING"
    assert result.direction == -1
    # 新会话仍保留命令斜率限制，允许首帧先从旧负命令减速到零。
    assert executor.update(TrackingState(0, 0, yaw), 0.05).command.angular_z_radps > 0
