"""Joint speed/yaw slew limits for a fixed tracking curvature."""


def tracking_speed(
    curvature, desired_speed, measured_speed, measured_yaw_rate, dt, policy
):
    """Return feasible travel speed, or None when stopping is required.

    Intersect translational and angular acceleration intervals; independent
    clipping of omega would change the intended curvature.
    """
    lower = max(0.0, measured_speed - policy.max_linear_decel_mps2 * dt)
    upper = min(desired_speed, measured_speed + policy.max_linear_accel_mps2 * dt)
    yaw_lower = measured_yaw_rate - policy.max_angular_accel_radps2 * dt
    yaw_upper = measured_yaw_rate + policy.max_angular_accel_radps2 * dt
    if abs(curvature) <= 1e-9:
        if not yaw_lower <= 0.0 <= yaw_upper:
            return None
    else:
        a, b = sorted((yaw_lower / curvature, yaw_upper / curvature))
        lower, upper = max(lower, a), min(upper, b)
    return max(0.0, upper) if lower <= upper else None


def rolling_tracking_command(
    curvature, desired_speed, measured_speed, measured_yaw_rate, dt, policy
):
    """Approach a replacement path's curvature within the existing limits.

    Prefer exact curvature. When its yaw slew is infeasible, transition at a
    feasible travel speed instead of stopping for a small heading change.
    The caller still owns geometric deviation, corners and direction changes.
    """
    speed = tracking_speed(
        curvature, desired_speed, measured_speed, measured_yaw_rate, dt, policy
    )
    if speed is not None:
        return speed, speed * curvature
    lower = max(0.0, measured_speed - policy.max_linear_decel_mps2 * dt)
    speed = min(desired_speed, measured_speed + policy.max_linear_accel_mps2 * dt)
    if speed < lower or speed <= 1e-9:
        return None
    yaw_limit = min(
        policy.max_angular_radps,
        speed * policy.max_curvature_per_m,
        policy.max_lateral_accel_mps2 / speed,
    )
    yaw_lower = max(-yaw_limit, measured_yaw_rate - policy.max_angular_accel_radps2 * dt)
    yaw_upper = min(yaw_limit, measured_yaw_rate + policy.max_angular_accel_radps2 * dt)
    if yaw_lower > yaw_upper:
        return None
    return speed, min(yaw_upper, max(yaw_lower, speed * curvature))
