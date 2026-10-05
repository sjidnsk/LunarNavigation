// Frozen benchmark capabilities copied from the 2026-09-24 synthetic terrain protocol.
// These are experiment fixtures, not a claim of real vehicle validation.
#pragma once
using namespace lunar::incremental_navigation;
WheeledCapability MakeComparisonWheel(){ WheeledCapability c;
c.wheel_diameter_m = 0.319;
c.wheel_width_m = 0.148;
c.wheelbase_m = 0.8175;
c.track_width_m = 0.67;
c.minimum_underbody_clearance_m = 0.21;
c.maximum_local_obstacle_relief_m = 0.2;
c.allow_unsupported_gap = false;
c.maximum_forward_speed_mps = 0.2;
c.maximum_reverse_speed_mps = 0.2;
c.maximum_spin_rate_radps = 1.0;
c.maximum_acceleration_mps2 = 0.5;
c.maximum_braking_deceleration_mps2 = 0.5;
c.maximum_yaw_acceleration_radps2 = 0.5;
c.maximum_lateral_acceleration_mps2 = 0.5;
c.maximum_curvature_per_m = 1.0;
c.maximum_slope_rad = 0.3490658503988659;
c.minimum_clearance_m = 0.2;
c.body_extent_m = {1.182,0.818,1.29996};
c.footprint_xy_m = {{0.591,0.409},{0.591,-0.409},{-0.591,-0.409},{-0.591,0.409}};
{WheelMotionPrimitive p;p.primitive_id="forward";p.relative_end_pose.orientation={.w=1.0,.z=0.0};p.kind=WheelPrimitiveKind::kForward;p.relative_end_pose.position_m={0.2,0.0,0.0};c.motion_primitives.push_back(p);}
{WheelMotionPrimitive p;p.primitive_id="reverse";p.relative_end_pose.orientation={.w=1.0,.z=0.0};p.kind=WheelPrimitiveKind::kReverse;p.relative_end_pose.position_m={-0.2,0.0,0.0};c.motion_primitives.push_back(p);}
{WheelMotionPrimitive p;p.primitive_id="forward-arc-left";p.relative_end_pose.orientation={.w=0.9951847266721969,.z=0.0980171403295606};p.kind=WheelPrimitiveKind::kForwardArc;p.relative_end_pose.position_m={0.19509032201612825,0.01921471959676957,0.0};c.motion_primitives.push_back(p);}
{WheelMotionPrimitive p;p.primitive_id="forward-arc-right";p.relative_end_pose.orientation={.w=0.9951847266721969,.z=-0.0980171403295606};p.kind=WheelPrimitiveKind::kForwardArc;p.relative_end_pose.position_m={0.19509032201612825,-0.01921471959676957,0.0};c.motion_primitives.push_back(p);}
{WheelMotionPrimitive p;p.primitive_id="reverse-arc-left";p.relative_end_pose.orientation={.w=0.9951847266721969,.z=-0.0980171403295606};p.kind=WheelPrimitiveKind::kReverseArc;p.relative_end_pose.position_m={-0.19509032201612825,0.01921471959676957,0.0};c.motion_primitives.push_back(p);}
{WheelMotionPrimitive p;p.primitive_id="reverse-arc-right";p.relative_end_pose.orientation={.w=0.9951847266721969,.z=0.0980171403295606};p.kind=WheelPrimitiveKind::kReverseArc;p.relative_end_pose.position_m={-0.19509032201612825,-0.01921471959676957,0.0};c.motion_primitives.push_back(p);}
{WheelMotionPrimitive p;p.primitive_id="spin-left";p.relative_end_pose.orientation={.w=0.9951847266721969,.z=0.0980171403295606};p.kind=WheelPrimitiveKind::kSpinCounterclockwise;p.relative_end_pose.position_m={0.0,0.0,0.0};c.motion_primitives.push_back(p);}
{WheelMotionPrimitive p;p.primitive_id="spin-right";p.relative_end_pose.orientation={.w=0.9951847266721969,.z=-0.0980171403295606};p.kind=WheelPrimitiveKind::kSpinClockwise;p.relative_end_pose.position_m={0.0,0.0,0.0};c.motion_primitives.push_back(p);}
{WheelMotionPrimitive p;p.primitive_id="stop-switch";p.relative_end_pose.orientation={.w=1.0,.z=0.0};p.kind=WheelPrimitiveKind::kStopAndSwitch;p.relative_end_pose.position_m={0.0,0.0,0.0};c.motion_primitives.push_back(p);}
return c;}
LeggedCapability MakeComparisonLegged(){ LeggedCapability c;
c.nominal_body_height_m = 0.33;
c.platform_mass_kg = 15.89;
c.nominal_payload_kg = 8.0;
c.maximum_payload_kg = 10.0;
c.maximum_linear_acceleration_mps2 = 1.0;
c.maximum_yaw_acceleration_radps2 = 1.0;
c.maximum_slope_rad = 0.5235987755982988;
c.maximum_step_height_m = 0.5;
c.maximum_gap_width_m = 0.3;
c.minimum_body_clearance_m = 0.3;
c.step_vertical_rate_mps = 0.1;
c.unknown_is_traversable = false;
c.body_extent_m = {0.68,0.33,0.35};
c.body_height_m={.28,.38};
c.forward_speed_mps={-1.5,1.5};
c.lateral_speed_mps={-.8,.8};
c.yaw_rate_radps={-1.,1.};
c.motion_primitives.push_back({.primitive_id="forward",.kind=LeggedPrimitiveKind::kForward,.body_frame_displacement_m={0.2,0.0,0.0},.yaw_change_rad=0.0});
c.motion_primitives.push_back({.primitive_id="backward",.kind=LeggedPrimitiveKind::kBackward,.body_frame_displacement_m={-0.2,0.0,0.0},.yaw_change_rad=0.0});
c.motion_primitives.push_back({.primitive_id="lateral-left",.kind=LeggedPrimitiveKind::kLateralLeft,.body_frame_displacement_m={0.0,0.2,0.0},.yaw_change_rad=0.0});
c.motion_primitives.push_back({.primitive_id="lateral-right",.kind=LeggedPrimitiveKind::kLateralRight,.body_frame_displacement_m={0.0,-0.2,0.0},.yaw_change_rad=0.0});
c.motion_primitives.push_back({.primitive_id="spin-left",.kind=LeggedPrimitiveKind::kSpin,.body_frame_displacement_m={0.0,0.0,0.0},.yaw_change_rad=0.19634954084936207});
c.motion_primitives.push_back({.primitive_id="spin-right",.kind=LeggedPrimitiveKind::kSpin,.body_frame_displacement_m={0.0,0.0,0.0},.yaw_change_rad=-0.19634954084936207});
return c;}