#pragma once
// 由2026-11-14板端runtime.yaml和生产参数映射生成；只用于离线测试，不是飞行配置修改。
// 来源报告ZIP SHA256 ca2a2cc685995169526e08658d552103c4419bccb08c8cb2e7f1a4cf4a7eac4f。
inline fc01::Config captured_config() {
  fc01::Config result;
  auto& c = result.attitude;
  c.visual.enabled = false; // visual_navigation_enabled
  c.rc_navigation_enabled = true; // rc_navigation_enabled
  c.rc_target_height = 1.5; // rc_target_height
  c.rc_navigation_mode_slot = 1; // rc_navigation_mode_slot
  c.rc_arm_to_navigation_timeout = 8.0; // rc_arm_to_navigation_timeout
  c.flight_actions_enabled = true; // flight_actions_enabled
  c.bench_arm_only_enabled = false; // bench_arm_only_enabled
  c.arm_enabled = false; // arm_enabled
  c.takeoff_enabled = true; // takeoff_enabled
  c.parameters_confirmed = true; // flight_parameters_confirmed
  c.rc_takeover_confirmed = true; // rc_takeover_confirmed
  c.autonomous_failsafe_confirmed = false; // autonomous_failsafe_confirmed
  c.px4_land_fallback_confirmed = false; // px4_land_fallback_confirmed
  c.px4_land_evidence_id = ""; // px4_land_evidence_id
  c.feedback_guard.enabled = true; // aa_vertical_feedback_guard_enabled
  c.loop_hz = 20.0; // control_loop_hz
  c.armed_dwell_seconds = 1.0; // aa_armed_dwell_seconds
  c.feedback_guard.rate_tolerance_mps = 0.2; // aa_feedback_rate_tolerance_mps
  c.feedback_guard.dwell_seconds = 0.5; // aa_feedback_dwell_seconds
  c.liftoff_dwell_seconds = 0.2; // aa_liftoff_dwell_seconds
  c.liftoff_height_m = 0.12; // aa_liftoff_height_m
  c.liftoff_speed_mps = 0.08; // aa_liftoff_speed_mps
  c.touchdown_dwell_seconds = 0.3; // aa_touchdown_dwell_seconds
  c.height_lower_margin_m = 0.25; // aa_height_lower_margin_m
  c.z_jump_limit_m = 0.45; // aa_z_jump_limit_m
  c.vz_jump_limit_mps = 1.5; // aa_vz_jump_limit_mps
  c.prestream_seconds = 1.2; // aa_prestream_seconds
  c.ground_seconds = 2.0; // aa_ground_seconds
  c.stall_seconds = 0.3; // aa_stall_seconds
  c.attitude_timeout = 0.5; // aa_attitude_timeout
  c.local_timeout = 0.5; // aa_local_timeout
  c.status_timeout = 2.5; // aa_status_timeout
  c.land_timeout = 2.5; // aa_land_timeout
  c.land_sample_max_gap = 1.25; // aa_land_sample_max_gap
  c.flags_timeout = 2.5; // aa_flags_timeout
  c.console_timeout = 3.0; // aa_console_timeout
  c.command_timeout = 5.0; // aa_command_timeout
  c.ground_vz_tolerance = 0.1; // aa_ground_vz_tolerance
  c.max_tilt_deg = 10.0; // aa_max_tilt_deg
  c.bench_ground_down_tolerance_m = 0.5; // aa_bench_ground_down_tolerance_m
  c.bench_ground_vz_tolerance_mps = 0.25; // aa_bench_ground_vz_tolerance_mps
  c.vertical.hover = 0.5; // aa_hover_thrust
  c.vertical.minimum = 0.12; // aa_thrust_min
  c.vertical.maximum = 0.8; // aa_thrust_max
  c.vertical.slew = 0.4; // aa_thrust_slew
  c.vertical.height_gain = 0.5; // aa_height_gain
  c.vertical.velocity_gain = 0.15; // aa_velocity_gain
  c.vertical.integral_gain = 0.03; // aa_integral_gain
  c.vertical.integral_limit = 0.08; // aa_integral_limit
  c.vertical.max_height = 5.0; // aa_max_height
  c.vertical.max_climb = 0.3; // aa_max_climb
  c.vertical.max_descent = 0.2; // aa_max_descent
  c.ramp_seconds = 2.0; // aa_ramp_seconds
  c.settle_seconds = 2.0; // aa_settle_seconds
  c.landing_seconds = 3.0; // aa_landing_seconds
  c.takeoff_reserve_thrust = 0.04; // aa_takeoff_reserve_thrust
  c.height_tolerance = 0.15; // aa_height_tolerance
  c.speed_tolerance = 0.15; // aa_speed_tolerance
  c.ascent_timeout = 10.0; // aa_ascent_timeout
  c.descent_timeout = 10.0; // aa_descent_timeout
  c.recovery_seconds = 15.0; // aa_recovery_seconds
  c.px4_land_ack_seconds = 5.0; // aa_px4_land_ack_seconds
  c.saturation_seconds = 1.5; // aa_saturation_seconds
  c.max_flight_seconds = 30.0; // aa_max_flight_seconds
  result.profile = "attitude_altitude";
  result.runtime_mode = "flight-test";
  result.entry_version = "20261112.1";
  result.flight_enabled = false;
  result.validate();
  return result;
}
