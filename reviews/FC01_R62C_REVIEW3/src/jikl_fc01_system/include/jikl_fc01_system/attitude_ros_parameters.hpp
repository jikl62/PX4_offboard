#pragma once
#include "jikl_fc01_system/attitude_altitude_core.hpp"
#include <rclcpp/rclcpp.hpp>
namespace fc01 {
inline attitude::Config attitude_parameters(rclcpp::Node& node){
  attitude::Config c;
  c.flight_actions_enabled=node.declare_parameter<bool>("flight_actions_enabled",false);
  c.bench_arm_only_enabled=node.declare_parameter<bool>("bench_arm_only_enabled",false);
  c.arm_enabled=node.declare_parameter<bool>("arm_enabled",false);
  c.takeoff_enabled=node.declare_parameter<bool>("takeoff_enabled",false);
  c.parameters_confirmed=node.declare_parameter<bool>("flight_parameters_confirmed",false);
  c.rc_takeover_confirmed=node.declare_parameter<bool>("rc_takeover_confirmed",false);
  c.autonomous_failsafe_confirmed=node.declare_parameter<bool>("autonomous_failsafe_confirmed",false);
  c.px4_land_fallback_confirmed=node.declare_parameter<bool>("px4_land_fallback_confirmed",false);
  c.px4_land_evidence_id=node.declare_parameter<std::string>("px4_land_evidence_id","");
  c.loop_hz=node.declare_parameter<double>("control_loop_hz",20.);
  struct Entry {const char* name;double* value;};
  for(const auto& e:{
      Entry{"aa_armed_dwell_seconds",&c.armed_dwell_seconds},
      {"aa_liftoff_dwell_seconds",&c.liftoff_dwell_seconds},
      {"aa_liftoff_height_m",&c.liftoff_height_m},
      {"aa_liftoff_speed_mps",&c.liftoff_speed_mps},
      {"aa_touchdown_dwell_seconds",&c.touchdown_dwell_seconds},
      {"aa_height_lower_margin_m",&c.height_lower_margin_m},
      {"aa_z_jump_limit_m",&c.z_jump_limit_m},
      {"aa_vz_jump_limit_mps",&c.vz_jump_limit_mps},
      {"aa_prestream_seconds",&c.prestream_seconds},{"aa_ground_seconds",&c.ground_seconds},
      {"aa_stall_seconds",&c.stall_seconds},{"aa_attitude_timeout",&c.attitude_timeout},
      {"aa_local_timeout",&c.local_timeout},{"aa_status_timeout",&c.status_timeout},
      {"aa_land_timeout",&c.land_timeout},{"aa_land_sample_max_gap",&c.land_sample_max_gap},{"aa_flags_timeout",&c.flags_timeout},
      {"aa_console_timeout",&c.console_timeout},{"aa_command_timeout",&c.command_timeout},
      {"aa_ground_vz_tolerance",&c.ground_vz_tolerance},{"aa_max_tilt_deg",&c.max_tilt_deg},
      {"aa_bench_ground_down_tolerance_m",&c.bench_ground_down_tolerance_m},
      {"aa_bench_ground_vz_tolerance_mps",&c.bench_ground_vz_tolerance_mps},
      {"aa_hover_thrust",&c.vertical.hover},{"aa_thrust_min",&c.vertical.minimum},
      {"aa_thrust_max",&c.vertical.maximum},{"aa_thrust_slew",&c.vertical.slew},
      {"aa_height_gain",&c.vertical.height_gain},{"aa_velocity_gain",&c.vertical.velocity_gain},
      {"aa_integral_gain",&c.vertical.integral_gain},{"aa_integral_limit",&c.vertical.integral_limit},
      {"aa_max_height",&c.vertical.max_height},{"aa_max_climb",&c.vertical.max_climb},
      {"aa_max_descent",&c.vertical.max_descent},{"aa_ramp_seconds",&c.ramp_seconds},
      {"aa_settle_seconds",&c.settle_seconds},{"aa_landing_seconds",&c.landing_seconds},
      {"aa_height_tolerance",&c.height_tolerance},{"aa_speed_tolerance",&c.speed_tolerance},
      {"aa_ascent_timeout",&c.ascent_timeout},{"aa_descent_timeout",&c.descent_timeout},
      {"aa_recovery_seconds",&c.recovery_seconds},{"aa_px4_land_ack_seconds",&c.px4_land_ack_seconds},
      {"aa_saturation_seconds",&c.saturation_seconds},{"aa_max_flight_seconds",&c.max_flight_seconds}})
    *e.value=node.declare_parameter<double>(e.name,*e.value);
  c.vertical.ground_lower_margin=c.height_lower_margin_m>0?c.height_lower_margin_m:c.vertical.ground_lower_margin;
  return c;
}
} // namespace fc01
