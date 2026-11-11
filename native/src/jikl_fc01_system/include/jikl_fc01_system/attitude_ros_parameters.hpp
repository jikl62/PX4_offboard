#pragma once
// 姿态/高度配置入口：将 ROS 参数逐项写入 Core 的配置结构，不直接执行飞行动作。
// 本函数与 ros_config.hpp 共同供实际节点和安装前校验器使用，避免两者解释不同配置。
// 时间单位 s，高度/距离 m，速度 m/s，推力为归一化幅值；aa_* 保留既有名称和默认策略。
#include "jikl_fc01_system/attitude_altitude_core.hpp"
#include <rclcpp/rclcpp.hpp>
namespace fc01 {
inline attitude::Config attitude_parameters(rclcpp::Node& node){
  attitude::Config c;
  // 视觉、RC 起飞和动作门分开声明：打开视觉开关不能代替解锁/起飞/人工确认合同。
  c.visual.enabled=node.declare_parameter<bool>("visual_navigation_enabled",false);
  c.rc_navigation_enabled=node.declare_parameter<bool>("rc_navigation_enabled",false);
  c.rc_target_height=node.declare_parameter<double>("rc_target_height",c.rc_target_height);
  c.rc_navigation_mode_slot=node.declare_parameter<std::int64_t>("rc_navigation_mode_slot",6);
  c.rc_arm_to_navigation_timeout=node.declare_parameter<double>("rc_arm_to_navigation_timeout",8.);
  // 四类动作/确认参数各自独立，缺省拒绝；人工 RC 解锁路线仍由原 Core 检查。
  c.flight_actions_enabled=node.declare_parameter<bool>("flight_actions_enabled",false);
  c.bench_arm_only_enabled=node.declare_parameter<bool>("bench_arm_only_enabled",false);
  c.arm_enabled=node.declare_parameter<bool>("arm_enabled",false);
  c.takeoff_enabled=node.declare_parameter<bool>("takeoff_enabled",false);
  c.parameters_confirmed=node.declare_parameter<bool>("flight_parameters_confirmed",false);
  c.rc_takeover_confirmed=node.declare_parameter<bool>("rc_takeover_confirmed",false);
  c.autonomous_failsafe_confirmed=node.declare_parameter<bool>("autonomous_failsafe_confirmed",false);
  c.px4_land_fallback_confirmed=node.declare_parameter<bool>("px4_land_fallback_confirmed",false);
  c.px4_land_evidence_id=node.declare_parameter<std::string>("px4_land_evidence_id","");
  c.feedback_guard.enabled=node.declare_parameter<bool>("aa_vertical_feedback_guard_enabled",true);
  c.loop_hz=node.declare_parameter<double>("control_loop_hz",20.);
  // 表内每项把参数名绑定到对应 double 成员地址，循环只负责声明/读取，不改变物理量计算。
  struct Entry {const char* name;double* value;};
  for(const auto& e:{
      Entry{"visual_state_timeout",&c.visual.state_timeout},
      {"visual_command_timeout",&c.visual.command_timeout},
      {"visual_max_measurement_delay",&c.visual.max_measurement_delay},
      {"visual_future_tolerance",&c.visual.future_tolerance},
      {"visual_position_gain",&c.visual.position_gain},{"visual_velocity_gain",&c.visual.velocity_gain},
      {"visual_max_speed",&c.visual.max_speed},{"visual_max_acceleration",&c.visual.max_acceleration},
      {"visual_max_tilt_deg",&c.visual.max_tilt_deg},{"visual_max_yaw_rate",&c.visual.max_yaw_rate},
      {"visual_max_vertical_speed",&c.visual.max_vertical_speed},{"visual_max_radius",&c.visual.max_radius},
      {"visual_max_height",&c.visual.max_height},{"visual_min_quality",&c.visual.min_quality},
      {"visual_max_position_jump",&c.visual.max_position_jump},{"visual_max_velocity_jump",&c.visual.max_velocity_jump},
      // 以下是原起飞、离地证据、样本新鲜度、推力和降落/恢复包络；它们不是视觉目标参数。
      Entry{"aa_armed_dwell_seconds",&c.armed_dwell_seconds},
      {"aa_feedback_rate_tolerance_mps",&c.feedback_guard.rate_tolerance_mps},
      {"aa_feedback_dwell_seconds",&c.feedback_guard.dwell_seconds},
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
      {"aa_takeoff_reserve_thrust",&c.takeoff_reserve_thrust},
      {"aa_height_tolerance",&c.height_tolerance},{"aa_speed_tolerance",&c.speed_tolerance},
      {"aa_ascent_timeout",&c.ascent_timeout},{"aa_descent_timeout",&c.descent_timeout},
      {"aa_recovery_seconds",&c.recovery_seconds},{"aa_px4_land_ack_seconds",&c.px4_land_ack_seconds},
      {"aa_saturation_seconds",&c.saturation_seconds},{"aa_max_flight_seconds",&c.max_flight_seconds}})
    *e.value=node.declare_parameter<double>(e.name,*e.value);
  // 仅在显式高度下界为正时传递给垂向控制器；未设值沿用控制器原默认边界。
  c.vertical.ground_lower_margin=c.height_lower_margin_m>0?c.height_lower_margin_m:c.vertical.ground_lower_margin;
  return c;
}
} // namespace fc01
