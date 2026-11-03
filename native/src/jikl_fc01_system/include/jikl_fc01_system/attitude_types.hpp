#pragma once
#include "jikl_fc01_system/attitude_reference.hpp"
#include "jikl_fc01_system/vertical_controller.hpp"
#include "jikl_fc01_system/vertical_feedback_guard.hpp"
#include "jikl_fc01_system/authority.hpp"
#include "jikl_fc01_system/vision_navigation.hpp"
#include <cstdint>

namespace fc01::attitude {
struct Config {
  bool flight_actions_enabled{},arm_enabled{},takeoff_enabled{},parameters_confirmed{};
  bool rc_navigation_enabled{};
  double rc_target_height{1.5},rc_arm_to_navigation_timeout{8};
  std::int64_t rc_navigation_mode_slot{6};
  // Dedicated no-propeller engineering mode, disabled by default. Normal TAKEOFF is forbidden.
  bool bench_arm_only_enabled{};
  // Bench-only estimator grace. Upward height still uses the flight liftoff
  // threshold; these values cannot admit an ordinary TAKEOFF or touchdown.
  double bench_ground_down_tolerance_m{.50},bench_ground_vz_tolerance_mps{.25};
  bool rc_takeover_confirmed{},autonomous_failsafe_confirmed{};
  VerticalConfig vertical;
  VerticalFeedbackConfig feedback_guard;
  vision::Config visual;
  double prestream_seconds{1.2},ground_seconds{2},loop_hz{20},stall_seconds{.3};
  double attitude_timeout{.5},local_timeout{.5},status_timeout{2.5},land_timeout{2.5},flags_timeout{2.5};
  double console_timeout{3},command_timeout{5},ground_vz_tolerance{.1};
  double land_sample_max_gap{.5};
  // Flight-only supervised envelope; values MUST be validated on the actual airframe.
  double armed_dwell_seconds{-.1},liftoff_dwell_seconds{-.1},liftoff_height_m{-.1},liftoff_speed_mps{-.1};
  double touchdown_dwell_seconds{-.1},height_lower_margin_m{-.1},z_jump_limit_m{-.1},vz_jump_limit_mps{-.1};
  // Normal TAKEOFF only: bounded, time-ramped lift reserve above historical hover.
  // Zero preserves R6.3.1 behavior; value is NOT an airborne calibration.
  double takeoff_reserve_thrust{0.0};
  double ramp_seconds{-1},settle_seconds{-1},landing_seconds{-1},height_tolerance{-1},speed_tolerance{-1};
  double ascent_timeout{-1},descent_timeout{-1},saturation_seconds{-1},max_flight_seconds{-1},max_tilt_deg{-1};
  // Recovery completion deadline, not an output-authorization lease. One
  // budget per flight; LAND/console loss/shutdown/bounce cannot renew it.
  // Expiry must be disclosed, but cannot prove PX4 has taken airborne custody.
  double recovery_seconds{15},px4_land_ack_seconds{5};
  bool px4_land_fallback_confirmed{};
  std::string px4_land_evidence_id;
  void validate() const{
    visual.validate();
    if(visual.enabled&&(!rc_navigation_enabled||visual.max_tilt_deg>max_tilt_deg||
       visual.max_height>vertical.max_height||visual.max_vertical_speed>std::min(vertical.max_climb,vertical.max_descent)))
      throw std::invalid_argument("VISUAL_NAVIGATION_REQUIRES_RC_ROUTE_AND_EXISTING_ENVELOPE");
    if(rc_navigation_enabled&&(!flight_actions_enabled||arm_enabled||!takeoff_enabled||!parameters_confirmed||
       !rc_takeover_confirmed||autonomous_failsafe_confirmed||bench_arm_only_enabled||rc_navigation_mode_slot<1||rc_navigation_mode_slot>6||
       !std::isfinite(rc_target_height)||rc_target_height<=0||rc_target_height>=vertical.max_height||
       !std::isfinite(rc_arm_to_navigation_timeout)||rc_arm_to_navigation_timeout<1||rc_arm_to_navigation_timeout>10||
       max_flight_seconds>30||max_tilt_deg>10))throw std::invalid_argument("INVALID_MANUAL_RC_NAVIGATION_CONFIG");
    feedback_guard.validate();
    for(double value:{prestream_seconds,ground_seconds,loop_hz,stall_seconds,attitude_timeout,
        local_timeout,status_timeout,land_timeout,flags_timeout,console_timeout,command_timeout,ground_vz_tolerance,land_sample_max_gap,
        bench_ground_down_tolerance_m,bench_ground_vz_tolerance_mps})
      if(!std::isfinite(value)||value<=0)throw std::invalid_argument("INVALID_PREPARATION_PARAMETER");
    if(bench_ground_down_tolerance_m>.50||bench_ground_vz_tolerance_mps>.25)
      throw std::invalid_argument("BENCH_GROUND_GRACE_OUTSIDE_BOUNDED_LIMITS");
    if(land_sample_max_gap>=land_timeout)throw std::invalid_argument("INVALID_LAND_SAMPLE_GAP");
    if(!std::isfinite(recovery_seconds)||recovery_seconds<=0||recovery_seconds>60||
       !std::isfinite(px4_land_ack_seconds)||px4_land_ack_seconds<=0||px4_land_ack_seconds>5)
      throw std::invalid_argument("INVALID_BOUNDED_RECOVERY_POLICY");
    if(px4_land_fallback_confirmed&&px4_land_evidence_id.empty())
      throw std::invalid_argument("PX4_LAND_FALLBACK_NEEDS_EXTERNAL_EVIDENCE_ID");
    if(prestream_seconds<1.1||ground_seconds<2||loop_hz<20||loop_hz>50||command_timeout!=5)
      throw std::invalid_argument("PRESTREAM_REST_RATE_OR_ACK_TIMEOUT_INVALID");
    if(bench_arm_only_enabled&&(!flight_actions_enabled||!arm_enabled||takeoff_enabled||!parameters_confirmed))
      throw std::invalid_argument("ARM_ONLY_REQUIRES_EXCLUSIVE_GATES_TAKEOFF_DISABLED");
    if(flight_actions_enabled){  //flight_actions_enabled=false 时，不执行完整飞行动力参数校验。
      if(!parameters_confirmed)throw std::invalid_argument("FLIGHT_PARAMETERS_NOT_CONFIRMED_BY_OPERATOR");
      vertical.validate();  //设为 true 后，首先检查 flight_parameters_confirmed。随后执行 vertical.validate()。参数不满足要求时，节点直接拒绝启动。
      for(double value:{ramp_seconds,settle_seconds,landing_seconds,height_tolerance,speed_tolerance,
          ascent_timeout,descent_timeout,saturation_seconds,max_flight_seconds,max_tilt_deg,
          armed_dwell_seconds,liftoff_dwell_seconds,liftoff_height_m,liftoff_speed_mps,
          touchdown_dwell_seconds,height_lower_margin_m,z_jump_limit_m,vz_jump_limit_mps})
        if(!std::isfinite(value)||value<=0)throw std::invalid_argument("EXPLICIT_VALIDATED_FLIGHT_LIMITS_REQUIRED");
      if(!std::isfinite(takeoff_reserve_thrust)||takeoff_reserve_thrust<0||
         takeoff_reserve_thrust>.06||vertical.hover+takeoff_reserve_thrust>vertical.maximum)
        throw std::invalid_argument("INVALID_BOUNDED_TAKEOFF_RESERVE");
      if(armed_dwell_seconds<.1||liftoff_height_m>=vertical.max_height||
         height_lower_margin_m>=vertical.max_height||z_jump_limit_m>=vertical.max_height||
         touchdown_dwell_seconds<.1)throw std::invalid_argument("FLIGHT_OBSERVATION_ENVELOPE_INVALID");
      if(max_tilt_deg>=90)throw std::invalid_argument("MAX_TILT_MUST_BE_BELOW_90_DEGREES");
    }
  }
};
struct Feedback {
  vision::Input visual_input;
  Quaternion q{1,0,0,0};
  std::uint8_t quat_reset{},z_reset{},vz_reset{};
  double z{},vz{},stationary_seconds{},battery_voltage{NAN};
  std::uint64_t status_stamp{},land_stamp{};
  bool attitude_fresh{},local_fresh{},status_fresh{},land_fresh{},failsafe_fresh{};
  // Pure-Core fixtures have no host clock. The ROS adapter always overwrites
  // both fields from its checked source timestamps at every snapshot.
  bool source_clock_valid{true},source_clock_validated{};
  std::uint64_t source_host_stamp{}; // synchronized adapter clock; never armed_time
  bool local_sample_fresh{},attitude_sample_fresh{};
  bool rc_source_clock_valid{},rc_switch_clock_valid{};
  bool z_valid{},vz_valid{},armed{},landed{},at_rest{},offboard{},failsafe{},exclusive{},time_regressed{};
  bool calibration{},hil{},preflight{},battery_ready{},manual_ready{};
  bool stabilized_selectable{},stabilized_selected{};
  // manual_ready above is a legacy observation, never an authority predicate.
  bool rc_link_ready{},rc_switches_ready{},kill{},operator_takeover{},offboard_signal_lost{};
  // Observation diagnostics only. These do not replace the route predicates.
  bool rc_source_seen{},rc_source_valid{},rc_source_selected{},rc_sticks_moving{};
  bool rc_switches_seen{},rc_switches_fresh{};
  std::uint8_t rc_data_source{},rc_mode_slot{},rc_arm_switch{},rc_kill_switch{};
  bool rc_axes_safe{},rc_aux_switches_safe{};
  double rc_throttle{NAN};
  std::uint64_t rc_switch_sample_stamp{},armed_time{};
  std::uint8_t latest_arming_reason{};
  std::uint64_t operator_switch_generation{};
  std::uint64_t local_stamp{};
  bool xy_valid{},vxy_valid{},land_selectable{},auto_land_selected{};
  double z_deriv{NAN}; // PX4 down-position derivative, never a new altitude source
  std::uint64_t vertical_sample_stamp{}; // raw PX4 timestamp_sample, not message timestamp
};
inline authority::Inputs authority_inputs(const Config& c,const Feedback& s){
  return {s.rc_link_ready,s.rc_switches_ready,c.rc_takeover_confirmed,c.autonomous_failsafe_confirmed};
}
struct Transaction {
  bool accepted{},rejected{},timed_out{};
  // For reused Command 400, ACK cannot identify the operation. Runtime
  // supplies independent, CURRENT safe-ground PX4 evidence after final commit.
  bool ambiguous{},state_evidenced{};
};
// Arm-only has a disarmed ground anchor and can publish only exact-zero
// thrust. Motor vibration can prevent PX4 at_rest while it is armed. This
// observation is ONLY for that bench route, never for ordinary touchdown.
inline bool bench_vertical_observation(const Config& c,const Feedback& s,double z0){
  const double height=z0-s.z;
  return std::isfinite(z0)&&std::isfinite(s.z)&&std::isfinite(s.vz)&&
    height<=std::min(c.height_tolerance,c.liftoff_height_m)&&
    height>=-c.bench_ground_down_tolerance_m&&std::abs(s.vz)<=c.bench_ground_vz_tolerance_mps;
}
inline std::uint32_t bench_ground_failed_mask(const Config& c,const Feedback& s,double z0){
  const double height=z0-s.z;
  return (!s.armed?1U:0U)|(!s.offboard?2U:0U)|(!s.landed?4U:0U)|
    ((!std::isfinite(height)||height>std::min(c.height_tolerance,c.liftoff_height_m)||
      height< -c.bench_ground_down_tolerance_m)?8U:0U)|
    ((!std::isfinite(s.vz)||std::abs(s.vz)>c.bench_ground_vz_tolerance_mps)?16U:0U);
}
inline bool bench_ground_observation(const Config& c,const Feedback& s,double z0){
  return c.bench_arm_only_enabled&&!c.takeoff_enabled&&
    s.status_fresh&&s.land_fresh&&s.local_fresh&&s.z_valid&&s.vz_valid&&
    s.offboard&&s.landed&&bench_vertical_observation(c,s,z0);
}
// Post-stop observation only: an old barometric origin is not evidence that
// a DISARMED fixed bench has taken off. Callers must separately establish a
// new Disarmed status and a new at_rest dwell. This cannot authorize a 400
// command, Armed ground admission, TAKEOFF, or ordinary flight touchdown.
inline bool bench_disarmed_ground_observation(const Config& c,const Feedback& s){
  return c.bench_arm_only_enabled&&!c.takeoff_enabled&&!s.armed&&
    s.status_fresh&&s.land_fresh&&s.local_fresh&&s.z_valid&&s.vz_valid&&
    s.landed&&std::isfinite(s.z)&&std::isfinite(s.vz);
}
enum class Phase {Idle,SelfCheck,Prestream,RequestOffboard,OffboardReady,RequestArm,ArmedGround,BenchArmedGround,
  ThrustRamp,Ascend,HeightHold,Descent,RecoveryDescent,Px4LandPending,PoweredGround,Landed,RequestDisarm,SafeDisarmed,ReleaseOffboard,Safety,
  RcPrestream,RcWaitArm,RcWaitNavigation};
inline const char* name(Phase phase){
  switch(phase){
    case Phase::Idle:return "IDLE";case Phase::SelfCheck:return "SELF_CHECK";
    case Phase::Prestream:return "PRESTREAM";case Phase::RequestOffboard:return "REQUEST_OFFBOARD";
    case Phase::OffboardReady:return "OFFBOARD_READY";case Phase::RequestArm:return "REQUEST_ARM";
    case Phase::ArmedGround:return "ARMED_GROUND";case Phase::BenchArmedGround:return "BENCH_ARMED_GROUND";case Phase::ThrustRamp:return "THRUST_RAMP";
    case Phase::Ascend:return "ASCEND";case Phase::HeightHold:return "HEIGHT_HOLD";
    case Phase::Descent:return "DESCENT";case Phase::PoweredGround:return "POWERED_GROUND_UNCONFIRMED";case Phase::Landed:return "LANDED";
    case Phase::RecoveryDescent:return "RECOVERY_DESCENT";
    case Phase::Px4LandPending:return "PX4_LAND_PENDING";
    case Phase::RequestDisarm:return "REQUEST_DISARM";case Phase::SafeDisarmed:return "SAFE_DISARMED";
    case Phase::ReleaseOffboard:return "RELEASE_OFFBOARD";
    case Phase::Safety:return "SAFETY";
    case Phase::RcPrestream:return "RC_PRESTREAM";
    case Phase::RcWaitArm:return "RC_READY_WAIT_ARM";
    case Phase::RcWaitNavigation:return "RC_ARMED_WAIT_NAVIGATION";
  }return "UNKNOWN";
}
enum class Command {None,Offboard,Arm,Disarm,ReleaseOffboard,Px4Land};
enum class FaultClass {None,GroundAbort,ControlledDescent,Px4Handoff,UnknownAirborne};
inline const char* name(FaultClass f){switch(f){
  case FaultClass::None:return "NONE";case FaultClass::GroundAbort:return "GROUND_ABORT";
  case FaultClass::ControlledDescent:return "CONTROLLED_DESCENT";
  case FaultClass::Px4Handoff:return "PX4_HANDOFF";
  case FaultClass::UnknownAirborne:return "UNKNOWN_AIRBORNE";
}return "UNKNOWN_AIRBORNE";}
struct Output {
  bool visual_motion{};
  bool publish{};Quaternion q{1,0,0,0};std::array<double,3> thrust{};
  Command command{Command::None};double height{},speed{},target_height{},velocity_target{};
  bool saturated{},slew_limited{};
  bool vertical_closed_loop{},takeoff_thrust_limited{};
  double proportional_thrust{},integral_thrust{},raw_thrust{},thrust_ceiling{};
};
// Captured within the same Core::step as the ARMED_GROUND fault.  This is
// evidence only: it grants no output authority and changes no fault predicate.
struct ArmedGroundFaultEvidence {
  bool valid{};
  std::uint32_t failed_mask{}; // 1=armed, 2=offboard, 4=landed, 8=height, 16=vz
  bool armed{},offboard{},landed{};
  double z0{},z{},vz{},height{},up_speed{},height_limit{},vz_limit{};
  std::uint64_t status_stamp{},land_stamp{},local_stamp{};
  double downward_height_limit{}; // populated for bench faults only
};
// Immutable triggering-sample audit. This has no effect on the flight state machine.
// In particular it does not adjust the tilt threshold, attitude setpoint,
// motor output or Offboard heartbeat decision.
struct TiltFaultEvidence {
  bool valid{};
  double measured_tilt_deg{},limit_deg{},height{},up_speed{};
  Quaternion actual_q{1,0,0,0},requested_q{1,0,0,0};
  bool armed{},landed{},offboard{};
  std::uint8_t quat_reset{};
  std::uint64_t status_stamp{},land_stamp{},local_stamp{};
};
struct AuthorityGroundEvidence {
  bool valid{};std::uint32_t failed_mask{};
  double stationary_seconds{},vz{};
  std::uint64_t status_stamp{},land_stamp{},local_stamp{};
};
} // namespace fc01::attitude
