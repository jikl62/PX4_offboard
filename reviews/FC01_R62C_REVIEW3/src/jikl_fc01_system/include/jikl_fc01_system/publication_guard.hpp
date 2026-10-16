#pragma once
#include "jikl_fc01_system/attitude_altitude_core.hpp"
#include <string>

namespace fc01::attitude {
// Immutable per-publication view, sampled again by the ROS adapter at the sink.
// No ROS/DDS dependencies, side effects, clamping, or state transitions here.
struct BoundaryContext {
  std::string profile;
  Phase phase{Phase::Idle};
  Config config;
  Feedback feedback;
  bool shutdown{},session_connected{},operator_lease_live{},control_fresh{};
  bool preparation_lease_valid{true},flight_started{},reference_ready{};
  double reference_z{};
  std::uint8_t reference_z_reset{},reference_vz_reset{},reference_quat_reset{};
  authority::State authority;
  bool offboard_observed{};
  bool ground_confirmed{},arm_confirmed{},ground_release_pending{},arm_cancel_pending{};
  bool disarm_command_committed{};
  bool bench_arm_only_active{},bench_disarm_proven{},arm_command_committed{};
  std::uint64_t bench_arm_status_baseline{},bench_arm_local_baseline{};
  std::uint64_t bench_disarm_status_floor{},bench_disarm_local_floor{},bench_disarm_land_floor{};
  bool px4_land_command_committed{};
  std::uint64_t touchdown_baseline{};
  double yaw_reference{};
};
enum class BoundaryReason {
  Allowed,WrongProfile,Shutdown,SessionDisconnected,OperatorLease,ControlStall,
  OutputDisabled,InvalidOutput,StageOneNonzero,WrongOutputPhase,FlightActions,
  ArmEnabled,TakeoffEnabled,ParametersConfirmed,InvalidFlightConfig,ArmPhase,
  ArmDisarmed,ArmGround,FlightPhase,ArmedOffboard,FlightReference,InvalidThrust,
  Conflict,TelemetryStale,Failsafe,CalibrationHil,AttitudeInvalid,VerticalInvalid,
  NonfiniteTelemetry,TimeRegression,SourceClock,EstimatorReset,BatteryPreflight,FlightEnvelope,
  AuthorityUnset,AuthorityNotLocked,AuthorityChanged,OperatorTakeover,
  RcLink,RcUnconfirmed,RcTelemetry,AutonomousUnconfirmed,InvalidAuthority,CommandedAttitude,DisarmPhase,DisarmGround,
  ReleasePhase,DisarmTransaction,LandFallback
};
inline const char* name(BoundaryReason reason){
  switch(reason){
    case BoundaryReason::Allowed:return "ALLOWED";
    case BoundaryReason::WrongProfile:return "WRONG_PROFILE";
    case BoundaryReason::Shutdown:return "SHUTDOWN_LATCHED";
    case BoundaryReason::SessionDisconnected:return "OPERATOR_SESSION_DISCONNECTED";
    case BoundaryReason::OperatorLease:return "LIVE_OPERATOR_LEASE_REQUIRED";
    case BoundaryReason::ControlStall:return "CONTROL_LOOP_STALL";
    case BoundaryReason::OutputDisabled:return "OUTPUT_DISABLED";
    case BoundaryReason::InvalidOutput:return "INVALID_WIRE_OUTPUT";
    case BoundaryReason::StageOneNonzero:return "STAGE_ONE_NONZERO_THRUST";
    case BoundaryReason::WrongOutputPhase:return "OUTPUT_WRONG_PHASE";
    case BoundaryReason::FlightActions:return "FLIGHT_ACTIONS_DISABLED";
    case BoundaryReason::ArmEnabled:return "ARM_DISABLED";
    case BoundaryReason::TakeoffEnabled:return "TAKEOFF_DISABLED";
    case BoundaryReason::ParametersConfirmed:return "FLIGHT_PARAMETERS_NOT_CONFIRMED";
    case BoundaryReason::InvalidFlightConfig:return "INVALID_FLIGHT_CONFIGURATION";
    case BoundaryReason::ArmPhase:return "ARM_WRONG_PHASE";
    case BoundaryReason::ArmDisarmed:return "ARM_REQUIRES_DISARMED";
    case BoundaryReason::ArmGround:return "ARM_REQUIRES_OFFBOARD_GROUND_REFERENCE";
    case BoundaryReason::FlightPhase:return "NONZERO_WRONG_FLIGHT_PHASE";
    case BoundaryReason::ArmedOffboard:return "NONZERO_REQUIRES_ARMED_OFFBOARD";
    case BoundaryReason::FlightReference:return "FLIGHT_REFERENCE_REQUIRED";
    case BoundaryReason::InvalidThrust:return "INVALID_MULTICOPTER_THRUST";
    case BoundaryReason::Conflict:return "PUBLISHER_OR_TELEMETRY_CONFLICT";
    case BoundaryReason::TelemetryStale:return "DDS_STATUS_LAND_OR_FAILSAFE_STALE";
    case BoundaryReason::Failsafe:return "PX4_FAILSAFE";
    case BoundaryReason::CalibrationHil:return "CALIBRATION_OR_HIL_ACTIVE";
    case BoundaryReason::AttitudeInvalid:return "ATTITUDE_STALE_OR_INVALID";
    case BoundaryReason::VerticalInvalid:return "Z_VZ_STALE_OR_INVALID";
    case BoundaryReason::NonfiniteTelemetry:return "NONFINITE_VERTICAL_TELEMETRY";
    case BoundaryReason::TimeRegression:return "TELEMETRY_TIME_REGRESSION";
    case BoundaryReason::SourceClock:return "DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE";
    case BoundaryReason::EstimatorReset:return "ESTIMATOR_RESET_AFTER_CONTROL_SAMPLE";
    case BoundaryReason::BatteryPreflight:return "BATTERY_PREFLIGHT_REQUIRED";
    case BoundaryReason::FlightEnvelope:return "FLIGHT_ENVELOPE_EXCEEDED";
    case BoundaryReason::AuthorityUnset:return "AUTHORITY_MODE_UNSET";
    case BoundaryReason::AuthorityNotLocked:return "AUTHORITY_MODE_NOT_LOCKED";
    case BoundaryReason::AuthorityChanged:return "AUTHORITY_CHANGED_DURING_ATTEMPT";
    case BoundaryReason::OperatorTakeover:return "OPERATOR_TAKEOVER_LATCHED";
    case BoundaryReason::RcLink:return "RC_LINK_REQUIRED";
    case BoundaryReason::RcUnconfirmed:return "RC_TAKEOVER_CONTRACT_UNCONFIRMED";
    case BoundaryReason::RcTelemetry:return "RC_TAKEOVER_TELEMETRY_REQUIRED";
    case BoundaryReason::AutonomousUnconfirmed:return "AUTONOMOUS_FAILSAFE_UNCONFIRMED";
    case BoundaryReason::InvalidAuthority:return "INVALID_AUTHORITY_MODE";
    case BoundaryReason::CommandedAttitude:return "COMMAND_TARGET_NOT_LEVEL_LOCKED_YAW";
    case BoundaryReason::DisarmPhase:return "DISARM_WRONG_PHASE";
    case BoundaryReason::DisarmGround:return "DISARM_REQUIRES_NEW_FRESH_CONFIRMED_GROUND";
    case BoundaryReason::ReleasePhase:return "RELEASE_WRONG_PHASE_OR_GROUND";
    case BoundaryReason::DisarmTransaction:return "DISARM_ALREADY_COMMITTED_NO_RETRY";
    case BoundaryReason::LandFallback:return "PX4_LAND_FALLBACK_CONTRACT_OR_CURRENT_POSITION_UNVERIFIED";
  }return "UNKNOWN_DENIED";
}
struct Decision {bool allowed{};BoundaryReason reason{BoundaryReason::InvalidOutput};};
inline Decision deny(BoundaryReason reason){return {false,reason};}
inline Decision allow(){return {true,BoundaryReason::Allowed};}
inline Decision boundary_authority(const BoundaryContext& c){
  switch(authority::evaluate(c.authority,authority_inputs(c.config,c.feedback))){
    case authority::Reason::Allowed:return allow();
    case authority::Reason::Unset:return deny(BoundaryReason::AuthorityUnset);
    case authority::Reason::NotLocked:return deny(BoundaryReason::AuthorityNotLocked);
    case authority::Reason::Changed:return deny(BoundaryReason::AuthorityChanged);
    case authority::Reason::Takeover:return deny(BoundaryReason::OperatorTakeover);
    case authority::Reason::RcLink:return deny(BoundaryReason::RcLink);
    case authority::Reason::RcUnconfirmed:return deny(BoundaryReason::RcUnconfirmed);
    case authority::Reason::RcTelemetry:return deny(BoundaryReason::RcTelemetry);
    case authority::Reason::AutonomousUnconfirmed:return deny(BoundaryReason::AutonomousUnconfirmed);
    case authority::Reason::Invalid:return deny(BoundaryReason::InvalidAuthority);
  }return deny(BoundaryReason::InvalidAuthority);
}
inline bool stage_one(Phase p){return p==Phase::Idle||p==Phase::SelfCheck||p==Phase::Prestream||p==Phase::RequestOffboard||p==Phase::OffboardReady;}
inline bool nonzero(const std::array<double,3>& thrust){return thrust[0]!=0||thrust[1]!=0||thrust[2]!=0;}
inline Decision flight_gates(const Config& c){
  if(!c.flight_actions_enabled)return deny(BoundaryReason::FlightActions);
  if(!c.arm_enabled)return deny(BoundaryReason::ArmEnabled);
  if(!c.takeoff_enabled&&!c.bench_arm_only_enabled)return deny(BoundaryReason::TakeoffEnabled);
  if(!c.parameters_confirmed)return deny(BoundaryReason::ParametersConfirmed);
  try{c.validate();}catch(const std::invalid_argument&){return deny(BoundaryReason::InvalidFlightConfig);}
  return allow();
}
inline Decision boundary_health(const BoundaryContext& c){
  if(c.profile!="attitude_altitude")return deny(BoundaryReason::WrongProfile);
  if(c.shutdown&&!(c.phase==Phase::RequestArm&&c.arm_cancel_pending)&&c.phase!=Phase::Descent&&
     c.phase!=Phase::RecoveryDescent&&c.phase!=Phase::Px4LandPending&&c.phase!=Phase::PoweredGround&&c.phase!=Phase::Landed&&
     c.phase!=Phase::RequestDisarm&&c.phase!=Phase::ReleaseOffboard&&
     !(c.phase==Phase::RequestOffboard&&c.ground_release_pending&&
       !c.feedback.armed&&c.feedback.landed&&c.feedback.at_rest))
    return deny(BoundaryReason::Shutdown);
  if(!c.control_fresh)return deny(BoundaryReason::ControlStall);
  const auto& s=c.feedback;
  if(c.authority.takeover_latched||s.kill||s.operator_takeover||
     ((c.offboard_observed||c.flight_started||c.phase==Phase::OffboardReady)&&s.offboard_signal_lost))
    return deny(BoundaryReason::OperatorTakeover);
  if(c.authority.locked&&c.authority.mode!=c.authority.attempt_mode)return deny(BoundaryReason::AuthorityChanged);
  if(s.time_regressed)return deny(BoundaryReason::TimeRegression);
  if(!s.source_clock_valid)return deny(BoundaryReason::SourceClock);
  if(c.reference_ready&&(c.flight_started||s.armed)&&
     (s.z_reset!=c.reference_z_reset||s.vz_reset!=c.reference_vz_reset||
      s.quat_reset!=c.reference_quat_reset))return deny(BoundaryReason::EstimatorReset);
  if(!s.exclusive)return deny(BoundaryReason::Conflict);
  if(!s.status_fresh||!s.land_fresh||!s.failsafe_fresh)return deny(BoundaryReason::TelemetryStale);
  if(s.failsafe)return deny(BoundaryReason::Failsafe);
  if(s.calibration||s.hil)return deny(BoundaryReason::CalibrationHil);
  if(!s.attitude_fresh||!normalized(s.q))return deny(BoundaryReason::AttitudeInvalid);
  if(!s.local_fresh||!s.z_valid||!s.vz_valid)return deny(BoundaryReason::VerticalInvalid);
  if(!std::isfinite(s.z)||!std::isfinite(s.vz)||!std::isfinite(s.stationary_seconds))return deny(BoundaryReason::NonfiniteTelemetry);
  return allow();
}
inline Decision arm_publication_policy(const BoundaryContext& c){
  const auto healthy=boundary_health(c);if(!healthy.allowed)return healthy;
  const auto gates=flight_gates(c.config);if(!gates.allowed)return gates;
  const auto authority=boundary_authority(c);if(!authority.allowed)return authority;
  if(c.phase!=Phase::RequestArm)return deny(BoundaryReason::ArmPhase);
  if(!c.session_connected)return deny(BoundaryReason::SessionDisconnected);
  if(!c.operator_lease_live)return deny(BoundaryReason::OperatorLease);
  const auto& s=c.feedback;
  if(s.armed)return deny(BoundaryReason::ArmDisarmed);
  if(!c.flight_started||!c.reference_ready||!s.offboard||!s.landed||!s.at_rest||
     s.stationary_seconds<c.config.ground_seconds||std::abs(s.vz)>c.config.ground_vz_tolerance)
    return deny(BoundaryReason::ArmGround);
  if(!s.battery_ready||!s.preflight)return deny(BoundaryReason::BatteryPreflight);
  return allow();
}
inline Decision disarm_publication_policy(const BoundaryContext& c){
  if(c.phase!=Phase::RequestDisarm)return deny(BoundaryReason::DisarmPhase);
  // rclcpp::publish is a local commit, not proof of PX4 receipt/execution.
  // A second routine Disarm cannot be justified by an uncorrelated error ACK.
  if(c.disarm_command_committed)return deny(BoundaryReason::DisarmTransaction);
  const auto health=boundary_health(c);if(!health.allowed)return health;
  const auto gates=flight_gates(c.config);if(!gates.allowed)return gates;
  const auto authority=boundary_authority(c);if(!authority.allowed)return authority;
  const auto& s=c.feedback;
  const bool bench=c.config.bench_arm_only_enabled&&c.bench_arm_only_active&&
    c.bench_disarm_proven&&c.arm_command_committed&&
    s.status_stamp>c.bench_arm_status_baseline&&s.local_stamp>c.bench_arm_local_baseline&&
    s.status_stamp>=c.bench_disarm_status_floor&&s.local_stamp>=c.bench_disarm_local_floor&&
    s.land_stamp>=c.bench_disarm_land_floor&&
    bench_ground_observation(c.config,s,c.reference_z);
  if((c.config.bench_arm_only_enabled||c.bench_arm_only_active)&&!bench)
    return deny(BoundaryReason::DisarmGround);
  if(!c.flight_started||!c.arm_confirmed||!c.reference_ready||!c.ground_confirmed||
     !s.armed||!s.offboard||!s.landed||(!s.at_rest&&!bench)||
     s.land_stamp<=c.touchdown_baseline||
     !std::isfinite(c.reference_z)||
     (!bench&&(std::abs(c.reference_z-s.z)>c.config.height_tolerance||
       std::abs(s.vz)>c.config.speed_tolerance)))
    return deny(BoundaryReason::DisarmGround);
  return allow();
}
inline Decision land_fallback_publication_policy(const BoundaryContext& c){
  const auto health=boundary_health(c);if(!health.allowed)return health;
  const auto gates=flight_gates(c.config);if(!gates.allowed)return gates;
  const auto auth=boundary_authority(c);if(!auth.allowed)return auth;
  const auto& s=c.feedback;
  if(c.phase!=Phase::Px4LandPending||!c.flight_started||!c.arm_confirmed||c.px4_land_command_committed||
     !c.config.px4_land_fallback_confirmed||c.config.px4_land_evidence_id.empty()||
     !s.armed||!s.offboard||!s.local_fresh||!s.xy_valid||!s.vxy_valid||!s.land_selectable||
     !s.battery_ready)return deny(BoundaryReason::LandFallback);
  return allow();
}
// Only a previously UNARMED ground preparation may treat stale z0 drift as
// irrelevant to releasing Offboard. The PX4 status and land detector must still
// say disarmed/landed/at_rest, and boundary_health checks current Z/Vz, resets
// of time, telemetry freshness and sole publication ownership. Flight paths
// retain the original reference-height requirement.
inline bool unarmed_preparation_release(const BoundaryContext& c){
  return !c.flight_started&&!c.arm_confirmed;
}
// A fresh external mode change can overtake a planned Command 176 between
// Core::step and the final ROS sink. Cancel, never publish a second mode
// request into a PX4 mode that is already under operator control.
inline bool external_ground_mode_departure(const BoundaryContext& c){
  const auto& s=c.feedback;
  return c.profile=="attitude_altitude"&&c.phase==Phase::ReleaseOffboard&&
    unarmed_preparation_release(c)&&c.offboard_observed&&
    s.status_fresh&&s.land_fresh&&s.local_fresh&&s.failsafe_fresh&&
    s.exclusive&&!s.time_regressed&&!s.failsafe&&!s.kill&&
    !s.armed&&!s.offboard&&s.landed&&s.at_rest&&
    s.z_valid&&s.vz_valid&&std::isfinite(s.z)&&std::isfinite(s.vz);
}
inline Decision release_publication_policy(const BoundaryContext& c){
  if(c.phase!=Phase::ReleaseOffboard)return deny(BoundaryReason::ReleasePhase);
  const auto health=boundary_health(c);if(!health.allowed)return health;
  const auto& s=c.feedback;
  // Stage-one ground release must work even with all flight gates closed and
  // flight-only envelope parameters unset. Use the prepared ground tolerance.
  const double ground_height_tolerance=c.config.height_tolerance>0?
    c.config.height_tolerance:.30;
  const double ground_speed_tolerance=c.config.speed_tolerance>0?
    c.config.speed_tolerance:c.config.ground_vz_tolerance;
  const bool bench=c.config.bench_arm_only_enabled&&c.bench_arm_only_active&&c.arm_confirmed&&
    !c.config.takeoff_enabled;
  const bool vertical=bench?bench_vertical_observation(c.config,s,c.reference_z):
    ((unarmed_preparation_release(c)||std::abs(c.reference_z-s.z)<=ground_height_tolerance)&&
     std::abs(s.vz)<=ground_speed_tolerance);
  if(!s.status_fresh||!s.land_fresh||s.armed||!s.landed||!s.at_rest||
     !s.offboard||!s.stabilized_selectable||!c.reference_ready||
     !std::isfinite(c.reference_z)||
     !vertical)return deny(BoundaryReason::ReleasePhase);
  return allow();
}
inline Decision attitude_publication_policy(const BoundaryContext& c,const Output& output){
  if(!output.publish)return deny(BoundaryReason::OutputDisabled);
  for(double x:output.thrust)if(!std::isfinite(x)||x<-1||x>1)return deny(BoundaryReason::InvalidOutput);
  // Evaluate the original doubles: underflow at float conversion must never
  // conceal an upstream stage-one error. Reject, do not silently clamp.
  const bool thrust_nonzero=nonzero(output.thrust);
  if((stage_one(c.phase)||c.config.bench_arm_only_enabled)&&thrust_nonzero)
    return deny(BoundaryReason::StageOneNonzero);
  if(!normalized(output.q))return deny(BoundaryReason::InvalidOutput);
  const auto healthy=boundary_health(c);if(!healthy.allowed)return healthy;
  switch(c.phase){
    case Phase::Prestream:case Phase::RequestOffboard:case Phase::OffboardReady:
    case Phase::RequestArm:case Phase::ArmedGround:case Phase::BenchArmedGround:case Phase::ThrustRamp:case Phase::Ascend:
    case Phase::HeightHold:case Phase::Descent:case Phase::RecoveryDescent:case Phase::Px4LandPending:
    case Phase::PoweredGround:case Phase::Landed:case Phase::RequestDisarm:
    case Phase::ReleaseOffboard:break;
    default:return deny(BoundaryReason::WrongOutputPhase);
  }
  if(!c.reference_ready)return deny(BoundaryReason::FlightReference);
  // Common final sink gate: no ZERO thrust, stage-one, or release bypass of
  // target attitude/yaw reference. Exact-double nonzero test remains above.
  const auto commanded=normalized(output.q);
  if(!commanded||!std::isfinite(c.yaw_reference))return deny(BoundaryReason::CommandedAttitude);
  const Quaternion level{std::cos(c.yaw_reference/2),0,0,std::sin(c.yaw_reference/2)};
  double dot=0;for(unsigned k=0;k<4;++k)dot+=(*commanded)[k]*level[k];
  const double angle=2*std::acos(std::clamp(std::abs(dot),0.,1.));
  if(!std::isfinite(angle)||angle>2.*3.14159265358979323846/180.)
    return deny(BoundaryReason::CommandedAttitude);
  if(c.phase==Phase::ReleaseOffboard){
    if(thrust_nonzero)return deny(BoundaryReason::StageOneNonzero);
    // A disarmed-ground release retains zero heartbeat until PX4 switches
    // modes. This is NOT an in-flight setpoint bypass; flight gates are not
    // required here because release also serves closed-gate preparation.
    return release_publication_policy(c);
  }
  if(stage_one(c.phase)){
    const bool cancelled_ground_mode=c.phase==Phase::RequestOffboard&&c.ground_release_pending&&
      unarmed_preparation_release(c)&&!c.feedback.armed&&c.feedback.landed&&c.feedback.at_rest;
    if(!c.session_connected&&!cancelled_ground_mode)return deny(BoundaryReason::SessionDisconnected);
    if(!c.preparation_lease_valid)return deny(BoundaryReason::OperatorLease);
    if(c.feedback.armed)return deny(BoundaryReason::ArmDisarmed);
    if(c.phase==Phase::OffboardReady&&!c.feedback.offboard)return deny(BoundaryReason::ArmedOffboard);
    return allow(); // Four closed flight gates do not block zero-thrust prepare.
  }
  const auto gates=flight_gates(c.config);if(!gates.allowed)return gates;
  const auto authority=boundary_authority(c);if(!authority.allowed)return authority;
  if(!c.flight_started||!c.feedback.offboard)return deny(BoundaryReason::FlightReference);
  if(!thrust_nonzero)return allow(); // RequestArm/ArmedGround/RequestDisarm use zero.
  if(c.phase!=Phase::ThrustRamp&&c.phase!=Phase::Ascend&&c.phase!=Phase::HeightHold&&c.phase!=Phase::Descent&&
     c.phase!=Phase::RecoveryDescent&&c.phase!=Phase::Px4LandPending&&c.phase!=Phase::PoweredGround&&c.phase!=Phase::Landed)
    return deny(BoundaryReason::FlightPhase);
  if(!c.feedback.armed)return deny(BoundaryReason::ArmedOffboard);
  if(output.thrust[0]!=0||output.thrust[1]!=0||output.thrust[2]>0||-output.thrust[2]>c.config.vertical.maximum)
    return deny(BoundaryReason::InvalidThrust);
  if(!c.feedback.battery_ready)return deny(BoundaryReason::BatteryPreflight);
  if(c.phase!=Phase::Descent&&c.phase!=Phase::RecoveryDescent&&c.phase!=Phase::Px4LandPending&&
     c.phase!=Phase::PoweredGround&&c.phase!=Phase::Landed){
    if(!c.session_connected)return deny(BoundaryReason::SessionDisconnected);
    if(!c.operator_lease_live)return deny(BoundaryReason::OperatorLease);
  }
  const auto q=normalized(c.feedback.q);
  const double cosine=std::clamp(1-2*((*q)[1]*(*q)[1]+(*q)[2]*(*q)[2]),-1.,1.);
  if(!std::isfinite(c.reference_z)||c.reference_z-c.feedback.z>c.config.vertical.max_height||
     c.reference_z-c.feedback.z< -c.config.height_lower_margin_m||
     std::acos(cosine)*180/3.14159265358979323846>c.config.max_tilt_deg)return deny(BoundaryReason::FlightEnvelope);
  return allow();
}
inline std::string flight_gates_json(const Config& c){
  auto boolean=[](bool b){return b?"true":"false";};
  return std::string("\"flight_actions_enabled\":")+boolean(c.flight_actions_enabled)+
    ",\"arm_enabled\":"+boolean(c.arm_enabled)+",\"takeoff_enabled\":"+boolean(c.takeoff_enabled)+
    ",\"flight_parameters_confirmed\":"+boolean(c.parameters_confirmed)+
    ",\"bench_arm_only_enabled\":"+boolean(c.bench_arm_only_enabled);
}
} // namespace fc01::attitude
