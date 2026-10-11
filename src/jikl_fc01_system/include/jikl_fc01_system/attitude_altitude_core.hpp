#pragma once
#include "jikl_fc01_system/attitude_reference.hpp"
#include "jikl_fc01_system/vertical_controller.hpp"
#include "jikl_fc01_system/authority.hpp"
#include <memory>

namespace fc01::attitude {
struct Config {
  bool flight_actions_enabled{},arm_enabled{},takeoff_enabled{},parameters_confirmed{};
  bool rc_takeover_confirmed{},autonomous_failsafe_confirmed{};
  VerticalConfig vertical;
  double prestream_seconds{1.2},ground_seconds{2},loop_hz{20},stall_seconds{.3};
  double attitude_timeout{.5},local_timeout{.5},status_timeout{2.5},land_timeout{2.5},flags_timeout{2.5};
  double console_timeout{3},command_timeout{5},ground_vz_tolerance{.1};
  double land_sample_max_gap{.5};
  // Flight-only supervised envelope; values MUST be validated on the actual airframe.
  double armed_dwell_seconds{-.1},liftoff_dwell_seconds{-.1},liftoff_height_m{-.1},liftoff_speed_mps{-.1};
  double touchdown_dwell_seconds{-.1},height_lower_margin_m{-.1},z_jump_limit_m{-.1},vz_jump_limit_mps{-.1};
  double ramp_seconds{-1},settle_seconds{-1},landing_seconds{-1},height_tolerance{-1},speed_tolerance{-1};
  double ascent_timeout{-1},descent_timeout{-1},saturation_seconds{-1},max_flight_seconds{-1},max_tilt_deg{-1};
  // Recovery completion deadline, not an output-authorization lease. One
  // budget per flight; LAND/console loss/shutdown/bounce cannot renew it.
  // Expiry must be disclosed, but cannot prove PX4 has taken airborne custody.
  double recovery_seconds{15},px4_land_ack_seconds{5};
  bool px4_land_fallback_confirmed{};
  std::string px4_land_evidence_id;
  void validate() const{
    for(double value:{prestream_seconds,ground_seconds,loop_hz,stall_seconds,attitude_timeout,
        local_timeout,status_timeout,land_timeout,flags_timeout,console_timeout,command_timeout,ground_vz_tolerance,land_sample_max_gap})
      if(!std::isfinite(value)||value<=0)throw std::invalid_argument("INVALID_PREPARATION_PARAMETER");
    if(land_sample_max_gap>=land_timeout)throw std::invalid_argument("INVALID_LAND_SAMPLE_GAP");
    if(!std::isfinite(recovery_seconds)||recovery_seconds<=0||recovery_seconds>60||
       !std::isfinite(px4_land_ack_seconds)||px4_land_ack_seconds<=0||px4_land_ack_seconds>5)
      throw std::invalid_argument("INVALID_BOUNDED_RECOVERY_POLICY");
    if(px4_land_fallback_confirmed&&px4_land_evidence_id.empty())
      throw std::invalid_argument("PX4_LAND_FALLBACK_NEEDS_EXTERNAL_EVIDENCE_ID");
    if(prestream_seconds<1.1||ground_seconds<2||loop_hz<20||loop_hz>50||command_timeout!=5)
      throw std::invalid_argument("PRESTREAM_REST_RATE_OR_ACK_TIMEOUT_INVALID");
    if(flight_actions_enabled){  //flight_actions_enabled=false 时，不执行完整飞行动力参数校验。
      if(!parameters_confirmed)throw std::invalid_argument("FLIGHT_PARAMETERS_NOT_CONFIRMED_BY_OPERATOR");
      vertical.validate();  //设为 true 后，首先检查 flight_parameters_confirmed。随后执行 vertical.validate()。参数不满足要求时，节点直接拒绝启动。
      for(double value:{ramp_seconds,settle_seconds,landing_seconds,height_tolerance,speed_tolerance,
          ascent_timeout,descent_timeout,saturation_seconds,max_flight_seconds,max_tilt_deg,
          armed_dwell_seconds,liftoff_dwell_seconds,liftoff_height_m,liftoff_speed_mps,
          touchdown_dwell_seconds,height_lower_margin_m,z_jump_limit_m,vz_jump_limit_mps})
        if(!std::isfinite(value)||value<=0)throw std::invalid_argument("EXPLICIT_VALIDATED_FLIGHT_LIMITS_REQUIRED");
      if(armed_dwell_seconds<.1||liftoff_height_m>=vertical.max_height||
         height_lower_margin_m>=vertical.max_height||z_jump_limit_m>=vertical.max_height||
         touchdown_dwell_seconds<.1)throw std::invalid_argument("FLIGHT_OBSERVATION_ENVELOPE_INVALID");
      if(max_tilt_deg>=90)throw std::invalid_argument("MAX_TILT_MUST_BE_BELOW_90_DEGREES");
    }
  }
};
struct Feedback {
  Quaternion q{1,0,0,0};
  std::uint8_t quat_reset{},z_reset{},vz_reset{};
  double z{},vz{},stationary_seconds{},battery_voltage{NAN};
  std::uint64_t status_stamp{},land_stamp{};
  bool attitude_fresh{},local_fresh{},status_fresh{},land_fresh{},failsafe_fresh{};
  bool z_valid{},vz_valid{},armed{},landed{},at_rest{},offboard{},failsafe{},exclusive{},time_regressed{};
  bool calibration{},hil{},preflight{},battery_ready{},manual_ready{};
  bool stabilized_selectable{},stabilized_selected{};
  // manual_ready above is a legacy observation, never an authority predicate.
  bool rc_link_ready{},rc_switches_ready{},kill{},operator_takeover{},offboard_signal_lost{};
  // Observation diagnostics only. These do not replace the route predicates.
  bool rc_source_seen{},rc_source_valid{},rc_source_selected{},rc_sticks_moving{};
  bool rc_switches_seen{},rc_switches_fresh{};
  std::uint8_t rc_data_source{},rc_mode_slot{},rc_arm_switch{},rc_kill_switch{};
  std::uint64_t operator_switch_generation{};
  std::uint64_t local_stamp{};
  bool xy_valid{},vxy_valid{},land_selectable{},auto_land_selected{};
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
enum class Phase {Idle,SelfCheck,Prestream,RequestOffboard,OffboardReady,RequestArm,ArmedGround,
  ThrustRamp,Ascend,HeightHold,Descent,RecoveryDescent,Px4LandPending,PoweredGround,Landed,RequestDisarm,SafeDisarmed,ReleaseOffboard,Safety};
inline const char* name(Phase phase){
  switch(phase){
    case Phase::Idle:return "IDLE";case Phase::SelfCheck:return "SELF_CHECK";
    case Phase::Prestream:return "PRESTREAM";case Phase::RequestOffboard:return "REQUEST_OFFBOARD";
    case Phase::OffboardReady:return "OFFBOARD_READY";case Phase::RequestArm:return "REQUEST_ARM";
    case Phase::ArmedGround:return "ARMED_GROUND";case Phase::ThrustRamp:return "THRUST_RAMP";
    case Phase::Ascend:return "ASCEND";case Phase::HeightHold:return "HEIGHT_HOLD";
    case Phase::Descent:return "DESCENT";case Phase::PoweredGround:return "POWERED_GROUND_UNCONFIRMED";case Phase::Landed:return "LANDED";
    case Phase::RecoveryDescent:return "RECOVERY_DESCENT";
    case Phase::Px4LandPending:return "PX4_LAND_PENDING";
    case Phase::RequestDisarm:return "REQUEST_DISARM";case Phase::SafeDisarmed:return "SAFE_DISARMED";
    case Phase::ReleaseOffboard:return "RELEASE_OFFBOARD";
    case Phase::Safety:return "SAFETY";
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
  bool publish{};Quaternion q{1,0,0,0};std::array<double,3> thrust{};
  Command command{Command::None};double height{},speed{},target_height{},velocity_target{};
  bool saturated{},slew_limited{};
};
class Core {
 public:
  explicit Core(Config config={}):c_(config){c_.validate();
    if(c_.flight_actions_enabled){
      auto vertical=c_.vertical;
      vertical.ground_lower_margin=c_.height_lower_margin_m;
      vertical_=std::make_unique<VerticalController>(vertical);
    }
  }
  Phase phase() const{return phase_;}
  const std::string& reason() const{return reason_;}
  const std::string& event() const{return event_;}
  bool active() const{return phase_>=Phase::SelfCheck&&phase_<=Phase::ReleaseOffboard;}
  bool flight_started() const{return flight_started_;}
  bool disarm_requested() const{return phase_==Phase::RequestDisarm||phase_==Phase::SafeDisarmed;}
  bool reference_ready() const{return z_set_;}
  double reference_z() const{return z0_;}
  double yaw_reference() const{return reference_.yaw();}
  const authority::State& authority_state() const{return authority_;}
  const std::string& takeover_reason() const{return takeover_reason_;}
  bool restart_required() const{return restart_required_;}
  bool offboard_observed() const{return offboard_observed_;}
  bool ground_confirmed() const{return ground_confirmed_;}
  bool ground_release_pending() const{return ground_release_pending_;}
  bool arm_confirmed() const{return arm_confirmed_;}
  bool arm_cancel_pending() const{return arm_cancel_pending_;}
  bool arm_command_committed() const{return arm_command_committed_;}
  bool disarm_command_committed() const{return disarm_command_committed_;}
  bool disarm_outcome_uncertain() const{return disarm_outcome_uncertain_;}
  // Only Runtime may call this after its FINAL command sink commits. A plan
  // or Core::Output::command is not evidence of an emitted VehicleCommand.
  void note_command_publication(Command command,bool committed,std::uint64_t land_stamp=0,
      std::uint64_t status_stamp=0){
    if(!committed)return;
    if(command==Command::Arm)arm_command_committed_=true;
    if(command==Command::Disarm){disarm_command_committed_=true;disarm_in_flight_=true;
      disarm_land_commit_stamp_=land_stamp;}
    if(command==Command::Px4Land){px4_land_committed_=true;handoff_status_baseline_=status_stamp;}
  }
  std::uint64_t touchdown_baseline() const{return touchdown_baseline_;}
  FaultClass fault_class() const{return fault_class_;}
  double recovery_deadline() const{return recovery_deadline_;}
  bool recovery_budget_exhausted() const{return recovery_budget_exhausted_;}
  bool px4_land_command_committed() const{return px4_land_committed_;}
  bool recovery_progress_observed() const{return recovery_progress_observed_;}
  bool px4_land_mode_observed() const{return px4_land_mode_observed_;}
  // Explicit ownership disclosure, NOT an assertion that PX4 has safely landed.
  // Unrecoverable stale-telemetry, Kill and program failures cannot guarantee
  // an offboard maneuver; those routes require separately validated PX4 failsafe.
  const char* recovery_responsibility(const Feedback& s) const{
    if(s.kill)return "EMERGENCY_KILL_NO_ROUTINE_DISARM";
    if(external_ground_disarmed_confirmed_)return "GROUND_DISARMED_CONFIRMED";
    if(phase_==Phase::Safety)
      return s.status_fresh&&!s.offboard?"PX4_FAILSAFE_OBSERVED_BUT_CONTROL_UNVERIFIED":
        "PX4_FAILSAFE_PENDING_UNKNOWN_AIRBORNE";
    if(phase_==Phase::RecoveryDescent||phase_==Phase::Px4LandPending){
      if(recovery_budget_exhausted_)
        return "COMPANION_GUARDED_DESCENT_ACTIVE_RECOVERY_OVERDUE_HANDOFF_UNVERIFIED";
      return phase_==Phase::RecoveryDescent?"CONTROLLED_DESCENT_ACTIVE_BOUNDED_RECOVERY":
        "PX4_HANDOFF_REQUEST_PENDING_COMPANION_DESCENT_ACTIVE";
    }
    if(phase_==Phase::PoweredGround||phase_==Phase::Landed)
      return "COMPANION_SLEW_LIMITED_POWER_REMOVAL_AND_NEW_GROUND_PROOF";
    if(phase_==Phase::Descent)return "COMPANION_CONTROLLED_DESCENT_ACTIVE";
    if(arm_cancel_pending_&&phase_==Phase::RequestArm)
      return "ARM_400_OUTCOME_UNCERTAIN_GUARDED_HEARTBEAT";
    if(disarm_in_flight_&&s.armed&&s.offboard)
      return "DISARM_400_OUTCOME_UNCERTAIN_GUARDED_RECOVERY";
    if(operator_release_requested_&&s.offboard&&active())
      return "OPERATOR_HANDOVER_REQUESTED_NOT_PX4_CONFIRMED";
    if(phase_==Phase::SafeDisarmed||phase_==Phase::ReleaseOffboard)
      return "DISARMED_GROUND_PX4_MODE_RELEASE_PENDING";
    return flight_started_?"COMPANION_OFFBOARD_ATTITUDE_CONTROL_ACTIVE":
      "DISARMED_OR_PREPARATION_NO_FLIGHT";
  }
  bool operator_intervened(const Feedback& s) const{
    return s.operator_takeover||(active()&&s.operator_switch_generation!=switch_generation_);
  }
  const char* takeover_cause(const Feedback& s) const{
    if(s.kill)return "KILL_OR_TERMINATION";
    if(operator_intervened(s)&&!operator_release_requested_)return "EXPLICIT_OPERATOR_INTERVENTION";
    if(s.failsafe)return "PX4_FAILSAFE";
    if(offboard_observed_&&s.offboard_signal_lost)return "OFFBOARD_HEARTBEAT_LOST";
    if(offboard_observed_&&s.status_fresh&&!s.offboard&&phase_!=Phase::ReleaseOffboard&&
       phase_!=Phase::SafeDisarmed)return "PX4_LEFT_OFFBOARD";
    if(flight_started_&&authority_.mode==authority::Mode::RcTakeover){
      if(!s.rc_link_ready)return "RC_LINK_LOST";
      if(!s.rc_switches_ready)return "RC_TAKEOVER_TELEMETRY_LOST";
    }
    return nullptr;
  }
  bool operator_lease_live(double now) const{
    return interactive_&&std::isfinite(now)&&ping_>=0&&now>=ping_&&now-ping_<=c_.console_timeout;
  }
  bool preparation_lease_valid(double now) const{return !interactive_||operator_lease_live(now);}
  void live_console(double now){interactive_=true;ping_=now;}
  void ping(double now){if(interactive_)ping_=now;}
  std::string select_authority(authority::Mode mode,const Feedback& s,double now,
      bool outputs_active,bool pending_command,bool publication_fault){
    if(authority_.locked)return "AUTHORITY_MODE_LOCKED";
    if(phase_!=Phase::Idle||flight_started_)return "AUTHORITY_SELECTION_REQUIRES_IDLE";
    if(mode!=authority::Mode::RcTakeover&&mode!=authority::Mode::AutonomousNoRc)return "INVALID_AUTHORITY_MODE";
    auto why=safe_authority_ground(s,now,outputs_active,pending_command,publication_fault);
    if(!why.empty())return why;
    authority_.mode=mode;authority_.attempt_mode=authority::Mode::Unset;
    return {};
  }
  std::string reset_authority(const Feedback& s,double now,bool outputs_active,
      bool pending_command,bool publication_fault){
    // Generic/publication faults cannot be cleared through the operator API.
    if(restart_required_||(phase_==Phase::Safety&&!authority_.takeover_latched))return "AUTHORITY_SAFETY_RESTART_REQUIRED";
    if(active())return "AUTHORITY_MODE_LOCKED";
    auto why=safe_authority_ground(s,now,outputs_active,pending_command,publication_fault);
    if(!why.empty())return why;
    authority_={};flight_started_=false;offboard_observed_=false;z_set_=false;
    operator_release_requested_=false;
    reference_.clear();switch_generation_=s.operator_switch_generation;takeover_reason_.clear();
    enter(Phase::Idle,now);reason_="AUTHORITY_RESET_TO_UNSET_SAFE_GROUND";return {};
  }
  // Caller must prove PX4 actually left Offboard before declaring transfer complete.
  // An operator *request* is instead a controlled landing request on armed Offboard.
  std::string request_operator_takeover(const Feedback& s,double now){
    if(!s.status_fresh)return "TAKEOVER_NEEDS_FRESH_PX4_STATUS";
    if(!s.offboard){release_to_operator("PX4_MODE_HANDOVER_OBSERVED");return {}; }
    operator_release_requested_=true;
    if(s.armed||phase_==Phase::RequestArm){const auto why=descend(s,now);if(!why.empty())return why;
      reason_="TAKEOVER_REQUEST_PENDING_CONTROLLED_GROUND_RECOVERY";}
    else if(z_set_){enter(Phase::ReleaseOffboard,now);reason_="TAKEOVER_REQUEST_PENDING_GROUND_MODE_ACK";}
    else return "TAKEOVER_NO_GROUND_REFERENCE";
    return {};
  }
  bool operator_release_requested() const{return operator_release_requested_;}
  bool handover_observed() const{return authority_.takeover_latched&&!offboard_observed_;}
  void release_to_operator(const std::string& why){
    authority_.takeover_latched=true;authority_.locked=true;takeover_reason_=why;
    fault("OPERATOR_TAKEOVER_LATCHED",false);
  }
  std::string prepare(const Feedback& s,double now){
    if(phase_==Phase::Safety)return "SAFETY_RESTART_ON_SAFE_GROUND_REQUIRED";
    if(active())return "TASK_ACTIVE";
    if(authority_.locked)return "AUTHORITY_SAFE_GROUND_RESET_REQUIRED";
    auto why=health(s);if(!why.empty())return why;
    if(s.armed||!s.landed||!s.at_rest||s.offboard)return "PREPARE_REQUIRES_DISARMED_REST_OUTSIDE_OFFBOARD";
    if(!std::isfinite(now))return "INVALID_MONOTONIC_TIME";
    authority_.locked=true;authority_.attempt_mode=authority_.mode;
    offboard_observed_=false;switch_generation_=s.operator_switch_generation;
    reference_.clear();reference_.observe(s.q,s.quat_reset,false);
    z_set_=false;flight_started_=false;ground_abort_=false;ground_confirmed_=false;ground_release_pending_=false;
    liftoff_evidenced_=false;arm_confirmed_=false;
    arm_cancel_pending_=arm_command_committed_=false;
    disarm_in_flight_=disarm_command_committed_=disarm_outcome_uncertain_=false;
    disarm_land_commit_stamp_=0;
    recovery_deadline_=-1;px4_land_committed_=px4_land_attempted_=false;
    recovery_budget_exhausted_=false;
    px4_land_mode_observed_=recovery_progress_observed_=external_ground_disarmed_confirmed_=false;
    recovery_last_local_stamp_=0;
    fault_class_=FaultClass::None;last_sample_set_=false;operator_release_requested_=false;
    command_sent_=false;last_=now;event_="LOCAL_YAW_REFERENCE_CAPTURED";
    enter(Phase::SelfCheck,now);return {};
  }
  std::string takeoff(const Feedback& s,double height,double now){
    if(!c_.flight_actions_enabled)return "FLIGHT_ACTIONS_DISABLED";
    if(!c_.arm_enabled)return "ARM_DISABLED";
    if(!c_.takeoff_enabled)return "TAKEOFF_DISABLED";
    if(!c_.parameters_confirmed)return "FLIGHT_PARAMETERS_NOT_CONFIRMED";
    if(restart_required_)return "AUTHORITY_SAFETY_RESTART_REQUIRED";
    if(takeover_cause(s))return "OPERATOR_TAKEOVER_LATCHED";
    const auto policy=authority::evaluate(authority_,authority_inputs(c_,s));
    if(policy!=authority::Reason::Allowed)return authority::name(policy);
    if(phase_!=Phase::OffboardReady)return "TAKEOFF_REQUIRES_OFFBOARD_READY";
    if(!interactive_||ping_<0||now<ping_||now-ping_>.5)return "LIVE_INTERACTIVE_CONSOLE_REQUIRED";
    auto why=health(s);if(!why.empty())return why;
    if(!s.offboard||s.armed||!s.landed||!s.at_rest||!z_set_||s.stationary_seconds<c_.ground_seconds)
      return "TAKEOFF_REQUIRES_CONFIRMED_DISARMED_OFFBOARD_GROUND_REFERENCE";
    if(!s.battery_ready||!s.preflight)return "BATTERY_PREFLIGHT_REQUIRED";
    if(!std::isfinite(height)||height<=0||height>=c_.vertical.max_height)
      return "HEIGHT_OUTSIDE_REVIEWED_ENVELOPE";
    target_=height;command_height_=0;flight_started_=true;flight_since_=now;
    last_sample_set_=false;ground_confirmed_=false;liftoff_evidenced_=false;
    arm_confirmed_=false;arm_cancel_pending_=arm_command_committed_=false;
    disarm_in_flight_=disarm_command_committed_=disarm_outcome_uncertain_=false;
    disarm_land_commit_stamp_=0;
    vertical_->reset(0);
    enter(Phase::RequestArm,now);return {};
  }
  std::string descend(const Feedback& s,double now){
    if(phase_==Phase::Safety)return "SAFETY_RELEASED_TO_PX4_OPERATOR";
    if(phase_==Phase::ReleaseOffboard||phase_==Phase::SafeDisarmed)return {};
    if(!flight_started_){
      if(s.armed)return "UNEXPECTED_ARMED_CANNOT_RECOVER_WITH_STAGE_ONE";
      if(phase_==Phase::RequestOffboard){
        // Command 176 may still switch to Offboard after this request returns.
        // Continue exact-zero prestream until ACK + new PX4 status, then release.
        ground_release_pending_=true;reason_="WAIT_OFFBOARD_RESULT_BEFORE_GROUND_RELEASE";
        return {};
      }
      if(s.offboard&&z_set_){enter(Phase::ReleaseOffboard,now);reason_="GROUND_RELEASE_HANDSHAKE_REQUIRED";}
      else {enter(Phase::Idle,now);reason_="DISARMED_PREPARATION_RELEASED";}return {};
    }
    if(phase_==Phase::RequestArm){
      // Command 400 may already have crossed the final sink; a LAND request
      // cannot retract it. Keep the exact-zero guarded Offboard setpoint while
      // waiting for PX4's NEW status. No further Arm is planned after cancel.
      arm_cancel_pending_=true;
      reason_=arm_command_committed_?"ARM_COMMITTED_CANCEL_PENDING_PX4_STATUS":
        "ARM_PLANNED_CANCEL_PENDING_FINAL_SINK_RESULT";
      return {};
    }
    if(!s.armed)return "ACTIVE_FLIGHT_NOT_ARMED";
    if(phase_!=Phase::Descent&&phase_!=Phase::RecoveryDescent&&phase_!=Phase::Px4LandPending&&
       phase_!=Phase::Landed&&phase_!=Phase::RequestDisarm){
      command_height_=std::max(0.,z0_-s.z);land_baseline_=s.land_stamp;
      const bool plausible_ground=health(s).empty()&&s.landed&&
        std::abs(z0_-s.z)<=c_.height_tolerance&&std::abs(s.vz)<=c_.speed_tolerance;
      ground_abort_=plausible_ground;
      fault_class_=plausible_ground?FaultClass::GroundAbort:FaultClass::ControlledDescent;
      // at_rest may be false after Arm or while motors are still spinning.
      // No routine cancellation may claim airborne/stop streaming solely for this.
      // PoweredGround retains a bounded ramp-to-zero and demands NEW detector samples.
      enter(plausible_ground?Phase::PoweredGround:Phase::Descent,now);
    }return {};
  }
  void disconnect(const Feedback& s,double now){
    interactive_=false;ping_=-1;
    if(active()){auto why=descend(s,now);if(!why.empty())fault(why);}
  }
  void stop_disarmed(const Feedback& s,double now){
    if(!flight_started_&&phase_!=Phase::Safety){
      if(phase_==Phase::RequestOffboard){ground_release_pending_=true;return;}
      if(s.offboard&&z_set_)enter(Phase::ReleaseOffboard,now);
      else enter(Phase::Idle,now);
    }
  }
  void block(const std::string& reason){fault(reason);}
  Output step(const Feedback& s,double now,Transaction mode,Transaction arm,Transaction px4_land={}){
    Output out;
    if(phase_==Phase::Safety){observe_external_terminal(s,now);return out;}
    last_status_stamp_=s.status_stamp;last_land_stamp_=s.land_stamp;
    if(s.armed&&!flight_started_){fault("UNEXPECTED_ARMED_STAGE_ONE_REFUSED");return out;}
    if(!active())return out;
    const double elapsed=now-last_;last_=now;
    if(!std::isfinite(now)||!std::isfinite(elapsed)||elapsed<0){fault("TIME_REGRESSION");return out;}
    if(elapsed>c_.stall_seconds){fault("CONTROL_LOOP_STALL");return out;}
    // A successful local sink + NEW mode is ownership evidence, not proof
    // that PX4's selected flight task runs. ACK alone never releases custody.
    if(phase_==Phase::Px4LandPending&&px4_land_committed_&&s.status_fresh&&
       !s.offboard&&s.status_stamp>handoff_status_baseline_){
      px4_land_mode_observed_=s.auto_land_selected;
      fault_class_=FaultClass::Px4Handoff;
      release_to_operator(s.auto_land_selected?
        (px4_land.accepted?"PX4_LAND_ACK_AND_NEW_MODE_OBSERVED_CONTROL_UNVERIFIED":
         "PX4_LAND_NEW_MODE_OBSERVED_ACK_UNATTRIBUTED_CONTROL_UNVERIFIED"):
        "PX4_OTHER_MODE_OBSERVED_CONTROL_UNVERIFIED");return out;
    }
    if(s.status_fresh&&s.offboard)offboard_observed_=true;
    if(const auto cause=takeover_cause(s)){
      if(s.offboard&&s.armed&&!s.kill&&!s.failsafe&&s.status_fresh&&
         std::string(cause)=="EXPLICIT_OPERATOR_INTERVENTION"){
        const auto requested=request_operator_takeover(s,now);
        if(!requested.empty()){fault(requested);return out;}
        // Observing an RC switch is not itself a proven PX4 mode switch.
        // Keep controlled descent rather than silently abandoning Offboard.
      }else{
        fault_class_=(s.armed&&!verified_ground(s))?FaultClass::Px4Handoff:FaultClass::GroundAbort;
        release_to_operator(cause);return out;
      }
    }
    const auto why=health(s);if(!why.empty()){
      fault_class_=s.armed?FaultClass::UnknownAirborne:FaultClass::GroundAbort;
      fault(why);return out;
    }
    if(!flight_started_&&interactive_&&(ping_<0||now<ping_||now-ping_>c_.console_timeout)){
      fault("PREPARATION_CONSOLE_LEASE_EXPIRED");return out;
    }
    if(flight_started_&&(!c_.flight_actions_enabled||!c_.arm_enabled||!c_.takeoff_enabled||!c_.parameters_confirmed)){
      fault("FLIGHT_GATE_CLOSED");return out;
    }
    if(flight_started_){
      const auto policy=authority::evaluate(authority_,authority_inputs(c_,s));
      if(policy!=authority::Reason::Allowed){fault(authority::name(policy));return out;}
      if(!s.battery_ready){fault("BATTERY_NOT_READY");return out;}
    }
    const auto ref=reference_.observe(s.q,s.quat_reset,flight_started_||s.armed);
    if(ref==ReferenceResult::ActiveReset){fault("ACTIVE_QUATERNION_RESET");return out;}
    if(ref==ReferenceResult::Rebased)event_="DISARMED_QUATERNION_REBASED";
    if(z_set_&&(s.z_reset!=z_reset_||s.vz_reset!=vz_reset_)){
      if(flight_started_||s.armed){fault("ACTIVE_VERTICAL_ESTIMATOR_RESET");return out;}
      if(!s.landed||!s.at_rest||s.stationary_seconds<c_.ground_seconds){fault("GROUND_RESET_REFERENCE_NOT_STABLE");return out;}
      z0_=s.z;z_reset_=s.z_reset;vz_reset_=s.vz_reset;event_="DISARMED_Z_REFERENCE_REBASED";
    }
    if(phase_>=Phase::OffboardReady&&phase_<=Phase::RequestDisarm&&!s.offboard){
      fault_class_=s.armed?FaultClass::Px4Handoff:FaultClass::GroundAbort;
      fault("OFFBOARD_LOST_OR_OPERATOR_MODE_CHANGED");return out;
    }
    if(flight_started_&&phase_>=Phase::ArmedGround&&phase_!=Phase::RequestDisarm&&
       phase_!=Phase::ReleaseOffboard&&!s.armed){
      if(disarm_in_flight_&&s.status_stamp>status_baseline_&&
         s.land_stamp>disarm_land_commit_stamp_&&verified_ground(s)){
        // Status is independent evidence; do not label a command ACK success
        // until its own transaction ACK is also observed.
        if(disarm_terminal(arm)){enter(Phase::SafeDisarmed,now);flight_started_=false;
          disarm_in_flight_=false;reason_=arm.accepted?"DISARM_ACK_AND_FRESH_SAFE_GROUND_STATUS":
            "DISARMED_FRESH_GROUND_STATE_ACK_UNATTRIBUTED";return out;}
        reason_="DISARMED_STATUS_PRESENT_AWAITING_400_ACK";
      }else{
        fault_class_=FaultClass::UnknownAirborne;
        fault(disarm_in_flight_?"DISARMED_WHILE_GROUND_UNCONFIRMED_EXTERNAL_FAILSAFE_REQUIRED":
          "UNEXPECTED_DISARM_ACTIVE_FLIGHT");return out;
      }
    }
    if(flight_started_&&phase_!=Phase::RequestArm&&phase_<=Phase::HeightHold&&
       ((!interactive_||ping_<0||now-ping_>c_.console_timeout)||now-flight_since_>c_.max_flight_seconds))
      descend(s,now);
    if(phase_==Phase::SelfCheck){
      if(s.armed||!s.landed||!s.at_rest){fault("GROUND_STATE_CHANGED_DURING_SELF_CHECK");return out;}
      if(s.stationary_seconds<c_.ground_seconds){reason_="WAITING_FOR_VALID_GROUND_REFERENCE";return out;}
      z0_=s.z;z_reset_=s.z_reset;vz_reset_=s.vz_reset;z_set_=true;enter(Phase::Prestream,now);
    }
    out.publish=true;out.q=reference_.level();out.height=z0_-s.z;out.speed=-s.vz;out.target_height=command_height_;
    if(!std::isfinite(out.height)||!std::isfinite(out.speed)){fault("NONFINITE_RELATIVE_HEIGHT");return {};}
    if(flight_started_&&last_sample_set_&&
       (std::abs(s.z-last_sample_z_)>c_.z_jump_limit_m||
        std::abs(s.vz-last_sample_vz_)>c_.vz_jump_limit_mps)){
      fault_class_=FaultClass::UnknownAirborne;fault("UNEXPLAINED_Z_VZ_ESTIMATOR_JUMP");return {};
    }
    if(flight_started_){last_sample_z_=s.z;last_sample_vz_=s.vz;last_sample_set_=true;}
    if(flight_started_){
      if(out.height>c_.vertical.max_height||out.height< -c_.height_lower_margin_m ||
         std::abs(s.vz)>c_.vertical.max_climb+c_.vertical.max_descent+2*c_.vz_jump_limit_mps){
        fault_class_=FaultClass::UnknownAirborne;fault("HEIGHT_OR_SPEED_ENVELOPE_EXCEEDED");return {};
      }
      const auto q=normalized(s.q);
      const double cosine=std::clamp(1-2*((*q)[1]*(*q)[1]+(*q)[2]*(*q)[2]),-1.,1.);
      if(std::acos(cosine)*180/3.14159265358979323846>c_.max_tilt_deg){fault("EXCESS_TILT");return {};}
    }
    if(recovery_deadline_>=0&&s.armed&&!verified_ground(s)&&now>=recovery_deadline_){
      // Timeout is not evidence of landing or an actual PX4 takeover. All
      // existing health/authority/estimator/envelope gates have passed above.
      // Retain feedback-controlled, bounded/slew-limited descent through the
      // same final sinks. This is custody retention, NOT a safety guarantee
      // for indefinitely ineffective thrust or any fatal/hard-process failure.
      if(!recovery_budget_exhausted_)event_="RECOVERY_BUDGET_EXHAUSTED_NO_CONFIRMED_HANDOFF";
      recovery_budget_exhausted_=true;
    }
    if(phase_==Phase::Prestream){
      if(now-entered_>=c_.prestream_seconds){enter(Phase::RequestOffboard,now);status_baseline_=s.status_stamp;command_sent_=true;out.command=Command::Offboard;}
    }else if(phase_==Phase::RequestOffboard){
      if(mode.rejected||mode.timed_out){fault("OFFBOARD_COMMAND_REJECTED_OR_TIMEOUT");return {};}
      if(mode.accepted&&s.offboard&&s.status_stamp>status_baseline_){
        if(ground_release_pending_){
          ground_release_pending_=false;enter(Phase::ReleaseOffboard,now);
          reason_="GROUND_RELEASE_AFTER_OFFBOARD_ACK_AND_STATUS";
        }else {enter(Phase::OffboardReady,now);
          reason_="ACK_ACCEPTED_AND_NEW_OFFBOARD_DISARMED_STATUS";}
      }
      else if(now-entered_>=c_.command_timeout){fault(mode.accepted?"OFFBOARD_ACK_ACCEPTED_STATUS_TIMEOUT":"OFFBOARD_ACK_TIMEOUT");return {};}
    }else if(phase_==Phase::OffboardReady){
      // Preparation is always zero thrust, even if stage-two config is enabled.
    }else if(phase_==Phase::RequestArm){
      if(!interactive_||ping_<0||now-ping_>c_.console_timeout)
        arm_cancel_pending_=true;
      // Only a genuinely new PX4 armed observation can determine the
      // outcome of a submitted ARM. ACK alone cannot prove a physical state.
      if(s.armed&&s.status_stamp>status_baseline_){
        arm_confirmed_=true;
        if(arm_cancel_pending_||arm.rejected||arm.timed_out){
          const bool plausible_ground=s.landed&&std::abs(out.height)<=c_.height_tolerance&&
            std::abs(out.speed)<=c_.speed_tolerance;
          fault_class_=plausible_ground?FaultClass::GroundAbort:FaultClass::ControlledDescent;
          land_baseline_=s.land_stamp;
          enter(plausible_ground?Phase::PoweredGround:Phase::Descent,now);
          command_height_=std::max(0.,out.height);
          // Continuity on the transition tick: controlled zero/ramp, NEVER
          // an unguarded stop of an armed Offboard stream.
          if(phase_==Phase::PoweredGround){
            if(!apply(vertical_->ramp(0,elapsed),out,now))return {};
          }else if(!apply(vertical_->step({z0_,s.z,s.vz,true,true,true,s.z_reset,s.vz_reset},
              command_height_,elapsed),out,now))return {};
          reason_="LATE_ARM_STATUS_SUPERVISED_GROUND_RECOVERY";
        }else if(arm.accepted){
          enter(Phase::ArmedGround,now);
        }else if(now-entered_>=c_.command_timeout){
          arm_cancel_pending_=true;reason_="ARMED_WITHOUT_MATCHED_ACK_SUPERVISED_RECOVERY_REQUIRED";
        }
      }else if(arm_cancel_pending_&&(!arm_command_committed_&&!command_sent_)){
        // No command was even planned at this point. Release zero-thrust
        // preparation through the existing final-mode-release handshake.
        if(!s.armed&&s.landed&&s.at_rest){enter(Phase::ReleaseOffboard,now);
          reason_="UNSENT_ARM_CANCEL_GROUND_MODE_RELEASE";}
      }else if(!command_sent_&&!arm_cancel_pending_){
        command_sent_=true;status_baseline_=s.status_stamp;out.command=Command::Arm;
      }else if(arm.rejected&&arm_cancel_pending_&&!s.armed&&s.status_stamp>status_baseline_&&
               s.landed&&s.at_rest){
        enter(Phase::ReleaseOffboard,now);
        reason_="ARM_REJECTED_AND_NEW_DISARMED_STATUS_GROUND_RELEASE";
      }else if(arm.rejected||arm.timed_out||now-entered_>=c_.command_timeout){
        arm_cancel_pending_=true;
        // An ACK timeout does NOT prove the Arm never reached PX4. Hold a
        // safe, FINAL-guarded zero setpoint; no automatic second 400.
        reason_=arm.rejected?"ARM_REJECTED_AWAIT_NEW_DISARMED_STATUS":
          "ARM_OUTCOME_UNCERTAIN_WAIT_PX4_STATUS_NO_NEW_TASK";
      }
    }else if(phase_==Phase::ArmedGround){
      if(!s.armed||!s.offboard||!s.landed||std::abs(out.height)>c_.height_tolerance||
         std::abs(out.speed)>c_.ground_vz_tolerance){
        fault_class_=FaultClass::UnknownAirborne;fault("ARMED_GROUND_NOT_STABLE");return {};
      }
      // ArmBench demonstrated that PX4 may report at_rest=false immediately after Arm.
      // Do not use at_rest as an unconditional ArmedGround predicate.
      if(dwell_land(s.land_stamp>0,now,c_.armed_dwell_seconds,s.land_stamp)){
        land_baseline_=s.land_stamp;enter(Phase::ThrustRamp,now);
      }
    }else if(phase_==Phase::ThrustRamp){
      if(now-entered_>=c_.ascent_timeout){
        const bool plausible_ground=s.landed&&std::abs(out.height)<=c_.height_tolerance&&
          std::abs(out.speed)<=c_.speed_tolerance;
        fault_class_=plausible_ground?FaultClass::GroundAbort:FaultClass::ControlledDescent;
        land_baseline_=s.land_stamp;
        enter(plausible_ground?Phase::PoweredGround:Phase::Descent,now);
        reason_="LIFTOFF_TIMEOUT_ACTIVE_POWER_RECOVERY";
        // No more acceleration. Even the transition tick MUST use the
        // configured slew limiter; a default-zero Output is NOT a ramp.
        if(phase_==Phase::PoweredGround){
          const auto reduce=vertical_->ramp(0,elapsed);
          if(!apply(reduce,out,now))return {};
        }else{
          command_height_=std::max(0.,out.height);
          const auto hold=vertical_->step({z0_,s.z,s.vz,true,true,true,s.z_reset,s.vz_reset},command_height_,elapsed);
          if(!apply(hold,out,now))return {};
        }
      }
      if(phase_==Phase::ThrustRamp){
        auto control=vertical_->ramp(c_.vertical.hover*std::min(1.,(now-entered_)/c_.ramp_seconds),elapsed);
        if(!apply(control,out,now))return {};
        const bool lifting=!s.landed&&s.land_stamp>land_baseline_&&
          out.height>=c_.liftoff_height_m&&out.speed>=c_.liftoff_speed_mps;
        if(dwell_land(lifting,now,c_.liftoff_dwell_seconds,s.land_stamp)){
          liftoff_evidenced_=true;enter(Phase::Ascend,now);
        }
      }
      // A clock or hover-thrust ramp alone is never evidence of liftoff.
    }else if(phase_==Phase::Ascend||phase_==Phase::HeightHold||phase_==Phase::Descent||
             phase_==Phase::RecoveryDescent||phase_==Phase::Px4LandPending){
      bool descending=phase_==Phase::Descent||phase_==Phase::RecoveryDescent||phase_==Phase::Px4LandPending;
      if(phase_==Phase::Ascend&&now-entered_>=c_.ascent_timeout){
        // A navigation timeout is NOT permission to abandon a live armed
        // Offboard vehicle. Health, authority, limits and sink gates above
        // have already been checked; enter the supervised landing path now.
        // This does NOT claim landing was achieved or PX4 took ownership.
        const auto recovery=descend(s,now);
        if(!recovery.empty()){
          fault_class_=FaultClass::UnknownAirborne;fault(recovery);return {};
        }
        reason_="ASCENT_TIMEOUT_SUPERVISED_RECOVERY";
        if(phase_==Phase::PoweredGround){
          if(!apply(vertical_->ramp(0,elapsed),out,now))return {};
          out.target_height=command_height_;
          return out;
        }
        descending=true;
      }
      if(phase_==Phase::Descent&&now-entered_>=c_.descent_timeout){
        if(recovery_deadline_<0)recovery_deadline_=now+c_.recovery_seconds;
        recovery_entry_height_=out.height;recovery_last_local_stamp_=s.local_stamp;
        command_height_=std::min(command_height_,std::max(0.,out.height));
        enter(Phase::RecoveryDescent,now);fault_class_=FaultClass::ControlledDescent;
        reason_="DESCENT_TIMEOUT_CONTROLLED_RECOVERY_ACTIVE_HANDOFF_UNVERIFIED";
      }
      if(phase_==Phase::RecoveryDescent||phase_==Phase::Px4LandPending){
        if(s.local_stamp>recovery_last_local_stamp_){
          recovery_last_local_stamp_=s.local_stamp;
          if(out.height<recovery_entry_height_-c_.height_tolerance)
            recovery_progress_observed_=true;
        }
        // Cancel any remaining upward target; never reset the physical slew
        // state or lower the detector/height/velocity proof thresholds.
        command_height_=std::min(command_height_,std::max(0.,out.height));
        if(phase_==Phase::RecoveryDescent&&!px4_land_attempted_&&land_fallback_ready(s)){
          px4_land_attempted_=true;enter(Phase::Px4LandPending,now);
          handoff_status_baseline_=s.status_stamp;out.command=Command::Px4Land;command_sent_=true;
          reason_="PX4_LAND_REQUEST_PLANNED_CONTINUOUS_GUARDED_RECOVERY";
        }else if(phase_==Phase::Px4LandPending&&
                 (px4_land.rejected||px4_land.timed_out||now-entered_>=c_.px4_land_ack_seconds)){
          // A rejected/missing ACK does not remove healthy control or justify
          // another 21/176/400. Same absolute completion deadline survives.
          enter(Phase::RecoveryDescent,now);
          reason_="PX4_LAND_ACK_OR_MODE_UNRESOLVED_NO_RETRY_GUARDED_RECOVERY";
        }
      }
      command_height_=descending?std::max(0.,command_height_-c_.vertical.max_descent*elapsed):
                                 std::min(target_,command_height_+c_.vertical.max_climb*elapsed);
      auto control=vertical_->step({z0_,s.z,s.vz,true,true,true,s.z_reset,s.vz_reset},command_height_,elapsed);
      if(!apply(control,out,now))return {};
      out.target_height=command_height_;
      if(phase_==Phase::Ascend&&dwell(command_height_>=target_&& !s.landed&&std::abs(out.height-target_)<=c_.height_tolerance&&
          std::abs(out.speed)<=c_.speed_tolerance,now,c_.settle_seconds))enter(Phase::HeightHold,now);
      if(descending&&dwell_land(s.landed&&s.land_stamp>land_baseline_&&
          std::abs(out.height)<=c_.height_tolerance&&
          std::abs(out.speed)<=c_.speed_tolerance,now,c_.touchdown_dwell_seconds,s.land_stamp)){
        // At_rest can remain false under powered touchdown. Require it later,
        // after a controlled thrust-down sequence, never as liftoff proof.
        touchdown_baseline_=s.land_stamp;enter(Phase::Landed,now);
      }
    }else if(phase_==Phase::PoweredGround){
      // This phase owns any armed motor-power removal until a new ground
      // observation proves it safe. It is deliberately not a terminal Safety.
      if(!s.landed||std::abs(out.height)>c_.height_tolerance||
         std::abs(out.speed)>c_.speed_tolerance){
        ground_abort_=false;fault_class_=FaultClass::ControlledDescent;
        land_baseline_=s.land_stamp;enter(Phase::Descent,now);
        command_height_=std::max(0.,out.height);
        const auto hold=vertical_->step({z0_,s.z,s.vz,true,true,true,s.z_reset,s.vz_reset},command_height_,elapsed);
        if(!apply(hold,out,now))return {};
      }else{
        auto control=vertical_->ramp(0,elapsed);if(!apply(control,out,now))return {};
        if(dwell_land(s.at_rest&&control.thrust<=1e-9,now,c_.touchdown_dwell_seconds,s.land_stamp)){
          // LAND continues observation / new-sample dwell and final sink proof.
          const auto first=land_dwell_first_stamp_;enter(Phase::Landed,now);land_baseline_=first;
        }
      }
    }else if(phase_==Phase::Landed){
      if(!s.landed||std::abs(out.height)>c_.height_tolerance||
         std::abs(out.speed)>c_.speed_tolerance){
        ground_confirmed_=false;
        if(health(s).empty()&&s.armed){
          land_baseline_=s.land_stamp;enter(Phase::Descent,now);
          command_height_=std::max(0.,out.height);
          const auto hold=vertical_->step({z0_,s.z,s.vz,true,true,true,s.z_reset,s.vz_reset},command_height_,elapsed);
          if(!apply(hold,out,now))return {};
          // New control arrives under the same Final Sink without heartbeat gap.
        }else{fault_class_=FaultClass::UnknownAirborne;fault("LANDED_CONFIRMATION_LOST");return {};}
      }else{
      auto control=vertical_->ramp(0,elapsed);if(!apply(control,out,now))return {};
      const bool confirmed=dwell_land(s.at_rest&&s.land_stamp>land_baseline_&&
        std::abs(out.height)<=c_.height_tolerance&&std::abs(out.speed)<=c_.speed_tolerance,
        now,c_.landing_seconds,s.land_stamp);
      if(confirmed&&control.thrust<=1e-9){
        if(disarm_in_flight_){
          // Previous Disarm cannot be recalled or replayed blindly. Await
          // ACK and independent NEW disarmed PX4 status while still streaming.
          if(disarm_terminal(arm)&&!s.armed&&s.status_stamp>status_baseline_&&
             s.land_stamp>disarm_land_commit_stamp_){
            disarm_in_flight_=false;enter(Phase::SafeDisarmed,now);flight_started_=false;
          }else if(arm.rejected&&s.armed&&!disarm_command_committed_){
            disarm_in_flight_=false;
            land_baseline_=s.land_stamp;enter(Phase::PoweredGround,now);
            reason_="POST_BOUNCE_DISARM_REJECTED_REACQUIRE_GROUND_EVIDENCE";
          }else if(arm.ambiguous||arm.timed_out||(arm.rejected&&disarm_command_committed_)){
            disarm_outcome_uncertain_=true;
            reason_=arm.ambiguous?"AMBIGUOUS_400_ACK_POST_BOUNCE_NO_RETRY":
              "POST_BOUNCE_DISARM_OUTCOME_UNCERTAIN_NO_BLIND_RETRY";
          }else reason_="PREVIOUS_DISARM_SUBMITTED_AWAIT_ACK_AND_STATE";
        }else if(!disarm_outcome_uncertain_){
          const auto first=land_dwell_first_stamp_;
          ground_confirmed_=true;enter(Phase::RequestDisarm,now);touchdown_baseline_=first;status_baseline_=s.status_stamp;
          out.command=Command::Disarm;command_sent_=true;
          disarm_in_flight_=true; // Planned; Runtime separately records final sink commit.
        }else reason_="DISARM_TIMEOUT_NO_BLIND_RETRY_EXTERNAL_REVIEW_REQUIRED";
      }
      }
    }else if(phase_==Phase::RequestDisarm){
      if(disarm_terminal(arm)&&!s.armed&&s.status_stamp>status_baseline_&&
         s.land_stamp>disarm_land_commit_stamp_&&verified_ground(s)){
        disarm_in_flight_=false;enter(Phase::SafeDisarmed,now);flight_started_=false;
        reason_=arm.accepted?"DISARM_ACK_AND_NEW_DISARMED_LANDED_STATUS":
          "DISARMED_NEW_SAFE_GROUND_STATE_ACK_UNATTRIBUTED";return {};
      }
      if(!ground_confirmed_||!s.landed||!s.at_rest||s.land_stamp<=touchdown_baseline_||
         std::abs(out.height)>c_.height_tolerance||std::abs(out.speed)>c_.speed_tolerance){
        ground_confirmed_=false;
        if(!s.armed){fault_class_=FaultClass::UnknownAirborne;
          fault("DISARMED_BUT_GROUND_LOST_EXTERNAL_FAILSAFE_REQUIRED");return {};}
        // A submitted 400 is irreversible. Bounce does not cancel it and
        // must not tear down healthy Armed+Offboard custody. Descend with
        // the same bounded vertical controller and final publication sink.
        fault_class_=FaultClass::ControlledDescent;land_baseline_=s.land_stamp;
        enter(Phase::Descent,now);command_height_=std::max(0.,out.height);
        if(!apply(vertical_->step({z0_,s.z,s.vz,true,true,true,s.z_reset,s.vz_reset},
              command_height_,elapsed),out,now))return {};
        reason_="DISARM_SUBMITTED_GROUND_BOUNCE_SUPERVISED_DESCENT";
      }else if(arm.rejected&&!disarm_command_committed_){
        disarm_in_flight_=false;land_baseline_=s.land_stamp;
        enter(Phase::PoweredGround,now);
        if(!apply(vertical_->ramp(0,elapsed),out,now))return {};
        reason_="DISARM_REJECTED_NEW_GROUND_PROOF_REQUIRED";
      }else if(arm.ambiguous||arm.rejected||arm.timed_out||now-entered_>=c_.command_timeout){
        disarm_outcome_uncertain_=true;
        reason_=arm.ambiguous?"AMBIGUOUS_400_ACK_HOLD_GUARDED_OUTPUT_NO_RETRY":
          "DISARM_ACK_OR_STATUS_UNCERTAIN_HOLD_GUARDED_OUTPUT";
      }
    }else if(phase_==Phase::SafeDisarmed){
      if(!verified_ground(s)||s.armed){fault("SAFE_DISARMED_GROUND_LOST");return {};}
      enter(Phase::ReleaseOffboard,now);
    }
    if(phase_==Phase::ReleaseOffboard){
      // Keep exact-zero Offboard heartbeat while the ground-only mode release
      // is outstanding. After PX4 reports the new mode, do not keep sending
      // an Offboard heartbeat while awaiting this transaction's ACK.
      out.publish=s.offboard;
      if(s.armed||!s.landed||!s.at_rest||!s.status_fresh||!s.land_fresh){
        fault("RELEASE_REQUIRES_DISARMED_GROUND");return {};
      }
      if(!s.offboard){
        if(!command_sent_&&s.stabilized_selected){enter(Phase::Idle,now);return {};}
        if(command_sent_&&mode.accepted&&s.stabilized_selected&&s.status_stamp>status_baseline_){
          enter(Phase::Idle,now);reason_="RELEASE_ACK_AND_NEW_STABILIZED_STATUS";return {};
        }
      }
      if(!command_sent_){
        if(!s.stabilized_selectable){fault("STABILIZED_MODE_NOT_AVAILABLE_FOR_RELEASE");return {};}
        command_sent_=true;status_baseline_=s.status_stamp;out.command=Command::ReleaseOffboard;
      }else if(mode.rejected||mode.timed_out||now-entered_>=c_.command_timeout){
        fault("RELEASE_MODE_ACK_OR_STATUS_TIMEOUT");return {};
      }
    }
    for(double value:out.thrust)if(!std::isfinite(value)){fault("NONFINITE_OUTPUT");return {};}
    if(!normalized(out.q)){fault("INVALID_OUTPUT_QUATERNION");return {};}
    if(recovery_budget_exhausted_&&s.armed&&s.offboard&&!verified_ground(s))
      reason_="RECOVERY_BUDGET_EXHAUSTED_GUARDED_CUSTODY_ACTIVE_HANDOFF_UNVERIFIED";
    return out;
  }
 private:
  bool land_fallback_ready(const Feedback& s) const{
    return c_.px4_land_fallback_confirmed&&!c_.px4_land_evidence_id.empty()&&
      s.local_fresh&&s.xy_valid&&s.vxy_valid&&s.z_valid&&s.vz_valid&&s.land_selectable&&
      s.status_fresh&&s.armed&&s.offboard&&!s.failsafe;
  }
  void observe_external_terminal(const Feedback& s,double now){
    // Continue read-only observation after giving up control. Two NEW land
    // stamps and NEW status after failure are needed before declaring ground.
    const bool ground=std::isfinite(now)&&!s.time_regressed&&s.status_fresh&&
      s.status_stamp>fault_status_baseline_&&!s.armed&&s.land_fresh&&
      s.land_stamp>fault_land_baseline_&&verified_ground(s);
    const bool confirmed=dwell_land(ground,now,c_.ground_seconds,s.land_stamp);
    if(!ground)external_ground_disarmed_confirmed_=false;
    if(confirmed)external_ground_disarmed_confirmed_=true;
  }
  bool disarm_terminal(const Transaction& transaction) const{
    return transaction.accepted||
      (disarm_command_committed_&&transaction.state_evidenced);
  }
  std::string safe_authority_ground(const Feedback& s,double now,bool outputs_active,
      bool pending_command,bool publication_fault) const{
    if(publication_fault)return "AUTHORITY_PUBLICATION_FAULT_RESTART_REQUIRED";
    if(!operator_lease_live(now))return "LIVE_INTERACTIVE_CONSOLE_REQUIRED";
    if(outputs_active)return "AUTHORITY_OUTPUTS_MUST_BE_STOPPED";
    if(pending_command)return "AUTHORITY_PENDING_TRANSACTION";
    auto why=health(s);if(!why.empty())return why;
    if(s.kill||s.operator_takeover)return "OPERATOR_TAKEOVER_LATCHED";
    if(s.armed||!s.landed||!s.at_rest||s.stationary_seconds<c_.ground_seconds||
       std::abs(s.vz)>c_.ground_vz_tolerance)return "AUTHORITY_SAFE_GROUND_REQUIRED";
    return {};
  }
  std::string health(const Feedback& s) const{
    if(s.time_regressed)return "TELEMETRY_TIME_REGRESSION";
    if(!s.exclusive)return "PUBLISHER_OR_TELEMETRY_CONFLICT";
    if(!s.status_fresh||!s.land_fresh||!s.failsafe_fresh)return "DDS_STATUS_LAND_OR_FAILSAFE_STALE";
    if(s.failsafe)return "PX4_FAILSAFE";
    if(s.calibration||s.hil)return "CALIBRATION_OR_HIL_ACTIVE";
    if(!s.attitude_fresh||!normalized(s.q))return "ATTITUDE_STALE_OR_INVALID";
    if(!s.local_fresh||!s.z_valid||!s.vz_valid)return "Z_VZ_STALE_OR_INVALID";
    if(!std::isfinite(s.z)||!std::isfinite(s.vz)||!std::isfinite(s.stationary_seconds))return "NONFINITE_VERTICAL_TELEMETRY";
    return {};
  }
  bool apply(const VerticalOutput& value,Output& output,double now){
    if(!value.healthy){fault(value.reason);return false;}
    if(value.saturated){if(saturated_since_<0)saturated_since_=now;
      if(now-saturated_since_>=c_.saturation_seconds){fault("PERSISTENT_THRUST_SATURATION");return false;}}
    else saturated_since_=-1;
    output.thrust={0,0,-value.thrust};output.saturated=value.saturated;output.slew_limited=value.slew_limited;
    output.velocity_target=value.velocity_target;return true;
  }
  bool verified_ground(const Feedback& s) const{
    return s.status_fresh&&s.land_fresh&&s.local_fresh&&s.z_valid&&s.vz_valid&&
      s.landed&&s.at_rest&&std::isfinite(s.vz)&&std::abs(s.vz)<=c_.speed_tolerance&&
      z_set_&&std::isfinite(s.z)&&std::abs(z0_-s.z)<=c_.height_tolerance;
  }
  bool dwell(bool condition,double now,double duration){
    if(!condition){dwell_since_=-1;return false;}if(dwell_since_<0)dwell_since_=now;return now-dwell_since_>=duration;
  }
  // A hold observed at 20 Hz is not 20 independent land detector samples.
  // Dwell may complete ONLY on a NEW strictly-increasing detector stamp;
  // long gaps, regressions and bounce restart evidence acquisition.
  bool dwell_land(bool condition,double now,double duration,std::uint64_t stamp){
    const double max_gap=std::min(c_.land_sample_max_gap,c_.land_timeout*.5);
    if(!condition||!stamp||!std::isfinite(now)){
      // A detector stamp that witnessed a bounce / false condition cannot
      // later be relabelled as a fresh positive observation just because the
      // Z/VZ or landed cache changed between detector callbacks.
      land_dwell_blocked_stamp_=std::max(land_dwell_blocked_stamp_,stamp);
      clear_land_dwell();return false;
    }
    if(stamp<=land_dwell_blocked_stamp_)return false;
    if(land_dwell_last_stamp_&&stamp<land_dwell_last_stamp_){
      // A regressed detector sequence cannot prove a new touchdown/liftoff.
      land_dwell_blocked_stamp_=std::max(land_dwell_blocked_stamp_,land_dwell_last_stamp_);
      clear_land_dwell();return false;
    }
    if(stamp==land_dwell_last_stamp_){
      // Replaying the same message at loop_hz never counts as an independent
      // detector observation. In particular, it must not seed a fresh dwell
      // after a prolonged freeze (the original S10 did that).
      return false;
    }
    if(land_dwell_last_time_>=0&&
       (now<land_dwell_last_time_||now-land_dwell_last_time_>max_gap)){
      clear_land_dwell(); // Only THIS genuinely new message may start over.
    }
    if(land_dwell_first_time_<0){land_dwell_first_time_=now;land_dwell_first_stamp_=stamp;}
    land_dwell_last_time_=now;land_dwell_last_stamp_=stamp;
    // 20Hz steady-clock time accumulated in double may be a few ulps under
    // 1.000000 on an exact 1Hz detector. The nanosecond-scale epsilon does
    // NOT relax the requirement for two distinct detector messages.
    return stamp>land_dwell_first_stamp_&&now-land_dwell_first_time_+1e-9>=duration;
  }
  void clear_land_dwell(){land_dwell_first_time_=land_dwell_last_time_=-1;
    land_dwell_first_stamp_=land_dwell_last_stamp_=0;}
  void enter(Phase phase,double now){phase_=phase;entered_=now;dwell_since_=-1;
    clear_land_dwell();land_dwell_blocked_stamp_=0;command_sent_=false;reason_=name(phase);}
  void fault(const std::string& why,bool restart_required=true){
    restart_required_=restart_required_||restart_required;
    if(fault_class_==FaultClass::None||fault_class_==FaultClass::ControlledDescent){
      fault_class_=flight_started_?FaultClass::UnknownAirborne:FaultClass::GroundAbort;
    }
    ground_confirmed_=false;phase_=Phase::Safety;reason_=why;
    fault_status_baseline_=std::max(fault_status_baseline_,last_status_stamp_);
    fault_land_baseline_=std::max(fault_land_baseline_,last_land_stamp_);
    clear_land_dwell();land_dwell_blocked_stamp_=fault_land_baseline_;
  }
  Config c_;Phase phase_{Phase::Idle};AttitudeReference reference_;
  std::unique_ptr<VerticalController> vertical_;
  bool interactive_{},flight_started_{},ground_abort_{},z_set_{},command_sent_{};
  bool arm_cancel_pending_{},arm_command_committed_{};
  bool disarm_in_flight_{},disarm_command_committed_{},disarm_outcome_uncertain_{};
  bool operator_release_requested_{};
  bool ground_confirmed_{},liftoff_evidenced_{},last_sample_set_{},arm_confirmed_{},
       ground_release_pending_{};
  FaultClass fault_class_{FaultClass::None};
  double last_sample_z_{},last_sample_vz_{};
  std::uint64_t touchdown_baseline_{},disarm_land_commit_stamp_{};
  std::uint8_t z_reset_{},vz_reset_{};
  std::uint64_t status_baseline_{},land_baseline_{};
  double entered_{},last_{},ping_{-1},z0_{},target_{},command_height_{},flight_since_{};
  double dwell_since_{-1},saturated_since_{-1};
  double land_dwell_first_time_{-1},land_dwell_last_time_{-1};
  double recovery_deadline_{-1},recovery_entry_height_{};
  bool recovery_budget_exhausted_{},px4_land_committed_{},px4_land_attempted_{},px4_land_mode_observed_{},
       recovery_progress_observed_{},external_ground_disarmed_confirmed_{};
  std::uint64_t recovery_last_local_stamp_{},handoff_status_baseline_{},
    fault_status_baseline_{},fault_land_baseline_{},last_status_stamp_{},last_land_stamp_{};
  std::uint64_t land_dwell_first_stamp_{},land_dwell_last_stamp_{},land_dwell_blocked_stamp_{};
  std::string reason_{"IDLE_NO_AUTOMATIC_ARM"},event_;
  authority::State authority_;
  bool offboard_observed_{},restart_required_{};
  std::uint64_t switch_generation_{};
  std::string takeover_reason_;
};
} // namespace fc01::attitude
