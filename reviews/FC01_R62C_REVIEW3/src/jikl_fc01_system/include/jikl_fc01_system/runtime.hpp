#pragma once
#include "jikl_fc01_system/source_clock.hpp"
#include "jikl_fc01_system/ground_link.hpp"
#include "jikl_fc01_system/velocity_core.hpp"
#include "jikl_fc01_system/attitude_altitude_core.hpp"
#include "jikl_fc01_system/publication_guard.hpp"
#include "jikl_fc01_system/protocol.hpp"
#include <array>
#include <memory>
#include <sstream>
#include <vector>

namespace fc01 {
namespace v = jikl_velocity;
namespace g = jikl_fc01_ground_link;
struct Stamp {
  std::uint64_t value{};
  double received{-1};
  bool regressed{};
  bool accept(std::uint64_t next, double now) {
    if(next && value && next < value) regressed=true;
    if(!next || next<=value || regressed) return false;
    value=next;received=now;return true;
  }
  bool fresh(double now,double limit) const {
    return !regressed&&received>=0&&now>=received&&now-received<limit;
  }
};
struct Config {
  std::string profile{"ground"};
  bool flight_enabled{};
  attitude::Config attitude;
  v::Config control=[](){v::Config c;c.height=1;c.max_height=5;c.max_radius=3;
    c.xy_speed=.3;c.z_speed=.3;c.climb_speed=.3;return c;}();
  void validate() const {
    control.validate();attitude.validate();
    if(profile=="attitude_altitude"&&flight_enabled)
      throw std::invalid_argument("attitude_altitude uses separate flight_actions_enabled gates");
    if(profile!="ground"&&profile!="flight"&&profile!="sim"&&profile!="attitude_altitude")
      throw std::invalid_argument("profile must be ground, flight, sim, or attitude_altitude");
    if(profile=="ground"&&flight_enabled)
      throw std::invalid_argument("ground profile cannot enable flight");
    if(control.command_timeout!=.5||control.land_after_loss!=3.)
      throw std::invalid_argument("command timeout/lease timeout must be 0.5/3 seconds");
  }
};
struct Snapshot {
  v::Feedback feedback{};
  attitude::Feedback attitude;
  g::Traffic traffic{};
  std::uint64_t status_stamp{},land_stamp{};
  bool status_fresh{},land_fresh{},local_fresh{},at_rest{};
  // Pure-core fixtures historically supply an explicit armed/disarmed bool.
  // The real adapter MUST set this from the fixed raw arming-state enum.
  bool arming_state_known{true};
  bool battery_fresh{},battery_connected{},power_valid{},manual_fresh{},manual_lost{};
  bool dead_reckoning{},time_regressed{},calibration{},hil{};
  bool stabilized_selectable{true};
  int battery_warning{},nav_state{4},system{1},component{1};
  double stationary_seconds{};
  bool attitude_output_active{},offboard_heartbeat_active{},publication_fault_latched{};
};
struct Reply {bool accepted{};std::string message;};
struct Output {v::Output flight{};attitude::Output attitude;bool send_ground{};};
enum class Command400Kind {None,Arm,Disarm};
enum class CommandStage {None,Planned,CommittedAwaitAck,AckAcceptedAwaitState,Rejected,TimedOut,FinalSinkRejected,StateVerifiedAwaitAck,VerifiedTerminal,
  AckInProgress,Ambiguous400Ack,StateOnlyTerminal,UnknownAckResult,
  StateAckContradictory,StateAckObservedUnresolved};
inline const char* command400_name(Command400Kind kind){switch(kind){
  case Command400Kind::None:return "NONE";
  case Command400Kind::Arm:return "ARM";
  case Command400Kind::Disarm:return "DISARM";
}return "UNKNOWN";}
inline const char* command_stage_name(CommandStage stage){switch(stage){
  case CommandStage::None:return "NONE";
  case CommandStage::Planned:return "PLANNED";
  case CommandStage::CommittedAwaitAck:return "SINK_COMMITTED_AWAIT_ACK";
  case CommandStage::AckAcceptedAwaitState:return "ACK_ACCEPTED_AWAIT_STATE";
  case CommandStage::Rejected:return "ACK_REJECTED";
  case CommandStage::TimedOut:return "ACK_TIMEOUT";
  case CommandStage::FinalSinkRejected:return "FINAL_SINK_REJECTED";
  case CommandStage::StateVerifiedAwaitAck:return "PX4_STATE_VERIFIED_ACK_MISSING";
  case CommandStage::VerifiedTerminal:return "ACK_AND_PX4_STATE_VERIFIED";
  case CommandStage::AckInProgress:return "ACK_IN_PROGRESS_AWAIT_STATE";
  case CommandStage::Ambiguous400Ack:return "AMBIGUOUS_400_ACK";
  case CommandStage::StateOnlyTerminal:return "PX4_STATE_ONLY_TERMINAL_ACK_UNATTRIBUTED";
  case CommandStage::UnknownAckResult:return "UNKNOWN_ACK_RESULT_AWAIT_STATE";
  case CommandStage::StateAckContradictory:return "PX4_STATE_EVIDENCED_ACK_CONTRADICTORY";
  case CommandStage::StateAckObservedUnresolved:return "PX4_STATE_EVIDENCED_ACK_OBSERVED_UNRESOLVED";
}return "UNKNOWN";}
// Exact PX4 v1.16/MAV_RESULT names. FAILED(4) is not DENIED(2).
inline const char* ack_result_name(int result){switch(result){
  case -1:return "NOT_OBSERVED";case 0:return "ACCEPTED";
  case 1:return "TEMPORARILY_REJECTED";case 2:return "DENIED";
  case 3:return "UNSUPPORTED";case 4:return "FAILED";
  case 5:return "IN_PROGRESS";case 6:return "CANCELLED";
}return "UNKNOWN_RESULT";}
struct Pending {
  // sent: ACK-eligible wire publish (or synthetic runtime-only tests).
  // requested: core asked for command; published: final node sink succeeded.
  bool sent{},accepted{},rejected{},timed_out{},requested{},published{},sink_denied{};
  int result{-1};double sent_at{};std::uint64_t baseline{},last_ack{};
  std::uint64_t wire_stamp{},status_at_wire{},land_at_wire{};
  double rejected_at{-1},ack_received_at{-1};
  Command400Kind command400_kind{Command400Kind::None};
  CommandStage stage{CommandStage::None};
  std::uint64_t ack_stamp{},verified_status_stamp{};
  bool state_verified{};
  double verified_at{-1};
  // generation is a LOCAL audit ordinal, never a wire nonce. Once 400 has
  // been reused, *all* its ACK results are observations, not attributed ACKs,
  // even if an earlier ACK + state was seen (duplicates can still arrive).
  unsigned generation{},ack_observations{};
  bool ack_ambiguous{},previous400_unresolved{};
  bool source_clock_validated{};
  std::uint64_t causal_floor{};
  unsigned unattributed_clock_acks{};
};
class Runtime {
 public:
  explicit Runtime(Config c={}):config_(std::move(c)),core_(config_.control),attitude_core_(config_.attitude){config_.validate();}
  v::Phase phase() const{return core_.phase();}
  bool active() const{return config_.profile=="attitude_altitude"?attitude_core_.active():core_.active()||recover_land_;}
  bool shutting_down() const{return shutdown_latched_;}
  void begin_shutdown(const Snapshot& s,double now){
    if(shutdown_latched_)return;
    shutdown_latched_=true;request("LAND",{},s,now);
  }
  void abort_attitude(const std::string& reason){attitude_core_.block(reason);message_=reason;}
  const attitude::Core& attitude_core() const{return attitude_core_;}
  const char* phase_name() const{return config_.profile=="attitude_altitude"?attitude::name(attitude_core_.phase()):v::name(core_.phase());}
  g::State ground_state() const{return ground_?ground_->state():g::State::Waiting;}
  const std::string& message() const{return message_;}
  const v::Core& flight_core() const{return core_;}
  const Pending& pending(unsigned cmd) const{return pending_[index(cmd)];}
  const std::vector<Pending>& command400_history() const{return command400_history_;}
  // Enable in the production ROS adapter. Pure-core fixtures intentionally
  // retain their historical implicit publication model.
  void require_attitude_sink_commit(){if(config_.profile=="attitude_altitude")
    attitude_sink_commit_required_=true;}
  void confirm_attitude_command_publication(attitude::Command command,bool allowed,
      std::uint64_t wire_stamp,const Snapshot& actual,double now){
    if(command==attitude::Command::None)return;
    const unsigned cmd=command==attitude::Command::Offboard||
      command==attitude::Command::ReleaseOffboard?176:command==attitude::Command::Px4Land?21:400;
    auto& p=pending_[index(cmd)];
    if(!p.requested)return;
    if(p.published)return; // Duplicate local completion cannot commit twice.
    if(cmd==400&&p.command400_kind!=
      (command==attitude::Command::Arm?Command400Kind::Arm:Command400Kind::Disarm)){
      // Same Command 400 number does not identify its param1. Never commit
      // the wrong Arm/Disarm transaction to a stale pending tracker.
      attitude_core_.block("COMMAND_400_INTENT_MISMATCH");return;
    }
    // "validated" records that this adapter uses/checks the synchronized
    // domain, independently of whether THIS post-sink sample is still fresh.
    // A committed command cannot lose its wire floor when telemetry expires.
    const bool source_clock=actual.attitude.source_clock_validated;
    const auto causal_floor=source_clock?source_causal_floor(wire_stamp):0;
    attitude_core_.note_command_publication(command,allowed,
      source_clock?std::max(actual.land_stamp,causal_floor):actual.land_stamp,
      source_clock?std::max(actual.status_stamp,causal_floor):actual.status_stamp);
    if(allowed){p.published=true;p.sent=true;p.sent_at=now;p.wire_stamp=wire_stamp;
      p.source_clock_validated=source_clock;
      p.causal_floor=causal_floor;
      p.status_at_wire=actual.status_stamp;p.land_at_wire=actual.land_stamp;
      p.baseline=std::max(p.baseline,actual.status_stamp);
      if(cmd==400)record400_commit(p);
      p.stage=CommandStage::CommittedAwaitAck;}
    else {p.published=false;p.sent=false;p.sink_denied=true;p.rejected=true;p.rejected_at=now;
      p.result=-1;p.stage=CommandStage::FinalSinkRejected;}
  }
  attitude::BoundaryContext publication_context(const Snapshot& s,double now,bool connected,bool control_fresh,bool shutdown=false) const{
    attitude::BoundaryContext c;c.profile=config_.profile;c.phase=attitude_core_.phase();
    c.config=config_.attitude;c.feedback=attitude_snapshot(s);c.shutdown=shutdown||shutdown_latched_;
    c.session_connected=connected;c.control_fresh=control_fresh;c.operator_lease_live=attitude_core_.operator_lease_live(now);
    c.preparation_lease_valid=attitude_core_.preparation_lease_valid(now);c.flight_started=attitude_core_.flight_started();
    c.reference_ready=attitude_core_.reference_ready();c.reference_z=attitude_core_.reference_z();
    c.reference_z_reset=attitude_core_.reference_z_reset();
    c.reference_vz_reset=attitude_core_.reference_vz_reset();
    c.reference_quat_reset=attitude_core_.reference_quat_reset();
    c.authority=attitude_core_.authority_state();c.offboard_observed=attitude_core_.offboard_observed();
    c.feedback.operator_takeover=attitude_core_.operator_intervened(c.feedback)&&
      !(attitude_core_.operator_release_requested()&&c.feedback.offboard&&!c.feedback.kill);
    c.ground_confirmed=attitude_core_.ground_confirmed();
    c.ground_release_pending=attitude_core_.ground_release_pending();
    c.arm_confirmed=attitude_core_.arm_confirmed();
    c.bench_arm_only_active=attitude_core_.bench_arm_only_active();
    c.bench_disarm_proven=attitude_core_.bench_disarm_proven();
    c.arm_command_committed=attitude_core_.arm_command_committed();
    c.bench_arm_status_baseline=attitude_core_.bench_arm_status_baseline();
    c.bench_arm_local_baseline=attitude_core_.bench_arm_local_baseline();
    c.bench_disarm_status_floor=attitude_core_.bench_disarm_status_floor();
    c.bench_disarm_local_floor=attitude_core_.bench_disarm_local_floor();
    c.bench_disarm_land_floor=attitude_core_.bench_disarm_land_floor();
    c.arm_cancel_pending=attitude_core_.arm_cancel_pending();
    c.px4_land_command_committed=attitude_core_.px4_land_command_committed();
    // Boundary replay protection follows the ACTUAL sink audit, not the
    // historical implicit-publication model used by old pure-Core fixtures.
    c.disarm_command_committed=pending_[index(400)].command400_kind==Command400Kind::Disarm&&
      pending_[index(400)].published;
    c.touchdown_baseline=attitude_core_.touchdown_baseline();
    c.yaw_reference=attitude_core_.yaw_reference();
    c.shutdown=c.shutdown||s.publication_fault_latched;return c;
  }
  std::string authority_json(const Snapshot& s) const{
    const auto& state=attitude_core_.authority_state();const auto a=attitude_snapshot(s);
    const auto in=attitude::authority_inputs(config_.attitude,a);
    const auto policy=authority::evaluate(state,in,false);
    const bool restart=shutdown_latched_||s.publication_fault_latched||attitude_core_.restart_required()||
      (attitude_core_.phase()==attitude::Phase::Safety&&!state.takeover_latched);
    const bool takeover=state.takeover_latched||attitude_core_.takeover_cause(a);
    const bool blocked=restart||takeover;
    auto boolean=[](bool value){return value?"true":"false";};
    auto number=[](double value){return std::isfinite(value)?std::to_string(value):std::string("null");};
    const auto& ground=attitude_core_.authority_ground_evidence();
    const char* why=restart?"AUTHORITY_SAFETY_RESTART_REQUIRED":takeover?"OPERATOR_TAKEOVER_LATCHED":authority::name(policy);
    return std::string("\"authority_mode\":")+quote(authority::name(state.mode))+
      ",\"authority_attempt_mode\":"+quote(authority::name(state.attempt_mode))+
      ",\"authority_locked\":"+boolean(state.locked)+
      ",\"authority_ready\":"+boolean(!blocked&&policy==authority::Reason::Allowed)+
      ",\"authority_reason\":"+quote(why)+
      ",\"rc_link_ready\":"+boolean(in.rc_link_ready)+
      ",\"rc_source_seen\":"+boolean(a.rc_source_seen)+
      ",\"rc_source_valid\":"+boolean(a.rc_source_valid)+
      ",\"rc_source_selected\":"+boolean(a.rc_source_selected)+
      ",\"rc_data_source\":"+std::to_string(unsigned(a.rc_data_source))+
      ",\"rc_sticks_moving\":"+boolean(a.rc_sticks_moving)+
      ",\"rc_switches_seen\":"+boolean(a.rc_switches_seen)+
      ",\"rc_switches_fresh\":"+boolean(a.rc_switches_fresh)+
      ",\"rc_switches_ready\":"+boolean(in.rc_switches_ready)+
      ",\"rc_mode_slot\":"+std::to_string(unsigned(a.rc_mode_slot))+
      ",\"rc_arm_switch\":"+std::to_string(unsigned(a.rc_arm_switch))+
      ",\"rc_kill_switch\":"+std::to_string(unsigned(a.rc_kill_switch))+
      ",\"rc_kill_latched\":"+boolean(a.kill)+
      ",\"rc_takeover_ready\":"+boolean(!blocked&&in.rc_link_ready&&in.rc_switches_ready&&in.rc_takeover_confirmed)+
      ",\"rc_takeover_confirmed\":"+boolean(in.rc_takeover_confirmed)+
      ",\"autonomous_failsafe_ready\":"+boolean(!blocked&&in.autonomous_failsafe_confirmed)+
      ",\"autonomous_failsafe_confirmed\":"+boolean(in.autonomous_failsafe_confirmed)+
      ",\"operator_takeover_latched\":"+boolean(state.takeover_latched)+
      ",\"takeover_reason\":"+quote(attitude_core_.takeover_reason())+
      ",\"bench_disarm_ground_proven\":"+boolean(attitude_core_.bench_disarm_proven())+
      ",\"bench_disarm_status_baseline\":"+std::to_string(attitude_core_.bench_arm_status_baseline())+
      ",\"bench_disarm_local_baseline\":"+std::to_string(attitude_core_.bench_arm_local_baseline())+
      ",\"bench_ground_down_tolerance_m\":"+number(config_.attitude.bench_ground_down_tolerance_m)+
      ",\"bench_ground_vz_tolerance_mps\":"+number(config_.attitude.bench_ground_vz_tolerance_mps)+
      ",\"bench_ground_dwell_s\":"+number(attitude_core_.bench_ground_dwell_seconds())+
      ",\"stationary_seconds\":"+number(a.stationary_seconds)+
      ",\"required_ground_seconds\":"+number(config_.attitude.ground_seconds)+
      ",\"ground_vz_tolerance\":"+number(config_.attitude.ground_vz_tolerance)+
      ",\"ground_vz_ned\":"+number(a.vz)+
      ",\"authority_ground_failed_mask\":"+std::to_string(attitude_core_.authority_ground_failed_mask(a))+
      ",\"last_authority_ground_evidence_valid\":"+boolean(ground.valid)+
      ",\"last_authority_ground_failed_mask\":"+std::to_string(ground.failed_mask)+
      ",\"last_authority_ground_stationary_seconds\":"+number(ground.stationary_seconds)+
      ",\"last_authority_ground_vz_ned\":"+number(ground.vz)+
      ",\"last_authority_ground_status_stamp\":"+std::to_string(ground.status_stamp)+
      ",\"last_authority_ground_land_stamp\":"+std::to_string(ground.land_stamp)+
      ",\"last_authority_ground_local_stamp\":"+std::to_string(ground.local_stamp);
  }
  std::string ground_name() const{return ground_?g::name(ground_->state()):"NOT_RUN";}
  std::string ground_reason() const{return ground_?ground_->reason():"NOT_RUN";}
  Reply request(const std::string& op,const std::vector<double>& a,const Snapshot& s,double now){
    for(double n:a)if(!std::isfinite(n))return refuse("NONFINITE_ARGUMENT");
    const std::size_t arity=op=="TAKEOFF"?1:op=="VEL"?4:0;
    if(a.size()!=arity)return refuse("ARGUMENT_COUNT");
    if(op=="STATUS")return {true,"STATUS"};
    if(op=="AUTHORITY_STATUS")return config_.profile=="attitude_altitude"?Reply{true,"AUTHORITY_STATUS"}:refuse("AUTHORITY_REQUIRES_ATTITUDE_PROFILE");
    if(shutdown_latched_&&op!="PING"&&op!="LAND"&&op!="STOP"&&op!="UPDATE_CHECK")
      return refuse("SHUTDOWN_NO_NEW_CONTROL_TASKS");
    if(op=="INTERACTIVE"&&config_.profile!="attitude_altitude")return {true,"LIVE_CONSOLE_SESSION"};
    if(config_.profile=="attitude_altitude"&&op!="MODE"&&op!="UPDATE_CHECK"){
      if(maintenance_&&op!="PING"&&op!="LAND"&&op!="STOP")return refuse("MAINTENANCE_RESTART_REQUIRED");
      if(ground_&&!ground_->finished()&&op!="PING")return refuse("GROUND_MODE_TEST_ACTIVE");
      auto feedback=attitude_snapshot(s);std::string why;
      if(op=="PING"){attitude_core_.ping(now);return {true,"ALIVE"};}
      if(op=="INTERACTIVE"){attitude_core_.live_console(now);return {true,"LIVE_CONSOLE_SESSION"};}
      const bool outputs_active=last_attitude_output_active_||s.attitude_output_active||s.offboard_heartbeat_active;
      const auto outstanding=[](const Pending& p){
        // A state-only safe Disarm terminal ends physical recovery, not ACK
        // ambiguity. Core's reset admission independently rechecks CURRENT
        // disarmed ground, stopped outputs and mode. Retain history below.
        if(p.state_verified&&(p.command400_kind==Command400Kind::Disarm||
           p.command400_kind==Command400Kind::None))return false;
        return p.sent&&(p.ack_ambiguous||(!p.accepted&&!p.rejected&&!p.timed_out));
      };
      const bool transaction=outstanding(pending_[0])||outstanding(pending_[1]);
      if(op=="AUTHORITY_RC"||op=="AUTHORITY_AUTONOMOUS")
        why=attitude_core_.select_authority(op=="AUTHORITY_RC"?authority::Mode::RcTakeover:authority::Mode::AutonomousNoRc,
          feedback,now,outputs_active,transaction,s.publication_fault_latched);
      else if(op=="AUTHORITY_RESET")why=attitude_core_.reset_authority(feedback,now,outputs_active,transaction,s.publication_fault_latched);
      else if(op=="AUTHORITY_TAKEOVER"){
        if(!attitude_core_.operator_lease_live(now))return refuse("LIVE_INTERACTIVE_CONSOLE_REQUIRED");
        if(!attitude_core_.active())return refuse("NO_ACTIVE_COMPANION_ATTEMPT");
        why=attitude_core_.request_operator_takeover(feedback,now);
      }
      else if(op=="ATTITUDE_READY")why=attitude_core_.prepare(feedback,now);
      else if(op=="ARM_ONLY"){
        if(command400_commits_>0)return refuse("COMMAND_400_REUSE_REQUIRES_SAFE_GROUND_RESTART");
        why=attitude_core_.arm_only(feedback,now);
      }
      else if(op=="TAKEOFF"){
        // A new mission must not commit an Arm whose ACK can never be
        // attributed. Ordinary ground reset cannot erase protocol history.
        if(command400_commits_>0)return refuse("COMMAND_400_REUSE_REQUIRES_SAFE_GROUND_RESTART");
        why=attitude_core_.takeoff(feedback,a[0],now);
      }
      else if(op=="LAND")why=attitude_core_.descend(feedback,now);
      else if(op=="STOP"){
        if(!s.status_fresh||!s.land_fresh||s.feedback.armed||!s.feedback.landed||!s.at_rest)
          return refuse("STOP_REQUIRES_CONFIRMED_DISARMED_GROUND_STATE");
        attitude_core_.stop_disarmed(feedback,now);return {true,"STOP_ALLOWED"};
      }else return refuse("COMMAND_NOT_AVAILABLE_IN_ATTITUDE_PROFILE");
      if(!why.empty())return refuse(why);
      message_=op=="ATTITUDE_READY"?"ATTITUDE_PREPARATION_PENDING_ACK_AND_NEW_STATUS":op+"_ACCEPTED";
      return {true,message_};
    }
    if(op=="PING"){
      last_ping_=now;
      if(now>=velocity_until_)core_.receive({},now);
      return {true,"ALIVE"};
    }
    if(op=="UPDATE_CHECK"){
      if(active()||(ground_&&!ground_->finished()))return refuse("TASK_ACTIVE");
      if(!s.status_fresh||!s.land_fresh||s.time_regressed)return refuse("GROUND_TELEMETRY_REQUIRED");
      if(s.feedback.armed||!s.feedback.landed||!s.at_rest||s.stationary_seconds<2||s.feedback.offboard)
        return refuse("UPDATE_REQUIRES_DISARMED_LANDED_AT_REST_OUTSIDE_OFFBOARD");
      if(!s.feedback.exclusive)return refuse("UPDATE_REQUIRES_UNIQUE_STATUS");
      maintenance_=true;
      return {true,"UPDATE_ALLOWED"};
    }
    if(op=="STOP"){
      auto permit=request("UPDATE_CHECK",{},s,now);
      return permit.accepted?Reply{true,"STOP_ALLOWED"}:permit;
    }
    if(maintenance_&&op!="LAND")return refuse("MAINTENANCE_RESTART_REQUIRED");
    if(op=="MODE"){
      if(active()||(ground_&&!ground_->finished()))return refuse("TASK_ACTIVE");
      if(s.time_regressed)return refuse("TIME_REGRESSION_RESTART_ON_GROUND");
      if(!s.feedback.exclusive)return refuse("SOURCE_OR_CONTROL_CONFLICT");
      if(!s.status_fresh||!s.land_fresh||s.feedback.armed||!s.feedback.landed||!s.at_rest)
        return refuse("MODE_REQUIRES_FRESH_DISARMED_GROUND_STATE");
      ground_=std::make_unique<g::Core>();ground_origin_=now;
      return {true,"MODE_PENDING_ACK_AND_NEW_STATUS"};
    }
    if(op=="LAND"){
      if(ground_&&!ground_->finished())return refuse("GROUND_MODE_TEST_ACTIVE");
      if(!s.status_fresh||s.time_regressed||!s.feedback.exclusive)
        return refuse("LAND_REQUIRES_UNIQUE_FRESH_STATUS");
      velocity_until_=now;core_.invalidate();
      if(core_.active())core_.land(s.feedback,now,"operator requested landing");
      else if(s.feedback.armed){recover_land_=true;recover_started_=now;recover_nav_=s.nav_state;pending_[2]={};}
      return {true,s.feedback.armed?"LAND_PENDING_ACK_AND_STATE":"ALREADY_DISARMED"};
    }
    if(op=="HOLD"){
      if(!core_.accepts_motion())return refuse("HOLD_REQUIRES_ACTIVE_FLIGHT");
      last_ping_=now;velocity_until_=now;core_.receive({},now);return {true,"BRAKE_THEN_HOLD"};
    }
    if(op=="VEL"){
      if(!core_.accepts_motion())return refuse("VELOCITY_REQUIRES_SETTLED_TAKEOFF");
      if(!s.feedback.fresh||!s.feedback.valid||!s.feedback.heading_ok||!s.feedback.exclusive)
        return refuse("VELOCITY_STATE_INVALID");
      if(std::hypot(a[0],a[1])>config_.control.xy_speed||std::abs(a[2])>config_.control.z_speed||std::abs(a[3])>config_.control.yaw_speed)
        return refuse("VELOCITY_EXCEEDS_CONFIGURED_LIMIT");
      last_ping_=now;velocity_until_=now+.5;
      const bool ok=core_.receive({a[0],a[1],a[2],a[3]},now);
      return ok?Reply{true,"VELOCITY_ACCEPTED_0.5S"}:refuse("VELOCITY_REJECTED");
    }
    if(op=="TAKEOFF"){
      if(!config_.flight_enabled||config_.profile=="ground")return refuse("FLIGHT_DISABLED");
      if(active()||(ground_&&!ground_->finished()))return refuse("TASK_ACTIVE");
      const auto why=hardware_gate(s);if(!why.empty())return refuse(why);
      if(!s.feedback.fresh||!s.feedback.valid||!s.feedback.heading_ok||s.dead_reckoning)
        return refuse("XY_VXY_Z_VZ_HEADING_VALID_AIDED_ESTIMATE_REQUIRED");
      if(s.stationary_seconds<2||!s.at_rest)return refuse("GROUND_REST_WINDOW_2S_REQUIRED");
      if(s.time_regressed)return refuse("TIME_REGRESSION_RESTART_ON_GROUND");
      if(a[0]<=0||a[0]>config_.control.max_height-1.)return refuse("HEIGHT_OUTSIDE_ENVELOPE_1M_MARGIN_REQUIRED");
      if(last_ping_<0||now-last_ping_>.5)return refuse("LIVE_OPERATOR_CONSOLE_REQUIRED");
      auto next=config_.control;next.height=a[0];next.validate();
      v::Core candidate(next);candidate.receive({},now);
      if(!candidate.start(s.feedback,now))return refuse(candidate.reason());
      core_=std::move(candidate);pending_={};velocity_until_=now;
      message_="TAKEOFF_PENDING_OFFBOARD_ACK_ARM_ACK_AND_FEEDBACK";
      return {true,message_};
    }
    return refuse("UNKNOWN_COMMAND");
  }
  void ack(const g::Ack& a,double now){
    if(ground_&&!ground_->finished())ground_->ack(a,now-ground_origin_);
    if(a.command!=176&&a.command!=400&&a.command!=21)return;
    auto& p=pending_[index(a.command)];
    if(a.system!=1||a.component!=191)return;
    // from_external is not a correlation key for PX4 v1.16 Commander ACKs:
    // answer_command() leaves it false even when the originating VehicleCommand
    // was external. These filters rule OUT some ACKs; they cannot prove which
    // Arm/Disarm a reused 400 ACK belongs to.
    if(!p.sent){if(a.timestamp>p.baseline)p.baseline=a.timestamp;return;}
    if(p.source_clock_validated){
      if(!std::isfinite(now)||now<p.sent_at)return;
      const double receipt_upper=double(p.wire_stamp)+(now-p.sent_at)*1e6+double(source_clock_skew_budget_us);
      if(a.timestamp<=p.causal_floor||double(a.timestamp)>receipt_upper){
        ++p.unattributed_clock_acks;return;
      }
    }
    if(a.command==400&&config_.profile=="attitude_altitude"&&p.ack_ambiguous){
      if(!std::isfinite(now)||now<p.sent_at||a.timestamp<=p.baseline||
         a.timestamp<=p.last_ack||a.timestamp<=p.status_at_wire)return;
      p.last_ack=a.timestamp;p.ack_stamp=a.timestamp;p.ack_received_at=now;
      p.result=a.result;++p.ack_observations;
      // Observation remains useful even after timeout / state-only completion.
      // No ACCEPTED, FAILED, DENIED, TEMP_REJECTED, IN_PROGRESS or CANCELLED
      // result can authorize completion or a retry of this reused transaction.
      if(now-p.sent_at>=config_.attitude.command_timeout)p.timed_out=true;
      p.stage=p.state_verified?CommandStage::StateOnlyTerminal:CommandStage::Ambiguous400Ack;
      return;
    }
    if(p.accepted||p.rejected||p.timed_out||now<p.sent_at||a.timestamp<=p.baseline||
       a.timestamp<=p.last_ack||a.timestamp<=p.status_at_wire)return;
    if(now-p.sent_at>=5.){p.timed_out=true;p.stage=CommandStage::TimedOut;return;}
    p.last_ack=a.timestamp;p.ack_stamp=a.timestamp;p.ack_received_at=now;p.result=a.result;++p.ack_observations;
    switch(a.result){
      case 0:p.accepted=true;p.stage=CommandStage::AckAcceptedAwaitState;break;
      case 5:p.stage=CommandStage::AckInProgress;break;
      case 1:case 2:case 3:case 4:case 6:p.rejected=true;p.stage=CommandStage::Rejected;
      // Legacy velocity profile retains its existing rejected Arm handling;
      // the attitude profile processes Arm AND Disarm refusal separately.
        if(a.command==400&&config_.profile!="attitude_altitude")core_.rejected_arm();
        break;
      default:
        // Preserve legacy 176/21/velocity handling. Unknown 400 results in
        // attitude mode remain uncertain rather than claiming a refusal.
        if(a.command!=400||config_.profile!="attitude_altitude"){
          p.rejected=true;p.stage=CommandStage::Rejected;
          if(a.command==400)core_.rejected_arm();
        }else p.stage=CommandStage::UnknownAckResult;
        break;
    }
  }
  void disconnect(const Snapshot& s,double now){
    if(config_.profile=="attitude_altitude"){attitude_core_.disconnect(attitude_snapshot(s),now);message_=attitude_core_.reason();return;}
    last_ping_=-1;velocity_until_=now;core_.invalidate();
    if(core_.active())core_.land(s.feedback,now,"operator console disconnected");
    message_="CONSOLE_DISCONNECTED_OLD_MOTION_CLEARED";
  }
  Output step(const Snapshot& s,double now){
    Output out;
    if(ground_&&!ground_->finished()){
      if(s.time_regressed||!s.status_fresh||!s.land_fresh){
        ground_->abort(s.time_regressed?"TIME_REGRESSION_RESTART_ON_GROUND":!s.status_fresh?"STATUS_STALE":"LAND_STALE");
        message_=ground_->reason();return out;
      }
      ground_->status({s.status_stamp,!s.feedback.armed,s.feedback.failsafe,s.calibration,s.hil,s.stabilized_selectable,s.system,s.component,s.nav_state},now-ground_origin_);
      ground_->land({s.land_stamp,s.feedback.landed,s.at_rest},now-ground_origin_);
      auto traffic=s.traffic;
      if(!s.feedback.exclusive)traffic.command_publishers=2;
      out.send_ground=ground_->tick(now-ground_origin_,traffic);
      message_=ground_->reason();return out;
    }
    if(config_.profile=="attitude_altitude"){
      auto& command400=pending_[index(400)];
      const bool disarm_state=disarm_state_evidence(s,now);
      if(command400.sent&&!command400.accepted&&!command400.rejected&&
         now-command400.sent_at>=config_.attitude.command_timeout){
        command400.timed_out=true;
        if(!command400.state_verified)command400.stage=command400.ack_ambiguous?
          CommandStage::Ambiguous400Ack:CommandStage::TimedOut;
      }
      auto terminal=[this,disarm_state](unsigned cmd){const auto& p=pending_[index(cmd)];
        return attitude::Transaction{p.accepted,p.rejected,p.timed_out,p.ack_ambiguous,
          cmd==400&&disarm_state};};
      auto& land21=pending_[index(21)];
      if(land21.sent&&!land21.accepted&&!land21.rejected&&
         now-land21.sent_at>=config_.attitude.px4_land_ack_seconds)land21.timed_out=true;
      out.attitude=attitude_core_.step(attitude_snapshot(s),now,terminal(176),terminal(400),terminal(21));
      auto& mode176=pending_[index(176)];
      if(mode176.published&&attitude_core_.ground_mode_state_only_allowed()&&
         s.status_fresh&&s.land_fresh&&!s.feedback.armed&&s.feedback.landed&&s.at_rest&&
         s.status_stamp>mode176.status_at_wire&&
         (!mode176.source_clock_validated||s.status_stamp>mode176.causal_floor)&&
         ((attitude_core_.phase()==attitude::Phase::OffboardReady&&s.feedback.offboard&&s.nav_state==14)||
          (attitude_core_.phase()==attitude::Phase::Idle&&!s.feedback.offboard&&s.nav_state==15))){
        mode176.state_verified=true;mode176.verified_status_stamp=s.status_stamp;mode176.verified_at=now;
        mode176.stage=mode176.accepted?CommandStage::VerifiedTerminal:CommandStage::StateOnlyTerminal;
      }
      // A freshly observed PX4 armed/disarmed status is a separate channel of
      // evidence from the Command 400 ACK. Retain it even if the ACK is lost;
      // otherwise the next Disarm would collide with an old unresolved Arm
      // and the previous Runtime would cut the healthy Offboard stream.
      if(command400.sent&&s.arming_state_known&&s.status_fresh&&
         (!command400.source_clock_validated||s.status_stamp>command400.causal_floor)&&
         s.status_stamp>command400.status_at_wire&&
         s.status_stamp>command400.baseline&&
         ((command400.command400_kind==Command400Kind::Arm&&s.feedback.armed&&
           attitude_core_.active()&&attitude_core_.arm_confirmed())||
          (command400.command400_kind==Command400Kind::Disarm&&disarm_state&&
           attitude_core_.phase()==attitude::Phase::SafeDisarmed))){
        // Candidate evidence must also survive Core's reference-reset, stall,
        // estimator-jump and envelope checks on THIS step. A Safety label is
        // never a successful state-only Disarm terminal.
        command400.state_verified=true;
        command400.verified_at=now;command400.verified_status_stamp=s.status_stamp;
        command400.stage=command400.ack_ambiguous||
          (command400.command400_kind==Command400Kind::Disarm&&!command400.accepted)?
          CommandStage::StateOnlyTerminal:command400.accepted?
          CommandStage::VerifiedTerminal:command400.rejected?CommandStage::StateAckContradictory:
          command400.ack_observations?CommandStage::StateAckObservedUnresolved:CommandStage::StateVerifiedAwaitAck;
      }
      if(out.attitude.command!=attitude::Command::None){
        const unsigned cmd=(out.attitude.command==attitude::Command::Offboard||
          out.attitude.command==attitude::Command::ReleaseOffboard)?176:
          out.attitude.command==attitude::Command::Px4Land?21:400;
        auto& p=pending_[index(cmd)];
        if(p.sent&&!p.accepted&&!p.rejected&&!p.timed_out&&!p.state_verified){
          // Runtime prevents a second same-ID 400 command when neither the
          // original ACK NOR its independent PX4 terminal state is known.
          // Core recovery should normally defer this attempt, but if the
          // precondition fails we cannot justify publishing either command.
          attitude_core_.block("INDISTINGUISHABLE_OUTSTANDING_COMMAND");out.attitude={};
        }else {
          const auto watermark=std::max({p.baseline,p.last_ack,p.status_at_wire});
          const bool unresolved=cmd==400&&p.sent&&
            (p.ack_ambiguous||!p.accepted||!p.state_verified);
          if(cmd==400&&p.requested)command400_history_.push_back(p);
          p={};p.baseline=watermark;p.previous400_unresolved=unresolved;
          track(cmd,s,now,out.attitude.command);
        }
      }
      last_attitude_output_active_=out.attitude.publish;
      message_=attitude_core_.reason();return out;
    }
    if(recover_land_){
      if(s.status_fresh&&s.land_fresh&&!s.feedback.armed&&s.feedback.landed){recover_land_=false;message_="LANDED_AND_DISARMED_CONFIRMED";return out;}
      if(!s.status_fresh||!s.feedback.exclusive||s.feedback.failsafe||s.time_regressed||now-recover_started_>60){recover_land_=false;message_="RECOVERY_RELEASED_TO_PX4_OPERATOR";return out;}
      if(!s.feedback.landing&&s.nav_state!=recover_nav_){recover_land_=false;message_="OPERATOR_MODE_CHANGED_RECOVERY_RELEASED";return out;}
      const auto why=land_handshake_failure(s,now);
      if(!why.empty()){recover_land_=false;message_=why;return out;}
      if(!pending_[2].sent){out.flight.command=v::VehicleCommand::Land;track(21,s,now);}
      return out;
    }
    // Fault still needs the reused core's unresolved, late-ARM recovery path.
    if(!core_.active()&&core_.phase()!=v::Phase::Fault)return out;
    if(core_.phase()==v::Phase::Fault&&s.time_regressed){core_.release(now,"time regression; unresolved ARM released to PX4/operator");message_=core_.reason();return out;}
    auto actual=s.feedback;
    const auto why=hardware_gate(s);
    if(!why.empty()||s.time_regressed||!s.feedback.heading_ok||s.dead_reckoning)
      core_.land(actual,now,why.empty()?"estimate/time invalid":why);
    if(last_ping_<0||now-last_ping_>3.)core_.land(actual,now,"operator heartbeat lost");
    if(s.feedback.fresh&&core_.origin().z-s.feedback.position.z>config_.control.max_height)
      core_.land(actual,now,"height envelope exceeded");
    for(std::size_t i=0;i<2;++i){auto& p=pending_[i];
      if(p.sent&&!p.accepted&&(p.rejected||now-p.sent_at>=5.)){
        p.timed_out=!p.rejected;core_.land(actual,now,p.rejected?"PX4 command rejected":"PX4 command ACK timeout");
      }
    }
    if(core_.phase()==v::Phase::Landing){
      const auto failed=land_handshake_failure(s,now);
      if(!failed.empty()){core_.release(now,failed);message_=failed;return out;}
    }
    auto feedback=actual;
    // Never let a mode change bypass its ACK and cause ARM to be emitted.
    if(core_.phase()==v::Phase::Offboard)feedback.offboard=actual.offboard&&pending_[0].accepted;
    if(core_.phase()==v::Phase::Arm)feedback.armed=actual.armed&&pending_[1].accepted;
    out.flight=core_.step(feedback,now);
    if(out.flight.command!=v::VehicleCommand::None){
      const unsigned cmd=out.flight.command==v::VehicleCommand::Offboard?176:out.flight.command==v::VehicleCommand::Arm?400:21;
      auto& p=pending_[index(cmd)];
      if(p.sent)out.flight.command=v::VehicleCommand::None;
      else track(cmd,s,now);
    }
    if(!core_.reason().empty())message_=core_.reason();
    return out;
  }
 private:
  bool disarm_state_evidence(const Snapshot& s,double now) const{
    const auto& p=pending_[index(400)];
    if(!p.sent||p.sink_denied||!s.arming_state_known||p.command400_kind!=Command400Kind::Disarm||
       (p.source_clock_validated&&(s.status_stamp<=p.causal_floor||s.land_stamp<=p.causal_floor))||
       s.status_stamp<=p.status_at_wire||s.status_stamp<=p.baseline||
       s.land_stamp<=p.land_at_wire||s.system!=1||s.component!=1||
       !s.feedback.offboard||s.nav_state!=14||s.feedback.armed)return false;
    const auto c=publication_context(s,now,true,true);
    return attitude::boundary_health(c).allowed&&attitude::flight_gates(config_.attitude).allowed&&
      attitude::boundary_authority(c).allowed&&c.reference_ready&&c.feedback.landed&&c.feedback.at_rest&&
      (c.bench_arm_only_active&&c.bench_disarm_proven?
        attitude::bench_ground_observation(config_.attitude,c.feedback,c.reference_z):
        (std::isfinite(c.reference_z)&&std::abs(c.reference_z-c.feedback.z)<=config_.attitude.height_tolerance&&
         std::abs(c.feedback.vz)<=config_.attitude.speed_tolerance));
  }
  void record400_commit(Pending& p){
    if(p.generation)return;
    p.ack_ambiguous=command400_commits_>0;p.generation=++command400_commits_;
  }
  attitude::Feedback attitude_snapshot(const Snapshot& s) const{
    auto a=s.attitude;a.status_fresh=s.status_fresh&&s.arming_state_known;a.land_fresh=s.land_fresh;a.local_fresh=s.local_fresh;
    a.status_stamp=s.status_stamp;a.land_stamp=s.land_stamp;a.stationary_seconds=s.stationary_seconds;
    a.armed=s.feedback.armed;a.landed=s.feedback.landed;a.at_rest=s.at_rest;
    a.offboard=s.feedback.offboard;a.failsafe=s.feedback.failsafe;a.exclusive=s.feedback.exclusive;
    a.time_regressed=s.time_regressed;a.calibration=s.calibration;a.hil=s.hil;a.preflight=s.feedback.preflight;
    a.battery_ready=s.battery_fresh&&s.battery_connected&&s.power_valid&&s.battery_warning==0&&std::isfinite(a.battery_voltage)&&a.battery_voltage>0;
    a.manual_ready=s.manual_fresh&&!s.manual_lost;
    a.stabilized_selectable=s.stabilized_selectable;a.stabilized_selected=s.nav_state==15;
    a.auto_land_selected=s.nav_state==18;
    return a;
  }
  static std::size_t index(unsigned cmd){return cmd==176?0:cmd==400?1:2;}
  std::string land_handshake_failure(const Snapshot& s,double now){
    auto& p=pending_[2];
    if(!p.sent||(s.status_fresh&&s.feedback.landing)||(s.status_fresh&&s.land_fresh&&!s.feedback.armed&&s.feedback.landed))return {};
    if(p.rejected)return "LAND_REJECTED_RELEASED_TO_PX4_OPERATOR";
    if(now-p.sent_at>=5.){
      p.timed_out=!p.accepted;
      return p.accepted?"LAND_ACK_ACCEPTED_MODE_TIMEOUT_RELEASED_TO_PX4_OPERATOR":"LAND_ACK_TIMEOUT_RELEASED_TO_PX4_OPERATOR";
    }
    return {};
  }
  void track(unsigned cmd,const Snapshot& s,double now,
      attitude::Command intent=attitude::Command::None){auto& p=pending_[index(cmd)];
    p.requested=true;p.sent=!(attitude_sink_commit_required_&&config_.profile=="attitude_altitude");
    p.command400_kind=intent==attitude::Command::Arm?Command400Kind::Arm:
      intent==attitude::Command::Disarm?Command400Kind::Disarm:Command400Kind::None;
    p.stage=p.sent?CommandStage::CommittedAwaitAck:CommandStage::Planned;
    p.published=false;p.sent_at=now;p.baseline=std::max(p.baseline,s.status_stamp);
    if(cmd==400&&config_.profile=="attitude_altitude"){
      p.ack_ambiguous=command400_commits_>0;
      if(p.sent){p.status_at_wire=s.status_stamp;p.land_at_wire=s.land_stamp;record400_commit(p);}
    }
    if(p.sent&&intent!=attitude::Command::None)attitude_core_.note_command_publication(intent,true,s.land_stamp);
  }
  Reply refuse(const std::string& why){message_=why;return {false,why};}
  std::string hardware_gate(const Snapshot& s) const {
    if(!s.feedback.exclusive)return "SOURCE_OR_CONTROL_CONFLICT";
    if(s.calibration||s.hil)return "CALIBRATION_OR_HIL_ACTIVE";
    if(config_.profile!="sim"){
      if(!s.battery_fresh||!s.battery_connected||!s.power_valid||s.battery_warning!=0)return "BATTERY_NOT_READY";
      if(!s.manual_fresh||s.manual_lost)return "MANUAL_TAKEOVER_SIGNAL_REQUIRED";
    }
    return {};
  }
  Config config_;v::Core core_;attitude::Core attitude_core_;std::unique_ptr<g::Core> ground_;
  double last_ping_{-1},velocity_until_{-1},ground_origin_{},recover_started_{};
  bool recover_land_{},maintenance_{},shutdown_latched_{};
  bool last_attitude_output_active_{},attitude_sink_commit_required_{};
  int recover_nav_{};
  std::array<Pending,3> pending_{};
  unsigned command400_commits_{};
  std::vector<Pending> command400_history_;
  std::string message_{"IDLE_NO_AUTOMATIC_ARM"};
};
} // namespace fc01
