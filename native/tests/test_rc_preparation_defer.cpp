#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace authority_test;
static void need(bool ok,const char* reason){if(!ok)throw std::runtime_error(reason);}
static constexpr const char* expired="RC_PREPARATION_LOCAL_ATTITUDE_DATA_EXPIRED";
static f::Config rc_config(){
  auto c=config();auto& p=c.attitude;
  p.rc_navigation_enabled=true;p.arm_enabled=false;p.rc_takeover_confirmed=true;
  p.max_flight_seconds=30;p.max_tilt_deg=10;c.validate();return c;
}
struct Mode {
  std::uint64_t timestamp{};
  bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};
};
struct Target {
  std::uint64_t timestamp{};float yaw_sp_move_rate{};
  std::array<float,4> q_d{};std::array<float,3> thrust_body{};
  bool reset_integral{},fw_control_yaw_wheel{};
};
struct Preparation {
  f::Config c{rc_config()};f::Runtime r{c};f::Snapshot s{sample()};
  double now{};f::Output out;unsigned targets{},heartbeats{};
  Preparation(){
    r.require_attitude_sink_commit();s.nav_state=15;s.attitude.source_clock_validated=true;
    s.attitude.rc_link_ready=s.attitude.rc_switches_ready=true;
    s.attitude.rc_source_clock_valid=s.attitude.rc_switch_clock_valid=true;
    s.attitude.rc_source_selected=s.attitude.rc_axes_safe=s.attitude.rc_aux_switches_safe=true;
    s.attitude.rc_mode_slot=1;s.attitude.rc_arm_switch=s.attitude.rc_kill_switch=3;
    s.attitude.rc_switch_sample_stamp=1000;
  }
  void tick(bool commit=true){
    now+=.05;s.status_stamp+=50000;s.land_stamp+=50000;
    s.attitude.local_stamp+=50000;s.attitude.vertical_sample_stamp+=50000;
    s.attitude.rc_switch_sample_stamp+=50000;s.attitude.source_host_stamp=s.status_stamp;
    out=r.step(s,now);
    need(out.attitude.command==a::Command::None,"preparation cannot synthesize commands");
    need(!a::nonzero(out.attitude.thrust),"preparation cannot produce thrust");
    if(commit&&out.attitude.publish){
      const auto d=a::publish_attitude_resampled<Mode,Target>(
        [&]{return r.publication_context(s,now,true,true);},out.attitude,s.status_stamp,
        [&](const Target&){++targets;},[&](const Mode&){++heartbeats;});
      need(d.allowed,"healthy actual RC prestream boundary must allow zero output");
      r.confirm_rc_output(true,now);
    }
  }
  void ready(){
    for(int i=0;i<40;++i)tick();
    need(r.attitude_core().phase()==a::Phase::RcWaitArm,"committed prestream must reach WaitArm");
  }
  f::Snapshot age_expired()const{
    auto value=s;value.attitude.source_clock_valid=false;
    value.local_fresh=value.attitude.attitude_fresh=false;return value;
  }
};
static void refuse_before_sinks_restarts_the_complete_zero_prestream(){
  for(bool wait_arm:{false,true}){
    Preparation q;if(wait_arm)q.ready();else q.tick();
    // A real unsolicited ACK watermark is wire history, not an Arm request.
    q.r.ack({q.s.status_stamp+9,176,0,1,191,false},q.now);
    const auto watermark=q.r.pending(176).baseline;
    need(watermark>0&&!q.r.pending(176).requested,"fixture retains unsolicited ACK history");
    q.tick(false);const auto before_targets=q.targets,before_heartbeats=q.heartbeats;
    auto stale=q.age_expired();unsigned checks{};
    const auto d=a::publish_attitude_resampled<Mode,Target>(
      [&]{++checks;return q.r.publication_context(stale,q.now,true,true);},q.out.attitude,q.s.status_stamp,
      [&](const Target&){++q.targets;},[&](const Mode&){++q.heartbeats;});
    need(!d.allowed&&d.reason==a::BoundaryReason::SourceClock&&checks==1,
      "source expiry must refuse the real boundary before either sink");
    need(q.targets==before_targets&&q.heartbeats==before_heartbeats,"refusal emits neither packet");
    need(q.r.defer_rc_preparation(stale,q.now,expired),"never-armed ground preparation may defer expiry");
    need(q.r.attitude_core().phase()==a::Phase::Idle&&q.r.attitude_core().reason()==expired,
      "defer returns to an explicit waiting state");
    need(q.r.attitude_core().rc_prestream_commits()==0&&!q.r.attitude_core().reference_ready()&&
      !q.r.attitude_core().restart_required(),"defer discards every old stream commit and ground anchor");
    need(q.r.pending(176).baseline==watermark&&!q.r.pending(176).requested&&
      q.r.command400_history().empty(),"defer preserves wire history and adds no transaction");
    q.s=stale;q.tick();need(!q.out.attitude.publish,"stale feedback never resumes output");
    q.s.attitude.source_clock_valid=q.s.local_fresh=q.s.attitude.attitude_fresh=true;
    q.tick();
    need(q.r.attitude_core().phase()==a::Phase::RcPrestream&&q.r.attitude_core().rc_prestream_commits()==1,
      "fresh feedback begins a new stream rather than reusing previous READY");
    for(int i=0;i<10;++i)q.tick();
    need(q.r.attitude_core().phase()==a::Phase::RcPrestream,"partial stream duration is insufficient");
    q.ready();
    need(!q.r.attitude_core().flight_started()&&!q.r.pending(400).requested&&!q.r.pending(176).requested,
      "complete recovery waits for new manual consent and cannot automatically arm");
  }
}
static void latest_safe_controls_cannot_erase_a_historical_arm_edge(){
  Preparation q;q.ready();
  q.s.attitude.rc_arm_switch=1;q.s.attitude.source_clock_valid=false;
  q.tick();need(!q.out.attitude.publish,"stale arm-intent tick must emit no output");
  // Before the new admission predicate, this health reset retained no arm
  // edge/time: recording intent before health() is necessary for this case.
  q.s.attitude.rc_arm_switch=3;q.s.attitude.source_clock_valid=true;q.ready();
  need(!q.r.defer_rc_preparation(q.age_expired(),q.now,expired),
    "latest arm-OFF cannot authorize a no-arm retry after historical arm-ON");
  Preparation matched;matched.ready();matched.s.attitude.rc_arm_switch=1;matched.tick();
  matched.s.attitude.rc_arm_switch=3;
  need(!matched.r.defer_rc_preparation(matched.age_expired(),matched.now,expired),
    "a captured arm edge cannot be reset while waiting for complementary evidence");
}
static void independent_guards_reject_defer_without_mutating_preparation(){
  Preparation q;q.ready();
  const auto phase=q.r.attitude_core().phase();
  const auto commits=q.r.attitude_core().rc_prestream_commits();
  for(int invalid=0;invalid<34;++invalid){
    auto s=q.age_expired();double now=q.now;
    switch(invalid){
      case 0:s.feedback.armed=true;break;
      case 1:s.feedback.offboard=true;break;
      case 2:s.feedback.landed=false;break;
      case 3:s.at_rest=false;break;
      case 4:s.nav_state=14;break;
      case 5:s.attitude.rc_arm_switch=1;break;
      case 6:s.attitude.rc_mode_slot=q.c.attitude.rc_navigation_mode_slot;break;
      case 7:s.attitude.z_reset++;break;
      case 8:s.attitude.vz_reset++;break;
      case 9:s.attitude.quat_reset++;break;
      case 10:s.publication_fault_latched=true;break;
      case 11:s.attitude_output_active=true;break;
      case 12:s.offboard_heartbeat_active=true;break;
      case 13:s.time_regressed=true;break;
      case 14:s.attitude.kill=true;break;
      case 15:s.attitude.operator_takeover=true;break;
      case 16:s.feedback.exclusive=false;break;
      case 17:s.status_fresh=false;break;
      case 18:s.land_fresh=false;break;
      case 19:s.attitude.failsafe_fresh=false;break;
      case 20:s.feedback.failsafe=true;break;
      case 21:s.attitude.rc_source_clock_valid=false;break;
      case 22:s.attitude.rc_switch_clock_valid=false;break;
      case 23:s.attitude.source_clock_validated=false;break;
      case 24:s.attitude.source_host_stamp=0;break;
      case 25:s.battery_fresh=false;break;
      case 26:s.feedback.preflight=false;break;
      case 27:s.arming_state_known=false;break;
      case 28:s.stationary_seconds=0;break;
      case 29:s.attitude.z+=q.c.attitude.height_tolerance+.01;break;
      case 30:s.attitude.q={0,0,0,0};break;
      case 31:s.attitude.vz=std::numeric_limits<double>::quiet_NaN();break;
      case 32:now=q.now-.01;break;
      case 33:now=std::numeric_limits<double>::quiet_NaN();break;
    }
    need(!q.r.defer_rc_preparation(s,now,expired),"unsafe or uncertain feedback must reject defer");
    need(q.r.attitude_core().phase()==phase&&q.r.attitude_core().rc_prestream_commits()==commits,
      "rejected defer must preserve the original state and evidence");
  }
}
static void any_command_history_and_a_partial_publication_block_recovery(){
  for(unsigned command:{176U,400U,21U}){
    Preparation q;q.ready();
    // Inject historical command evidence into this real Runtime to cover all
    // three tracker slots; production exposes no transaction mutator.
    auto& p=const_cast<f::Pending&>(q.r.pending(command));
    p.requested=true;p.state_verified=true;p.stage=f::CommandStage::VerifiedTerminal;
    need(!q.r.defer_rc_preparation(q.age_expired(),q.now,expired),
      "even terminal command history is outside the no-command preparation retry");
    need(p.requested&&p.state_verified,"refusal preserves historical transaction evidence");
  }
  Preparation q;q.ready();q.tick(false);unsigned checks{};bool threw{};
  try{
    a::publish_attitude_resampled<Mode,Target>([&]{
      ++checks;return q.r.publication_context(checks<3?q.s:q.age_expired(),q.now,true,true);
    },q.out.attitude,q.s.status_stamp,[&](const Target&){++q.targets;},[&](const Mode&){++q.heartbeats;});
  }catch(const std::runtime_error& e){
    threw=std::string(e.what())=="HEARTBEAT_FINAL_CONTEXT_CHANGED_DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE";
  }
  need(threw&&checks==3,"target-committed expiry follows the real partial-publication exception path");
  q.r.abort_attitude("PARTIAL_OR_FAILED_SETPOINT_PUBLICATION");
  need(!q.r.defer_rc_preparation(q.age_expired(),q.now,expired)&&q.r.attitude_core().restart_required(),
    "partial publication remains a serious latched fault rather than a preparation retry");
}
int main(){
  refuse_before_sinks_restarts_the_complete_zero_prestream();
  latest_safe_controls_cannot_erase_a_historical_arm_edge();
  independent_guards_reject_defer_without_mutating_preparation();
  any_command_history_and_a_partial_publication_block_recovery();
  std::cout<<"RC_PREPARATION_DEFER_PASS\n";
}
