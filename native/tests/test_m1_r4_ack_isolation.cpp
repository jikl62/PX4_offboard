// R4 positive safety properties. Real Runtime/Core/final boundary; offline sinks only.
// Mutations caught: accepting reused-400 ACKs, failure-led retries, ACK-only success,
// duplicate final Disarm, stale status/land completion, and recovery heartbeat loss.
#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <iostream>
#include <string>
#include <type_traits>
namespace f=fc01;namespace a=f::attitude;
template<class T,class=void>struct HasArmingTrust:std::false_type{};
template<class T>struct HasArmingTrust<T,std::void_t<decltype(std::declval<T&>().arming_state_known)>>:std::true_type{};
template<class T>static void raw_arming_state(T& s,int raw){
  s.feedback.armed=raw==2;
  if constexpr(HasArmingTrust<T>::value)s.arming_state_known=raw==1||raw==2;
}
struct Mode {std::uint64_t timestamp{};bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};};
struct Target {std::uint64_t timestamp{};float yaw_sp_move_rate{};std::array<float,4> q_d{};std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};};
static f::Config config(){auto c=authority_test::config();c.attitude.autonomous_failsafe_confirmed=true;return c;}
struct Harness {
  f::Runtime r{config()};f::Snapshot s{authority_test::sample()};double t{};
  int arms{},disarms{},targets{},modes{};bool connected{true};
  Harness(){authority_test::ready(r,s,t,"AUTHORITY_AUTONOMOUS");r.require_attitude_sink_commit();}
  a::Output tick(bool status=true,bool land=true){
    t+=.05;if(status)s.status_stamp+=1000;if(land)s.land_stamp+=1000;
    if(connected)r.request("PING",{},s,t);
    auto o=r.step(s,t).attitude;
    if(o.publish){auto c=r.publication_context(s,t,connected,true);
      const auto d=a::publish_attitude_at_boundary<Mode,Target>(c,o,10000000+std::uint64_t(t*1e6),
        [&](const Target&){++targets;},[&](const Mode&){++modes;});
      assert(d.allowed&&targets==modes);
    }
    if(o.command==a::Command::Arm||o.command==a::Command::Disarm){
      auto c=r.publication_context(s,t,connected,true);
      auto d=o.command==a::Command::Arm?a::publish_arm_at_boundary(c,[&]{++arms;}):
        a::publish_disarm_at_boundary(c,[&]{++disarms;});
      assert(d.allowed);
      r.confirm_attitude_command_publication(o.command,true,10000000+std::uint64_t(t*1e6),s,t);
    }else if(o.command==a::Command::ReleaseOffboard||o.command==a::Command::Offboard){
      if(o.command==a::Command::ReleaseOffboard)
        assert(a::publish_release_at_boundary(r.publication_context(s,t,connected,true),[]{}).allowed);
      r.confirm_attitude_command_publication(o.command,true,10000000+std::uint64_t(t*1e6),s,t);
    }
    return o;
  }
  void ack(int result,int offset=10,int system=1,int component=191){
    const auto stamp=std::max(r.pending(400).baseline,r.pending(400).last_ack)+offset;
    r.ack({stamp,400,result,system,component,false},t+.01);
  }
  void reach_disarm(int arm_result=-1,bool state_first=false){
    assert(r.request("TAKEOFF",{1},s,t).accepted);assert(tick().command==a::Command::Arm);
    assert(arms==1&&r.pending(400).published);
    assert(r.request("LAND",{},s,t).accepted);
    if(!state_first&&arm_result>=0)ack(arm_result);
    s.feedback.armed=true;assert(tick().publish);
    if(state_first&&arm_result>=0)ack(arm_result);
    for(int i=0;i<240&&r.attitude_core().phase()!=a::Phase::RequestDisarm;++i){assert(tick().publish);}
    assert(disarms==1&&r.pending(400).published&&r.pending(400).command400_kind==f::Command400Kind::Disarm);
  }
  void hold(int count=140){
    const int n=targets;
    for(int i=0;i<count;++i){auto o=tick();assert(o.publish&&o.command!=a::Command::Disarm);}
    assert(targets==n+count&&disarms==1&&arms==1&&s.feedback.armed&&s.feedback.offboard&&s.nav_state==14);
  }
};
static void c11(){
  Harness h;h.reach_disarm();h.ack(0);
  std::cout<<"R4_C11 accepted="<<h.r.pending(400).accepted<<" stage="<<f::command_stage_name(h.r.pending(400).stage)<<std::endl;
  assert(!h.r.pending(400).accepted&&!h.r.pending(400).rejected);
  h.hold(3);h.s.feedback.armed=false;auto o=h.tick();
  assert(!o.publish&&h.r.attitude_core().phase()==a::Phase::SafeDisarmed);
  assert(h.r.pending(400).state_verified&&!h.r.pending(400).accepted);
  assert(h.r.pending(400).stage!=f::CommandStage::VerifiedTerminal);
  std::cout<<"R4_C11 state_only_terminal=1 false_ACK_verification=0 arm_sinks="<<h.arms<<" disarm_sinks="<<h.disarms<<"\n";
}
static void c12(int result){
  for(int prior:{-1,0}){Harness h;h.reach_disarm(prior);h.ack(result);
    std::cout<<"R4_C12 prior_arm_result="<<prior<<" observed_result="<<result<<" rejected="<<h.r.pending(400).rejected<<std::endl;
    assert(!h.r.pending(400).accepted&&!h.r.pending(400).rejected);
    const auto observed=h.r.pending(400).ack_observations;
    const auto stamp=h.r.pending(400).ack_stamp;
    h.r.ack({stamp,400,result,1,191,false},h.t+.02);
    assert(h.r.pending(400).ack_observations==observed); // identical duplicate
    // Same old result with a new timestamp, plus repeated LAND: no retransmission.
    h.ack(result);assert(h.r.request("LAND",{},h.s,h.t).accepted);h.hold();
    assert(h.r.pending(400).result==result);
    // Direct attempted replay must also fail at the final policy boundary.
    int repeat=0;auto d=a::publish_disarm_at_boundary(h.r.publication_context(h.s,h.t,true,true),[&]{++repeat;});
    assert(!d.allowed&&repeat==0);
    h.s.feedback.armed=false;auto o=h.tick();
    assert(!o.publish&&h.r.attitude_core().phase()==a::Phase::SafeDisarmed);
    assert(h.r.pending(400).state_verified&&h.r.pending(400).stage!=f::CommandStage::VerifiedTerminal);
    std::cout<<"R4_C12 result="<<result<<" duplicate_sink_calls="<<repeat<<" disarm_sinks="<<h.disarms<<" healthy_heartbeat_continuous=1\n";
  }
}
static void missing_ack_and_stamps(){
  Harness h;h.reach_disarm();h.s.feedback.armed=false;
  auto o=h.tick(true,false);assert(o.publish&&!h.r.pending(400).state_verified);
  o=h.tick(false,false);assert(o.publish&&!h.r.pending(400).state_verified);
  // New status alone cannot use pre-submission/frozen ground evidence.
  o=h.tick();assert(!o.publish&&h.r.attitude_core().phase()==a::Phase::SafeDisarmed);
  assert(h.r.pending(400).state_verified&&!h.r.pending(400).accepted);
  Harness frozen_status;frozen_status.reach_disarm();frozen_status.s.feedback.armed=false;
  assert(frozen_status.tick(false,true).publish&&!frozen_status.r.pending(400).state_verified);
  assert(!frozen_status.tick().publish&&frozen_status.r.pending(400).state_verified);
  std::cout<<"R4_ACK_MISSING new_disarmed_ground_state_completes=1 frozen_land_blocks=1 disarm_sinks="<<h.disarms<<"\n";
}
static void bounce_and_cancel(const std::string& action){
  Harness h;h.reach_disarm(0,true);h.s.feedback.landed=false;h.s.at_rest=false;
  assert(h.tick().publish&&h.r.attitude_core().phase()==a::Phase::Descent);
  h.ack(4);
  if(action=="disconnect"){h.r.disconnect(h.s,h.t);h.connected=false;}
  else if(action=="shutdown")h.r.begin_shutdown(h.s,h.t);
  else if(action=="takeover")assert(h.r.request("AUTHORITY_TAKEOVER",{},h.s,h.t).accepted);
  h.s.feedback.landed=true;h.s.at_rest=true;h.hold();
  h.s.feedback.armed=false;auto o=h.tick();assert(!o.publish);
  assert(h.r.attitude_core().phase()==a::Phase::SafeDisarmed&&h.disarms==1);
  std::cout<<"R4_BOUNCE_"<<action<<" failure_ACK_no_retry=1 heartbeat_continuous=1\n";
}
static void first_arm_results(int result){
  Harness h;assert(h.r.request("TAKEOFF",{1},h.s,h.t).accepted);h.tick();h.ack(result);
  // Any ACK alone leaves the physical armed state unconfirmed.
  auto o=h.tick();assert(o.publish&&!h.r.attitude_core().arm_confirmed()&&h.arms==1);
  if(result==0)assert(h.r.pending(400).accepted);
  if(result>=1&&result<=6&&result!=5)assert(h.r.pending(400).rejected);
  if(result==5)assert(!h.r.pending(400).accepted&&!h.r.pending(400).rejected);
  // Failure contradicting NEW Armed telemetry must retain recovery output.
  h.s.feedback.armed=true;o=h.tick();assert(o.publish&&h.r.attitude_core().arm_confirmed());
  if(result>=1&&result<=6&&result!=5)
    assert(std::string(f::command_stage_name(h.r.pending(400).stage))=="PX4_STATE_EVIDENCED_ACK_CONTRADICTORY");
  assert(h.arms==1);
  std::cout<<"R4_FIRST_ARM result="<<result<<" ACK_only_no_completion=1 new_armed_heartbeat=1\n";
}
static void rejected_arm_ground_release(int result){
  Harness h;assert(h.r.request("TAKEOFF",{1},h.s,h.t).accepted);h.tick();h.ack(result);
  assert(h.r.request("LAND",{},h.s,h.t).accepted);
  assert(h.tick().command==a::Command::ReleaseOffboard);
  auto o=h.tick();
  std::cout<<"R4_ARM_REFUSAL_RELEASE result="<<result<<" phase="<<h.r.phase_name()<<" heartbeat="<<o.publish<<std::endl;
  assert(o.publish&&h.r.attitude_core().phase()==a::Phase::ReleaseOffboard);
  h.r.ack({h.r.pending(176).baseline+10,176,0,1,191,false},h.t+.01);
  h.s.feedback.offboard=false;h.s.nav_state=15;h.tick();
  assert(h.r.attitude_core().phase()==a::Phase::Idle&&h.disarms==0&&h.arms==1);
  assert(h.r.request("AUTHORITY_RESET",{},h.s,h.t).accepted);
  std::cout<<"R4_ARM_REFUSAL_RELEASE completed_ground_mode_and_reset=1 no_Disarm=1\n";
}
static void sink_and_target(){
  Harness h;assert(h.r.request("TAKEOFF",{1},h.s,h.t).accepted);
  h.t+=.05;h.s.status_stamp+=1000;h.s.land_stamp+=1000;
  auto o=h.r.step(h.s,h.t).attitude;assert(o.command==a::Command::Arm);
  h.ack(0);assert(!h.r.pending(400).accepted);
  h.r.confirm_attitude_command_publication(o.command,false,0,h.s,h.t);h.ack(0);
  assert(!h.r.pending(400).sent&&!h.r.pending(400).accepted&&!h.r.pending(400).published);
  Harness live;live.reach_disarm();live.ack(4,10,9,191);live.ack(0,10,1,190);
  assert(live.r.pending(400).result==-1);live.hold(4);
  std::cout<<"R4_SINK_REJECT_AND_OTHER_TARGET ACK_cannot_authorize=1\n";
}
static void safe_reset_retains_protocol_uncertainty(){
  Harness h;h.reach_disarm(0);h.s.feedback.armed=false;
  assert(!h.tick().publish&&h.r.attitude_core().phase()==a::Phase::SafeDisarmed);
  assert(h.tick().command==a::Command::ReleaseOffboard);
  h.r.ack({h.r.pending(176).baseline+10,176,0,1,191,false},h.t+.01);
  h.s.feedback.offboard=false;h.s.nav_state=15;h.tick();
  assert(h.r.attitude_core().phase()==a::Phase::Idle);
  const auto reset=h.r.request("AUTHORITY_RESET",{},h.s,h.t);
  std::cout<<"R4_SAFE_GROUND_RESET accepted="<<reset.accepted<<" reason="<<reset.message<<std::endl;
  assert(reset.accepted);
  assert(h.r.pending(400).state_verified&&h.r.pending(400).ack_ambiguous);
  assert(h.r.command400_history().size()==1);
  assert(h.r.request("AUTHORITY_AUTONOMOUS",{},h.s,h.t).accepted);
  assert(h.r.request("ATTITUDE_READY",{},h.s,h.t).accepted);
  for(int i=0;i<40&&h.r.attitude_core().phase()!=a::Phase::RequestOffboard;++i)h.tick();
  assert(h.r.attitude_core().phase()==a::Phase::RequestOffboard);
  h.r.ack({h.r.pending(176).baseline+10,176,0,1,191,false},h.t+.01);
  h.s.feedback.offboard=true;h.s.nav_state=14;h.tick();
  assert(h.r.attitude_core().phase()==a::Phase::OffboardReady);
  const auto takeoff=h.r.request("TAKEOFF",{1},h.s,h.t);
  assert(!takeoff.accepted&&takeoff.message=="COMMAND_400_REUSE_REQUIRES_SAFE_GROUND_RESTART");
  assert(h.tick().command!=a::Command::Arm&&h.arms==1);
  std::cout<<"R4_RESET history_retained=1 new_arm_blocked_until_external_quiescence=1\n";
}
static void state_first_matrix(){
  for(int result:{-1,0,1,2,3,4,5,6}){
    Harness h;h.reach_disarm(result,true);
    assert(h.r.pending(400).ack_ambiguous&&h.r.command400_history().size()==1);
    const auto& old=h.r.command400_history().front();
    assert(old.command400_kind==f::Command400Kind::Arm&&old.state_verified&&old.generation==1);
    h.ack(5);h.hold(2);h.ack(0);h.hold(2);
    h.s.feedback.armed=false;assert(!h.tick().publish);
    const auto observations=h.r.pending(400).ack_observations;
    h.ack(4); // ACK after state-only terminal never relabels it or restarts recovery.
    assert(h.r.pending(400).ack_observations==observations+1&&!h.r.pending(400).rejected);
    assert(h.r.pending(400).stage==f::CommandStage::StateOnlyTerminal);
    std::cout<<"R4_ARM_STATE_BEFORE_ACK result="<<result<<" history_retained=1 progress_then_success_unattributed=1 late_failure_no_retry=1\n";
  }
}
static void unhealthy_and_airborne_state(){
  for(int fault=0;fault<8;++fault){
    Harness h;h.reach_disarm();h.s.feedback.armed=false;
    if(fault==0)h.s.feedback.landed=false;
    if(fault==1)h.s.status_fresh=false;
    if(fault==2)h.s.land_fresh=false;
    if(fault==3)h.s.feedback.exclusive=false;
    if(fault==4)h.s.attitude.kill=true;
    if(fault==5)h.s.time_regressed=true;
    if(fault==6)h.s.attitude.z-=.4;
    if(fault==7){h.s.status_stamp-=2000;h.s.land_stamp-=2000;}
    auto o=h.tick(fault!=7,fault!=7);
    assert(!h.r.pending(400).state_verified);
    assert(h.r.attitude_core().phase()!=a::Phase::SafeDisarmed);
    assert(o.command!=a::Command::Disarm&&h.disarms==1);
    std::cout<<"R4_UNTRUSTED_DISARMED_STATE fault="<<fault<<" no_safe_terminal=1 no_repeat_400=1\n";
  }
}
static void unknown_arming_enum(){
  for(int raw:{0,3,255}){
    Harness h;h.reach_disarm();raw_arming_state(h.s,raw);
    auto o=h.tick();
    std::cout<<"R4_RAW_ARMING_STATE raw="<<raw<<" state_verified="<<h.r.pending(400).state_verified<<" phase="<<h.r.phase_name()<<std::endl;
    assert(!h.r.pending(400).state_verified&&h.r.attitude_core().phase()!=a::Phase::SafeDisarmed);
    assert(!o.publish&&h.disarms==1);
  }
  Harness known;known.reach_disarm();raw_arming_state(known.s,1);
  assert(!known.tick().publish&&known.r.pending(400).state_verified);
  std::cout<<"R4_RAW_ARMING_STATE known_DISARMED_only_safe_terminal=1\n";
}
static void core_reference_faults_cannot_verify_terminal(){
  for(int fault=0;fault<4;++fault){
    Harness h;h.reach_disarm();h.s.feedback.armed=false;
    if(fault==0)++h.s.attitude.z_reset;
    if(fault==1)++h.s.attitude.vz_reset;
    if(fault==2)++h.s.attitude.quat_reset;
    if(fault==3)h.t+=.4; // Core stall is beyond the boundary candidate's fields.
    auto o=h.tick();
    std::cout<<"R4_CORE_REFERENCE_FAULT fault="<<fault<<" phase="<<h.r.phase_name()<<" state_verified="<<h.r.pending(400).state_verified<<std::endl;
    assert(!h.r.pending(400).state_verified&&!o.publish&&h.r.attitude_core().phase()==a::Phase::Safety);
    assert(h.r.pending(400).stage!=f::CommandStage::StateOnlyTerminal&&h.disarms==1);
  }
}
int main(int argc,char** argv){
  const std::string scenario=argc>1?argv[1]:"all";
  if(scenario=="all"||scenario=="c11")c11();
  if(scenario=="all"||scenario=="c12")for(int result:{0,1,2,3,4,5,6,255})c12(result);
  if(scenario=="c12failed")c12(4);
  if(scenario=="all"||scenario=="missing")missing_ack_and_stamps();
  if(scenario=="all"||scenario=="bounce")for(const char* action:{"land","disconnect","shutdown","takeover"})bounce_and_cancel(action);
  if(scenario=="all"||scenario=="first")for(int result:{0,1,2,3,4,5,6})first_arm_results(result);
  if(scenario=="all"||scenario=="refusal")for(int result:{1,2,3,4,6})rejected_arm_ground_release(result);
  if(scenario=="all"||scenario=="sink")sink_and_target();
  if(scenario=="all"||scenario=="reset")safe_reset_retains_protocol_uncertainty();
  if(scenario=="all"||scenario=="statefirst")state_first_matrix();
  if(scenario=="all"||scenario=="unhealthy")unhealthy_and_airborne_state();
  if(scenario=="all"||scenario=="rawstate")unknown_arming_enum();
  if(scenario=="all"||scenario=="reference")core_reference_faults_cannot_verify_terminal();
  std::cout<<"R4_POSITIVE_SAFETY_ASSERTIONS PASS ("<<scenario<<")\n";
}
