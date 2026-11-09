#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>
using namespace authority_test;
static void need(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
static f::Config rc_config(std::int64_t slot=6){
  auto c=config();auto& p=c.attitude;
  p.rc_navigation_enabled=true;p.arm_enabled=false;p.rc_takeover_confirmed=true;
  p.rc_navigation_mode_slot=slot;p.rc_target_height=.5;
  p.max_flight_seconds=30;p.max_tilt_deg=10;p.land_sample_max_gap=1.25;
  p.height_tolerance=.15;p.vertical.max_climb=.3;
  c.validate();return c;
}
struct RcMode {
  std::uint64_t timestamp{};
  bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};
};
struct RcTarget {
  std::uint64_t timestamp{};float yaw_sp_move_rate{};
  std::array<float,4> q_d{};std::array<float,3> thrust_body{};
  bool reset_integral{},fw_control_yaw_wheel{};
};
struct Flight {
  f::Config c;f::Runtime r;f::Snapshot s{sample()};
  double t{};f::Output o;unsigned published{},arm_commands{},mode_commands{},target_commits{},heartbeat_commits{};
  explicit Flight(std::int64_t slot=6):Flight(rc_config(slot)){}
  explicit Flight(f::Config candidate):c(std::move(candidate)),r(c){
    s.nav_state=15;s.attitude.source_clock_validated=true;
    s.attitude.rc_link_ready=s.attitude.rc_switches_ready=true;
    s.attitude.rc_source_clock_valid=s.attitude.rc_switch_clock_valid=true;
    s.attitude.rc_source_selected=s.attitude.rc_axes_safe=s.attitude.rc_aux_switches_safe=true;
    s.attitude.rc_mode_slot=c.attitude.rc_navigation_mode_slot==1?6:1;s.attitude.rc_arm_switch=s.attitude.rc_kill_switch=3;
    s.attitude.rc_switch_sample_stamp=1000;
    s.attitude.local_stamp=s.attitude.vertical_sample_stamp=1000;
    s.attitude.z_deriv=s.attitude.vz=0;
    r.require_attitude_sink_commit();
  }
  void tick(bool commit=true,double dt=.05){
    t+=dt;s.status_stamp+=50000;s.land_stamp+=50000;
    s.attitude.local_stamp+=50000;s.attitude.vertical_sample_stamp+=50000;
    s.attitude.rc_switch_sample_stamp+=50000;
    s.attitude.source_host_stamp=s.status_stamp;
    o=r.step(s,t);
    if(o.attitude.command==a::Command::Arm)++arm_commands;
    if(o.attitude.command==a::Command::Offboard)++mode_commands;
    if(o.attitude.publish){
      const auto context=r.publication_context(s,t,true,true);
      const auto decision=commit?a::publish_attitude_at_boundary<RcMode,RcTarget>(context,o.attitude,s.status_stamp,
        [&](const RcTarget&){++target_commits;},[&](const RcMode&){++heartbeat_commits;}):
        a::attitude_publication_policy(context,o.attitude);
      need(decision.allowed,a::name(decision.reason));
      if(commit){++published;r.confirm_rc_output(true,t);}
    }
  }
  void warm(bool commit=true){for(int i=0;i<75;++i)tick(commit);}
  void manual_arm(bool status_first=false){
    if(status_first){s.feedback.armed=true;s.armed_time=10;s.latest_arming_reason=2;s.at_rest=false;tick();}
    s.attitude.rc_arm_switch=1;++s.attitude.operator_switch_generation;tick();
    s.feedback.armed=true;s.armed_time=10;s.latest_arming_reason=2;s.at_rest=false;tick();
  }
  void navigation(bool actual=true){
    s.attitude.rc_mode_slot=c.attitude.rc_navigation_mode_slot;++s.attitude.operator_switch_generation;tick();
    if(actual){s.feedback.offboard=true;s.nav_state=14;tick();}
  }
  void join(){warm();manual_arm();navigation();need(r.attitude_core().phase()==a::Phase::ArmedGround,"authorized RC path joins ArmedGround");}
};
static void valid_path_without_automatic_commands(){
  Flight q;q.warm();need(q.r.attitude_core().phase()==a::Phase::RcWaitArm,"prestream becomes ready");
  need(q.published>=20,"actual prestream commits");
  need(!q.r.attitude_core().flight_started(),"prestream isn't a flight");
  q.manual_arm();need(!q.r.attitude_core().flight_started(),"manual arm alone never takes off");
  q.navigation(false);need(!q.r.attitude_core().flight_started(),"RC navigation alone needs PX4 mode proof");
  q.s.feedback.offboard=true;q.s.nav_state=14;q.tick();
  need(q.r.attitude_core().phase()==a::Phase::ArmedGround,"new actual Offboard joins flight");
  need(!a::nonzero(q.o.attitude.thrust),"join starts with zero thrust");
  for(int i=0;i<16;++i)q.tick();
  need(q.r.attitude_core().phase()==a::Phase::ThrustRamp,"existing ground dwell leads to ramp");
  need(a::nonzero(q.o.attitude.thrust),"authorized route eventually produces ramp");
  need(q.arm_commands==0&&q.mode_commands==0,"RC route never asks PX4 to arm or enter Offboard");
  need(!q.r.pending(400).requested&&!q.r.pending(176).requested,"no fabricated command transactions");
}
static void status_switch_reordering(){
  Flight q;q.warm();q.manual_arm(true);q.navigation();
  need(q.r.attitude_core().phase()==a::Phase::ArmedGround,"status-first RC arm arrival is supported");
}
static void delayed_arm_evidence_keeps_continuous_zero_commits(){
  for(bool status_first:{false,true}){
    Flight q;q.warm();const auto before=q.published;
    const auto targets_before=q.target_commits,heartbeats_before=q.heartbeat_commits;
    if(status_first){q.s.feedback.armed=true;q.s.armed_time=10;q.s.latest_arming_reason=2;q.s.at_rest=false;}
    else {q.s.attitude.rc_arm_switch=1;++q.s.attitude.operator_switch_generation;}
    for(int i=0;i<10;++i){
      q.tick();
      need(q.o.attitude.publish,"500ms unmatched arm evidence retains every zero stream tick");
      need(!a::nonzero(q.o.attitude.thrust),"pending arm evidence cannot produce thrust");
      need(!q.r.attitude_core().flight_started()&&!q.r.attitude_core().rc_manual_arm_proven(),"independent evidence still required");
    }
    need(q.published==before+10,"all 10 delayed evidence ticks commit");
    need(q.target_commits==targets_before+10&&q.heartbeat_commits==heartbeats_before+10,
      "both actual boundary sinks remain reachable for every delayed evidence tick");
    q.s.attitude.rc_arm_switch=1;++q.s.attitude.operator_switch_generation;
    q.s.feedback.armed=true;q.s.armed_time=10;q.s.latest_arming_reason=2;q.s.at_rest=false;q.tick();
    q.navigation();
    need(q.r.attitude_core().phase()==a::Phase::ArmedGround,"continuous readiness joins without a second prestream dwell");
    need(q.arm_commands==0&&q.mode_commands==0,"reordering never creates pilot commands");
  }
}
static void configured_navigation_slots_require_matching_new_edge(){
  for(std::int64_t slot:{1,6}){
    Flight q(slot);q.join();
    need(!a::nonzero(q.o.attitude.thrust),"configured slot starts ArmedGround at zero");
    Flight wrong(slot);wrong.warm();wrong.manual_arm();
    wrong.s.attitude.rc_mode_slot=slot==1?6:1;wrong.tick();
    wrong.s.feedback.offboard=true;wrong.s.nav_state=14;
    wrong.tick();
    for(int i=0;i<52;++i)wrong.tick();
    need(wrong.r.attitude_core().phase()==a::Phase::Safety&&
      wrong.r.attitude_core().reason()=="RC_NAVIGATION_MODE_SLOT_MISMATCH", "unmatched slot expires with the precise admission mismatch");
    need(!wrong.r.attitude_core().flight_started(),"actual Offboard with wrong RC slot cannot join");
    need(!a::nonzero(wrong.o.attitude.thrust),"wrong RC slot cannot generate thrust");
  }
  for(std::int64_t slot:{-1,0,7,255}){
    bool rejected=false;try{rc_config(slot);}catch(const std::invalid_argument&){rejected=true;}
    need(rejected,"navigation mode slot outside 1 through 6 is rejected");
  }
}
static void missing_arm_evidence_has_an_unrenewable_deadline(){
  for(bool status_first:{false,true}){
    Flight q;q.warm();
    if(status_first){q.s.feedback.armed=true;q.s.armed_time=10;q.s.latest_arming_reason=2;q.s.at_rest=false;}
    else {q.s.attitude.rc_arm_switch=1;++q.s.attitude.operator_switch_generation;}
    q.tick();
    for(int i=0;i<45;++i){q.tick();need(q.o.attitude.publish,"zero stream remains available inside evidence deadline");}
    auto expired=q.r.publication_context(q.s,q.t+.26,true,true);
    need(!a::attitude_publication_policy(expired,q.o.attitude).allowed,"sink rejects expired pending evidence before the next core step");
    for(int i=0;i<10;++i)q.tick();
    need(q.r.attitude_core().phase()==a::Phase::Safety&&
      q.r.attitude_core().reason()=="RC_MANUAL_ARM_EVIDENCE_TIMEOUT","missing arm evidence expires despite fresh repeated source samples");
    need(!q.r.attitude_core().flight_started()&&!q.o.attitude.publish&&
      q.arm_commands==0&&q.mode_commands==0,"timeout cannot launch or synthesize commands");
  }
}
static void pending_permission_rechecks_current_safety_and_exact_zero(){
  Flight q;q.warm();q.s.feedback.armed=true;q.s.armed_time=10;q.s.latest_arming_reason=2;q.s.at_rest=false;q.tick();
  for(int k=0;k<20;++k){
    auto changed=q.s;
    if(k==0)changed.latest_arming_reason=4;
    if(k==1)changed.armed_time=0;
    if(k==2)changed.armed_time=11;
    if(k==3)changed.status_stamp=1;
    if(k==4)changed.attitude.kill=true;
    if(k==5)changed.attitude.rc_axes_safe=false;
    if(k==6)changed.attitude.rc_mode_slot=6;
    if(k==7)changed.feedback.offboard=true;
    if(k==8)changed.feedback.landed=false;
    if(k==9)changed.attitude.z+=.16;
    if(k==10)changed.attitude.vz=.11;
    if(k==11)changed.attitude.z_reset++;
    if(k==12)changed.attitude.source_clock_valid=false;
    if(k==13)changed.attitude.rc_source_selected=false;
    if(k==14)changed.attitude.rc_arm_switch=2;
    if(k==15)changed.attitude.q={std::cos(.11),std::sin(.11),0,0};
    if(k==16)changed.feedback.preflight=false;
    if(k==17)changed.status_fresh=false;
    if(k==18)changed.attitude.rc_aux_switches_safe=false;
    if(k==19)changed.attitude.rc_switch_clock_valid=false;
    unsigned targets=0,heartbeats=0;
    const auto decision=a::publish_attitude_at_boundary<RcMode,RcTarget>(
      q.r.publication_context(changed,q.t+.01,true,true),q.o.attitude,123,
      [&](const RcTarget&){++targets;},[&](const RcMode&){++heartbeats;});
    need(!decision.allowed&&targets==0&&heartbeats==0,"changed pending evidence must veto both final sinks");
  }
  const auto context=q.r.publication_context(q.s,q.t+.01,true,true);
  for(unsigned axis=0;axis<3;++axis){
    auto output=q.o.attitude;output.thrust[axis]=std::numeric_limits<double>::denorm_min();
    const auto denied=a::attitude_publication_policy(context,output);
    need(!denied.allowed&&denied.reason==a::BoundaryReason::StageOneNonzero,"pending permit rejects double nonzero even if float would underflow");
  }
  auto forged=context;forged.rc_arm_evidence_pending=false;
  need(!a::attitude_publication_policy(forged,q.o.attitude).allowed,"unproven armed startup needs its explicit pending permission");
  q.s.latest_arming_reason=4;q.tick();
  need(q.r.attitude_core().phase()==a::Phase::Safety&&q.r.attitude_core().reason()=="RC_ARM_REASON_NOT_RC_SWITCH",
    "wrong armed reason latches before producing output");
}
static void sink_resampling_handles_new_arm_observations_and_vetoes_changes(){
  for(bool status_first:{false,true}){
    Flight q;q.warm();
    if(status_first){q.s.feedback.armed=true;q.s.armed_time=10;q.s.latest_arming_reason=2;q.s.at_rest=false;q.s.status_stamp+=10000;}
    else {q.s.attitude.rc_arm_switch=1;q.s.attitude.rc_switch_sample_stamp+=10000;}
    unsigned targets=0,heartbeats=0;
    const auto context=q.r.publication_context(q.s,q.t+.01,true,true);
    need(context.phase==a::Phase::RcWaitArm,"race starts after core sampled disarmed ready");
    const auto decision=a::publish_attitude_at_boundary<RcMode,RcTarget>(context,q.o.attitude,123,
      [&](const RcTarget& target){++targets;for(float thrust:target.thrust_body)need(thrust==0,"race target has exact zero thrust");},
      [&](const RcMode&){++heartbeats;});
    need(decision.allowed&&targets==1&&heartbeats==1,"fresh arm event arriving between core and sink retains zero stream");
    bool threw=false;targets=heartbeats=0;
    try{
      a::publish_attitude_at_boundary<RcMode,RcTarget>(context,q.o.attitude,124,
        [&](const RcTarget&){
          ++targets;q.s.attitude.kill=true;
        },[&](const RcMode&){
          if(!a::attitude_publication_policy(q.r.publication_context(q.s,q.t+.02,true,true),q.o.attitude).allowed)
            throw std::runtime_error("HEARTBEAT_FINAL_CONTEXT_CHANGED");
          ++heartbeats;
        });
    }catch(const std::runtime_error&){threw=true;}
    need(threw&&targets==1&&heartbeats==0,"changed safety after target commit vetoes heartbeat independently");
  }
}
static void delayed_navigation_evidence_keeps_continuous_zero_commits(){
  for(bool offboard_first:{false,true}){
    Flight q;q.warm();q.manual_arm();
    const auto targets=q.target_commits,heartbeats=q.heartbeat_commits;
    if(offboard_first){q.s.feedback.offboard=true;q.s.nav_state=14;}
    else {q.s.attitude.rc_mode_slot=6;++q.s.attitude.operator_switch_generation;}
    for(int i=0;i<10;++i){
      q.tick();
      need(q.o.attitude.publish&&!a::nonzero(q.o.attitude.thrust),"500ms independent navigation evidence retains every zero stream tick");
      need(!q.r.attitude_core().flight_started()&&q.r.attitude_core().rc_manual_arm_proven(),"navigation pairing never substitutes for manual arm proof");
    }
    need(q.target_commits==targets+10&&q.heartbeat_commits==heartbeats+10,"both navigation arrival orders commit all ten target and heartbeat pairs");
    q.s.feedback.offboard=true;q.s.nav_state=14;q.s.attitude.rc_mode_slot=6;
    ++q.s.attitude.operator_switch_generation;q.tick();q.tick();
    need(q.r.attitude_core().phase()==a::Phase::ArmedGround,"new matching navigation source sequence joins after either transport order");
    need(!a::nonzero(q.o.attitude.thrust)&&q.arm_commands==0&&q.mode_commands==0,"navigation pairing grants no immediate thrust or automatic pilot commands");
  }
}
static void late_complementary_arm_evidence_cannot_revive_expired_attempt(){
  for(bool status_first:{false,true}){
    Flight q;q.warm();
    if(status_first){q.s.feedback.armed=true;q.s.armed_time=10;q.s.latest_arming_reason=2;q.s.at_rest=false;}
    else {q.s.attitude.rc_arm_switch=1;++q.s.attitude.operator_switch_generation;}
    q.tick();for(int i=0;i<49;++i)q.tick();
    q.s.feedback.armed=true;q.s.armed_time=10;q.s.latest_arming_reason=2;q.s.at_rest=false;
    q.s.attitude.rc_arm_switch=1;++q.s.attitude.operator_switch_generation;q.tick(true,.10);
    need(q.r.attitude_core().phase()==a::Phase::Safety&&
      q.r.attitude_core().reason()=="RC_MANUAL_ARM_EVIDENCE_TIMEOUT"&&!q.r.attitude_core().rc_manual_arm_proven(),
      "complementary arm evidence at 2.55s is rejected before proof is granted");
  }
}
static void navigation_pairing_deadline_cannot_be_renewed_or_crossed(){
  for(bool offboard_first:{false,true}){
    for(bool late_complement:{false,true}){
      Flight q;q.warm();q.manual_arm();
      if(offboard_first){q.s.feedback.offboard=true;q.s.nav_state=14;}
      else {q.s.attitude.rc_mode_slot=6;++q.s.attitude.operator_switch_generation;}
      q.tick();for(int i=0;i<49;++i)q.tick();
      if(late_complement){q.s.feedback.offboard=true;q.s.nav_state=14;q.s.attitude.rc_mode_slot=6;++q.s.attitude.operator_switch_generation;}
      auto expired=q.r.publication_context(q.s,q.t+.10,true,true);
      need(!a::attitude_publication_policy(expired,q.o.attitude).allowed,"final sink refuses navigation proof after its fixed pairing deadline");
      q.tick(true,.10);
      need(q.r.attitude_core().phase()==a::Phase::Safety&&!q.r.attitude_core().flight_started()&&!q.o.attitude.publish,
        "fresh repeated navigation samples and late complement cannot renew or cross the deadline");
      need(q.r.attitude_core().reason()==(offboard_first&&!late_complement?"RC_NAVIGATION_MODE_SLOT_MISMATCH":"RC_NAVIGATION_CONFIRMATION_TIMEOUT"),
        "navigation timeout distinguishes absent slot from missing timely confirmation");
    }
  }
}
static void navigation_pending_rejects_withdrawal_stale_sources_and_nonzero(){
  for(bool offboard_first:{false,true}){
    Flight q;q.warm();q.manual_arm();
    if(offboard_first){q.s.feedback.offboard=true;q.s.nav_state=14;}
    else {q.s.attitude.rc_mode_slot=6;++q.s.attitude.operator_switch_generation;}
    q.tick();
    auto nonzero=q.o.attitude;nonzero.thrust[2]=-std::numeric_limits<double>::denorm_min();
    need(!a::attitude_publication_policy(q.r.publication_context(q.s,q.t+.01,true,true),nonzero).allowed,
      "neither pending navigation order can publish even a sub-float nonzero thrust");
    if(offboard_first){q.s.feedback.offboard=false;q.s.nav_state=15;}
    else {q.s.attitude.rc_mode_slot=1;q.s.attitude.rc_switch_sample_stamp+=10000;}
    need(!a::attitude_publication_policy(q.r.publication_context(q.s,q.t+.01,true,true),q.o.attitude).allowed,
      "sink resampling refuses withdrawal of already observed navigation evidence");
    q.tick();
    need(q.r.attitude_core().phase()==a::Phase::Safety&&!q.r.attitude_core().flight_started(),"withdrawn navigation attempt latches without joining");
  }
  for(bool stale_mode:{false,true}){
    Flight q;q.warm();q.manual_arm();const auto stabilized_status=q.s.status_stamp;
    if(stale_mode){
      q.s.feedback.offboard=true;q.s.nav_state=14;q.s.status_stamp=stabilized_status;
    }else {
      q.s.attitude.rc_mode_slot=6;q.s.attitude.rc_switch_sample_stamp=1;
    }
    need(!a::attitude_publication_policy(q.r.publication_context(q.s,q.t+.01,true,true),q.o.attitude).allowed,
      "new navigation values carried by old source timestamps cannot use pending permission");
  }
  Flight q;q.warm();q.manual_arm();q.s.feedback.offboard=true;q.s.nav_state=14;
  q.s.status_stamp+=10000;
  auto context=q.r.publication_context(q.s,q.t+.01,true,true);
  need(a::attitude_publication_policy(context,q.o.attitude).allowed,"new Offboard observation between core and sink is covered by zero-only pending permission");
  q.s.armed_time=11;
  need(!a::attitude_publication_policy(q.r.publication_context(q.s,q.t+.02,true,true),q.o.attitude).allowed,
    "navigation pending permission cannot cross an armed episode");
}
static void delayed_pre_ready_source_events_cannot_authorize(){
  // Extra clock member lets this regression execute against the pre-fix
  // admission implementation, which ignored host time entirely.
  struct TimedFeedback:a::Feedback{std::uint64_t source_host_stamp{};};
  Flight fixture;TimedFeedback s;
  static_cast<a::Feedback&>(s)=fixture.r.publication_context(fixture.s,0,true,true).feedback;
  a::RcNavigationStart start;double now=0;
  const auto step=[&]{
    now+=.05;s.source_host_stamp=100000000+std::uint64_t(now*1e6);
    s.status_stamp=s.rc_switch_sample_stamp=s.source_host_stamp-500000;
    const bool output=start.step(fixture.c.attitude,s,now,"");
    if(output)start.note_commit(true,now);
  };
  while(start.phase()!=a::RcStartPhase::WaitArm&&now<5)step();
  need(start.phase()==a::RcStartPhase::WaitArm,"delayed but healthy ground may prepare");
  s.rc_arm_switch=1;s.armed=true;s.latest_arming_reason=2;s.armed_time=10;s.at_rest=false;step();
  need(!start.manual_arm_proven(),"pre-ready arm source event must not authorize after delayed delivery");
  s.rc_mode_slot=6;s.offboard=true;s.stabilized_selected=false;step();
  need(start.phase()!=a::RcStartPhase::Joined,"pre-ready sequence cannot join flight");

  // Transport delay itself remains supported when the actual events are
  // post-ready, including armed status arriving ahead of its switch sample.
  a::RcNavigationStart valid;now=0;
  static_cast<a::Feedback&>(s)=fixture.r.publication_context(fixture.s,0,true,true).feedback;
  const auto valid_step=[&]{
    now+=.05;s.source_host_stamp=100000000+std::uint64_t(now*1e6);
    s.status_stamp=s.rc_switch_sample_stamp=s.source_host_stamp-500000;
    if(valid.step(fixture.c.attitude,s,now,""))valid.note_commit(true,now);
  };
  for(int i=0;i<100;++i)valid_step();
  need(valid.phase()==a::RcStartPhase::WaitArm,"valid delayed stream remains ready");
  s.armed=true;s.latest_arming_reason=2;s.armed_time=10;s.at_rest=false;valid_step();
  need(!valid.manual_arm_proven(),"status-first arrival still awaits independent switch");
  s.rc_arm_switch=1;valid_step();s.rc_mode_slot=6;valid_step();
  s.offboard=true;s.stabilized_selected=false;valid_step();
  need(valid.phase()==a::RcStartPhase::Joined,"post-ready delayed reordered events still join");
}
static void unavailable_evidence_never_launches(){
  for(int k=0;k<12;++k){
    Flight q;
    if(k==0){q.s.feedback.armed=true;q.s.armed_time=10;q.s.latest_arming_reason=2;q.s.attitude.rc_arm_switch=1;}
    if(k==1)q.s.attitude.rc_mode_slot=6;
    if(k==2)q.s.attitude.rc_arm_switch=1;
    if(k==3)q.s.attitude.rc_source_clock_valid=false;
    if(k==4)q.s.attitude.rc_axes_safe=false;
    if(k==5)q.s.local_fresh=false;
    if(k==6)q.s.attitude.kill=true;
    if(k==7)q.s.attitude.rc_aux_switches_safe=false;
    if(k==8)q.s.attitude.source_clock_valid=false;
    if(k==9)q.s.attitude.offboard_signal_lost=true;
    if(k==10)q.s.feedback.exclusive=false;
    q.warm(k!=11);
    need(!q.r.attitude_core().flight_started(),"ineligible startup cannot launch");
    need(!a::nonzero(q.o.attitude.thrust),"ineligible startup has no thrust");
    need(q.r.attitude_core().phase()!=a::Phase::RcWaitArm,"ineligible startup cannot claim arm readiness");
    need(q.arm_commands==0&&q.mode_commands==0,"startup never synthesizes pilot actions");
  }
}
static void arm_and_navigation_faults_latch(){
  for(int k=0;k<6;++k){
    Flight q;q.warm();
    if(k==0){q.s.attitude.rc_mode_slot=6;++q.s.attitude.operator_switch_generation;q.tick();}
    else{
      q.manual_arm();
      if(k==1)q.s.latest_arming_reason=4;
      if(k==2)q.s.feedback.landed=false;
      if(k==3)q.s.attitude.z-=.2;
      if(k==4)q.s.attitude.z_reset++;
      if(k==5)q.s.attitude.rc_axes_safe=false;
      q.tick();q.navigation();
    }
    need(!q.r.attitude_core().flight_started(),"bad admission cannot become a flight");
    need(q.r.attitude_core().phase()==a::Phase::Safety,"armed or navigation fault latches");
  }
  Flight timeout;timeout.warm();timeout.manual_arm();
  for(int i=0;i<170;++i)timeout.tick();
  need(timeout.r.attitude_core().phase()==a::Phase::Safety,"manual arm navigation timeout latches");
}
static void reference_is_disarmed_only(){
  Flight q;q.warm();q.s.attitude.z=2.03;q.tick();
  const auto before=q.r.attitude_core().reference_z();need(std::abs(before-2.03)<1e-9,"disarmed anchor refreshes");
  q.manual_arm();q.s.attitude.z=2.04;q.tick();
  need(q.r.attitude_core().reference_z()==before,"armed anchor never recaptures");
  q.navigation();need(q.r.attitude_core().reference_z()==before,"navigation keeps pre-arm anchor");
}
static void final_boundary_and_interface_reject_bypasses(){
  Flight boot;
  need(!boot.r.request("MODE",{},boot.s,boot.t).accepted,"RC role cannot invoke legacy ground Offboard test at boot");
  auto wrong_profile=rc_config();wrong_profile.profile="ground";bool mixed=false;
  try{wrong_profile.validate();}catch(const std::invalid_argument&){mixed=true;}
  need(mixed,"RC role requires actual attitude profile");
  Flight q;q.warm();
  for(const char* op:{"INTERACTIVE","PING","ATTITUDE_READY","TAKEOFF","ARM_ONLY","AUTHORITY_RC"})
    need(!q.r.request(op,std::string(op)=="TAKEOFF"?std::vector<double>{.5}:std::vector<double>{},q.s,q.t).accepted,"RC role rejects synthetic console flight path");
  auto c=q.r.publication_context(q.s,q.t,true,true);
  a::Output invalid=q.o.attitude;invalid.thrust[2]=-.1;
  need(!a::attitude_publication_policy(c,invalid).allowed,"startup nonzero final sink denied");
  c.phase=a::Phase::RequestArm;c.flight_started=true;
  need(!a::arm_publication_policy(c).allowed,"final ARM sink denied independent of upstream phase");
  auto p=q.c.attitude;p.arm_enabled=true;bool rejected=false;
  try{p.validate();}catch(const std::invalid_argument&){rejected=true;}
  need(rejected,"manual RC profile rejects automatic arm flag");
  q.manual_arm();q.navigation();
  c=q.r.publication_context(q.s,q.t,true,true);invalid=q.o.attitude;
  c.feedback.rc_axes_safe=false;
  need(!a::attitude_publication_policy(c,invalid).allowed,"resampled unsafe axes veto join output");
}
static void exits_and_faults_preserve_existing_policy(){
  Flight q;q.join();for(int i=0;i<16;++i)q.tick();
  q.s.attitude.rc_mode_slot=1;++q.s.attitude.operator_switch_generation;q.tick();
  need(q.r.attitude_core().phase()==a::Phase::PoweredGround||q.r.attitude_core().phase()==a::Phase::Descent,"mode request retains guarded recovery until confirmed");
  q.s.feedback.offboard=false;q.s.nav_state=15;q.tick();
  need(!q.o.attitude.publish&&q.r.attitude_core().phase()==a::Phase::Safety,"actual mode-out transfers ownership");
  need(q.mode_commands==0,"no mode re-entry after pilot exit");
  Flight stop;stop.join();for(int i=0;i<16;++i)stop.tick();
  stop.r.begin_shutdown(stop.s,stop.t);stop.tick();
  need(stop.o.attitude.publish,"shutdown keeps native recovery stream");
  Flight bad;bad.join();bad.s.attitude.z_reset++;bad.tick();
  need(bad.r.attitude_core().phase()==a::Phase::Safety&&!bad.o.attitude.publish,"flight estimator guard unchanged");
}
static void full_height_hold_land_and_safe_reset(){
  Flight q;q.join();for(int i=0;i<16;++i)q.tick();
  double height=0;
  while(height<.5-1e-9){height=std::min(.5,height+.01);
    q.s.attitude.z=2-height;q.s.attitude.vz=q.s.attitude.z_deriv=-.2;
    q.s.feedback.landed=q.s.at_rest=false;q.tick();}
  q.s.attitude.vz=q.s.attitude.z_deriv=0;
  for(int i=0;i<60;++i)q.tick();
  need(q.r.attitude_core().phase()==a::Phase::HeightHold,"RC flight reaches height hold using existing vertical loop");
  need(q.r.request("LAND",{},q.s,q.t).accepted,"land available without interactive lease");
  while(height>1e-9){height=std::max(0.,height-.005);
    q.s.attitude.z=2-height;q.s.attitude.vz=q.s.attitude.z_deriv=.1;q.tick();}
  q.s.attitude.vz=q.s.attitude.z_deriv=0;q.s.feedback.landed=q.s.at_rest=true;
  bool disarm=false;
  for(int i=0;i<100&&!disarm;++i){q.tick();
    if(q.o.attitude.command==a::Command::Disarm){
      need(a::disarm_publication_policy(q.r.publication_context(q.s,q.t,true,true)).allowed,"fresh proven landing authorizes ground Disarm");
      q.r.confirm_attitude_command_publication(a::Command::Disarm,true,q.s.status_stamp+1,q.s,q.t);
      q.r.ack({q.r.pending(400).baseline+1,400,0,1,191,false},q.t+.001);disarm=true;
    }
  }
  need(disarm,"landing reaches first actual Disarm transaction");
  q.s.feedback.armed=false;q.s.latest_arming_reason=4;
  // Retain the original 100 ms source-clock causal floor after a real sink.
  for(int i=0;i<10&&q.r.attitude_core().phase()!=a::Phase::SafeDisarmed;++i)q.tick();
  need(q.r.attitude_core().phase()==a::Phase::SafeDisarmed,"new disarmed status completes physical landing proof");
  need(!q.r.rc_restart_allowed(q.s),"held navigation/arm cannot restart mission");
  q.s.feedback.offboard=false;q.s.nav_state=15;q.s.attitude.rc_mode_slot=1;q.s.attitude.rc_arm_switch=3;
  q.s.stationary_seconds=3;q.tick();
  need(q.r.rc_restart_allowed(q.s),"new safe disarmed ground plus reset controls permits a fresh RC attempt");
  const auto completed_disarm=q.r.pending(400);const auto history=q.r.command400_history().size();
  for(int failure=0;failure<7;++failure){auto bad=q.s;
    if(failure==0)bad.attitude_output_active=true;
    if(failure==1)bad.offboard_heartbeat_active=true;
    if(failure==2)bad.publication_fault_latched=true;
    if(failure==3)bad.time_regressed=true;
    if(failure==4)bad.status_fresh=false;
    if(failure==5)bad.feedback.armed=true;
    if(failure==6)bad.attitude.source_clock_validated=false;
    need(!q.r.reprepare_rc(bad,q.t),"unsafe current evidence cannot rebuild the RC core");
  }
  need(q.r.reprepare_rc(q.s,q.t),"safe completed RC mission must reprepare in the same Runtime");
  need(q.r.attitude_core().phase()==a::Phase::Idle&&!q.r.attitude_core().reference_ready(),
    "reprepare captures a fresh ground reference rather than retaining the old attempt");
  need(q.r.pending(400).generation==completed_disarm.generation&&q.r.pending(400).wire_stamp==completed_disarm.wire_stamp&&
    q.r.pending(400).state_verified==completed_disarm.state_verified&&q.r.command400_history().size()==history,
    "reprepare must preserve command400 wire history and its local generation");
  need(!q.r.reprepare_rc(q.s,q.t),"idle new attempt cannot repeatedly rebuild itself");
  q.s.attitude.z=2.02;q.warm();
  need(q.r.attitude_core().phase()==a::Phase::RcWaitArm&&q.r.attitude_core().reference_z()==2.02,
    "same process reaches second readiness using the current ground reference");
  q.s.attitude.rc_arm_switch=1;++q.s.attitude.operator_switch_generation;q.tick();
  q.s.feedback.armed=true;q.s.armed_time=20;q.s.latest_arming_reason=2;q.s.at_rest=false;q.tick();
  q.navigation();need(q.r.attitude_core().phase()==a::Phase::ArmedGround,
    "fresh manual arm and navigation edges start the second mission");
  need(q.r.pending(400).generation==completed_disarm.generation,
    "second manual arm never fabricates a new command400 transaction");
  q.s.at_rest=true;
  need(q.r.request("LAND",{},q.s,q.t).accepted,"second mission can request guarded landing");
  bool second_disarm=false;
  for(int i=0;i<100&&!second_disarm;++i){q.tick();
    if(q.o.attitude.command==a::Command::Disarm){
      q.r.confirm_attitude_command_publication(a::Command::Disarm,true,q.s.status_stamp+1,q.s,q.t);
      second_disarm=true;
    }
  }
  need(second_disarm&&q.r.pending(400).generation==2&&q.r.pending(400).ack_ambiguous&&
    q.r.command400_history().size()==history+1,"second Disarm preserves reuse ambiguity and archives the first transaction");
  q.r.ack({q.s.status_stamp+200001,400,0,1,191,false},q.t+.201);
  need(!q.r.pending(400).accepted&&!q.r.pending(400).state_verified,
    "a delayed first-mission ACK cannot complete the reused second Disarm");
  q.s.feedback.armed=false;
  for(int i=0;i<10&&q.r.attitude_core().phase()!=a::Phase::SafeDisarmed;++i)q.tick();
  need(q.r.attitude_core().phase()==a::Phase::SafeDisarmed&&q.r.pending(400).state_verified,
    "independent new disarmed ground evidence completes the second physical landing");
  need(q.arm_commands==0&&q.mode_commands==0,"full flight never emits automatic Arm or Offboard entry");
}
// Values in the selected Flight Test runtime profile. Unlike the historical
// .5 m fixtures above, this keeps the deployed ramp, PID, limits and guards.
static f::Config production_no_vision_rc_config(){
  auto c=config();c.runtime_mode="flight-test";c.flight_enabled=false;
  auto& p=c.attitude;
  p.flight_actions_enabled=true;p.arm_enabled=false;p.takeoff_enabled=true;
  p.parameters_confirmed=true;p.rc_navigation_enabled=true;
  p.rc_takeover_confirmed=true;p.autonomous_failsafe_confirmed=false;
  p.rc_navigation_mode_slot=1;p.rc_target_height=1.5;p.max_flight_seconds=30;
  p.max_tilt_deg=10;p.land_sample_max_gap=1.25;
  p.vertical={.50,.12,.80,.40,.50,.15,.03,.08,5.,.30,.20,.25};
  p.takeoff_reserve_thrust=.04;
  p.ramp_seconds=2.;p.settle_seconds=2.;p.landing_seconds=3.;
  p.height_tolerance=.15;p.speed_tolerance=.15;p.ascent_timeout=10.;p.descent_timeout=10.;
  p.saturation_seconds=1.5;p.armed_dwell_seconds=1.;p.liftoff_dwell_seconds=.20;
  p.liftoff_height_m=.12;p.liftoff_speed_mps=.08;p.touchdown_dwell_seconds=.30;
  p.height_lower_margin_m=.25;p.z_jump_limit_m=.45;p.vz_jump_limit_mps=1.5;
  p.feedback_guard.enabled=true;p.feedback_guard.rate_tolerance_mps=.20;p.feedback_guard.dwell_seconds=.50;
  c.validate();return c;
}

static void production_no_vision_rc_1p5m_lifecycle_uses_vertical_only(){
  auto c=production_no_vision_rc_config();
  bool rejected=false;auto invalid=c;invalid.flight_enabled=true;
  try{invalid.validate();}catch(const std::invalid_argument&){rejected=true;}
  need(rejected,"attitude altitude rejects the unrelated legacy flight gate");
  for(int gate=0;gate<4;++gate){
    auto disabled=c;auto& p=disabled.attitude;
    if(gate==0)p.flight_actions_enabled=false;
    if(gate==1)p.takeoff_enabled=false;
    if(gate==2)p.parameters_confirmed=false;
    if(gate==3)p.rc_takeover_confirmed=false;
    rejected=false;try{disabled.validate();}catch(const std::invalid_argument&){rejected=true;}
    need(rejected,"each independent RC flight permission remains mandatory");
  }

  Flight q(c);
  // Observed diagnostic condition: horizontal position and heading are
  // unavailable, while the independent attitude and vertical estimates work.
  q.s.feedback.valid=false;q.s.feedback.heading_ok=false;
  q.s.attitude.xy_valid=q.s.attitude.vxy_valid=false;
  q.s.attitude.z_valid=q.s.attitude.vz_valid=true;
  q.join();
  const double reference=q.r.attitude_core().reference_z();
  for(int i=0;i<25;++i)q.tick();
  need(q.r.attitude_core().phase()==a::Phase::ThrustRamp,
    "production RC route reaches thrust ramp without XY or heading validity");
  need(!q.r.pending(400).requested&&!q.r.pending(176).requested&&
       q.arm_commands==0&&q.mode_commands==0,"RC route retains manual Arm and Offboard ownership");

  // Time and requested thrust alone must not certify liftoff. Holding the
  // ground sample also verifies that the bounded reserve can reach .54.
  for(int i=0;i<85;++i)q.tick();
  need(q.r.attitude_core().phase()==a::Phase::ThrustRamp&&
       std::abs(q.o.attitude.thrust[2]+.54)<1e-9&&!q.r.attitude_core().liftoff_confirmed(),
    "production ramp reaches hover plus reserve without inventing liftoff");
  q.s.feedback.landed=false;q.s.at_rest=false;
  double height=0.;
  for(int i=0;i<50&&q.r.attitude_core().phase()==a::Phase::ThrustRamp;++i){
    height+=.005;q.s.attitude.z=reference-height;
    q.s.attitude.vz=q.s.attitude.z_deriv=-.10;q.tick();
  }
  need(q.r.attitude_core().liftoff_confirmed()&&q.r.attitude_core().phase()==a::Phase::Ascend,
    (std::string("independent land detector and z/vz evidence confirm liftoff: ")+q.r.attitude_core().reason()).c_str());

  // Simulated measured NED climb, not a physical flight simulation: all
  // source stamps advance and z_deriv stays consistent with vertical motion.
  for(int i=0;i<190&&q.r.attitude_core().phase()!=a::Phase::HeightHold;++i){
    height=std::min(1.5,height+.015);q.s.attitude.z=reference-height;
    q.s.attitude.vz=height<1.5?-.30:0.;q.s.attitude.z_deriv=q.s.attitude.vz;q.tick();
  }
  need(q.r.attitude_core().phase()==a::Phase::HeightHold&&
       q.r.attitude_core().target_height_settled()&&q.r.attitude_core().reference_z()==reference&&
       std::abs(q.o.attitude.target_height-1.5)<1e-9,
    "vertical-only feedback settles at the original reference plus 1.5 m");
  need(q.o.attitude.vertical_closed_loop&&q.o.attitude.thrust_ceiling==.80&&
       !q.o.attitude.visual_motion,"height hold uses the global thrust envelope and no vision");
  for(int event=0;event<5;++event){
    auto context=q.r.publication_context(q.s,q.t,true,true);
    if(event==0)context.feedback.kill=true;
    if(event==1)context.feedback.rc_link_ready=false;
    if(event==2)context.feedback.z_valid=false;
    if(event==3)context.feedback.q={0,0,0,0};
    if(event==4)context.feedback.offboard_signal_lost=true;
    need(!a::attitude_publication_policy(context,q.o.attitude).allowed,
      "Kill RC loss invalid vertical attitude and Offboard loss veto the final sink");
  }

  need(q.r.request("LAND",{},q.s,q.t).accepted,"manual landing remains available");
  bool descent_seen=false,disarm_seen=false;
  for(int i=0;i<260&&q.r.attitude_core().phase()!=a::Phase::SafeDisarmed;++i){
    height=std::max(0.,height-.01);q.s.attitude.z=reference-height;
    q.s.attitude.vz=q.s.attitude.z_deriv=height>0?.20:0.;
    if(height<=0.){q.s.feedback.landed=true;q.s.at_rest=true;}
    q.tick();descent_seen=descent_seen||q.r.attitude_core().phase()==a::Phase::Descent;
    if(q.o.attitude.command==a::Command::Disarm){
      need(a::disarm_publication_policy(q.r.publication_context(q.s,q.t,true,true)).allowed,
        "new touchdown evidence authorizes guarded disarm");
      q.r.confirm_attitude_command_publication(a::Command::Disarm,true,q.s.status_stamp+1,q.s,q.t);
      q.r.ack({q.r.pending(400).baseline+1,400,0,1,191,false},q.t+.001);
      q.s.feedback.armed=false;disarm_seen=true;
    }
  }
  need(descent_seen&&disarm_seen&&q.r.attitude_core().phase()==a::Phase::SafeDisarmed,
    "production 1.5 m mission confirms descent touchdown and actual disarmed status");
  // The operator now confirms the PX4 mode release on the ground.  Feeding
  // that fresh Stabilized status before the first SafeDisarmed follow-up tick
  // exercises the normal external-release path and must leave the core in
  // Idle without issuing a redundant 176 transaction.
  q.s.feedback.offboard=false;q.s.nav_state=15;q.s.attitude.rc_mode_slot=6;
  q.s.attitude.rc_arm_switch=3;q.s.stationary_seconds=3;q.tick();
  need(q.r.attitude_core().phase()==a::Phase::Idle&&
       !q.o.attitude.publish&&!a::nonzero(q.o.attitude.thrust),
    "fresh disarmed Stabilized status completes the release to Idle");
  need(q.arm_commands==0&&q.mode_commands==0&&q.target_commits==q.heartbeat_commits,
    "full no-vision lifecycle retains paired sinks and no automatic Arm or Offboard entry");
  std::cout<<"RC_PRODUCTION_NO_VISION_1P5M_VERTICAL_ONLY_FLIGHT_ENABLED_FALSE_PASS\n";
}

static void ended_fault_needs_new_safe_ground_cycle(){
  auto c=rc_config();c.attitude.feedback_guard.enabled=true;c.validate();Flight q(c);
  q.join();for(int i=0;i<16;++i)q.tick();
  q.s.attitude.z_deriv=-.6;
  for(int i=0;i<16&&q.r.attitude_core().phase()!=a::Phase::Safety;++i){
    q.s.attitude.z-=.03;q.tick();
  }
  need(q.r.attitude_core().phase()==a::Phase::Safety&&
    q.r.attitude_core().reason()=="VERTICAL_VELOCITY_INCONSISTENT",
    "real vertical disagreement faults the active RC task");
  const auto generation=q.r.pending(400).generation;
  const auto observe=[&]{
    q.r.begin_attitude_safety_observation();
    const auto fb=q.r.publication_context(q.s,q.t,true,true).feedback;
    q.r.observe_attitude_safety_event(fb,a::Core::SafetyEvent::Vertical,q.t);
    q.r.observe_attitude_safety_event(fb,a::Core::SafetyEvent::Authority,q.t);
    q.tick();need(!q.o.attitude.publish&&!a::nonzero(q.o.attitude.thrust),
      "fault recovery observation is output-silent");
  };
  q.s.attitude.z_deriv=0;q.s.attitude.z=2;
  for(int i=0;i<50;++i){observe();need(!q.r.reprepare_rc(q.s,q.t),"still-armed fault cannot restart");}
  q.s.feedback.armed=false;q.s.feedback.offboard=false;q.s.nav_state=15;
  q.s.feedback.landed=q.s.at_rest=true;q.s.stationary_seconds=3;
  q.s.attitude.rc_arm_switch=3;q.s.attitude.rc_mode_slot=1;
  observe();need(!q.r.reprepare_rc(q.s,q.t),"one healthy ground sample cannot clear the old fault");
  q.s.attitude.z_deriv=.6;
  for(int i=0;i<45;++i){observe();need(!q.r.reprepare_rc(q.s,q.t),"original disagreement still blocks recovery");}
  q.s.attitude.z_deriv=0;q.s.attitude.kill=true;
  for(int i=0;i<45;++i){observe();need(!q.r.reprepare_rc(q.s,q.t),"held Kill forbids a new cycle");}
  q.s.attitude.kill=false;
  for(int i=0;i<43;++i)observe();
  need(q.r.rc_restart_allowed(q.s),"new healthy multi-frame ground dwell admits ended fault recovery");
  // A bad event followed by a healthy latest snapshot must invalidate the
  // completed dwell before Runtime can reconstruct task state in this tick.
  q.r.begin_attitude_safety_observation();auto bad=q.r.publication_context(q.s,q.t,true,true).feedback;
  bad.q={0,0,0,0};q.r.observe_attitude_safety_event(bad,a::Core::SafetyEvent::Attitude,q.t);
  const auto good=q.r.publication_context(q.s,q.t,true,true).feedback;
  q.r.observe_attitude_safety_event(good,a::Core::SafetyEvent::Attitude,q.t);q.tick();
  need(!q.r.reprepare_rc(q.s,q.t),"same-round healthy attitude cannot erase bad recovery evidence");
  for(int i=0;i<43;++i)observe();
  for(int invalid=0;invalid<9;++invalid){auto changed=q.s;
    if(invalid==0)changed.arming_state_known=false;
    if(invalid==1)changed.feedback.landed=false;
    if(invalid==2)changed.attitude.kill=true;
    if(invalid==3)changed.attitude.rc_switch_sample_stamp=1;
    if(invalid==4)changed.attitude.z_deriv=.6;
    if(invalid==5)changed.attitude.rc_axes_safe=false;
    if(invalid==6)changed.attitude.rc_sticks_moving=true;
    if(invalid==7)changed.attitude.rc_arm_switch=1;
    if(invalid==8)changed.attitude.rc_mode_slot=6;
    need(!q.r.reprepare_rc(changed,q.t),"current unsafe or old RC evidence cannot consume an earlier recovery dwell");
  }
  need(q.r.reprepare_rc(q.s,q.t),"ended measurement fault safely resets task state in the same process");
  need(q.r.last_rc_attempt_reason()=="VERTICAL_VELOCITY_INCONSISTENT"&&q.r.rc_attempts_completed()==1,
    "reprepare preserves historical reason and attempt sequence");
  need(q.r.pending(400).generation==generation,"safe reprepare preserves command wire history");
  q.warm();need(q.r.attitude_core().phase()==a::Phase::RcWaitArm,
    "recovered service returns to the ordinary next-manual-arm readiness");
  need(!q.r.attitude_core().flight_started()&&q.arm_commands==0&&q.mode_commands==0,
    "recovery cannot automatically arm or start the next task");
  q.s.attitude.rc_arm_switch=1;++q.s.attitude.operator_switch_generation;q.tick();
  q.s.feedback.armed=true;q.s.armed_time=20;q.s.latest_arming_reason=2;q.s.at_rest=false;q.tick();
  q.navigation();need(q.r.attitude_core().phase()==a::Phase::ArmedGround,
    "new manual arm and new Offboard evidence can join a subsequent task");
}
static void cold_start_never_resumes_armed_or_unknown_flight(){
  for(int condition=0;condition<4;++condition){Flight q;
    if(condition==0){q.s.feedback.armed=true;q.s.feedback.offboard=true;q.s.nav_state=14;
      q.s.attitude.rc_arm_switch=1;q.s.attitude.rc_mode_slot=6;}
    if(condition==1)q.s.status_fresh=false;
    if(condition==2)q.s.land_fresh=false;
    if(condition==3)q.s.arming_state_known=false;
    for(int i=0;i<60;++i){q.tick();need(!q.o.attitude.publish&&!a::nonzero(q.o.attitude.thrust)&&
      !q.r.attitude_core().flight_started(),"cold start cannot continue armed or unknown custody");}
    need(q.arm_commands==0&&q.mode_commands==0,"process restart creates no arm or Offboard command");
  }
}
static void integrity_faults_stay_latched_after_safe_ground(){
  for(const char* reason:{"TELEMETRY_TIME_REGRESSION","ROS_ADAPTER_CONTROL_LOOP_STALL",
      "ATTITUDE_PUBLICATION_PARTIAL_PAIR","COMMAND_400_INTENT_MISMATCH",
      "PUBLISHER_OR_TELEMETRY_CONFLICT"}){
    Flight q;q.join();q.r.abort_attitude(reason);q.s.feedback.armed=false;
    q.s.feedback.offboard=false;q.s.nav_state=15;q.s.at_rest=true;
    q.s.attitude.rc_arm_switch=3;q.s.attitude.rc_mode_slot=1;
    for(int i=0;i<60;++i){q.tick();need(!q.r.reprepare_rc(q.s,q.t),
      "safe switches cannot clear unresolved execution integrity faults");}
    q.r.abort_attitude("RC_MANUAL_ARM_EVIDENCE_TIMEOUT");
    need(q.r.attitude_core().reason()==reason&&!q.r.reprepare_rc(q.s,q.t),
      "a recoverable-looking later reason cannot replace the original fault");
  }
}
static void fault_after_disarm_preserves_and_resolves_only_committed_history(){
  for(bool committed:{false,true}){Flight q;q.join();q.s.at_rest=true;
    need(q.r.request("LAND",{},q.s,q.t).accepted,"ground task can request its existing guarded finish");
    bool planned=false;
    for(int i=0;i<100&&!planned;++i){q.tick();planned=q.o.attitude.command==a::Command::Disarm;}
    need(planned,"test reaches actual planned disarm");
    if(committed)q.r.confirm_attitude_command_publication(a::Command::Disarm,true,
      q.s.status_stamp+1,q.s,q.t);
    const auto pending=q.r.pending(400);
    q.r.abort_attitude("ATTITUDE_STALE_OR_INVALID");
    q.s.feedback.armed=false;q.s.feedback.offboard=false;q.s.nav_state=15;
    q.s.feedback.landed=q.s.at_rest=true;q.s.attitude.rc_arm_switch=3;q.s.attitude.rc_mode_slot=1;
    for(int i=0;i<45;++i)q.tick();
    need(q.r.rc_restart_allowed(q.s),"fresh safe ground qualifies ended task only");
    need(q.r.reprepare_rc(q.s,q.t)==committed,
      "independent ground proof settles committed Disarm but cannot erase an uncommitted plan");
    need(q.r.pending(400).wire_stamp==pending.wire_stamp&&q.r.pending(400).generation==pending.generation,
      "physical terminal observation preserves original wire stamp and generation");
    if(committed)need(q.r.pending(400).state_verified&&!q.r.pending(400).accepted&&
      q.r.pending(400).stage==f::CommandStage::StateOnlyTerminal,
      "safe ground resolves physical outcome without inventing an accepted ACK");
  }
}
int main(int argc,char** argv){
  if(argc==2){
    if(std::string(argv[1])=="delay")delayed_arm_evidence_keeps_continuous_zero_commits();
    else if(std::string(argv[1])=="slot")configured_navigation_slots_require_matching_new_edge();
    else if(std::string(argv[1])=="timeout")missing_arm_evidence_has_an_unrenewable_deadline();
    else if(std::string(argv[1])=="navigation")delayed_navigation_evidence_keeps_continuous_zero_commits();
    else if(std::string(argv[1])=="late_arm")late_complementary_arm_evidence_cannot_revive_expired_attempt();
    else if(std::string(argv[1])=="production_1p5m")production_no_vision_rc_1p5m_lifecycle_uses_vertical_only();
    else if(std::string(argv[1])=="lifecycle"){
      ended_fault_needs_new_safe_ground_cycle();cold_start_never_resumes_armed_or_unknown_flight();
      integrity_faults_stay_latched_after_safe_ground();
      fault_after_disarm_preserves_and_resolves_only_committed_history();
      std::cout<<"RC_SAFE_LIFECYCLE_PASS\n";
    }
    else throw std::runtime_error("unknown regression selector");
    return 0;
  }
  delayed_arm_evidence_keeps_continuous_zero_commits();configured_navigation_slots_require_matching_new_edge();
  missing_arm_evidence_has_an_unrenewable_deadline();pending_permission_rechecks_current_safety_and_exact_zero();
  sink_resampling_handles_new_arm_observations_and_vetoes_changes();
  delayed_navigation_evidence_keeps_continuous_zero_commits();late_complementary_arm_evidence_cannot_revive_expired_attempt();
  navigation_pairing_deadline_cannot_be_renewed_or_crossed();navigation_pending_rejects_withdrawal_stale_sources_and_nonzero();
  delayed_pre_ready_source_events_cannot_authorize();
  valid_path_without_automatic_commands();status_switch_reordering();unavailable_evidence_never_launches();
  arm_and_navigation_faults_latch();reference_is_disarmed_only();final_boundary_and_interface_reject_bypasses();
  exits_and_faults_preserve_existing_policy();
  full_height_hold_land_and_safe_reset();
  ended_fault_needs_new_safe_ground_cycle();cold_start_never_resumes_armed_or_unknown_flight();
  integrity_faults_stay_latched_after_safe_ground();
  fault_after_disarm_preserves_and_resolves_only_committed_history();
  production_no_vision_rc_1p5m_lifecycle_uses_vertical_only();
  std::cout<<"RC_NAVIGATION_CONTRACT_PASS\n";
}
