#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace authority_test;
static void need(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
static f::Config rc_config(std::int64_t slot=6){
  auto c=config();auto& p=c.attitude;
  p.rc_navigation_enabled=true;p.arm_enabled=false;p.rc_takeover_confirmed=true;
  p.rc_navigation_mode_slot=slot;
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
  explicit Flight(std::int64_t slot=6):c(rc_config(slot)),r(c){
    s.nav_state=15;s.attitude.source_clock_validated=true;
    s.attitude.rc_link_ready=s.attitude.rc_switches_ready=true;
    s.attitude.rc_source_clock_valid=s.attitude.rc_switch_clock_valid=true;
    s.attitude.rc_source_selected=s.attitude.rc_axes_safe=s.attitude.rc_aux_switches_safe=true;
    s.attitude.rc_mode_slot=slot==1?6:1;s.attitude.rc_arm_switch=s.attitude.rc_kill_switch=3;
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
  need(q.r.rc_restart_allowed(q.s),"new safe disarmed ground plus reset controls permits fresh process only");
  need(q.arm_commands==0&&q.mode_commands==0,"full flight never emits automatic Arm or Offboard entry");
}
int main(int argc,char** argv){
  if(argc==2){
    if(std::string(argv[1])=="delay")delayed_arm_evidence_keeps_continuous_zero_commits();
    else if(std::string(argv[1])=="slot")configured_navigation_slots_require_matching_new_edge();
    else if(std::string(argv[1])=="timeout")missing_arm_evidence_has_an_unrenewable_deadline();
    else if(std::string(argv[1])=="navigation")delayed_navigation_evidence_keeps_continuous_zero_commits();
    else if(std::string(argv[1])=="late_arm")late_complementary_arm_evidence_cannot_revive_expired_attempt();
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
  std::cout<<"RC_NAVIGATION_CONTRACT_PASS\n";
}
