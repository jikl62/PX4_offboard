// Positive regression against the actual R5 Armed+Offboard timeout defect.
// Synthetic offline observations, never physical controller calibration.
#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <iostream>
#include <stdexcept>
namespace a=fc01::attitude;
namespace f=fc01;
struct ModeR6 {std::uint64_t timestamp{};bool attitude{},position{},velocity{};};
struct TargetR6 {std::uint64_t timestamp{};float yaw_sp_move_rate{};
  std::array<float,4> q_d{};std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};};
static void check(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
struct RecoveryFixture {
  f::Config c;f::Runtime r;f::Snapshot s;double now{};
  unsigned targets{},heartbeats{};
  explicit RecoveryFixture(f::Config cfg=config()):c(cfg),r(c),s(authority_test::sample()){
    authority_test::armed(r,s,now,"AUTHORITY_AUTONOMOUS");
    s.feedback.landed=false;s.at_rest=false;s.attitude.z=1.8;s.attitude.vz=0;
    check(r.request("LAND",{},s,now).accepted,"LAND rejected");
    s.attitude.local_stamp=1000;
  }
  static f::Config config(){auto c=authority_test::config();
    c.attitude.autonomous_failsafe_confirmed=true;
    c.attitude.saturation_seconds=100;return c;}
  a::Output tick(bool ping=true){now+=.05;s.status_stamp+=1000;s.land_stamp+=1000;
    s.attitude.local_stamp+=1000;
    if(ping)r.request("PING",{},s,now);
    auto o=r.step(s,now).attitude;
    if(o.publish){const auto d=a::publish_attitude_at_boundary<ModeR6,TargetR6>(
      r.publication_context(s,now,!r.shutting_down(),true),o,std::uint64_t(now*1e6),
      [&](const TargetR6&){++targets;},[&](const ModeR6&){++heartbeats;});
      check(d.allowed,"final sink refused recovery");}
    return o;
  }
};
static void timeout_retains_both_sinks(bool shutdown){
  RecoveryFixture h;if(shutdown)h.r.begin_shutdown(h.s,h.now);
  double previous=0;
  for(int i=0;i<230;++i){const auto n=h.targets,m=h.heartbeats;const auto o=h.tick(!shutdown);
    check(o.publish,"DESCENT_TIMEOUT dropped healthy armed Offboard output");
    check(h.targets==n+1&&h.heartbeats==m+1,"DESCENT_TIMEOUT dropped a final sink");
    check(o.command!=a::Command::Disarm,"airborne routine Disarm");
    check(-o.thrust[2]>=0&&-o.thrust[2]<=h.c.attitude.vertical.maximum,"unbounded recovery thrust");
    check(std::abs(o.thrust[2]-previous)<=h.c.attitude.vertical.slew*.05+1e-9,"recovery thrust step bypassed slew");
    previous=o.thrust[2];
  }
  std::cout<<(shutdown?"R6_GRACEFUL_SHUTDOWN":"R6_DESCENT_TIMEOUT")<<" positive dual-sink continuity PASS\n";
}
static void recovery_budget_cannot_be_renewed(){
  RecoveryFixture h;
  double deadline=-1;
  for(int i=0;i<600;++i){
    if(i%7==0)check(h.r.request("LAND",{},h.s,h.now).accepted,"repeat LAND refused");
    if(i==220)h.r.begin_shutdown(h.s,h.now);
    auto o=h.tick();
    if(h.r.attitude_core().recovery_deadline()>=0){
      if(deadline<0)deadline=h.r.attitude_core().recovery_deadline();
      check(h.r.attitude_core().recovery_deadline()==deadline,"LAND renewed absolute recovery deadline");
    }
    check(o.command!=a::Command::Disarm,"recovery sent airborne Disarm");
    if(h.r.attitude_core().recovery_budget_exhausted()){
      check(o.publish&&h.now>=deadline,"expiry must retain healthy airborne custody");
      auto fb=h.r.publication_context(h.s,h.now,false,true).feedback;
      check(std::string(h.r.attitude_core().recovery_responsibility(fb))==
        "COMPANION_GUARDED_DESCENT_ACTIVE_RECOVERY_OVERDUE_HANDOFF_UNVERIFIED",
        "expiry must disclose guarded custody without claiming PX4 takeover");
      check(h.r.pending(400).command400_kind==f::Command400Kind::Arm,"budget planned Disarm");
      check(!h.r.request("TAKEOFF",{1},h.s,h.now).accepted,"rearmed after uncertainty");
      std::cout<<"R6.1_ABSOLUTE_BUDGET no LAND/shutdown renewal; overdue healthy custody retained PASS\n";return;
    }
    check(o.publish,"early recovery stream loss");
  }
  throw std::runtime_error("recovery expiry not observed");
}
static f::Config fallback_config(){auto c=RecoveryFixture::config();
  c.attitude.px4_land_fallback_confirmed=true;c.attitude.px4_land_evidence_id="OFFLINE_SYNTHETIC_ONLY";return c;}
static void land21_contract_and_current_sensors(){
  for(int missing=0;missing<5;++missing){
    auto c=missing==0?RecoveryFixture::config():fallback_config();RecoveryFixture h(c);
    h.s.attitude.xy_valid=missing!=1;h.s.attitude.vxy_valid=missing!=2;
    h.s.attitude.land_selectable=missing!=3;
    if(missing==4)h.s.attitude.xy_valid=false;
    for(int i=0;i<240;++i){auto o=h.tick();check(o.publish,"contract refusal killed recovery");
      check(o.command!=a::Command::Px4Land,"unverified local position triggered Land21");}
  }
  std::cout<<"R6_LAND21 current position and prior contract required PASS\n";
}
static void land21_unresolved_never_releases(int ack_case){
  RecoveryFixture h(fallback_config());h.r.require_attitude_sink_commit();
  h.s.attitude.xy_valid=h.s.attitude.vxy_valid=h.s.attitude.land_selectable=true;
  // Existing Arm was implicit in this fixture; 21 explicitly uses final commit.
  int lands=0;
  for(int i=0;i<1200;++i){auto o=h.tick();check(o.publish,"21 unresolved stopped stream after completion deadline");
    if(o.command==a::Command::Px4Land){
      ++lands;auto ctx=h.r.publication_context(h.s,h.now,true,true);
      check(a::publish_land_fallback_at_boundary(ctx,[]{}).allowed,"Land21 final guard refused valid contract");
      auto bad=ctx;bad.feedback.xy_valid=false;
      check(!a::publish_land_fallback_at_boundary(bad,[]{throw std::runtime_error("bad 21 reached sink");}).allowed,
        "Land21 ignored final XY loss");
      h.r.confirm_attitude_command_publication(o.command,true,2000000,h.s,h.now);
      if(ack_case!=2)h.r.ack({h.r.pending(21).baseline+1,21,std::uint8_t(ack_case==0?0:2),1,191,false},h.now+.001);
    }
    check(o.command!=a::Command::Disarm,"21 recovery sent Disarm");
  }
  check(lands==1&&h.r.pending(21).published,"21 retried or lacked final commit");
  check(h.r.attitude_core().recovery_budget_exhausted(),"21 observation did not cross full recovery budget");
  check(h.r.attitude_core().phase()==a::Phase::RecoveryDescent,"21 ACK fabricated mode handoff");
  std::cout<<"R6.1_LAND21 ack_case="<<ack_case<<" (accept/reject/missing) without new mode retains stream past budget; no retry PASS\n";
}
static void land21_requires_post_commit_status(){
  RecoveryFixture h(fallback_config());h.r.require_attitude_sink_commit();
  h.s.attitude.xy_valid=h.s.attitude.vxy_valid=h.s.attitude.land_selectable=true;
  for(int i=0;i<240;++i){auto o=h.tick();if(o.command==a::Command::Px4Land){
    h.r.confirm_attitude_command_publication(o.command,true,2000000,h.s,h.now);
    h.s.feedback.offboard=false;h.s.nav_state=18;
    h.tick();
    check(h.r.attitude_core().px4_land_mode_observed(),"new Land mode not recorded");
    const auto fb=h.r.publication_context(h.s,h.now,true,true).feedback;
    check(std::string(h.r.attitude_core().recovery_responsibility(fb))==
      "PX4_FAILSAFE_OBSERVED_BUT_CONTROL_UNVERIFIED","mode name fabricated working task");
    check(!h.tick().publish,"continued Offboard after PX4 mode exit");
    std::cout<<"R6_LAND21 new mode is observed ownership, task still unverified PASS\n";return;
  }}throw std::runtime_error("21 not planned");
}
static void failures_never_fabricate_control(){
  for(int which=0;which<8;++which){RecoveryFixture h;for(int i=0;i<220;++i)h.tick();
    if(which==0)h.s.local_fresh=false;
    if(which==1)h.s.attitude.z_valid=false;
    if(which==2)h.s.attitude.vz_valid=false;
    if(which==3)h.s.attitude.q={0,0,0,0};
    if(which==4)h.s.attitude.kill=true;
    if(which==5)h.s.feedback.exclusive=false; // includes DDS Agent disconnect / competing sinks.
    if(which==6)h.s.attitude.z_reset++;
    if(which==7)h.now+=.5; // scheduler stall / observed heartbeat gap.
    const auto o=h.tick();check(!o.publish&&o.command==a::Command::None,"unhealthy input kept flight output");
    check(h.r.attitude_core().phase()==a::Phase::Safety,"unhealthy input was called controlled recovery");
    check(h.r.attitude_core().fault_class()!=a::FaultClass::ControlledDescent,
      "fatal recovery fault retained ControlledDescent label after stopping output");
  }
  std::cout<<"R6_UNHEALTHY 8 injected control failures release with no command PASS\n";
}
static void submitted_disarm_fault_is_not_guarded_recovery(){
  auto c=RecoveryFixture::config();f::Runtime r(c);auto s=authority_test::sample();double now{};
  authority_test::armed(r,s,now,"AUTHORITY_AUTONOMOUS");
  check(r.request("LAND",{},s,now).accepted,"ground LAND refused");
  for(int i=0;i<80;++i){auto o=authority_test::tick(r,s,now).attitude;
    if(o.command==a::Command::Disarm){
      r.confirm_attitude_command_publication(o.command,true,3000000,s,now);
      s.attitude.z_valid=false;
      o=authority_test::tick(r,s,now).attitude;
      check(!o.publish,"invalid Z after submitted Disarm kept output");
      auto fb=r.publication_context(s,now,false,true).feedback;
      check(std::string(r.attitude_core().recovery_responsibility(fb))==
        "PX4_FAILSAFE_PENDING_UNKNOWN_AIRBORNE","Safety after submitted Disarm falsely claimed guarded recovery");
      std::cout<<"R6_SUBMITTED_DISARM_FAULT explicitly unknown ownership PASS\n";return;
    }
  }throw std::runtime_error("ground Disarm not planned");
}
static void external_ground_terminal_needs_contiguous_new_evidence(){
  auto cfg=RecoveryFixture::config();cfg.attitude.land_sample_max_gap=1.25;
  RecoveryFixture h(cfg);
  // A completion deadline is no longer a fatal control failure. Exercise
  // the ORIGINAL read-only Safety observer after an actual invalid-Z fault.
  h.s.attitude.z_valid=false;h.tick();h.s.attitude.z_valid=true;
  check(h.r.attitude_core().phase()==a::Phase::Safety,"invalid Z did not enter Safety");
  h.s.feedback.armed=false;h.s.feedback.landed=true;h.s.at_rest=true;
  h.s.attitude.z=2;h.s.attitude.vz=0;
  h.tick();
  for(int i=0;i<49;++i){h.now+=.05;h.s.status_stamp+=1000;h.r.step(h.s,h.now);}
  h.tick(); // Two observations separated by >1.25s do not prove ground dwell.
  auto fb=h.r.publication_context(h.s,h.now,false,true).feedback;
  check(std::string(h.r.attitude_core().recovery_responsibility(fb))!=
    "GROUND_DISARMED_CONFIRMED","gapped external ground samples claimed safe terminal");
  for(int i=0;i<45;++i)h.tick();
  fb=h.r.publication_context(h.s,h.now,false,true).feedback;
  check(std::string(h.r.attitude_core().recovery_responsibility(fb))==
    "GROUND_DISARMED_CONFIRMED","fresh independent ground dwell not observed");
  std::cout<<"R6_EXTERNAL_GROUND_TERMINAL contiguous new evidence required PASS\n";
}
int main(){timeout_retains_both_sinks(false);timeout_retains_both_sinks(true);
  recovery_budget_cannot_be_renewed();land21_contract_and_current_sensors();
  for(int ack_case=0;ack_case<3;++ack_case)land21_unresolved_never_releases(ack_case);
  land21_requires_post_commit_status();failures_never_fabricate_control();
  submitted_disarm_fault_is_not_guarded_recovery();external_ground_terminal_needs_contiguous_new_evidence();}
