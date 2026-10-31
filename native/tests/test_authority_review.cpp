// Independent-review regressions exercise the actual observation helper, Core,
// Runtime and final publisher, with synthetic configuration and fake sinks only.
#include "authority_test_support.hpp"
#include "jikl_fc01_system/authority_telemetry.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <iostream>
using namespace authority_test;
struct Manual {
  static constexpr std::uint8_t SOURCE_UNKNOWN=0,SOURCE_RC=1,SOURCE_MAVLINK_0=2;
  std::uint64_t timestamp{1000000},timestamp_sample{1000000};
  bool valid{true};std::uint8_t data_source{SOURCE_RC};bool sticks_moving{};
};
struct Switches {
  std::uint64_t timestamp{1000000},timestamp_sample{1000000};
  std::uint8_t mode_slot{1},arm_switch{3},return_switch{3},loiter_switch{3},offboard_switch{3},kill_switch{3};
  std::uint32_t switch_changes{};
};
struct Mode {std::uint64_t timestamp{};bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};};
struct Target {std::uint64_t timestamp{};float yaw_sp_move_rate{};std::array<float,4> q_d{};std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};};
static void first_sample(){
  auto c=config();c.attitude.rc_takeover_confirmed=true;f::Runtime r(c);
  auto s=sample();double t=0;ready(r,s,t,"AUTHORITY_RC");
  f::AuthorityTelemetry observer;Manual manual;Switches sw;
  observer.manual(manual,t);observer.switches(sw,t,c.attitude.flags_timeout);
  auto facts=observer.facts(t,c.attitude.flags_timeout,true,false,1,1);
  assert(!facts.rc_link_ready&&!facts.rc_switches_ready);
  s.attitude.rc_link_ready=facts.rc_link_ready;s.attitude.rc_switches_ready=facts.rc_switches_ready;
  assert(!r.request("TAKEOFF",{1},s,t).accepted);
  int arms=0;a::publish_arm_at_boundary(r.publication_context(s,t,true,true),[&]{++arms;});assert(arms==0);
  manual.timestamp+=1000;sw.timestamp+=1000; // fresh envelopes alone do not prove raw progression
  observer.manual(manual,t+.01);observer.switches(sw,t+.01,c.attitude.flags_timeout);
  facts=observer.facts(t+.01,c.attitude.flags_timeout,true,false,1,1);
  assert(!facts.rc_link_ready&&!facts.rc_switches_ready);
  manual.timestamp+=1000;manual.timestamp_sample+=2000;
  sw.timestamp+=1000;sw.timestamp_sample+=2000;
  observer.manual(manual,t+.02);observer.switches(sw,t+.02,c.attitude.flags_timeout);
  facts=observer.facts(t+.02,c.attitude.flags_timeout,true,false,1,1);
  assert(facts.rc_link_ready&&facts.rc_switches_ready);
  s.attitude.rc_link_ready=facts.rc_link_ready;s.attitude.rc_switches_ready=facts.rc_switches_ready;
  assert(r.request("TAKEOFF",{1},s,t+.02).accepted);
  tick(r,s,t);a::publish_arm_at_boundary(r.publication_context(s,t,true,true),[&]{++arms;});assert(arms==1);
  // After a stale gap, one new raw sample must not immediately restore readiness.
  const double late=t+c.attitude.flags_timeout+1;
  manual.timestamp+=1000;manual.timestamp_sample+=1000;sw.timestamp+=1000;sw.timestamp_sample+=1000;
  observer.manual(manual,late);observer.switches(sw,late,c.attitude.flags_timeout);
  facts=observer.facts(late,c.attitude.flags_timeout,true,false,1,1);
  assert(!facts.rc_link_ready&&!facts.rc_switches_ready);
}
static void fault_after_takeover(){
  {
    auto c=config();f::Runtime r(c);auto s=sample();double t=0;
    ready(r,s,t);s.attitude.kill=true;tick(r,s,t);s.attitude.kill=false;
    assert(r.attitude_core().authority_state().takeover_latched);
    r.abort_attitude("ROS_ADAPTER_CONTROL_LOOP_STALL");
    tick(r,s,t);
    assert(!r.request("AUTHORITY_RESET",{},s,t).accepted);
    // Direct Core API release must also preserve an earlier ordinary fault.
    a::Core core(c.attitude);core.live_console(t);
    core.block("ROS_ADAPTER_CONTROL_LOOP_STALL");core.release_to_operator("LIVE_OPERATOR_RELEASE");
    assert(core.reset_authority(r.publication_context(s,t,true,true).feedback,t,false,false,false)==
      "AUTHORITY_SAFETY_RESTART_REQUIRED");
    assert(r.request("AUTHORITY_RESET",{},s,t).message=="AUTHORITY_SAFETY_RESTART_REQUIRED");
    r.request("STOP",{},s,t);r.request("LAND",{},s,t);r.request("AUTHORITY_TAKEOVER",{},s,t);
    assert(!r.request("AUTHORITY_RESET",{},s,t).accepted);
  }
}
static void switch_counter(){
  auto c=config();c.attitude.autonomous_failsafe_confirmed=true;f::Runtime r(c);
  auto s=sample();double t=0;f::AuthorityTelemetry observer;Switches sw;
  observer.switches(sw,t,c.attitude.flags_timeout);
  s.attitude.operator_switch_generation=observer.facts(t,c.attitude.flags_timeout,true,false,1,1).switch_generation;
  armed(r,s,t,"AUTHORITY_AUTONOMOUS");auto old=tick(r,s,t).attitude;assert(old.publish);
  sw.timestamp+=2000000;sw.timestamp_sample+=2000000;sw.switch_changes=2;
  observer.switches(sw,t,c.attitude.flags_timeout);
  const auto facts=observer.facts(t,c.attitude.flags_timeout,true,false,1,1);
  assert(facts.switch_generation!=s.attitude.operator_switch_generation);
  s.attitude.operator_switch_generation=facts.switch_generation;
  int targets=0,heartbeats=0;
  auto decision=a::publish_attitude_at_boundary<Mode,Target>(r.publication_context(s,t,true,true),old,42,
    [&](const Target&){++targets;},[&](const Mode&){++heartbeats;});
  assert(!decision.allowed&&targets==0&&heartbeats==0);
  auto out=tick(r,s,t);
  // RC switch request alone is not actual PX4 nav-state handoff: keep
  // controlled descent / ground-motor reduction while still Offboard.
  assert(out.attitude.publish&&out.attitude.command==a::Command::None);
  assert(r.attitude_core().operator_release_requested());
  assert(!r.attitude_core().authority_state().takeover_latched);
  for(int i=0;i<5;++i){out=tick(r,s,t);assert(out.attitude.publish);}
  s.feedback.offboard=false;s.nav_state=15;
  out=tick(r,s,t);assert(!out.attitude.publish);
  assert(r.attitude_core().authority_state().takeover_latched);
  // Counter reset/wrap also records an intervention, without interpreting direction.
  sw.timestamp+=1000;sw.timestamp_sample+=1000;sw.switch_changes=0;
  observer.switches(sw,t,c.attitude.flags_timeout);
  assert(observer.facts(t,c.attitude.flags_timeout,true,false,1,1).switch_generation>facts.switch_generation);
}
static void immediate_diagnostics(){
  for(int event=0;event<6;++event){
    auto c=config();c.attitude.autonomous_failsafe_confirmed=true;f::Runtime r(c);
    auto s=sample();double t=0;
    if(event==0){r.request("INTERACTIVE",{},s,t);r.request("AUTHORITY_AUTONOMOUS",{},s,t);}
    else ready(r,s,t,"AUTHORITY_AUTONOMOUS");
    switch(event){case 0:case 1:s.attitude.kill=true;break;case 2:s.attitude.operator_takeover=true;break;
      case 3:s.feedback.failsafe=true;break;case 4:s.attitude.offboard_signal_lost=true;break;
      case 5:++s.attitude.operator_switch_generation;break;}
    auto json=r.authority_json(s);
    assert(json.find("\"authority_ready\":false")!=std::string::npos);
    assert(json.find("\"authority_reason\":\"OPERATOR_TAKEOVER_LATCHED\"")!=std::string::npos);
    if(event)assert(!r.request("TAKEOFF",{1},s,t).accepted);
  }
}
int main(int argc,char** argv){
  const std::string which=argc>1?argv[1]:"all";
  if(which=="first"||which=="all")first_sample();
  if(which=="fault"||which=="all")fault_after_takeover();
  if(which=="counter"||which=="all")switch_counter();
  if(which=="diagnostic"||which=="all")immediate_diagnostics();
  std::cout<<"independent review regressions ("<<which<<"): PASS\n";
}
