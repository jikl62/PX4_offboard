// Mutations caught: upstream UNSET admission; unsafe selection; unlocking during
// preparation; clearing pending Arm; reacting to healthy inputs by reacquiring.
#include "authority_test_support.hpp"
#include <iostream>
#include <set>
using namespace authority_test;
static void default_and_selection(){
  auto c=config();c.attitude.autonomous_failsafe_confirmed=true;f::Runtime r(c);auto s=sample();double t=0;
  s.manual_fresh=true; // satisfy the old ambiguous gate to isolate the new UNSET requirement
  ready(r,s,t);
  assert(!r.request("TAKEOFF",{1},s,t).accepted); // baseline RED: succeeds without a mode
  assert(!r.pending(400).sent&&state(r,s,t).mode==auth::Mode::Unset);
  assert(state(r,s,t).locked);
  assert(r.request("AUTHORITY_RC",{},s,t).message=="AUTHORITY_MODE_LOCKED");
  f::Runtime idle(c);s=sample();t=0;
  assert(!idle.request("AUTHORITY_AUTONOMOUS",{},s,t).accepted);
  assert(idle.request("INTERACTIVE",{},s,t).accepted);
  for(int failure=0;failure<10;++failure){auto bad=s;
    switch(failure){case 0:bad.status_fresh=false;break;case 1:bad.land_fresh=false;break;
      case 2:bad.feedback.armed=true;break;case 3:bad.feedback.landed=false;break;
      case 4:bad.at_rest=false;break;case 5:bad.feedback.exclusive=false;break;case 6:bad.time_regressed=true;break;
      case 7:bad.attitude_output_active=true;break;case 8:bad.offboard_heartbeat_active=true;break;
      case 9:bad.publication_fault_latched=true;break;}
    assert(!idle.request("AUTHORITY_AUTONOMOUS",{},bad,t).accepted);
  }
  assert(idle.request("AUTHORITY_AUTONOMOUS",{},s,t).accepted);
  assert(state(idle,s,t).mode==auth::Mode::AutonomousNoRc&&!state(idle,s,t).locked);
  assert(idle.request("ATTITUDE_READY",{},s,t).accepted);
  assert(state(idle,s,t).locked);
  assert(!idle.request("AUTHORITY_RC",{},s,t).accepted);
  idle.disconnect(s,t);tick(idle,s,t);
  assert(!idle.request("ATTITUDE_READY",{},s,t).accepted);
  idle.request("INTERACTIVE",{},s,t);
  assert(idle.request("AUTHORITY_RESET",{},s,t).accepted);
}
static void route_admission(){
  for(int failure=0;failure<4;++failure){auto c=config();auto s=sample();double t=0;
    c.attitude.rc_takeover_confirmed=failure!=1;
    s.attitude.rc_link_ready=failure!=0;s.attitude.rc_switches_ready=failure!=2;
    f::Runtime r(c);ready(r,s,t,"AUTHORITY_RC");
    const auto reply=r.request("TAKEOFF",{1},s,t);
    assert(reply.accepted==(failure==3));
    if(failure<3)assert(!r.pending(400).sent);
  }
  for(bool confirmed:{false,true}){auto c=config();c.attitude.autonomous_failsafe_confirmed=confirmed;
    f::Runtime r(c);auto s=sample();double t=0;ready(r,s,t,"AUTHORITY_AUTONOMOUS");
    assert(r.request("TAKEOFF",{1},s,t).accepted==confirmed);
    assert(!s.manual_fresh&&!s.attitude.rc_link_ready&&!s.attitude.rc_switches_ready);
  }
}
static void takeover_and_no_reacquire(){
  for(int failure=0;failure<7;++failure){auto c=config();c.attitude.rc_takeover_confirmed=true;
    f::Runtime r(c);auto s=sample();s.attitude.rc_link_ready=s.attitude.rc_switches_ready=true;
    double t=0;armed(r,s,t,"AUTHORITY_RC");
    switch(failure){case 0:s.feedback.offboard=false;s.nav_state=15;break;
      case 1:s.attitude.operator_takeover=true;break;case 2:s.attitude.rc_link_ready=false;break;
      case 3:s.attitude.kill=true;break;case 4:s.feedback.failsafe=true;break;
      case 5:s.attitude.offboard_signal_lost=true;break;case 6:++s.attitude.operator_switch_generation;break;}
    auto out=tick(r,s,t);
    if(failure==1||failure==6){
      assert(out.attitude.publish&&r.attitude_core().operator_release_requested());
      assert(!state(r,s,t).takeover_latched);
      // RC request without real PX4 nav departure does NOT transfer custody.
      s.feedback.offboard=false;s.nav_state=15;out=tick(r,s,t);
    }
    assert(!out.attitude.publish&&out.attitude.command==a::Command::None);
    assert(state(r,s,t).takeover_latched);
    s=sample();s.feedback.offboard=true;s.attitude.rc_link_ready=s.attitude.rc_switches_ready=true;
    for(int i=0;i<8;++i){out=tick(r,s,t);assert(!out.attitude.publish&&out.attitude.command==a::Command::None);}
    assert(!r.request("ATTITUDE_READY",{},s,t).accepted);
    // Selection cannot clear a takeover; explicit safe reset is separate.
    assert(!r.request("AUTHORITY_AUTONOMOUS",{},s,t).accepted);
    assert(r.request("AUTHORITY_RESET",{},s,t).accepted);
    assert(r.attitude_core().phase()==a::Phase::Idle&&!state(r,s,t).locked);
    assert(state(r,s,t).mode==auth::Mode::Unset&&!state(r,s,t).takeover_latched);
    assert(!tick(r,s,t).attitude.publish);
  }
  auto c=config();f::Runtime r(c);auto s=sample();double t=0;
  r.request("INTERACTIVE",{},s,t);r.request("ATTITUDE_READY",{},s,t);r.step(s,t);
  while(r.attitude_core().phase()!=a::Phase::RequestOffboard)tick(r,s,t);
  s.feedback.offboard=true;tick(r,s,t); // observed before ACK
  s.feedback.offboard=false;auto out=tick(r,s,t);
  assert(state(r,s,t).takeover_latched&&!out.attitude.publish&&out.attitude.command==a::Command::None);
}
static void pending_and_faults(){
  auto c=config();c.attitude.autonomous_failsafe_confirmed=true;f::Runtime r(c);auto s=sample();double t=0;
  ready(r,s,t,"AUTHORITY_AUTONOMOUS");r.request("TAKEOFF",{1},s,t);tick(r,s,t);
  assert(r.pending(400).sent&&!r.pending(400).accepted);
  s.attitude.operator_takeover=true;tick(r,s,t);s.attitude.operator_takeover=false;
  assert(!r.request("AUTHORITY_RESET",{},s,t).accepted);
  r.ack({r.pending(400).baseline+1,400,0,1,191,false},t+.01);
  tick(r,s,t);assert(r.request("AUTHORITY_RESET",{},s,t).accepted);
  f::Runtime fault(c);s=sample();t=0;ready(fault,s,t,"AUTHORITY_AUTONOMOUS");
  fault.abort_attitude("PUBLISHER_BOUNDARY_TEST_FAULT");tick(fault,s,t);
  assert(!fault.request("AUTHORITY_RESET",{},s,t).accepted);
  assert(fault.request("STOP",{},s,t).accepted);
  assert(!fault.request("AUTHORITY_RESET",{},s,t).accepted); // STOP cannot erase a safety fault
}
static void completion_and_all_phases(){
  auto c=config();c.attitude.autonomous_failsafe_confirmed=true;f::Runtime r(c);auto s=sample();double t=0;
  ready(r,s,t,"AUTHORITY_AUTONOMOUS");assert(r.request("TAKEOFF",{1},s,t).accepted);
  std::set<a::Phase> seen;
  auto locked=[&]{seen.insert(r.attitude_core().phase());assert(state(r,s,t).locked);
    assert(r.request("AUTHORITY_RC",{},s,t).message=="AUTHORITY_MODE_LOCKED");};
  locked();assert(r.attitude_core().phase()==a::Phase::RequestArm);
  auto out=tick(r,s,t);assert(out.attitude.command==a::Command::Arm);
  r.ack({r.pending(400).baseline+1,400,0,1,191,false},t+.01);s.feedback.armed=true;
  tick(r,s,t);locked();tick(r,s,t);locked();
  for(int i=0;i<7;++i){tick(r,s,t);locked();}
  assert(r.attitude_core().phase()==a::Phase::ThrustRamp);
  s.feedback.landed=false;s.at_rest=false;s.attitude.vz=-.1;
  for(int i=0;i<70;++i){
    s.attitude.z=std::max(1.,s.attitude.z-.035);
    s.attitude.vz=s.attitude.z>1.01?-.1:0;
    tick(r,s,t);locked();
  }
  assert(r.attitude_core().phase()==a::Phase::HeightHold);
  r.request("LAND",{},s,t);locked();
  for(int i=0;i<95&&r.attitude_core().phase()!=a::Phase::RequestDisarm;++i){
    s.attitude.z=std::min(2.,s.attitude.z+.03);
    s.attitude.vz=s.attitude.z<1.99?.1:0;
    if(s.attitude.z>=1.99){s.feedback.landed=s.at_rest=true;}
    tick(r,s,t);locked();
  }
  assert(r.attitude_core().phase()==a::Phase::RequestDisarm);
  assert(!r.request("AUTHORITY_RESET",{},s,t).accepted);
  r.ack({r.pending(400).baseline+1,400,0,1,191,false},t+.01);s.feedback.armed=false;tick(r,s,t);
  assert(r.attitude_core().phase()==a::Phase::SafeDisarmed&&state(r,s,t).locked);
  assert(!r.request("AUTHORITY_RC",{},s,t).accepted);
  assert(!r.request("AUTHORITY_RESET",{},s,t).accepted);
  auto released=tick(r,s,t);assert(released.attitude.command==a::Command::ReleaseOffboard);
  const auto watermark=r.pending(176).baseline;
  r.ack({watermark+5,176,0,1,191,false},t+.01);
  s.feedback.offboard=false;s.nav_state=15;tick(r,s,t);
  assert(r.attitude_core().phase()==a::Phase::Idle);
  assert(r.request("AUTHORITY_RESET",{},s,t).accepted);
  assert(r.request("AUTHORITY_RC",{},s,t).accepted);
  for(auto phase:{a::Phase::RequestArm,a::Phase::ArmedGround,a::Phase::ThrustRamp,a::Phase::Ascend,a::Phase::HeightHold,a::Phase::Descent,a::Phase::Landed,a::Phase::RequestDisarm})assert(seen.count(phase));
}
int main(){default_and_selection();route_admission();takeover_and_no_reacquire();pending_and_faults();completion_and_all_phases();
  std::cout<<"authority Runtime admission/selection/phase locks/takeover/ground reset/pending400: PASS\n";
}
