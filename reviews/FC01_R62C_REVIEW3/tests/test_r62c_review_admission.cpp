#include "authority_test_support.hpp"
#include <iostream>
using namespace authority_test;
static f::Config bench(){
  auto c=config();c.attitude.bench_arm_only_enabled=true;c.attitude.takeoff_enabled=false;
  c.attitude.rc_takeover_confirmed=true;c.validate();return c;
}
static void unset_is_rejected_without_lock(){
  f::Runtime r(bench());auto s=sample();double t=0;
  r.request("INTERACTIVE",{},s,t);
  const auto reply=r.request("ATTITUDE_READY",{},s,t);
  assert(!reply.accepted&&reply.message=="AUTHORITY_MODE_UNSET");
  assert(r.attitude_core().phase()==a::Phase::Idle&&!state(r,s,t).locked);
  assert(!r.pending(176).requested&&!r.pending(400).requested);
  s.attitude.rc_link_ready=s.attitude.rc_switches_ready=true;
  assert(r.request("AUTHORITY_RC",{},s,t).accepted);
  assert(r.request("ATTITUDE_READY",{},s,t).accepted);
  assert(state(r,s,t).locked);
  std::cout<<"REVIEW_UNSET_AUTHORITY_NO_OFFBOARD_DEAD_END PASS\n";
}
static void unready_route_is_rejected_before_preparation(){
  for(int fault=0;fault<3;++fault){
    auto c=bench();auto s=sample();s.attitude.rc_link_ready=s.attitude.rc_switches_ready=true;
    if(fault==0)s.attitude.rc_link_ready=false;
    if(fault==1)s.attitude.rc_switches_ready=false;
    if(fault==2)c.attitude.rc_takeover_confirmed=false;
    f::Runtime r(c);r.request("INTERACTIVE",{},s,0);
    assert(r.request("AUTHORITY_RC",{},s,0).accepted);
    assert(!r.request("ATTITUDE_READY",{},s,0).accepted);
    assert(r.attitude_core().phase()==a::Phase::Idle&&!r.attitude_core().authority_state().locked);
  }
  std::cout<<"REVIEW_UNREADY_AUTHORITY_NO_PREPARATION PASS\n";
}
static void diagnostic_preparation_remains_unpowered(){
  auto c=bench();c.attitude.bench_arm_only_enabled=false;
  c.attitude.flight_actions_enabled=c.attitude.arm_enabled=c.attitude.takeoff_enabled=false;
  f::Runtime r(c);auto s=sample();
  assert(r.request("ATTITUDE_READY",{},s,0).accepted);
  assert(!r.request("ARM_ONLY",{},s,0).accepted);
  std::cout<<"REVIEW_CLOSED_GATES_DIAGNOSTIC_PREP_RETAINED PASS\n";
}
int main(){unset_is_rejected_without_lock();unready_route_is_rejected_before_preparation();
  diagnostic_preparation_remains_unpowered();}
