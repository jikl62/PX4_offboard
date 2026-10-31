#include "jikl_fc01_system/ground_link.hpp"
#include "jikl_fc01_system/runtime.hpp"
#include <cassert>
#include <iostream>

namespace g = jikl_fc01_ground_link;
namespace f = fc01;

static g::Traffic traffic(){ return {1,1,1,1,1}; }

static void ready_and_send(g::Core& c, std::uint64_t stamp=2000){
  c.status({1000,true,false,false,false,true,1,1,4},0.0);
  c.land({1000,true,true},0.0);
  assert(!c.tick(0.0,traffic()));
  c.status({stamp,true,false,false,false,true,1,1,4},2.1);
  c.land({stamp,true,true},2.1);
  assert(c.tick(2.1,traffic()));
  assert(c.state()==g::State::Sent);
}

static void test_accepted_ack_from_external_false(){
  g::Core c; ready_and_send(c);
  assert(c.ack({3000,176,0,1,191,false},2.2));
  c.status({4000,true,false,false,false,true,1,1,15},2.3);
  c.land({4000,true,true},2.3);
  c.tick(2.3,traffic());
  assert(c.state()==g::State::Pass);
}

static void test_denied_ack(){
  g::Core c; ready_and_send(c);
  assert(c.ack({3000,176,2,1,191,false},2.2));
  assert(c.state()==g::State::Rejected && c.ack_result()==2);
}

static void test_failed_ack(){
  g::Core c; ready_and_send(c);
  assert(c.ack({3000,176,4,1,191,false},2.2));
  assert(c.state()==g::State::Rejected && c.ack_result()==4);
}

static void test_in_progress_then_accepted(){
  g::Core c; ready_and_send(c);
  assert(c.ack({3000,176,5,1,191,false},2.2));
  assert(c.state()==g::State::Sent && c.ack_result()==5);
  assert(c.ack({4000,176,0,1,191,false},2.3));
  c.status({5000,true,false,false,false,true,1,1,15},2.4);
  c.land({5000,true,true},2.4);
  c.tick(2.4,traffic());
  assert(c.state()==g::State::Pass);
}

static void test_wrong_command(){
  g::Core c; ready_and_send(c);
  assert(!c.ack({3000,512,0,1,191,false},2.2));
  assert(!c.ack_seen());
}

static void test_wrong_target(){
  g::Core c; ready_and_send(c);
  assert(!c.ack({3000,176,0,255,190,false},2.2));
  assert(!c.ack_seen());
}

static void test_stale_ack(){
  g::Core c; ready_and_send(c,3000);
  assert(!c.ack({3000,176,0,1,191,false},2.2));
  assert(!c.ack_seen());
}

static void test_duplicate_ack(){
  g::Core c; ready_and_send(c);
  assert(c.ack({3000,176,5,1,191,false},2.2));
  assert(!c.ack({3000,176,5,1,191,false},2.21));
  assert(c.ack_result()==5 && c.state()==g::State::Sent);
}

static void test_ack_before_command(){
  g::Core c;
  c.status({1000,true,false,false,false,true,1,1,4},0.0);
  c.land({1000,true,true},0.0);
  assert(!c.ack({1500,176,0,1,191,false},0.1));
  assert(!c.tick(0.0,traffic()));
  c.status({2000,true,false,false,false,true,1,1,4},2.1);
  c.land({2000,true,true},2.1);
  assert(c.tick(2.1,traffic()));
  assert(!c.ack({1500,176,0,1,191,false},2.2));
  assert(!c.ack_seen());
}

static void test_new_status_without_valid_ack(){
  g::Core c; ready_and_send(c);
  c.status({3000,true,false,false,false,true,1,1,15},2.2);
  c.land({3000,true,true},2.2);
  c.tick(2.2,traffic());
  assert(c.state()==g::State::Sent);
  c.status({4000,true,false,false,false,true,1,1,15},4.6);
  c.land({4000,true,true},4.6);
  c.tick(4.6,traffic());
  c.status({5000,true,false,false,false,true,1,1,15},7.2);
  c.land({5000,true,true},7.2);
  c.tick(7.2,traffic());
  assert(c.state()==g::State::AckTimeout);
}

static f::Snapshot flight_snapshot(){
  f::Snapshot s;
  s.status_fresh=s.land_fresh=s.local_fresh=true;
  s.at_rest=true;s.stationary_seconds=3;
  s.battery_fresh=true;s.battery_connected=true;s.power_valid=true;s.battery_warning=0;
  s.manual_fresh=true;s.manual_lost=false;s.dead_reckoning=false;s.time_regressed=false;
  s.status_stamp=10000;s.land_stamp=10000;s.system=1;s.component=1;s.nav_state=4;
  s.feedback.fresh=true;s.feedback.valid=true;s.feedback.heading_ok=true;s.feedback.preflight=true;
  s.feedback.armed=false;s.feedback.landed=true;s.feedback.offboard=false;s.feedback.landing=false;s.feedback.failsafe=false;s.feedback.exclusive=true;
  s.feedback.position={0,0,0};s.feedback.velocity={0,0,0};s.feedback.yaw=0;
  return s;
}

static f::Runtime make_flight_runtime(){
  f::Config c;c.profile="flight";c.flight_enabled=true;return f::Runtime(c);
}

static void drive_to_offboard_pending(f::Runtime& r, f::Snapshot& s){
  assert(r.request("PING",{},s,0.0).accepted);
  assert(r.request("TAKEOFF",{1.0},s,0.1).accepted);
  for(double t=0.1;t<=2.4;t+=0.1){
    s.status_stamp += 1000;s.land_stamp += 1000;
    auto out=r.step(s,t);
    (void)out;
  }
  assert(r.pending(176).sent);
}

static void test_runtime_strict_correlation_and_false_external(){
  auto r=make_flight_runtime();auto s=flight_snapshot();drive_to_offboard_pending(r,s);
  const auto base=r.pending(176).baseline;
  r.ack({base+1,512,0,1,191,false},2.5);
  assert(!r.pending(176).accepted);
  r.ack({base+2,176,0,255,190,false},2.5);
  assert(!r.pending(176).accepted);
  r.ack({base,176,0,1,191,false},2.5);
  assert(!r.pending(176).accepted);
  r.ack({base+3,176,5,1,191,false},2.5);
  assert(r.pending(176).result==5 && !r.pending(176).accepted);
  r.ack({base+4,176,0,1,191,false},2.6);
  assert(r.pending(176).accepted && r.pending(176).result==0);
  r.ack({base+4,176,0,1,191,false},2.7);
  assert(r.pending(176).accepted && r.pending(176).last_ack==base+4);
}

int main(){
  test_accepted_ack_from_external_false();
  test_denied_ack();
  test_failed_ack();
  test_in_progress_then_accepted();
  test_wrong_command();
  test_wrong_target();
  test_stale_ack();
  test_duplicate_ack();
  test_ack_before_command();
  test_new_status_without_valid_ack();
  test_runtime_strict_correlation_and_false_external();
  std::cout << "ack lifecycle tests: PASS\n";
}
