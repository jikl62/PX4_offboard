// Reuse existing independent fake-PX4 runtime fixture. No hardware required.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main p0_attitude_runtime_fixture_main_not_called
#include "test_attitude_runtime.cpp"
#undef main
#pragma GCC diagnostic pop
#include <cassert>
#include <cmath>
#include <iostream>
int main(){
  f::Runtime r(cfg(true));
  auto s=sample();
  double t=ready(r,s,true);
  assert(r.request("INTERACTIVE",{},s,t).accepted);
  assert(r.request("PING",{},s,t).accepted);
  assert(r.request("TAKEOFF",{.6},s,t).accepted);
  auto o=tick(r,s,t+=.05);
  assert(o.attitude.command==a::Command::Arm);
  r.ack({r.pending(400).baseline+1,400,0,1,191,false},t+.01);
  s.feedback.armed=true;
  o=tick(r,s,t+=.05);
  assert(r.attitude_core().phase()==a::Phase::ArmedGround);
  assert(!r.attitude_core().tilt_fault_evidence().valid);
  s.attitude.q={std::cos(20.2*3.14159265358979323846/360.),
    std::sin(20.2*3.14159265358979323846/360.),0,0};
  const auto status_at_fault=s.status_stamp+50000;
  const auto land_at_fault=s.land_stamp+50000;
  const auto local_at_fault=s.attitude.local_stamp+50000;
  o=tick(r,s,t+=.05);
  assert(!o.attitude.publish);
  assert(r.attitude_core().phase()==a::Phase::Safety);
  assert(r.attitude_core().reason()=="EXCESS_TILT");
  auto evidence=r.attitude_core().tilt_fault_evidence();
  assert(evidence.valid);
  assert(std::abs(evidence.measured_tilt_deg-20.2)<1e-8);
  assert(std::abs(evidence.limit_deg-cfg(true).attitude.max_tilt_deg)<1e-8);
  assert(evidence.armed&&evidence.offboard);
  assert(evidence.status_stamp==status_at_fault&&evidence.land_stamp==land_at_fault&&evidence.local_stamp==local_at_fault);
  assert(std::abs(evidence.actual_q[1]-s.attitude.q[1])<1e-12);
  const auto stored=evidence;
  s.attitude.q={1,0,0,0};
  tick(r,s,t+=.05);
  evidence=r.attitude_core().tilt_fault_evidence();
  assert(evidence.valid&&evidence.actual_q==stored.actual_q);
  std::cout<<"FC01_P0_TILT_TRIGGER_SNAPSHOT_PASS\n";
}
