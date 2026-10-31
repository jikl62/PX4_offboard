// FC01 R6.2 regression: actual ArmedGround failure samples and safe ground
// reference recapture. All sinks synthetic, NEVER connected to hardware.
#define main original_c_s10_main_not_invoked
#include "c_s10_independent/C_S10_SAFETY_ATTACKS.cpp"
#undef main
#include <cassert>
#include <cmath>
#include <iostream>

static void ground_reference_rebased_on_admitted_takeoff(){
  Harness h;
  h.ready();
  const double old_z=h.r.attitude_core().reference_z();
  h.s.attitude.z=old_z+.10; // two-minute preparation drift from field CSV
  h.s.attitude.vz=.05;
  assert(h.r.request("PING",{},h.s,h.t).accepted);
  auto r=h.r.request("TAKEOFF",{.6},h.s,h.t);
  assert(r.accepted);
  assert(std::abs(h.r.attitude_core().reference_z()-h.s.attitude.z)<1e-12);
  assert(h.r.attitude_core().event()=="TAKEOFF_DISARMED_GROUND_Z_REFERENCE_RECAPTURED");
  auto o=h.tick();
  assert(o.publish&&o.command==a::Command::Arm&&o.thrust[2]==0);
  assert(std::abs(o.height)<1e-12);
  assert(h.arms==1);
  std::cout<<"R62_ground_reference_takeoff_recapture PASS\n";
}
static void unstable_ground_before_takeoff_is_refused(){
  Harness h;h.ready();
  const double old_z=h.r.attitude_core().reference_z();
  h.s.attitude.vz=.137490094;
  auto r=h.r.request("TAKEOFF",{.6},h.s,h.t);
  assert(!r.accepted&&r.message=="TAKEOFF_GROUND_VZ_NOT_STABLE");
  assert(h.r.attitude_core().phase()==a::Phase::OffboardReady);
  assert(std::abs(h.r.attitude_core().reference_z()-old_z)<1e-12);
  assert(!h.r.attitude_core().armed_ground_fault_evidence().valid);
  auto o=h.tick();assert(o.publish&&o.command==a::Command::None&&o.thrust[2]==0);
  assert(h.arms==0);
  std::cout<<"R62_pre_arm_ground_vz_refusal PASS\n";
}
static void fail_exact_mask(double delta_z,double vz,bool landed,unsigned expected_mask){
  Harness h;h.armed();
  auto& c=h.r.attitude_core();
  const auto n_target=h.attitude_sinks,n_mode=h.mode_sinks;
  const double z0=c.reference_z();
  h.s.attitude.z=z0+delta_z;
  h.s.attitude.vz=vz;
  h.s.feedback.landed=landed;
  const auto stamp=h.s.land_stamp+1000;
  h.s.land_stamp=stamp;
  const auto o=h.tick(false);
  assert(!o.publish&&o.command==a::Command::None);
  assert(c.phase()==a::Phase::Safety&&c.reason()=="ARMED_GROUND_NOT_STABLE");
  const auto& e=c.armed_ground_fault_evidence();
  assert(e.valid&&e.failed_mask==expected_mask);
  assert(e.landed==landed&&e.armed&&e.offboard);
  assert(e.land_stamp==stamp&&e.status_stamp==h.s.status_stamp);
  assert(std::abs(e.z0-z0)<1e-12);
  assert(std::abs(e.z-(z0+delta_z))<1e-10);
  assert(std::abs(e.vz-vz)<1e-10);
  assert(std::abs(e.height+delta_z)<1e-10);
  assert(std::abs(e.up_speed+vz)<1e-10);
  assert(std::abs(e.height_limit-.15)<1e-12);
  assert(std::abs(e.vz_limit-.1)<1e-12);
  assert(h.attitude_sinks==n_target&&h.mode_sinks==n_mode);
  for(int i=0;i<15;++i){const auto x=h.tick();assert(!x.publish&&x.command==a::Command::None);}
  assert(h.disarms==0&&h.arms==1);
}
static void separate_early_guards_not_misattributed(){
  {Harness h;h.armed();h.s.feedback.armed=false;auto o=h.tick();
   assert(!o.publish&&h.r.attitude_core().phase()==a::Phase::Safety);
   assert(!h.r.attitude_core().armed_ground_fault_evidence().valid);}
  {Harness h;h.armed();h.s.feedback.offboard=false;h.s.nav_state=15;auto o=h.tick();
   assert(!o.publish&&h.r.attitude_core().phase()==a::Phase::Safety);
   assert(!h.r.attitude_core().armed_ground_fault_evidence().valid);}
  {Harness h;h.armed();h.s.attitude.vz_valid=false;auto o=h.tick();
   assert(!o.publish&&h.r.attitude_core().phase()==a::Phase::Safety);
   assert(!h.r.attitude_core().armed_ground_fault_evidence().valid);}
  std::cout<<"R62_earlier_guards_no_false_armed_ground_evidence PASS\n";
}
static void healthy_one_hz_must_still_reach_ramp(){
  Harness h;h.armed();h.one_hz=true;
  for(int i=0;i<80&&h.r.attitude_core().phase()!=a::Phase::ThrustRamp;++i){
    auto o=h.tick();assert(o.publish&&o.command==a::Command::None);
    assert(h.disarms==0);
  }
  assert(h.r.attitude_core().phase()==a::Phase::ThrustRamp);
  assert(!h.r.attitude_core().armed_ground_fault_evidence().valid);
  std::cout<<"R62_1Hz_detector_fresh_dwell_reaches_ramp PASS\n";
}
int main(){
  ground_reference_rebased_on_admitted_takeoff();
  unstable_ground_before_takeoff_is_refused();
  fail_exact_mask(0.,.137490094,true,16U);
  fail_exact_mask(0.,-.137490094,true,16U);
  fail_exact_mask(.20,0.,true,8U);
  fail_exact_mask(0.,0.,false,4U);
  fail_exact_mask(.20,.137490094,true,24U);
  separate_early_guards_not_misattributed();
  healthy_one_hz_must_still_reach_ramp();
  std::cout<<"R62_ARMED_GROUND_DIAGNOSTIC_AND_RECAPTURE: PASS\n";
  return 0;
}
