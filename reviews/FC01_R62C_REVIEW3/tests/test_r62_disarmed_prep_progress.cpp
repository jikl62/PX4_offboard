// R6.2a: zero-thrust disarmed preparation only, with independent 1Hz land samples.
// No ROS/PX4/FC01 or actuator access.
#define main original_c_s10_main_not_invoked
#include "c_s10_independent/C_S10_SAFETY_ATTACKS.cpp"
#undef main
#include <cassert>
#include <cmath>
#include <iostream>

static f::Config diag_config(){
  auto c=config();c.attitude.flight_actions_enabled=false;
  c.attitude.arm_enabled=false;c.attitude.takeoff_enabled=false;
  c.validate();return c;
}
static void start(f::Runtime& r,f::Snapshot& s){
  assert(r.request("ATTITUDE_READY",{},s,0).accepted);
  assert(r.attitude_core().phase()==a::Phase::SelfCheck);
}
static a::Output sample_tick(f::Runtime& r,f::Snapshot& s,int index,double z=2.,double vz=.2){
  s.attitude.z=z;s.attitude.vz=vz;
  s.status_stamp+=1000;s.attitude.local_stamp+=1000;
  if(index%20==0)s.land_stamp+=1000; // Genuine 1Hz detector
  return r.step(s,index*.05).attitude;
}
static void prepared_without_relaxing_arm_gate(){
  f::Runtime r(diag_config());auto s=snapshot();s.stationary_seconds=0;
  s.attitude.local_stamp=100000;s.attitude.vz=.2;
  start(r,s);
  int reached=-1;
  for(int i=0;i<85;++i){
    const auto out=sample_tick(r,s,i,2.+.009*std::sin(i*.1),.2);
    if(r.attitude_core().phase()==a::Phase::Prestream){
      reached=i;assert(out.publish&&out.thrust[2]==0.0&&out.command==a::Command::None);break;
    }
    assert(r.attitude_core().phase()==a::Phase::SelfCheck);
    assert(!out.publish&&out.command==a::Command::None);
  }
  assert(reached>=40); // Cannot forge 2s of detector observations
  assert(!r.request("TAKEOFF",{.6},s,reached*.05).accepted);
  assert(r.attitude_core().phase()==a::Phase::Prestream);
  std::cout<<"R62a_DISARMED_DIAGNOSTIC_PREP_CAN_PROGRESS_WITH_INCONSISTENT_VZ PASS\n";
}
static void strict_mode_must_still_block(){
  f::Runtime r(config());auto s=snapshot();s.stationary_seconds=0;
  s.attitude.local_stamp=100000;s.attitude.vz=.2;
  start(r,s);
  for(int i=0;i<90;++i){
    auto out=sample_tick(r,s,i,2.+.004*std::sin(i*.1),.2);
    assert(r.attitude_core().phase()==a::Phase::SelfCheck);
    assert(!out.publish);
  }
  std::cout<<"R62a_FLIGHT_GATES_ENABLED_RETAIN_STRICT_GROUND_VZ PASS\n";
}
static void altitude_jump_or_reset_blocks_preparation(){
  // At 1Hz land samples, an actual 0.2m Z change prevents the alternative
  // ground-proof route. An estimator reset also invalidates evidence.
  for(int scenario=0;scenario<2;++scenario){
    f::Runtime r(diag_config());auto s=snapshot();s.stationary_seconds=0;
    s.attitude.local_stamp=100000;start(r,s);
    for(int i=0;i<65;++i){
      if(i==30&&scenario==1)++s.attitude.z_reset;
      const auto o=sample_tick(r,s,i,i<30?2.:2.22,.2);
      assert(r.attitude_core().phase()==a::Phase::SelfCheck);
      assert(!o.publish);
    }
  }
  std::cout<<"R62a_VERTICAL_JUMP_AND_RESET_DISALLOW_PREP PASS\n";
}
static void frozen_land_stamps_cannot_create_dwell(){
  f::Runtime r(diag_config());auto s=snapshot();s.stationary_seconds=0;
  s.attitude.local_stamp=100000;start(r,s);
  for(int i=0;i<100;++i){
    s.attitude.local_stamp+=1000;s.status_stamp+=1000;
    s.attitude.z=2.;s.attitude.vz=.2;
    const auto out=r.step(s,i*.05).attitude;
    assert(r.attitude_core().phase()==a::Phase::SelfCheck);
    assert(!out.publish);
  }
  std::cout<<"R62a_FROZEN_LAND_DETECTOR_BLOCKS_PREP PASS\n";
}
int main(){
  prepared_without_relaxing_arm_gate();strict_mode_must_still_block();
  altitude_jump_or_reset_blocks_preparation();frozen_land_stamps_cannot_create_dwell();
  return 0;
}
