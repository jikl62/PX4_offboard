#include "authority_test_support.hpp"
#include <iostream>
#include <stdexcept>
using namespace authority_test;
static void require_rc(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
static a::Config rc_admission_config(){
  auto c=config().attitude;c.rc_navigation_enabled=true;c.arm_enabled=false;
  c.rc_takeover_confirmed=true;c.max_flight_seconds=30;c.max_tilt_deg=10;
  c.validate();return c;
}
static a::Feedback safe_rc_feedback(){
  a::Feedback s;
  s.attitude_fresh=s.local_fresh=s.status_fresh=s.land_fresh=s.failsafe_fresh=true;
  s.z_valid=s.vz_valid=s.landed=s.at_rest=s.exclusive=true;
  s.source_clock_validated=true;s.source_host_stamp=1000000;
  s.rc_link_ready=s.rc_switches_ready=s.rc_source_selected=true;
  s.rc_source_clock_valid=s.rc_switch_clock_valid=true;
  s.rc_axes_safe=s.rc_aux_switches_safe=true;
  s.rc_arm_switch=s.rc_kill_switch=3;s.rc_mode_slot=1;s.stabilized_selected=true;
  s.battery_ready=s.preflight=true;s.stationary_seconds=3;
  s.status_stamp=s.land_stamp=s.local_stamp=s.rc_switch_sample_stamp=1000000;
  return s;
}
static void preaccumulated_ground_rest_does_not_wait_twice(){
  a::Core core(rc_admission_config());auto s=safe_rc_feedback();
  const auto out=core.step(s,.05,{},{});
  require_rc(core.phase()==a::Phase::RcPrestream&&out.publish,
    "already proven stationary ground must begin RC prestream without a second two-second rest wait");
}
static void fatal_faults_cannot_be_cleared_by_ground_controls(){
  for(const char* reason:{"ACTIVE_VERTICAL_ESTIMATOR_RESET","TELEMETRY_TIME_REGRESSION",
      "ROS_ADAPTER_CONTROL_LOOP_STALL","PERSISTENT_THRUST_SATURATION",
      "RC_ARMED_ESTIMATOR_RESET","PUBLISHER_OR_TELEMETRY_CONFLICT"}){
    a::Core core(rc_admission_config());core.block(reason);
    require_rc(!core.rc_restart_allowed(safe_rc_feedback()),
      "returning RC controls to safe ground must not erase a fatal or general fault");
    core.block("RC_MANUAL_ARM_EVIDENCE_TIMEOUT");
    require_rc(!core.rc_restart_allowed(safe_rc_feedback()),
      "a later recoverable-looking reason cannot overwrite an earlier permanent fault lock");
  }
}
struct RetryFixture {
  a::Core core{rc_admission_config()};a::Feedback s{safe_rc_feedback()};double now{};
  RetryFixture(){
    for(int i=0;i<40;++i)advance();
    require_rc(core.phase()==a::Phase::RcWaitArm,"actual committed prestream reaches readiness");
    s.rc_arm_switch=1;++s.operator_switch_generation;
    for(int i=0;i<60;++i)advance();
    require_rc(core.phase()==a::Phase::Safety&&core.reason()=="RC_MANUAL_ARM_EVIDENCE_TIMEOUT",
      "missing complementary armed status reaches the actual bounded preparation timeout");
    s.rc_arm_switch=3;
  }
  void advance(){
    now+=.05;s.status_stamp+=50000;s.land_stamp+=50000;s.local_stamp+=50000;
    s.vertical_sample_stamp+=50000;s.rc_switch_sample_stamp+=50000;s.source_host_stamp+=50000;
    const auto out=core.step(s,now,{},{});if(out.publish)core.note_rc_publication(true,now);
  }
};
static void known_preparation_timeouts_can_reprepare_only_on_safe_ground(){
  RetryFixture q;
  require_rc(q.core.rc_restart_allowed(q.s),"a known preparation timeout can restart after fresh safe ground and switch reset");
  for(int invalid=0;invalid<4;++invalid){auto s=q.s;
    if(invalid==0)s.rc_arm_switch=1;
    if(invalid==1)s.time_regressed=true;
    if(invalid==2)s.rc_mode_slot=7;
    if(invalid==3)s.source_host_stamp=0;
    require_rc(!q.core.rc_restart_allowed(s),"reprepare rejects unsafe switches, clocks and invalid RC mode slots");
  }
}
static void invalid_rc_mode_slot_cannot_begin_prestream(){
  a::Core core(rc_admission_config());auto s=safe_rc_feedback();s.rc_mode_slot=7;
  require_rc(!core.step(s,.05,{},{}).publish,"invalid RC mode slot cannot begin a new prepared stream");
}
int main(int argc,char** argv){
  const std::string which=argc>1?argv[1]:"all";
  if(which=="all"||which=="wait")preaccumulated_ground_rest_does_not_wait_twice();
  if(which=="all"||which=="fatal")fatal_faults_cannot_be_cleared_by_ground_controls();
  if(which=="all"||which=="timeout")known_preparation_timeouts_can_reprepare_only_on_safe_ground();
  if(which=="all"||which=="slot")invalid_rc_mode_slot_cannot_begin_prestream();
  std::cout<<"RC_REPREPARE_ADMISSION_PASS "<<which<<'\n';
}
