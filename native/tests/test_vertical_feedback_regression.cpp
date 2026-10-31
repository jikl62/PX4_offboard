// Compiles against both the archived baseline and this candidate. The same
// behavioural assertion must fail on the baseline, then pass on the candidate.
#include "ramp_guard_test_support.hpp"
#if __has_include("jikl_fc01_system/vertical_feedback_guard.hpp")
#define HAS_VERTICAL_FEEDBACK_GUARD 1
#endif
static f::Config feedback_config(){
  auto c=guard_config();
#ifdef HAS_VERTICAL_FEEDBACK_GUARD
  c.attitude.feedback_guard.enabled=true;
#endif
  return c;
}
static void sustained_contradiction(){
  f::Runtime r(feedback_config());auto s=sample();double t=ready(r,s,true);
  require_guard(r.request("INTERACTIVE",{},s,t).accepted,"interactive admission");
  require_guard(r.request("TAKEOFF",{.5},s,t).accepted,"takeoff admitted");
  f::Output o;
  for(int i=0;i<100;++i){
    r.request("PING",{},s,t);s.status_stamp+=50000;s.land_stamp+=50000;
    s.attitude.local_stamp+=50000;
#ifdef HAS_VERTICAL_FEEDBACK_GUARD
    s.attitude.z_deriv=0;
    s.attitude.vertical_sample_stamp=s.attitude.local_stamp;
#endif
    o=r.step(s,t+=.05);
    if(o.attitude.command==a::Command::Arm){
      r.ack({r.pending(400).baseline+1,400,0,1,191,false},t+.001);s.feedback.armed=true;
    }
    if(r.attitude_core().phase()==a::Phase::ThrustRamp)break;
  }
  require_guard(r.attitude_core().phase()==a::Phase::ThrustRamp,"ramp reached");
  // Apparent fast climb velocity, but no position change for 0.8 seconds.
  // Below the old jump/envelope limits; baseline keeps accepting the data.
  s.attitude.vz=-.4;
  for(int i=0;i<16 && r.attitude_core().active();++i){
    r.request("PING",{},s,t);s.status_stamp+=50000;s.land_stamp+=50000;
    s.attitude.local_stamp+=50000;
#ifdef HAS_VERTICAL_FEEDBACK_GUARD
    s.attitude.vertical_sample_stamp=s.attitude.local_stamp;
#endif
    o=r.step(s,t+=.05);
  }
  require_guard(r.attitude_core().phase()==a::Phase::Safety,
    "sustained contradictory vertical feedback must stop autonomous control");
  require_guard(!o.attitude.publish,"fault must suppress publication, never publish a zero-thrust packet");
  require_guard(!r.request("TAKEOFF",{.5},s,t).accepted,"no rearm from a feedback fault");
}
#ifdef HAS_VERTICAL_FEEDBACK_GUARD
static void raw_samples_not_envelopes(){
  GuardFlight q(.5,feedback_config());q.s.attitude.z_deriv=0;
  q.s.attitude.vz=-.4;
  for(int i=0;i<6;++i)q.advance();
  const auto duration=q.r.attitude_core().vertical_feedback_evidence().disagreement_seconds;
  for(int i=0;i<6;++i){
    q.s.attitude.local_stamp+=50000; // new envelope, repeated raw estimate
    q.advance(.05,true,false);
    require_guard(q.r.attitude_core().active(),"repeated raw estimate must not falsely complete a disagreement dwell");
  }
  require_guard(q.r.attitude_core().vertical_feedback_evidence().disagreement_seconds==duration,
    "only strictly new raw samples supply disagreement duration");
}
static void guarded_lifecycle(){
  GuardFlight q(.5,feedback_config());q.s.attitude.z_deriv=0;q.ground_ramp();
  q.s.feedback.landed=false;q.s.at_rest=false;
  for(int i=1;i<=50;++i){q.observe(i*.01,.2);q.advance();}
  q.observe(.5,0);
  for(int i=0;i<55;++i)q.advance();
  require_guard(q.r.attitude_core().phase()==a::Phase::HeightHold,"enabled guard must allow normal height hold");
  require_guard(q.r.request("LAND",{},q.s,q.t).accepted,"guarded landing admitted");
  for(int i=49;i>=0;--i){q.observe(i*.01,i?-.2:0);q.advance();}
  q.s.feedback.landed=q.s.at_rest=true;q.s.stationary_seconds=3;
  bool disarmed=false;
  for(int i=0;i<250&&q.r.attitude_core().active();++i){
    q.observe(0,0);q.advance();
    if(q.o.attitude.command==a::Command::Disarm){
      q.r.ack({q.r.pending(400).baseline+1,400,0,1,191,false},q.t+.001);
      q.s.feedback.armed=false;disarmed=true;
    }
    if(q.o.attitude.command==a::Command::ReleaseOffboard){
      q.r.ack({q.r.pending(176).baseline+1,176,0,1,191,false},q.t+.001);
      q.s.feedback.offboard=false;q.s.nav_state=15;q.s.stabilized_selectable=true;
    }
  }
  require_guard(disarmed&&!q.s.feedback.armed,"enabled guard must allow ordinary disarm");
  require_guard(q.r.attitude_core().phase()!=a::Phase::Safety,"normal guarded lifecycle must not fault");
  require_guard(!q.r.attitude_core().active(),"normal guarded lifecycle must finish release");
}
static void bad_raw_clock_fails_closed(){
  for(int k=0;k<2;++k){
    GuardFlight q(.5,feedback_config());q.s.attitude.z_deriv=0;q.advance();
    if(k==0)q.s.attitude.vertical_sample_stamp=0;
    else --q.s.attitude.vertical_sample_stamp;
    q.advance(.05,true,false);
    require_guard(q.r.attitude_core().phase()==a::Phase::Safety&&!q.o.attitude.publish,
      "missing/regressed raw clock must fail closed without envelope fallback");
  }
}
#endif
int main(int argc,char** argv){
#ifdef HAS_VERTICAL_FEEDBACK_GUARD
  if(argc>1&&std::string(argv[1])=="raw-clock"){raw_samples_not_envelopes();return 0;}
#else
  (void)argc;(void)argv;
#endif
  sustained_contradiction();
#ifdef HAS_VERTICAL_FEEDBACK_GUARD
  raw_samples_not_envelopes();
  guarded_lifecycle();bad_raw_clock_fails_closed();
#endif
  std::cout<<"VERTICAL_FEEDBACK_REGRESSION_PASS\n";
}
