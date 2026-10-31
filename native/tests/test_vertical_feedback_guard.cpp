#include "jikl_fc01_system/vertical_feedback_guard.hpp"
#include "jikl_fc01_system/vertical_controller.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>
namespace a=fc01::attitude;
static void check(bool ok,const char* message){
  if(!ok){std::cerr<<"FEEDBACK_GUARD_FAIL: "<<message<<'\n';std::exit(1);}
}
static a::VerticalFeedbackGuard monitor(){a::VerticalFeedbackConfig c;c.enabled=true;return a::VerticalFeedbackGuard(c);}
static void nominal_motion(){
  auto g=monitor();double z=0,v=0;std::uint64_t stamp=1000000;
  for(int i=0;i<240;++i){
    const double next=.3*std::sin(i*.05);z+=(v+next)*.5*.05;v=next;
    const auto& e=g.observe(stamp+=50000,z,v,v,.5);
    check(e.healthy&&!e.suspect,"consistent accelerating climb/descent must remain healthy");
  }
}
static void persistent_rate_error(){
  auto g=monitor();std::uint64_t stamp=1000000;int stopped=0;
  for(int i=0;i<20;++i){
    const auto& e=g.observe(stamp+=50000,0,-.4,0,.5);
    if(!e.healthy){stopped=i;check(e.reason=="VERTICAL_VELOCITY_INCONSISTENT","rate fault must be explicit");break;}
  }
  check(stopped>=10&&stopped<=11,"persistent mismatch must fault after configured dwell");
  check(!g.observe(stamp+50000,0,0,0,.5).healthy,"fault must latch despite recovered data");
  g.reset();check(g.observe(stamp+50000,0,0,0,.5).healthy,"explicit reset clears monitor on a new flight");
}
static void transient_and_cached(){
  auto g=monitor();std::uint64_t stamp=1000000;
  for(int i=0;i<40;++i){
    const double vz=(i>=2&&i<=5)?-.4:0.;
    check(g.observe(stamp+=50000,0,vz,0,.5).healthy,"short rate burst must not trip guard");
  }
  auto cached=monitor();const auto first=cached.observe(1000000,0,-.4,0,.5);
  for(int i=0;i<100;++i){
    const auto& e=cached.observe(1000000,0,-.4,0,.5);
    check(e.healthy&&e.disagreement_samples==first.disagreement_samples&&e.disagreement_seconds==0,
      "cached sample must not accumulate duration or sample evidence");
  }
}
static void coherent_rates_wrong_displacement(){
  auto g=monitor();std::uint64_t stamp=1000000;bool stopped=false;
  for(int i=0;i<15;++i){
    const auto& e=g.observe(stamp+=50000,0,-.4,-.4,.5);
    if(!e.healthy){stopped=true;check(e.reason=="VERTICAL_POSITION_RATE_INCONSISTENT","position/rate fault must be explicit");break;}
  }
  check(stopped,"matching rates cannot hide a contradictory position history");
}
static void position_check_uses_position_derivative(){
  auto g=monitor();std::uint64_t stamp=1000000;
  // PX4 vz and z_deriv have distinct meanings. A subthreshold difference
  // belongs to the rate comparison, not to the position-derivative residual.
  for(int i=0;i<20;++i){
    const auto& e=g.observe(stamp+=50000,.25*i*.05,.06,.25,.5);
    check(e.healthy,"subthreshold rate difference must not stop coherent position motion");
    if(e.position_window_checked)
      check(e.position_rate_error_mps<1e-10,"coherent z and z_deriv must have zero displacement residual");
  }
}
static void invalid_inputs(){
  for(int k=0;k<4;++k){
    auto g=monitor();g.observe(1000000,0,0,0,.5);
    const auto& e=k==0?g.observe(1050000,0,0,NAN,.5):
      k==1?g.observe(999999,0,0,0,.5):
      k==2?g.observe(1600000,0,0,0,.5):g.observe(0,0,0,0,.5);
    check(!e.healthy,"nonfinite/missing/regressed/gapped data must fail closed");
  }
  for(double v:{std::numeric_limits<double>::quiet_NaN(),0.,-1.,1.1}){
    a::VerticalFeedbackConfig c;c.rate_tolerance_mps=v;bool refused=false;
    try{c.validate();}catch(const std::invalid_argument&){refused=true;}
    check(refused,"invalid guard thresholds must be rejected");
  }
}
static void freeze_integral_preserves_braking(){
  a::VerticalConfig c;c.hover=.475;c.minimum=.12;c.maximum=.75;c.slew=.4;
  c.height_gain=.5;c.velocity_gain=.15;c.integral_gain=.03;c.integral_limit=.08;
  c.max_height=5;c.max_climb=.3;c.max_descent=.2;
  a::VerticalController controller(c);controller.reset(.475);controller.freeze_integral(true);
  double prior=.475;
  for(int i=0;i<20;++i){
    auto o=controller.step({0,-.5,-.4,true,true,true,0,0},.5,.05);
    check(o.healthy&&std::abs(o.thrust-prior)<=.02+1e-12,"freeze must preserve physical slew limit");
    check(controller.integral()==0,"suspect feedback must not accumulate negative integral");
    check(o.thrust<.475,"braking must remain available below hover; no thrust floor");prior=o.thrust;
  }
  controller.freeze_integral(false);controller.step({0,-.5,-.4,true,true,true,0,0},.5,.05);
  check(controller.integral()<0,"consistent feedback may resume normal integration");
}
int main(){
  nominal_motion();persistent_rate_error();transient_and_cached();
  coherent_rates_wrong_displacement();invalid_inputs();freeze_integral_preserves_braking();
  position_check_uses_position_derivative();
  std::cout<<"VERTICAL_FEEDBACK_GUARD_PASS (7 groups)\n";
}
