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
    check(e.healthy&&!e.suspect&&!e.rate_warning&&e.reason.empty(),
      "consistent accelerating climb/descent must remain healthy without a rate warning");
  }
}
static void persistent_rate_error_is_diagnostic(){
  auto g=monitor();std::uint64_t stamp=1000000;
  for(int i=0;i<24;++i){
    const auto& e=g.observe(stamp+=50000,0,-.4,0,.5);
    check(e.healthy&&e.suspect&&e.reason.empty(),
      "persistent vz/z_deriv disagreement is diagnostic and cannot create a hard fault");
    check(std::abs(e.rate_error_mps-.4)<1e-12&&e.disagreement_samples==std::uint32_t(i+1),
      "rate diagnostics retain the measured difference and every new disagreeing sample");
    check(std::abs(e.disagreement_seconds-i*.05)<1e-12&&e.rate_warning==(i>=10),
      "the rate warning begins only after the configured continuous dwell");
  }
  const auto& recovered=g.observe(stamp+=50000,0,0,0,.5);
  check(recovered.healthy&&!recovered.suspect&&!recovered.rate_warning&&recovered.reason.empty()&&
      recovered.rate_error_mps==0&&recovered.disagreement_samples==0&&recovered.disagreement_seconds==0,
    "a new agreeing sample clears rate diagnostics without resetting the monitor");
  const auto& renewed=g.observe(stamp+50000,0,-.4,0,.5);
  check(renewed.healthy&&renewed.suspect&&!renewed.rate_warning&&
      renewed.disagreement_samples==1&&renewed.disagreement_seconds==0,
    "a new disagreement cannot reuse an earlier warning dwell");
  auto sparse=monitor();sparse.observe(1000000,0,-.4,0,1.);
  check(!sparse.observe(1600000,0,-.4,0,1.).rate_warning,
    "elapsed time alone cannot replace the three-sample warning requirement");
  check(sparse.observe(1650000,0,-.4,0,1.).rate_warning,
    "three independent samples and the configured dwell may report a warning");
}
static void transient_and_cached(){
  auto g=monitor();std::uint64_t stamp=1000000;
  for(int i=0;i<40;++i){
    const double vz=(i>=2&&i<=5)?-.4:0.;
    const auto& e=g.observe(stamp+=50000,0,vz,0,.5);
    check(e.healthy&&!e.rate_warning,"short rate bursts cannot create a fault or persistent warning");
  }
  auto cached=monitor();const auto first=cached.observe(1000000,0,-.4,0,.5);
  for(int i=0;i<100;++i){
    const auto& e=cached.observe(1000000,0,-.4,0,.5);
    check(e.healthy&&!e.rate_warning&&e.disagreement_samples==first.disagreement_samples&&e.disagreement_seconds==0,
      "cached sample must not accumulate duration or sample evidence");
  }
  for(int i=1;i<=10;++i)cached.observe(1000000+std::uint64_t(i)*50000,0,-.4,0,.5);
  const auto warned=cached.evidence();check(warned.rate_warning,"new samples establish a persistent warning");
  for(int i=0;i<100;++i){
    const auto& e=cached.observe(warned.sample_stamp,0,-.4,0,.5);
    check(e.healthy&&e.rate_warning&&e.disagreement_samples==warned.disagreement_samples&&
        e.disagreement_seconds==warned.disagreement_seconds,
      "cached samples also leave an established warning duration and count unchanged");
  }
}
static void coherent_rates_wrong_displacement(){
  auto g=monitor();std::uint64_t stamp=1000000;bool stopped=false;
  for(int i=0;i<15;++i){
    const auto& e=g.observe(stamp+=50000,0,-.4,-.4,.5);
    if(!e.healthy){stopped=true;check(e.reason=="VERTICAL_POSITION_RATE_INCONSISTENT","position/rate fault must be explicit");break;}
  }
  check(stopped,"matching rates cannot hide a contradictory position history");
  const auto& latched=g.observe(stamp+50000,0,0,0,.5);
  check(!latched.healthy&&latched.reason=="VERTICAL_POSITION_RATE_INCONSISTENT",
    "agreeing feedback cannot clear a hard position-consistency fault in flight");
  g.reset();check(g.observe(stamp+50000,0,0,0,.5).healthy,
    "an explicit monitor reset clears a hard fault for a new task");
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
  for(int k=0;k<8;++k){
    auto g=monitor();g.observe(1000000,0,0,0,.5);
    const auto& e=k==0?g.observe(1050000,0,0,NAN,.5):
      k==1?g.observe(999999,0,0,0,.5):
      k==2?g.observe(1600000,0,0,0,.5):
      k==3?g.observe(0,0,0,0,.5):
      k==4?g.observe(1050000,NAN,0,0,.5):
      k==5?g.observe(1050000,0,NAN,0,.5):
      k==6?g.observe(1050000,0,0,0,0):g.observe(1050000,0,0,0,NAN);
    const char* reason=k==1?"VERTICAL_FEEDBACK_TIME_REGRESSION":
      k==2?"VERTICAL_FEEDBACK_SAMPLE_GAP":"VERTICAL_FEEDBACK_INPUT_INVALID";
    check(!e.healthy&&e.reason==reason,"nonfinite/missing/regressed/gapped data retain their hard fault reason");
  }
  for(double v:{std::numeric_limits<double>::quiet_NaN(),0.,-1.,1.1}){
    a::VerticalFeedbackConfig c;c.rate_tolerance_mps=v;bool refused=false;
    try{c.validate();}catch(const std::invalid_argument&){refused=true;}
    check(refused,"invalid guard thresholds must be rejected");
  }
}
static void explicit_integral_freeze_preserves_braking(){
  a::VerticalConfig c;c.hover=.475;c.minimum=.12;c.maximum=.75;c.slew=.4;
  c.height_gain=.5;c.velocity_gain=.15;c.integral_gain=.03;c.integral_limit=.08;
  c.max_height=5;c.max_climb=.3;c.max_descent=.2;
  a::VerticalController controller(c);controller.reset(.475);controller.freeze_integral(true);
  double prior=.475;
  for(int i=0;i<20;++i){
    auto o=controller.step({0,-.5,-.4,true,true,true,0,0},.5,.05);
    check(o.healthy&&std::abs(o.thrust-prior)<=.02+1e-12,"freeze must preserve physical slew limit");
    check(controller.integral()==0,"an explicit freeze request prevents integral accumulation");
    check(o.thrust<.475,"braking must remain available below hover; no thrust floor");prior=o.thrust;
  }
  controller.freeze_integral(false);controller.step({0,-.5,-.4,true,true,true,0,0},.5,.05);
  check(controller.integral()<0,"releasing an explicit freeze resumes normal integration");
}
int main(){
  nominal_motion();persistent_rate_error_is_diagnostic();transient_and_cached();
  coherent_rates_wrong_displacement();invalid_inputs();explicit_integral_freeze_preserves_braking();
  position_check_uses_position_derivative();
  std::cout<<"VERTICAL_FEEDBACK_GUARD_PASS (7 groups)\n";
}
