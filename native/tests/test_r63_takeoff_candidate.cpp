// R6.3 normal-flight candidate regression (synthetic PX4 samples only).
// Reuse the original fully independent normal-flight runtime fixture.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main baseline_attitude_runtime_main_not_called
#include "test_attitude_runtime.cpp"
#undef main
#pragma GCC diagnostic pop
#include <algorithm>
#include <cmath>

static f::Config r63_config(){
  auto c=cfg(true);
  c.attitude.vertical.hover=.475;
  c.attitude.bench_arm_only_enabled=false;
  c.validate();
  return c;
}
static void r63_hover_mapping(){
  auto c=r63_config();
  assert(!c.attitude.bench_arm_only_enabled);
  a::VerticalController controller(c.attitude.vertical);
  controller.reset(.475);
  a::VerticalSample s{s.z0=2.,s.z=2.,s.vz=0.,s.fresh=true,s.z_valid=true,s.vz_valid=true};
  auto o=controller.step(s,0.,.05);
  assert(o.healthy&&std::abs(o.thrust-.475)<1e-12);
  // Body FRD/NED thrust in VehicleAttitudeSetpoint: command is (0,0,-0.475).
  std::cout<<"R63_HOVER_0P475_NORMALIZED_NEGATIVE_BODY_Z_PASS\n";
}
static void r63_flight_chain(){
  f::Runtime r(r63_config());auto s=sample();double t=ready(r,s,true);
  assert(r.request("INTERACTIVE",{},s,t).accepted);
  assert(r.request("PING",{},s,t).accepted);
  assert(r.request("TAKEOFF",{.6},s,t).accepted);
  auto o=tick(r,s,t+=.05);assert(o.attitude.command==a::Command::Arm);
  auto base=r.pending(400).baseline;
  r.ack({base+1,400,0,1,191,false},t+.01);
  s.feedback.armed=true;
  o=tick(r,s,t+=.05);
  assert(r.attitude_core().phase()==a::Phase::ArmedGround);
  for(int i=0;i<20&&r.attitude_core().phase()!=a::Phase::ThrustRamp;++i){
    assert(r.request("PING",{},s,t).accepted);
    o=tick(r,s,t+=.05);
    assert(o.attitude.publish&&o.attitude.thrust[2]==0);
  }
  assert(r.attitude_core().phase()==a::Phase::ThrustRamp);
  bool saw_ramp=false;
  for(int i=0;i<45;++i){
    assert(r.request("PING",{},s,t).accepted);
    o=tick(r,s,t+=.05);
    assert(o.attitude.publish&&o.attitude.command==a::Command::None);
    assert(o.attitude.thrust[2]<=0&&o.attitude.thrust[2]>=-.4750001);
    if(o.attitude.thrust[2]<-.01)saw_ramp=true;
    assert(r.attitude_core().phase()==a::Phase::ThrustRamp);
  }
  assert(saw_ramp); // Timer alone cannot prove liftoff.
  assert(std::abs(o.attitude.thrust[2]+.475)<1e-7);
  s.feedback.landed=false;s.at_rest=false;
  s.attitude.z=1.97;s.attitude.vz=-.02;
  for(int i=0;i<4;++i){
    assert(r.request("PING",{},s,t).accepted);
    tick(r,s,t+=.05);
    assert(r.attitude_core().phase()==a::Phase::ThrustRamp);
  }
  s.attitude.z=1.87;s.attitude.vz=-.1;
  for(int i=0;i<8&&r.attitude_core().phase()!=a::Phase::Ascend;++i){
    s.attitude.z-=.005; // new local samples observe the claimed 0.1 m/s rise
    assert(r.request("PING",{},s,t).accepted);
    o=tick(r,s,t+=.05);
  }
  assert(r.attitude_core().phase()==a::Phase::Ascend);
  bool got_hold=false;
  for(int i=0;i<125;++i){
    s.attitude.z=std::max(1.4,s.attitude.z-.02);
    s.attitude.vz=s.attitude.z>1.401?-.1:0.;
    assert(r.request("PING",{},s,t).accepted);
    o=tick(r,s,t+=.05);
    if(r.attitude_core().phase()==a::Phase::HeightHold){got_hold=true;break;}
    assert(o.attitude.publish);
  }
  assert(got_hold);
  assert(r.pending(400).sent);
  std::cout<<"R63_NORMAL_TAKEOFF_RAMP_ASCEND_HEIGHT_HOLD_PASS\n";
}
static void r63_no_liftoff_fallback(){
  f::Runtime r(r63_config());auto s=sample();double t=ready(r,s,true);
  assert(r.request("INTERACTIVE",{},s,t).accepted);
  assert(r.request("TAKEOFF",{.6},s,t).accepted);
  auto o=tick(r,s,t+=.05);assert(o.attitude.command==a::Command::Arm);
  auto base=r.pending(400).baseline;
  r.ack({base+1,400,0,1,191,false},t+.01);
  s.feedback.armed=true;
  tick(r,s,t+=.05);
  bool entered_ramp=false, saw_no_liftoff_recovery=false;
  for(int i=0;i<260;++i){
    assert(r.request("PING",{},s,t).accepted);
    o=tick(r,s,t+=.05);
    const auto p=r.attitude_core().phase();
    if(p==a::Phase::ThrustRamp)entered_ramp=true;
    assert(p!=a::Phase::Ascend&&p!=a::Phase::HeightHold);
    if(entered_ramp&&p==a::Phase::PoweredGround){
      saw_no_liftoff_recovery=true;break;
    }
    assert(o.attitude.publish);
  }
  assert(entered_ramp&&saw_no_liftoff_recovery);
  std::cout<<"R63_RAMP_TIMEOUT_NO_SYNTHETIC_LIFTOFF_PASS\n";
}
static void r63_bench_isolation_and_estimator_denial(){
  {auto c=r63_config();c.attitude.bench_arm_only_enabled=true;
    c.attitude.takeoff_enabled=false;c.validate();
    f::Runtime r(c);auto s=sample();double t=ready(r,s,true);
    assert(r.request("INTERACTIVE",{},s,t).accepted);
    auto v=r.request("TAKEOFF",{.6},s,t);
    assert(!v.accepted);assert(r.attitude_core().phase()==a::Phase::OffboardReady);
  }
  {f::Runtime r(r63_config());auto s=sample();double t=ready(r,s,true);
    assert(r.request("INTERACTIVE",{},s,t).accepted);
    assert(r.request("TAKEOFF",{.6},s,t).accepted);
    auto o=tick(r,s,t+=.05);assert(o.attitude.command==a::Command::Arm);
    auto base=r.pending(400).baseline;
    r.ack({base+1,400,0,1,191,false},t+.01);
    s.feedback.armed=true;
    tick(r,s,t+=.05);
    s.attitude.z_valid=false;
    o=tick(r,s,t+=.05);
    assert(!o.attitude.publish&&o.attitude.command==a::Command::None);
    assert(r.attitude_core().phase()==a::Phase::Safety);
  }
  std::cout<<"R63_BENCH_ISOLATION_AND_INVALID_ESTIMATOR_BLOCK_PASS\n";
}
int main(){
  r63_hover_mapping();
  r63_flight_chain();
  r63_no_liftoff_fallback();
  r63_bench_isolation_and_estimator_denial();
  std::cout<<"R63_NORMAL_TAKEOFF_CANDIDATE_REGRESSION_PASS\n";
}
