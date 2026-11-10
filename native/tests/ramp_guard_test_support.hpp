#pragma once
// Real Runtime state machine, synthetic feedback only. No ROS/PX4 output.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main baseline_attitude_runtime_main_not_called
#include "test_attitude_runtime.cpp"
#undef main
#pragma GCC diagnostic pop
#include <cstdlib>
#include <string>

static void require_guard(bool ok,const char* message){
  if(!ok){std::cerr<<"RAMP_GUARD_FAIL: "<<message<<'\n';std::exit(1);}
}
static f::Config guard_config(){
  auto c=cfg(true);auto& p=c.attitude;
  p.vertical.hover=.50;p.vertical.minimum=.12;p.vertical.maximum=.80;p.vertical.slew=.4;
  p.vertical.height_gain=.5;p.vertical.velocity_gain=.15;p.vertical.integral_gain=.03;
  p.vertical.integral_limit=.08;p.vertical.max_climb=.3;p.vertical.max_descent=.2;
  p.armed_dwell_seconds=1.;p.liftoff_dwell_seconds=.2;
  p.height_tolerance=.15;p.speed_tolerance=.15;p.settle_seconds=2.;
  p.z_jump_limit_m=.45;p.vz_jump_limit_mps=1.5;
  p.ramp_seconds=2.;p.takeoff_reserve_thrust=.04;
  p.land_sample_max_gap=1.25;p.max_tilt_deg=10.;p.max_flight_seconds=30.;
  c.validate();return c;
}
struct GuardFlight {
  f::Config c{guard_config()};f::Runtime r{c};f::Snapshot s{sample()};
  double t{},ramp_start{};f::Output o;
  std::uint64_t land_elapsed_us{},local_elapsed_us{};
  explicit GuardFlight(double target=.5,f::Config config=guard_config()):c(config),r(c){
    t=ready(r,s,true);
    require_guard(r.request("INTERACTIVE",{},s,t).accepted,"interactive admission");
    require_guard(r.request("TAKEOFF",{target},s,t).accepted,"takeoff admission");
    advance();require_guard(o.attitude.command==a::Command::Arm,"normal Arm transaction");
    r.ack({r.pending(400).baseline+1,400,0,1,191,false},t+.01);
    s.feedback.armed=true;
    for(int i=0;i<60&&r.attitude_core().phase()!=a::Phase::ThrustRamp;++i)advance();
    require_guard(r.attitude_core().phase()==a::Phase::ThrustRamp,"ground dwell then ramp");
    ramp_start=t;
  }
  void advance(double dt=.05,bool new_land=true,bool new_local=true){
    require_guard(r.request("PING",{},s,t).accepted,"console lease");
    const auto increment=static_cast<std::uint64_t>(std::llround(dt*1e6));
    s.status_stamp+=increment;land_elapsed_us+=increment;local_elapsed_us+=increment;
    if(new_land){s.land_stamp+=land_elapsed_us;land_elapsed_us=0;}
    if(new_local){
      s.attitude.local_stamp+=local_elapsed_us;s.attitude.vertical_sample_stamp+=local_elapsed_us;
      local_elapsed_us=0;
    }
    o=r.step(s,t+=dt);
  }
  void observe(double height,double up_speed){s.attitude.z=2.-height;s.attitude.vz=-up_speed;s.attitude.z_deriv=-up_speed;}
  double thrust()const{return -o.attitude.thrust[2];}
  void ground_ramp(double seconds=4.2){
    for(int i=0;i<int(seconds/.05);++i)advance();
    require_guard(r.attitude_core().phase()==a::Phase::ThrustRamp,"ground time cannot prove liftoff");
  }
};
