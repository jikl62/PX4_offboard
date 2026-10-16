#pragma once
// Offline audit probe; compile against the untouched R6.2c baseline.
#include "jikl_fc01_system/runtime.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <cassert>
#include <iostream>
namespace f=fc01;namespace a=fc01::attitude;
f::Config config(bool bench_mode=false){
  f::Config c;c.profile="attitude_altitude";auto& p=c.attitude;
  p.flight_actions_enabled=p.arm_enabled=p.takeoff_enabled=p.parameters_confirmed=true;
  p.rc_takeover_confirmed=p.autonomous_failsafe_confirmed=true;
  p.vertical={.5,.12,.75,.4,.5,.15,.03,.08,1.0,.3,.2};
  p.prestream_seconds=1.2;p.ground_seconds=2.;p.loop_hz=20;p.stall_seconds=.3;
  p.status_timeout=p.land_timeout=p.flags_timeout=2.5;p.local_timeout=p.attitude_timeout=.5;
  p.land_sample_max_gap=1.25;p.command_timeout=5.;p.console_timeout=3.;
  p.ramp_seconds=2.;p.settle_seconds=2.;p.landing_seconds=3.;
  p.height_tolerance=.15;p.speed_tolerance=.15;p.ascent_timeout=p.descent_timeout=10.;
  p.saturation_seconds=1.5;p.max_flight_seconds=30.;p.max_tilt_deg=10.;
  p.armed_dwell_seconds=1.;p.liftoff_dwell_seconds=.2;p.liftoff_height_m=.12;
  p.liftoff_speed_mps=.08;p.touchdown_dwell_seconds=.3;
  p.height_lower_margin_m=.25;p.z_jump_limit_m=.45;p.vz_jump_limit_mps=1.5;
  if(bench_mode){p.bench_arm_only_enabled=true;p.takeoff_enabled=false;}
  c.validate();return c;
}
struct Harness {
  f::Runtime r{config()};f::Snapshot s;double t=1.;
  static constexpr std::uint64_t epoch=1700000000000000ULL;
  std::uint64_t wire() const{return epoch+std::uint64_t(t*1e6);}
  Harness(bool bench_mode=false):r(config(bench_mode)){
    r.require_attitude_sink_commit();
    s.status_fresh=s.land_fresh=s.local_fresh=s.arming_state_known=true;
    s.status_stamp=s.land_stamp=wire()-200000;s.system=s.component=1;
    s.feedback.exclusive=s.feedback.landed=s.feedback.preflight=true;
    s.at_rest=true;s.stationary_seconds=3.;
    s.battery_fresh=s.battery_connected=s.power_valid=true;
    s.attitude.attitude_fresh=s.attitude.failsafe_fresh=s.attitude.z_valid=s.attitude.vz_valid=true;
    s.attitude.q={1,0,0,0};s.attitude.z=2.;s.attitude.vz=0.;s.attitude.battery_voltage=7.6;
    s.attitude.source_clock_validated=true; // production-source domain, verified separately below
    assert(r.request("INTERACTIVE",{},s,t).accepted);
    assert(r.request("AUTHORITY_AUTONOMOUS",{},s,t).accepted);
    assert(r.request("ATTITUDE_READY",{},s,t).accepted);
  }
  a::Output tick(){
    t+=.05;s.status_stamp=s.land_stamp=wire()-200000;
    s.attitude.local_stamp=wire()-200000;
    r.request("PING",{},s,t);auto o=r.step(s,t).attitude;
    if(o.command!=a::Command::None)r.confirm_attitude_command_publication(o.command,true,wire(),s,t);
    return o;
  }
  void ready(){
    for(int i=0;i<60&&r.attitude_core().phase()!=a::Phase::RequestOffboard;++i)tick();
    assert(r.attitude_core().phase()==a::Phase::RequestOffboard);
    r.ack({r.pending(176).wire_stamp+f::source_clock_skew_budget_us+1000,176,0,1,191,false},t+.01);
    assert(r.pending(176).accepted);
    s.feedback.offboard=true;s.nav_state=14;
    for(int i=0;i<12&&r.attitude_core().phase()!=a::Phase::OffboardReady;++i)tick();
    assert(r.attitude_core().phase()==a::Phase::OffboardReady);
  }
};
