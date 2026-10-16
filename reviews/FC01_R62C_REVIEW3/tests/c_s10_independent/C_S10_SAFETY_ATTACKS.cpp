// Agent C: new independent, positive safety properties and reproducible failures.
// S10 source under test is READ-ONLY; no ROS/PX4/FC01 connection and no real flight.
#include "jikl_fc01_system/runtime.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
namespace f=fc01;namespace a=fc01::attitude;
struct Mode {std::uint64_t timestamp{};bool attitude{},position{},velocity{};};
struct Target {std::uint64_t timestamp{};float yaw_sp_move_rate{};std::array<float,4> q_d{};std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};};
static f::Config config(){
  f::Config c;c.profile="attitude_altitude";auto& p=c.attitude;
  p.flight_actions_enabled=p.arm_enabled=p.takeoff_enabled=p.parameters_confirmed=true;
  p.rc_takeover_confirmed=true;p.autonomous_failsafe_confirmed=true;
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
  c.validate();return c;
}
static f::Snapshot snapshot(){
  f::Snapshot s{};s.status_fresh=s.land_fresh=s.local_fresh=s.arming_state_known=true;
  s.status_stamp=s.land_stamp=10000;s.system=s.component=1;
  s.feedback.exclusive=s.feedback.landed=s.feedback.preflight=true;
  s.at_rest=true;s.stationary_seconds=3.;
  s.battery_fresh=s.battery_connected=s.power_valid=true;s.battery_warning=0;
  s.attitude.attitude_fresh=s.attitude.failsafe_fresh=s.attitude.z_valid=s.attitude.vz_valid=true;
  s.attitude.q={1,0,0,0};s.attitude.z=2.;s.attitude.vz=0.;s.attitude.battery_voltage=7.6;
  return s;
}
struct Harness{
  f::Runtime r{config()};f::Snapshot s{snapshot()};double t=0.;
  bool connected=true,one_hz=false;
  unsigned long ticks=0,attitude_sinks=0,mode_sinks=0,arms=0,disarms=0,mode_commands=0;
  std::string first_bad;
  Harness(){r.require_attitude_sink_commit();
    assert(r.request("INTERACTIVE",{},s,t).accepted);
    assert(r.request("AUTHORITY_AUTONOMOUS",{},s,t).accepted);
    assert(r.request("ATTITUDE_READY",{},s,t).accepted);
  }
  a::Output tick(bool advance_land=true,bool advance_status=true,bool do_ping=true){
    t+=.05;++ticks;
    if(advance_status)s.status_stamp+=1000;
    if(advance_land&&(!one_hz||ticks%20==0))s.land_stamp+=1000;
    if(do_ping&&connected)r.request("PING",{},s,t);
    a::Output o=r.step(s,t).attitude;
    if(o.publish){
      auto context=r.publication_context(s,t,connected,true);
      const auto d=a::publish_attitude_at_boundary<Mode,Target>(context,o,1000000+std::uint64_t(t*1e6),
        [&](const Target&){++attitude_sinks;},[&](const Mode&){++mode_sinks;});
      if(!d.allowed){if(first_bad.empty())first_bad=a::name(d.reason);}
      assert(d.allowed);assert(mode_sinks==attitude_sinks);
    }
    if(o.command!=a::Command::None){
      if(o.command==a::Command::Arm||o.command==a::Command::Disarm){
        auto ctx=r.publication_context(s,t,connected,true);
        const auto d=o.command==a::Command::Arm?
          a::publish_arm_at_boundary(ctx,[&]{++arms;}):a::publish_disarm_at_boundary(ctx,[&]{++disarms;});
        assert(d.allowed);
      }else{++mode_commands;
        if(o.command==a::Command::ReleaseOffboard)
          assert(a::publish_release_at_boundary(r.publication_context(s,t,connected,true),[]{}).allowed);
      }
      r.confirm_attitude_command_publication(o.command,true,1000000+std::uint64_t(t*1e6),s,t);
    }
    return o;
  }
  void ready(){
    for(int i=0;i<60&&r.attitude_core().phase()!=a::Phase::RequestOffboard;++i)tick();
    assert(r.attitude_core().phase()==a::Phase::RequestOffboard);
    assert(mode_commands==1);
    r.ack({r.pending(176).baseline+3000,176,0,1,191,false},t+.001);
    s.feedback.offboard=true;s.nav_state=14;
    tick();assert(r.attitude_core().phase()==a::Phase::OffboardReady);
  }
  void armed(){
    ready();assert(r.request("TAKEOFF",{.6},s,t).accepted);
    auto o=tick();assert(o.command==a::Command::Arm);
    assert(arms==1);r.ack({r.pending(400).baseline+3000,400,0,1,191,false},t+.001);
    s.feedback.armed=true;
    tick();assert(r.attitude_core().phase()==a::Phase::ArmedGround);
  }
  void ramp(){
    armed();one_hz=true;
    for(int i=0;i<80&&r.attitude_core().phase()!=a::Phase::ThrustRamp;++i)tick();
    assert(r.attitude_core().phase()==a::Phase::ThrustRamp);
  }
};
static void timeout_armed_offboard(){
  Harness h;h.ramp();
  // Synthetic PX4 Takeoff detected (landed=false) without validated liftoff height/vz.
  // This deliberately models an unconfirmed airborne state, not actual ULog dynamics.
  h.s.feedback.landed=false;h.s.at_rest=false;h.s.attitude.z=1.97;h.s.attitude.vz=-.02;
  bool saw_descent=false;int first_descent=-1,first_safety=-1;
  for(int i=0;i<460;++i){
    auto o=h.tick();
    if(h.r.attitude_core().phase()==a::Phase::Descent&&!saw_descent){saw_descent=true;first_descent=int(h.t*100);}
    if(h.r.attitude_core().phase()==a::Phase::Safety){first_safety=int(h.t*100);
      assert(!o.publish&&h.s.feedback.armed&&h.s.feedback.offboard);
      const auto a=h.attitude_sinks,m=h.mode_sinks;
      for(int j=0;j<30;++j){auto next=h.tick();assert(!next.publish);}
      assert(h.attitude_sinks==a&&h.mode_sinks==m);
      break;
    }
  }
  assert(saw_descent&&first_safety>=0);assert(h.arms==1&&h.disarms==0);
  assert(h.r.attitude_core().reason()=="DESCENT_TIMEOUT");
  std::cout<<"C-S10-01 EXPECTED COUNTEREXAMPLE / SAFETY PROPERTY FAIL: "
    <<"SITL_like_takeoff_without_core_liftoff=1; entered_DESCENT=1; "
    <<"DESCENT_TIMEOUT=1; armed=1; offboard=1; heartbeat_sinks_after_fault=0; "
    <<"arm_final_sinks="<<h.arms<<" disarm_final_sinks="<<h.disarms<<"; descent_at_cs="<<first_descent<<" safety_at_cs="<<first_safety<<"\n";
}
static void estimator_loss_live(){
  Harness h;h.ramp();h.s.attitude.z_valid=false;
  auto o=h.tick();assert(!o.publish);assert(h.r.attitude_core().phase()==a::Phase::Safety);
  assert(h.s.feedback.armed&&h.s.feedback.offboard&&h.disarms==0);
  std::cout<<"C-S10-02 EXPECTED COUNTEREXAMPLE / CONTRACT FAIL: estimator_z_invalid -> Safety, no publication, armed+offboard remains; explicit independent PX4 fallback proof REQUIRED\n";
}
static void observed_mode_exit(){
  Harness h;h.ramp();h.s.feedback.offboard=false;h.s.nav_state=2;
  auto o=h.tick();assert(!o.publish);assert(h.r.attitude_core().phase()==a::Phase::Safety);
  assert(h.arms==1&&h.disarms==0);
  std::cout<<"C-S10-03 OBSERVED MODE EXIT PASS: companion does not pretend to retain Offboard, no blind Disarm; PX4 Position control quality UNKNOWN\n";
}
static void no_xy_proof(){
  Harness h;h.ramp();h.s.feedback.valid=false;h.s.dead_reckoning=true;
  // Real node's attitude_snapshot deliberately does not depend on XY validity.
  auto out=h.tick();assert(out.publish&&h.s.feedback.armed&&h.s.feedback.offboard);
  std::cout<<"C-S10-04 CONTRACT GAP CONFIRMED: attitude Offboard output allowed with XY INVALID and dead_reckoning true; Position fallback cannot be assumed validated\n";
}
static void landed_one_pulse(){
  Harness h;h.ramp();
  h.s.feedback.landed=false;h.s.at_rest=false;
  h.s.attitude.z=1.97;h.s.attitude.vz=0;
  assert(h.r.request("LAND",{},h.s,h.t).accepted);
  auto x=h.tick();assert(x.publish&&h.r.attitude_core().phase()==a::Phase::Descent);
  // Detector glitches to landed for exactly ONE new frame while near z0/vz0;
  // a genuine touchdown requires multiple NEW samples and dwell.
  h.s.feedback.landed=true;h.s.at_rest=true;
  x=h.tick();assert(x.publish);
  h.s.feedback.landed=false;h.s.at_rest=false;
  for(int i=0;i<7;++i){x=h.tick();assert(x.publish);}
  assert(h.disarms==0&&h.r.attitude_core().phase()==a::Phase::Descent);
  std::cout<<"C-S10-05 POSITIVE PASS: single transient landed=true sample in near-ground DESCENT cannot generate Disarm; arm_sinks=1 disarm_sinks=0\n";
}
static void shutdown_timeout(){
  Harness h;h.ramp();h.s.feedback.landed=false;h.s.at_rest=false;h.s.attitude.z=1.97;
  h.r.begin_shutdown(h.s,h.t);assert(h.r.shutting_down());
  bool saw_descent=false;bool stopped=false;
  for(int i=0;i<260;++i){auto o=h.tick();
    if(h.r.attitude_core().phase()==a::Phase::Descent){saw_descent=true;assert(o.publish);}
    if(h.r.attitude_core().phase()==a::Phase::Safety){assert(!o.publish);stopped=true;break;}
  }
  assert(saw_descent&&stopped&&h.s.feedback.armed&&h.s.feedback.offboard);
  std::cout<<"C-S10-06 EXPECTED COUNTEREXAMPLE / SHUTDOWN CONTRACT FAIL: SIGINT-style graceful shutdown -> DESCENT -> timeout -> no heartbeat while Armed+Offboard\n";
}
static void late_ack_never_blind_disarm(){
  Harness h;h.armed();assert(h.r.request("LAND",{},h.s,h.t).accepted);
  // PoweredGround recovery with fresh 1Hz land samples, retained ACK ambiguity.
  h.one_hz=true;
  for(int i=0;i<300&&h.disarms==0;++i)h.tick();
  assert(h.disarms==1&&h.r.pending(400).command400_kind==f::Command400Kind::Disarm);
  assert(h.r.pending(400).ack_ambiguous);
  const auto previous=h.disarms;
  for(int result:{0,4,6,255}){
    const auto stamp=std::max(h.r.pending(400).baseline,h.r.pending(400).last_ack)+1000;
    h.r.ack({stamp,400,result,1,191,false},h.t+.001);
    assert(!h.r.pending(400).accepted&&!h.r.pending(400).rejected);
    for(int i=0;i<10;++i){auto out=h.tick();assert(out.publish);}
    assert(h.disarms==previous);
  }
  std::cout<<"C-S10-07 POSITIVE PASS: reused 400 ACK results [0,4,6,255] not attributed, no repeat Disarm, dual sink stream continuous; Disarm final sinks="<<h.disarms<<"\n";
}
static void non_atomic_sinks(){
  Harness h;h.ramp();auto o=h.tick();assert(o.publish);
  auto c=h.r.publication_context(h.s,h.t,true,true);
  int target=0,mode=0;bool threw=false;
  try{
    a::publish_attitude_at_boundary<Mode,Target>(c,o,999,
      [&](const Target&){++target;},
      [&](const Mode&){++mode;throw std::runtime_error("MODE_SINK_THROW_SYNTHETIC");});
  }catch(const std::runtime_error&){threw=true;}
  assert(threw&&target==1&&mode==1);
  std::cout<<"C-S10-08 PARTIAL SINK COUNTEREXAMPLE: target publish succeeded, mode sink raised; no atomic commit or PX4 reception guarantee; targets="<<target<<" mode_calls="<<mode<<"\n";
}
int main(){
  timeout_armed_offboard();estimator_loss_live();observed_mode_exit();no_xy_proof();
  landed_one_pulse();shutdown_timeout();late_ack_never_blind_disarm();non_atomic_sinks();
  std::cout<<"INDEPENDENT REPRODUCERS COMPLETED (exit=0 means findings/assertions reproduced, NOT flight-safety PASS)\n";
  return 0;
}
