#if __has_include("jikl_fc01_system/attitude_altitude_core.hpp")
#include "jikl_fc01_system/runtime.hpp"
#include <cassert>
#include <iostream>
namespace f=fc01;namespace a=fc01::attitude;
static f::Config cfg(bool enabled=false){
  f::Config c;c.profile="attitude_altitude";
  if(enabled){
    auto& p=c.attitude;p.flight_actions_enabled=p.arm_enabled=p.takeoff_enabled=p.parameters_confirmed=true;
    p.autonomous_failsafe_confirmed=true; // synthetic policy, never deployed
    p.vertical.hover=.4;p.vertical.minimum=0;p.vertical.maximum=.7;p.vertical.slew=1;
    p.vertical.height_gain=1;p.vertical.velocity_gain=.2;p.vertical.integral_gain=.1;
    p.vertical.integral_limit=.1;p.vertical.max_height=5;p.vertical.max_climb=.5;p.vertical.max_descent=.3;
    p.ramp_seconds=.5;p.settle_seconds=.1;p.landing_seconds=.1;p.height_tolerance=.1;
    p.speed_tolerance=.1;p.ascent_timeout=10;p.descent_timeout=10;p.saturation_seconds=1;p.max_flight_seconds=60;p.max_tilt_deg=20;
  p.armed_dwell_seconds=.2;p.liftoff_dwell_seconds=.1;p.liftoff_height_m=.12;
  p.liftoff_speed_mps=.08;p.touchdown_dwell_seconds=.1;
  p.height_lower_margin_m=.25;p.z_jump_limit_m=1.2;p.vz_jump_limit_mps=2.0;
  }return c;
}
static f::Snapshot sample(){
  f::Snapshot s;s.status_fresh=s.land_fresh=s.local_fresh=true;s.at_rest=true;s.stationary_seconds=3;
  s.status_stamp=s.land_stamp=1000;s.system=s.component=1;
  s.feedback.exclusive=true;s.feedback.landed=true;
  s.battery_fresh=s.battery_connected=s.power_valid=s.manual_fresh=true;s.feedback.preflight=true;
  s.attitude.attitude_fresh=s.attitude.failsafe_fresh=s.attitude.z_valid=s.attitude.vz_valid=true;
  s.attitude.battery_voltage=7.6;s.attitude.q={1,0,0,0};s.attitude.z=2;s.attitude.vz=0;return s;
}
static f::Output tick(f::Runtime& r,f::Snapshot& s,double now){s.status_stamp+=1000;s.land_stamp+=1000;return r.step(s,now);}
static double prepare(f::Runtime& r,f::Snapshot& s,bool select_authority=false){
  if(select_authority){
    assert(r.request("INTERACTIVE",{},s,0).accepted);
    assert(r.request("AUTHORITY_AUTONOMOUS",{},s,0).accepted);
  }
  assert(r.request("ATTITUDE_READY",{},s,0).accepted);
  assert(r.attitude_core().phase()==a::Phase::SelfCheck);
  assert(tick(r,s,0).attitude.publish);
  assert(r.attitude_core().phase()==a::Phase::Prestream);
  for(int i=1;i<24;++i){auto out=tick(r,s,i*.05);assert(out.attitude.publish&&out.attitude.thrust[2]==0);assert(out.attitude.command==a::Command::None);}
  auto out=tick(r,s,1.25);assert(out.attitude.command==a::Command::Offboard);
  assert(r.pending(176).sent&&!r.pending(400).sent);return 1.25;
}
static double ready(f::Runtime& r,f::Snapshot& s,bool select_authority=false){
  prepare(r,s,select_authority);auto base=r.pending(176).baseline;
  r.ack({base+1,176,0,1,191,false},1.26);
  s.feedback.offboard=true;s.nav_state=14;tick(r,s,1.3);
  assert(r.attitude_core().phase()==a::Phase::OffboardReady);return 1.3;
}
static void strict_ready(){
  f::Runtime r(cfg());auto s=sample();prepare(r,s);auto b=r.pending(176).baseline;
  r.ack({b+1,512,0,1,191,false},1.26);r.ack({b+2,176,0,255,190,false},1.26);
  r.ack({b,176,0,1,191,false},1.26);
  s.nav_state=14;s.feedback.offboard=true;tick(r,s,1.3);
  assert(r.attitude_core().phase()==a::Phase::RequestOffboard);
  r.ack({b+3,176,5,1,191,false},1.31);assert(!r.pending(176).accepted);
  r.ack({b+3,176,0,1,191,false},1.32);assert(!r.pending(176).accepted);
  r.ack({b+4,176,0,1,191,false},1.33);tick(r,s,1.35);
  assert(r.attitude_core().phase()==a::Phase::OffboardReady);
  assert(!r.pending(400).sent);
  f::Runtime no_status(cfg());s=sample();prepare(no_status,s);b=no_status.pending(176).baseline;
  no_status.ack({b+1,176,0,1,191,false},1.26);tick(no_status,s,1.3);
  assert(no_status.attitude_core().phase()==a::Phase::RequestOffboard);
  for(int i=27;i<=130;++i)tick(no_status,s,i*.05);
  assert(no_status.attitude_core().phase()==a::Phase::Safety);
}
static void gates(){
  for(int gate=0;gate<4;++gate){auto c=cfg(true);
    if(gate==0)c.attitude.flight_actions_enabled=false;
    if(gate==1)c.attitude.arm_enabled=false;
    if(gate==2)c.attitude.takeoff_enabled=false;
    f::Runtime r(c);auto s=sample();auto t=ready(r,s);
    if(gate<3){assert(r.request("INTERACTIVE",{},s,t).accepted);assert(r.request("PING",{},s,t).accepted);}
    assert(!r.request("TAKEOFF",{1},s,t).accepted);assert(!r.pending(400).sent);
    auto o=tick(r,s,t+.05);assert(o.attitude.publish&&o.attitude.thrust[2]==0);
    assert(!r.request("VEL",{0,0,0,0},s,t+.05).accepted);
  }
  bool rejected=false;auto c=cfg();c.attitude.flight_actions_enabled=true;
  try{f::Runtime r(c);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
  f::Runtime r(cfg());auto s=sample();ready(r,s);s.feedback.armed=true;
  auto o=tick(r,s,1.35);assert(!o.attitude.publish&&r.attitude_core().phase()==a::Phase::Safety);
  assert(!r.pending(400).sent);
}
static void faults(){
  for(int failure=0;failure<12;++failure){f::Runtime r(cfg());auto s=sample();ready(r,s);
    double t=1.35;
    switch(failure){
      case 0:s.attitude.attitude_fresh=false;break;case 1:s.local_fresh=false;break;
      case 2:s.attitude.z_valid=false;break;case 3:s.attitude.vz_valid=false;break;
      case 4:s.attitude.q[0]=NAN;break;case 5:s.attitude.vz=INFINITY;break;
      case 6:s.feedback.failsafe=true;break;case 7:s.feedback.exclusive=false;break;
      case 8:s.time_regressed=true;break;case 9:t=1.7;break;
      case 10:s.attitude.failsafe_fresh=false;break;case 11:t=1.2;break;
    }
    auto o=tick(r,s,t);assert(!o.attitude.publish&&o.attitude.command==a::Command::None);
    assert(r.attitude_core().phase()==a::Phase::Safety);s=sample();o=tick(r,s,t+.05);assert(!o.attitude.publish);
  }
  f::Runtime r(cfg());auto s=sample();ready(r,s);
  s.attitude.q={0,0,0,1};s.attitude.quat_reset=1;auto o=tick(r,s,1.35);
  assert(o.attitude.publish&&std::abs(o.attitude.q[3]-1)<1e-8);
  r.disconnect(s,1.36);
  auto released=tick(r,s,1.4);
  assert(released.attitude.command==a::Command::ReleaseOffboard);
  assert(released.attitude.thrust[2]==0);
}
static void flight_and_sequential_400(){
  f::Runtime r(cfg(true));auto s=sample();double t=ready(r,s,true);
  assert(r.request("INTERACTIVE",{},s,t).accepted);assert(r.request("PING",{},s,t).accepted);
  assert(r.request("TAKEOFF",{1},s,t).accepted);
  auto o=tick(r,s,t+=.05);assert(o.attitude.command==a::Command::Arm);
  const auto arm_base=r.pending(400).baseline;
  assert(!r.request("TAKEOFF",{1},s,t).accepted);
  r.ack({arm_base+1,400,5,1,191,false},t+.01);
  r.ack({arm_base+2,400,0,1,191,false},t+.02);
  s.feedback.armed=true;tick(r,s,t+=.05);assert(r.attitude_core().phase()==a::Phase::ArmedGround);
  tick(r,s,t+=.05);assert(r.attitude_core().phase()==a::Phase::ArmedGround);
  for(int i=0;i<4;++i){tick(r,s,t+=.05);}
  assert(r.attitude_core().phase()==a::Phase::ThrustRamp);
  for(int i=0;i<14;++i){r.request("PING",{},s,t);o=tick(r,s,t+=.05);assert(o.attitude.publish&&o.attitude.thrust[2]<=0);}
  assert(r.attitude_core().phase()==a::Phase::ThrustRamp); // timer cannot count as liftoff
  s.feedback.landed=false;s.at_rest=false;s.attitude.vz=-.1;
  for(int i=0;i<6;++i){s.attitude.z-=.04;r.request("PING",{},s,t);tick(r,s,t+=.05);}
  assert(r.attitude_core().phase()==a::Phase::Ascend);
  for(int i=0;i<5;++i){s.attitude.z-=.04;r.request("PING",{},s,t);tick(r,s,t+=.05);}
  assert(r.attitude_core().phase()==a::Phase::Ascend); // a premature height sample cannot bypass the target ramp
  // Synthetic estimator follows commanded climb with bounded sample continuity.
  for(int i=0;i<45;++i){s.attitude.z=std::max(1.,s.attitude.z-.025);
    s.attitude.vz=s.attitude.z>1.01?-.1:0;
    r.request("PING",{},s,t);tick(r,s,t+=.05);}
  assert(r.attitude_core().phase()==a::Phase::HeightHold);
  assert(r.request("LAND",{},s,t).accepted);
  s.feedback.landed=false;s.at_rest=false;
  bool bounced=false;
  for(int i=0;i<105&&!r.attitude_core().disarm_requested();++i){
    s.attitude.z=std::min(2.,s.attitude.z+.025);s.attitude.vz=s.attitude.z<1.99?.1:0;
    if(s.attitude.z>=1.99){s.feedback.landed=true;s.at_rest=true;}
    if(!bounced&&r.attitude_core().phase()==a::Phase::Landed){
      // Contact momentarily disappears during thrust-down: never disarm.
      bounced=true;s.feedback.landed=s.at_rest=false;
    }
    r.request("PING",{},s,t);o=tick(r,s,t+=.05);
    if(bounced&&!s.feedback.landed)assert(r.attitude_core().phase()==a::Phase::Descent);
  }
  assert(bounced&&r.attitude_core().phase()==a::Phase::RequestDisarm);
  assert(o.attitude.command==a::Command::Disarm);
  const auto disarm_base=r.pending(400).baseline;
  assert(disarm_base>=arm_base+2&&!r.pending(400).accepted);
  r.ack({arm_base+2,400,0,1,191,false},t+.01);assert(!r.pending(400).accepted);
  r.ack({disarm_base+1,400,0,255,190,false},t+.01);assert(!r.pending(400).accepted);
  r.ack({disarm_base+2,400,0,1,191,false},t+.02);
  // R4: even a later timestamp cannot distinguish an old duplicate Arm ACK.
  assert(!r.pending(400).accepted&&r.pending(400).ack_ambiguous);
  tick(r,s,t+=.05);assert(r.attitude_core().phase()==a::Phase::RequestDisarm);
  s.feedback.armed=false;o=tick(r,s,t+=.05);
  assert(r.attitude_core().phase()==a::Phase::SafeDisarmed&&!o.attitude.publish);
  o=tick(r,s,t+=.05);assert(o.attitude.command==a::Command::ReleaseOffboard);
  const auto mode_baseline=r.pending(176).baseline;
  r.ack({mode_baseline+2,176,0,1,191,false},t+.01);
  s.feedback.offboard=false;s.nav_state=15;tick(r,s,t+=.05);
  assert(r.attitude_core().phase()==a::Phase::Idle);
}
static void active_reset_and_timeout(){
  for(int reset=0;reset<6;++reset){f::Runtime r(cfg(true));auto s=sample();double t=ready(r,s,true);
    r.request("INTERACTIVE",{},s,t);r.request("PING",{},s,t);r.request("TAKEOFF",{1},s,t);tick(r,s,t+=.05);
    auto b=r.pending(400).baseline;r.ack({b+1,400,0,1,191,false},t+.01);s.feedback.armed=true;tick(r,s,t+=.05);
    if(reset==0)++s.attitude.quat_reset;
    if(reset==1)++s.attitude.z_reset;
    if(reset==2)++s.attitude.vz_reset;
    if(reset==3){s.feedback.offboard=false;s.nav_state=15;}
    if(reset==4)s.attitude.z=-4; // max height applies during ramp as well as hold
    if(reset==5)s.attitude.q={std::sqrt(.5),std::sqrt(.5),0,0}; // ninety degree tilt
    auto o=tick(r,s,t+.05);assert(!o.attitude.publish&&r.attitude_core().phase()==a::Phase::Safety);
  }
  f::Runtime r(cfg(true));auto s=sample();double t=ready(r,s,true);
  r.request("INTERACTIVE",{},s,t);r.request("PING",{},s,t);r.request("TAKEOFF",{1},s,t);tick(r,s,t+=.05);
  for(int i=0;i<105;++i){r.request("PING",{},s,t);tick(r,s,t+=.05);}
  // R3: timed-out Command 400 is not proof of non-execution. Retain guarded
  // zero heartbeat; a late armed PX4 status enters supervised recovery.
  assert(r.attitude_core().phase()==a::Phase::RequestArm);
  assert(r.attitude_core().arm_cancel_pending());
  assert(tick(r,s,t+=.05).attitude.publish);
  s.feedback.armed=true;auto late=tick(r,s,t+=.05);
  assert(late.attitude.publish);
  assert(r.attitude_core().phase()==a::Phase::PoweredGround);
}
static void cancel_on_armed_ground(){
  for(int progress=0;progress<3;++progress)for(int cancellation=0;cancellation<3;++cancellation){
    auto c=cfg(true);c.attitude.landing_seconds=.6;
    if(progress==2)c.attitude.vertical.slew=.1;
    f::Runtime r(c);auto s=sample();double t=ready(r,s,true);
    r.request("INTERACTIVE",{},s,t);r.request("PING",{},s,t);
    assert(r.request("TAKEOFF",{1},s,t).accepted);
    auto o=tick(r,s,t+=.05);assert(o.attitude.command==a::Command::Arm);
    r.ack({r.pending(400).baseline+1,400,0,1,191,false},t+.01);
    s.feedback.armed=true;o=tick(r,s,t+=.05);
    assert(r.attitude_core().phase()==a::Phase::ArmedGround);
    if(progress>=1){for(int i=0;i<7;++i){r.request("PING",{},s,t);o=tick(r,s,t+=.05);}
      assert(o.attitude.thrust[2]<0);}
    if(progress==2){
      for(int i=0;i<20&&r.attitude_core().phase()!=a::Phase::Ascend;++i){
        r.request("PING",{},s,t);o=tick(r,s,t+=.05);
      }
      assert(r.attitude_core().phase()==a::Phase::ThrustRamp&&s.feedback.landed&&s.at_rest);
      assert(-o.attitude.thrust[2]<c.attitude.vertical.hover); // no liftoff evidence
    }
    double previous=-o.attitude.thrust[2];
    if(cancellation==1)r.disconnect(s,t);
    else if(cancellation==2)r.begin_shutdown(s,t);
    else assert(r.request("LAND",{},s,t).accepted);
    const double cancelled_at=t;
    for(int i=0;i<60&&!r.attitude_core().disarm_requested();++i){
      o=tick(r,s,t+=.05);
      assert(o.attitude.publish&&o.attitude.command!=a::Command::Arm);
      const double thrust=-o.attitude.thrust[2];
      assert(thrust<=previous+1e-12); // cancellation must never add hover feed-forward
      previous=thrust;
      if(t-cancelled_at<c.attitude.landing_seconds)assert(o.attitude.command!=a::Command::Disarm);
    }
    assert(r.attitude_core().phase()==a::Phase::RequestDisarm&&previous<=1e-9);
    assert(o.attitude.command==a::Command::Disarm);
    r.ack({r.pending(400).baseline+1,400,0,1,191,false},t+.01);
    s.feedback.armed=false;assert(!tick(r,s,t+=.05).attitude.publish);
    assert(r.attitude_core().phase()==a::Phase::SafeDisarmed);
  }
}
static void shutdown_rejects_queued_starts(){
  for(int phase=0;phase<3;++phase){
    f::Runtime r(cfg(true));auto s=sample();double t=0;
    if(phase==1){assert(r.request("ATTITUDE_READY",{},s,t).accepted);tick(r,s,t);}
    if(phase==2)t=ready(r,s);
    r.begin_shutdown(s,t);assert(r.shutting_down());
    // Same poll batch: a queued prepare/interactive/takeoff cannot restart shutdown.
    for(const std::string op:{"ATTITUDE_READY","INTERACTIVE","TAKEOFF","MODE"}){
      auto reply=r.request(op,op=="TAKEOFF"?std::vector<double>{1}:std::vector<double>{},s,t);
      assert(!reply.accepted&&reply.message=="SHUTDOWN_NO_NEW_CONTROL_TASKS");
    }
    assert(r.request("STATUS",{},s,t).accepted&&r.request("PING",{},s,t).accepted);
    assert(r.request("LAND",{},s,t).accepted);
    for(int i=0;i<30;++i){auto o=tick(r,s,t+=.05);
      assert(o.attitude.thrust[2]==0&&o.attitude.command!=a::Command::Arm);
      if(phase==2)assert(r.attitude_core().phase()==a::Phase::ReleaseOffboard);
    }
    assert(!r.pending(400).sent);
  }
  f::Runtime r(cfg(true));auto s=sample();double t=ready(r,s,true);
  r.request("INTERACTIVE",{},s,t);r.request("TAKEOFF",{1},s,t);
  r.begin_shutdown(s,t); // before the queued Arm would be emitted
  auto o=tick(r,s,t+=.05);
  // R3: unsent Arm is cancelled before publication; the already-active
  // Offboard mode must still be returned via ground-only 176 handshake.
  assert(o.attitude.publish&&o.attitude.thrust[2]==0);
  assert(o.attitude.command==a::Command::ReleaseOffboard);
  assert(!r.pending(400).sent&&r.attitude_core().phase()==a::Phase::ReleaseOffboard);
}
static void preparation_interactive_lease(){
  auto c=cfg();c.attitude.console_timeout=.4;
  f::Runtime prestream(c);auto s=sample();
  assert(prestream.request("INTERACTIVE",{},s,0).accepted);
  assert(prestream.request("ATTITUDE_READY",{},s,0).accepted);
  for(int i=0;i<=10;++i)tick(prestream,s,i*.05);
  assert(prestream.attitude_core().phase()==a::Phase::Safety&&!prestream.pending(176).sent);
  assert(!tick(prestream,s,.55).attitude.publish);
  f::Runtime interactive(cfg());s=sample();double t=ready(interactive,s);
  assert(interactive.request("INTERACTIVE",{},s,t).accepted);
  for(int i=0;i<64;++i)tick(interactive,s,t+=.05);
  assert(interactive.attitude_core().phase()==a::Phase::Safety);
  assert(interactive.message()=="PREPARATION_CONSOLE_LEASE_EXPIRED");
  f::Runtime one_shot(cfg());s=sample();t=ready(one_shot,s);
  // The one-shot ready client never creates an interactive lease; it ends on disconnect.
  for(int i=0;i<80;++i){auto o=tick(one_shot,s,t+=.05);assert(o.attitude.publish&&o.attitude.thrust[2]==0);}
  assert(one_shot.attitude_core().phase()==a::Phase::OffboardReady);
  one_shot.disconnect(s,t);
  auto release=tick(one_shot,s,t+.05);
  assert(release.attitude.publish&&release.attitude.thrust[2]==0&&
    release.attitude.command==a::Command::ReleaseOffboard);
}
int main(){
  f::Runtime ground;auto ground_sample=sample();assert(ground.request("INTERACTIVE",{},ground_sample,0).accepted);
  strict_ready();gates();faults();flight_and_sequential_400();active_reset_and_timeout();cancel_on_armed_ground();shutdown_rejects_queued_starts();preparation_interactive_lease();
  std::cout<<"attitude-altitude runtime tests: PASS (stage locks, ACK+status, faults, full flight lifecycle, sequential 400)\n";
}
#else
#include <cassert>
int main(){assert(!"attitude-altitude state machine required by contract is absent");}
#endif
