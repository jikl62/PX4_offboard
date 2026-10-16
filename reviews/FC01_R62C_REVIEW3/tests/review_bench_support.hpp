#pragma once
// Read-only independent R6.2c audit; includes baseline fixtures and headers.
// No ROS, socket, PX4 connection or physical commands.
#define main stock_s10_not_invoked
#include "c_s10_independent/C_S10_SAFETY_ATTACKS.cpp"
#undef main

static f::Config bench(){auto c=config();c.attitude.bench_arm_only_enabled=true;
  c.attitude.takeoff_enabled=false;c.validate();return c;}

struct B {
  f::Runtime r{bench()};f::Snapshot s{snapshot()};double t{};
  unsigned tick_count{},targets{},heartbeats{},arms{},disarms{},releases{},denials{};
  bool connected{true};
  a::Decision last_decision=a::allow();
  explicit B(bool rc=false){r.require_attitude_sink_commit();
    if(rc)s.attitude.rc_link_ready=s.attitude.rc_switches_ready=true;
    assert(r.request("INTERACTIVE",{},s,t).accepted);
    assert(r.request(rc?"AUTHORITY_RC":"AUTHORITY_AUTONOMOUS",{},s,t).accepted);
    assert(r.request("ATTITUDE_READY",{},s,t).accepted);}
  a::Output next(bool ping=true,bool newland=true,bool publish=true,bool newlocal=true){
    t+=.05;++tick_count;s.status_stamp+=50000;
    if(newlocal)s.attitude.local_stamp+=50000;
    if(newland&&tick_count%20==0)s.land_stamp+=1000000;
    if(ping&&connected)assert(r.request("PING",{},s,t).accepted);
    auto o=r.step(s,t).attitude;
    assert(!a::nonzero(o.thrust));
    if(publish)commit(o);
    return o;
  }
  void commit(const a::Output& o){
    if(o.publish){
      last_decision=a::publish_attitude_at_boundary<Mode,Target>(
        r.publication_context(s,t,connected,true),o,1000000+std::uint64_t(t*1e6),
        [&](const Target& x){assert(x.thrust_body[0]==0&&x.thrust_body[1]==0&&x.thrust_body[2]==0);++targets;},
        [&](const Mode&){++heartbeats;});
      if(!last_decision.allowed){++denials;return;}
    }
    if(o.command==a::Command::None)return;
    const auto c=r.publication_context(s,t,connected,true);
    a::Decision d=a::allow();
    if(o.command==a::Command::Arm)d=a::publish_arm_at_boundary(c,[&]{++arms;});
    else if(o.command==a::Command::Disarm)d=a::publish_disarm_at_boundary(c,[&]{++disarms;});
    else if(o.command==a::Command::ReleaseOffboard)d=a::publish_release_at_boundary(c,[&]{++releases;});
    assert(d.allowed);
    r.confirm_attitude_command_publication(o.command,true,1000000+std::uint64_t(t*1e6),s,t);
  }
  void ready(){
    for(int i=0;i<60&&r.attitude_core().phase()!=a::Phase::RequestOffboard;++i)next();
    assert(r.attitude_core().phase()==a::Phase::RequestOffboard);
    r.ack({r.pending(176).wire_stamp+1000,176,0,1,191,false},t+.001);
    s.feedback.offboard=true;s.nav_state=14;next();
    assert(r.attitude_core().phase()==a::Phase::OffboardReady);
  }
  void planned_arm(){ready();assert(r.request("ARM_ONLY",{},s,t).accepted);
    assert(next().command==a::Command::Arm&&arms==1);}
  void armed(){planned_arm();
    r.ack({r.pending(400).wire_stamp+1000,400,0,1,191,false},t+.001);
    s.feedback.armed=true;next();
    assert(r.attitude_core().phase()==a::Phase::BenchArmedGround);
  }
  void log(const char* id,const a::Output& o){
    std::cout<<id<<" phase="<<a::name(r.attitude_core().phase())
      <<" reason="<<r.attitude_core().reason()<<" output_publish="<<o.publish
      <<" armed="<<s.feedback.armed<<" offboard="<<s.feedback.offboard
      <<" at_rest="<<s.at_rest<<" arm_sinks="<<arms<<" disarm_sinks="<<disarms
      <<" boundary="<<a::name(last_decision.reason)<<" targets="<<targets
      <<" heartbeats="<<heartbeats<<"\n";
  }
};
