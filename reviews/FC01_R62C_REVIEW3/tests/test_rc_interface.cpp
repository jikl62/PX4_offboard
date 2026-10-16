#include "authority_test_support.hpp"
#include "jikl_fc01_system/authority_telemetry.hpp"
#include <cassert>
#include <iostream>
using namespace authority_test;
struct Manual {
  static constexpr std::uint8_t SOURCE_UNKNOWN=0,SOURCE_RC=1,SOURCE_MAVLINK_0=2;
  std::uint64_t timestamp{1000000},timestamp_sample{1000000};
  bool valid{true};std::uint8_t data_source{SOURCE_RC};bool sticks_moving{};
};
struct Switches {
  std::uint64_t timestamp{1000000},timestamp_sample{1000000};
  std::uint8_t mode_slot{1},arm_switch{3},return_switch{3},loiter_switch{3},offboard_switch{3},kill_switch{3};
  std::uint32_t switch_changes{};
};
static void advance(Manual& m,std::uint64_t us=1000){m.timestamp+=us;m.timestamp_sample+=us;}
static void advance(Switches& s,std::uint64_t us=1000){s.timestamp+=us;s.timestamp_sample+=us;}
int main(){
  constexpr double timeout=2.5;
  {
    f::AuthorityTelemetry o;
    auto x=o.facts(0,timeout,true,false,0,0);
    assert(!x.rc_link_ready&&!x.rc_switches_ready&&!x.manual_seen&&!x.switches_seen);
  }
  {
    f::AuthorityTelemetry o;Manual m;
    m.valid=false;o.manual(m,0);advance(m);o.manual(m,.01);
    assert(!o.facts(.01,timeout,true,false,1,0).rc_link_ready);
  }
  {
    f::AuthorityTelemetry o;Manual m;m.data_source=Manual::SOURCE_MAVLINK_0;
    o.manual(m,0);advance(m);o.manual(m,.01);
    auto x=o.facts(.01,timeout,true,false,1,0);assert(!x.rc_link_ready&&!x.rc_source_selected&&x.data_source==Manual::SOURCE_MAVLINK_0);
  }
  {
    f::AuthorityTelemetry o;Manual m;
    o.manual(m,0);assert(!o.facts(0,timeout,true,false,1,0).rc_link_ready);
    m.timestamp+=1000;o.manual(m,.01); // duplicate raw timestamp cannot refresh activity
    assert(!o.facts(.01,timeout,true,false,1,0).rc_link_ready);
    m.timestamp+=1000;m.timestamp_sample+=2000;o.manual(m,.02);
    auto x=o.facts(.02,timeout,true,false,1,0);assert(x.rc_link_ready&&x.rc_source_selected);
    assert(!o.facts(3.0,timeout,true,false,1,0).rc_link_ready);
    m.timestamp+=1000;m.timestamp_sample+=1000;o.manual(m,3.0);
    assert(!o.facts(3.0,timeout,true,false,1,0).rc_link_ready); // first after stale gap
    m.timestamp+=1000;m.timestamp_sample+=1000;o.manual(m,3.01);
    assert(o.facts(3.01,timeout,true,false,1,0).rc_link_ready);
  }
  {
    f::AuthorityTelemetry o;Manual m;Switches sw;
    o.manual(m,0);m.timestamp+=1000;m.timestamp_sample+=1000;o.manual(m,.01);
    auto x=o.facts(.01,timeout,true,false,1,0);assert(x.rc_link_ready&&!x.rc_switches_ready);
    o.switches(sw,.01,timeout);assert(!o.facts(.01,timeout,true,false,1,1).rc_switches_ready);
    advance(sw);o.switches(sw,.02,timeout);x=o.facts(.02,timeout,true,false,1,1);assert(x.rc_switches_ready&&x.kill_switch==3);
    sw.kill_switch=1;advance(sw);o.switches(sw,.03,timeout);x=o.facts(.03,timeout,true,false,1,1);assert(x.kill&&!x.rc_switches_ready);
    // Missing/stale/unknown does not clear a latched kill.
    assert(o.facts(10,timeout,true,false,1,0).kill);
    sw.kill_switch=0;advance(sw);o.switches(sw,10.0,timeout);assert(o.facts(10.0,timeout,true,false,1,1).kill);
    sw.kill_switch=3;advance(sw);o.switches(sw,10.01,timeout);assert(!o.facts(10.01,timeout,true,false,1,1).kill);
  }
  {
    // sticks_moving is diagnostic-only in 0.4.3: it must come from a fresh,
    // valid, selected RC sample and must not itself satisfy switch/confirmation gates.
    f::AuthorityTelemetry o;Manual m;m.sticks_moving=true;
    o.manual(m,0);m.timestamp+=1000;m.timestamp_sample+=1000;o.manual(m,.01);
    auto x=o.facts(.01,timeout,true,false,1,0);assert(x.rc_link_ready&&x.sticks_moving&&!x.rc_switches_ready);
    auto c=config();f::Runtime r(c);auto s=sample();double t=0;
    s.attitude.rc_link_ready=true;s.attitude.rc_sticks_moving=true;
    assert(r.request("INTERACTIVE",{},s,t).accepted);
    assert(r.request("AUTHORITY_RC",{},s,t).accepted);
    assert(r.request("ATTITUDE_READY",{},s,t).accepted);
    r.step(s,t);
    assert(!r.attitude_core().authority_state().takeover_latched);
    auto j=r.authority_json(s);
    assert(j.find("\"rc_sticks_moving\":true")!=std::string::npos);
    assert(j.find("\"rc_takeover_ready\":false")!=std::string::npos);
  }
  {
    auto c=config();c.attitude.autonomous_failsafe_confirmed=true;f::Runtime r(c);auto s=sample();double t=0;
    assert(r.request("INTERACTIVE",{},s,t).accepted);
    assert(r.request("AUTHORITY_AUTONOMOUS",{},s,t).accepted);
    assert(r.request("ATTITUDE_READY",{},s,t).accepted); // RC telemetry intentionally absent
  }
  std::cout<<"ManualControlSetpoint RC interface / switch fail-closed regressions: PASS\n";
}
