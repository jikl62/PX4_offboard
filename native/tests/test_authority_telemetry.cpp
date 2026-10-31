// Mutations caught: invalid/non-RC manual source admission, refreshed duplicate
// envelopes, stale raw sample disguised by a fresh envelope, missing switch DDS,
// and unmapped Kill clearing.
#include "jikl_fc01_system/authority_telemetry.hpp"
#include <cassert>
#include <iostream>
struct Manual {
  static constexpr std::uint8_t SOURCE_UNKNOWN=0,SOURCE_RC=1,SOURCE_MAVLINK_0=2;
  std::uint64_t timestamp{1000000},timestamp_sample{1000000};
  bool valid{true};std::uint8_t data_source{SOURCE_RC};bool sticks_moving{};
};
struct Switches {std::uint64_t timestamp{1000000},timestamp_sample{1000000};
  std::uint8_t mode_slot{1},arm_switch{3},return_switch{3},loiter_switch{3},offboard_switch{3},kill_switch{3};std::uint32_t switch_changes{};};
int main(){
  fc01::AuthorityTelemetry observer;auto manual=Manual{};auto sw=Switches{};
  auto facts=[&](double now=0,bool flags=true,bool lost=false,int manual_sources=1,int switches=1){return observer.facts(now,.5,flags,lost,manual_sources,switches);};
  assert(!facts().rc_link_ready&&!facts().rc_switches_ready);
  observer.manual(manual,0);observer.switches(sw,0,.5);
  assert(!facts().rc_link_ready&&!facts().rc_switches_ready&&!facts().kill);
  manual.timestamp+=1000;manual.timestamp_sample+=1000;sw.timestamp+=1000;sw.timestamp_sample+=1000;
  observer.manual(manual,.01);observer.switches(sw,.01,.5);
  assert(facts(.01).rc_link_ready&&facts(.01).rc_switches_ready);
  assert(!facts(.01,false).rc_link_ready&&!facts(.01,true,true).rc_link_ready);
  assert(!facts(.01,true,false,2).rc_link_ready&&!facts(.01,true,false,1,2).rc_switches_ready);
  manual.timestamp+=1000;sw.timestamp+=1000;observer.manual(manual,.4);observer.switches(sw,.4,.5); // duplicate raw timestamps
  assert(!facts(.52).rc_link_ready&&!facts(.52).rc_switches_ready);
  for(int failure=0;failure<4;++failure){fc01::AuthorityTelemetry x;auto bad=Manual{};
    switch(failure){case 0:bad.valid=false;break;case 1:bad.data_source=Manual::SOURCE_UNKNOWN;break;
      case 2:bad.data_source=Manual::SOURCE_MAVLINK_0;break;case 3:bad.timestamp_sample=bad.timestamp+1;break;}
    x.manual(bad,0);bad.timestamp+=1000;bad.timestamp_sample+=1000;x.manual(bad,.01);
    assert(!x.facts(.01,.5,true,false,1,1).rc_link_ready);
  }
  for(std::uint8_t kill:{0,1,2,4}){fc01::AuthorityTelemetry x;auto bad=sw;bad.kill_switch=kill;
    x.switches(bad,0,.5);bad.timestamp+=1000;bad.timestamp_sample+=1000;x.switches(bad,.01,.5);
    auto f=x.facts(.01,.5,true,false,1,1);assert(!f.rc_switches_ready&&f.kill==(kill==1));
  }
  fc01::AuthorityTelemetry changes;auto sw2=Switches{};changes.switches(sw2,0,.5);assert(changes.facts(0,.5,true,false,1,1).switch_generation==0);
  sw2.timestamp+=1000;sw2.timestamp_sample+=1000;++sw2.mode_slot;changes.switches(sw2,.05,.5);
  assert(changes.facts(.05,.5,true,false,1,1).switch_generation==1);
  sw2.timestamp+=1000;sw2.timestamp_sample+=1000;changes.switches(sw2,.1,.5);
  assert(changes.facts(.1,.5,true,false,1,1).switch_generation==1);
  fc01::AuthorityTelemetry killed;auto sw3=Switches{};sw3.kill_switch=1;killed.switches(sw3,0,.5);
  sw3.timestamp+=1000;sw3.timestamp_sample+=1000;sw3.kill_switch=0;killed.switches(sw3,.05,.5);
  assert(killed.facts(.05,.5,true,false,1,1).kill);
  sw3.timestamp+=1000;sw3.kill_switch=3;killed.switches(sw3,.1,.5); // duplicate raw sample cannot clear
  assert(killed.facts(.1,.5,true,false,1,1).kill);
  sw3.timestamp+=1000;sw3.timestamp_sample+=1000;killed.switches(sw3,.15,.5);
  assert(!killed.facts(.15,.5,true,false,1,1).kill);
  std::cout<<"actual ManualControlSetpoint/switch authority observation helper: PASS\n";
}
