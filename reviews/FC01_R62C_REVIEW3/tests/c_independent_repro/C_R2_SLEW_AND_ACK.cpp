#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <iostream>
#include <cassert>
namespace f=fc01;
namespace a=fc01::attitude;
static f::Config config(){auto c=authority_test::config();c.attitude.autonomous_failsafe_confirmed=true;return c;}
int main(){
  // C09: independently verify the takeoff-timeout transition tick,
  // using actual R2 state transitions, not assertions imported from B.
  {
    f::Runtime r(config());auto s=authority_test::sample();double t=0;
    authority_test::armed(r,s,t,"AUTHORITY_AUTONOMOUS");s.at_rest=false;
    a::Output previous;double maxjump=0,last=0;bool transition=false;bool all_publish=true;
    for(int i=0;i<300;++i){
      auto o=authority_test::tick(r,s,t).attitude;
      const double power=-o.thrust[2];
      maxjump=std::max(maxjump,std::abs(power-last));
      last=power;all_publish=all_publish&&o.publish;
      if(r.attitude_core().phase()==a::Phase::PoweredGround){
        transition=true;std::cout<<"C09_TIMEOUT_TICK phase="<<a::name(r.attitude_core().phase())
          <<" output="<<power<<" previous="<<-previous.thrust[2]
          <<" jump="<<std::abs(power+previous.thrust[2])
          <<" max_step_slew="<<config().attitude.vertical.slew*.05<<" heartbeat="<<o.publish<<"\n";
        break;
      }
      previous=o;
    }
    std::cout<<"C09_TIMEOUT_SUMMARY transition="<<transition<<" max_adjacent_delta="<<maxjump
      <<" stream_continuous="<<all_publish<<"\n";
    assert(transition&&all_publish&&maxjump<=config().attitude.vertical.slew*.05+1e-7);
  }
  // C10: post-request pre-commit ACK must not be accepted; a denied final
  // sink must never be recorded as a committed publication.
  {
    f::Runtime r(config());auto s=authority_test::sample();double t=0;
    authority_test::ready(r,s,t,"AUTHORITY_AUTONOMOUS");
    r.require_attitude_sink_commit();
    assert(r.request("TAKEOFF",{1},s,t).accepted);
    auto o=authority_test::tick(r,s,t);
    assert(o.attitude.command==a::Command::Arm);
    const auto before=r.pending(400).baseline;
    r.ack({before+1,400,0,1,191,false},t+.001);
    const bool premature=r.pending(400).accepted;
    r.confirm_attitude_command_publication(a::Command::Arm,false,0,s,t+.002);
    r.ack({before+2,400,0,1,191,false},t+.003);
    auto p=r.pending(400);
    std::cout<<"C10_ACK_BEFORE_DENIED_SINK premature_accepted="<<premature
      <<" requested="<<p.requested<<" published="<<p.published<<" sent="<<p.sent
      <<" sink_denied="<<p.sink_denied<<" later_accepted="<<p.accepted<<"\n";
    assert(!premature&&p.requested&&!p.published&&!p.sent&&p.sink_denied&&!p.accepted);
  }
}
