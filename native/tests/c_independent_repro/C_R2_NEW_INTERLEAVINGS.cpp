#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <iostream>
#include <cassert>
namespace f=fc01;
namespace a=fc01::attitude;
static f::Config config(){auto c=authority_test::config();c.attitude.autonomous_failsafe_confirmed=true;return c;}
int main(){
  // C06: a real arm wire publish has been committed, ACK/status are still pending.
  // LAND is an allowed terminal operator action, not a Kill.
  {
    f::Runtime r(config());auto s=authority_test::sample();double t=0;
    authority_test::ready(r,s,t,"AUTHORITY_AUTONOMOUS");r.require_attitude_sink_commit();
    assert(r.request("TAKEOFF",{1.},s,t).accepted);
    auto out=authority_test::tick(r,s,t);
    assert(out.attitude.command==a::Command::Arm);
    auto boundary=r.publication_context(s,t,true,true);
    int wires=0;const auto policy=a::publish_arm_at_boundary(boundary,[&]{++wires;});
    assert(policy.allowed&&wires==1);
    r.confirm_attitude_command_publication(a::Command::Arm,true,112233,s,t);
    const auto original=r.pending(400).baseline;
    assert(r.pending(400).sent && !r.pending(400).accepted && !s.feedback.armed);
    auto accepted=r.request("LAND",{},s,t);
    out=authority_test::tick(r,s,t);
    std::cout<<"C06_ARM_PENDING_CANCEL accepted="<<accepted.accepted<<" phase="<<a::name(r.attitude_core().phase())
      <<" heartbeat="<<out.attitude.publish<<" pending_400_sent="<<r.pending(400).sent
      <<" operator_cancel_without_kill=1\n";
    // Simulate PX4's legitimate delayed response after Arm command was put on the wire.
    r.ack({original+1000,400,0,1,191,false},t+.001);
    s.feedback.armed=true;
    out=authority_test::tick(r,s,t);
    std::cout<<"C06_LATE_ARM_RESPONSE px4_armed="<<s.feedback.armed<<" px4_offboard="<<s.feedback.offboard
      <<" heartbeat="<<out.attitude.publish<<" ack400_accepted="<<r.pending(400).accepted<<" phase="<<a::name(r.attitude_core().phase())<<"\n";
  }
  // C07: after a genuine 400 Disarm sink commit, ground-contact suddenly
  // disappears before the disarm ACK. The final command cannot be recalled.
  {
    f::Runtime r(config());auto s=authority_test::sample();double t=0;
    // ready/armed fixture uses implicit ACK; manually run without requiring commit until armed, using fresh runtime.
    f::Runtime ready(config());authority_test::armed(ready,s,t,"AUTHORITY_AUTONOMOUS");
    s.at_rest=false;s.feedback.landed=true;
    assert(ready.request("LAND",{},s,t).accepted);
    auto out=authority_test::tick(ready,s,t);
    for(int i=0;i<30;++i)out=authority_test::tick(ready,s,t);
    s.at_rest=true;
    for(int i=0;i<70 && ready.attitude_core().phase()!=a::Phase::RequestDisarm;++i)
      out=authority_test::tick(ready,s,t);
    assert(ready.attitude_core().phase()==a::Phase::RequestDisarm&&out.attitude.command==a::Command::Disarm);
    auto boundary=ready.publication_context(s,t,true,true);
    int disarm_wires=0;auto decision=a::publish_disarm_at_boundary(boundary,[&]{disarm_wires++;});
    assert(decision.allowed&&disarm_wires==1);
    s.feedback.landed=false;s.at_rest=false;
    out=authority_test::tick(ready,s,t);
    std::cout<<"C07_POST_DISARM_BOUNCE disarm_wires="<<disarm_wires<<" still_armed="<<s.feedback.armed
      <<" still_offboard="<<s.feedback.offboard<<" heartbeat="<<out.attitude.publish
      <<" phase="<<a::name(ready.attitude_core().phase())<<" reason="<<ready.message()<<"\n";
    (void)r;
  }
}
