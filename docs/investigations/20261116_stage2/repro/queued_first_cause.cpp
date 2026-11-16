#include "vision_test_support.hpp"
#include <iostream>
using namespace vision_test;
int main(){
 auto c=config();c.attitude.visual.enabled=false;
 VisionFlight q(c);
 auto early=q.r.publication_context(q.s,q.t,true,true).feedback;
 early.preflight=false;
 q.r.begin_attitude_safety_observation();
 q.r.observe_attitude_safety_event(early,a::Core::SafetyEvent::Status,q.t);
 std::cout<<"early phase="<<int(q.r.attitude_core().phase())<<" mask="<<q.r.attitude_core().queued_safety_evidence().mask<<"\n";
 q.warm();q.manual_arm();
 need(q.r.attitude_core().phase()==a::Phase::RcWaitNavigation,"not waiting navigation");
 auto bad=q.r.publication_context(q.s,q.t,true,true).feedback;bad.failsafe=true;
 q.r.begin_attitude_safety_observation();
 q.r.observe_attitude_safety_event(bad,a::Core::SafetyEvent::Failsafe,q.t);
 const auto evidence=q.r.attitude_core().queued_safety_evidence();
 std::cout<<"fault phase="<<int(q.r.attitude_core().phase())<<" mask="<<evidence.mask<<" stamp="<<evidence.status_stamp<<" expected_failsafe_bit=2 current_status="<<bad.status_stamp<<"\n";
 return (evidence.mask&2U)?0:1;
}
