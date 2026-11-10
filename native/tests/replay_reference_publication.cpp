// Diagnostic offline replay. All sinks are local counters; no ROS or aircraft.
#define main rc_navigation_fixture_main
#include "test_rc_navigation.cpp"
#undef main
static void snapshot(const char* scenario,const char* point,const Flight& q){
  const auto& c=q.r.attitude_core();const auto& e=c.armed_ground_fault_evidence();
  std::cout<<"{\"scenario\":\""<<scenario<<"\",\"point\":\""<<point
    <<"\",\"phase\":\""<<a::name(c.phase())<<"\",\"reason\":\""<<c.reason()
    <<"\",\"armed\":"<<q.s.feedback.armed<<",\"offboard\":"<<q.s.feedback.offboard
    <<",\"z0\":"<<c.reference_z()<<",\"z\":"<<q.s.attitude.z
    <<",\"relative_height\":"<<c.reference_z()-q.s.attitude.z
    <<",\"reference_qualified\":"<<c.rc_ground_reference_qualified()
    <<",\"status_stamp\":"<<q.s.status_stamp<<",\"local_stamp\":"<<q.s.attitude.local_stamp
    <<",\"trigger_evidence_valid\":"<<e.valid<<",\"trigger_height\":"<<e.height
    <<",\"trigger_local_stamp\":"<<e.local_stamp
    <<",\"completed_attempts\":"<<q.r.rc_attempts_completed()<<"}\n";
}
int main(){
  for(double offset:{.09,.16}){Flight q(production_no_vision_rc_config());
    q.s.attitude.z=2-offset;q.warm();snapshot(offset==.09?"static_0.09":"static_0.16","ready",q);}
  Flight ground(production_no_vision_rc_config());ground.warm();snapshot("A","before",ground);
  ground.s.attitude.rc_arm_switch=1;ground.tick();ground.s.attitude.z-=.16;ground.tick();
  snapshot("A","trigger",ground);
  ground.s.attitude.z+=.07;ground.tick();snapshot("A","later_display",ground);
  ground.s.attitude.rc_arm_switch=3;for(int i=0;i<60;++i)ground.tick();
  need(ground.r.reprepare_rc(ground.s,ground.t),"replay A safe recovery");ground.warm();snapshot("A","next_ready",ground);
  Flight publication(production_no_vision_rc_config());publication.warm();snapshot("B","before",publication);
  unsigned calls=0,targets=0,heartbeats=0;a::PublicationAttempt attempt;std::string reason;
  try{a::publish_attitude_resampled<RcMode,RcTarget>([&]{
    auto c=publication.r.publication_context(publication.s,publication.t,true,++calls!=3);return c;
  },publication.o.attitude,publication.s.status_stamp,[&](const auto&){++targets;},
    [&](const auto&){++heartbeats;},&attempt);
  }catch(const std::exception& error){reason=std::string("PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_")+error.what();}
  need(attempt.decision.reason==a::BoundaryReason::ControlStall&&!attempt.sink_exception&&targets==1&&heartbeats==0,
    "B replay is a known post-target deadline refusal, not an unknown publisher exception");
  need(publication.r.abort_rc_preparation_timing(reason,publication.s),"replay B zero-preparation classification");
  publication.s.publication_fault_latched=true;snapshot("B","trigger",publication);
  std::cout<<"{\"scenario\":\"B\",\"point\":\"local_sink_evidence\",\"target_commits\":"<<targets
    <<",\"heartbeat_commits\":"<<heartbeats<<",\"stage\":\""<<a::name(attempt.stage)
    <<"\",\"sink_exception\":"<<attempt.sink_exception<<"}\n";
  for(int i=0;i<60;++i)publication.tick();
  need(publication.r.reprepare_rc(publication.s,publication.t),"replay B safe recovery");
  publication.s.publication_fault_latched=false;publication.warm();snapshot("B","next_ready",publication);
}
