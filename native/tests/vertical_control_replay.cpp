// Fixed historical vertical inputs through the real Runtime, with synthetic
// level attitude and pre-approved test authority. No ROS or publication sink.
// This is a controller comparison, never a reconstruction of actual motion.
#include "ramp_guard_test_support.hpp"
#include <iomanip>
int main(){
  auto c=guard_config();c.attitude.feedback_guard.enabled=true;
  GuardFlight q(.5,c);
  double dt=0,height=0,vz=0,z_deriv=0;std::uint64_t status=0,land=0,local=0,raw=0;
  int landed=0,rest=0;double elapsed=0;
  std::cout<<std::setprecision(12);
  while(std::cin>>dt>>height>>vz>>z_deriv>>status>>land>>local>>raw>>landed>>rest){
    q.observe(height,-vz);q.s.attitude.z_deriv=z_deriv;
    q.s.status_stamp=status;q.s.land_stamp=land;
    q.s.attitude.local_stamp=local;q.s.attitude.vertical_sample_stamp=raw;
    q.s.feedback.landed=landed!=0;q.s.at_rest=rest!=0;
    require_guard(q.r.request("PING",{},q.s,q.t).accepted,"historical replay console lease");
    q.o=q.r.step(q.s,q.t+=dt);elapsed+=dt;
    const auto& o=q.o.attitude;
    std::cout<<elapsed<<','<<a::name(q.r.attitude_core().phase())<<','<<o.publish<<','
      <<-o.thrust[2]<<','<<o.target_height<<','<<o.velocity_target;
#ifdef REFINEMENT_DIAGNOSTICS
    std::cout<<','<<o.vertical_closed_loop<<','<<o.takeoff_thrust_limited<<','
      <<o.proportional_thrust<<','<<o.integral_thrust<<','<<o.raw_thrust<<','<<o.thrust_ceiling;
#else
    std::cout<<",0,0,0,0,0,0";
#endif
    std::cout<<','<<q.r.attitude_core().reason()<<'\n';
    if(q.r.attitude_core().phase()==a::Phase::Safety)break;
  }
}
