// Independent Agent C: POSITIVE property gate. Expected RED on original S10.
// Reuse C's own read-only synthetic fixture, not an original B/C fixture.
#define main C_S10_repro_driver_not_called
#include "C_S10_SAFETY_ATTACKS.cpp"
#undef main
#include <cstdio>
int main(int argc,char** argv){
  const std::string scenario=argc>1?argv[1]:"timeout";
  Harness h;h.ramp();h.s.feedback.landed=false;h.s.at_rest=false;
  h.s.attitude.z=1.97;h.s.attitude.vz=-.02;
  if(scenario=="shutdown")h.r.begin_shutdown(h.s,h.t);
  else if(scenario!="timeout")return 64;
  // Under valid attitude+Z/VZ with PX4 *still* reporting Armed+Offboard,
  // the companion must not abandon the two streams without a verified handoff.
  for(int i=0;i<460;++i){
    const auto before_target=h.attitude_sinks,before_mode=h.mode_sinks;
    const auto out=h.tick();
    const bool healthy=h.s.status_fresh&&h.s.land_fresh&&h.s.local_fresh&&
      h.s.attitude.z_valid&&h.s.attitude.vz_valid&&h.s.attitude.attitude_fresh&&
      h.s.attitude.failsafe_fresh&&!h.s.attitude.failsafe;
    if(healthy&&h.s.feedback.armed&&h.s.feedback.offboard&&
       (!out.publish||h.attitude_sinks!=before_target+1||h.mode_sinks!=before_mode+1)){
      std::fprintf(stderr,"POSITIVE_SAFETY_GATE FAIL [%s] t=%.2f phase=%s reason=%s armed=%d offboard=%d out.publish=%d delta_attitude=%lu delta_heartbeat=%lu\n",
        scenario.c_str(),h.t,h.r.phase_name(),h.r.attitude_core().reason().c_str(),
        int(h.s.feedback.armed),int(h.s.feedback.offboard),int(out.publish),
        h.attitude_sinks-before_target,h.mode_sinks-before_mode);
      return 1;
    }
  }
  std::puts("POSITIVE_SAFETY_GATE PASS: healthy Armed+Offboard retained both active sinks (no observed PX4 handoff)");
  return 0;
}
