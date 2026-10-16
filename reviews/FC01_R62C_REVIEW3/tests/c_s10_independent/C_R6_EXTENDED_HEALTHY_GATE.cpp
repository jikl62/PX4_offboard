// New gate; original C-S10 files are unchanged. This deliberately extends
// the original finite observation beyond R6's TOTAL recovery budget.
// Expected RED until independent fallback/physical custody is proven.
#define main C_S10_original_driver_not_called
#include "C_S10_SAFETY_ATTACKS.cpp"
#undef main
#include <cstdio>
int main(){
  Harness h;h.ramp();h.s.feedback.landed=false;h.s.at_rest=false;
  h.s.attitude.z=1.97;h.s.attitude.vz=-.02;
  for(int i=0;i<1000;++i){const auto a=h.attitude_sinks,m=h.mode_sinks;auto o=h.tick();
    if(h.s.feedback.armed&&h.s.feedback.offboard&&
       (!o.publish||h.attitude_sinks!=a+1||h.mode_sinks!=m+1)){
      std::fprintf(stderr,"R6_EXTENDED_OWNERSHIP_GATE RED t=%.2f phase=%s reason=%s Armed+Offboard retained, no independently verified PX4 fallback\n",
        h.t,h.r.phase_name(),h.r.attitude_core().reason().c_str());return 1;
    }
  }
  std::puts("R6_EXTENDED_OWNERSHIP_GATE GREEN");return 0;
}
