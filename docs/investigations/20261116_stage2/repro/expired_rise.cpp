#include "ramp_guard_test_support.hpp"
#include <iostream>
int main() {
 GuardFlight q;
 q.ground_ramp(3.);
 for(int i=0;i<8;++i) {q.observe(.13+.01*i,.2); q.advance(.05,false,true);}
 std::cout<<"rise="<<q.r.attitude_core().liftoff_evidence().rise_confirmed<<" phase="<<int(q.r.attitude_core().phase())<<"\n";
 q.observe(.20,0);
 // .35s last-rise + 2.25s fresh stationary local = age2.25, then freeze local for .3s.
 for(int i=0;i<45;++i) q.advance(.05,false,true);
 q.s.feedback.landed=false; q.s.at_rest=false;
 q.advance(.05,true,false);
 for(int i=0;i<5;++i) q.advance(.05,i==4,false);
 auto e=q.r.attitude_core().liftoff_evidence();
 std::cout<<"after_expiry rise="<<e.rise_confirmed<<" detector="<<e.detector_confirmed<<" confirmed="<<e.confirmed<<" phase="<<int(q.r.attitude_core().phase())<<" reason="<<q.r.attitude_core().reason()<<"\n";
 return q.r.attitude_core().liftoff_confirmed()?1:0;
}
