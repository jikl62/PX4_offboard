#include "jikl_fc01_system/publication_audit.hpp"
#include <cassert>
#include <iostream>
int main(){
  fc01::attitude::PublicationAudit a;
  a.begin(1.0);a.target_committed(1.0,-.42);a.mode_committed(1.01);
  assert(a.last_target_thrust_z==-.42);
  assert(a.target_count==1&&a.mode_count==1&&a.target_this_tick&&a.mode_this_tick);
  a.begin(1.05);a.target_committed(1.05,-.39);a.failed();
  assert(a.last_target_thrust_z==-.39);
  assert(a.target_count==2&&a.mode_count==1&&a.target_this_tick&&!a.mode_this_tick&&a.exception_this_tick);
  a.begin(1.10);assert(!a.target_this_tick&&!a.mode_this_tick&&!a.exception_this_tick);
  assert(a.target_count==2&&a.mode_count==1);
  a.mode_committed(1.4);assert(a.max_mode_gap>.3&&a.mode_gap_violations==1);
  a.begin(2);a.failed();assert(a.target_count==2&&a.mode_count==2);
  std::cout<<"R6_PUBLICATION_AUDIT independent target/mode commits, partial exception and heartbeat gap PASS\n";
}
