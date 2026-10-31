// Offline stdin/stdout only; uses the production C++ guard, no ROS connection.
#include "jikl_fc01_system/vertical_feedback_guard.hpp"
#include <iomanip>
#include <iostream>
int main(){
  fc01::attitude::VerticalFeedbackConfig c;c.enabled=true;
  fc01::attitude::VerticalFeedbackGuard guard(c);
  std::uint64_t stamp;double z,vz,z_deriv;
  std::cout<<std::setprecision(12);
  while(std::cin>>stamp>>z>>vz>>z_deriv){
    const auto& e=guard.observe(stamp,z,vz,z_deriv,.5);
    std::cout<<stamp<<','<<e.healthy<<','<<e.suspect<<','<<e.rate_error_mps<<','
      <<e.position_rate_error_mps<<','<<e.disagreement_seconds<<','<<e.reason<<'\n';
    if(!e.healthy)break;
  }
}
