#include "authority_test_support.hpp"
#include <iostream>
int main(){
  auto c=authority_test::config();fc01::Runtime r(c);auto s=authority_test::sample();
  std::cout<<"{"<<r.authority_json(s)<<"}\n";
  r.request("INTERACTIVE",{},s,0);r.request("AUTHORITY_AUTONOMOUS",{},s,0);
  std::cout<<"{"<<r.authority_json(s)<<"}\n";
  c.attitude.autonomous_failsafe_confirmed=true;fc01::Runtime live(c);double t=0;
  authority_test::ready(live,s,t,"AUTHORITY_AUTONOMOUS");
  std::cout<<"{"<<live.authority_json(s)<<"}\n";
  s.attitude.kill=true;std::cout<<"{"<<live.authority_json(s)<<"}\n";
  authority_test::tick(live,s,t);s.attitude.kill=false;live.abort_attitude("ROS_ADAPTER_CONTROL_LOOP_STALL");
  std::cout<<"{"<<live.authority_json(s)<<"}\n";
}
