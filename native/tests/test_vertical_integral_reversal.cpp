#include "jikl_fc01_system/vertical_controller.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
namespace a=fc01::attitude;
static void require_integral(bool ok,const char* message){
  if(!ok){std::cerr<<"INTEGRAL_REVERSAL_FAIL: "<<message<<'\n';std::exit(1);}
}
static a::VerticalConfig config(){
  a::VerticalConfig c;c.hover=.475;c.minimum=.12;c.maximum=.75;c.slew=.4;
  c.height_gain=.5;c.velocity_gain=.15;c.integral_gain=.03;c.integral_limit=.08;
  c.max_height=5;c.max_climb=.3;c.max_descent=.2;return c;
}
static void reversal_does_not_wind_through_zero(double sign,bool output_saturation){
  auto c=config();if(output_saturation)c.slew=100;
  a::VerticalController ctl(c);ctl.reset(.475);
  a::VerticalSample s{0,-1,0,true,true,true,0,0};
  const double target=1.+sign*.2;
  auto o=ctl.step(s,target,.05);
  require_integral(o.healthy&&sign*ctl.integral()>0,"seed a real unsaturated integral");
  s.vz=-sign*(output_saturation?4.:1.);
  o=ctl.step(s,target,.05);
  require_integral(o.healthy&&(output_saturation?o.saturated:o.slew_limited),
    "large reversal must respect the active output or slew constraint");
  require_integral(std::abs(ctl.integral())<1e-12,
    "unwinding under a constraint must stop at zero before accumulating the opposite sign");
  const double prior=o.thrust;
  o=ctl.step(s,target,.05);
  require_integral(std::abs(ctl.integral())<1e-12,"continued slew constraint must freeze integration");
  require_integral(std::abs(o.thrust-prior)<=c.slew*.05+1e-12,"physical thrust slew retained");
}
int main(){
  for(bool saturated:{false,true}){
    reversal_does_not_wind_through_zero(1.,saturated);
    reversal_does_not_wind_through_zero(-1.,saturated);
  }
  std::cout<<"VERTICAL_INTEGRAL_REVERSAL_PASS\n";
}
