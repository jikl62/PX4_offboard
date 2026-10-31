#if __has_include("jikl_fc01_system/attitude_reference.hpp")
#include "jikl_fc01_system/attitude_reference.hpp"
#include "jikl_fc01_system/vertical_controller.hpp"
#include "jikl_fc01_system/setpoint_ownership.hpp"
#include <cassert>
#include <iostream>
#include <limits>
namespace a=fc01::attitude;
static bool close(double x,double y){return std::abs(x-y)<1e-8;}
static a::VerticalConfig config(){
  a::VerticalConfig c;c.hover=.4;c.minimum=0;c.maximum=.6;c.slew=10;
  c.height_gain=1;c.velocity_gain=.1;c.integral_gain=.2;c.integral_limit=.1;
  c.max_height=5;c.max_climb=.5;c.max_descent=.3;return c;
}
int main(){
  a::AttitudeReference ref;
  assert(ref.observe({1,0,0,0},0,false)==a::ReferenceResult::Rebased);
  assert(close(ref.level()[0],1)&&close(ref.level()[1],0)&&close(ref.level()[2],0));
  const double half=std::sqrt(.5);
  assert(ref.observe({half,0,0,half},1,false)==a::ReferenceResult::Rebased);
  auto q=ref.level();assert(close(q[0],half)&&close(q[3],half));
  assert(ref.observe({1,0,0,0},1,false)==a::ReferenceResult::Healthy);
  assert(close(ref.yaw(),1.5707963267948966)); // fixed reference, not current yaw
  assert(ref.observe({1,0,0,0},2,true)==a::ReferenceResult::ActiveReset);
  assert(close(ref.yaw(),1.5707963267948966));
  assert(!a::normalized({0,0,0,0}));
  assert(!a::normalized({NAN,0,0,0}));assert(!a::normalized({INFINITY,0,0,0}));
  assert(!a::normalized({2,0,0,0}));assert(a::normalized({1.01,0,0,0}));
  a::AttitudeReference tilted;
  assert(tilted.observe({.8660254037844386,.5,0,0},0,false)==a::ReferenceResult::Rebased);
  assert(close(tilted.level()[0],1)&&close(tilted.level()[1],0));
  a::VerticalSample s;s.z0=3;s.z=1;s.vz=-.2;s.fresh=s.z_valid=s.vz_valid=true;
  a::VerticalController ctl(config());ctl.reset(.4);
  auto o=ctl.step(s,2,.05);
  assert(o.healthy&&close(o.height,2)&&close(o.speed,.2)&&o.thrust<.4);
  s.vz=0;ctl.reset(.4);o=ctl.step(s,2,.05);assert(close(o.thrust,.4));
  ctl.reset(.4);o=ctl.step(s,3,.05);assert(o.healthy&&o.thrust>.4&&close(o.velocity_target,.5));
  ctl.reset(.4);o=ctl.step(s,1,.05);assert(o.thrust<.4&&close(o.velocity_target,-.3));
  auto c=config();c.velocity_gain=10;c.integral_gain=1;
  a::VerticalController saturated(c);saturated.reset(.4);
  for(int i=0;i<30;++i){o=saturated.step(s,3,.05);assert(o.healthy&&close(o.thrust,.6));}
  assert(close(saturated.integral(),0)); // upper saturation must not wind up
  saturated.reset(.4);s.vz=-1;o=saturated.step(s,2,.05);assert(close(o.thrust,0));
  c=config();c.slew=.2;a::VerticalController slew(c);slew.reset(0);s.vz=0;
  o=slew.step(s,3,.05);assert(close(o.thrust,.01)&&o.slew_limited);
  s.fresh=false;assert(!slew.step(s,2,.05).healthy);s.fresh=true;
  s.z_valid=false;assert(!slew.step(s,2,.05).healthy);s.z_valid=true;
  s.vz_valid=false;assert(!slew.step(s,2,.05).healthy);s.vz_valid=true;
  s.z_reset=1;assert(!slew.step(s,2,.05).healthy);slew.reset();
  assert(slew.step(s,2,.05).healthy);s.vz_reset=1;assert(!slew.step(s,2,.05).healthy);
  s.vz_reset=0;s.z=NAN;assert(!ctl.step(s,2,.05).healthy);
  s.z=1;s.vz=INFINITY;assert(!ctl.step(s,2,.05).healthy);s.vz=0;
  assert(!ctl.step(s,NAN,.05).healthy);assert(!ctl.step(s,2,0).healthy);
  s.z=-3;assert(!ctl.step(s,2,.05).healthy); // h=6 > max_height=5
  s.z=3.3;assert(!ctl.step(s,0,.05).healthy); // h=-.3 < -.25 ground margin
  s.z=3.2;ctl.reset(.4);assert(ctl.step(s,0,.05).healthy);
  bool rejected=false;try{a::VerticalController invalid(a::VerticalConfig{});}catch(const std::invalid_argument&){rejected=true;}
  assert(rejected);
  a::OwnershipCounts own;own.mode_publishers=own.mode_subscribers=1;
  own.attitude_publishers=own.attitude_subscribers=1;
  assert(a::owns_setpoints("attitude_altitude",own));
  own.trajectory_publishers=1;assert(!a::owns_setpoints("attitude_altitude",own));
  own.trajectory_subscribers=1;assert(!a::owns_setpoints("flight",own));
  own.attitude_publishers=0;assert(a::owns_setpoints("flight",own));
  own.mode_publishers=own.trajectory_publishers=0;assert(a::owns_setpoints("ground",own));
  own.other_publishers=1;assert(!a::owns_setpoints("ground",own));
  std::cout<<"attitude/vertical/ownership tests: PASS (quaternion, resets, signs, feed-forward, limits, anti-windup, faults)\n";
}
#else
#include <cassert>
int main(){assert(!"attitude/vertical implementation required by contract is absent");}
#endif
