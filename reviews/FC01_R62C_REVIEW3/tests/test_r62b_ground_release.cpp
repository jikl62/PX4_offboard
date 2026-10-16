// R6.2b: unarmed, ground-only Offboard release races. No real ROS/flight actions.
#define main C_S10_original_main_unused
#include "c_s10_independent/C_S10_SAFETY_ATTACKS.cpp"
#undef main
#include <iostream>
#include <string>

static a::BoundaryContext ground_ctx(){
  auto c=a::BoundaryContext{};c.profile="attitude_altitude";
  c.phase=a::Phase::ReleaseOffboard;
  c.control_fresh=c.reference_ready=c.offboard_observed=true;
  c.reference_z=2.;c.config.height_tolerance=.15;
  c.config.speed_tolerance=.15;
  c.config.ground_vz_tolerance=.1;
  auto &s=c.feedback;
  s.attitude_fresh=s.local_fresh=s.status_fresh=s.land_fresh=s.failsafe_fresh=true;
  s.z_valid=s.vz_valid=s.exclusive=s.offboard=s.landed=s.at_rest=true;
  s.stabilized_selectable=true;s.q={1,0,0,0};s.z=1.73;s.vz=.02;
  s.stationary_seconds=3;s.status_stamp=s.land_stamp=1000;
  return c;
}
static void unarmed_ground_stale_z0_release_allowed(){
  auto c=ground_ctx();
  int sends=0;
  const auto r=a::publish_release_at_boundary(c,[&]{++sends;});
  assert(r.allowed&&sends==1);
  a::Output out;out.publish=true;out.q={1,0,0,0};out.thrust={0,0,0};
  int targets=0,heartbeats=0;
  const auto d=a::publish_attitude_at_boundary<Mode,Target>(c,out,123,
    [&](const Target&){++targets;},[&](const Mode&){++heartbeats;});
  assert(d.allowed&&targets==1&&heartbeats==1);
  std::cout<<"R62B_UNARMED_GROUND_STALE_Z0_RELEASE PASS\n";
}
static void armed_and_unhealthy_release_still_denied(){
  const auto base=ground_ctx();
  auto denied=[](const a::BoundaryContext& c){
    int count=0;
    assert(!a::publish_release_at_boundary(c,[&]{++count;}).allowed&&count==0);
  };
  {auto c=base;c.flight_started=true;denied(c);}
  {auto c=base;c.arm_confirmed=true;denied(c);}
  {auto c=base;c.feedback.armed=true;denied(c);}
  {auto c=base;c.feedback.offboard=false;denied(c);}
  {auto c=base;c.feedback.landed=false;denied(c);}
  {auto c=base;c.feedback.at_rest=false;denied(c);}
  {auto c=base;c.feedback.vz=.16;denied(c);}
  {auto c=base;c.feedback.exclusive=false;denied(c);}
  {auto c=base;c.feedback.status_fresh=false;denied(c);}
  {auto c=base;c.feedback.z_valid=false;denied(c);}
  {auto c=base;c.feedback.kill=true;denied(c);}
  std::cout<<"R62B_ARMED_AND_UNHEALTHY_RELEASE_DENIED PASS\n";
}
static void external_mode_departure_is_cancellation_not_command(){
  auto c=ground_ctx();c.feedback.offboard=false;
  assert(a::external_ground_mode_departure(c));
  c.feedback.armed=true;assert(!a::external_ground_mode_departure(c));c.feedback.armed=false;
  c.feedback.landed=false;assert(!a::external_ground_mode_departure(c));c.feedback.landed=true;
  c.flight_started=true;assert(!a::external_ground_mode_departure(c));c.flight_started=false;
  c.arm_confirmed=true;assert(!a::external_ground_mode_departure(c));c.arm_confirmed=false;
  c.feedback.status_fresh=false;assert(!a::external_ground_mode_departure(c));c.feedback.status_fresh=true;
  c.feedback.offboard=true;assert(!a::external_ground_mode_departure(c));
  std::cout<<"R62B_FINAL_SINK_EXTERNAL_MODE_CANCELLATION_PREDICATE PASS\n";
}
static void core_observed_external_altitude_never_plans_release(){
  Harness h;h.ready();
  assert(h.r.request("LAND",{},h.s,h.t).accepted);
  assert(h.r.attitude_core().phase()==a::Phase::ReleaseOffboard);
  const auto before=h.mode_commands;
  // PX4/RC changes to Altitude before the next Core tick; fresh status.
  h.s.feedback.offboard=false;h.s.nav_state=1;
  auto out=h.tick();
  assert(!out.publish&&out.command==a::Command::None);
  assert(h.mode_commands==before);
  assert(h.r.attitude_core().phase()==a::Phase::Idle);
  assert(h.r.attitude_core().reason()==
    "EXTERNAL_DISARMED_GROUND_MODE_OBSERVED_NO_RELEASE_176");
  std::cout<<"R62B_CORE_EXTERNAL_ALTITUDE_DISARMED_NO_EXTRA_176 PASS\n";
}
int main(){
  unarmed_ground_stale_z0_release_allowed();
  armed_and_unhealthy_release_still_denied();
  external_mode_departure_is_cancellation_not_command();
  core_observed_external_altitude_never_plans_release();
  std::cout<<"R62B_GROUND_RELEASE_RACE_TESTS_PASS\n";
}
