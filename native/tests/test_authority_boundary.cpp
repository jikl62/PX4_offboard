// Mutations caught: removing authority checks at either real final sink;
// trusting manual_ready, one confirmation, or a changed/unlocked attempt.
#include "jikl_fc01_system/publication_boundary.hpp"
#include <cassert>
#include <iostream>
#include <limits>
namespace a=fc01::attitude;
struct Mode {std::uint64_t timestamp{};bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};};
struct Target {std::uint64_t timestamp{};float yaw_sp_move_rate{};std::array<float,4> q_d{};std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};};
static a::BoundaryContext complete(){
  a::BoundaryContext c;c.profile="attitude_altitude";c.phase=a::Phase::RequestArm;
  c.session_connected=c.control_fresh=c.reference_ready=c.flight_started=c.operator_lease_live=true;
  auto& s=c.feedback;s.attitude_fresh=s.local_fresh=s.status_fresh=s.land_fresh=s.failsafe_fresh=true;
  s.z_valid=s.vz_valid=s.landed=s.at_rest=s.exclusive=s.preflight=s.battery_ready=s.manual_ready=true;
  s.offboard=true;s.stationary_seconds=3;
  auto& p=c.config;p.flight_actions_enabled=p.arm_enabled=p.takeoff_enabled=p.parameters_confirmed=true;
  p.vertical={.4,0,.7,1,1,.2,.1,.1,5,.5,.3};
  p.ramp_seconds=.5;p.settle_seconds=p.landing_seconds=p.height_tolerance=p.speed_tolerance=.1;
  p.ascent_timeout=p.descent_timeout=10;p.saturation_seconds=1;p.max_flight_seconds=60;p.max_tilt_deg=20;
  p.armed_dwell_seconds=.2;p.liftoff_dwell_seconds=.1;p.liftoff_height_m=.12;
  p.liftoff_speed_mps=.08;p.touchdown_dwell_seconds=.1;
  p.height_lower_margin_m=.25;p.z_jump_limit_m=1.2;p.vz_jump_limit_mps=2.0;
  return c;
}
static void denied(a::BoundaryContext c,const char* expected){
  int arms=0,targets=0,modes=0;
  auto d=a::publish_arm_at_boundary(c,[&]{++arms;});
  assert(!d.allowed&&arms==0);
  if(expected)assert(std::string(a::name(d.reason))==expected);
  c.phase=a::Phase::Ascend;c.feedback.armed=true;
  a::Output o;o.publish=true;o.thrust={0,0,-.4};
  d=a::publish_attitude_at_boundary<Mode,Target>(c,o,42,[&](const Target&){++targets;},[&](const Mode&){++modes;});
  assert(!d.allowed&&targets==0&&modes==0);
  if(expected)assert(std::string(a::name(d.reason))==expected);
}
int main(){
  denied(complete(),"AUTHORITY_MODE_UNSET"); // actual baseline RED: Arm was allowed
#if __has_include("jikl_fc01_system/authority.hpp")
  namespace auth=fc01::authority;
  auto c=complete();
  c.authority={auth::Mode::AutonomousNoRc,auth::Mode::AutonomousNoRc,true,false};
  denied(c,"AUTONOMOUS_FAILSAFE_UNCONFIRMED");
  c.config.autonomous_failsafe_confirmed=true;
  int arms=0;assert(a::publish_arm_at_boundary(c,[&]{++arms;}).allowed&&arms==1);
  c.phase=a::Phase::Ascend;c.feedback.armed=true;
  a::Output o;o.publish=true;o.thrust={0,0,-.4};int targets=0,modes=0;
  const auto published=a::publish_attitude_at_boundary<Mode,Target>(c,o,1,[&](const Target&){++targets;},[&](const Mode&){++modes;});
  assert(published.allowed);
  assert(targets==1&&modes==1&&!c.feedback.rc_link_ready&&!c.feedback.rc_switches_ready);
  c=complete();c.authority={auth::Mode::RcTakeover,auth::Mode::RcTakeover,true,false};
  denied(c,"RC_LINK_REQUIRED");
  c.feedback.rc_link_ready=true;denied(c,"RC_TAKEOVER_CONTRACT_UNCONFIRMED");
  c.config.rc_takeover_confirmed=true;denied(c,"RC_TAKEOVER_TELEMETRY_REQUIRED");
  c.feedback.rc_switches_ready=true;
  assert(a::arm_publication_policy(c).allowed);
  c.authority.locked=false;denied(c,"AUTHORITY_MODE_NOT_LOCKED");
  c.authority.locked=true;c.authority.attempt_mode=auth::Mode::AutonomousNoRc;
  denied(c,"AUTHORITY_CHANGED_DURING_ATTEMPT");
  c.authority.attempt_mode=c.authority.mode;c.authority.takeover_latched=true;
  denied(c,"OPERATOR_TAKEOVER_LATCHED");
  c.authority.takeover_latched=false;c.feedback.kill=true;denied(c,"OPERATOR_TAKEOVER_LATCHED");
  c.feedback.kill=false;c.feedback.operator_takeover=true;denied(c,"OPERATOR_TAKEOVER_LATCHED");
  c.feedback.operator_takeover=false;c.feedback.offboard_signal_lost=true;
  denied(c,"OPERATOR_TAKEOVER_LATCHED");
  c.feedback.offboard_signal_lost=false;
  for(int gate=0;gate<4;++gate){auto x=c;
    bool* flags[]={&x.config.flight_actions_enabled,&x.config.arm_enabled,&x.config.takeoff_enabled,&x.config.parameters_confirmed};
    *flags[gate]=false;denied(x,nullptr);
  }
  c=complete();c.phase=a::Phase::Prestream;c.authority={};c.config={};c.flight_started=false;c.feedback.offboard=false;
  o.thrust={0,0,0};assert(a::attitude_publication_policy(c,o).allowed);
  o.thrust[2]=-std::numeric_limits<double>::denorm_min();
  assert(a::attitude_publication_policy(c,o).reason==a::BoundaryReason::StageOneNonzero);
  c=complete();c.authority.mode=static_cast<auth::Mode>(999);c.authority.attempt_mode=c.authority.mode;c.authority.locked=true;
  denied(c,"INVALID_AUTHORITY_MODE");
#endif
  std::cout<<"authority final Arm/nonzero sinks, both routes, lock identity and exact-zero regression: PASS\n";
}
