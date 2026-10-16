#if __has_include("jikl_fc01_system/publication_boundary.hpp")
#include "jikl_fc01_system/publication_boundary.hpp"
#include "jikl_fc01_system/runtime.hpp"
#include <cassert>
#include <iostream>
#include <limits>
namespace a=fc01::attitude;
struct Mode {std::uint64_t timestamp{};bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};};
struct Target {std::uint64_t timestamp{};float yaw_sp_move_rate{};std::array<float,4> q_d{};std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};};
static a::BoundaryContext context(bool flight=false){
  a::BoundaryContext c;c.profile="attitude_altitude";c.phase=flight?a::Phase::RequestArm:a::Phase::Prestream;
  c.session_connected=c.control_fresh=c.reference_ready=true;c.flight_started=flight;c.operator_lease_live=flight;
  auto& s=c.feedback;s.attitude_fresh=s.local_fresh=s.status_fresh=s.land_fresh=s.failsafe_fresh=true;
  s.z_valid=s.vz_valid=s.landed=s.at_rest=s.exclusive=s.preflight=s.battery_ready=s.manual_ready=true;
  s.q={1,0,0,0};s.stationary_seconds=3;s.offboard=flight;s.stabilized_selectable=true;
  if(flight){auto& p=c.config;p.flight_actions_enabled=p.arm_enabled=p.takeoff_enabled=p.parameters_confirmed=true;
    p.autonomous_failsafe_confirmed=true;
    c.authority={fc01::authority::Mode::AutonomousNoRc,fc01::authority::Mode::AutonomousNoRc,true,false};
    p.vertical={.4,0,.7,1,1,.2,.1,.1,5,.5,.3};
    p.ramp_seconds=.5;p.settle_seconds=p.landing_seconds=p.height_tolerance=p.speed_tolerance=.1;
    p.ascent_timeout=p.descent_timeout=10;p.saturation_seconds=1;p.max_flight_seconds=60;p.max_tilt_deg=20;
  p.armed_dwell_seconds=.2;p.liftoff_dwell_seconds=.1;p.liftoff_height_m=.12;
  p.liftoff_speed_mps=.08;p.touchdown_dwell_seconds=.1;
  p.height_lower_margin_m=.25;p.z_jump_limit_m=1.2;p.vz_jump_limit_mps=2.0;
  }return c;
}
static void pair_denied(const a::BoundaryContext& c,const a::Output& o,const char* reason=nullptr){
  int target=0,heartbeat=0;
  const auto d=a::publish_attitude_at_boundary<Mode,Target>(c,o,123,[&](const Target&){++target;},[&](const Mode&){++heartbeat;});
  assert(!d.allowed&&target==0&&heartbeat==0);
  if(reason)assert(std::string(a::name(d.reason))==reason);
}
static void arm_denied(const a::BoundaryContext& c,const char* reason){
  int sent=0;const auto d=a::publish_arm_at_boundary(c,[&]{++sent;});
  assert(!d.allowed&&sent==0&&std::string(a::name(d.reason))==reason);
}
static void stage_one(){
  a::Output o;o.publish=true;
  for(auto phase:{a::Phase::Idle,a::Phase::SelfCheck,a::Phase::Prestream,a::Phase::RequestOffboard,a::Phase::OffboardReady}){
    auto c=context();c.phase=phase;c.feedback.offboard=phase==a::Phase::OffboardReady;
    for(int axis=0;axis<3;++axis)for(double value:{-.4,.4,std::numeric_limits<double>::denorm_min()}){
      o.thrust={0,0,0};o.thrust[axis]=value;pair_denied(c,o,"STAGE_ONE_NONZERO_THRUST");
    }
  }
  std::cout<<"stage_one_nonzero_thrust_rejected_at_final_boundary: PASS\n";
  for(auto phase:{a::Phase::Prestream,a::Phase::RequestOffboard,a::Phase::OffboardReady}){
    auto c=context();c.phase=phase;c.feedback.offboard=phase==a::Phase::OffboardReady;o.thrust={-0.,0,0};
    int target=0,heartbeat=0;
    auto d=a::publish_attitude_at_boundary<Mode,Target>(c,o,456,[&](const Target& t){
      ++target;for(float x:t.thrust_body)assert(x==0&&!std::signbit(x));assert(t.timestamp==456);
    },[&](const Mode& m){++heartbeat;assert(m.attitude&&!m.position&&!m.velocity&&!m.acceleration&&!m.body_rate&&!m.thrust_and_torque&&!m.direct_actuator);});
    assert(d.allowed&&target==1&&heartbeat==1);
  }
  std::cout<<"stage_one_zero_thrust_allowed_with_four_gates_false: PASS\n";
}
static void arm(){
  const char* reasons[]={"FLIGHT_ACTIONS_DISABLED","ARM_DISABLED","TAKEOFF_DISABLED","FLIGHT_PARAMETERS_NOT_CONFIRMED"};
  const char* tests[]={"arm_blocked_if_flight_actions_false","arm_blocked_if_arm_enabled_false","arm_blocked_if_takeoff_enabled_false","arm_blocked_if_parameters_confirmed_false"};
  for(int gate=0;gate<4;++gate){auto c=context(true);
    bool* gates[]={&c.config.flight_actions_enabled,&c.config.arm_enabled,&c.config.takeoff_enabled,&c.config.parameters_confirmed};
    *gates[gate]=false;arm_denied(c,reasons[gate]);std::cout<<tests[gate]<<": PASS\n";
  }
  auto c=context(true);c.phase=a::Phase::OffboardReady;arm_denied(c,"ARM_WRONG_PHASE");
  c=context(true);c.shutdown=true;arm_denied(c,"SHUTDOWN_LATCHED");
  c=context(true);c.operator_lease_live=false;arm_denied(c,"LIVE_OPERATOR_LEASE_REQUIRED");
  c=context(true);c.session_connected=false;arm_denied(c,"OPERATOR_SESSION_DISCONNECTED");
  c=context(true);c.feedback.armed=true;arm_denied(c,"ARM_REQUIRES_DISARMED");
  for(const std::string profile:{"ground","flight","sim","unknown"}){c=context(true);c.profile=profile;arm_denied(c,"WRONG_PROFILE");}
  c=context(true);c.feedback.preflight=false;arm_denied(c,"BATTERY_PREFLIGHT_REQUIRED");
  c=context(true);c.feedback.landed=false;arm_denied(c,"ARM_REQUIRES_OFFBOARD_GROUND_REFERENCE");
  c=context(true);int sent=0;auto d=a::publish_arm_at_boundary(c,[&]{++sent;});assert(d.allowed&&sent==1);
  std::cout<<"arm_phase_shutdown_lease_disarmed_health_and_all_inputs_policy_only: PASS\n";
}
static void thrust_and_supervision(){
  auto c=context(true);c.phase=a::Phase::Ascend;c.feedback.armed=true;
  a::Output o;o.publish=true;o.thrust={0,0,-.4};
  for(int mask=0;mask<15;++mask){auto x=c;x.config.flight_actions_enabled=mask&1;x.config.arm_enabled=mask&2;x.config.takeoff_enabled=mask&4;x.config.parameters_confirmed=mask&8;pair_denied(x,o);}
  for(auto phase:{a::Phase::Idle,a::Phase::SelfCheck,a::Phase::RequestArm,a::Phase::ArmedGround,a::Phase::RequestDisarm,a::Phase::SafeDisarmed,a::Phase::Safety}){auto x=c;x.phase=phase;pair_denied(x,o);}
  auto x=c;x.shutdown=true;pair_denied(x,o,"SHUTDOWN_LATCHED");
  x=c;x.control_fresh=false;pair_denied(x,o,"CONTROL_LOOP_STALL");
  x=c;x.feedback.attitude_fresh=false;pair_denied(x,o,"ATTITUDE_STALE_OR_INVALID");
  x=c;x.feedback.z_valid=false;pair_denied(x,o,"Z_VZ_STALE_OR_INVALID");
  x=c;x.feedback.failsafe=true;pair_denied(x,o,"PX4_FAILSAFE");
  x=c;x.feedback.exclusive=false;pair_denied(x,o,"PUBLISHER_OR_TELEMETRY_CONFLICT");
  x=c;x.operator_lease_live=false;pair_denied(x,o,"LIVE_OPERATOR_LEASE_REQUIRED");
  x=c;x.feedback.armed=false;pair_denied(x,o,"NONZERO_REQUIRES_ARMED_OFFBOARD");
  for(auto thrust:{std::array<double,3>{.1,0,-.4},std::array<double,3>{0,0,.1},std::array<double,3>{0,0,NAN}}){o.thrust=thrust;pair_denied(c,o);}
  o.thrust={0,0,-.4};int count=0;
  auto d=a::publish_attitude_at_boundary<Mode,Target>(c,o,1,[&](const Target& t){assert(t.thrust_body[2]<0);++count;},[&](const Mode&){++count;});assert(d.allowed&&count==2);
  // Existing emergency controlled descent can continue after operator lease loss.
  c.phase=a::Phase::Descent;c.session_connected=c.operator_lease_live=false;
  assert(a::attitude_publication_policy(c,o).allowed);
  o.publish=false;pair_denied(c,o,"OUTPUT_DISABLED");
  o.publish=true;o.thrust={0,0,0};c=context(true);c.phase=static_cast<a::Phase>(999);
  pair_denied(c,o,"OUTPUT_WRONG_PHASE");
  std::cout<<"nonzero_gate_truth_table_phase_shutdown_health_lease_and_heartbeat_supervision: PASS\n";
}
int main(){stage_one();arm();thrust_and_supervision();
  auto c=context();std::string json="{"+a::flight_gates_json(c.config)+"}";
  for(const char* field:{"flight_actions_enabled","arm_enabled","takeoff_enabled","flight_parameters_confirmed"})assert(json.find(std::string("\"")+field+"\":false")!=std::string::npos);
  assert(!a::publisher_plan("attitude_altitude").trajectory);
  std::cout<<"four_gates_json_and_attitude_profile_ownership: PASS\n";
}
#else
#include <cassert>
int main(){assert(!"final publisher guard missing (D I-01)");}
#endif
