// M1 safety regression: fully synthetic, never touches devices or ROS graph.
#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <cassert>
#include <iostream>
namespace a=fc01::attitude;
static a::BoundaryContext flight_context(){
  auto c=authority_test::config();c.attitude.autonomous_failsafe_confirmed=true;fc01::Runtime r(c);
  auto s=authority_test::sample();auto b=r.publication_context(s,0,true,true);
  b.phase=a::Phase::RequestDisarm;
  b.authority={fc01::authority::Mode::AutonomousNoRc,
    fc01::authority::Mode::AutonomousNoRc,true,false};
  b.session_connected=b.control_fresh=b.operator_lease_live=true;
  b.reference_ready=b.flight_started=b.arm_confirmed=b.ground_confirmed=true;
  b.reference_z=2;b.yaw_reference=0;b.touchdown_baseline=500;
  b.feedback.armed=b.feedback.offboard=b.feedback.battery_ready=true;
  b.feedback.landed=b.feedback.at_rest=true;
  b.feedback.stationary_seconds=3;
  b.feedback.land_stamp=2000;b.feedback.status_stamp=3000;
  return b;
}
static void disarm_sink_toctou(){
  const auto good=flight_context();int sink=0;
  assert(a::publish_disarm_at_boundary(good,[&]{++sink;}).allowed&&sink==1);
  auto deny=[&](a::BoundaryContext sample){
    const auto before=sink;
    assert(!a::publish_disarm_at_boundary(sample,[&]{++sink;}).allowed);
    assert(sink==before);
  };
  for(auto phase:{a::Phase::RequestArm,a::Phase::Landed,a::Phase::Safety,a::Phase::SafeDisarmed}){
    auto c=good;c.phase=phase;deny(c);
  }
  {auto c=good;c.feedback.landed=false;deny(c);} // contact lost between core and sink
  {auto c=good;c.feedback.at_rest=false;deny(c);}
  {auto c=good;c.feedback.armed=false;deny(c);}
  {auto c=good;c.feedback.offboard=false;deny(c);}
  {auto c=good;c.feedback.land_fresh=false;deny(c);}
  {auto c=good;c.feedback.status_fresh=false;deny(c);}
  {auto c=good;c.feedback.z+=1;deny(c);}
  {auto c=good;c.feedback.vz=.3;deny(c);}
  {auto c=good;c.feedback.kill=true;deny(c);}
  {auto c=good;c.feedback.exclusive=false;deny(c);}
  {auto c=good;c.feedback.land_stamp=500;deny(c);} // no post-baseline contact
  {auto c=good;c.arm_confirmed=false;deny(c);}
  {auto c=good;c.ground_confirmed=false;deny(c);}
  {auto c=good;c.config.takeoff_enabled=false;deny(c);}
  {auto c=good;c.feedback.time_regressed=true;deny(c);}
  {auto c=good;c.shutdown=true;assert(a::disarm_publication_policy(c).allowed);} // planned grounded shutdown
  std::cout<<"M1 disarm final-sink TOCTOU and transaction evidence: PASS\n";
}
static void commanded_quaternion_attack(){
  auto c=flight_context();c.phase=a::Phase::HeightHold;c.feedback.landed=false;
  c.feedback.at_rest=false;c.reference_z=2;c.feedback.z=1.;
  a::Output out;out.publish=true;out.thrust={0,0,-.4};out.q={1,0,0,0};
  assert(a::attitude_publication_policy(c,out).allowed);
  out.q={std::sqrt(.5),std::sqrt(.5),0,0};
  int targets=0,modes=0;
  struct Mode {std::uint64_t timestamp{};bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};};
  struct Target {std::uint64_t timestamp{};float yaw_sp_move_rate{};std::array<float,4> q_d{};std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};};
  const auto result=a::publish_attitude_at_boundary<Mode,Target>(c,out,42,
    [&](const Target&){++targets;},[&](const Mode&){++modes;});
  assert(!result.allowed&&result.reason==a::BoundaryReason::CommandedAttitude);
  assert(targets==0&&modes==0);
  out.q={std::cos(.25),0,0,std::sin(.25)}; // yaw attack with level roll/pitch
  assert(a::attitude_publication_policy(c,out).reason==a::BoundaryReason::CommandedAttitude);
  c.shutdown=true;assert(!a::attitude_publication_policy(c,out).allowed);
  std::cout<<"M1 nonzero commanded q 90-degree roll / yaw injection: PASS\n";
}
static void ground_only_release(){
  auto c=flight_context();c.config=a::Config{};
  c.phase=a::Phase::ReleaseOffboard;c.feedback.armed=false;c.flight_started=false;
  c.feedback.stabilized_selectable=true;c.shutdown=true;
  a::Output zero;zero.publish=true;zero.q={1,0,0,0};
  assert(a::attitude_publication_policy(c,zero).allowed);
  zero.thrust={0,0,-.1};
  assert(a::attitude_publication_policy(c,zero).reason==a::BoundaryReason::StageOneNonzero);
  zero.thrust={};
  int sent=0;assert(a::publish_release_at_boundary(c,[&]{++sent;}).allowed);
  assert(sent==1);
  auto denied=[&](a::BoundaryContext x){assert(!a::publish_release_at_boundary(x,[&]{++sent;}).allowed&&sent==1);};
  {auto x=c;x.feedback.landed=false;denied(x);}
  {auto x=c;x.feedback.armed=true;denied(x);}
  {auto x=c;x.feedback.land_fresh=false;denied(x);}
  {auto x=c;x.feedback.status_fresh=false;denied(x);}
  {auto x=c;x.feedback.offboard=false;denied(x);}
  {auto x=c;x.feedback.stabilized_selectable=false;denied(x);}
  {auto x=c;x.feedback.z+=.5;denied(x);}
  {auto x=c;x.phase=a::Phase::Descent;denied(x);}
  std::cout<<"M1 Offboard release mode command is disarmed-ground-only: PASS\n";
}
static void armed_dwell_no_fake_liftoff_and_jump(){
  auto c=authority_test::config();c.attitude.autonomous_failsafe_confirmed=true;
  fc01::Runtime runtime(c);auto s=authority_test::sample();double now=0;
  authority_test::armed(runtime,s,now,"AUTHORITY_AUTONOMOUS");
  s.at_rest=false; // observed PX4 armed behavior can be false
  for(int k=0;k<3;++k){auto output=authority_test::tick(runtime,s,now);
    assert(runtime.attitude_core().phase()==a::Phase::ArmedGround);
    assert(output.attitude.publish&&output.attitude.thrust[2]==0);
  }
  for(int k=0;k<10;++k){authority_test::tick(runtime,s,now);}
  assert(runtime.attitude_core().phase()==a::Phase::ThrustRamp);
  for(int k=0;k<30;++k){authority_test::tick(runtime,s,now);
    assert(runtime.attitude_core().phase()==a::Phase::ThrustRamp);
  }
  assert(runtime.attitude_core().fault_class()==a::FaultClass::None);
  for(int k=0;k<210&&runtime.attitude_core().phase()!=a::Phase::RequestDisarm;++k)
    authority_test::tick(runtime,s,now);
  // Timeout with motor power must NOT silently go to Safety.
  // With at_rest=false it keeps sending bounded zero/reducing thrust until
  // additional independent touchdown / at_rest samples arrive.
  assert(runtime.attitude_core().phase()==a::Phase::PoweredGround);
  assert(runtime.attitude_core().fault_class()==a::FaultClass::GroundAbort);
  const auto recovering=authority_test::tick(runtime,s,now);
  assert(recovering.attitude.publish&&recovering.attitude.thrust[2]==0);
  s.at_rest=true;
  for(int k=0;k<30&&runtime.attitude_core().phase()!=a::Phase::RequestDisarm;++k)
    authority_test::tick(runtime,s,now);
  assert(runtime.attitude_core().phase()==a::Phase::RequestDisarm);
  fc01::Runtime broken(c);s=authority_test::sample();now=0;
  authority_test::armed(broken,s,now,"AUTHORITY_AUTONOMOUS");
  authority_test::tick(broken,s,now);s.attitude.z-=1.25;
  auto o=authority_test::tick(broken,s,now);
  assert(!o.attitude.publish&&broken.attitude_core().phase()==a::Phase::Safety);
  assert(broken.attitude_core().fault_class()==a::FaultClass::UnknownAirborne);
  std::cout<<"M1 dwell, armed at_rest=false, no timer-only liftoff, jump abort: PASS\n";
}
static void pending_offboard_stop_release_ack(){
  fc01::Runtime r(authority_test::config());auto s=authority_test::sample();
  s.stabilized_selectable=true;double now=0;
  assert(r.request("ATTITUDE_READY",{},s,now).accepted);
  r.step(s,now);
  for(int k=0;k<30&&r.attitude_core().phase()!=a::Phase::RequestOffboard;++k)
    authority_test::tick(r,s,now);
  assert(r.attitude_core().phase()==a::Phase::RequestOffboard);
  assert(r.pending(176).sent);
  assert(r.request("STOP",{},s,now).accepted);
  r.begin_shutdown(s,now);
  assert(r.attitude_core().ground_release_pending());
  auto out=authority_test::tick(r,s,now);
  assert(out.attitude.publish&&out.attitude.thrust[2]==0);
  const auto first_ack=r.pending(176).baseline;
  r.ack({first_ack+1,176,0,1,191,false},now+.001);
  s.feedback.offboard=true;s.nav_state=14;
  out=authority_test::tick(r,s,now);
  assert(r.attitude_core().phase()==a::Phase::ReleaseOffboard);
  assert(out.attitude.command==a::Command::ReleaseOffboard&&out.attitude.publish);
  auto c=r.publication_context(s,now,true,true,true);
  assert(a::attitude_publication_policy(c,out.attitude).allowed);
  assert(a::publish_release_at_boundary(c,[]{}).allowed);
  const auto release_ack=r.pending(176).baseline;
  r.ack({release_ack,176,0,1,191,false},now+.001);
  assert(!r.pending(176).accepted);
  r.ack({release_ack+1,176,0,1,191,false},now+.002);
  assert(r.pending(176).accepted);
  s.feedback.offboard=false;s.nav_state=15;
  out=authority_test::tick(r,s,now);
  assert(r.attitude_core().phase()==a::Phase::Idle&&!out.attitude.publish);
  assert(!r.pending(400).sent);
  std::cout<<"M1 queued STOP / mode 176 ACK-status handshake / zero release: PASS\n";
}
int main(){disarm_sink_toctou();commanded_quaternion_attack();ground_only_release();armed_dwell_no_fake_liftoff_and_jump();pending_offboard_stop_release_ack();}
