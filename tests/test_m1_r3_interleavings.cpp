// R3 synthetic-only state/command/sink interleavings. No ROS graph or FC01.
#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <cassert>
#include <iostream>
namespace f=fc01;
namespace a=f::attitude;
struct Mode {std::uint64_t timestamp{};bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};};
struct Target {std::uint64_t timestamp{};float yaw_sp_move_rate{};std::array<float,4> q_d{};std::array<float,3> thrust_body{};bool reset_integral{},fw_control_yaw_wheel{};};
static f::Config config(){auto c=authority_test::config();c.attitude.autonomous_failsafe_confirmed=true;return c;}
static void final_heartbeat(const f::Runtime& r,const f::Snapshot& s,double t,const a::Output& out,bool shutdown=false,bool connected=true){
  auto c=r.publication_context(s,t,connected,true,shutdown);
  int target=0,mode=0;
  const auto decision=a::publish_attitude_at_boundary<Mode,Target>(c,out,42000,
    [&](const Target&){target++;},[&](const Mode&){mode++;});
  assert(decision.allowed&&target==1&&mode==1);
}
static void arm_cancel_after_final_sink(const char* action){
  f::Runtime r(config());auto s=authority_test::sample();double t=0;
  authority_test::ready(r,s,t,"AUTHORITY_AUTONOMOUS");r.require_attitude_sink_commit();
  assert(r.request("TAKEOFF",{1.},s,t).accepted);
  auto out=authority_test::tick(r,s,t);
  assert(out.attitude.command==a::Command::Arm);
  auto context=r.publication_context(s,t,true,true);int arm_wire=0;
  assert(a::publish_arm_at_boundary(context,[&]{arm_wire++;}).allowed&&arm_wire==1);
  r.confirm_attitude_command_publication(a::Command::Arm,true,100010,s,t);
  assert(r.pending(400).command400_kind==f::Command400Kind::Arm);
  assert(r.pending(400).stage==f::CommandStage::CommittedAwaitAck);
  if(std::string(action)=="LAND")assert(r.request("LAND",{},s,t).accepted);
  else if(std::string(action)=="SHUTDOWN")r.begin_shutdown(s,t);
  else if(std::string(action)=="DISCONNECT")r.disconnect(s,t);
  else if(std::string(action)=="TAKEOVER")assert(r.request("AUTHORITY_TAKEOVER",{},s,t).accepted);
  else assert(false);
  out=authority_test::tick(r,s,t);
  assert(r.attitude_core().phase()==a::Phase::RequestArm);
  assert(r.attitude_core().arm_cancel_pending());
  assert(out.attitude.publish&&out.attitude.thrust[2]==0&&out.attitude.command==a::Command::None);
  final_heartbeat(r,s,t,out.attitude,std::string(action)=="SHUTDOWN",std::string(action)!="DISCONNECT");
  assert(!r.request("TAKEOFF",{1.},s,t).accepted);
  const auto base=r.pending(400).baseline;
  r.ack({base+100,400,0,255,190,false},t+.001);
  assert(!r.pending(400).accepted);
  r.ack({base+200,400,0,1,191,false},t+.002);
  assert(r.pending(400).accepted);
  s.feedback.armed=true;
  out=authority_test::tick(r,s,t);
  assert(r.attitude_core().phase()==a::Phase::PoweredGround);
  assert(r.pending(400).state_verified&&r.pending(400).stage==f::CommandStage::VerifiedTerminal);
  assert(out.attitude.publish&&out.attitude.command==a::Command::None);
  final_heartbeat(r,s,t,out.attitude,true,false);
  std::cout<<"R3_ARM_COMMITTED_"<<action<<" heartbeat_after_cancel=1 late_arm_recovery=1 sink_pair=1\n";
}
static void arm_timeout_and_refusal(){
  {f::Runtime r(config());auto s=authority_test::sample();double t=0;
    authority_test::ready(r,s,t,"AUTHORITY_AUTONOMOUS");r.require_attitude_sink_commit();
    assert(r.request("TAKEOFF",{1},s,t).accepted);
    auto o=authority_test::tick(r,s,t);assert(o.attitude.command==a::Command::Arm);
    r.confirm_attitude_command_publication(a::Command::Arm,true,9950,s,t);
    assert(r.request("LAND",{},s,t).accepted);
    r.ack({r.pending(400).baseline+1,400,4,1,191,false},t+.01);
    o=authority_test::tick(r,s,t);
    assert(r.attitude_core().phase()==a::Phase::ReleaseOffboard);
    assert(o.attitude.publish&&o.attitude.command==a::Command::ReleaseOffboard);
    assert(!r.pending(400).accepted&&r.pending(400).rejected);
    std::cout<<"R3_ARM_REJECTED_NEW_GROUND_MODE_RELEASE=1\n";
  }
  {f::Runtime r(config());auto s=authority_test::sample();double t=0;
    authority_test::ready(r,s,t,"AUTHORITY_AUTONOMOUS");r.require_attitude_sink_commit();
    assert(r.request("TAKEOFF",{1},s,t).accepted);
    auto o=authority_test::tick(r,s,t);
    r.confirm_attitude_command_publication(a::Command::Arm,true,9950,s,t);
    assert(r.request("LAND",{},s,t).accepted);
    for(int i=0;i<110;++i){o=authority_test::tick(r,s,t);assert(o.attitude.publish);}
    assert(r.attitude_core().phase()==a::Phase::RequestArm);
    s.feedback.armed=true;o=authority_test::tick(r,s,t);
    assert(o.attitude.publish&&r.attitude_core().phase()==a::Phase::PoweredGround);
    assert(r.pending(400).state_verified&&!r.pending(400).accepted);
    // A *new* armed status ends the Arm uncertainty even if ACK went missing.
    // A later ground Disarm must not collide and drop the heartbeat.
    s.at_rest=true;s.feedback.landed=true;
    for(int i=0;i<200&&r.attitude_core().phase()!=a::Phase::RequestDisarm;++i){o=authority_test::tick(r,s,t);assert(o.attitude.publish);}
    assert(r.attitude_core().phase()==a::Phase::RequestDisarm);
    assert(o.attitude.command==a::Command::Disarm&&r.pending(400).command400_kind==f::Command400Kind::Disarm);
    assert(r.pending(400).stage==f::CommandStage::Planned);
    std::cout<<"R3_ARM_MISSING_ACK_STATUS_RECONCILED_DISARM_TRACKED=1\n";
  }
}
static void disarm_bounce_after_commit(){
  f::Runtime r(config());auto s=authority_test::sample();double t=0;
  authority_test::armed(r,s,t,"AUTHORITY_AUTONOMOUS");r.require_attitude_sink_commit();
  s.at_rest=false;s.feedback.landed=true;
  assert(r.request("LAND",{},s,t).accepted);
  for(int i=0;i<30;++i)authority_test::tick(r,s,t);
  s.at_rest=true;a::Output o;
  for(int i=0;i<80&&r.attitude_core().phase()!=a::Phase::RequestDisarm;++i)
    o=authority_test::tick(r,s,t).attitude;
  assert(r.attitude_core().phase()==a::Phase::RequestDisarm&&o.command==a::Command::Disarm);
  auto boundary=r.publication_context(s,t,true,true);int wire=0;
  assert(a::publish_disarm_at_boundary(boundary,[&]{wire++;}).allowed&&wire==1);
  r.confirm_attitude_command_publication(a::Command::Disarm,true,55501,s,t);
  assert(r.pending(400).command400_kind==f::Command400Kind::Disarm);
  assert(r.pending(400).stage==f::CommandStage::CommittedAwaitAck);
  s.feedback.landed=false;s.at_rest=false;
  auto out=authority_test::tick(r,s,t);assert(out.attitude.publish);
  assert(r.attitude_core().phase()==a::Phase::Descent);
  final_heartbeat(r,s,t,out.attitude);
  // Arm/Disarm reuse: third-party ACK, stale ACK, unrelated target must not
  // close a 400 transaction. Real ACK cannot carry a param1/nonce.
  const auto base=r.pending(400).baseline;
  r.ack({base,400,0,1,191,false},t+.001);
  r.ack({base+2,400,0,9,191,false},t+.002);
  assert(!r.pending(400).accepted);
  r.ack({base+3,400,0,1,191,false},t+.003);
  assert(!r.pending(400).accepted&&r.pending(400).ack_ambiguous&&!r.pending(400).state_verified);
  for(int i=0;i<4;++i){out=authority_test::tick(r,s,t);assert(out.attitude.publish);
    assert(out.attitude.command!=a::Command::Disarm);}
  // Contact returns: old 400 is not republished while status remains Armed.
  s.feedback.landed=true;s.at_rest=true;
  for(int i=0;i<100;++i){out=authority_test::tick(r,s,t);assert(out.attitude.publish);
    assert(out.attitude.command!=a::Command::Disarm);}
  assert(r.pending(400).command400_kind==f::Command400Kind::Disarm);
  assert(!r.pending(400).state_verified);
  s.feedback.armed=false;out=authority_test::tick(r,s,t);
  assert(r.attitude_core().phase()==a::Phase::SafeDisarmed&&!out.attitude.publish);
  assert(r.pending(400).state_verified&&r.pending(400).stage==f::CommandStage::StateOnlyTerminal);
  std::cout<<"R3_DISARM_BOUNCE_R4_STATE_ONLY_NO_REPEAT=1\n";
}
static void disarm_bounce_rejected(){
  f::Runtime r(config());auto s=authority_test::sample();double t=0;
  authority_test::armed(r,s,t,"AUTHORITY_AUTONOMOUS");
  s.at_rest=false;assert(r.request("LAND",{},s,t).accepted);
  for(int i=0;i<30;++i)authority_test::tick(r,s,t);
  s.at_rest=true;a::Output output;
  for(int i=0;i<90&&r.attitude_core().phase()!=a::Phase::RequestDisarm;++i)
    output=authority_test::tick(r,s,t).attitude;
  assert(output.command==a::Command::Disarm);
  s.feedback.landed=false;s.at_rest=false;
  auto out=authority_test::tick(r,s,t);assert(out.attitude.publish&&r.attitude_core().phase()==a::Phase::Descent);
  r.ack({r.pending(400).baseline+10,400,4,1,191,false},t+.001);
  s.feedback.landed=true;s.at_rest=true;
  // R4: an uncorrelated FAILED result cannot authorize a second Disarm.
  const auto generation=r.pending(400).generation;
  for(int i=0;i<125;++i){out=authority_test::tick(r,s,t);
    assert(out.attitude.publish&&out.attitude.command!=a::Command::Disarm);}
  assert(r.pending(400).ack_ambiguous&&!r.pending(400).rejected);
  assert(r.pending(400).generation==generation);
  s.feedback.armed=false;out=authority_test::tick(r,s,t);
  assert(r.attitude_core().phase()==a::Phase::SafeDisarmed&&!out.attitude.publish);
  assert(r.pending(400).command400_kind==f::Command400Kind::Disarm);
  std::cout<<"R3_POST_BOUNCE_R4_AMBIGUOUS_FAILED_NO_NEW_DISARM=1\n";
}
static void disarm_post_commit_sensor_freeze(){
  f::Runtime r(config());auto s=authority_test::sample();double t=0;
  authority_test::armed(r,s,t,"AUTHORITY_AUTONOMOUS");r.require_attitude_sink_commit();
  s.at_rest=false;assert(r.request("LAND",{},s,t).accepted);
  for(int i=0;i<30;++i)authority_test::tick(r,s,t);
  s.at_rest=true;a::Output o;
  for(int i=0;i<100&&r.attitude_core().phase()!=a::Phase::RequestDisarm;++i)
    o=authority_test::tick(r,s,t).attitude;
  assert(o.command==a::Command::Disarm);
  r.confirm_attitude_command_publication(a::Command::Disarm,true,77011,s,t);
  const auto land_at_commit=s.land_stamp;
  r.ack({r.pending(400).baseline+2,400,0,1,191,false},t+.001);
  assert(!r.pending(400).accepted&&r.pending(400).ack_ambiguous);
  s.feedback.armed=false;
  // Status is new, but land detector is FROZEN on exactly the sink sample.
  t+=.05;s.status_stamp+=2000;r.request("PING",{},s,t);
  o=r.step(s,t).attitude;
  assert(o.publish&&r.attitude_core().phase()==a::Phase::RequestDisarm);
  assert(!r.pending(400).state_verified&&s.land_stamp==land_at_commit);
  s.land_stamp+=1000;
  o=authority_test::tick(r,s,t).attitude;
  assert(r.attitude_core().phase()==a::Phase::SafeDisarmed&&!o.publish);
  assert(r.pending(400).state_verified);
  std::cout<<"R3_POST_COMMIT_FROZEN_LAND_CANNOT_RELEASE_UNTIL_NEW_SAMPLE=1\n";
}
static void disarm_ack_timeout_no_blind_repeat(){
  f::Runtime r(config());auto s=authority_test::sample();double t=0;
  authority_test::armed(r,s,t,"AUTHORITY_AUTONOMOUS");r.require_attitude_sink_commit();
  s.at_rest=false;assert(r.request("LAND",{},s,t).accepted);
  for(int i=0;i<30;++i)authority_test::tick(r,s,t);
  s.at_rest=true;a::Output o;
  for(int i=0;i<100&&r.attitude_core().phase()!=a::Phase::RequestDisarm;++i)
    o=authority_test::tick(r,s,t).attitude;
  assert(o.command==a::Command::Disarm);
  r.confirm_attitude_command_publication(a::Command::Disarm,true,88011,s,t);
  for(int i=0;i<125;++i){auto ticked=authority_test::tick(r,s,t);
    assert(ticked.attitude.publish&&ticked.attitude.command!=a::Command::Disarm);}
  assert(r.attitude_core().phase()==a::Phase::RequestDisarm);
  assert(r.attitude_core().disarm_outcome_uncertain());
  assert(r.pending(400).sent&&!r.pending(400).accepted);
  std::cout<<"R3_DISARM_ACK_TIMEOUT_RETAINS_GUARDED_HEARTBEAT_NO_BLIND_RETRY=1\n";
}
int main(){
  for(const char* action:{"LAND","SHUTDOWN","DISCONNECT","TAKEOVER"})arm_cancel_after_final_sink(action);
  arm_timeout_and_refusal();disarm_bounce_after_commit();disarm_bounce_rejected();
  disarm_post_commit_sensor_freeze();disarm_ack_timeout_no_blind_repeat();
  std::cout<<"R3_M1_ASYNC_INTERLEAVINGS_FINAL_SINK=PASS\n";
}
