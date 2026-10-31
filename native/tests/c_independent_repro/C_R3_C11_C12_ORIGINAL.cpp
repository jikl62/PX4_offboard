// Independent synthetic harness; reads R3 headers, does not edit R2 or R3 files.
#include "authority_test_support.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include <iostream>
#include <cassert>
namespace f=fc01;namespace a=f::attitude;
static void reach_disarm(f::Runtime& r,f::Snapshot& s,double& t){
  authority_test::ready(r,s,t,"AUTHORITY_AUTONOMOUS");r.require_attitude_sink_commit();
  assert(r.request("TAKEOFF",{1},s,t).accepted);
  auto out=authority_test::tick(r,s,t);
  assert(out.attitude.command==a::Command::Arm);
  r.confirm_attitude_command_publication(a::Command::Arm,true,1234567,s,t);
  assert(r.request("LAND",{},s,t).accepted);
  // Arm really occurs, while Arm ACK is permanently delayed.
  s.feedback.armed=true;
  out=authority_test::tick(r,s,t);
  assert(out.attitude.publish&&r.pending(400).state_verified&&!r.pending(400).accepted);
  s.feedback.landed=true;s.at_rest=true;
  bool got=false;
  for(int i=0;i<240;++i){out=authority_test::tick(r,s,t);
    assert(out.attitude.publish);
    if(out.attitude.command==a::Command::Disarm){got=true;break;}}
  assert(got&&r.attitude_core().phase()==a::Phase::RequestDisarm);
  assert(r.pending(400).command400_kind==f::Command400Kind::Disarm);
  r.confirm_attitude_command_publication(a::Command::Disarm,true,2000000,s,t);
  assert(r.pending(400).sent&&r.pending(400).stage==f::CommandStage::CommittedAwaitAck);
}
static f::Config test_config(){auto c=authority_test::config();c.attitude.autonomous_failsafe_confirmed=true;return c;}
static void late_arm_accept_labels_disarm_ack(){
  f::Runtime r(test_config());auto s=authority_test::sample();double t=0;reach_disarm(r,s,t);
  const auto base=r.pending(400).baseline;
  // A legitimate delayed ACK from old Arm with a later PX4 timestamp can
  // fulfill R3's checks, because command 400 has no nonce nor param1.
  r.ack({base+5000,400,0,1,191,false},t+.01);
  const auto& p=r.pending(400);
  std::cout<<"C11_OLD_ARM_ACCEPT_ACK_MISATTRIBUTED_TO_DISARM "
      <<"tracker_kind="<<f::command400_name(p.command400_kind)
      <<" accepted="<<p.accepted<<" state_verified="<<p.state_verified
      <<" stage="<<f::command_stage_name(p.stage)<<"\n";
  assert(p.accepted&&p.command400_kind==f::Command400Kind::Disarm&&!p.state_verified);
  s.feedback.armed=false;authority_test::tick(r,s,t);
  std::cout<<"C11_FAKE_VERIFIED_DISARM_ACK phase="<<r.phase_name()
       <<" stage="<<f::command_stage_name(r.pending(400).stage)<<"\n";
  assert(r.attitude_core().phase()==a::Phase::SafeDisarmed&&r.pending(400).stage==f::CommandStage::VerifiedTerminal);
}
static void late_arm_rejected_triggers_new_disarm(){
  f::Runtime r(test_config());auto s=authority_test::sample();double t=0;reach_disarm(r,s,t);
  // Old Arm ACK RESULT_DENIED delivered after new Disarm was sent.
  r.ack({r.pending(400).baseline+6000,400,4,1,191,false},t+.01);
  auto o=authority_test::tick(r,s,t);
  auto phase=r.attitude_core().phase();
  std::cout<<"C12_OLD_ARM_REJECT_ACK_WHILE_DISARM_PENDING "
   <<"pending_kind="<<f::command400_name(r.pending(400).command400_kind)
   <<" rejected="<<r.pending(400).rejected
   <<" phase="<<a::name(phase)<<"\n";
  assert(r.pending(400).rejected&&phase==a::Phase::PoweredGround);
  bool new_disarm=false;
  for(int i=0;i<200;++i){o=authority_test::tick(r,s,t);
   if(o.attitude.command==a::Command::Disarm){new_disarm=true;break;}}
  std::cout<<"C12_SECOND_400_DISARM_ATTEMPT while_prior_delivery_unknown="<<new_disarm
   <<" new_phase="<<a::name(r.attitude_core().phase())<<"\n";
  assert(new_disarm);
  int second_wires=0;
  auto policy=a::publish_disarm_at_boundary(r.publication_context(s,t,true,true),
      [&]{second_wires++;});
  if(policy.allowed)r.confirm_attitude_command_publication(a::Command::Disarm,true,3000000,s,t);
  std::cout<<"C12_DUPLICATE_FINAL_DISARM_SINK_ALLOWED="<<policy.allowed
       <<" second_wires="<<second_wires<<"\n";
  assert(policy.allowed&&second_wires==1);
}
int main(){late_arm_accept_labels_disarm_ack();late_arm_rejected_triggers_new_disarm();}
