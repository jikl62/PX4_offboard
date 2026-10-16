#include "review_bench_support.hpp"
#include <string>
static void exit_retains_zero_until_new_safe_ground(){
  for(bool shutdown:{false,true}){
    B h;h.armed();h.s.at_rest=false;
    if(shutdown)h.r.begin_shutdown(h.s,h.t);
    else {h.connected=false;h.r.disconnect(h.s,h.t);}
    auto o=h.next(false);
    assert(h.r.attitude_core().phase()==a::Phase::PoweredGround);
    assert(o.publish&&h.last_decision.allowed&&h.disarms==0);
    const auto targets=h.targets;
    for(int i=0;i<80;++i){o=h.next(false,false);assert(o.publish&&h.disarms==0&&h.denials==0);}
    assert(h.targets>targets&&h.targets==h.heartbeats);
    // New landed/Z/Vz evidence permits bench Disarm even while motors vibrate.
    for(int i=0;i<160&&h.disarms==0;++i)h.next(false);
    assert(h.disarms==1&&h.arms==1);
  }
  std::cout<<"REVIEW2_BENCH_EXIT_FROZEN_LAND_WAIT_THEN_NEW_GROUND_DISARM PASS\n";
}
static void reset_between_plan_and_commit_is_denied(){
  B h;h.armed();a::Output o;
  for(int i=0;i<300;++i){o=h.next(true,true,false);if(o.command==a::Command::Disarm)break;h.commit(o);}
  assert(o.command==a::Command::Disarm&&h.disarms==0);
  assert(a::disarm_publication_policy(h.r.publication_context(h.s,h.t,true,true)).allowed);
  for(int which=0;which<3;++which){
    auto race=h.s;
    if(which==0)++race.attitude.z_reset;
    if(which==1)++race.attitude.vz_reset;
    if(which==2)++race.attitude.quat_reset;
    auto c=h.r.publication_context(race,h.t,true,true);
    unsigned targets=0,modes=0,commands=0;
    const auto output_decision=a::publish_attitude_at_boundary<Mode,Target>(c,o,42,
      [&](const Target&){++targets;},[&](const Mode&){++modes;});
    assert(!output_decision.allowed);
    assert(!a::publish_disarm_at_boundary(c,[&]{++commands;}).allowed);
    assert(targets==0&&modes==0&&commands==0);
  }
  std::cout<<"REVIEW_FINAL_SINK_Z_VZ_QUAT_RESET_RACE_DENIED PASS\n";
}
static void pending_offboard_disconnect_can_release_late_mode(){
  B h;
  for(int i=0;i<60&&h.r.attitude_core().phase()!=a::Phase::RequestOffboard;++i)h.next();
  assert(h.r.pending(176).published);
  h.connected=false;h.r.disconnect(h.s,h.t);
  auto o=h.next(false);
  assert(o.publish&&h.last_decision.allowed&&h.denials==0);
  h.r.ack({h.r.pending(176).wire_stamp+1000,176,0,1,191,false},h.t+.001);
  h.s.feedback.offboard=true;h.s.nav_state=14;
  o=h.next(false);
  assert(h.r.attitude_core().phase()==a::Phase::ReleaseOffboard);
  assert(h.releases==1&&h.arms==0&&h.disarms==0);
  h.s.feedback.offboard=false;h.s.nav_state=15;
  h.r.ack({h.r.pending(176).wire_stamp+1000,176,0,1,191,false},h.t+.001);
  o=h.next(false);assert(!o.publish&&h.r.attitude_core().phase()==a::Phase::Idle);
  std::cout<<"REVIEW_PENDING_OFFBOARD_DISCONNECT_SAFE_GROUND_RELEASE PASS\n";
}
static void external_bench_disarm_requires_new_ground_proof(){
  B h;h.armed();h.s.feedback.armed=false;
  auto o=h.next();
  assert(o.publish&&h.disarms==0);
  for(int i=0;i<45;++i){o=h.next(true,false);assert(o.publish&&h.disarms==0);}
  assert(h.r.attitude_core().phase()!=a::Phase::SafeDisarmed);
  for(int i=0;i<100&&h.r.attitude_core().phase()!=a::Phase::SafeDisarmed;++i)h.next();
  assert(h.r.attitude_core().phase()==a::Phase::SafeDisarmed&&h.disarms==0);
  assert(h.r.attitude_core().reason()=="BENCH_EXTERNAL_DISARM_NEW_SAFE_GROUND_EVIDENCED");
  std::cout<<"REVIEW_EXTERNAL_BENCH_DISARM_NO_EXTRA_400_OR_FALSE_COMMAND_SUCCESS PASS\n";
}
static void late_cancelled_bench_arm_never_enters_vertical_control(){
  B h;h.planned_arm();h.r.request("LAND",{},h.s,h.t);
  h.s.feedback.armed=true;h.s.feedback.landed=false;h.s.at_rest=false;h.s.attitude.z=1.8;
  const auto o=h.next();
  assert(!o.publish&&h.disarms==0);
  assert(h.r.attitude_core().reason()=="BENCH_LATE_ARM_GROUND_UNCONFIRMED_OPERATOR_INTERVENTION_REQUIRED");
  std::cout<<"REVIEW_CANCELLED_BENCH_ARM_NEVER_RUNS_NORMAL_DESCENT_CONTROLLER PASS\n";
}
static void real_frozen_stamp_expires_without_disarm(){
  B h;h.armed();f::Stamp detector;assert(detector.accept(h.s.land_stamp,h.t));
  bool expired=false;
  for(int i=0;i<65;++i){
    assert(!detector.accept(h.s.land_stamp,h.t+.05));
    h.s.land_fresh=detector.fresh(h.t+.05,2.5);
    const auto o=h.next(true,false);
    if(!h.s.land_fresh){expired=true;assert(!o.publish);}
    assert(h.disarms==0);
  }
  assert(expired&&h.r.attitude_core().phase()==a::Phase::Safety);
  std::cout<<"REVIEW_REAL_FROZEN_LAND_STAMP_EXPIRES_NO_FALSE_GROUND_PROOF PASS\n";
}
static void overdue_wait_is_disclosed_without_blind_disarm(){
  B h;h.armed();h.s.at_rest=false;
  for(int i=0;i<80&&h.r.attitude_core().phase()!=a::Phase::PoweredGround;++i)h.next();
  assert(h.r.attitude_core().phase()==a::Phase::PoweredGround);
  for(int i=0;i<220;++i){const auto o=h.next(true,false);assert(o.publish&&h.disarms==0);}
  assert(h.r.attitude_core().phase()==a::Phase::PoweredGround);
  assert(h.r.attitude_core().bench_wait_overdue(h.t));
  std::cout<<"REVIEW_POWERED_GROUND_WAIT_OVERDUE_DISCLOSED_NO_BLIND_DISARM PASS\n";
}
int main(int argc,char** argv){
  const std::string which=argc>1?argv[1]:"all";
  if(which=="exit"||which=="all")exit_retains_zero_until_new_safe_ground();
  if(which=="reset"||which=="all")reset_between_plan_and_commit_is_denied();
  if(which=="pending"||which=="all")pending_offboard_disconnect_can_release_late_mode();
  if(which=="external"||which=="all")external_bench_disarm_requires_new_ground_proof();
  if(which=="late"||which=="all")late_cancelled_bench_arm_never_enters_vertical_control();
  if(which=="freeze"||which=="all")real_frozen_stamp_expires_without_disarm();
  if(which=="overdue"||which=="all")overdue_wait_is_disclosed_without_blind_disarm();
}
