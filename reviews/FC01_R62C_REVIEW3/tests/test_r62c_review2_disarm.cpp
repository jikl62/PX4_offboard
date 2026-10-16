#include "review_bench_support.hpp"
#include <functional>
#include <string>

// Drive real Runtime and final sinks; only PX4 observations are synthetic.
struct R2Bench : B {
  R2Bench():B(true){}
  a::Output tick(bool newland=true, bool newlocal=true) {
    return next(true,newland,true,newlocal);
  }
  void spinning(){armed();s.at_rest=false;s.stationary_seconds=0;}
  a::Output plan_disarm(){
    for(int i=0;i<155;++i){
      auto o=next(true,true,false);
      if(o.command==a::Command::Disarm)return o;
      commit(o);
    }
    assert(false&&"fresh landed zero-thrust bench must plan one Disarm despite at_rest=false");
    return {};
  }
};

static void spinning_bench_completes_without_at_rest_or_attributed_ack(){
  R2Bench h;h.spinning();const double armed_at=h.t;
  auto o=h.plan_disarm();
  assert(h.t-armed_at>=2&&h.t-armed_at<8);
  h.commit(o);assert(h.arms==1&&h.disarms==1);
  assert(h.r.command400_history().size()==1);
  assert(h.r.pending(400).command400_kind==f::Command400Kind::Disarm);
  assert(h.r.pending(400).generation==2);
  // A late Arm ACK / duplicate 400 ACK cannot finish the Disarm transaction.
  h.r.ack({h.r.pending(400).wire_stamp+1,400,0,1,191,false},h.t+.001);
  for(int i=0;i<20;++i){h.tick();assert(h.r.attitude_core().phase()==a::Phase::RequestDisarm);}
  h.s.feedback.armed=false;
  for(int i=0;i<30;++i){h.tick();assert(h.r.attitude_core().phase()==a::Phase::RequestDisarm);}
  // Disarmed alone is insufficient: require NEW, stationary ground dwell.
  h.s.at_rest=true;h.s.stationary_seconds=3;
  for(int i=0;i<45;++i)h.tick(false);
  assert(h.r.attitude_core().phase()==a::Phase::RequestDisarm&&h.releases==0);
  for(int i=0;i<110&&h.r.attitude_core().phase()!=a::Phase::SafeDisarmed;++i)h.tick();
  assert(h.r.attitude_core().phase()==a::Phase::SafeDisarmed);
  assert(h.r.pending(400).state_verified&&h.disarms==1);
  h.tick();assert(h.releases==1);
  h.r.ack({h.r.pending(176).wire_stamp+1000,176,0,1,191,false},h.t+.001);
  h.s.feedback.offboard=false;h.s.nav_state=15;h.tick();
  assert(h.r.attitude_core().phase()==a::Phase::Idle);
  assert(h.arms==1&&h.disarms==1&&h.targets==h.heartbeats);
  std::cout<<"REVIEW2_SPINNING_BENCH_ONE_ARM_ONE_DISARM_AND_NEW_REST_RELEASE PASS\n";
}

static void missing_new_observations_never_disarm(){
  for(int which=0;which<3;++which){
    R2Bench h;h.spinning();const auto status=h.s.status_stamp;
    for(int i=0;i<180;++i){
      if(which==2)h.s.status_stamp=status-50000;
      h.tick(which!=0,which!=1);
      assert(h.disarms==0);
    }
    assert(h.r.attitude_core().bench_wait_overdue(h.t));
  }
  // A single post-Arm update cannot disguise a status/local stream that
  // subsequently freezes while the independent detector keeps publishing.
  for(int which=0;which<2;++which){
    R2Bench h;h.spinning();
    for(int i=0;i<80&&h.r.attitude_core().phase()!=a::Phase::PoweredGround;++i)h.tick();
    h.tick();const auto status=h.s.status_stamp;
    for(int i=0;i<130;++i){
      if(which==0)h.s.status_stamp=status-50000;
      h.tick(true,which!=1);assert(h.disarms==0);
    }
  }
  std::cout<<"REVIEW2_FROZEN_LAND_LOCAL_OR_STATUS_CANNOT_AUTHORIZE_DISARM PASS\n";
}

static void final_sink_rechecks_actual_ground_and_authority(){
  R2Bench h;h.spinning();const auto o=h.plan_disarm();
  const auto original=h.r.publication_context(h.s,h.t,true,true);
  assert(a::disarm_publication_policy(original).allowed);
  const std::function<void(a::BoundaryContext&)> changes[]={
    [](auto& c){c.feedback.landed=false;},
    [](auto& c){c.feedback.armed=false;},
    [](auto& c){c.feedback.offboard=false;},
    [](auto& c){c.feedback.z-=.5;},
    [](auto& c){c.feedback.vz=.251;},
    [](auto& c){c.feedback.z=NAN;},
    [](auto& c){c.feedback.vz=NAN;},
    [](auto& c){c.feedback.land_fresh=false;},
    [](auto& c){c.feedback.status_fresh=false;},
    [](auto& c){c.feedback.local_fresh=false;},
    [](auto& c){c.feedback.source_clock_valid=false;},
    [](auto& c){++c.feedback.z_reset;},
    [](auto& c){++c.feedback.vz_reset;},
    [](auto& c){++c.feedback.quat_reset;},
    [](auto& c){c.feedback.kill=true;},
    [](auto& c){c.feedback.operator_takeover=true;},
    [](auto& c){c.feedback.failsafe=true;},
    [](auto& c){c.feedback.exclusive=false;},
    [](auto& c){c.feedback.rc_link_ready=false;},
    [](auto& c){c.feedback.rc_switches_ready=false;},
    [](auto& c){c.config.bench_arm_only_enabled=false;},
    [](auto& c){c.config.takeoff_enabled=true;},
    [](auto& c){c.ground_confirmed=false;},
    [](auto& c){c.bench_disarm_proven=false;c.feedback.at_rest=true;},
    [](auto& c){c.bench_arm_only_active=false;c.feedback.at_rest=true;},
    [](auto& c){c.arm_command_committed=false;c.feedback.at_rest=true;},
    [](auto& c){c.feedback.status_stamp=c.bench_arm_status_baseline;c.feedback.at_rest=true;},
    [](auto& c){c.feedback.local_stamp=c.bench_arm_local_baseline;c.feedback.at_rest=true;},
    [](auto& c){c.feedback.status_stamp=c.bench_disarm_status_floor-1;},
    [](auto& c){c.feedback.local_stamp=c.bench_disarm_local_floor-1;},
    [](auto& c){c.feedback.land_stamp=c.bench_disarm_land_floor-1;},
    [](auto& c){c.disarm_command_committed=true;},
    [](auto& c){c.feedback.land_stamp=c.touchdown_baseline;}
  };
  for(const auto& change:changes){
    auto race=original;change(race);unsigned commands=0;
    assert(!a::publish_disarm_at_boundary(race,[&]{++commands;}).allowed);
    assert(commands==0);
  }
  h.commit(o);assert(h.disarms==1);
  for(int i=0;i<130;++i)h.tick();
  assert(h.disarms==1&&h.r.attitude_core().phase()==a::Phase::RequestDisarm);
  assert(h.r.attitude_core().disarm_outcome_uncertain());
  std::cout<<"REVIEW2_DISARM_SINK_RACES_AND_NO_BLIND_400_RETRY PASS\n";
}

static void rearm_during_post_disarm_verification_is_latched(){
  R2Bench h;h.spinning();h.commit(h.plan_disarm());
  h.s.feedback.armed=false;h.s.at_rest=true;h.s.stationary_seconds=3;
  for(int i=0;i<25;++i)h.tick();
  assert(h.r.attitude_core().phase()==a::Phase::RequestDisarm);
  h.s.feedback.armed=true;h.tick();
  assert(h.r.attitude_core().phase()==a::Phase::Safety);
  assert(h.r.attitude_core().reason()=="BENCH_REARM_DURING_DISARM_VERIFICATION");
  h.s.feedback.armed=false;
  for(int i=0;i<70;++i)h.tick();
  assert(h.disarms==1&&h.releases==0);
  std::cout<<"REVIEW2_POST_DISARM_REARM_LATCHED_NO_OLD_DWELL_REUSE PASS\n";
}

int main(int argc,char** argv){
  const std::string which=argc>1?argv[1]:"all";
  if(which=="positive"||which=="all")spinning_bench_completes_without_at_rest_or_attributed_ack();
  if(which=="freeze"||which=="all")missing_new_observations_never_disarm();
  if(which=="boundary"||which=="all")final_sink_rechecks_actual_ground_and_authority();
  if(which=="rearm"||which=="all")rearm_during_post_disarm_verification_is_latched();
}
