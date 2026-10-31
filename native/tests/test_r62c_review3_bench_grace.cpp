#include "review_bench_support.hpp"
#include <functional>
#include <string>

// Review2's five reported samples are measurements. The longer traces and
// post-Disarm responses below are explicit synthetic observations, not a ULog.
struct GraceBench : B {
  GraceBench():B(true){}
  void spinning(){armed();s.at_rest=false;s.stationary_seconds=0;}
  void powered(){
    spinning();
    for(int i=0;i<85&&r.attitude_core().phase()!=a::Phase::PoweredGround;++i)next();
    assert(r.attitude_core().phase()==a::Phase::PoweredGround);
  }
  a::Output plan(){
    for(int i=0;i<160;++i){
      auto o=next(true,true,false);
      if(o.command==a::Command::Disarm)return o;
      commit(o);
      assert(r.attitude_core().phase()!=a::Phase::Safety);
    }
    assert(false&&"bounded downward drift must admit Disarm with new ground dwell");
    return {};
  }
};

static void reported_five_samples_survive(){
  GraceBench h;h.s.attitude.z=.274003655;h.powered();
  const double z[]={.380003422,.383861989,.388992816,.393225074,.398496389};
  const double vz[]={.0927877054,.0947041512,.0970299244,.0988691226,.102222808};
  for(int i=0;i<5;++i){
    h.s.attitude.z=z[i];h.s.attitude.vz=vz[i];
    const auto o=h.next();
    std::cout<<"REVIEW3_REPORTED_SAMPLE="<<i+1<<" phase="<<a::name(h.r.attitude_core().phase())
      <<" reason="<<h.r.attitude_core().reason()<<std::endl;
    assert(o.publish&&h.r.attitude_core().phase()!=a::Phase::Safety);
    assert(h.disarms==0); // 0.2s of measurements is not a two-second dwell.
  }
  std::cout<<"REVIEW3_FIVE_MEASURED_VALUES_ACCEPTED_WITHOUT_INVENTING_DWELL PASS\n";
}

static void downward_drift_completes_entire_closed_loop(){
  GraceBench h;h.spinning();const double start=h.t;
  const double anchor=h.r.attitude_core().reference_z();
  for(int i=0;i<155&&!h.disarms;++i){
    h.s.attitude.z=anchor+std::min(.45,.10*(h.t-start+.05));
    h.s.attitude.vz=.11;
    h.next();
    assert(h.r.attitude_core().phase()!=a::Phase::Safety);
    assert(h.r.attitude_core().phase()!=a::Phase::ThrustRamp);
  }
  assert(h.disarms==1&&h.arms==1&&h.t-start>=2&&h.t-start<8);
  assert(h.s.attitude.z-anchor>.25); // Exercises the former global lower bound.
  assert(h.r.pending(400).generation==2);
  // Keep the estimated offset after stopping. It must not be required to
  // return to the old +/-0.15m flight tolerance to finish the bench release.
  h.s.feedback.armed=false;
  for(int i=0;i<30;++i){h.next();assert(h.releases==0);}
  assert(h.r.attitude_core().phase()==a::Phase::RequestDisarm);
  // The adapter's original strict Vz-based stationary timer can remain zero.
  // Completion still requires newly published at_rest=true over a full dwell.
  h.s.at_rest=true;h.s.stationary_seconds=0;h.s.attitude.vz=.18;
  for(int i=0;i<100&&h.r.attitude_core().phase()!=a::Phase::SafeDisarmed;++i)h.next();
  assert(h.r.attitude_core().phase()==a::Phase::SafeDisarmed);
  assert(h.r.pending(400).state_verified);
  h.next();assert(h.releases==1);
  h.r.ack({h.r.pending(176).wire_stamp+1000,176,0,1,191,false},h.t+.001);
  h.s.feedback.offboard=false;h.s.nav_state=15;h.next();
  assert(h.r.attitude_core().phase()==a::Phase::Idle);
  assert(h.arms==1&&h.disarms==1&&h.targets==h.heartbeats&&h.denials==0);
  std::cout<<"REVIEW3_1HZ_DOWNWARD_DRIFT_ONE_ARM_ONE_DISARM_NEW_REST_TO_IDLE PASS\n";
}

static void final_sink_uses_same_bounded_bench_envelope(){
  GraceBench h;h.powered();
  h.s.attitude.z=h.r.attitude_core().reference_z()+.30;h.s.attitude.vz=.20;
  const auto o=h.plan();
  const auto original=h.r.publication_context(h.s,h.t,true,true);
  assert(a::disarm_publication_policy(original).allowed);
  const std::function<void(a::BoundaryContext&)> attacks[]={
    [](auto& c){c.feedback.z=c.reference_z-.120001;},
    [](auto& c){c.feedback.z=c.reference_z+.500001;},
    [](auto& c){c.feedback.vz=.250001;},
    [](auto& c){c.feedback.vz=-.250001;},
    [](auto& c){c.feedback.landed=false;},
    [](auto& c){c.feedback.local_fresh=false;},
    [](auto& c){c.feedback.z=NAN;},
    [](auto& c){++c.feedback.z_reset;},
    [](auto& c){c.feedback.kill=true;},
    [](auto& c){c.feedback.rc_link_ready=false;},
    [](auto& c){c.bench_disarm_proven=false;},
    [](auto& c){c.bench_arm_only_active=false;},
    [](auto& c){c.config.takeoff_enabled=true;}
  };
  for(const auto& attack:attacks){auto c=original;attack(c);unsigned count=0;
    assert(!a::publish_disarm_at_boundary(c,[&]{++count;}).allowed);assert(count==0);
  }
  h.commit(o);assert(h.disarms==1);
  for(int i=0;i<125;++i)h.next();
  assert(h.disarms==1);
  std::cout<<"REVIEW3_FINAL_SINK_GRACE_RECHECKS_NO_BLIND_RETRY PASS\n";
}

static void above_ground_and_excessive_motion_still_stop(){
  for(int phase=0;phase<2;++phase)for(int which=0;which<5;++which){
    GraceBench h;if(phase)h.powered();else h.spinning();
    const auto anchor=h.r.attitude_core().reference_z();
    if(which==0)h.s.attitude.z=anchor-.121;
    if(which==1){h.s.attitude.z=anchor+.24;h.next();
      assert(h.r.attitude_core().phase()!=a::Phase::Safety);
      h.s.attitude.z=anchor+.49;h.next();
      assert(h.r.attitude_core().phase()!=a::Phase::Safety);h.s.attitude.z=anchor+.501;}
    if(which==2)h.s.attitude.vz=.251;
    if(which==3)h.s.attitude.vz=-.251;
    if(which==4)h.s.feedback.landed=false;
    h.next();assert(h.r.attitude_core().phase()==a::Phase::Safety&&h.disarms==0);
  }
  std::cout<<"REVIEW3_UPWARD_HEIGHT_DOWNWARD_LIMIT_SPEED_AND_LANDED_LOSS_BLOCKED PASS\n";
}

static void frozen_streams_do_not_acquire_ground_dwell(){
  for(int which=0;which<3;++which){
    GraceBench h;h.powered();h.s.attitude.z=h.r.attitude_core().reference_z()+.2;
    h.s.attitude.vz=.12;const auto status=h.s.status_stamp;
    for(int i=0;i<140;++i){
      if(which==2)h.s.status_stamp=status-50000;
      h.next(true,which!=0,true,which!=1);assert(h.disarms==0);
    }
  }
  std::cout<<"REVIEW3_WIDE_ENVELOPE_FROZEN_LAND_LOCAL_STATUS_NO_DISARM PASS\n";
}

static void powered_fault_preserves_trigger_sample(){
  GraceBench h;h.powered();h.s.attitude.vz=.26;h.next();
  assert(h.r.attitude_core().phase()==a::Phase::Safety);
  const auto& e=h.r.attitude_core().armed_ground_fault_evidence();
  assert(e.valid&&e.failed_mask==16U&&std::abs(e.vz-.26)<1e-12);
  assert(e.status_stamp==h.s.status_stamp&&e.land_stamp==h.s.land_stamp);
  assert(e.local_stamp==h.s.attitude.local_stamp);
  std::cout<<"REVIEW3_POWERED_GROUND_EXACT_FAULT_SAMPLE_CAPTURED PASS\n";
}

static void grace_parameters_reject_invalid_or_unbounded_values(){
  for(int which=0;which<2;++which)for(const double value:std::array<double,5>{0.,-.1,NAN,INFINITY,1.001}){
    auto c=bench();
    if(which)c.attitude.bench_ground_vz_tolerance_mps=value;
    else c.attitude.bench_ground_down_tolerance_m=value;
    bool rejected=false;try{c.validate();}catch(const std::invalid_argument&){rejected=true;}
    assert(rejected);
  }
  for(int which=0;which<2;++which){
    auto limited=bench();
    if(which)limited.attitude.bench_ground_vz_tolerance_mps=.250001;
    else limited.attitude.bench_ground_down_tolerance_m=.500001;
    bool rejected=false;try{limited.validate();}catch(const std::invalid_argument&){rejected=true;}
    assert(rejected&&"configuration cannot widen beyond the tested bench grace");
  }
  auto c=bench();c.attitude.bench_ground_down_tolerance_m=.3;
  c.attitude.bench_ground_vz_tolerance_mps=.2;c.validate();
  a::Feedback s;s.z=.31;s.vz=.1;
  assert(!a::bench_vertical_observation(c.attitude,s,0.));
  s.z=.29;assert(a::bench_vertical_observation(c.attitude,s,0.));
  s.vz=.21;assert(!a::bench_vertical_observation(c.attitude,s,0.));
  std::cout<<"REVIEW3_GRACE_PARAMETERS_VALIDATED_AND_SMALLER_LIMITS_HONORED PASS\n";
}

static void external_disarm_cannot_bypass_estimator_jump_check(){
  GraceBench h;h.spinning();h.s.feedback.armed=false;h.s.at_rest=true;
  h.s.attitude.z=h.r.attitude_core().reference_z()+.49;
  h.next();
  assert(h.r.attitude_core().phase()==a::Phase::Safety);
  assert(h.r.attitude_core().reason()=="UNEXPLAINED_Z_VZ_ESTIMATOR_JUMP");
  assert(h.disarms==0&&h.releases==0);
  std::cout<<"REVIEW3_EXTERNAL_DISARM_CANNOT_HIDE_SINGLE_SAMPLE_ESTIMATOR_JUMP PASS\n";
}

static void external_disarm_after_continuous_drift_can_release_without_400(){
  GraceBench h;h.spinning();const auto anchor=h.r.attitude_core().reference_z();
  // Each step is below z_jump_limit; all remain inside the bench envelope.
  for(int i=1;i<=4;++i){h.s.attitude.z=anchor+.10*i;h.s.attitude.vz=.20;h.next();}
  h.s.feedback.armed=false;h.s.at_rest=true;h.s.stationary_seconds=0;
  for(int i=0;i<100&&h.r.attitude_core().phase()!=a::Phase::SafeDisarmed;++i)h.next();
  assert(h.r.attitude_core().phase()==a::Phase::SafeDisarmed&&h.disarms==0);
  h.next();assert(h.releases==1);
  h.r.ack({h.r.pending(176).wire_stamp+1000,176,0,1,191,false},h.t+.001);
  h.s.feedback.offboard=false;h.s.nav_state=15;h.next();
  assert(h.r.attitude_core().phase()==a::Phase::Idle&&h.disarms==0);
  std::cout<<"REVIEW3_EXTERNAL_DISARM_CONTINUOUS_DRIFT_NEW_REST_RELEASE_NO_400 PASS\n";
}

int main(int argc,char** argv){
  const std::string which=argc>1?argv[1]:"all";
  if(which=="recorded"||which=="all")reported_five_samples_survive();
  if(which=="drift"||which=="all")downward_drift_completes_entire_closed_loop();
  if(which=="sink"||which=="all")final_sink_uses_same_bounded_bench_envelope();
  if(which=="limits"||which=="all")above_ground_and_excessive_motion_still_stop();
  if(which=="freeze"||which=="all")frozen_streams_do_not_acquire_ground_dwell();
  if(which=="diagnostics"||which=="all")powered_fault_preserves_trigger_sample();
  if(which=="config"||which=="all")grace_parameters_reject_invalid_or_unbounded_values();
  if(which=="external_jump"||which=="all")external_disarm_cannot_bypass_estimator_jump_check();
  if(which=="external_drift"||which=="all")external_disarm_after_continuous_drift_can_release_without_400();
}
