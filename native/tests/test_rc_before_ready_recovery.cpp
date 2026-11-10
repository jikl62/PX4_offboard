// Exercise the real Runtime/Core and final zero-publication boundary.
#define main existing_rc_navigation_contract_main
#include "test_rc_navigation.cpp"
#undef main

enum class BeforeReady {PrestreamArm,PrestreamNavigation,ArmSource,ArmedState};
static const char* before_ready_reason(BeforeReady which){
  if(which==BeforeReady::ArmSource)return "RC_ARM_SOURCE_BEFORE_READY";
  if(which==BeforeReady::ArmedState)return "RC_ARM_STATE_BEFORE_READY";
  return "RC_ARM_OR_NAVIGATION_BEFORE_READY";
}
static bool needs_full_ground_proof(BeforeReady which){
  return which==BeforeReady::ArmSource||which==BeforeReady::ArmedState;
}
struct BeforeReadyRetry {
  Flight q{production_no_vision_rc_config()};
  bool resting{true};double rest_since{-3};
  void advance(bool fresh=true){
    q.t+=.05;
    if(fresh){
      q.s.status_stamp+=50000;q.s.land_stamp+=50000;
      q.s.attitude.local_stamp+=50000;q.s.attitude.vertical_sample_stamp+=50000;
      q.s.attitude.rc_switch_sample_stamp+=50000;
    }
    q.s.attitude.source_host_stamp+=50000;
    const bool ground=!q.s.feedback.armed&&q.s.feedback.landed&&q.s.at_rest;
    if(ground){if(!resting)rest_since=q.t;q.s.stationary_seconds=q.t-rest_since;}
    else q.s.stationary_seconds=0;
    resting=ground;
    // The node scans independent callback payloads before Runtime::step.
    // Even a callback on every tick must not starve the fault observer.
    q.r.begin_attitude_safety_observation();
    const auto feedback=q.r.publication_context(q.s,q.t,true,true).feedback;
    for(auto source:{a::Core::SafetyEvent::Vertical,a::Core::SafetyEvent::Status,
        a::Core::SafetyEvent::Battery,a::Core::SafetyEvent::Land,a::Core::SafetyEvent::Authority})
      q.r.observe_attitude_safety_event(feedback,source,q.t);
    q.o=q.r.step(q.s,q.t);
    need(q.o.attitude.command==a::Command::None&&!a::nonzero(q.o.attitude.thrust),
      "a rejected/preparing RC attempt must issue no automatic command or thrust");
    if(q.o.attitude.publish){
      const auto context=q.r.publication_context(q.s,q.t,true,true);
      const auto decision=a::publish_attitude_at_boundary<RcMode,RcTarget>(context,
        q.o.attitude,q.s.status_stamp,[](const RcTarget&){},[](const RcMode&){});
      need(decision.allowed,a::name(decision.reason));q.r.confirm_rc_output(true,q.t);
    }
    need(!q.r.pending(176).requested&&!q.r.pending(400).requested,
      "no command transaction may be manufactured by recovery or preparation");
  }
  void trigger(BeforeReady which){
    advance();need(q.r.attitude_core().phase()==a::Phase::RcPrestream,
      "fresh safe ground begins an actual zero-only prestream");
    if(needs_full_ground_proof(which)){
      for(unsigned i=0;i<100&&q.r.attitude_core().phase()!=a::Phase::RcWaitArm;++i)advance();
      need(q.r.attitude_core().phase()==a::Phase::RcWaitArm,"committed prestream genuinely reaches READY");
    }
    if(which==BeforeReady::PrestreamNavigation)q.s.attitude.rc_mode_slot=q.c.attitude.rc_navigation_mode_slot;
    else if(which==BeforeReady::ArmedState){
      q.s.feedback.armed=true;q.s.armed_time=10;q.s.latest_arming_reason=2;q.s.at_rest=false;
    }else q.s.attitude.rc_arm_switch=1;
    // READY reserves a 100ms source-clock allowance. A new edge/status in
    // the next 50ms is refused even though the current snapshot is healthy.
    advance();
    const auto& core=q.r.attitude_core();
    need(core.phase()==a::Phase::Safety&&core.reason()==before_ready_reason(which),
      "each case must reach its real Before_READY rejection, not a synthetic block()");
    need(!core.flight_started()&&!core.rc_manual_arm_proven()&&!q.o.attitude.publish,
      "a Before_READY rejection cannot prove arm intent or join the flight");
    need(core.restart_required()==needs_full_ground_proof(which),
      "source/state causal failures must retain the full restart/proof requirement");
  }
  void reset_controls(bool remain_armed=false){
    q.s.attitude.rc_arm_switch=3;
    q.s.attitude.rc_mode_slot=q.c.attitude.rc_navigation_mode_slot==1?6:1;
    q.s.feedback.armed=remain_armed;q.s.feedback.offboard=false;
    q.s.at_rest=!remain_armed;q.s.feedback.landed=true;
    q.s.attitude.z=2.4;
  }
};
static void before_ready_fault_requires_new_safe_task(BeforeReady which){
  BeforeReadyRetry f;f.trigger(which);f.reset_controls(true);
  for(unsigned i=0;i<10;++i){
    f.advance(false);
    need(!f.q.r.reprepare_rc(f.q.s,f.q.t),"safe switch reset alone cannot restart a still-armed vehicle");
  }
  f.reset_controls();
  std::string frozen_reason;
  if(needs_full_ground_proof(which)){
    // No source may reuse the fault's own sample as post-fault proof, even
    // after host time and the independent stationary timer have advanced.
    for(unsigned i=0;i<105;++i){
      f.advance(false);
      need(!f.q.r.reprepare_rc(f.q.s,f.q.t),"cached pre-fault feedback cannot recover a causal admission fault");
    }
    frozen_reason=f.q.r.attitude_core().rc_recovery_diagnostics().block_reason;
  }
  bool rebuilt=false;const double fresh_since=f.q.t;
  for(unsigned i=0;i<125&&!rebuilt;++i){
    f.advance();
    const auto& core=f.q.r.attitude_core();
    const auto d=core.rc_recovery_diagnostics();
    if(needs_full_ground_proof(which)&&f.q.t-fresh_since<2)
      need(!core.rc_restart_allowed(f.q.r.publication_context(f.q.s,f.q.t,true,true).feedback),
        "one new sample or a partial fresh window cannot clear the failed attempt");
    if(core.rc_restart_allowed(f.q.r.publication_context(f.q.s,f.q.t,true,true).feedback)){
      if(needs_full_ground_proof(which))need(d.ready&&d.status_samples>=2&&d.land_samples>=3&&
        d.local_samples>=3&&d.elapsed_s+1e-9>=f.q.c.attitude.ground_seconds,
        "source/state failures require independent full ground proof");
      auto unsafe=f.q.s;unsafe.feedback.armed=true;
      need(!f.q.r.reprepare_rc(unsafe,f.q.t),"current armed feedback still vetoes a completed recovery proof");
    }
    rebuilt=f.q.r.reprepare_rc(f.q.s,f.q.t);
  }
  std::cout<<before_ready_reason(which)<<" rebuilt="<<rebuilt
    <<" frozen_reason="<<frozen_reason
    <<" current_block="<<f.q.r.attitude_core().rc_recovery_diagnostics().block_reason<<std::endl;
  need(rebuilt,"a rejected Before_READY attempt must eventually reprepare after independent new safe ground proof");
  if(needs_full_ground_proof(which))need(frozen_reason=="RC_RECOVERY_WAIT_NEW_STATUS",
    "frozen status must remain an explicit causal watermark blocker");
  need(f.q.r.rc_attempts_completed()==1&&f.q.r.last_rc_attempt_reason()==before_ready_reason(which),
    "Runtime must preserve the failed attempt history while replacing only task state");
  const auto d=f.q.r.attitude_core().rc_recovery_diagnostics();
  need(d.first_fault_reason.empty()&&d.latest_fault_reason.empty()&&!d.ready&&d.status_samples==0&&
    !f.q.r.attitude_core().rc_manual_arm_proven()&&!f.q.r.attitude_core().reference_ready(),
    "the rebuilt task must inherit neither the old fault proof, anchor nor arm authorization");
  for(unsigned i=0;i<100&&f.q.r.attitude_core().phase()!=a::Phase::RcWaitArm;++i)f.advance();
  need(f.q.r.attitude_core().phase()==a::Phase::RcWaitArm&&
    std::abs(f.q.r.attitude_core().reference_z()-2.4)<1e-9&&
    !f.q.r.attitude_core().rc_manual_arm_proven()&&!f.q.r.attitude_core().flight_started(),
    "reprepared READY needs its new ground anchor and still awaits new manual arm authorization");
}
int main(int argc,char** argv){
  const std::string which=argc>1?argv[1]:"all";
  if(which=="all"||which=="or"){
    before_ready_fault_requires_new_safe_task(BeforeReady::PrestreamArm);
    before_ready_fault_requires_new_safe_task(BeforeReady::PrestreamNavigation);
  }
  if(which=="all"||which=="source")before_ready_fault_requires_new_safe_task(BeforeReady::ArmSource);
  if(which=="all"||which=="state")before_ready_fault_requires_new_safe_task(BeforeReady::ArmedState);
  std::cout<<"RC_BEFORE_READY_CAUSAL_GROUND_RECOVERY_PASS "<<which<<'\n';
}
