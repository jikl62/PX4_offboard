// Reuse the real Core fixture while keeping this boundary audit independent.
#define main existing_rc_recovery_progress_main
#include "test_rc_recovery_progress.cpp"
#undef main

static void first_unsafe_queued_observation_remains_the_visible_blocker(){
  Recovery q;q.input(0,2);q.observe(0);q.input(.05,2);
  q.core.begin_safety_observation();auto first=q.s;first.landed=false;
  q.core.observe_safety_event(first,a::Core::SafetyEvent::Land,.05);
  auto later=q.s;later.source_clock_valid=false;
  q.core.observe_safety_event(later,a::Core::SafetyEvent::Status,.05);
  q.core.observe_safety_event(q.s,a::Core::SafetyEvent::Land,.05);q.finish(.05);
  auto d=q.core.rc_recovery_diagnostics();
  need(d.event_blocked&&!d.ready&&!d.window_active&&d.land_samples==0,
    "any unsafe payload must veto the entire observation cycle");
  need(d.block_reason=="RC_RECOVERY_QUEUED_EVENT_RC_RECOVERY_LANDED_AT_REST_REQUIRED",
    "later bad or benign payloads cannot replace the first actual queued blocker");
  need(d.first_fault_reason=="VERTICAL_POSITION_RATE_INCONSISTENT"&&
       d.latest_fault_reason==d.first_fault_reason,
    "queued recovery blockers are observations, not replacement fault history");
  q.input(.1,2);q.observe(.1);d=q.core.rc_recovery_diagnostics();
  need(!d.event_blocked&&d.window_active&&d.block_reason.find("QUEUED_EVENT")==std::string::npos,
    "a new healthy cycle must report its current wait rather than the old queued blocker");
}
static void time_rewind_inside_the_window_cannot_reuse_progress(){
  Recovery q;
  for(unsigned tick=0;tick<=44;++tick){q.input(tick*.05,2);q.observe(tick*.05);}
  need(q.core.rc_fault_recovery_ready(),"fixture first establishes genuine recovery readiness");
  q.observe(2.15);auto d=q.core.rc_recovery_diagnostics();
  need(!d.ready&&d.elapsed_s==0&&d.land_samples==1&&d.status_samples==1&&
       d.block_reason=="RC_RECOVERY_NEW_PROOF_AFTER_DISCONTINUITY",
    "a monotonic rewind after the window began must reset all prior proof");
  for(unsigned tick=45;tick<=82;++tick){q.input(tick*.05,2);q.observe(tick*.05);
    need(!q.core.rc_fault_recovery_ready(),"rewound progress cannot finish another window early");}
  for(unsigned tick=83;tick<=105;++tick){q.input(tick*.05,2);q.observe(tick*.05);}
  need(q.core.rc_restart_allowed(q.s),"window reset after rewind introduces no permanent new fault latch");
}
static void later_fault_invalidates_ready_counts_and_requires_new_source_watermarks(){
  Recovery q;
  for(unsigned tick=0;tick<=44;++tick){q.input(tick*.05,2);q.observe(tick*.05);}
  need(q.core.rc_fault_recovery_ready(),"fixture is ready before the secondary fault");
  const auto old=q.s;q.core.block("BATTERY_NOT_READY");auto d=q.core.rc_recovery_diagnostics();
  need(!d.ready&&!d.window_active&&d.elapsed_s==0&&d.status_samples==0&&d.land_samples==0&&d.local_samples==0,
    "a new fault must immediately discard every ready/progress diagnostic");
  need(d.first_fault_reason=="VERTICAL_POSITION_RATE_INCONSISTENT"&&d.latest_fault_reason=="BATTERY_NOT_READY",
    "the first cause stays historical while the new fault remains visible");
  q.observe(2.25);
  need(q.core.rc_recovery_diagnostics().block_reason=="RC_RECOVERY_WAIT_NEW_STATUS",
    "previously consumed feedback cannot seed the secondary fault's new proof");
  q.s=old;++q.s.status_stamp;q.observe(2.3);
  need(q.core.rc_recovery_diagnostics().block_reason=="RC_RECOVERY_WAIT_NEW_LAND",
    "secondary fault needs new land feedback too");
  ++q.s.land_stamp;q.observe(2.35);
  need(q.core.rc_recovery_diagnostics().block_reason=="RC_RECOVERY_WAIT_NEW_LOCAL",
    "secondary fault needs a new local observation");
  q.s.vertical_sample_stamp=++q.s.local_stamp;q.observe(2.4);
  need(q.core.rc_recovery_diagnostics().block_reason=="RC_RECOVERY_WAIT_NEW_RC_SWITCHES",
    "secondary fault needs a genuinely new safe switch observation");
}
static void unsafe_feedback_keeps_causal_watermarks_after_clearing_the_window(){
  for(bool queued:{false,true}){
    Recovery q;
    for(unsigned tick=0;tick<=20;++tick){q.input(tick*.05,2);q.observe(tick*.05);}
    q.input(1.5,2);q.core.begin_safety_observation();auto bad=q.s;bad.landed=false;
    if(queued)q.core.observe_safety_event(bad,a::Core::SafetyEvent::Land,1.5);
    else{q.s=bad;q.finish(1.5);}
    need(!q.core.rc_recovery_diagnostics().window_active,"unsafe feedback clears the current proof");
    q.core.block("BATTERY_NOT_READY");q.s=bad;q.s.landed=true;q.observe(1.55);
    need(q.core.rc_recovery_diagnostics().block_reason=="RC_RECOVERY_WAIT_NEW_STATUS",
      "clearing proof cannot discard the newest observed source watermarks before a later fault");
    // A fully fresh window remains possible; keeping the floor is not a new
    // permanent latch and does not require a new process/task incarnation.
    for(unsigned tick=32;tick<=110;++tick){q.input(tick*.05,2);q.observe(tick*.05);}
    need(q.core.rc_restart_allowed(q.s),"secondary fault with a cleared proof still recovers after new observations");
  }
}
struct PreparationAudit {
  a::Config config{recovery_config()};a::Core core{config};a::Feedback s{safe_feedback()};
  double now{},rest_since{-1};
  void tick(){
    now+=.05;s.status_stamp=s.land_stamp=s.local_stamp=s.vertical_sample_stamp=
      s.rc_switch_sample_stamp=s.source_host_stamp=stamp(now);
    const bool resting=!s.armed&&s.landed&&s.at_rest&&std::abs(s.vz)<=config.ground_vz_tolerance;
    if(resting){if(rest_since<0)rest_since=now;s.stationary_seconds=now-rest_since;}
    else{rest_since=-1;s.stationary_seconds=0;}
    core.begin_safety_observation();const auto out=core.step(s,now,{},{});
    need(out.command==a::Command::None&&!a::nonzero(out.thrust),"preparation audit may emit only zero targets and no commands");
    if(out.publish)core.note_rc_publication(true,now);
  }
  void warm(){for(unsigned i=0;i<100;++i)tick();need(core.phase()==a::Phase::RcWaitArm,"fixture reaches actual manual readiness");}
};
static void transient_rest_loss_rebuilds_reference_without_creating_a_fault(){
  PreparationAudit q;q.warm();q.s.at_rest=false;q.tick();
  need(q.core.phase()==a::Phase::Idle&&!q.core.rc_ground_reference_qualified()&&
       !q.core.reference_ready()&&!q.core.restart_required(),
    "a disarmed detector rest withdrawal must invalidate the prepared reference and wait");
  q.s.at_rest=true;const double returned=q.now;
  for(unsigned i=0;i<39;++i){q.tick();need(q.core.phase()==a::Phase::Idle,"ground rest must be acquired anew");}
  for(unsigned i=0;i<45;++i)q.tick();
  need(q.core.phase()==a::Phase::RcWaitArm&&q.now-returned>=4&&
       q.core.rc_recovery_diagnostics().first_fault_reason.empty(),
    "fresh rest and a new independent ground reference restore readiness without fault/reset history");
}
static void withdrawn_arm_retry_and_ended_task_recovery_use_distinct_gates(){
  PreparationAudit q;q.warm();q.s.rc_arm_switch=1;q.s.armed=true;
  q.s.armed_time=1;q.s.latest_arming_reason=2;q.s.at_rest=false;q.tick();
  need(q.core.phase()==a::Phase::RcWaitNavigation&&q.core.rc_manual_arm_proven()&&!q.core.flight_started(),
    "manual arm is genuine but still cannot join a flight without the new navigation admission");
  q.s.armed=false;q.s.rc_arm_switch=3;q.s.at_rest=true;q.tick();
  need(q.core.phase()==a::Phase::Safety&&q.core.reason()=="RC_ARM_AUTHORIZATION_WITHDRAWN"&&!q.core.restart_required(),
    "withdrawing arm before navigation retains the existing preparation retry classification");
  for(unsigned i=0;i<39;++i){q.tick();need(!q.core.rc_restart_allowed(q.s),"withdrawn authorization still requires real new ground rest");}
  for(unsigned i=0;i<3;++i)q.tick();
  need(q.core.rc_restart_allowed(q.s)&&!q.core.rc_fault_recovery_ready(),
    "the known unjoined preparation retry can resume before a joined-task recovery window completes");
  q.core=a::Core(q.config);const auto d=q.core.rc_recovery_diagnostics();
  need(d.first_fault_reason.empty()&&d.latest_fault_reason.empty()&&!d.ready&&d.status_samples==0,
    "the next task starts with empty current recovery and fault diagnostics");
  q.warm();need(!q.core.flight_started()&&!q.core.rc_manual_arm_proven(),
    "reprepared readiness cannot inherit the ended manual-arm authorization");
}
int main(){
  first_unsafe_queued_observation_remains_the_visible_blocker();
  time_rewind_inside_the_window_cannot_reuse_progress();
  later_fault_invalidates_ready_counts_and_requires_new_source_watermarks();
  unsafe_feedback_keeps_causal_watermarks_after_clearing_the_window();
  transient_rest_loss_rebuilds_reference_without_creating_a_fault();
  withdrawn_arm_retry_and_ended_task_recovery_use_distinct_gates();
  std::cout<<"RC recovery event, time, fault-watermark and preparation boundaries: PASS\n";
}
