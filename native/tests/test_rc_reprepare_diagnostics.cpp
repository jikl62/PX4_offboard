// Exercise Runtime admission against a genuinely ended RC task. No ROS/DDS.
#define main existing_rc_navigation_test_main
#include "test_rc_navigation.cpp"
#undef main

static void safe_ground_after_takeover(Flight& q){
  q.join();q.s.feedback.offboard=false;q.s.nav_state=15;
  q.s.attitude.rc_mode_slot=1;++q.s.attitude.operator_switch_generation;q.tick();
  need(q.r.attitude_core().reason()=="OPERATOR_TAKEOVER_LATCHED","actual mode exit ends task");
  q.s.feedback.armed=false;q.s.feedback.landed=q.s.at_rest=true;
  q.s.stationary_seconds=3;q.s.attitude.rc_arm_switch=3;
  for(unsigned i=0;i<60;++i)q.tick();
  need(q.r.rc_restart_allowed(q.s),"independent new ground evidence qualifies task recovery");
  need(q.r.rc_reprepare_block_reason(q.s,q.t).empty(),"healthy ended task has no admission blocker");
}
static void current_runtime_blockers_are_visible_and_do_not_clear_history(){
  Flight q;safe_ground_after_takeover(q);
  for(unsigned failure=0;failure<6;++failure){auto s=q.s;const char* expected="";
    if(failure==0){s.publication_fault_latched=true;expected="RC_PUBLICATION_FAULT_LATCHED";}
    if(failure==1){s.time_regressed=true;expected="RC_TIME_REGRESSION_LATCHED";}
    if(failure==2){s.control_cycle_fresh=false;expected="RC_CONTROL_CYCLE_NOT_FRESH";}
    if(failure==3){s.attitude_output_active=true;expected="RC_OUTPUT_STILL_ACTIVE";}
    if(failure==4){s.offboard_heartbeat_active=true;expected="RC_OUTPUT_STILL_ACTIVE";}
    if(failure==5){s.feedback.armed=true;expected="RC_CORE_RESTART_CONDITIONS_NOT_MET";}
    need(q.r.rc_reprepare_block_reason(s,q.t)==expected,"read-only reason identifies current Runtime veto");
    need(!q.r.reprepare_rc(s,q.t)&&q.r.rc_attempts_completed()==0,"blocked admission never rebuilds task");
  }
  need(q.r.rc_reprepare_block_reason(q.s,q.t).empty(),"diagnostic query never creates a latch");
  need(q.r.reprepare_rc(q.s,q.t),"healthy ground consumes exactly the shared admission gates");
  q.warm();need(q.r.attitude_core().phase()==a::Phase::RcWaitArm&&q.arm_commands==0&&q.mode_commands==0,
    "recovery returns to waiting for new manual arm without automatic commands");
}
static void unresolved_command_outcomes_remain_blocking(){
  for(unsigned command:{176U,400U,21U}){
    Flight q;safe_ground_after_takeover(q);
    auto& p=const_cast<f::Pending&>(q.r.pending(command));
    p.requested=p.sent=p.published=p.timed_out=true;p.wire_stamp=q.s.status_stamp;
    if(command==400)p.command400_kind=f::Command400Kind::Arm;
    const auto expected="RC_COMMAND_"+std::to_string(command)+"_OUTCOME_UNRESOLVED";
    need(q.r.rc_reprepare_block_reason(q.s,q.t)==expected,"timeout still reports unresolved exact command");
    need(!q.r.reprepare_rc(q.s,q.t)&&p.requested&&p.timed_out&&p.wire_stamp==q.s.status_stamp,
      "diagnostics and failed reprepare preserve wire evidence");
    p.state_verified=true;
    need(q.r.rc_reprepare_block_reason(q.s,q.t).empty(),"proven terminal command removes only that blocker");
    need(q.r.reprepare_rc(q.s,q.t)&&q.r.pending(command).wire_stamp==q.s.status_stamp,
      "rebuild retains settled command history");
  }
}
static void later_fatal_fault_does_not_hide_behind_takeover(){
  Flight q;safe_ground_after_takeover(q);
  q.r.abort_attitude("PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_UNKNOWN_EXCEPTION");q.tick();
  const auto d=q.r.attitude_core().rc_recovery_diagnostics();
  need(d.first_fault_reason=="OPERATOR_TAKEOVER_LATCHED"&&
    d.latest_fault_reason=="PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_UNKNOWN_EXCEPTION"&&!d.recoverable,
    "diagnostics expose later fatal failure without losing the first fault");
  need(!q.r.reprepare_rc(q.s,q.t),"fault diagnosis does not relax integrity latch");
}
int main(){
  current_runtime_blockers_are_visible_and_do_not_clear_history();
  unresolved_command_outcomes_remain_blocking();later_fatal_fault_does_not_hide_behind_takeover();
  std::cout<<"RC_REPREPARE_DIAGNOSTICS_PASS\n";
}
