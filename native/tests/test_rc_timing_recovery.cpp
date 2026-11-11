// 未解锁准备截止超时恢复回归：仅已知严格零流截止拒绝可恢复，重复超时与未知异常仍拦截。
// 真实RC夹具只产生本地假sink，新增恢复证明期间完全静默，不能用早先快速tick证明整个周期健康。
// Existing synthetic RC fixture; no ROS/DDS or hardware sinks.
#define main rc_navigation_fixture_main
#include "test_rc_navigation.cpp"
#undef main

// 三种已知截止原因逐一验证：晚期overrun、长调度gap、重复stall都清窗；完整新健康地面窗才重准备。
static void preparation_deadline_needs_healthy_new_cycle() {
  for (
      const char* reason :
      {"ROS_ADAPTER_CONTROL_LOOP_STALL", "PUBLISHER_BOUNDARY_CONTROL_LOOP_STALL",
       "PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_HEARTBEAT_FINAL_CONTEXT_CHANGED_CONTROL_LOOP_STALL"}) {
    Flight q;
    q.warm();
    const auto commits = q.heartbeat_commits;
    need(q.r.abort_rc_preparation_timing(reason, q.s),
         "known zero-preparation deadline has explicit recovery scope");
    q.s.publication_fault_latched = true;
    for (int i = 0; i < 44; ++i) {
      q.tick();
      need(!q.o.attitude.publish, "fault never continues publication");
    }
    // A repeated overrun invalidates previously accumulated health proof.
    q.s.control_cycle_fresh = false;
    for (int i = 0; i < 45; ++i) {
      q.tick();
      need(!q.r.reprepare_rc(q.s, q.t), "continuing stalls cannot clear the fault");
    }
    q.s.control_cycle_fresh = true;
    q.tick();
    need(!q.r.reprepare_rc(q.s, q.t), "one fast cycle cannot erase repeated stalls");
    for (int i = 0; i < 44; ++i)
      q.tick();
    auto late = q.s;
    late.control_cycle_fresh = false;
    need(!q.r.reprepare_rc(late, q.t), "overrun after Runtime step also invalidates ground dwell");
    q.tick();
    need(!q.r.reprepare_rc(q.s, q.t), "next early-fast cycle cannot reuse a late failed cycle");
    for (int i = 0; i < 44; ++i)
      q.tick();
    q.tick(true, .4);
    need(!q.r.reprepare_rc(q.s, q.t),
         "long scheduling gap cannot accumulate apparently fast recovery ticks");
    for (int i = 0; i < 44; ++i)
      q.tick();
    for (int bad = 0; bad < 6; ++bad) {
      auto current = q.s;
      if (bad == 0)
        current.feedback.armed = true;
      if (bad == 1)
        current.feedback.offboard = true;
      if (bad == 2)
        current.feedback.landed = false;
      if (bad == 3)
        current.attitude.kill = true;
      if (bad == 4)
        current.status_fresh = false;
      if (bad == 5)
        current.control_cycle_fresh = false;
      need(!q.r.reprepare_rc(current, q.t), "ground recovery consumes current safe evidence only");
    }
    for (int i = 0; i < 44; ++i)
      q.tick();
    need(q.r.reprepare_rc(q.s, q.t),
         "new healthy ground cycle can consume classified deadline latch");
    need(q.r.last_rc_attempt_reason() == reason && q.r.rc_attempts_completed() == 1,
         "history retains exact original publication fault");
    need(q.heartbeat_commits == commits, "recovery proof emits no heartbeat or target");
    q.s.publication_fault_latched = false;
    q.warm();
    need(q.r.attitude_core().phase() == a::Phase::RcWaitArm && q.arm_commands == 0 &&
             q.mode_commands == 0,
         "recovery reaches standby without automatic commands");
    q.manual_arm();
    q.navigation();
    need(q.r.attitude_core().phase() == a::Phase::ArmedGround,
         "only a new manual RC arm and new Offboard episode can start next task");
  }
}
// 已armed/Offboard、未知arming、Arm意图、倒时、未知sink异常或已加入任务均不得借准备截止范围恢复。
static void unknown_or_armed_failure_cannot_acquire_recovery_scope() {
  for (int unsafe = 0; unsafe < 6; ++unsafe) {
    Flight q;
    q.warm();
    if (unsafe == 0)
      q.s.feedback.armed = true;
    if (unsafe == 1)
      q.s.feedback.offboard = true;
    if (unsafe == 2)
      q.s.arming_state_known = false;
    if (unsafe == 3)
      q.s.attitude.rc_arm_switch = 1;
    if (unsafe == 4)
      q.r.abort_attitude("PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_sink failed");
    if (unsafe == 5)
      q.s.time_regressed = true;
    if (unsafe == 3)
      q.tick();
    need(!q.r.abort_rc_preparation_timing("PUBLISHER_BOUNDARY_CONTROL_LOOP_STALL", q.s),
         "only untouched disarmed preparation can receive timing recovery scope");
  }
  Flight unknown;
  unknown.warm();
  need(!unknown.r.abort_rc_preparation_timing(
           "PARTIAL_OR_FAILED_SETPOINT_PUBLICATION_UNKNOWN_EXCEPTION", unknown.s),
       "unknown sink failure is never classified as a deadline");
  Flight active;
  active.join();
  need(!active.r.abort_rc_preparation_timing("ROS_ADAPTER_CONTROL_LOOP_STALL", active.s),
       "airborne or joined custody cannot enter untouched-preparation recovery");
}
int main() {
  preparation_deadline_needs_healthy_new_cycle();
  unknown_or_armed_failure_cannot_acquire_recovery_scope();
  std::cout << "RC_TIMING_FAULT_SAFE_RECOVERY_PASS\n";
}
