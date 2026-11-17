// 只用合成反馈调用真实 Runtime/Core/发布边界；不链接 ROS，不访问串口或设备。
// 用板端已采集配置复现状态门，不能当作机体动力学或真实故障根因证明。
#include "vision_test_support.hpp"
#include "jikl_fc01_system/source_clock.hpp"
#include "captured_config.hpp"
#include <iomanip>
#include <iostream>
using namespace vision_test;
static void row(const char* test, const VisionFlight& q) {
  std::cout << std::setprecision(12)
            << "{\"case\":\"" << test << "\",\"t\":" << q.t
            << ",\"phase\":\"" << a::name(q.r.attitude_core().phase())
            << "\",\"reason\":\"" << q.r.attitude_core().reason()
            << "\",\"height\":" << q.r.attitude_core().reference_z()-q.s.attitude.z
            << ",\"z0\":" << q.r.attitude_core().reference_z()
            << ",\"up_speed\":" << -q.s.attitude.vz
            << ",\"command_height\":" << q.o.attitude.target_height
            << ",\"liftoff\":" << q.r.attitude_core().liftoff_confirmed()
            << ",\"settled\":" << q.r.attitude_core().target_height_settled()
            << ",\"publish_plan\":" << q.o.attitude.publish
            << ",\"heartbeat_commits\":" << q.heartbeat_commits
            << ",\"target_commits\":" << q.target_commits << "}\n";
}
static void ramp(VisionFlight& q, const char* label) {
  q.join(); row(label,q);
  for(int i=0;i<100 && q.r.attitude_core().phase()!=a::Phase::ThrustRamp;++i) q.tick();
  need(q.r.attitude_core().phase()==a::Phase::ThrustRamp,"RAMP_NOT_REACHED"); row(label,q);
}
static void climb(VisionFlight& q, const char* label, double goal) {
  double height=0;
  while(height<goal-1e-9) {
    height=std::min(goal,height+.01);
    q.observe(height,.2); q.s.feedback.landed=q.s.at_rest=false; q.tick(); row(label,q);
    need(q.r.attitude_core().phase()!=a::Phase::Safety,"UNEXPECTED_SAFETY_DURING_CLIMB");
  }
}
int main() {
  const auto c=captured_config();
  need(!c.attitude.visual.enabled && !c.attitude.arm_enabled,"PRODUCTION_GATES_CHANGED");
  {
    VisionFlight q(c); ramp(q,"normal_1p5"); climb(q,"normal_1p5",1.5);
    q.observe(1.5,0);
    for(int i=0;i<100 && q.r.attitude_core().phase()!=a::Phase::HeightHold;++i) {q.tick();row("normal_1p5",q);}
    need(q.r.attitude_core().phase()==a::Phase::HeightHold,"NORMAL_HOLD_NOT_REACHED");
    need(q.arm_commands==0 && q.mode_commands==0,"AUTOMATIC_RC_COMMAND");
    const double z0=q.r.attitude_core().reference_z();
    const auto heartbeats=q.heartbeat_commits, targets=q.target_commits;
    // 四元数保持完全相同，仅增加参考重置计数：验证故障看的是参考不连续事件。
    ++q.s.attitude.quat_reset; q.tick(); row("active_reset_same_quaternion",q);
    need(q.r.attitude_core().phase()==a::Phase::Safety,"RESET_NOT_LATCHED");
    need(q.r.attitude_core().reason()=="ACTIVE_QUATERNION_RESET","WRONG_RESET_REASON");
    for(int i=0;i<20;++i) {q.tick();row("healthy_after_reset",q);}
    need(!q.o.attitude.publish && q.heartbeat_commits==heartbeats && q.target_commits==targets,
         "HEALTHY_FRAME_RESTARTED_PUBLICATION");
    need(q.r.attitude_core().reference_z()==z0,"ACTIVE_RESET_REBASED_Z0");
  }
  {
    VisionFlight q(c); ramp(q,"landed_blocks_liftoff");
    for(int i=0;i<12;++i) {q.observe(.13+.01*i,.2);q.tick();row("landed_blocks_liftoff",q);}
    need(q.r.attitude_core().phase()==a::Phase::ThrustRamp && !q.r.attitude_core().liftoff_confirmed(),
         "HEIGHT_ALONE_CERTIFIED_LIFTOFF");
    need(q.o.attitude.publish,"LANDED_CACHE_UNEXPECTED_STOP");
  }
  {
    VisionFlight q(c); ramp(q,"cross_height_without_settling");
    climb(q,"cross_height_without_settling",1.70);
    q.observe(1.70,0);
    for(int i=0;i<120 && q.r.attitude_core().phase()!=a::Phase::Descent;++i) {q.tick();row("cross_height_without_settling",q);}
    need(q.r.attitude_core().phase()==a::Phase::Descent,"ASCENT_TIMEOUT_NOT_REACHED");
    need(q.r.attitude_core().reason()=="ASCENT_TIMEOUT_SUPERVISED_RECOVERY","WRONG_ASCENT_TIMEOUT");
    need(!q.r.attitude_core().target_height_settled() && q.o.attitude.publish,"TIMEOUT_CUT_PUBLICATION");
  }
  // 板端 journal 的同一通用原因可来自不同不安全事实；不能据原因名反推唯一根因。
  for (bool stick_event : {false,true}) {
    VisionFlight q(c); q.warm(); q.manual_arm();
    need(q.r.attitude_core().phase()==a::Phase::RcWaitNavigation,"WAIT_NAVIGATION_NOT_REACHED");
    const auto good=q.r.publication_context(q.s,q.t,true,true).feedback;
    auto bad=good;
    if(stick_event) bad.rc_sticks_moving=true;
    else bad.failsafe=true;
    q.r.begin_attitude_safety_observation();
    q.r.observe_attitude_safety_event(bad,a::Core::SafetyEvent::Failsafe,q.t);
    q.r.observe_attitude_safety_event(good,a::Core::SafetyEvent::Status,q.t);
    q.tick(); row(stick_event?"queued_stick_then_healthy":"queued_failsafe_then_healthy",q);
    need(q.r.attitude_core().phase()==a::Phase::Safety && !q.o.attitude.publish,
         "QUEUED_HAZARD_OVERWRITTEN_BY_GOOD_SNAPSHOT");
    need(q.r.attitude_core().reason()=="RC_QUEUED_SAFETY_CONDITION","WRONG_QUEUED_REASON");
  }
  {
    f::SourceSampleProgress p; const std::uint64_t epoch=1700000000000000ULL;
    p.observe(epoch,epoch,0); p.observe(epoch+400000,epoch,.4);
    need(p.received==0 && !p.fresh(epoch+600000,.6,.5),"ENVELOPE_RENEWED_OLD_RAW_SAMPLE");
    p.observe(epoch+700000,epoch-1,.7);
    p.observe(epoch+800000,epoch+800000,.8);
    need(p.regressed && !p.fresh(epoch+800000,.8,.5),"REGRESSION_WAS_CLEARED");
  }
  std::cerr << "STAGE1_REPRO_PASS: normal hold, active reset latch/no further commits, landed gate, ascent timeout, queued failsafe/stick latch, raw freshness/regression\n";
}
