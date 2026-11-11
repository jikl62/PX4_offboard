// RC 独立地面恢复进度回归：不同源频率不必一一配对，仍须各自推进并覆盖完整连续安全窗。
// status测试0.5/1/2 Hz、land1 Hz、local/RC20 Hz；stamp为固定epoch微秒，time为合成单调秒。
#include "authority_test_support.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace a=fc01::attitude;
static void need(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
static a::Config recovery_config(){
  auto c=authority_test::config().attitude;
  c.rc_navigation_enabled=true;c.arm_enabled=false;c.rc_takeover_confirmed=true;
  c.rc_navigation_mode_slot=6;c.max_tilt_deg=10;c.max_flight_seconds=30;
  c.land_sample_max_gap=1.25;c.feedback_guard.enabled=true;c.validate();return c;
}
static std::uint64_t stamp(double time){return 10000000+std::uint64_t(std::llround(time*1e6));}
static double preceding(double time,double period,double phase=0){
  return phase+std::floor((time-phase+1e-8)/period)*period;
}
static a::Feedback safe_feedback(){
  a::Feedback s;
  s.attitude_fresh=s.local_fresh=s.status_fresh=s.land_fresh=s.failsafe_fresh=true;
  s.source_clock_valid=s.source_clock_validated=true;s.source_host_stamp=stamp(0);
  s.z_valid=s.vz_valid=s.landed=s.at_rest=s.exclusive=s.preflight=s.battery_ready=true;
  s.stabilized_selected=true;s.stationary_seconds=3;s.z=2;s.z_deriv=s.vz=0;
  s.rc_link_ready=s.rc_switches_ready=s.rc_source_clock_valid=s.rc_switch_clock_valid=true;
  s.rc_source_selected=s.rc_axes_safe=s.rc_aux_switches_safe=true;
  s.rc_arm_switch=s.rc_kill_switch=3;s.rc_mode_slot=1;return s;
}
// 先锁存真实类别位置不一致故障；input按各自采样相位量化源戳，finish要求所有恢复观察完全静默。
struct Recovery {
  a::Core core; a::Feedback s{safe_feedback()};
  explicit Recovery(a::Config c=recovery_config()):core(c){core.block("VERTICAL_POSITION_RATE_INCONSISTENT");}
  void input(double time,double status_hz=.5,double phase=0){
    s.status_stamp=stamp(preceding(time,1/status_hz,phase));
    s.land_stamp=stamp(preceding(time,1));
    s.local_stamp=s.vertical_sample_stamp=s.rc_switch_sample_stamp=s.source_host_stamp=stamp(time);
  }
  void observe(double time){
    core.begin_safety_observation();finish(time);
  }
  void finish(double time){
    const auto out=core.step(s,time,{},{});
    need(!out.publish&&out.command==a::Command::None,"ground recovery must remain completely output silent");
  }
};
// 不同status频率和相位均应自然在2至3 s恢复；诊断必须披露每路独立新样本数。
static void independent_fresh_sources_recover_at_each_supported_rate(){
  for(double hz:{.5,1.,2.})for(double phase:{0.,.05,.45}){
    Recovery f;double recovered=-1;
    for(unsigned tick=0;tick<=80;++tick){
      const double time=tick*.05;f.input(time,hz,phase);f.observe(time);
      if(f.core.rc_restart_allowed(f.s)){recovered=time;break;}
    }
    need(recovered>=2&&recovered<=3,"independent 0.5/1/2Hz status and 1Hz land must naturally recover");
    const auto d=f.core.rc_recovery_diagnostics();
    need(d.ready&&d.window_active&&d.recoverable&&!d.event_blocked&&d.block_reason.empty(),
      "completed recovery must report current readiness separately from its original fault");
    need(d.status_samples>=2&&d.land_samples>=3&&d.local_samples>=3&&d.elapsed_s>=d.required_s,
      "diagnostics must expose independent source counts and the full elapsed window");
  }
}
// 冻结任一路或短检测burst均不能制造dwell，并给出精确缺进度来源。
static void frozen_sources_and_bursts_cannot_manufacture_dwell(){
  for(unsigned frozen=0;frozen<3;++frozen){
    Recovery f;
    for(unsigned tick=0;tick<=120;++tick){
      const double time=tick*.05;f.input(time);
      if(frozen==0)f.s.status_stamp=stamp(0);
      if(frozen==1)f.s.local_stamp=f.s.vertical_sample_stamp=stamp(0);
      if(frozen==2)f.s.land_stamp=stamp(std::min(time,.2));
      f.observe(time);need(!f.core.rc_restart_allowed(f.s),"replayed source stamps or a short detector burst cannot prove recovery");
    }
    const auto d=f.core.rc_recovery_diagnostics();
    if(frozen==0)need(d.status_samples==1&&d.block_reason=="RC_RECOVERY_WAIT_STATUS_PROGRESS","frozen status needs a precise progress diagnostic");
    if(frozen==1)need(d.local_samples==1&&d.block_reason=="RC_RECOVERY_WAIT_LOCAL_PROGRESS","frozen local data needs a precise progress diagnostic");
  }
}
// 十类过期/不安全/reset/回退撤销旧窗口；恢复健康后必须完整新窗，不能拿前半窗拼接。
static void failures_restart_a_complete_ground_window(){
  for(unsigned failure=0;failure<10;++failure){
    Recovery f;
    for(unsigned tick=0;tick<=20;++tick){f.input(tick*.05,2);f.observe(tick*.05);}
    const auto healthy=f.s;f.input(1.05,2);
    if(failure==0)f.s.status_fresh=false;
    if(failure==1)f.s.local_fresh=false;
    if(failure==2)f.s.source_clock_valid=false;
    if(failure==3)f.s.armed=true;
    if(failure==4)f.s.landed=false;
    if(failure==5)f.s.at_rest=false;
    if(failure==6)++f.s.z_reset;
    if(failure==7)f.s.status_stamp=healthy.status_stamp-1;
    if(failure==8)f.s.local_stamp=f.s.vertical_sample_stamp=healthy.local_stamp-1;
    if(failure==9)f.s.rc_switch_sample_stamp=healthy.rc_switch_sample_stamp-1;
    f.observe(1.05);need(!f.core.rc_restart_allowed(f.s),"a new unsafe observation cannot retain readiness");
    need(!f.core.rc_recovery_diagnostics().block_reason.empty(),"unsafe/reset observations need a visible recovery blocker");
    f.s=healthy;
    double recovered=-1;
    for(unsigned tick=22;tick<=110;++tick){
      const double time=tick*.05;f.input(time,2);f.observe(time);
      if(time<3.05)need(!f.core.rc_restart_allowed(f.s),"fresh data after a failure must establish a complete new dwell");
      if(f.core.rc_restart_allowed(f.s)){recovered=time;break;}
    }
    need(recovered>=3.05&&recovered<=5,"a transient input/reset condition must permit a later healthy ground recovery");
  }
  Recovery gap;gap.input(0,2);gap.observe(0);gap.input(1.5,2);gap.observe(1.5);
  const auto d=gap.core.rc_recovery_diagnostics();
  need(!d.ready&&d.land_samples==1&&d.elapsed_s==0,"a detector/local gap must seed a new proof rather than join old observations");
}
// 逐路只推进status、land、local、RC，核对每个故障后新水位等待原因。
static void fault_baselines_require_new_evidence_from_every_source(){
  auto c=recovery_config();a::Core core(c);auto s=safe_feedback();
  s.status_stamp=s.land_stamp=s.local_stamp=s.vertical_sample_stamp=s.rc_switch_sample_stamp=stamp(0);
  core.step(s,0,{},{});core.block("Z_VZ_STALE_OR_INVALID");
  const char* reasons[]={"RC_RECOVERY_WAIT_NEW_STATUS","RC_RECOVERY_WAIT_NEW_LAND",
                         "RC_RECOVERY_WAIT_NEW_LOCAL","RC_RECOVERY_WAIT_NEW_RC_SWITCHES"};
  for(unsigned source=0;source<4;++source){
    core.begin_safety_observation();core.observe_safety_terminal(s,1+.05*source);
    need(core.rc_recovery_diagnostics().block_reason==reasons[source],"fault baselines must identify exactly which source has not advanced");
    if(source==0)++s.status_stamp;
    if(source==1)++s.land_stamp;
    if(source==2)s.vertical_sample_stamp=++s.local_stamp;
    if(source==3)++s.rc_switch_sample_stamp;
  }
}
// 同周期坏landed后好尾帧仍不可恢复，必须保持event_blocked并清窗口。
static void queued_bad_events_cannot_be_erased_by_a_benign_tail(){
  Recovery f;f.input(0,2);f.observe(0);f.input(.05,2);
  f.core.begin_safety_observation();auto bad=f.s;bad.landed=false;
  f.core.observe_safety_event(bad,a::Core::SafetyEvent::Land,.05);
  f.core.observe_safety_event(f.s,a::Core::SafetyEvent::Land,.05);f.finish(.05);
  const auto d=f.core.rc_recovery_diagnostics();
  need(d.event_blocked&&!d.window_active&&d.land_samples==0&&
    d.block_reason=="RC_RECOVERY_QUEUED_EVENT_RC_RECOVERY_LANDED_AT_REST_REQUIRED",
    "queued unsafe evidence must remain visible and veto this cycle even behind healthy data");
}
// 仅Vz与位置导数速度差诊断，且Vz仍在地面速度包络内时，不应反复清安全地面窗。
static void healthy_rate_diagnostics_do_not_block_ground_recovery(){
  auto c=recovery_config();c.feedback_guard.rate_tolerance_mps=.05;Recovery f(c);f.s.vz=.09;
  for(unsigned tick=0;tick<=50;++tick){
    const double time=tick*.05;f.input(time,1);f.core.begin_safety_observation();
    f.core.observe_safety_event(f.s,a::Core::SafetyEvent::Vertical,time);
    f.core.observe_safety_event(f.s,a::Core::SafetyEvent::Battery,time);f.finish(time);
  }
  need(f.core.rc_restart_allowed(f.s),"healthy queued observations and velocity-only warnings must not reset recovery");
}
// 第一根因与最新故障分开；执行完整性/未知故障继续要求重启，不可由正常地面帧抹掉。
static void fault_history_and_integrity_rules_remain_distinct(){
  Recovery f;f.core.block("BATTERY_NOT_READY");
  auto d=f.core.rc_recovery_diagnostics();
  need(d.first_fault_reason=="VERTICAL_POSITION_RATE_INCONSISTENT"&&d.latest_fault_reason=="BATTERY_NOT_READY"&&
    f.core.reason()==d.first_fault_reason,"first fault history must survive later fault causes");
  for(const char* integrity:{"TELEMETRY_TIME_REGRESSION","PUBLISHER_BOUNDARY_CONTROL_LOOP_STALL",
                            "PARTIAL_OR_FAILED_SETPOINT_PUBLICATION","UNKNOWN_FAULT"}){
    Recovery blocked;blocked.core.block(integrity);
    for(unsigned tick=0;tick<=100;++tick){blocked.input(tick*.05,2);blocked.observe(tick*.05);}
    d=blocked.core.rc_recovery_diagnostics();
    need(!d.recoverable&&!d.ready&&d.latest_fault_reason==integrity&&
      d.block_reason=="RC_RECOVERY_FAULT_REQUIRES_RESTART"&&!blocked.core.rc_restart_allowed(blocked.s),
      "independent source progress must never clear an integrity or unknown fault");
  }
}
int main(){
  independent_fresh_sources_recover_at_each_supported_rate();
  frozen_sources_and_bursts_cannot_manufacture_dwell();
  failures_restart_a_complete_ground_window();
  fault_baselines_require_new_evidence_from_every_source();
  queued_bad_events_cannot_be_erased_by_a_benign_tail();
  healthy_rate_diagnostics_do_not_block_ground_recovery();
  fault_history_and_integrity_rules_remain_distinct();
  std::cout<<"RC independent recovery progress, diagnostics and retained integrity guards: PASS\n";
  return 0;
}
