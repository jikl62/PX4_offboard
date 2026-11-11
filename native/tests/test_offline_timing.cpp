// 离线时间合同回归：分开源时钟、原始采样、回调到达寿命、主机跳时和当前控制周期。
// epoch为任意固定同步微秒域，不要求日历准确；elapsed/receipt/cycle均以独立单调秒计算。
// Mode/Target仅保留字段形状；测试用sink计数确定目标已发而心跳被拒绝的部分发布。
#include "jikl_fc01_system/source_diagnostics.hpp"
#include "jikl_fc01_system/runtime.hpp"
#include "jikl_fc01_system/publication_boundary.hpp"
#include "jikl_fc01_system/control_diagnostics.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <string>

namespace f=fc01;
namespace a=fc01::attitude;
constexpr std::uint64_t epoch=1794000000000000ULL;
static void close(double actual,double expected){assert(std::abs(actual-expected)<1e-9);}

// 稳定但不准确的日历同样可算年龄；精确检查空戳、未来戳、raw晚于envelope与过期各自原因。
static void offline_epoch_and_precise_reasons(){
  // A stable, inaccurate calendar has exactly the same elapsed-time rules.
  for(std::uint64_t host:std::array<std::uint64_t,3>{1000000ULL,1234000000000ULL,epoch}){
    const auto d=f::diagnose_source_timing(host-489000,host-490000,true,host,4.,3.99,.5);
    assert(d.source_reason==f::SourceTimeReason::Valid);
    assert(d.receipt_reason==f::ReceiptTimeReason::Valid);
    close(d.envelope_age.seconds,.489);close(d.sample_age.seconds,.490);
    close(d.envelope_sample_gap.seconds,.001);close(d.receipt_age.seconds,.01);
    assert(d.sample_age.available&&d.envelope_sample_gap.available);
    f::HostClockMonitor monitor;
    assert(monitor.observe(host,0).change==f::HostClockChange::Initial);
    const auto stable=monitor.observe(host+50000,.05);
    assert(stable.change==f::HostClockChange::Consistent);close(stable.delta_error.seconds,0);
  }
  const auto expired=f::diagnose_source_timing(epoch-600000,epoch-601000,true,epoch,4.,4.,.5);
  assert(expired.source_reason==f::SourceTimeReason::EnvelopeExpired);
  assert(expired.receipt_reason==f::ReceiptTimeReason::Valid);
  const auto frozen=f::diagnose_source_timing(epoch,epoch-600000,true,epoch,4.,4.,.5);
  assert(frozen.source_reason==f::SourceTimeReason::SampleExpired);
  const auto absent=f::diagnose_source_timing(epoch,0,true,epoch,4.,-1,.5);
  assert(absent.source_reason==f::SourceTimeReason::MissingSample);
  assert(!absent.sample_age.available&&!absent.envelope_sample_gap.available);
  assert(absent.receipt_reason==f::ReceiptTimeReason::MissingReceipt);
  assert(!absent.receipt_age.available);
  assert(f::source_time_reason(epoch,epoch+1,true,epoch,.5)==f::SourceTimeReason::SampleAfterEnvelope);
  assert(f::source_time_reason(epoch+100001,0,false,epoch,.5)==f::SourceTimeReason::EnvelopeTooFarFuture);
  assert(f::source_time_reason(epoch,0,false,0,.5)==f::SourceTimeReason::MissingHostClock);
  assert(f::source_time_reason(epoch,0,false,epoch,0)==f::SourceTimeReason::InvalidTimeout);
  assert(f::receipt_time_reason(4,4.5,.5)==f::ReceiptTimeReason::Expired);
  assert(f::receipt_time_reason(4,3.9,.5)==f::ReceiptTimeReason::ReceiptFromFuture);
  f::SourceTimingSet all;
  assert(all.size()==6);
  assert(std::string(f::source_slot_name(f::SourceSlot::Battery))=="battery");
  // At the inclusive source boundary the independent receive TTL stays strict.
  assert(f::source_time_reason(epoch-500000,0,false,epoch,.5)==f::SourceTimeReason::Valid);
  std::cout<<"OFFLINE_STABLE_ARBITRARY_CALENDAR_AND_EXACT_SOURCE_REASONS PASS\n";
}

// RMW接收时间不可用时必须保留unavailable，不能显示0延迟；有测量时保留其真实正负差。
static void callback_observations_do_not_invent_measurements(){
  const auto unavailable=f::observe_source_receipt(epoch-489000,epoch-490000,true,epoch,7.);
  assert(!unavailable.middleware_callback_age.available);
  close(unavailable.envelope_host_age.seconds,.489);close(unavailable.sample_host_age.seconds,.490);
  const auto measured=f::observe_source_receipt(epoch-489000,epoch-490000,true,epoch,7.,
    static_cast<std::int64_t>((epoch-20000)*1000));
  assert(measured.middleware_callback_age.available);close(measured.middleware_callback_age.seconds,.020);
  const auto future=f::observe_source_receipt(epoch,0,false,epoch,7.,
    static_cast<std::int64_t>((epoch+1000)*1000));
  assert(future.middleware_callback_age.available);close(future.middleware_callback_age.seconds,-.001);
  assert(!future.sample_host_age.available);
  const auto no_host=f::observe_source_receipt(epoch,epoch,true,0,7.,1);
  assert(!no_host.envelope_host_age.available&&!no_host.middleware_callback_age.available);
  const auto invalid_rmw=f::observe_source_receipt(epoch,epoch,true,epoch,7.,-1);
  assert(!invalid_rmw.middleware_callback_age.available);
  std::cout<<"OPTIONAL_RMW_RECEIPT_UNAVAILABLE_IS_NOT_ZERO_DELAY PASS\n";
}

// 比较wall增量与steady增量识别前跳/后跳；诊断不能通过增补偏移把旧数据重新合法化。
static void host_jumps_are_separate_from_source_delay(){
  f::HostClockMonitor monitor;
  monitor.observe(epoch,10.);
  auto result=monitor.observe(epoch+50000,10.05);
  assert(result.change==f::HostClockChange::Consistent);
  // Monotonic time advances 50 ms while wall time advances an extra 300 ms.
  result=monitor.observe(epoch+400000,10.1);
  assert(result.change==f::HostClockChange::ForwardJump);
  close(result.delta_error.seconds,.3);close(result.anchor_error.seconds,.3);
  result=monitor.observe(epoch+200000,10.15);
  assert(result.change==f::HostClockChange::BackwardJump&&result.host_regressed);
  close(result.delta_error.seconds,-.25);
  result=monitor.observe(epoch+250000,10.);
  assert(result.change==f::HostClockChange::SteadyRegression&&result.steady_regressed);
  assert(monitor.observe(0,10.2).change==f::HostClockChange::Unavailable);
  const auto delayed=f::diagnose_source_timing(epoch,epoch,true,epoch+510000,1.,1.,.5);
  assert(delayed.source_reason==f::SourceTimeReason::EnvelopeExpired);
  const auto backwards=f::diagnose_source_timing(epoch,epoch,true,epoch-200000,1.,1.,.5);
  assert(backwards.source_reason==f::SourceTimeReason::EnvelopeTooFarFuture);
  // Recording a clock jump does not create a safety offset or legalize data.
  assert(!f::source_sample_in_clock(epoch,epoch,epoch+510000,.5));
  assert(!f::source_sample_in_clock(epoch,epoch,epoch-200000,.5));
  std::cout<<"OFFLINE_FORWARD_BACKWARD_HOST_JUMPS_REMAIN_FAIL_CLOSED PASS\n";
}

// 新外壳不能续期冻结raw；时间回退锁存不被正常尾帧或新进程中的错误时钟域绕过。
static void raw_lifetime_and_regression_latches_are_preserved(){
  struct Estimate{std::uint64_t timestamp{},timestamp_sample{};};
  Estimate held{},input{epoch,epoch};
  f::Stamp envelope;f::SourceSampleProgress raw;
  assert(f::receive_sampled_source(input,held,envelope,raw,0));
  input.timestamp+=500000;
  assert(f::receive_sampled_source(input,held,envelope,raw,.5));
  assert(envelope.fresh(.5,.5)&&!raw.fresh(input.timestamp,.5,.5));
  // A fresh envelope cannot extend raw lifetime, and a later good sample works.
  input.timestamp+=10000;input.timestamp_sample=input.timestamp;
  assert(f::receive_sampled_source(input,held,envelope,raw,.51));
  assert(raw.fresh(input.timestamp,.51,.5));
  input.timestamp-=1000;
  assert(!f::receive_sampled_source(input,held,envelope,raw,.52));
  assert(envelope.regressed&&!envelope.fresh(.52,.5));
  input.timestamp+=100000;input.timestamp_sample=input.timestamp;
  assert(!f::receive_sampled_source(input,held,envelope,raw,.53));
  assert(envelope.regressed); // neither diagnostics nor newer data clear it
  f::SourceSampleProgress reset;
  reset.observe(epoch,epoch,0);
  reset.observe(epoch+10000,1000000,.01);
  assert(reset.regressed&&!reset.fresh(epoch+10000,.01,.5));
  f::Stamp process_restarted;
  assert(process_restarted.accept(1000000,.01));
  assert(!f::source_stamp_in_clock(process_restarted.value,epoch,.5));
  // A fresh process has no previous latch but wrong clock-domain data still fail.
  std::cout<<"FROZEN_RAW_REORDER_PX4_RESET_AND_PROCESS_RESTART_STAY_DISTINCT PASS\n";
}

struct Mode {
  std::uint64_t timestamp{};
  bool position{},velocity{},acceleration{},attitude{},body_rate{},thrust_and_torque{},direct_actuator{};
};
struct Target {
  std::uint64_t timestamp{};float yaw_sp_move_rate{};
  std::array<float,4> q_d{};std::array<float,3> thrust_body{};
  bool reset_integral{},fw_control_yaw_wheel{};
};
static a::BoundaryContext prepared(){
  a::BoundaryContext c;c.profile="attitude_altitude";c.phase=a::Phase::Prestream;
  c.session_connected=c.control_fresh=c.reference_ready=true;
  auto& s=c.feedback;
  s.attitude_fresh=s.local_fresh=s.status_fresh=s.land_fresh=s.failsafe_fresh=true;
  s.z_valid=s.vz_valid=s.landed=s.at_rest=s.exclusive=s.preflight=s.battery_ready=s.manual_ready=true;
  s.source_clock_valid=s.source_clock_validated=true;
  s.q={1,0,0,0};s.stationary_seconds=3;s.stabilized_selectable=true;
  return c;
}

// 仅local/attitude源年龄过期且实际到达/raw进展新鲜才能归入窄准备重试；这仍不允许过期输出。
static void preparation_expiry_classification_never_legalizes_stale_output(){
  constexpr double timeout=.5;
  const auto local=std::size_t(f::SourceSlot::Local);
  const auto attitude=std::size_t(f::SourceSlot::Attitude);
  auto sources_at=[&](std::uint64_t host,double now,double received){
    f::SourceTimingSet sources;
    for(auto& source:sources)
      source=f::diagnose_source_timing(host,0,false,host,now,received,timeout);
    for(auto slot:{local,attitude})
      sources[slot]=f::diagnose_source_timing(epoch,epoch,true,host,now,received,timeout);
    return sources;
  };
  const std::array<f::ReceiptTimeReason,2> raw_fresh{
    f::ReceiptTimeReason::Valid,f::ReceiptTimeReason::Valid};
  assert(!f::only_preparation_source_age_expired(sources_at(epoch+490000,4.,4.),raw_fresh));
  assert(!f::only_preparation_source_age_expired(sources_at(epoch+500000,4.01,4.01),raw_fresh));
  auto expired=sources_at(epoch+510000,4.02,4.02);
  close(expired[local].sample_age.seconds,.51);
  close(expired[local].receipt_age.seconds,0);
  assert(f::only_preparation_source_age_expired(expired,raw_fresh));

  // A newly received raw sample can advance locally while still being older
  // than the independent, unchanged 500 ms source-age limit.
  f::SourceSampleProgress raw;
  raw.observe(epoch-1000,epoch-1000,3.9);
  raw.observe(epoch,epoch,4.02);
  close(raw.received,4.02);
  assert(f::receipt_time_reason(raw.received,4.02,timeout)==f::ReceiptTimeReason::Valid);
  assert(!raw.fresh(epoch+510000,4.02,timeout));
  unsigned targets=0,heartbeats=0;
  a::Output output;output.publish=true;
  auto context=prepared();
  context.feedback.source_clock_valid=f::source_sample_in_clock(epoch,epoch,epoch+510000,timeout);
  auto decision=a::publish_attitude_resampled<Mode,Target>([&]{return context;},output,epoch,
    [&](const Target&){++targets;},[&](const Mode&){++heartbeats;});
  assert(!decision.allowed&&decision.reason==a::BoundaryReason::SourceClock);
  assert(targets==0&&heartbeats==0);

  // Either sampled source may expire alone, including a frozen raw sample
  // inside a fresh envelope. No other source may use this retry category.
  for(auto slot:{local,attitude}){
    auto one=sources_at(epoch+510000,4.02,4.02);
    const auto other=slot==local?attitude:local;
    one[other]=f::diagnose_source_timing(epoch+510000,epoch+510000,true,
      epoch+510000,4.02,4.02,timeout);
    assert(f::only_preparation_source_age_expired(one,raw_fresh));
    one[slot]=f::diagnose_source_timing(epoch+510000,epoch,true,
      epoch+510000,4.02,4.02,timeout);
    assert(one[slot].source_reason==f::SourceTimeReason::SampleExpired);
    assert(f::only_preparation_source_age_expired(one,raw_fresh));
  }
  for(std::size_t slot=0;slot<expired.size();++slot){
    auto bad=expired;
    const bool sampled=slot==local||slot==attitude;
    bad[slot]=f::diagnose_source_timing(0,0,sampled,epoch+510000,4.02,4.02,timeout);
    assert(!f::only_preparation_source_age_expired(bad,raw_fresh));
    bad=expired;
    bad[slot]=f::diagnose_source_timing(epoch+610001,epoch+610001,sampled,
      epoch+510000,4.02,4.02,timeout);
    assert(bad[slot].source_reason==f::SourceTimeReason::EnvelopeTooFarFuture);
    assert(!f::only_preparation_source_age_expired(bad,raw_fresh));
    bad=expired;
    bad[slot].receipt_reason=f::receipt_time_reason(3.5,4.02,timeout);
    assert(bad[slot].receipt_reason==f::ReceiptTimeReason::Expired);
    assert(!f::only_preparation_source_age_expired(bad,raw_fresh));
    if(!sampled){
      bad=expired;
      bad[slot]=f::diagnose_source_timing(epoch,0,false,epoch+510000,4.02,4.02,timeout);
      assert(!f::only_preparation_source_age_expired(bad,raw_fresh));
    }
  }
  for(auto slot:{local,attitude}){
    for(auto sample:{std::uint64_t(0),epoch+510001}){
      auto bad=expired;
      bad[slot]=f::diagnose_source_timing(epoch+510000,sample,true,
        epoch+510000,4.02,4.02,timeout);
      assert(!f::only_preparation_source_age_expired(bad,raw_fresh));
    }
  }
  for(std::size_t slot=0;slot<raw_fresh.size();++slot){
    for(auto reason:{f::ReceiptTimeReason::Expired,f::ReceiptTimeReason::MissingReceipt,
        f::ReceiptTimeReason::ReceiptFromFuture,f::ReceiptTimeReason::InvalidNow,
        f::ReceiptTimeReason::InvalidTimeout}){
      auto bad_raw=raw_fresh;bad_raw[slot]=reason;
      assert(!f::only_preparation_source_age_expired(expired,bad_raw));
    }
  }
  std::cout<<"PREPARATION_RETRY_ONLY_FOR_FRESH_RECEIPT_AND_RAW_PROGRESS_WITH_SOURCE_AGE_EXPIRY PASS\n";
}

// 第三次上下文采样跨过500 ms源寿命时，必须保留target已发事实并抑制heartbeat、抛出部分发布。
static void source_expiry_after_target_suppresses_heartbeat(){
  a::Output output;output.publish=true;
  unsigned reads=0,targets=0,heartbeats=0;bool partial=false;
  auto context=[&]{
    auto c=prepared();const auto host=epoch+(++reads==3?510000:490000);
    c.feedback.source_clock_valid=f::source_sample_in_clock(epoch,epoch,host,.5);
    return c;
  };
  try{
    a::publish_attitude_resampled<Mode,Target>(context,output,epoch,
      [&](const Target&){++targets;},[&](const Mode&){++heartbeats;});
  }catch(const std::runtime_error& error){
    partial=std::string(error.what())==
      "HEARTBEAT_FINAL_CONTEXT_CHANGED_DDS_SOURCE_CLOCK_UNSYNCHRONIZED_OR_STALE";
  }
  assert(partial&&targets==1&&heartbeats==0&&reads==3);
  std::cout<<"SOURCE_049_TO_051_AFTER_TARGET_PRESERVES_FINAL_HEARTBEAT_REFUSAL PASS\n";
}

// 336 ms控制周期超过300 ms截止；目标前拒绝不触sink，目标后拒绝必须报告部分提交。
static void current_cycle_stall_is_refused_before_or_after_target(){
  for(bool after_target:{false,true}){
    f::CurrentControlCycle cycle;cycle.begin(10.);cycle.callbacks_done(10.01);
    double now=10.02;unsigned targets=0,heartbeats=0;bool partial=false;
    a::Output output;output.publish=true;
    auto context=[&]{auto c=prepared();c.control_fresh=cycle.current(now,.3);return c;};
    if(!after_target)now=10.336;
    a::Decision decision;
    try{
      decision=a::publish_attitude_resampled<Mode,Target>(context,output,epoch,
        [&](const Target&){++targets;now=10.336;},[&](const Mode&){++heartbeats;});
    }catch(const std::runtime_error& error){
      partial=std::string(error.what())=="HEARTBEAT_FINAL_CONTEXT_CHANGED_CONTROL_LOOP_STALL";
    }
    assert(heartbeats==0);
    if(after_target)assert(partial&&targets==1);
    else assert(!partial&&!decision.allowed&&decision.reason==a::BoundaryReason::ControlStall&&targets==0);
  }
  std::cout<<"CURRENT_0336_SECOND_CYCLE_STALL_NEVER_SENDS_HEARTBEAT PASS\n";
}
int main(){
  offline_epoch_and_precise_reasons();callback_observations_do_not_invent_measurements();
  host_jumps_are_separate_from_source_delay();raw_lifetime_and_regression_latches_are_preserved();
  preparation_expiry_classification_never_legalizes_stale_output();
  source_expiry_after_target_suppresses_heartbeat();current_cycle_stall_is_refused_before_or_after_target();
}
