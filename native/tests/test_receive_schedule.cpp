// 有界完整补读调度回归：每轮spin完整执行，额外轮次在无工作、事件、预算或轮数门停止。
// 预算5 ms是协作检查，单个回调不可抢占；当前300 ms飞控截止独立包含整个回调时间。
#include "jikl_fc01_system/receive_schedule.hpp"
#include "jikl_fc01_system/authority_telemetry.hpp"
#include "jikl_fc01_system/control_diagnostics.hpp"
#include <cassert>
#include <iostream>
#include <limits>

namespace f=fc01;
using Stop=f::ReceivePassStop;

// 首次完整轮后无新回调即停，本轮停后新工作留下一周期；永远有工作也最多十轮。
static void idle_and_stable_buffered_work(){
  f::ReceivePassSchedule schedule;
  assert(!schedule.next_pass(1.,0,0));
  schedule.begin(1.,.005,0,0);
  assert(schedule.next_pass(1.,0,0));
  assert(!schedule.next_pass(1.0001,0,0));
  assert(schedule.pass_count()==1&&schedule.reason()==Stop::Idle);
  // Work arriving after the stop is handled by the next control cycle.
  assert(!schedule.next_pass(1.0002,1,0));
  schedule.begin(2.,.005,1,0);
  double now=2.;std::uint64_t callbacks=1;unsigned consumed=0;
  while(schedule.next_pass(now,callbacks,0)){
    if(consumed<3){++consumed;++callbacks;}
    now+=.0001;
  }
  assert(consumed==3&&schedule.pass_count()==4&&schedule.reason()==Stop::Idle);
  // Even permanently ready work cannot start more than ten complete passes.
  schedule.begin(3.,.005,callbacks,0);now=3.;consumed=0;
  while(schedule.next_pass(now,callbacks,0)){++consumed;++callbacks;now+=.0001;}
  assert(consumed==10&&schedule.pass_count()==10&&schedule.reason()==Stop::PassLimit);
}

// 即使setup已超预算仍保留原首次完整轮；慢回调真实耗时不能从控制截止漏掉。
static void cooperative_budget_keeps_full_pass_and_control_guard(){
  f::ReceivePassSchedule schedule;f::CurrentControlCycle cycle;
  // The existing full first pass remains available after setup consumes 5ms.
  schedule.begin(1.,.005,0,0);
  assert(schedule.next_pass(1.006,0,0));
  assert(!schedule.next_pass(1.0061,1,0)&&schedule.reason()==Stop::Budget);
  schedule.begin(2.,.005,0,0);
  assert(schedule.next_pass(2.,0,0));
  // A non-preemptible callback may exceed 5ms; do not claim a hard deadline.
  assert(!schedule.next_pass(2.006,1,0)&&schedule.reason()==Stop::Budget);
  cycle.begin(3.);schedule.begin(3.,.005,0,0);
  assert(schedule.next_pass(3.,0,0));
  cycle.callbacks_done(3.336);
  assert(!schedule.next_pass(3.336,1,0)&&schedule.reason()==Stop::Budget);
  // The unchanged .3s publication guard includes the whole slow callback.
  assert(!cycle.current(3.337,.3));
}

// NaN/Inf/负时间、坏预算、时间/计数回退或新安全事件均在本周期粘性停止。
static void invalid_clocks_and_generations_stop_until_new_cycle(){
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  f::ReceivePassSchedule schedule;
  for(double invalid:{nan,inf,-1.}){
    schedule.begin(invalid,.005,0,0);
    assert(!schedule.next_pass(1.,0,0)&&schedule.reason()==Stop::InvalidClock);
    schedule.begin(1.,.005,0,0);
    assert(schedule.next_pass(1.,0,0));
    assert(!schedule.next_pass(invalid,1,0)&&schedule.reason()==Stop::InvalidClock);
  }
  for(double invalid:{nan,inf,-1.,0.}){
    schedule.begin(1.,invalid,0,0);
    assert(!schedule.next_pass(1.,0,0)&&schedule.reason()==Stop::InvalidBudget);
  }
  schedule.begin(1.,.005,10,4);
  assert(schedule.next_pass(1.,10,4));
  assert(!schedule.next_pass(.999,11,4)&&schedule.reason()==Stop::InvalidClock);
  assert(!schedule.next_pass(1.001,12,4));
  schedule.begin(1.,.005,10,4);
  assert(schedule.next_pass(1.,10,4));
  assert(!schedule.next_pass(1.001,9,4)&&schedule.reason()==Stop::InvalidCounter);
  schedule.begin(1.,.005,10,4);
  assert(schedule.next_pass(1.,10,4));
  assert(!schedule.next_pass(1.001,11,3)&&schedule.reason()==Stop::InvalidCounter);
  schedule.begin(1.,.005,10,4);
  assert(schedule.next_pass(1.,10,4));
  assert(!schedule.next_pass(1.001,11,5)&&schedule.reason()==Stop::Event);
  assert(!schedule.next_pass(1.002,12,5));
}

struct Manual {
  static constexpr std::uint8_t SOURCE_RC=1;
  std::uint64_t timestamp{1000000},timestamp_sample{1000000};
  bool valid{true},sticks_moving{};std::uint8_t data_source{SOURCE_RC};
  float roll{},pitch{},yaw{},throttle{-1};
};
struct Switches {
  std::uint64_t timestamp{1000000},timestamp_sample{1000000};
  std::uint8_t mode_slot{1},arm_switch{3},return_switch{3},loiter_switch{3},offboard_switch{3},kill_switch{3};
  std::uint32_t switch_changes{};
};
// Kill/Arm/不安全摇杆脉冲先触事件，必须先交step看，再由下一周期读取OFF，不能正常尾帧覆盖。
static void rc_pulses_reach_control_before_later_valid_samples(){
  for(int pulse=0;pulse<3;++pulse){
    f::AuthorityTelemetry rc;Manual manual;Switches switches;
    auto facts=[&](double now){return rc.facts(now,.5,true,false,1,1);};
    rc.manual(manual,0);rc.switches(switches,0,.5);
    manual.timestamp+=1000;manual.timestamp_sample+=1000;
    switches.timestamp+=1000;switches.timestamp_sample+=1000;
    rc.manual(manual,.001);rc.switches(switches,.001,.5);
    assert(facts(.001).rc_axes_safe&&!facts(.001).kill);
    f::ReceivePassSchedule schedule;std::uint64_t callbacks=0,events=0;
    double now=.002;unsigned consumed=0;
    auto observe=[&](bool asserted){
      const auto before=facts(now);
      if(pulse==2){
        manual.timestamp+=1000;manual.timestamp_sample+=1000;
        manual.sticks_moving=asserted;manual.roll=asserted?.3f:0.f;
        rc.manual(manual,now);
      }else{
        switches.timestamp+=1000;switches.timestamp_sample+=1000;
        if(pulse==0)switches.kill_switch=asserted?1:3;
        else switches.arm_switch=asserted?1:3;
        ++switches.switch_changes;rc.switches(switches,now,.5);
      }
      ++callbacks;const auto after=facts(now);
      // The real observer's event facts stop an additional receive pass.
      // They do not change the controller's policy or clear any fault.
      if(after.switch_generation!=before.switch_generation||after.kill||
         after.sticks_moving||!after.rc_axes_safe)++events;
    };
    schedule.begin(now,.005,callbacks,events);
    while(schedule.next_pass(now,callbacks,events)){
      assert(consumed<2);observe(consumed==0);++consumed;now+=.0001;
    }
    assert(consumed==1&&schedule.reason()==Stop::Event);
    const auto at_control=facts(now);
    assert(pulse==0?at_control.kill:pulse==1?at_control.arm_switch==1:
      at_control.sticks_moving&&!at_control.rc_axes_safe);
    // Only after Runtime::step gets that pulse may the next cycle read OFF.
    schedule.begin(now,.005,callbacks,events);
    assert(schedule.next_pass(now,callbacks,events));observe(false);
    const auto released=facts(now);
    assert(!released.kill&&released.arm_switch==3&&!released.sticks_moving&&released.rc_axes_safe);
  }
}

// 补读只改变回调消耗，不修改raw真实年龄；外壳新仍不能续原始测量寿命或清回退锁。
static void extra_receipt_passes_do_not_retime_old_samples(){
  struct Estimate{std::uint64_t timestamp{},timestamp_sample{};};
  f::Stamp envelope;f::SourceSampleProgress raw;Estimate held;
  Estimate message{1000000,1000000};
  assert(f::receive_sampled_source(message,held,envelope,raw,0.));
  f::ReceivePassSchedule schedule;double now=.6;std::uint64_t callbacks=0;
  schedule.begin(now,.005,callbacks,0);
  for(unsigned i=0;i<3;++i){
    assert(schedule.next_pass(now,callbacks,0));
    message.timestamp+=10000; // new envelope, frozen original measurement
    assert(f::receive_sampled_source(message,held,envelope,raw,now));
    ++callbacks;now+=.0001;
  }
  assert(envelope.fresh(now,.5));
  assert(!raw.fresh(1600000,now,.5));
  assert(!f::source_sample_in_clock(held.timestamp,held.timestamp_sample,1600000,.5));
  assert(raw.value==1000000&&raw.received==0.);
  // A source-time regression remains latched despite later valid callbacks.
  message.timestamp+=10000;message.timestamp_sample=999000;
  assert(f::receive_sampled_source(message,held,envelope,raw,now)&&raw.regressed);
  message.timestamp+=10000;message.timestamp_sample=message.timestamp;
  assert(f::receive_sampled_source(message,held,envelope,raw,now+.0001));
  assert(raw.regressed&&!raw.fresh(message.timestamp,now+.0001,.5));
}

int main(){
  idle_and_stable_buffered_work();
  cooperative_budget_keeps_full_pass_and_control_guard();
  invalid_clocks_and_generations_stop_until_new_cycle();
  rc_pulses_reach_control_before_later_valid_samples();
  extra_receipt_passes_do_not_retime_old_samples();
  std::cout<<"bounded complete receive passes, RC event visibility, unchanged source lifetime: PASS\n";
}
