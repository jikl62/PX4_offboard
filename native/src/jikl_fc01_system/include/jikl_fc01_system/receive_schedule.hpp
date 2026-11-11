#pragma once
// 合作式 DDS 补读预算：保持原 spin_some 内部回调顺序，只决定能否再开始一轮。
// now/budget 单位为单调秒；回调计数/安全事件计数只能递增，倒退说明上下文无效。
// 它不能抢占某个阻塞回调；最终 CurrentControlCycle 和发布门仍负责回调后的控制截止检查。
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace fc01 {
// 停止原因区分空闲、安全事件、时钟/预算/计数错误和资源上限，便于实际调度诊断。
enum class ReceivePassStop {Running,Idle,Event,InvalidClock,InvalidBudget,InvalidCounter,Budget,PassLimit};
inline const char* receive_pass_stop_name(ReceivePassStop value){
  switch(value){
    case ReceivePassStop::Running:return "RUNNING";
    case ReceivePassStop::Idle:return "IDLE";
    case ReceivePassStop::Event:return "SAFETY_EVENT";
    case ReceivePassStop::InvalidClock:return "INVALID_MONOTONIC_CLOCK";
    case ReceivePassStop::InvalidBudget:return "INVALID_RECEIVE_BUDGET";
    case ReceivePassStop::InvalidCounter:return "COUNTER_REGRESSION";
    case ReceivePassStop::Budget:return "BUDGET_EXHAUSTED";
    case ReceivePassStop::PassLimit:return "PASS_LIMIT";
  }
  return "UNKNOWN";
}

// Each authorized pass is a complete, unlimited spin_some(), with the same
// per-pass callback ordering as the original loop. Its callbacks cannot be
// preempted. The cooperative budget only decides whether to START another
// pass; CurrentControlCycle and the publication boundary still enforce the
// control deadline after callbacks finish. The first pass is retained even
// if setup work has already used the receive budget.
//
// Callbacks increment the total callback count, and increment the event count
// when a safety-relevant observation must reach Runtime::step before another
// sample can overwrite it. An output-silent Safety observer may keep collecting
// bounded passes while the adapter retains every discrete payload for its
// recovery scan. This scheduler does not interpret measurements, alter
// timeouts, or grant control authority. A stopped schedule stays stopped until
// begin() starts the next control cycle.
class ReceivePassSchedule {
 public:
  // 每个控制周期最多十轮；不是每轮最多十个回调，第一轮保持原无限 spin_some 语义。
  static constexpr std::size_t max_passes=10;
  // 保存本周期预算起点和原计数；stop_on_events=false 只适用于保留全部离散证据的无输出安全观察。
  void begin(double now,double budget,std::uint64_t callbacks,std::uint64_t events,
      bool stop_on_events=true){
    started_=observed_=now;budget_=budget;callbacks_=callbacks;events_=events;passes_=0;
    stop_on_events_=stop_on_events;
    reason_=ReceivePassStop::Running;
    if(!std::isfinite(now)||now<0)reason_=ReceivePassStop::InvalidClock;
    else if(!std::isfinite(budget)||budget<=0)reason_=ReceivePassStop::InvalidBudget;
  }
  // 停止原因一旦设定保持到下一次 begin，后续正常时间不能使同周期重新开始补读。
  bool next_pass(double now,std::uint64_t callbacks,std::uint64_t events){
    if(reason_!=ReceivePassStop::Running)return false;
    if(!std::isfinite(now)||now<observed_)return stop(ReceivePassStop::InvalidClock);
    observed_=now;
    if(callbacks<callbacks_||events<events_)return stop(ReceivePassStop::InvalidCounter);
    // 新安全事件优先结束补读，控制步骤必须先消费它，不能追新掩盖。
    if(stop_on_events_&&events!=events_)return stop(ReceivePassStop::Event);
    // 第一轮保留；从第二轮起，没回调、预算耗尽或达上限均停止。
    if(passes_){
      if(callbacks==callbacks_)return stop(ReceivePassStop::Idle);
      if(now-started_>=budget_)return stop(ReceivePassStop::Budget);
      if(passes_>=max_passes)return stop(ReceivePassStop::PassLimit);
    }
    callbacks_=callbacks;events_=events;++passes_;return true;
  }
  std::size_t pass_count()const{return passes_;}
  ReceivePassStop reason()const{return reason_;}
  const char* stop_reason()const{return receive_pass_stop_name(reason_);}
  double elapsed_seconds()const{return observed_-started_;}
 private:
  bool stop(ReceivePassStop reason){reason_=reason;return false;}
  // started_/observed_ 是单调时间；callbacks_/events_ 只缓存前次允许轮的计数。
  double started_{},observed_{},budget_{};
  std::uint64_t callbacks_{},events_{};
  bool stop_on_events_{true};
  std::size_t passes_{};
  ReceivePassStop reason_{ReceivePassStop::InvalidClock};
};
} // namespace fc01
