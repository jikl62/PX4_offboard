#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace fc01 {
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
  static constexpr std::size_t max_passes=10;
  void begin(double now,double budget,std::uint64_t callbacks,std::uint64_t events,
      bool stop_on_events=true){
    started_=observed_=now;budget_=budget;callbacks_=callbacks;events_=events;passes_=0;
    stop_on_events_=stop_on_events;
    reason_=ReceivePassStop::Running;
    if(!std::isfinite(now)||now<0)reason_=ReceivePassStop::InvalidClock;
    else if(!std::isfinite(budget)||budget<=0)reason_=ReceivePassStop::InvalidBudget;
  }
  bool next_pass(double now,std::uint64_t callbacks,std::uint64_t events){
    if(reason_!=ReceivePassStop::Running)return false;
    if(!std::isfinite(now)||now<observed_)return stop(ReceivePassStop::InvalidClock);
    observed_=now;
    if(callbacks<callbacks_||events<events_)return stop(ReceivePassStop::InvalidCounter);
    if(stop_on_events_&&events!=events_)return stop(ReceivePassStop::Event);
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
  double started_{},observed_{},budget_{};
  std::uint64_t callbacks_{},events_{};
  bool stop_on_events_{true};
  std::size_t passes_{};
  ReceivePassStop reason_{ReceivePassStop::InvalidClock};
};
} // namespace fc01
