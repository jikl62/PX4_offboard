#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace fc01 {
// The publication budget begins before executor callbacks. A check must use a
// newly sampled monotonic time AFTER its snapshot, so the snapshot's own work
// cannot be omitted. Failure remains closed until the next cycle begins.
class CurrentControlCycle {
 public:
  void begin(double now){
    started_=observed_=now;callbacks_complete_=false;
    valid_=std::isfinite(now)&&now>=0;
  }
  void callbacks_done(double now){
    if(!valid_||callbacks_complete_||!std::isfinite(now)||now<started_){
      valid_=false;return;
    }
    observed_=now;callbacks_complete_=true;
  }
  bool current(double now,double stall_limit){
    if(!valid_||!callbacks_complete_||!std::isfinite(now)||
       !std::isfinite(stall_limit)||stall_limit<=0||now<observed_||
       now-started_>stall_limit){
      valid_=false;return false;
    }
    observed_=now;return true;
  }
 private:
  bool valid_{},callbacks_complete_{};
  double started_{},observed_{};
};
// Observation of the last completed cycle; no timing metric grants authority.
struct ControlTiming {
  bool valid{},period_valid{};
  double callback_seconds{},control_seconds{},period_seconds{},max_period_seconds{};
  std::uint64_t stall_count{};
  void observe(double start,double callbacks_done,double done,double stall_limit){
    valid=std::isfinite(start)&&std::isfinite(callbacks_done)&&std::isfinite(done)&&
      callbacks_done>=start&&done>=callbacks_done&&(previous_start_<0||start>=previous_start_);
    if(!valid){period_valid=false;return;}
    callback_seconds=callbacks_done-start;control_seconds=done-callbacks_done;
    period_valid=previous_start_>=0;
    if(period_valid){
      period_seconds=start-previous_start_;max_period_seconds=std::max(max_period_seconds,period_seconds);
      if(period_seconds>stall_limit)++stall_count;
    }
    previous_start_=start;
  }
 private:
  double previous_start_{-1};
};
} // namespace fc01
