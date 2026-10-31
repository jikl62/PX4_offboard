#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
namespace fc01 {
constexpr std::uint64_t source_clock_skew_budget_us=100000;
inline std::uint64_t source_causal_floor(std::uint64_t wire){
  return wire>std::numeric_limits<std::uint64_t>::max()-source_clock_skew_budget_us?
    std::numeric_limits<std::uint64_t>::max():wire+source_clock_skew_budget_us;
}
// PX4 v1.16 uXRCE serialization adds the synchronized agent time offset.
// The production adapter checks this domain before comparing any source
// timestamp with its ROS command stamp. Boot-clock or stale sources fail closed.
inline bool source_stamp_in_clock(std::uint64_t source,std::uint64_t host,double timeout){
  if(!source||!host||!std::isfinite(timeout)||timeout<=0)return false;
  if(source>host)return source-host<=source_clock_skew_budget_us;
  return double(host-source)<=timeout*1e6;
}
inline bool source_sample_in_clock(std::uint64_t envelope,std::uint64_t raw,
    std::uint64_t host,double timeout){
  return raw&&raw<=envelope&&source_stamp_in_clock(envelope,host,timeout)&&
    source_stamp_in_clock(raw,host,timeout);
}
// A new envelope cannot refresh the lifetime of an unchanged raw estimate.
// Invalid current payloads cannot borrow the previous payload's freshness.
struct SourceSampleProgress {
  std::uint64_t value{};
  double received{-1};
  bool regressed{};
  void observe(std::uint64_t envelope,std::uint64_t sample,double now){
    valid_=sample&&sample<=envelope&&std::isfinite(now)&&now>=0;
    if(!valid_)return;
    if(value&&sample<value)regressed=true;
    if(regressed){valid_=false;return;}
    if(sample>value){value=sample;received=now;}
  }
  bool fresh(std::uint64_t host,double now,double timeout) const{
    return valid_&&!regressed&&received>=0&&std::isfinite(now)&&now>=received&&
      std::isfinite(timeout)&&timeout>0&&now-received<timeout&&
      source_stamp_in_clock(value,host,timeout);
  }
 private:
  bool valid_{};
};
template<class Message,class Envelope>
bool receive_sampled_source(const Message& message,Message& destination,
    Envelope& envelope,SourceSampleProgress& raw,double now){
  if(!envelope.accept(message.timestamp,now))return false;
  destination=message;raw.observe(message.timestamp,message.timestamp_sample,now);
  return true;
}
template<class Command,class Clock,class Sink>
void publish_command_with_stamp(Command& command,Clock&& clock,Sink&& sink){
  command.timestamp=clock();sink(command);
}
}
