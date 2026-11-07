#pragma once
#include "jikl_fc01_system/source_clock.hpp"
#include <array>
#include <cstddef>

namespace fc01 {
// Signed diagnostic values retain future timestamps. Unavailable timestamps
// remain unavailable; zero is never substituted for a missing measurement.
struct AgeMeasurement {
  double seconds{};
  bool available{};
};
inline AgeMeasurement source_age(std::uint64_t source,std::uint64_t host){
  if(!source||!host)return {};
  return {double((static_cast<long double>(host)-source)*1e-6L),true};
}
inline AgeMeasurement steady_age(double received,double now){
  if(!std::isfinite(received)||received<0||!std::isfinite(now)||now<0)return {};
  return {now-received,true};
}

enum class SourceTimeReason {
  Valid,MissingEnvelope,MissingSample,MissingHostClock,InvalidTimeout,
  EnvelopeTooFarFuture,EnvelopeExpired,SampleAfterEnvelope,
  SampleTooFarFuture,SampleExpired
};
inline const char* source_time_reason_name(SourceTimeReason reason){
  switch(reason){
    case SourceTimeReason::Valid:return "VALID";
    case SourceTimeReason::MissingEnvelope:return "MISSING_ENVELOPE";
    case SourceTimeReason::MissingSample:return "MISSING_SAMPLE";
    case SourceTimeReason::MissingHostClock:return "MISSING_HOST_CLOCK";
    case SourceTimeReason::InvalidTimeout:return "INVALID_TIMEOUT";
    case SourceTimeReason::EnvelopeTooFarFuture:return "ENVELOPE_TOO_FAR_FUTURE";
    case SourceTimeReason::EnvelopeExpired:return "ENVELOPE_EXPIRED";
    case SourceTimeReason::SampleAfterEnvelope:return "SAMPLE_AFTER_ENVELOPE";
    case SourceTimeReason::SampleTooFarFuture:return "SAMPLE_TOO_FAR_FUTURE";
    case SourceTimeReason::SampleExpired:return "SAMPLE_EXPIRED";
  }
  return "UNKNOWN";
}
// This explains the existing source_clock.hpp predicate without changing it.
inline SourceTimeReason source_time_reason(std::uint64_t envelope,
    std::uint64_t sample,bool sampled,std::uint64_t host,double timeout){
  if(!envelope)return SourceTimeReason::MissingEnvelope;
  if(sampled&&!sample)return SourceTimeReason::MissingSample;
  if(!host)return SourceTimeReason::MissingHostClock;
  if(!std::isfinite(timeout)||timeout<=0)return SourceTimeReason::InvalidTimeout;
  if(sampled&&sample>envelope)return SourceTimeReason::SampleAfterEnvelope;
  if(!source_stamp_in_clock(envelope,host,timeout))return envelope>host?
    SourceTimeReason::EnvelopeTooFarFuture:SourceTimeReason::EnvelopeExpired;
  if(sampled&&!source_stamp_in_clock(sample,host,timeout))return sample>host?
    SourceTimeReason::SampleTooFarFuture:SourceTimeReason::SampleExpired;
  return SourceTimeReason::Valid;
}

enum class ReceiptTimeReason {Valid,MissingReceipt,InvalidNow,ReceiptFromFuture,Expired,InvalidTimeout};
inline const char* receipt_time_reason_name(ReceiptTimeReason reason){
  switch(reason){
    case ReceiptTimeReason::Valid:return "VALID";
    case ReceiptTimeReason::MissingReceipt:return "MISSING_RECEIPT";
    case ReceiptTimeReason::InvalidNow:return "INVALID_MONOTONIC_NOW";
    case ReceiptTimeReason::ReceiptFromFuture:return "MONOTONIC_REGRESSION";
    case ReceiptTimeReason::Expired:return "RECEIPT_EXPIRED";
    case ReceiptTimeReason::InvalidTimeout:return "INVALID_TIMEOUT";
  }
  return "UNKNOWN";
}
inline ReceiptTimeReason receipt_time_reason(double received,double now,double timeout){
  if(!std::isfinite(received)||received<0)return ReceiptTimeReason::MissingReceipt;
  if(!std::isfinite(now)||now<0)return ReceiptTimeReason::InvalidNow;
  if(!std::isfinite(timeout)||timeout<=0)return ReceiptTimeReason::InvalidTimeout;
  if(now<received)return ReceiptTimeReason::ReceiptFromFuture;
  return now-received<timeout?ReceiptTimeReason::Valid:ReceiptTimeReason::Expired;
}

enum class SourceSlot:std::size_t {Status,Land,Local,Attitude,Flags,Battery,Count};
inline const char* source_slot_name(SourceSlot slot){
  switch(slot){
    case SourceSlot::Status:return "status";
    case SourceSlot::Land:return "land";
    case SourceSlot::Local:return "local";
    case SourceSlot::Attitude:return "attitude";
    case SourceSlot::Flags:return "flags";
    case SourceSlot::Battery:return "battery";
    case SourceSlot::Count:break;
  }
  return "unknown";
}
struct SourceTiming {
  AgeMeasurement envelope_age,sample_age,receipt_age,envelope_sample_gap;
  SourceTimeReason source_reason{SourceTimeReason::MissingEnvelope};
  ReceiptTimeReason receipt_reason{ReceiptTimeReason::MissingReceipt};
};
using SourceTimingSet=std::array<SourceTiming,static_cast<std::size_t>(SourceSlot::Count)>;
inline SourceTiming diagnose_source_timing(std::uint64_t envelope,
    std::uint64_t sample,bool sampled,std::uint64_t host,double now,
    double received,double timeout){
  SourceTiming result;
  result.envelope_age=source_age(envelope,host);
  if(sampled){
    result.sample_age=source_age(sample,host);
    result.envelope_sample_gap=source_age(sample,envelope);
  }
  result.receipt_age=steady_age(received,now);
  result.source_reason=source_time_reason(envelope,sample,sampled,host,timeout);
  result.receipt_reason=receipt_time_reason(received,now,timeout);
  return result;
}

// Classify an expiry for a ground preparation retry, never permission to
// publish. The caller must separately prove no arm/command/sink history,
// no clock/reset fault, healthy ground feedback and all authority checks.
inline bool only_preparation_source_age_expired(const SourceTimingSet& sources,
    const std::array<ReceiptTimeReason,2>& raw_progress){
  bool expired=false;
  for(std::size_t i=0;i<sources.size();++i){
    const auto& source=sources[i];
    if(source.receipt_reason!=ReceiptTimeReason::Valid)return false;
    const bool sampled=i==std::size_t(SourceSlot::Local)||i==std::size_t(SourceSlot::Attitude);
    const bool age_expired=source.source_reason==SourceTimeReason::EnvelopeExpired||
      source.source_reason==SourceTimeReason::SampleExpired;
    if(source.source_reason!=SourceTimeReason::Valid&&(!sampled||!age_expired))return false;
    expired=expired||(sampled&&age_expired);
  }
  return expired&&raw_progress[0]==ReceiptTimeReason::Valid&&
    raw_progress[1]==ReceiptTimeReason::Valid;
}

struct SourceReceiptTiming {
  std::uint64_t envelope_stamp{},sample_stamp{},host_stamp{};
  double steady_stamp{-1};
  AgeMeasurement envelope_host_age,sample_host_age,middleware_callback_age;
};
// Record this at callback entry. It describes this callback's payload, even
// when the existing safety receiver rejects that payload as reordered. RMW
// receive timestamps are optional nanoseconds; zero means unavailable. The
// receive-to-callback subtraction requires an RMW using the host wall epoch
// (verified for the deployed Fast DDS implementation). Other RMWs may use a
// different epoch. This signed difference alone is not proof of queue delay,
// and is diagnostic evidence only, never a TTL or a validity override.
inline SourceReceiptTiming observe_source_receipt(std::uint64_t envelope,
    std::uint64_t sample,bool sampled,std::uint64_t host,double received,
    std::int64_t rmw_received_ns=0){
  SourceReceiptTiming result;
  result.envelope_stamp=envelope;result.sample_stamp=sampled?sample:0;
  result.host_stamp=host;result.steady_stamp=received;
  result.envelope_host_age=source_age(envelope,host);
  if(sampled)result.sample_host_age=source_age(sample,host);
  if(host&&rmw_received_ns>0)result.middleware_callback_age={
    double((static_cast<long double>(host)*1000-rmw_received_ns)*1e-9L),true};
  return result;
}

enum class HostClockChange {Unavailable,Initial,Consistent,ForwardJump,BackwardJump,SteadyRegression};
inline const char* host_clock_change_name(HostClockChange change){
  switch(change){
    case HostClockChange::Unavailable:return "UNAVAILABLE";
    case HostClockChange::Initial:return "INITIAL";
    case HostClockChange::Consistent:return "CONSISTENT";
    case HostClockChange::ForwardJump:return "HOST_CLOCK_FORWARD_JUMP";
    case HostClockChange::BackwardJump:return "HOST_CLOCK_BACKWARD_JUMP";
    case HostClockChange::SteadyRegression:return "MONOTONIC_REGRESSION";
  }
  return "UNKNOWN";
}
struct HostClockObservation {
  HostClockChange change{HostClockChange::Unavailable};
  AgeMeasurement wall_delta,steady_delta,delta_error,anchor_error;
  bool host_regressed{},steady_regressed{};
};
// Wall and monotonic clocks are compared only by elapsed increments, never
// by subtracting their unrelated epochs. No estimated offset is applied to
// source timestamps or used to relax their validity. The reporting budget
// reuses the existing 100 ms skew budget. Sampling preemption can also cause
// an increment mismatch: this is evidence to investigate, not proof of a
// wall-clock adjustment. It cannot make any source valid or enable output.
class HostClockMonitor {
 public:
  HostClockObservation observe(std::uint64_t host,double now){
    HostClockObservation result;
    if(!host||!std::isfinite(now)||now<0)return result;
    if(!previous_host_){
      anchor_host_=previous_host_=host;anchor_steady_=previous_steady_=now;
      result.change=HostClockChange::Initial;return result;
    }
    result.wall_delta=source_age(previous_host_,host);
    result.steady_delta={now-previous_steady_,true};
    result.delta_error={result.wall_delta.seconds-result.steady_delta.seconds,true};
    result.anchor_error={source_age(anchor_host_,host).seconds-(now-anchor_steady_),true};
    result.host_regressed=host<previous_host_;
    result.steady_regressed=now<previous_steady_;
    constexpr double budget=double(source_clock_skew_budget_us)*1e-6;
    if(result.steady_regressed)result.change=HostClockChange::SteadyRegression;
    else if(result.host_regressed||result.delta_error.seconds< -budget)
      result.change=HostClockChange::BackwardJump;
    else if(result.delta_error.seconds>budget)result.change=HostClockChange::ForwardJump;
    else result.change=HostClockChange::Consistent;
    // Preserve the last monotonic observation on an impossible regression.
    if(!result.steady_regressed){previous_host_=host;previous_steady_=now;}
    return result;
  }
 private:
  std::uint64_t anchor_host_{},previous_host_{};
  double anchor_steady_{},previous_steady_{};
};
}
