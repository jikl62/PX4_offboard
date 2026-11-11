#pragma once
// PX4 源时钟与样本寿命：同时核对同步时钟域、消息外壳和原始测量进展。
// source/envelope/raw/host 均为微秒；received/now/timeout 为本机单调秒。两类寿命分别判断。
// Agent 同步域尚未成立、原始采样停滞或回退时均拒绝，不能因收到新 envelope 给旧测量续期。
#include <cmath>
#include <cstdint>
#include <limits>
namespace fc01 {
// 允许源时间最多领先本机 100 ms；这是既有同步偏差预算，不是额外 TTL。
constexpr std::uint64_t source_clock_skew_budget_us=100000;
// 命令后因果水位加同步容差并饱和到 uint64 上限，防止溢出绕回导致旧 ACK 归属新事务。
inline std::uint64_t source_causal_floor(std::uint64_t wire){
  return wire>std::numeric_limits<std::uint64_t>::max()-source_clock_skew_budget_us?
    std::numeric_limits<std::uint64_t>::max():wire+source_clock_skew_budget_us;
}
// PX4 v1.16 uXRCE serialization adds the synchronized agent time offset.
// The production adapter checks this domain before comparing any source
// timestamp with its ROS command stamp. Boot-clock or stale sources fail closed.
// 源 stamp 为 0/本机未取时/超时配置无效都失败；未来时间只在 100 ms 容差内接受。
inline bool source_stamp_in_clock(std::uint64_t source,std::uint64_t host,double timeout){
  if(!source||!host||!std::isfinite(timeout)||timeout<=0)return false;
  if(source>host)return source-host<=source_clock_skew_budget_us;
  return double(host-source)<=timeout*1e6;
}
// 原始测量不能晚于外壳，两者分别与 host 比较，外壳新鲜不能遮住 raw 过期。
inline bool source_sample_in_clock(std::uint64_t envelope,std::uint64_t raw,
    std::uint64_t host,double timeout){
  return raw&&raw<=envelope&&source_stamp_in_clock(envelope,host,timeout)&&
    source_stamp_in_clock(raw,host,timeout);
}
// A new envelope cannot refresh the lifetime of an unchanged raw estimate.
// Invalid current payloads cannot borrow the previous payload's freshness.
// 缓存 raw 最大源戳、最后推进的单调接收时刻和回退锁存；不是每收到消息就续期。
struct SourceSampleProgress {
  std::uint64_t value{};
  double received{-1};
  bool regressed{};
  // 先标记当前载荷有效性；一旦观测 raw 回退，后续健康包也不能在本对象内清锁存。
  void observe(std::uint64_t envelope,std::uint64_t sample,double now){
    valid_=sample&&sample<=envelope&&std::isfinite(now)&&now>=0;
    if(!valid_)return;
    if(value&&sample<value)regressed=true;
    if(regressed){valid_=false;return;}
    if(sample>value){value=sample;received=now;}
  }
  // 接收进展 TTL 与同步源 TTL 同时通过；合法重复包保持原 received，不延长寿命。
  bool fresh(std::uint64_t host,double now,double timeout) const{
    return valid_&&!regressed&&received>=0&&std::isfinite(now)&&now>=received&&
      std::isfinite(timeout)&&timeout>0&&now-received<timeout&&
      source_stamp_in_clock(value,host,timeout);
  }
 private:
  bool valid_{};
};
template<class Message,class Envelope>
// 先让外壳跟踪器接受；接受后保留真实载荷并独立记录 raw 状态，供下游解释当前无效样本。
bool receive_sampled_source(const Message& message,Message& destination,
    Envelope& envelope,SourceSampleProgress& raw,double now){
  if(!envelope.accept(message.timestamp,now))return false;
  destination=message;raw.observe(message.timestamp,message.timestamp_sample,now);
  return true;
}
template<class Command,class Clock,class Sink>
// 在真正 sink 前由调用者时钟重新填命令 stamp，避免排队或编码时长让时间戳过旧。
void publish_command_with_stamp(Command& command,Clock&& clock,Sink&& sink){
  command.timestamp=clock();sink(command);
}
}
