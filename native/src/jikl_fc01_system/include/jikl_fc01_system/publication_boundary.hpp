#pragma once
// 最终发布包装器：把 Core 输出在真正调用 ROS sink 前重新进行纯策略检查。
// 目标和心跳不是原子事务：目标已提交后，任何心跳拒绝都属于部分发布，必须显式抛出异常。
// Context 回调每次取最新反馈和时钟；PublicationAttempt 保留走到哪一步及本地提交事实。
#include "jikl_fc01_system/publication_guard.hpp"
#include "jikl_fc01_system/attitude_wire.hpp"
namespace fc01::attitude {
// Local sink evidence is separate from a task fault. A deadline refusal is
// not a publisher exception, and an emitted target is never an emitted pair.
// None=未尝试，InitialContext=初查，BeforeTarget/BeforeHeartbeat=各 sink 前，Complete=两者返回。
enum class PublicationStage {None,InitialContext,BeforeTarget,BeforeHeartbeat,Complete};
inline const char* name(PublicationStage stage){
  switch(stage){
    case PublicationStage::None:return "NOT_ATTEMPTED";
    case PublicationStage::InitialContext:return "INITIAL_CONTEXT";
    case PublicationStage::BeforeTarget:return "BEFORE_TARGET";
    case PublicationStage::BeforeHeartbeat:return "BEFORE_HEARTBEAT";
    case PublicationStage::Complete:return "COMPLETE";
  }
  return "UNKNOWN";
}
// decision 是最近一次判定；committed 只表示本地 sink 返回，sink_exception 单独标注异常。
struct PublicationAttempt {
  PublicationStage stage{PublicationStage::None};
  Decision decision{};
  bool target_committed{},heartbeat_committed{},sink_exception{};
};
// 解锁请求也必须经过最终上下文策略，不能因为 Core 已计划就无条件调用 sink。
template<class Sink>Decision publish_arm_at_boundary(const BoundaryContext& c,Sink&& sink){
  const auto decision=arm_publication_policy(c);
  if(decision.allowed)sink();
  return decision;
}
// 上锁要求原有新地面证据，重复事务不能依靠未关联错误 ACK 自动重发。
template<class Sink>Decision publish_disarm_at_boundary(const BoundaryContext& c,Sink&& sink){
  const auto decision=disarm_publication_policy(c);
  if(decision.allowed)sink();
  return decision;
}
// 释放 Offboard 仍只允许原策略证明的地面条件，不能当成飞行中切模式捷径。
template<class Sink>Decision publish_release_at_boundary(const BoundaryContext& c,Sink&& sink){
  const auto decision=release_publication_policy(c);
  if(decision.allowed)sink();
  return decision;
}
// PX4 降落后备命令只有其合同及当前定位证明完整时才允许。
template<class Sink>Decision publish_land_fallback_at_boundary(const BoundaryContext& c,Sink&& sink){
  const auto decision=land_fallback_publication_policy(c);
  if(decision.allowed)sink();
  return decision;
}
// Both sinks remain unreachable on any policy/encoding failure. A sink exception
// prevents heartbeat and propagates to the node's fail/exit handler.
// 静态上下文版本供确定性调用；编码失败时两个 sink 都不可达，目标 sink 异常时不发心跳。
template<class Mode,class Target,class TargetSink,class ModeSink>
Decision publish_attitude_at_boundary(const BoundaryContext& c,const Output& output,
    std::uint64_t stamp,TargetSink&& target_sink,ModeSink&& mode_sink){
  auto decision=attitude_publication_policy(c,output);if(!decision.allowed)return decision;
  auto packet=wire<Mode,Target>(output,stamp);
  if(!packet)return deny(BoundaryReason::InvalidOutput);
  if(!nonzero(output.thrust))packet->second.thrust_body.fill(0.f);
  target_sink(packet->second);mode_sink(packet->first);return allow();
}
// 生产使用的重采样版本：初查→编码→目标前复查→目标提交→心跳前复查→心跳提交。
template<class Mode,class Target,class Context,class TargetSink,class ModeSink>
Decision publish_attitude_resampled(Context&& context,const Output& output,std::uint64_t stamp,
    TargetSink&& target_sink,ModeSink&& mode_sink,PublicationAttempt* evidence=nullptr){
  PublicationAttempt local;auto& attempt=evidence?*evidence:local;attempt={};
  const auto check=[&](PublicationStage stage){
    attempt.stage=stage;attempt.decision=attitude_publication_policy(context(),output);
    return attempt.decision;
  };
  auto decision=check(PublicationStage::InitialContext);if(!decision.allowed)return decision;
  auto packet=wire<Mode,Target>(output,stamp);
  if(!packet){attempt.decision=deny(BoundaryReason::InvalidOutput);return attempt.decision;}
  if(!nonzero(output.thrust))packet->second.thrust_body.fill(0.f);
  // Encoding may consume the remaining mission-input lifetime. No sink has
  // run, so a refusal here remains recoverable by the native descent path.
  // 编码也消耗 TTL；尚未发任何消息时拒绝，仍可由原 native 下降流程处置。
  decision=check(PublicationStage::BeforeTarget);if(!decision.allowed)return decision;
  try{target_sink(packet->second);attempt.target_committed=true;}
  catch(...){attempt.sink_exception=true;throw;}
  // Once target committed, failure is partial publication, including TTL.
  // 此时目标已发；不能静默返回“未发”，否则上层会误判为完整未执行。
  decision=check(PublicationStage::BeforeHeartbeat);
  if(!decision.allowed)throw std::runtime_error(std::string("HEARTBEAT_FINAL_CONTEXT_CHANGED_")+name(decision.reason));
  try{mode_sink(packet->first);attempt.heartbeat_committed=true;}
  catch(...){attempt.sink_exception=true;throw;}
  attempt.stage=PublicationStage::Complete;return allow();
}
} // namespace fc01::attitude
