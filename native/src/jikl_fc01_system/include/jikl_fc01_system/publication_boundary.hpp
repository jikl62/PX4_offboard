#pragma once
#include "jikl_fc01_system/publication_guard.hpp"
#include "jikl_fc01_system/attitude_wire.hpp"
namespace fc01::attitude {
// Local sink evidence is separate from a task fault. A deadline refusal is
// not a publisher exception, and an emitted target is never an emitted pair.
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
struct PublicationAttempt {
  PublicationStage stage{PublicationStage::None};
  Decision decision{};
  bool target_committed{},heartbeat_committed{},sink_exception{};
};
template<class Sink>Decision publish_arm_at_boundary(const BoundaryContext& c,Sink&& sink){
  const auto decision=arm_publication_policy(c);
  if(decision.allowed)sink();
  return decision;
}
template<class Sink>Decision publish_disarm_at_boundary(const BoundaryContext& c,Sink&& sink){
  const auto decision=disarm_publication_policy(c);
  if(decision.allowed)sink();
  return decision;
}
template<class Sink>Decision publish_release_at_boundary(const BoundaryContext& c,Sink&& sink){
  const auto decision=release_publication_policy(c);
  if(decision.allowed)sink();
  return decision;
}
template<class Sink>Decision publish_land_fallback_at_boundary(const BoundaryContext& c,Sink&& sink){
  const auto decision=land_fallback_publication_policy(c);
  if(decision.allowed)sink();
  return decision;
}
// Both sinks remain unreachable on any policy/encoding failure. A sink exception
// prevents heartbeat and propagates to the node's fail/exit handler.
template<class Mode,class Target,class TargetSink,class ModeSink>
Decision publish_attitude_at_boundary(const BoundaryContext& c,const Output& output,
    std::uint64_t stamp,TargetSink&& target_sink,ModeSink&& mode_sink){
  auto decision=attitude_publication_policy(c,output);if(!decision.allowed)return decision;
  auto packet=wire<Mode,Target>(output,stamp);
  if(!packet)return deny(BoundaryReason::InvalidOutput);
  if(!nonzero(output.thrust))packet->second.thrust_body.fill(0.f);
  target_sink(packet->second);mode_sink(packet->first);return allow();
}
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
  decision=check(PublicationStage::BeforeTarget);if(!decision.allowed)return decision;
  try{target_sink(packet->second);attempt.target_committed=true;}
  catch(...){attempt.sink_exception=true;throw;}
  // Once target committed, failure is partial publication, including TTL.
  decision=check(PublicationStage::BeforeHeartbeat);
  if(!decision.allowed)throw std::runtime_error(std::string("HEARTBEAT_FINAL_CONTEXT_CHANGED_")+name(decision.reason));
  try{mode_sink(packet->first);attempt.heartbeat_committed=true;}
  catch(...){attempt.sink_exception=true;throw;}
  attempt.stage=PublicationStage::Complete;return allow();
}
} // namespace fc01::attitude
