#pragma once
#include "jikl_fc01_system/publication_guard.hpp"
#include "jikl_fc01_system/attitude_wire.hpp"
namespace fc01::attitude {
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
} // namespace fc01::attitude
