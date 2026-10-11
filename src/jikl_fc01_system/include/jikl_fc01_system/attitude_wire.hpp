#pragma once
#include "jikl_fc01_system/attitude_altitude_core.hpp"
#include "jikl_fc01_system/setpoint_ownership.hpp"
#include <optional>
#include <utility>
namespace fc01::attitude {
struct PublisherPlan {bool mode{},trajectory{},attitude{};};
inline PublisherPlan publisher_plan(const std::string& profile){
  if(profile=="ground")return {};
  if(profile=="attitude_altitude")return {true,false,true};
  if(profile=="flight"||profile=="sim")return {true,true,false};
  throw std::invalid_argument("UNKNOWN_CONTROL_PROFILE");
}
// Pure field adapter; instantiated with the actual pinned px4_msgs in the node.
// No packet exists when health-gated output is false or any output is invalid.
template<class Mode,class Target>
std::optional<std::pair<Mode,Target>> wire(const Output& output,std::uint64_t stamp){
  if(!output.publish||!normalized(output.q))return {};
  for(double value:output.thrust)if(!std::isfinite(value)||value<-1||value>1)return {};
  Mode mode{};mode.timestamp=stamp;mode.attitude=true;
  Target target{};target.timestamp=stamp;target.yaw_sp_move_rate=0;
  for(std::size_t i=0;i<4;++i)target.q_d[i]=float(output.q[i]);
  for(std::size_t i=0;i<3;++i)target.thrust_body[i]=float(output.thrust[i]);
  target.reset_integral=false;target.fw_control_yaw_wheel=false;
  return std::make_pair(mode,target);
}
} // namespace fc01::attitude
