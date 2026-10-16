#pragma once
#include <cstddef>
#include <string>

namespace fc01::attitude {
struct OwnershipCounts {
  std::size_t mode_publishers{},mode_subscribers{};
  std::size_t trajectory_publishers{},trajectory_subscribers{};
  std::size_t attitude_publishers{},attitude_subscribers{},other_publishers{};
};
inline bool owns_setpoints(const std::string& profile,const OwnershipCounts& c){
  if(c.other_publishers)return false;
  if(profile=="ground")return c.mode_publishers==0&&c.trajectory_publishers==0&&c.attitude_publishers==0;
  if(c.mode_publishers!=1||c.mode_subscribers!=1)return false;
  if(profile=="attitude_altitude")
    return c.attitude_publishers==1&&c.attitude_subscribers==1&&c.trajectory_publishers==0;
  if(profile=="flight"||profile=="sim")
    return c.trajectory_publishers==1&&c.trajectory_subscribers==1&&c.attitude_publishers==0;
  return false;
}
} // namespace fc01::attitude
