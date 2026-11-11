#pragma once
// 设定值来源独占检查：按当前 profile 核对本机与 PX4 的发布/订阅端数量。
// 图端点计数是控制权限的必要证据，不能替代数据新鲜度、执行状态或人工授权。
#include <cstddef>
#include <string>

namespace fc01::attitude {
// mode=心跳，trajectory=轨迹，attitude=姿态推力；other_publishers 为其它会竞争控制的来源。
struct OwnershipCounts {
  std::size_t mode_publishers{}, mode_subscribers{};
  std::size_t trajectory_publishers{}, trajectory_subscribers{};
  std::size_t attitude_publishers{}, attitude_subscribers{}, other_publishers{};
};
// 存在竞争者立即拒绝；地面 profile 要求零发布者，动力 profile 只允许所选的一类设定值。
inline bool owns_setpoints(const std::string& profile, const OwnershipCounts& c) {
  if (c.other_publishers)
    return false;
  if (profile == "ground")
    return c.mode_publishers == 0 && c.trajectory_publishers == 0 && c.attitude_publishers == 0;
  // 生产路径要求一个本机心跳发布者和一个预期订阅者；多源和缺源同样不能通过。
  if (c.mode_publishers != 1 || c.mode_subscribers != 1)
    return false;
  if (profile == "attitude_altitude")
    return c.attitude_publishers == 1 && c.attitude_subscribers == 1 &&
           c.trajectory_publishers == 0;
  if (profile == "flight" || profile == "sim")
    return c.trajectory_publishers == 1 && c.trajectory_subscribers == 1 &&
           c.attitude_publishers == 0;
  return false;
}
} // namespace fc01::attitude
