#pragma once
// PX4 姿态线格式适配：只生成消息字段，不负责决定能否获得 Offboard 权限。
// PublisherPlan 明确每个 profile 允许创建哪类发布者，避免姿态与轨迹同时争用同一飞控入口。
// 目标四元数 FRD→NED；thrust_body 为机体 FRD 归一化推力，旋翼升力通常是负 z。
#include "jikl_fc01_system/attitude_altitude_core.hpp"
#include "jikl_fc01_system/setpoint_ownership.hpp"
#include <optional>
#include <utility>
namespace fc01::attitude {
// mode=Offboard 心跳；trajectory=轨迹设定值；attitude=姿态/机体推力设定值。
struct PublisherPlan {
  bool mode{}, trajectory{}, attitude{};
};
// 地面 profile 无任何设定值发布者；未知 profile 抛错，不退回有动力的默认模式。
inline PublisherPlan publisher_plan(const std::string& profile) {
  if (profile == "ground")
    return {};
  if (profile == "attitude_altitude")
    return {true, false, true};
  if (profile == "flight" || profile == "sim")
    return {true, true, false};
  throw std::invalid_argument("UNKNOWN_CONTROL_PROFILE");
}
// Pure field adapter; instantiated with the actual pinned px4_msgs in the node.
// No packet exists when health-gated output is false or any output is invalid.
template <class Mode, class Target>
std::optional<std::pair<Mode, Target>> wire(const Output& output, std::uint64_t stamp) {
  // 必须先获上游 publish 许可且四元数有效；任一推力分量非法时整包不生成。
  if (!output.publish || !normalized(output.q))
    return {};
  for (double value : output.thrust)
    if (!std::isfinite(value) || value < -1 || value > 1)
      return {};
  // 同一 stamp（µs）用于这一对本地消息；两次 publish 本身仍不是原子操作或飞控接收证明。
  Mode mode{};
  mode.timestamp = stamp;
  mode.attitude = true;
  Target target{};
  target.timestamp = stamp;
  target.yaw_sp_move_rate = 0;
  for (std::size_t i = 0; i < 4; ++i)
    target.q_d[i] = float(output.q[i]);
  for (std::size_t i = 0; i < 3; ++i)
    target.thrust_body[i] = float(output.thrust[i]);
  // 不通过该通道隐式重置 PX4 积分器，也不打开固定翼前轮控制。
  target.reset_integral = false;
  target.fw_control_yaw_wheel = false;
  return std::make_pair(mode, target);
}
} // namespace fc01::attitude
