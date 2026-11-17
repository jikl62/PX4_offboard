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
// 纯字段适配器：节点用固定版本px4_msgs实例化模板；上游不许输出/字段无效时不生成消息包。
// Mode/Target是编译期类型参数，因此同一函数可用实际ROS消息或测试假消息，算法本身不变。
template <class Mode, class Target>
std::optional<std::pair<Mode, Target>> wire(const Output& output, std::uint64_t stamp) {
  // 返回optional<pair<Mode,Target>>：空表示拒绝；有值时first为心跳、second为姿态目标。
  // pair中存放两份消息值，不包含发布动作；实际调用顺序由publication_boundary确定，仍是先target后mode。
  // 必须先获上游 publish 许可且四元数有效；任一推力分量非法时整包不生成。
  if (!output.publish || !normalized(output.q))
    return {};
  for (double value : output.thrust)
    if (!std::isfinite(value) || value < -1 || value > 1)
      return {};
  // 同一 stamp（µs）用于这一对本地消息；两次 publish 本身仍不是原子操作或飞控接收证明。
  Mode mode{};
  // {}初始化未用字段，随后只置attitude=true，避免无意同时声明位置/速度/角速度控制。
  mode.timestamp = stamp;
  mode.attitude = true;
  Target target{};
  target.timestamp = stamp;
  target.yaw_sp_move_rate = 0;
  for (std::size_t i = 0; i < 4; ++i)
    // 内部double转PX4消息float；这是实际线格式精度转换，不能把上游控制计算也统一改成float。
    target.q_d[i] = float(output.q[i]);
  for (std::size_t i = 0; i < 3; ++i)
    target.thrust_body[i] = float(output.thrust[i]);
  // 不通过该通道隐式重置 PX4 积分器，也不打开固定翼前轮控制。
  target.reset_integral = false;
  target.fw_control_yaw_wheel = false;
  return std::make_pair(mode, target);
}
} // namespace fc01::attitude
