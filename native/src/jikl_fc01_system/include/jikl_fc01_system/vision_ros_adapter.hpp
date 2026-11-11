#pragma once
// 视觉 ROS 字段适配：把生成消息转换为纯 C++ 值，方便离线测试和生产共用检查。
// 该层不伪造接收时间，不授予控制权；本机 Inbox 在真正回调处记录 monotonic received_at。
// 所有字段按有效位/有效标志复制，未声明的数据保留 NaN，避免默认 0 被误认为可信测量。
#include "jikl_fc01_system/vision_navigation.hpp"
#include <cstdint>

namespace fc01::vision {
// 模板只要求输入对象具备用到的同名字段，因此真实ROS消息和离线小夹具可以复用同一转换。
// 字段映射不是坐标变换：本层假定上游已经提供内部NED/FRD，绝不把任意Odometry直接改名后使用。
// 回调接收时间只由本机Inbox填，不能相信发布者传来的“我刚被接收”时间。
// 仅轴约定的算例：世界ENU的[东1,北2,上3]对应NED[2,1,-3]；机体FLU[1,2,3]对应FRD[1,-2,-3]。
// 算例不含世界对齐、安装外参或杆臂；真实Odometry还须按父/子坐标语义转换，不能在这里交换数组冒充。
template <class Stamp>
// ROS sec/nanosec 转无符号 ns；负秒或超范围纳秒返回 0，让后续时间验证拒绝。
std::uint64_t message_stamp_ns(const Stamp& stamp) {
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U)
    return 0;
  // 先提升到64位再乘十亿，避免先在较窄整数中计算而溢出；0也作为后续校验的无效时间。
  // 这里只转换单位，不证明两端ROS时钟同步；时钟年龄/未来容差还要在核心检查。
  return std::uint64_t(stamp.sec) * 1000000000ULL + std::uint64_t(stamp.nanosec);
}
template <class Message>
// 实际状态与目标命令分开：保留身份/序号/reset 证据，版本或掩码错误时不复制有效测量。
// 返回State按值持有数值和字符串；文本复制可能抛分配异常，交给调用者处理。
// 例如位置[0,0,-1.5]表示相对视觉原点上方1.5m，不等于本轮PX4离地高度，也不是要求起飞1.5m。
State state_from_message(const Message& message) {
  // State默认的数值是NaN、有效位为0；提前返回的是可撤销旧资格的无效载荷，并非“没有回调”。
  State out;
  out.sequence = message.sequence;
  out.stamp_ns = message_stamp_ns(message.stamp);
  out.reset_counter = message.reset_counter;
  // DDS 已经反序列化；这里必须在首次复制到纯核心对象之前限制全部身份。
  // 用本机无效标记继续交给 Inbox 撤销旧缓存，不返回“无新消息”而保留旧计划。
  if (!identity_within_budget(message.frame_id) || !identity_within_budget(message.body_frame_id) ||
      !identity_within_budget(message.source_id) ||
      !identity_within_budget(message.angular_velocity_source)) {
    out.identity_resource_invalid = true;
    return out;
  }
  out.frame_id = message.frame_id;
  out.body_frame_id = message.body_frame_id;
  out.source_id = message.source_id;
  out.angular_velocity_source = message.angular_velocity_source;
  // V1 只定义低 7 个有效位；未知位不能被静默忽略成合法状态。
  if (message.schema_version != 1 || !out.stamp_ns || (message.valid_fields & ~std::uint16_t(127)))
    return out;
  // valid_fields按位声明测量：例如1|2|4|64=71，表示位置、速度、姿态和质量有效。
  // 数组恰好全0不构成有效性证据；未声明的数组继续保留NaN。
  out.initialized = message.initialized;
  out.tracking = message.tracking;
  out.calibration_valid = message.calibration_valid;
  out.valid_fields = message.valid_fields;
  if (out.valid_fields & 1)
    out.position = message.position;
  if (out.valid_fields & 2)
    out.velocity = message.velocity;
  if (out.valid_fields & 4)
    out.attitude = message.attitude;
  if (out.valid_fields & 8)
    out.angular_velocity = message.angular_velocity;
  if (out.valid_fields & 16)
    out.pose_covariance = message.pose_covariance;
  if (out.valid_fields & 32)
    out.twist_covariance = message.twist_covariance;
  if (out.valid_fields & 64)
    out.quality = message.quality;
  return out;
}
template <class Message>
// NavigationCommand 是既有模式/位置/杆量目标，不能与新 VelocityCommand 的 FRD 目标速度互换。
// 返回Command拥有所复制的字符串；不持有输入消息引用，分配失败向调用者传播。
Command command_from_message(const Message& message) {
  // 这是旧模式/位置/归一化杆量合同；与body_frd速度目标是两种类型，不在这里猜测或换算。
  Command out;
  out.sequence = message.sequence;
  out.stamp_ns = message_stamp_ns(message.stamp);
  out.reset_counter = message.reset_counter;
  if (!identity_within_budget(message.source_id) || !identity_within_budget(message.frame_id)) {
    out.identity_resource_invalid = true;
    return out;
  }
  out.source_id = message.source_id;
  out.frame_id = message.frame_id;
  // 模式只允许既有 Disabled/Hold/Cruise/Follow/Land；非法版本/时间/枚举保持默认无效。
  if (message.schema_version != 1 || !out.stamp_ns || message.mode > 4)
    return out;
  // 先把版本、时间和枚举范围挡住，才static_cast到内部枚举；强制类型转换本身不会自动验范围。
  out.mode = static_cast<Mode>(message.mode);
  out.valid = message.valid;
  out.position_valid = message.position_valid;
  out.yaw_valid = message.yaw_valid;
  out.sticks_valid = message.sticks_valid;
  if (out.position_valid)
    out.position = message.position;
  if (out.yaw_valid)
    out.yaw = message.yaw;
  if (out.sticks_valid)
    out.sticks = message.sticks;
  return out;
}
} // namespace fc01::vision
