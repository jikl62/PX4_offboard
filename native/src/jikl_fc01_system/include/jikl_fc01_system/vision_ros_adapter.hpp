#pragma once
// 视觉 ROS 字段适配：把生成消息转换为纯 C++ 值，方便离线测试和生产共用检查。
// 该层不伪造接收时间，不授予控制权；本机 Inbox 在真正回调处记录 monotonic received_at。
// 所有字段按有效位/有效标志复制，未声明的数据保留 NaN，避免默认 0 被误认为可信测量。
#include "jikl_fc01_system/vision_navigation.hpp"
#include <cstdint>

namespace fc01::vision {
// Pure mapping: accepts generated ROS messages or narrow host fixtures. Callback
// receive time belongs to Inbox, never to a timestamp supplied by a publisher.
template <class Stamp>
// ROS sec/nanosec 转无符号 ns；负秒或超范围纳秒返回 0，让后续时间验证拒绝。
std::uint64_t message_stamp_ns(const Stamp& stamp) {
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U)
    return 0;
  return std::uint64_t(stamp.sec) * 1000000000ULL + std::uint64_t(stamp.nanosec);
}
template <class Message>
// 实际状态与目标命令分开：保留身份/序号/reset 证据，版本或掩码错误时不复制有效测量。
State state_from_message(const Message& message) {
  State out;
  out.sequence = message.sequence;
  out.stamp_ns = message_stamp_ns(message.stamp);
  out.reset_counter = message.reset_counter;
  out.frame_id = message.frame_id;
  out.body_frame_id = message.body_frame_id;
  out.source_id = message.source_id;
  out.angular_velocity_source = message.angular_velocity_source;
  // V1 只定义低 7 个有效位；未知位不能被静默忽略成合法状态。
  if (message.schema_version != 1 || !out.stamp_ns || (message.valid_fields & ~std::uint16_t(127)))
    return out;
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
Command command_from_message(const Message& message) {
  Command out;
  out.sequence = message.sequence;
  out.stamp_ns = message_stamp_ns(message.stamp);
  out.reset_counter = message.reset_counter;
  out.source_id = message.source_id;
  out.frame_id = message.frame_id;
  // 模式只允许既有 Disabled/Hold/Cruise/Follow/Land；非法版本/时间/枚举保持默认无效。
  if (message.schema_version != 1 || !out.stamp_ns || message.mode > 4)
    return out;
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
