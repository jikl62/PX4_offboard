// 纯主机接口合同：实际状态/目标区分、消息失效到Inbox撤销、双时钟边界。
// 不使用ROS、不发布PX4消息；enabled=true仅让本进程内的合成Controller计算候选。
#include "jikl_fc01_system/vision_ros_adapter.hpp"
#include <iostream>
#include <stdexcept>

namespace v = fc01::vision;
static unsigned checks{};
static void need(bool value, const char* reason) {
  ++checks;
  if (!value)
    throw std::runtime_error(reason);
}

struct Stamp {
  std::int32_t sec{100};
  std::uint32_t nanosec{};
};
// 夹具只提供模板所需字段，数组故意有非零值，便于发现缺失有效位却复制测量的问题。
struct StateWire {
  std::uint16_t schema_version{1},
      valid_fields{v::Position | v::Velocity | v::Attitude | v::Quality};
  std::uint64_t sequence{1};
  Stamp stamp;
  std::uint32_t reset_counter{7};
  std::string frame_id{"fc01_vision_ned"}, body_frame_id{"base_link_frd"}, source_id{"fixture"},
      angular_velocity_source{"fixture-gyro"};
  bool initialized{true}, tracking{true}, calibration_valid{true};
  v::Vec3 position{1, 2, -1.5}, velocity{.1, .2, -.1}, angular_velocity{.2, .3, .4};
  v::Quaternion attitude{1, 0, 0, 0};
  std::array<double, 36> pose_covariance{}, twist_covariance{};
  double quality{.9};
};
struct CommandWire {
  std::uint16_t schema_version{1};
  std::uint64_t sequence{1};
  Stamp stamp;
  std::uint32_t reset_counter{7};
  std::string source_id{"fixture-client"}, frame_id{"fc01_vision_ned"};
  std::uint8_t mode{1};
  bool valid{true}, position_valid{}, yaw_valid{}, sticks_valid{};
  v::Vec3 position{.5, 0, -1.5};
  double yaw{.3};
  std::array<double, 4> sticks{.1, .2, .3, .4};
};
constexpr std::uint64_t epoch = 100000000000ULL;
static v::Config enabled_config() {
  v::Config c;
  c.enabled = true;
  return c;
}

// 映射层只按协议复制；未声明的数据保持NaN，也不因原始数组恰好像ENU而偷偷换轴。
static void mapping_preserves_declared_measurements_and_missing_values() {
  StateWire wire;
  const auto mapped = v::state_from_message(wire);
  need(mapped.position == wire.position && mapped.velocity == wire.velocity &&
           mapped.attitude == wire.attitude,
       "state mapping must preserve already-normalized internal coordinates");
  need(mapped.received_at == -1 && mapped.stamp_ns == epoch && mapped.reset_counter == 7,
       "source time/reset must not fabricate a local reception");
  need(std::isnan(mapped.angular_velocity[0]) && std::isnan(mapped.pose_covariance[0]) &&
           std::isnan(mapped.twist_covariance[0]),
       "unmarked optional measurements must remain unknown");
  wire.valid_fields = v::Attitude;
  const auto sparse = v::state_from_message(wire);
  need(std::isnan(sparse.position[0]) && std::isnan(sparse.velocity[0]) &&
           std::isnan(sparse.quality),
       "unmarked required measurements cannot be filled from present numeric fields");
  need(wire.position == v::Vec3({1, 2, -1.5}) && wire.valid_fields == v::Attitude,
       "mapping must not mutate the incoming message");
  CommandWire command;
  const auto target = v::command_from_message(command);
  need(std::isnan(target.position[0]) && std::isnan(target.yaw) && std::isnan(target.sticks[0]),
       "unmarked target fields must not become usable defaults");
  wire.stamp.nanosec = 1000000000U;
  const auto bad_stamp = v::state_from_message(wire);
  need(bad_stamp.stamp_ns == 0 && bad_stamp.valid_fields == 0 && !bad_stamp.initialized,
       "malformed source time must not produce a valid state");
}

// 健康流→协议失效→健康流全部发生在两次step之间，仍必须保存已识别的撤销代数。
// 此测试只覆盖有效源戳下的版本/掩码撤销，不冒称Inbox记住所有Controller拒绝原因。
static void mapped_state_protocol_withdrawal_survives_healthy_tail() {
  for (bool unknown_mask : {false, true}) {
    StateWire wire;
    v::Inbox inbox;
    need(inbox.state(v::state_from_message(wire), 10) &&
             inbox.command(v::command_from_message(CommandWire{}), 10),
         "initial mapped pair must enter the inbox");
    v::Controller controller(enabled_config());
    need(controller.step(inbox.snapshot(epoch, 1, 1), 10, 1.5, 0, .05).active,
         "healthy mapped pair must activate the synthetic controller");
    wire.stamp.nanosec = 1000000;
    if (unknown_mask)
      wire.valid_fields |= 128;
    else
      wire.schema_version = 2;
    const auto invalid = v::state_from_message(wire);
    need(invalid.sequence == 1 && invalid.source_id == "fixture" && invalid.valid_fields == 0,
         "protocol rejection must retain bounded identity but withdraw measurements");
    need(inbox.state(invalid, 10.001), "fresh protocol withdrawal must precede sequence dedup");
    wire = StateWire{};
    wire.sequence = 2;
    wire.stamp.nanosec = 2000000;
    need(inbox.state(v::state_from_message(wire), 10.002), "healthy tail must cache normally");
    const auto after = inbox.snapshot(epoch + 2000000, 1, 1);
    const auto result = controller.step(after, 10.002, 1.5, 0, .002);
    need(after.revocation_generation == 1 && result.request_land && !result.active &&
             result.reason == "NAVIGATION_INPUT_REVOKED",
         "healthy tail must not erase the protocol withdrawal");
  }
}

// 目标的版本/枚举错误也不能在适配层丢成“没有新消息”，从而保留旧运动目标。
static void mapped_command_protocol_withdrawal_survives_healthy_tail() {
  for (bool bad_mode : {false, true}) {
    CommandWire wire;
    v::Inbox inbox;
    need(inbox.state(v::state_from_message(StateWire{}), 10) &&
             inbox.command(v::command_from_message(wire), 10),
         "initial command fixture must enter the inbox");
    v::Controller controller(enabled_config());
    need(controller.step(inbox.snapshot(epoch, 1, 1), 10, 1.5, 0, .05).active,
         "healthy initial command must activate");
    wire.stamp.nanosec = 1000000;
    if (bad_mode)
      wire.mode = 255;
    else
      wire.schema_version = 2;
    const auto invalid = v::command_from_message(wire);
    need(!invalid.valid && invalid.mode == v::Mode::Disabled && invalid.source_id == wire.source_id,
         "unsupported command must map to explicit invalid input");
    need(inbox.command(invalid, 10.001), "command invalidation must survive repeated sequence");
    wire = CommandWire{};
    wire.sequence = 2;
    wire.stamp.nanosec = 2000000;
    need(inbox.command(v::command_from_message(wire), 10.002), "new healthy target may cache");
    const auto result =
        controller.step(inbox.snapshot(epoch + 2000000, 1, 1), 10.002, 1.5, 0, .002);
    need(result.request_land && !result.active && result.reason == "NAVIGATION_INPUT_REVOKED",
         "normal target cannot erase an observed protocol revocation");
  }
}

// 选二进制可精确表示的秒数，分别验证严格TTL和包含边界的到达延迟/未来容差。
// 源ROS时钟和本机单调时钟的epoch不同，只有同域相减后得到的年龄才能参与这些比较。
static void clock_limits_keep_their_independent_boundary_rules() {
  v::Config c;
  c.max_measurement_delay = .125;
  c.future_tolerance = .0625;
  const auto valid = [&](std::uint64_t stamp, double received, std::uint64_t ros_now, double now) {
    return v::detail::clock_valid(stamp, received, ros_now, now, .25, c);
  };
  need(valid(epoch - 125000000, 8, epoch, 8), "arrival delay equality is allowed");
  need(!valid(epoch - 125000001, 8, epoch, 8), "one extra arrival nanosecond is rejected");
  need(valid(epoch + 62500000, 8, epoch, 8), "future tolerance equality is allowed");
  need(!valid(epoch + 62500001, 8, epoch, 8), "one extra future nanosecond is rejected");
  need(valid(epoch, 8, epoch + 249999999, 8.125), "source age just below TTL remains valid");
  need(!valid(epoch, 8, epoch + 250000000, 8.125), "source TTL equality expires");
  need(!valid(epoch, 8, epoch, 8.25), "receive TTL equality expires even with frozen ROS time");
  need(!valid(epoch, 8.125, epoch, 8), "future local reception cannot qualify feedback");
}

int main() {
  mapping_preserves_declared_measurements_and_missing_values();
  mapped_state_protocol_withdrawal_survives_healthy_tail();
  mapped_command_protocol_withdrawal_survives_healthy_tail();
  clock_limits_keep_their_independent_boundary_rules();
  std::cout << "STAGE5_VISION_CONTRACT_PASS checks=" << checks << '\n';
}
