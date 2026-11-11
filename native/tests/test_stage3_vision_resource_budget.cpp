// 视觉身份预算回归：只用内存输入，验证首次复制前拒绝、迟到撤销和 UTF-8 字节边界。
// 不创建 ROS 节点，不授予新控制权；全局分配探针只检查本测试单线程同步调用。
#include "jikl_fc01_system/velocity_command_input.hpp"
#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <new>

namespace v = fc01::vision;
namespace vi = fc01::velocity_input;
static bool watch_allocations{};
static std::size_t largest_allocation{};
void* operator new(std::size_t size) {
  if (watch_allocations)
    largest_allocation = std::max(largest_allocation, size);
  if (void* result = std::malloc(size ? size : 1))
    return result;
  throw std::bad_alloc();
}
void operator delete(void* value) noexcept {
  std::free(value);
}
void operator delete(void* value, std::size_t) noexcept {
  std::free(value);
}

struct Stamp {
  std::int32_t sec{100};
  std::uint32_t nanosec{};
};
struct StateWire {
  std::uint16_t schema_version{1}, valid_fields{71};
  std::uint64_t sequence{1};
  Stamp stamp;
  std::uint32_t reset_counter{};
  std::string source_id{"camera"}, frame_id{"fc01_vision_ned"}, body_frame_id{"base_link_frd"},
      angular_velocity_source;
  bool initialized{true}, tracking{true}, calibration_valid{true};
  std::array<double, 3> position{}, velocity{}, angular_velocity{};
  std::array<double, 4> attitude{1, 0, 0, 0};
  std::array<double, 36> pose_covariance{}, twist_covariance{};
  double quality{1};
};
struct CommandWire {
  std::uint16_t schema_version{1};
  std::uint64_t sequence{1};
  Stamp stamp;
  std::uint32_t reset_counter{};
  std::string source_id{"guide"}, frame_id{"fc01_vision_ned"};
  std::uint8_t mode{1};
  bool valid{true}, position_valid{}, yaw_valid{}, sticks_valid{};
  std::array<double, 3> position{};
  std::array<double, 4> sticks{};
  double yaw{}, vx{}, vy{}, vz{};
};
static v::Input input() {
  v::Input result;
  result.state = v::state_from_message(StateWire{});
  result.command = v::command_from_message(CommandWire{});
  result.state.received_at = result.command.received_at = 10;
  result.ros_now_ns = 100000000000ULL;
  result.state_publishers = result.command_publishers = 1;
  return result;
}
static v::Controller controller() {
  v::Config config;
  config.enabled = true;
  return v::Controller(config);
}

// 256 是 UTF-8 字节上限，不是 Unicode 字符数；拒绝过长、截断、过度编码、代理项与 NUL。
static void byte_and_encoding_boundaries() {
  assert(v::identity_max_bytes == 256);
  assert(v::identity_within_budget(std::string(256, 'a')));
  assert(!v::identity_within_budget(std::string(257, 'a')));
  std::string unicode;
  for (unsigned i = 0; i < 64; ++i)
    unicode += "\xf0\x9f\x9a\x81";
  assert(v::identity_within_budget(unicode));
  assert(!v::identity_within_budget(unicode + "a"));
  for (const std::string& invalid :
       {std::string("x\0y", 3), std::string("\x80"), std::string("\xc0\x80"),
        std::string("\xed\xa0\x80"), std::string("\xf4\x90\x80\x80"), std::string("\xe4\xb8")})
    assert(!v::identity_within_budget(invalid));
  assert(v::identity_within_budget("")); // 必填仍由原协议门单独判断。
}

// 巨大身份在 ROS 映射与 Inbox 前不被再次分配；四种状态身份、两种目标身份都受同一门约束。
static void map_and_retract_without_large_copies() {
  StateWire state_wire;
  CommandWire command_wire;
  const std::string huge(1024 * 1024, 'a');
  for (unsigned field = 0; field < 4; ++field) {
    state_wire = {};
    auto* text = field == 0   ? &state_wire.source_id
                 : field == 1 ? &state_wire.frame_id
                 : field == 2 ? &state_wire.body_frame_id
                              : &state_wire.angular_velocity_source;
    *text = huge;
    largest_allocation = 0;
    watch_allocations = true;
    const auto mapped = v::state_from_message(state_wire);
    v::Inbox inbox;
    assert(inbox.state(mapped, 10));
    const auto snapshot = inbox.snapshot(100000000000ULL, 1, 1);
    watch_allocations = false;
    assert(largest_allocation < 4096);
    assert(mapped.identity_resource_invalid && mapped.source_id.empty());
    assert(snapshot.state.identity_resource_invalid && snapshot.state.valid_fields == 0);
  }
  for (unsigned field = 0; field < 2; ++field) {
    command_wire = {};
    (field ? command_wire.frame_id : command_wire.source_id) = huge;
    largest_allocation = 0;
    watch_allocations = true;
    const auto mapped = v::command_from_message(command_wire);
    const auto velocity = vi::from_message(command_wire);
    watch_allocations = false;
    assert(largest_allocation < 4096);
    assert(mapped.identity_resource_invalid && mapped.source_id.empty() && !mapped.valid);
    assert(velocity.identity_resource_invalid && velocity.source_id.empty() && !velocity.valid);
  }

  auto healthy = input();
  v::Inbox inbox;
  assert(inbox.state(healthy.state, 10) && inbox.command(healthy.command, 10));
  auto navigation = controller();
  assert(navigation.step(inbox.snapshot(healthy.ros_now_ns, 1, 1), 10, 1.5, 0, .05).active);
  const auto previous = inbox.snapshot(healthy.ros_now_ns, 1, 1).revocation_generation;
  auto late = healthy.state;
  late.sequence = late.stamp_ns = 0;
  late.source_id = huge;
  largest_allocation = 0;
  watch_allocations = true;
  assert(inbox.state(late, 10.01));
  watch_allocations = false;
  assert(largest_allocation < 4096);
  assert(inbox.snapshot(healthy.ros_now_ns, 1, 1).revocation_generation > previous);
  assert(!navigation.current(inbox.snapshot(healthy.ros_now_ns, 1, 1), 10.01));
  // 同一控制周期内恢复健康也不能抹掉刚才的预算拒绝，活动任务必须走原撤销/降落策略。
  auto restored = healthy.state;
  ++restored.sequence;
  restored.stamp_ns += 10000000ULL;
  assert(inbox.state(restored, 10.02));
  const auto result = navigation.step(inbox.snapshot(healthy.ros_now_ns, 1, 1), 10.02, 1.5, 0, .05);
  assert(!result.active && result.request_land && result.reason == "NAVIGATION_INPUT_REVOKED");
  assert(navigation.step(healthy, 10.03, 1.5, 0, .05).request_land);

  v::Inbox commands;
  assert(commands.command(healthy.command, 10));
  auto late_command = healthy.command;
  late_command.sequence = late_command.stamp_ns = 0;
  late_command.frame_id = huge;
  assert(commands.command(late_command, 10.01));
  assert(commands.snapshot(healthy.ros_now_ns, 1, 1).command.identity_resource_invalid);
}

// 资源错误不得擦掉同源防重放高水位；重复/倒退仍拒绝，真正的新来源/reset 保持原合同。
static void rejection_preserves_both_sequence_watermarks() {
  auto in = input();
  in.state.sequence = in.command.sequence = 10;
  v::Inbox inbox;
  assert(inbox.state(in.state, 10) && inbox.command(in.command, 10));
  auto bad_state = in.state;
  auto bad_command = in.command;
  bad_state.source_id.assign(1024 * 1024, 'x');
  bad_command.frame_id.assign(1024 * 1024, 'y');
  assert(inbox.state(bad_state, 10.01) && inbox.command(bad_command, 10.01));
  auto rejected = inbox.snapshot(in.ros_now_ns, 1, 1);
  assert(rejected.state.source_id == in.state.source_id && rejected.state.sequence == 10);
  assert(rejected.command.source_id == in.command.source_id && rejected.command.sequence == 10);
  assert(rejected.state.identity_resource_invalid && rejected.command.identity_resource_invalid);
  assert(rejected.state.valid_fields == 0 && !rejected.command.valid);
  const auto generation = rejected.revocation_generation;
  for (const std::uint64_t sequence : {std::uint64_t(1), std::uint64_t(10)}) {
    in.state.sequence = in.command.sequence = sequence;
    in.state.stamp_ns += 1000000ULL;
    in.command.stamp_ns += 1000000ULL;
    assert(!inbox.state(in.state, 10.02) && !inbox.command(in.command, 10.02));
    rejected = inbox.snapshot(in.ros_now_ns, 1, 1);
    assert(rejected.state.sequence == 10 && rejected.command.sequence == 10);
    assert(rejected.state.received_at == 10.01 && rejected.command.received_at == 10.01);
    assert(rejected.revocation_generation == generation);
  }
  in.state.sequence = in.command.sequence = 11;
  assert(inbox.state(in.state, 10.03) && inbox.command(in.command, 10.03));
  const auto fresh = inbox.snapshot(in.ros_now_ns, 1, 1);
  assert(!fresh.state.identity_resource_invalid && !fresh.command.identity_resource_invalid);
  assert(fresh.state.sequence == 11 && fresh.command.sequence == 11);
  assert(fresh.revocation_generation == generation);
  in.state.source_id = "different-camera";
  in.command.source_id = "different-guide";
  in.state.sequence = in.command.sequence = 1;
  assert(inbox.state(in.state, 10.04) && inbox.command(in.command, 10.04));
  ++in.state.reset_counter;
  ++in.command.reset_counter;
  assert(inbox.state(in.state, 10.05) && inbox.command(in.command, 10.05));
}

// 直接使用纯核心入口也不得绕过 ROS 层预算；预览只锁存离线错误，不携带飞机控制权。
static void pure_core_and_preview() {
  auto healthy = input();
  healthy.state.source_id.assign(256, 'a');
  healthy.command.source_id.assign(256, 'b');
  auto normal = controller();
  assert(normal.step(healthy, 10, 1.5, 0, .05).active);
  auto invalid = healthy;
  invalid.state.angular_velocity_source.assign(257, 'c');
  auto rejected = controller();
  auto result = rejected.step(invalid, 10, 1.5, 0, .05);
  assert(!result.active && result.reason == "VISION_STATE_IDENTITY_RESOURCE_LIMIT");
  invalid = healthy;
  invalid.command.source_id.push_back('b');
  auto rejected_command = controller();
  result = rejected_command.step(invalid, 10, 1.5, 0, .05);
  assert(!result.active && result.reason == "NAVIGATION_COMMAND_IDENTITY_RESOURCE_LIMIT");

  vi::Session session({.25, .30, .20, .05, 2., .08, .6});
  assert(session.state(healthy.state, 10, healthy.ros_now_ns));
  vi::Target target;
  target.schema_version = 1;
  target.sequence = 1;
  target.stamp_ns = healthy.ros_now_ns;
  target.source_id = "guide";
  target.frame_id = "body_frd";
  target.valid = true;
  target.body_velocity = {0, 0, 0};
  assert(session.command(target, 10, healthy.ros_now_ns));
  assert(session.preview(vi::Mode::VelocityPreview, 10, healthy.ros_now_ns, 0).valid);
  target.source_id.assign(1024 * 1024, 'c');
  largest_allocation = 0;
  watch_allocations = true;
  assert(!session.command(target, 10.01, healthy.ros_now_ns));
  watch_allocations = false;
  assert(largest_allocation < 4096);
  const auto preview = session.preview(vi::Mode::VelocityPreview, 10.01, healthy.ros_now_ns, 0);
  assert(!preview.valid && !preview.control_authority &&
         preview.reason == "COMMAND_IDENTITY_RESOURCE_LIMIT");
}
int main() {
  byte_and_encoding_boundaries();
  map_and_retract_without_large_copies();
  rejection_preserves_both_sequence_watermarks();
  pure_core_and_preview();
  std::cout << "STAGE3_VISION_IDENTITY_RESOURCE_BUDGET_PASS\n";
}
