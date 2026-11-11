// 命令 ACK 生命周期回归：验证严格因果、目标身份、重复过滤以及 ACK 与实际模式的双证据。
// ground_link 和通用 Runtime 分开测试；输入为合成 µs 源戳和单调秒，无飞控连接。
#include "jikl_fc01_system/ground_link.hpp"
#include "jikl_fc01_system/runtime.hpp"
#include <cassert>
#include <iostream>

namespace g = jikl_fc01_ground_link;
namespace f = fc01;

static g::Traffic traffic() {
  return {1, 1, 1, 1, 1};
}

// 建立连续两秒安全地面窗并触发唯一 Command176，为后续 ACK 场景固定发送基线。
static void ready_and_send(g::Core& c, std::uint64_t stamp = 2000) {
  c.status({1000, true, false, false, false, true, 1, 1, 4}, 0.0);
  c.land({1000, true, true}, 0.0);
  assert(!c.tick(0.0, traffic()));
  c.status({stamp, true, false, false, false, true, 1, 1, 4}, 2.1);
  c.land({stamp, true, true}, 2.1);
  assert(c.tick(2.1, traffic()));
  assert(c.state() == g::State::Sent);
}

// PX4 正常 ACK 的 from_external 可为 false，按命令/目标/新时间匹配后仍需新 Stabilized 状态。
static void test_accepted_ack_from_external_false() {
  g::Core c;
  ready_and_send(c);
  assert(c.ack({3000, 176, 0, 1, 191, false}, 2.2));
  c.status({4000, true, false, false, false, true, 1, 1, 15}, 2.3);
  c.land({4000, true, true}, 2.3);
  c.tick(2.3, traffic());
  assert(c.state() == g::State::Pass);
}

// DENIED=2 是终态拒绝，保留结果，不能继续等待当作成功。
static void test_denied_ack() {
  g::Core c;
  ready_and_send(c);
  assert(c.ack({3000, 176, 2, 1, 191, false}, 2.2));
  assert(c.state() == g::State::Rejected && c.ack_result() == 2);
}

// FAILED=4 同样为终态拒绝，不为请求生成自动重试权限。
static void test_failed_ack() {
  g::Core c;
  ready_and_send(c);
  assert(c.ack({3000, 176, 4, 1, 191, false}, 2.2));
  assert(c.state() == g::State::Rejected && c.ack_result() == 4);
}

// IN_PROGRESS=5 仅记录进展，后续 ACCEPTED=0 加发送后新模式反馈才通过。
static void test_in_progress_then_accepted() {
  g::Core c;
  ready_and_send(c);
  assert(c.ack({3000, 176, 5, 1, 191, false}, 2.2));
  assert(c.state() == g::State::Sent && c.ack_result() == 5);
  assert(c.ack({4000, 176, 0, 1, 191, false}, 2.3));
  c.status({5000, true, false, false, false, true, 1, 1, 15}, 2.4);
  c.land({5000, true, true}, 2.4);
  c.tick(2.4, traffic());
  assert(c.state() == g::State::Pass);
}

// 错误 command 编号不能更新本事务的 ack_seen。
static void test_wrong_command() {
  g::Core c;
  ready_and_send(c);
  assert(!c.ack({3000, 512, 0, 1, 191, false}, 2.2));
  assert(!c.ack_seen());
}

// 错误请求者 system/component 的 ACK 即使接受也无关本任务。
static void test_wrong_target() {
  g::Core c;
  ready_and_send(c);
  assert(!c.ack({3000, 176, 0, 255, 190, false}, 2.2));
  assert(!c.ack_seen());
}

// 与发送基线相等的 ACK 不具备“发送后”因果证据。
static void test_stale_ack() {
  g::Core c;
  ready_and_send(c, 3000);
  assert(!c.ack({3000, 176, 0, 1, 191, false}, 2.2));
  assert(!c.ack_seen());
}

// 同源戳重复 ACK 不推进结果或终态。
static void test_duplicate_ack() {
  g::Core c;
  ready_and_send(c);
  assert(c.ack({3000, 176, 5, 1, 191, false}, 2.2));
  assert(!c.ack({3000, 176, 5, 1, 191, false}, 2.21));
  assert(c.ack_result() == 5 && c.state() == g::State::Sent);
}

// 命令前先见到的 ACK 成为水位；发送后重放它仍不能被接受。
static void test_ack_before_command() {
  g::Core c;
  c.status({1000, true, false, false, false, true, 1, 1, 4}, 0.0);
  c.land({1000, true, true}, 0.0);
  assert(!c.ack({1500, 176, 0, 1, 191, false}, 0.1));
  assert(!c.tick(0.0, traffic()));
  c.status({2000, true, false, false, false, true, 1, 1, 4}, 2.1);
  c.land({2000, true, true}, 2.1);
  assert(c.tick(2.1, traffic()));
  assert(!c.ack({1500, 176, 0, 1, 191, false}, 2.2));
  assert(!c.ack_seen());
}

// 只有新模式状态、没有合法 ACK 时不能完成地面链路测试，最终应 ACK_TIMEOUT。
static void test_new_status_without_valid_ack() {
  g::Core c;
  ready_and_send(c);
  c.status({3000, true, false, false, false, true, 1, 1, 15}, 2.2);
  c.land({3000, true, true}, 2.2);
  c.tick(2.2, traffic());
  assert(c.state() == g::State::Sent);
  c.status({4000, true, false, false, false, true, 1, 1, 15}, 4.6);
  c.land({4000, true, true}, 4.6);
  c.tick(4.6, traffic());
  c.status({5000, true, false, false, false, true, 1, 1, 15}, 7.2);
  c.land({5000, true, true}, 7.2);
  c.tick(7.2, traffic());
  assert(c.state() == g::State::AckTimeout);
}

// 构造通用 flight 路线所需健康/未解锁/接地状态，位置与速度为 NED。
static f::Snapshot flight_snapshot() {
  f::Snapshot s;
  s.status_fresh = s.land_fresh = s.local_fresh = true;
  s.at_rest = true;
  s.stationary_seconds = 3;
  s.battery_fresh = true;
  s.battery_connected = true;
  s.power_valid = true;
  s.battery_warning = 0;
  s.manual_fresh = true;
  s.manual_lost = false;
  s.dead_reckoning = false;
  s.time_regressed = false;
  s.status_stamp = 10000;
  s.land_stamp = 10000;
  s.system = 1;
  s.component = 1;
  s.nav_state = 4;
  s.feedback.fresh = true;
  s.feedback.valid = true;
  s.feedback.heading_ok = true;
  s.feedback.preflight = true;
  s.feedback.armed = false;
  s.feedback.landed = true;
  s.feedback.offboard = false;
  s.feedback.landing = false;
  s.feedback.failsafe = false;
  s.feedback.exclusive = true;
  s.feedback.position = {0, 0, 0};
  s.feedback.velocity = {0, 0, 0};
  s.feedback.yaw = 0;
  return s;
}

static f::Runtime make_flight_runtime() {
  f::Config c;
  c.profile = "flight";
  c.flight_enabled = true;
  return f::Runtime(c);
}

// 维持新 status/land，走真实预发送时长直到 pending176.sent，不直接改内部状态。
static void drive_to_offboard_pending(f::Runtime& r, f::Snapshot& s) {
  assert(r.request("PING", {}, s, 0.0).accepted);
  assert(r.request("TAKEOFF", {1.0}, s, 0.1).accepted);
  for (double t = 0.1; t <= 2.4; t += 0.1) {
    s.status_stamp += 1000;
    s.land_stamp += 1000;
    auto out = r.step(s, t);
    (void)out;
  }
  assert(r.pending(176).sent);
}

// 在 Runtime 路线复核错误编号/目标/旧戳无效，进展不完成，合法新 ACK 可接受且重复不续期。
static void test_runtime_strict_correlation_and_false_external() {
  auto r = make_flight_runtime();
  auto s = flight_snapshot();
  drive_to_offboard_pending(r, s);
  const auto base = r.pending(176).baseline;
  r.ack({base + 1, 512, 0, 1, 191, false}, 2.5);
  assert(!r.pending(176).accepted);
  r.ack({base + 2, 176, 0, 255, 190, false}, 2.5);
  assert(!r.pending(176).accepted);
  r.ack({base, 176, 0, 1, 191, false}, 2.5);
  assert(!r.pending(176).accepted);
  r.ack({base + 3, 176, 5, 1, 191, false}, 2.5);
  assert(r.pending(176).result == 5 && !r.pending(176).accepted);
  r.ack({base + 4, 176, 0, 1, 191, false}, 2.6);
  assert(r.pending(176).accepted && r.pending(176).result == 0);
  r.ack({base + 4, 176, 0, 1, 191, false}, 2.7);
  assert(r.pending(176).accepted && r.pending(176).last_ack == base + 4);
}

int main() {
  test_accepted_ack_from_external_false();
  test_denied_ack();
  test_failed_ack();
  test_in_progress_then_accepted();
  test_wrong_command();
  test_wrong_target();
  test_stale_ack();
  test_duplicate_ack();
  test_ack_before_command();
  test_new_status_without_valid_ack();
  test_runtime_strict_correlation_and_false_external();
  std::cout << "ack lifecycle tests: PASS\n";
}
