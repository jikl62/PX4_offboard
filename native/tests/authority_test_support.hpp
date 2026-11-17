#pragma once
// 权限测试公共夹具：合成足够健康的地面状态与可验证的准备/解锁步骤。
// 这里的推力/时间参数只为确定性回归加速，不是可部署的 flight_test.yaml。
// tick 推进单调秒与 status/land/local/raw 源戳；避免重复原始样本被误当新反馈。
#include "jikl_fc01_system/runtime.hpp"
#include <cassert>
namespace authority_test {
namespace f = fc01;
namespace a = fc01::attitude;
namespace auth = fc01::authority;
// Synthetic numbers copied from the baseline runtime test. Never deployed.
// 构造测试专用动作门/闭环参数；每个测试仍须自己选择并确认所需 authority 合同。
inline f::Config config() {
  f::Config c;
  c.profile = "attitude_altitude";
  auto& p = c.attitude;
  p.flight_actions_enabled = p.arm_enabled = p.takeoff_enabled = p.parameters_confirmed = true;
  p.vertical = {.4, 0, .7, 1, 1, .2, .1, .1, 5, .5, .3};
  p.ramp_seconds = .5;
  p.settle_seconds = p.landing_seconds = p.height_tolerance = p.speed_tolerance = .1;
  p.ascent_timeout = p.descent_timeout = 10;
  p.saturation_seconds = 1;
  p.max_flight_seconds = 60;
  p.max_tilt_deg = 20;
  p.armed_dwell_seconds = .2;
  p.liftoff_dwell_seconds = .1;
  p.liftoff_height_m = .12;
  p.liftoff_speed_mps = .08;
  p.touchdown_dwell_seconds = .1;
  p.height_lower_margin_m = .25;
  p.z_jump_limit_m = 1.2;
  p.vz_jump_limit_mps = 2.0;
  return c;
}
// 构造未解锁、接地静止及新鲜反馈；z=2 m 为 NED 原点偏置，不是相对离地 2 m。
inline f::Snapshot sample() {
  f::Snapshot s;
  s.status_fresh = s.land_fresh = s.local_fresh = true;
  s.at_rest = true;
  s.stationary_seconds = 3;
  s.status_stamp = s.land_stamp = 1000;
  s.system = s.component = 1;
  s.feedback.exclusive = s.feedback.landed = s.feedback.preflight = true;
  s.battery_fresh = s.battery_connected = s.power_valid = true;
  s.attitude.attitude_fresh = s.attitude.failsafe_fresh = s.attitude.z_valid = s.attitude.vz_valid =
      true;
  s.attitude.battery_voltage = 7.6;
  s.attitude.z = 2;
  s.attitude.local_stamp = s.attitude.vertical_sample_stamp = 1000;
  return s;
}
// 50 ms 步长同步推进独立源戳并保持控制台租约，再调用真实 Runtime::step。
inline f::Output tick(f::Runtime& r, f::Snapshot& s, double& now) {
  now += .05;
  s.status_stamp += 50000;
  s.land_stamp += 50000;
  s.attitude.local_stamp += 50000;
  s.attitude.vertical_sample_stamp += 50000;
  r.request("PING", {}, s, now);
  return r.step(s, now);
}
// 准备要求命令 176 被接受且发送之后新状态进入 Offboard，不用单个 ACK 冒充 ready。
inline void ready(f::Runtime& r, f::Snapshot& s, double& now, const char* selection = nullptr) {
  assert(r.request("INTERACTIVE", {}, s, now).accepted);
  if (selection)
    assert(r.request(selection, {}, s, now).accepted);
  assert(r.request("ATTITUDE_READY", {}, s, now).accepted);
  r.step(s, now);
  for (int i = 0; i < 30 && r.attitude_core().phase() != a::Phase::RequestOffboard; ++i)
    tick(r, s, now);
  assert(r.attitude_core().phase() == a::Phase::RequestOffboard);
  r.ack({r.pending(176).baseline + 1, 176, 0, 1, 191, false}, now + .01);
  s.feedback.offboard = true;
  s.nav_state = 14;
  tick(r, s, now);
  assert(r.attitude_core().phase() == a::Phase::OffboardReady);
}
// 在 ready 后发送一次 TAKEOFF，模拟 Arm ACK 和新 armed 状态后才确认 ArmedGround。
inline void armed(f::Runtime& r, f::Snapshot& s, double& now, const char* selection) {
  ready(r, s, now, selection);
  assert(r.request("TAKEOFF", {1}, s, now).accepted);
  auto o = tick(r, s, now);
  assert(o.attitude.command == a::Command::Arm);
  r.ack({r.pending(400).baseline + 1, 400, 0, 1, 191, false}, now + .01);
  s.feedback.armed = true;
  tick(r, s, now);
  assert(r.attitude_core().phase() == a::Phase::ArmedGround);
}
inline auth::State state(f::Runtime& r, const f::Snapshot& s, double now) {
  return r.publication_context(s, now, true, true).authority;
}
} // namespace authority_test
