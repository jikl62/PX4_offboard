#pragma once
// 零推力台架审查夹具：以真实最终 sink 检查 ARM_ONLY 生命周期和上锁/释放。
// 包含历史独立合成夹具但不运行其 main；本文件本身不进行真实 ROS/PX4 通信。
// 台架启用 bench_arm_only、关闭 takeoff；每帧严格断言无任何非零推力。
// Read-only independent R6.2c audit; includes baseline fixtures and headers.
// No ROS, socket, PX4 connection or physical commands.
#define main stock_s10_not_invoked
#include "c_s10_independent/C_S10_SAFETY_ATTACKS.cpp"
#undef main

static f::Config bench() {
  auto c = config();
  c.attitude.bench_arm_only_enabled = true;
  c.attitude.takeoff_enabled = false;
  c.validate();
  return c;
}

struct B {
  f::Runtime r{bench()};
  f::Snapshot s{snapshot()};
  double t{};
  unsigned tick_count{}, targets{}, heartbeats{}, arms{}, disarms{}, releases{}, denials{};
  bool connected{true};
  a::Decision last_decision = a::allow();
  explicit B(bool rc = false) {
    r.require_attitude_sink_commit();
    if (rc)
      s.attitude.rc_link_ready = s.attitude.rc_switches_ready = true;
    assert(r.request("INTERACTIVE", {}, s, t).accepted);
    assert(r.request(rc ? "AUTHORITY_RC" : "AUTHORITY_AUTONOMOUS", {}, s, t).accepted);
    assert(r.request("ATTITUDE_READY", {}, s, t).accepted);
  }
  // 20 Hz 控制与 1 Hz land 分开推进；参数可冻结 ping/land/local 或跳过假提交以构造时序。
  a::Output next(bool ping = true, bool newland = true, bool publish = true, bool newlocal = true) {
    t += .05;
    ++tick_count;
    s.status_stamp += 50000;
    if (newlocal)
      s.attitude.local_stamp += 50000;
    if (newland && tick_count % 20 == 0)
      s.land_stamp += 1000000;
    if (ping && connected)
      assert(r.request("PING", {}, s, t).accepted);
    auto o = r.step(s, t).attitude;
    assert(!a::nonzero(o.thrust));
    if (publish)
      commit(o);
    return o;
  }
  // 本地提交要求真实边界允许，再向 Runtime 回报 wire stamp，区分计划与最终 sink 成功。
  void commit(const a::Output& o) {
    if (o.publish) {
      last_decision = a::publish_attitude_at_boundary<Mode, Target>(
          r.publication_context(s, t, connected, true), o, 1000000 + std::uint64_t(t * 1e6),
          [&](const Target& x) {
            assert(x.thrust_body[0] == 0 && x.thrust_body[1] == 0 && x.thrust_body[2] == 0);
            ++targets;
          },
          [&](const Mode&) {
            ++heartbeats;
          });
      if (!last_decision.allowed) {
        ++denials;
        return;
      }
    }
    if (o.command == a::Command::None)
      return;
    const auto c = r.publication_context(s, t, connected, true);
    a::Decision d = a::allow();
    if (o.command == a::Command::Arm)
      d = a::publish_arm_at_boundary(c, [&] {
        ++arms;
      });
    else if (o.command == a::Command::Disarm)
      d = a::publish_disarm_at_boundary(c, [&] {
        ++disarms;
      });
    else if (o.command == a::Command::ReleaseOffboard)
      d = a::publish_release_at_boundary(c, [&] {
        ++releases;
      });
    assert(d.allowed);
    r.confirm_attitude_command_publication(o.command, true, 1000000 + std::uint64_t(t * 1e6), s, t);
  }
  void ready() {
    for (int i = 0; i < 60 && r.attitude_core().phase() != a::Phase::RequestOffboard; ++i)
      next();
    assert(r.attitude_core().phase() == a::Phase::RequestOffboard);
    r.ack({r.pending(176).wire_stamp + 1000, 176, 0, 1, 191, false}, t + .001);
    s.feedback.offboard = true;
    s.nav_state = 14;
    next();
    assert(r.attitude_core().phase() == a::Phase::OffboardReady);
  }
  // 准备后只发送 ARM_ONLY 并断言 Arm sink 恰一次，不能借台架入口请求起飞。
  void planned_arm() {
    ready();
    assert(r.request("ARM_ONLY", {}, s, t).accepted);
    assert(next().command == a::Command::Arm && arms == 1);
  }
  void armed() {
    planned_arm();
    r.ack({r.pending(400).wire_stamp + 1000, 400, 0, 1, 191, false}, t + .001);
    s.feedback.armed = true;
    next();
    assert(r.attitude_core().phase() == a::Phase::BenchArmedGround);
  }
  // 记录阶段、失败原因、实际 sink 次数和当前反馈，供各审查测试定位失败点。
  void log(const char* id, const a::Output& o) {
    std::cout << id << " phase=" << a::name(r.attitude_core().phase())
              << " reason=" << r.attitude_core().reason() << " output_publish=" << o.publish
              << " armed=" << s.feedback.armed << " offboard=" << s.feedback.offboard
              << " at_rest=" << s.at_rest << " arm_sinks=" << arms << " disarm_sinks=" << disarms
              << " boundary=" << a::name(last_decision.reason) << " targets=" << targets
              << " heartbeats=" << heartbeats << "\n";
  }
};
